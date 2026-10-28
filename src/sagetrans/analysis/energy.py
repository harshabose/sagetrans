"""Analysis E: mission energy budget (section 10E, eq. 10.6) and reserve (Q7).

E_total = E_to + E_ft + E_climb + E_cruise + E_desc + E_bt + E_land + E_reserve
          <= eta_usable * SoH * f(T) * E_nom

`BudgetResult` is one leg at the mass of the supplied `VehicleParams`. `MissionBudget` composes
legs: a `round_trip` (loaded out, empty back) or a `one_way`, at a site (airfield) elevation with
the cruise altitude a fixed climb height above it. The range is an OUTPUT: whatever energy is left
after the fixed segments and the reserve goes to cruise. Segment powers use one mean state of
charge (`MissionProfile.soc_mean`); `soc_trace.py` walks the SoC along the mission and checks each
segment at both ends. The forward transition is a quasi-steady ESTIMATE until analyses B/C exist.

STATED SIMPLIFICATION, the glide-to-exit deceleration. The descent ends at the glide speed (the
cruise speed) and the back transition starts at the AIRBRAKE exit (`AIRSPEED_MIN - 2`, 15.0 to
19.4 m/s TAS at the four sites against 21.1 to 27.3 m/s). The slowing in between is in no
segment: its kinetic energy (0.82 to 1.36 Wh) is taken to be dissipated by drag in the descent, so
it is not charged to the battery, and the distance covered while slowing is not counted. The
effect on the range is at most 0.08 km. How to count it (a glide that slows, a level
deceleration) is a modelling choice that has not been made.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace

import numpy as np

from sagetrans import atmosphere
from sagetrans.analysis import braking, descent
from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.control.ardupilot_like import BackTransitionParams
from sagetrans.dynamics import trim
from sagetrans.physics import aero, rotor
from sagetrans.physics.params import VehicleParams

G = 9.80665
J_PER_WH = 3600.0


@dataclass(frozen=True)
class BatteryWindow:
    """Usable energy window (open question O-15; the values are ASSUMED)."""

    eta_usable: float = 0.80
    soh: float = 1.0
    temp_factor: float = 1.0
    soc_eod: float = 0.10  # state of charge at end of discharge, for the hover power check
    soc_start: float = 1.0  # state of charge at take-off (a full pack; the usable energy is a
    # fraction eta_usable of nominal, so the mission ends at 1 - eta_usable)

    def usable_J(self, vp: VehicleParams) -> float:
        return vp.battery.e_nominal_wh * J_PER_WH * self.eta_usable * self.soh * self.temp_factor


@dataclass(frozen=True)
class MissionProfile:
    """Idealised flight profile. Every default is an ASSUMED placeholder."""

    cruise_amsl_m: float = 1000.0
    ground_amsl_m: float = 0.0
    isa_offset_k: float = 0.0
    clearance_agl_m: float = 30.0  # hover climb before the forward transition
    fwd_alt_loss_max_m: float = 10.0  # accepted altitude loss in the forward transition (FT)
    fwd_alt_gain_max_m: float = 10.0  # accepted altitude gain in the forward transition (FT)
    vz_takeoff: float = 2.0
    t_hover_pre: float = 10.0  # s of hover before lift-off
    # AIRSPEED_MIN as loaded, an EQUIVALENT airspeed. `mission_at_site` turns it into the TRUE
    # airspeed at the clearance altitude, which is what the forward segment, Q2's acceleration
    # profile and the SoC trace read, so none of them converts again.
    v_trans: float = 14.0
    a_fwd: float = 1.5  # forward-transition acceleration (ASSUMED until analysis C)
    # Q_TRANSITION_MS in seconds: the rotor fade after the hand-over. 0 = no fade (the older
    # estimate that stops at V_trans); the study sets the register's or the recommended value.
    q_transition_s: float = 0.0
    # share of the weight the wing carries at the hand-over (the rotors carry the rest into the
    # fade); None = the quasi-steady `rotor_share` model
    wing_share_handover: float | None = None
    v_climb: float | None = None  # design TAS at `v_cruise_ref_amsl_m`; None = the cruise speed
    roc: float = 2.5
    v_cruise: float | None = None  # design TAS at `v_cruise_ref_amsl_m`; None = best-range speed
    v_glide: float | None = None  # as `v_climb`
    v_cruise_ref_amsl_m: float = 1500.0  # altitude where the design speeds hold
    climb_height_m: float = 200.0  # cruise altitude = site elevation + this
    payload_kg: float = 5.0  # mass delivered at the destination (the return leg is lighter)
    t_drop_s: float = 0.0  # hover at the destination for the drop, at the loaded mass
    h_bt_agl_m: float = 30.0  # height above ground where the back-transition starts
    soc_mean: float = 0.6
    # reserve (Q7)
    t_go_around: float = 60.0  # s of hover
    t_hold: float = 0.0
    d_diversion_m: float = 5000.0
    n_diversion_transitions: int = 0  # extra forward + back transition pairs

    @property
    def clearance_amsl(self) -> float:
        return self.ground_amsl_m + self.clearance_agl_m


@dataclass(frozen=True)
class SegmentResult:
    energy_J: float
    time_s: float
    distance_m: float
    source: str  # how the number was obtained and its evidence grade


@dataclass
class BudgetResult:
    """Energy bookkeeping as a function of range; construct with `from_segments`."""

    segments: dict[str, SegmentResult]  # all fixed segments (reserve excluded)
    e_cruise_J_per_m: float
    reserve_J: float
    usable_J: float
    record: RunRecord
    notes: dict[str, float] = field(default_factory=dict)

    @classmethod
    def from_segments(
        cls,
        segments: dict[str, SegmentResult],
        e_cruise_J_per_m: float,
        reserve_J: float,
        usable_J: float,
        record: RunRecord | None = None,
        notes: dict[str, float] | None = None,
    ) -> BudgetResult:
        return cls(
            segments, e_cruise_J_per_m, reserve_J, usable_J, record or make_record(),
            notes or {},
        )  # fmt: skip

    @property
    def fixed_J(self) -> float:
        return sum(s.energy_J for s in self.segments.values())

    @property
    def fixed_distance_m(self) -> float:
        return sum(s.distance_m for s in self.segments.values())

    def total_J(self, range_m: float) -> float:
        d_cruise = range_m - self.fixed_distance_m
        if d_cruise < 0.0:
            raise ValueError("range shorter than the distance covered by the fixed segments")
        return self.fixed_J + d_cruise * self.e_cruise_J_per_m + self.reserve_J

    def margin_J(self, range_m: float) -> float:
        return self.usable_J - self.total_J(range_m)

    @property
    def max_range_m(self) -> float:
        spare = self.usable_J - self.fixed_J - self.reserve_J
        return self.fixed_distance_m + max(0.0, spare) / self.e_cruise_J_per_m

    def table(self, ranges_m: list[float]) -> list[dict[str, float | bool]]:
        """Budget by range (Q1): segments in Wh, total, margin and feasibility."""
        rows: list[dict[str, float | bool]] = []
        for r in ranges_m:
            row: dict[str, float | bool] = {"range_km": r / 1e3}
            for name, s in self.segments.items():
                row[f"{name}_Wh"] = s.energy_J / J_PER_WH
            row["cruise_Wh"] = (r - self.fixed_distance_m) * self.e_cruise_J_per_m / J_PER_WH
            row["reserve_Wh"] = self.reserve_J / J_PER_WH
            row["total_Wh"] = self.total_J(r) / J_PER_WH
            row["usable_Wh"] = self.usable_J / J_PER_WH
            row["margin_Wh"] = self.margin_J(r) / J_PER_WH
            row["margin_pct"] = 100.0 * self.margin_J(r) / self.usable_J
            row["fits"] = self.margin_J(r) >= 0.0
            rows.append(row)
        return rows

    def conflicts(self, required_m: tuple[float, ...] = (100e3, 113e3)) -> dict[float, bool]:
        """True where the range requirement and the usable energy conflict."""
        return {r: self.max_range_m < r for r in required_m}


# ---- segment models -----------------------------------------------------------------------------


def wing_polar_drag(vp: VehicleParams, rho: float, V: float, lift: float) -> float:
    """Drag [N] for a given lift in the linear region: D = q S (C_D0 + k C_L^2)."""
    q_s = 0.5 * rho * V * V * vp.wing.S
    cl = lift / q_s
    return q_s * (vp.wing.CD0 + vp.wing.k * cl**2)


_wing_polar_drag = wing_polar_drag  # the old private name

DEFAULT_V_CLIMB = 18.0  # used only when neither v_climb nor v_cruise is set (a bare profile)
DEFAULT_V_GLIDE = 16.0


def climb_speed(mp: MissionProfile) -> float:
    return mp.v_climb if mp.v_climb is not None else (mp.v_cruise or DEFAULT_V_CLIMB)


def glide_speed(mp: MissionProfile) -> float:
    return mp.v_glide if mp.v_glide is not None else (mp.v_cruise or DEFAULT_V_GLIDE)


def design_speeds(mp: MissionProfile, altitude_amsl_m: float) -> tuple[float, float, float]:
    """(cruise, climb, glide) TAS at `altitude_amsl_m`: the design TAS scaled to the same
    dynamic pressure, so the equivalent airspeed is the one the design speed has at the
    reference altitude. A missing cruise speed (best-range) is returned as 0 and left to
    `build_budget`; a missing climb or glide speed follows the cruise speed."""
    k = math.sqrt(
        float(atmosphere.density(mp.v_cruise_ref_amsl_m, mp.isa_offset_k))
        / float(atmosphere.density(altitude_amsl_m, mp.isa_offset_k))
    )
    vc = mp.v_cruise * k if mp.v_cruise is not None else 0.0
    vcl = mp.v_climb * k if mp.v_climb is not None else vc
    vg = mp.v_glide * k if mp.v_glide is not None else vc
    return vc, vcl, vg


def mission_at_site(
    mp: MissionProfile, site_amsl_m: float, v_trans_convention: str = "eas"
) -> MissionProfile:
    """The profile for an airfield at `site_amsl_m`: cruise at site + climb height, the design
    speeds converted to that altitude. The segment functions then run unchanged on it.

    `v_trans` of the result is a TRUE airspeed: the loaded AIRSPEED_MIN (an equivalent airspeed)
    times sqrt(rho0 / rho) at the clearance altitude above the site. `v_trans_convention="tas"`
    (Q2.feed_convention) keeps the loaded value as it is. Apply this once to a loaded profile.
    """
    cruise = site_amsl_m + mp.climb_height_m
    vc, vcl, vg = design_speeds(mp, cruise)
    v_trans = mp.v_trans
    if v_trans_convention == "eas":
        rho_c = float(atmosphere.density(site_amsl_m + mp.clearance_agl_m, mp.isa_offset_k))
        v_trans = mp.v_trans * math.sqrt(float(atmosphere.density(0.0)) / rho_c)
    return replace(
        mp, ground_amsl_m=site_amsl_m, cruise_amsl_m=cruise, v_trans=v_trans,
        v_cruise=vc if mp.v_cruise is not None else None,
        v_climb=vcl if (mp.v_climb is not None or mp.v_cruise is not None) else None,
        v_glide=vg if (mp.v_glide is not None or mp.v_cruise is not None) else None,
    )  # fmt: skip


def stall_speed_1g(vp: VehicleParams, rho: float) -> float:
    return aero.stall_speed(vp.m, rho, vp.wing)


LIFT_FRACTION_AT_V_LW = 0.8  # the wing carries the weight at this fraction of C_L,max at V_lw


def rotor_share(vp: VehicleParams, rho: float, V: float) -> float:
    """Fraction of the weight the lift rotors carry at airspeed V (analyses E and Q2).

    The wing carries the share (V / V_lw)^2, with V_lw the speed where it carries the whole
    weight at 80% of C_L,max; the rotors carry the rest.
    """
    v_lw = stall_speed_1g(vp, rho) / math.sqrt(LIFT_FRACTION_AT_V_LW)
    return max(0.0, 1.0 - (V / v_lw) ** 2)


def cruise_sweep(
    vp: VehicleParams, rho: float, soc: float, speeds: np.ndarray
) -> dict[str, np.ndarray]:
    """Level-flight power and energy per metre against speed; infeasible speeds are NaN."""
    power = np.full(speeds.shape, np.nan)
    for i, v in enumerate(speeds):
        try:
            tr = trim.steady_trim(vp, rho, float(v), 0.0, soc)
        except ValueError:
            continue
        if tr.elec.feasible:
            power[i] = tr.elec.power
    return {"V": speeds, "power_W": power, "J_per_m": power / speeds}


def best_range_speed(vp: VehicleParams, rho: float, soc: float) -> float:
    """Speed minimising electrical energy per metre (level flight)."""
    v_lo = 1.1 * stall_speed_1g(vp, rho)
    speeds = np.linspace(v_lo, 45.0, 120)
    s = cruise_sweep(vp, rho, soc, speeds)
    if np.all(np.isnan(s["J_per_m"])):
        raise ValueError("no feasible cruise speed")
    return float(speeds[int(np.nanargmin(s["J_per_m"]))])


def takeoff_segment(vp: VehicleParams, mp: MissionProfile, soc: float) -> SegmentResult:
    """Hover on the pad, then a vertical climb to clearance height.

    Hover power over the whole time plus the potential-energy term m g h divided by the
    electrical-to-lift efficiency (ideal momentum power over actual bus power).
    """
    rho = float(atmosphere.density(mp.ground_amsl_m, mp.isa_offset_k))
    p_hov = trim.hover_power(vp, rho, soc)
    t_climb = mp.clearance_agl_m / mp.vz_takeoff
    a_tot = vp.rotor.A_tot
    p_ideal = rotor.ideal_hover_power(vp.m * G / vp.rotor.n_r, rho, a_tot / vp.rotor.n_r)
    eff = vp.rotor.n_r * p_ideal / p_hov
    e = p_hov * (mp.t_hover_pre + t_climb) + vp.m * G * mp.clearance_agl_m / eff
    return SegmentResult(e, mp.t_hover_pre + t_climb, 0.0, "hover model (assumed maps)")


def fade_estimate(vp: VehicleParams, mp: MissionProfile, soc: float, n: int = 100) -> SegmentResult:
    """The rotor fade after the hand-over: energy, time (`Q_TRANSITION_MS`) and distance.

    ArduPilot freezes the rotor throttle at the hand-over (`last_throttle`) and fades the command
    linearly to zero over `Q_TRANSITION_MS` (FT's controller does the same, floor 1 %). Here
    `last_throttle` is the command that gives the rotors' share of the weight at the hand-over,
    (1 - wing share) W with the wing share `mp.wing_share_handover` (E.fade_wing_share; without it
    the (V/V_lw)^2 of `rotor_share`, which is 1 above V_lw), through the same rotor and
    motor model as FT (`trim.electrical_state`: command from speed and torque, bus power from the
    motor power). The aircraft keeps its hand-over speed (the distance is a lower bound) and is in
    force balance at every instant: the wing carries what the fading rotors do not, W - T_rotor,
    and the pusher balances the wing's drag at that lift, so it works harder as the rotors let go.
    Holding the hand-over thrust m a_fwd + D instead overestimates by 40 % to 180 % against FT (it
    keeps accelerating, FT's TECS does not). Zero when `mp.q_transition_s` is zero.
    """
    q = mp.q_transition_s
    if q <= 0.0 or mp.v_trans <= 0.0:
        return SegmentResult(0.0, 0.0, 0.0, "no rotor fade")
    rho = float(atmosphere.density(mp.clearance_amsl, mp.isa_offset_k))
    v, w_n = mp.v_trans, vp.m * G
    share = (  # rotors' share of the weight at the hand-over
        rotor_share(vp, rho, v) if mp.wing_share_handover is None else 1.0 - mp.wing_share_handover
    )
    w0 = trim.rotor_speed_for_thrust(share * w_n, 0.0, v, rho, vp)
    speeds = np.linspace(0.0, w0, 41)
    cmd = np.zeros(speeds.size)
    power = np.zeros(speeds.size)
    for i, w in enumerate(speeds):
        t_r, q_r = 0.0, 0.0
        if w > 0.0:
            t_r, q_r = (float(x) for x in rotor.lift_rotor(float(w), 0.0, v, rho, vp.rotor)[:2])
        drag = _wing_polar_drag(vp, rho, v, max(w_n - t_r, 0.0))
        w_p = trim.pusher_speed_for_thrust(drag, v, rho, vp)
        q_p = float(rotor.pusher(w_p, v, rho, vp.pusher)[1])
        es = trim.electrical_state(vp, soc, 0.0, float(w), q_r, w_p, q_p)
        cmd[i], power[i] = es.delta_r_cmd, es.power
    last = float(cmd[-1])  # the throttle command at the hand-over
    tau = np.linspace(0.0, q, n)
    c = last * np.maximum(0.01, (q - tau) / q)  # FT's linear fade with its 1 % floor
    p = np.interp(c, cmd, power)  # `cmd` rises with the rotor speed
    return SegmentResult(
        float(np.trapezoid(p, tau)), q, v * q, f"rotor fade over {q:g} s from throttle {last:.2f}"
    )


def forward_transition_estimate(
    vp: VehicleParams, mp: MissionProfile, soc: float, n: int = 40, source: str | None = None
) -> SegmentResult:
    """Quasi-steady ESTIMATE: constant acceleration a_fwd to V_trans, rotors carrying W - L,
    then the rotor fade of `mp.q_transition_s` (`fade_estimate`).

    The wing lift is the share (V/V_lw)^2 of the weight with V_lw the speed where the wing
    carries the weight at 80% of C_L,max (`rotor_share`); the pusher supplies m a_fwd + D.
    a_fwd comes from analysis Q2 (`E.a_fwd_mode`) or the mission file. To be replaced by
    the simulated transition of analyses B/C.
    """
    rho = float(atmosphere.density(mp.clearance_amsl, mp.isa_offset_k))
    speeds = np.linspace(0.0, mp.v_trans, n)
    powers = np.zeros(n)
    w_n = vp.m * G
    for i, v in enumerate(speeds):
        share = rotor_share(vp, rho, float(v))
        t_r = share * w_n
        w_r = trim.rotor_speed_for_thrust(t_r, 0.0, float(v), rho, vp)
        q_r = float(rotor.lift_rotor(w_r, 0.0, float(v), rho, vp.rotor)[1]) if t_r > 0 else 0.0
        lift = (1.0 - share) * w_n
        drag = _wing_polar_drag(vp, rho, float(v), lift) if v > 0.5 else 0.0
        t_p = vp.m * mp.a_fwd + drag
        w_p = trim.pusher_speed_for_thrust(t_p, float(v), rho, vp)
        q_p = float(rotor.pusher(w_p, float(v), rho, vp.pusher)[1])
        powers[i] = trim.electrical_state(vp, soc, 0.0, w_r, q_r, w_p, q_p).power
    t = speeds / mp.a_fwd
    e = float(np.trapezoid(powers, t))
    fade = fade_estimate(vp, mp, soc)
    label = "quasi-steady estimate, a_fwd ASSUMED" if source is None else source
    return SegmentResult(
        e + fade.energy_J, mp.v_trans / mp.a_fwd + fade.time_s,
        mp.v_trans**2 / (2.0 * mp.a_fwd) + fade.distance_m,
        label + (f"; {fade.source}" if fade.time_s > 0.0 else ""),
    )  # fmt: skip


def climb_segment(vp: VehicleParams, mp: MissionProfile, soc: float) -> SegmentResult:
    dh = mp.cruise_amsl_m - mp.clearance_amsl
    if dh <= 0.0:
        return SegmentResult(0.0, 0.0, 0.0, "no climb")
    h_mean = 0.5 * (mp.cruise_amsl_m + mp.clearance_amsl)
    rho = float(atmosphere.density(h_mean, mp.isa_offset_k))
    v_climb = climb_speed(mp)
    gamma = math.asin(mp.roc / v_climb)
    tr = trim.steady_trim(vp, rho, v_climb, gamma, soc)
    t = dh / mp.roc
    return SegmentResult(
        tr.elec.power * t, t, v_climb * math.cos(gamma) * t, "steady-climb trim, pusher model"
    )


def descent_segment(vp: VehicleParams, mp: MissionProfile, soc: float) -> SegmentResult:
    dh = mp.cruise_amsl_m - (mp.ground_amsl_m + mp.h_bt_agl_m)
    if dh <= 0.0:
        return SegmentResult(0.0, 0.0, 0.0, "no descent")
    h_mean = 0.5 * (mp.cruise_amsl_m + mp.ground_amsl_m)
    rho = float(atmosphere.density(h_mean, mp.isa_offset_k))
    v_glide = glide_speed(mp)
    tr = trim.glide_trim(vp, rho, v_glide, soc)
    sink = v_glide * math.sin(-tr.gamma)
    t = dh / sink
    return SegmentResult(
        tr.elec.power * t, t, v_glide * math.cos(tr.gamma) * t, "pusher-off glide trim"
    )


def back_transition_segment(
    vp: VehicleParams,
    mp: MissionProfile,
    v0: float,
    soc: float,
    a_plan: float,
    bp: BackTransitionParams | None = None,
) -> SegmentResult:
    """Energy of the Position1 braking simulation (analysis D) from ground speed v0.

    `bp` is the configured controller (`stages.d_exit_params`, the one analysis D runs;
    `run_braking` sets the target height and the hover throttle itself). With it the run starts at
    the AIRBRAKE exit like D, C and B1 (`braking.airbrake_entry_state`: pitch `bp.theta_fw0`, the
    rotors carrying what the wing does not). None builds a controller from the code defaults and
    starts from the legacy hover state (rotors at hover speed, level), which no study path uses.
    """
    scen = Scenario(
        mp.ground_amsl_m + mp.h_bt_agl_m, mp.isa_offset_k, soc, ground_amsl_m=mp.ground_amsl_m
    )
    x0 = None
    if bp is None:
        bp = BackTransitionParams.for_vehicle(vp, scen.rho, soc, scen.altitude_amsl_m)
    else:
        x0, _ = braking.airbrake_entry_state(vp, scen, bp, v0)
    r = braking.landing_error(vp, scen, v0, a_plan, bp, agl0=mp.h_bt_agl_m, x0=x0)
    if r.sim.terminated_by != "ground_speed_reached":
        raise RuntimeError(f"back-transition did not complete ({r.sim.terminated_by})")
    return SegmentResult(
        r.sim.metrics["energy_J"],
        r.t_stop,
        r.x_actual,
        "simulation (analysis D)" + (", default controller" if bp is None else ""),
    )


def landing_segment(
    vp: VehicleParams, mp: MissionProfile, soc: float, limits: descent.DescentLimits
) -> SegmentResult:
    scen = Scenario(mp.ground_amsl_m + mp.h_bt_agl_m, mp.isa_offset_k, soc)
    prof = descent.descent_profile(vp, scen, mp.h_bt_agl_m, limits)
    return SegmentResult(prof.energy_J, prof.time, 0.0, f"analysis F, kappa: {prof.kappa_source}")


def hover_check(
    vp: VehicleParams, mp: MissionProfile, window: BatteryWindow, soc: float | None = None
) -> dict[str, float | bool]:
    """P_hover,req <= V_bus(soc) x I_available, plus the motor-level check.

    `soc` defaults to the end-of-discharge stress point `window.soc_eod`; the site is
    `mp.ground_amsl_m`."""
    rho = float(atmosphere.density(mp.ground_amsl_m, mp.isa_offset_k))
    es = trim.hover_state(vp, rho, window.soc_eod if soc is None else soc)
    # bus current at the lift-motor current limit and the throttle ceiling
    i_avail = vp.rotor.n_r * 0.95 * vp.motor_r.I_lim + vp.battery.I_av
    p_avail = es.v_bus * i_avail
    return {
        "p_hover_req_W": es.power,
        "v_bus_eod_V": es.v_bus,
        "p_available_W": p_avail,
        "margin_pct": 100.0 * (p_avail - es.power) / es.power,
        "throttle_cmd": es.delta_r_cmd,
        "ok": es.feasible and es.power <= p_avail,
    }


def reserve_energy(
    vp: VehicleParams,
    mp: MissionProfile,
    soc: float,
    e_cruise_J_per_m: float,
    e_ft: float,
    e_bt: float,
) -> dict[str, float]:
    """Go-around hover, hold and diversion (Q7), in joules."""
    rho = float(atmosphere.density(mp.ground_amsl_m, mp.isa_offset_k))
    p_hov = trim.hover_power(vp, rho, soc)
    parts = {
        "go_around": p_hov * mp.t_go_around,
        "hold": p_hov * mp.t_hold,
        "diversion_cruise": e_cruise_J_per_m * mp.d_diversion_m,
        "diversion_transitions": mp.n_diversion_transitions * (e_ft + e_bt),
    }
    parts["total"] = sum(parts.values())
    return parts


def build_budget(
    vp: VehicleParams,
    mp: MissionProfile,
    window: BatteryWindow,
    limits: descent.DescentLimits,
    a_plan: float = 2.5,
    a_fwd_source: str | None = None,
    *,
    bp: BackTransitionParams | None = None,
    v0_entry: float | None = None,
) -> BudgetResult:
    """First-pass budget of ONE leg from the component models and analyses D and F.

    `bp` is the controller of the back transition and `v0_entry` its entry ground speed. E and
    the sensitivity pass the AIRBRAKE exit speed (`braking.entry_airspeed`, the speed D, C and B1
    start from) and `inp.controller`; None keeps the legacy behaviour (code defaults, entry at the
    cruise speed), which no study path uses any more. `mp.v_trans` is a true airspeed here
    (`mission_at_site` converts the loaded AIRSPEED_MIN once).
    """
    soc = mp.soc_mean
    rho_cruise = float(atmosphere.density(mp.cruise_amsl_m, mp.isa_offset_k))
    v_cruise = mp.v_cruise or best_range_speed(vp, rho_cruise, soc)
    mp = replace(mp, v_climb=climb_speed(replace(mp, v_cruise=v_cruise)),
                 v_glide=glide_speed(replace(mp, v_cruise=v_cruise)))  # fmt: skip
    tr = trim.steady_trim(vp, rho_cruise, v_cruise, 0.0, soc)
    e_cruise = tr.elec.power / v_cruise
    segs = {
        "takeoff": takeoff_segment(vp, mp, soc),
        "fwd_transition": forward_transition_estimate(vp, mp, soc, source=a_fwd_source),
        "climb": climb_segment(vp, mp, soc),
        "descent": descent_segment(vp, mp, soc),
        "back_transition": back_transition_segment(
            vp, mp, v_cruise if v0_entry is None else v0_entry, soc, a_plan, bp
        ),
        "landing": landing_segment(vp, mp, soc, limits),
    }
    res = reserve_energy(
        vp, mp, soc, e_cruise, segs["fwd_transition"].energy_J, segs["back_transition"].energy_J
    )
    return BudgetResult.from_segments(
        segs, e_cruise, res["total"], window.usable_J(vp),
        make_record(vp, mp, window, limits, a_plan),
        {
            "v_cruise": v_cruise,
            "p_cruise_W": tr.elec.power,
            **{f"reserve_{k}": v for k, v in res.items()},
        },
    )  # fmt: skip


# ---- the mission: legs, reserve once ------------------------------------------------------------


@dataclass
class MissionBudget:
    """One or two legs and one reserve.

    `r_m` is the one-way distance every leg covers (climb and descent included); a round trip
    flies 2 r_m. The cruise distance of a leg is `r_m` minus its fixed segments' distance, so
    the energy is linear in `r_m` and the range is solved in closed form.
    """

    kind: str  # "round_trip" | "one_way"
    legs: list[tuple[str, float, BudgetResult]]  # (name, mass_kg, leg budget with reserve_J = 0)
    reserve_J: float  # counted once: the larger of the landings' reserves
    reserve_parts: dict[str, dict[str, float]]  # per leg: reserve_energy() output
    drop_J: float  # t_drop_s x hover power at the loaded mass (round trip only)
    usable_J: float
    record: RunRecord
    notes: dict[str, float | str] = field(default_factory=dict)
    vehicles: dict[str, VehicleParams] = field(default_factory=dict)  # leg -> vehicle at its mass
    profiles: dict[str, MissionProfile] = field(default_factory=dict)  # leg -> profile as run

    @property
    def fixed_J(self) -> float:
        return sum(b.fixed_J for _, _, b in self.legs)

    @property
    def fixed_distance_m(self) -> float:
        """The shortest one-way distance that exists: the largest leg's fixed segments."""
        return max(b.fixed_distance_m for _, _, b in self.legs)

    def total_J(self, r_m: float) -> float:
        if r_m < self.fixed_distance_m:
            raise ValueError("range shorter than the distance covered by the fixed segments")
        legs = sum(
            b.fixed_J + (r_m - b.fixed_distance_m) * b.e_cruise_J_per_m for _, _, b in self.legs
        )
        return legs + self.drop_J + self.reserve_J

    def margin_J(self, r_m: float) -> float:
        return self.usable_J - self.total_J(r_m)

    @property
    def max_range_m(self) -> float:
        """The one-way distance (radius for a round trip) that spends the usable energy."""
        num = self.usable_J - self.fixed_J - self.reserve_J - self.drop_J
        num += sum(b.fixed_distance_m * b.e_cruise_J_per_m for _, _, b in self.legs)
        r = num / sum(b.e_cruise_J_per_m for _, _, b in self.legs)
        return r if r >= self.fixed_distance_m else 0.0

    def table(self, ranges_m: list[float]) -> list[dict[str, float | bool]]:
        """Budget by range: leg-prefixed segments in Wh, cruise, reserve, total and margin."""
        rows: list[dict[str, float | bool]] = []
        for r in ranges_m:
            row: dict[str, float | bool] = {"range_km": r / 1e3}
            cruise = 0.0
            for name, _, b in self.legs:
                for seg, sr in b.segments.items():
                    row[f"{name}_{seg}_Wh"] = sr.energy_J / J_PER_WH
                cruise += (r - b.fixed_distance_m) * b.e_cruise_J_per_m
            row["cruise_Wh"] = cruise / J_PER_WH
            row["reserve_Wh"] = self.reserve_J / J_PER_WH
            row["drop_Wh"] = self.drop_J / J_PER_WH
            row["total_Wh"] = self.total_J(r) / J_PER_WH
            row["usable_Wh"] = self.usable_J / J_PER_WH
            row["margin_Wh"] = self.margin_J(r) / J_PER_WH
            row["margin_pct"] = 100.0 * self.margin_J(r) / self.usable_J
            row["fits"] = self.margin_J(r) >= 0.0
            rows.append(row)
        return rows

    def conflicts(self, required_m: tuple[float, ...] = (100e3, 113e3)) -> dict[float, bool]:
        """True where the requirement (the same one-way distance or radius) exceeds the range."""
        return {r: self.max_range_m < r for r in required_m}


def build_mission(
    vp: VehicleParams,
    mp: MissionProfile,
    window: BatteryWindow,
    limits: descent.DescentLimits,
    a_plan: float,
    kind: str,
    a_fwd_by_leg: dict[str, float],
    bp: BackTransitionParams | None = None,
    v0_entry: float | None = None,
    a_fwd_source: str | None = None,
) -> MissionBudget:
    """Budget of a `round_trip` (legs "out" and "ret") or a `one_way` (leg "leg").

    `mp` is the profile at the site (`mission_at_site`). The loaded leg flies at `vp.m`, the
    return leg at `vp.m - payload_kg`. Nothing is raised for an infeasible operating point: the
    numbers are returned and `soc_trace` judges the feasibility.
    """
    if kind == "round_trip":
        masses = [("out", vp.m), ("ret", vp.m - mp.payload_kg)]
    elif kind == "one_way":
        masses = [("leg", vp.m)]
    else:
        raise ValueError(f"unknown mission kind {kind!r}")
    legs: list[tuple[str, float, BudgetResult]] = []
    parts: dict[str, dict[str, float]] = {}
    vehicles: dict[str, VehicleParams] = {}
    profiles: dict[str, MissionProfile] = {}
    for name, m in masses:
        vpl = replace(vp, m=m)
        mpl = replace(mp, a_fwd=a_fwd_by_leg[name])
        b = build_budget(
            vpl, mpl, window, limits, a_plan, a_fwd_source, bp=bp, v0_entry=v0_entry
        )
        parts[name] = {k.removeprefix("reserve_"): v for k, v in b.notes.items()
                       if k.startswith("reserve_")}  # fmt: skip
        legs.append((name, m, replace(b, reserve_J=0.0)))
        vehicles[name] = vpl
        profiles[name] = replace(mpl, v_cruise=float(b.notes["v_cruise"]))
    binding = max(parts, key=lambda k: parts[k]["total"])
    rho_site = float(atmosphere.density(mp.ground_amsl_m, mp.isa_offset_k))
    drop = 0.0
    if kind == "round_trip" and mp.t_drop_s > 0.0:
        drop = mp.t_drop_s * trim.hover_power(vehicles["out"], rho_site, mp.soc_mean)
    return MissionBudget(
        kind, legs, parts[binding]["total"], parts, drop, window.usable_J(vp),
        make_record(vp, mp, window, limits, a_plan, kind),
        {"reserve_binding_leg": binding, "soc_mean": mp.soc_mean},
        vehicles, profiles,
    )  # fmt: skip


__all__ = [
    "BatteryWindow",
    "BudgetResult",
    "MissionBudget",
    "MissionProfile",
    "SegmentResult",
    "best_range_speed",
    "build_budget",
    "build_mission",
    "cruise_sweep",
    "design_speeds",
    "hover_check",
    "mission_at_site",
]
