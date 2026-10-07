"""Analysis E: mission energy budget (section 10E, eq. 10.6) and reserve (Q7).

E_total = E_to + E_ft + E_climb + E_cruise + E_desc + E_bt + E_land + E_reserve
          <= eta_usable * SoH * f(T) * E_nom

`range` is the flown distance of ONE leg at the mass of the supplied `VehicleParams`; a return
leg or a payload drop is a second call with another mass. Segment powers use one mean state of
charge (`MissionProfile.soc_mean`) rather than tracking the open-circuit voltage along the
mission. The forward transition is a quasi-steady ESTIMATE until analyses B/C exist (M5/M6).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

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

    def usable_J(self, vp: VehicleParams) -> float:
        return vp.battery.e_nominal_wh * J_PER_WH * self.eta_usable * self.soh * self.temp_factor


@dataclass(frozen=True)
class MissionProfile:
    """Idealised flight profile. Every default is an ASSUMED placeholder."""

    cruise_amsl_m: float = 1000.0
    ground_amsl_m: float = 0.0
    isa_offset_k: float = 0.0
    clearance_agl_m: float = 30.0  # hover climb before the forward transition
    vz_takeoff: float = 2.0
    t_hover_pre: float = 10.0  # s of hover before lift-off
    v_trans: float = 14.0  # AIRSPEED_MIN
    a_fwd: float = 1.5  # forward-transition acceleration (ASSUMED until analysis C)
    v_climb: float = 18.0
    roc: float = 2.5
    v_cruise: float | None = None  # None = best-range speed
    v_glide: float = 16.0
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


def _wing_polar_drag(vp: VehicleParams, rho: float, V: float, lift: float) -> float:
    """Drag [N] for a given lift in the linear region: D = q S (C_D0 + k C_L^2)."""
    q_s = 0.5 * rho * V * V * vp.wing.S
    cl = lift / q_s
    return q_s * (vp.wing.CD0 + vp.wing.k * cl**2)


def stall_speed_1g(vp: VehicleParams, rho: float) -> float:
    return aero.stall_speed(vp.m, rho, vp.wing)


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


def forward_transition_estimate(
    vp: VehicleParams, mp: MissionProfile, soc: float, n: int = 40
) -> SegmentResult:
    """Quasi-steady ESTIMATE: constant acceleration a_fwd to V_trans, rotors carrying W - L.

    The wing lift is the share (V/V_lw)^2 of the weight with V_lw the speed where the wing
    carries the weight at 80% of C_L,max; the pusher supplies m a_fwd + D. To be replaced by
    the simulated transition of analyses B/C.
    """
    rho = float(atmosphere.density(mp.clearance_amsl, mp.isa_offset_k))
    v_lw = stall_speed_1g(vp, rho) / math.sqrt(0.8)
    speeds = np.linspace(0.0, mp.v_trans, n)
    powers = np.zeros(n)
    w_n = vp.m * G
    for i, v in enumerate(speeds):
        share = max(0.0, 1.0 - (v / v_lw) ** 2)
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
    return SegmentResult(
        e, mp.v_trans / mp.a_fwd, mp.v_trans**2 / (2.0 * mp.a_fwd),
        "quasi-steady estimate, a_fwd ASSUMED",
    )  # fmt: skip


def climb_segment(vp: VehicleParams, mp: MissionProfile, soc: float) -> SegmentResult:
    dh = mp.cruise_amsl_m - mp.clearance_amsl
    if dh <= 0.0:
        return SegmentResult(0.0, 0.0, 0.0, "no climb")
    h_mean = 0.5 * (mp.cruise_amsl_m + mp.clearance_amsl)
    rho = float(atmosphere.density(h_mean, mp.isa_offset_k))
    gamma = math.asin(mp.roc / mp.v_climb)
    tr = trim.steady_trim(vp, rho, mp.v_climb, gamma, soc)
    t = dh / mp.roc
    return SegmentResult(
        tr.elec.power * t, t, mp.v_climb * math.cos(gamma) * t, "steady-climb trim, pusher model"
    )


def descent_segment(vp: VehicleParams, mp: MissionProfile, soc: float) -> SegmentResult:
    dh = mp.cruise_amsl_m - (mp.ground_amsl_m + mp.h_bt_agl_m)
    if dh <= 0.0:
        return SegmentResult(0.0, 0.0, 0.0, "no descent")
    h_mean = 0.5 * (mp.cruise_amsl_m + mp.ground_amsl_m)
    rho = float(atmosphere.density(h_mean, mp.isa_offset_k))
    tr = trim.glide_trim(vp, rho, mp.v_glide, soc)
    sink = mp.v_glide * math.sin(-tr.gamma)
    t = dh / sink
    return SegmentResult(
        tr.elec.power * t, t, mp.v_glide * math.cos(tr.gamma) * t, "pusher-off glide trim"
    )


def back_transition_segment(
    vp: VehicleParams, mp: MissionProfile, v0: float, soc: float, a_plan: float
) -> SegmentResult:
    """Energy of the Position1 braking simulation (analysis D) from ground speed v0."""
    scen = Scenario(
        mp.ground_amsl_m + mp.h_bt_agl_m, mp.isa_offset_k, soc, ground_amsl_m=mp.ground_amsl_m
    )
    bp = BackTransitionParams.for_vehicle(vp, scen.rho, soc, scen.altitude_amsl_m)
    r = braking.landing_error(vp, scen, v0, a_plan, bp, agl0=mp.h_bt_agl_m)
    if r.sim.terminated_by != "ground_speed_reached":
        raise RuntimeError(f"back-transition did not complete ({r.sim.terminated_by})")
    return SegmentResult(
        r.sim.metrics["energy_J"],
        r.t_stop,
        r.x_actual,
        "simulation (analysis D, placeholder gains)",
    )


def landing_segment(
    vp: VehicleParams, mp: MissionProfile, soc: float, limits: descent.DescentLimits
) -> SegmentResult:
    scen = Scenario(mp.ground_amsl_m + mp.h_bt_agl_m, mp.isa_offset_k, soc)
    prof = descent.descent_profile(vp, scen, mp.h_bt_agl_m, limits)
    return SegmentResult(prof.energy_J, prof.time, 0.0, f"analysis F, kappa: {prof.kappa_source}")


def hover_check(
    vp: VehicleParams, mp: MissionProfile, window: BatteryWindow
) -> dict[str, float | bool]:
    """P_hover,req <= V_bus(end of discharge) x I_available, plus the motor-level check."""
    rho = float(atmosphere.density(mp.ground_amsl_m, mp.isa_offset_k))
    es = trim.hover_state(vp, rho, window.soc_eod)
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
) -> BudgetResult:
    """First-pass mission budget from the component models and analyses D and F."""
    soc = mp.soc_mean
    rho_cruise = float(atmosphere.density(mp.cruise_amsl_m, mp.isa_offset_k))
    v_cruise = mp.v_cruise or best_range_speed(vp, rho_cruise, soc)
    tr = trim.steady_trim(vp, rho_cruise, v_cruise, 0.0, soc)
    e_cruise = tr.elec.power / v_cruise
    segs = {
        "takeoff": takeoff_segment(vp, mp, soc),
        "fwd_transition": forward_transition_estimate(vp, mp, soc),
        "climb": climb_segment(vp, mp, soc),
        "descent": descent_segment(vp, mp, soc),
        "back_transition": back_transition_segment(vp, mp, v_cruise, soc, a_plan),
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


__all__ = [
    "BatteryWindow",
    "BudgetResult",
    "MissionProfile",
    "SegmentResult",
    "best_range_speed",
    "build_budget",
    "cruise_sweep",
    "hover_check",
]
