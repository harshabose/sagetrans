"""ArduPilot-structured baseline controller (section 8.2).

`Position1Controller` is the back-transition part: pusher off, rotor altitude hold (8.5), the
pitch envelope (8.3), and the position cascade (8.9)-(8.10) mapped to pitch by (8.4).
`BaselineController` adds the forward transition (wait for AIRSPEED_MIN, then a linear rotor
fade), a cruise stub with a rate-limited pusher hand-off, and the switch-point test (8.11).
Not modelled: BATT_WATT_MAX, TECS energy blending, the airbrake. Gains are ASSUMED placeholders.

Q_BCK_PIT_LIM is a constant cap by default (`pit_lim_mode="constant"`); `"airspeed"` selects
ArduPilot's airspeed-scaled form (O-05). The pitch envelope ramps from the last fixed-wing pitch
(`theta_fw0`) to the cap over Q_BACKTRANS_MS, with the clock starting at the AIRBRAKE exit (O-04).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from sagetrans import atmosphere
from sagetrans.control.base import ModeState
from sagetrans.dynamics import trim
from sagetrans.physics.params import VehicleParams

G = 9.80665
RHO0 = float(atmosphere.density(0.0))


@dataclass(frozen=True)
class BackTransitionParams:
    """Back-transition controller parameters.

    `a_lim` is the deceleration limit of the position cascade, which never reads the planned
    deceleration. ArduPilot's Q_TRANS_DECEL does both jobs; this repo splits them: the planned
    value only places the target (`braking.landing_error`: x_tgt = V_ref^2 / (2 a_plan)).
    The fields from `theta_fw0` on default to the original behaviour.
    """

    theta_A: float = math.radians(20.0)  # Q_A_ANGLE_MAX
    theta_P: float = math.radians(20.0)  # PTCH_LIM_MAX_DEG (ArduPilot default)
    theta_B: float | None = None  # Q_BCK_PIT_LIM; None = no cap
    T_bt: float = 3.0  # Q_BACKTRANS_MS in seconds
    # altitude hold (8.5)
    h_tgt: float = 0.0
    delta_hover: float = 0.3
    delta_min: float = 0.1
    delta_max: float = 0.95
    K_h: float = 0.5
    K_vh: float = 0.02
    K_i: float = 0.01
    v_dem_max: float = 3.0
    i_max: float = 20.0
    # position cascade (8.9)
    x_tgt: float = 0.0
    K_p: float = 0.5
    K_v: float = 1.0
    a_lim: float = 3.0
    brake_at_limit: bool = False  # worst-case braking test for analysis D
    # pitch envelope as ArduPilot (O-04, O-05)
    theta_fw0: float = 0.0  # rad; last FW pitch at the AIRBRAKE exit (ramp start, initial pitch)
    pit_lim_mode: str = "constant"  # "constant" | "airspeed"
    v_min_ref: float | None = None  # m/s; V_min of the airspeed form (set from AIRSPEED_MIN)
    # altitude-hold feed-forward
    ff_mode: str = "repo"  # "repo" (constant hover throttle) | "lift_aware"
    ff_lift_tau_s: float = 0.0  # first-order filter on the lift estimate, s
    ff_lift_scale: float = 1.0  # multiplier on the lift estimate (estimation error)
    # limits and sensitivities of analysis D; all off by default
    alpha_cap: float | None = None  # rad; theta - gamma <= alpha_cap while V >= alpha_cap_vmin
    alpha_cap_vmin: float = 5.0  # m/s; below it the wing carries under 10% of the weight
    # the one lumped actuator lag of D (first order, on the rotor throttle): ESC slew, output
    # filtering, throttle smoothing and any rotor inertia beyond I_rot, which adds in series.
    # A sensitivity until a bench value is set; not ArduPilot's spool law
    rotor_lag_s: float = 0.0
    # the vertical loop of analysis C: "repo" is the altitude hold above (K_h, K_vh, K_i);
    # "ardupilot" is ArduPilot's structure with its default values (position P -> velocity P ->
    # acceleration PID on the measured vertical acceleration; open question O-12): master
    # 4.8.0-dev Plane defaults, not a pinned tag (O-08). In "ardupilot" mode K_h, K_vh, K_i,
    # i_max, v_dem_max, delta_hover and ff_mode are not read
    z_mode: str = "repo"  # "repo" | "ardupilot"
    z_pos_p: float = 1.0  # Q_P_POSZ_P, 1/s
    z_vel_p: float = 5.0  # Q_P_VELZ_P, (m/s^2) per (m/s)
    z_acc_p: float = 0.03  # Q_P_ACCZ_P, thrust_norm per (m/s^2)
    z_acc_i: float = 0.1  # Q_P_ACCZ_I, thrust_norm per (m/s^2) per s
    z_acc_imax: float = 0.8  # Q_P_ACCZ_IMAX, thrust_norm
    z_vmax: float = 2.5  # Q_PILOT_SPD_UP, m/s (applied symmetrically)
    z_vel_flt_hz: float = 5.0  # velocity-error filter
    z_acc_flt_hz: float = 10.0  # acceleration-error filter

    def __post_init__(self) -> None:
        if self.z_mode not in ("repo", "ardupilot"):
            raise ValueError(f"unknown z_mode {self.z_mode!r}")

    @classmethod
    def for_vehicle(
        cls, vp: VehicleParams, rho: float, soc: float, h_tgt: float, **kw: Any
    ) -> BackTransitionParams:
        base = cls(h_tgt=h_tgt, delta_hover=trim.hover_throttle(vp, rho, soc))
        return replace(base, **kw)

    def theta_limit(self, t_rel: float, eas: float | None = None) -> float:
        """theta_lim(t) of (8.3): the ramp from `theta_fw0` to the cap, and Q_BCK_PIT_LIM.

        `eas` (equivalent airspeed, m/s) is needed by the airspeed form only.
        """
        cap = min(self.theta_A, self.theta_P)
        start = min(max(self.theta_fw0, 0.0), cap)
        if self.T_bt <= 0.0:
            ramp = cap
        else:
            ramp = start + (cap - start) * min(1.0, max(t_rel, 0.0) / self.T_bt)
        if self.pit_lim_mode == "constant":
            return ramp if self.theta_B is None else min(ramp, self.theta_B)
        if self.pit_lim_mode != "airspeed":
            raise ValueError(f"unknown pit_lim_mode {self.pit_lim_mode!r}")
        if self.theta_B is None or self.v_min_ref is None or eas is None:
            raise ValueError("pit_lim_mode 'airspeed' needs theta_B, v_min_ref and an airspeed")
        if self.theta_B <= 0.0:
            return ramp  # the limit is disabled
        lim_b = min(self.theta_B * (max(self.v_min_ref, 5.0) / max(eas, 0.1)) ** 2, self.theta_A)
        return min(ramp, lim_b)


def altitude_hold(
    p: BackTransitionParams, h: float, vh: float, integ: float, dt: float
) -> tuple[float, float]:
    """Rotor altitude hold (8.5): returns the throttle command and the updated integral."""
    v_dem = max(-p.v_dem_max, min(p.v_dem_max, p.K_h * (p.h_tgt - h)))
    err = v_dem - vh
    integ = max(-p.i_max, min(p.i_max, integ + err * dt))
    delta = p.delta_hover + p.K_vh * err + p.K_i * integ
    return max(p.delta_min, min(p.delta_max, delta)), integ


class LiftFF:
    """Throttle for a rotor thrust, tabulated once, for the lift-aware altitude-hold feed-forward.

    Built from the scenario vehicle (mass applied), the density at the start altitude and the
    state of charge, so the control loop interpolates instead of solving for a rotor speed. The
    table spans thrust 0 to `t_max_factor` x W (default 1.3); the ArduPilot-structured Z loop needs
    the thrust at full command and is built with 3.0 and 70 points.
    """

    def __init__(
        self, vps: VehicleParams, rho: float, soc: float, t_max_factor: float = 1.3, n: int = 40
    ) -> None:
        self.weight = vps.m * G
        self.T = np.linspace(0.0, t_max_factor * self.weight, n)
        d = [0.0]
        for thrust in self.T[1:]:
            om = trim.rotor_speed_for_thrust(float(thrust), 0.0, 0.0, rho, vps)
            d.append(trim.required_rotor_command(om, 0.0, 0.0, 0.0, rho, vps, soc).delta_cmd)
        self.delta = np.array(d)
        # rotor thrust is monotone in the command; the accumulate only guards table noise
        self._delta_up = np.maximum.accumulate(self.delta)

    def delta_for_thrust(self, thrust: float) -> float:
        return float(np.interp(thrust, self.T, self.delta))

    def thrust_for_delta(self, delta: float) -> float:
        """Rotor thrust [N] for a throttle command (the inverse table)."""
        return float(np.interp(delta, self._delta_up, self.T))


class Position1Controller:
    """Rotor altitude hold plus pitch-limited position braking; pusher at zero.

    `ff_mode="lift_aware"` replaces the constant hover throttle of the altitude hold by the
    throttle for the thrust the wing does not supply, W - L. At `ff_lift_tau_s = 0` and
    `ff_lift_scale = 1` this is an ORACLE: it reads the simulated lift, which the aircraft does not
    measure. The filter and the scale stand in for an estimate. The gains are untouched.

    `alpha_cap` limits the pitch command to `alpha_cap + gamma` (wing angle of attack) above
    `alpha_cap_vmin`. `rotor_lag_s` is the one lumped actuator lag of D: a first-order lag on the
    rotor throttle command that stands for everything between ArduPilot's rotor output and the
    rotor-speed response that the vehicle model does not already contain (ESC slew, output
    filtering, throttle smoothing, rotor or motor inertia beyond the placeholder `I_rot`). It adds
    in series with the model's own lag (`braking.rotor_speed_tau`, small at the placeholder
    `I_rot`). It is not MOT_SPOOL_TIME (the rotors are already spooled in AIRBRAKE); its value is
    set from a bench step test, so until then the excursions it produces are a sensitivity.

    `z_mode="ardupilot"` swaps the vertical channel for ArduPilot's loop (O-12): position P gives a
    climb-rate target (limited to `z_vmax`), a velocity P gives an acceleration target, and an
    acceleration PID acts on the measured vertical acceleration and gives a normalised thrust. It
    has no lift or hover feed-forward (`ff_mode` is ignored); the integrator starts at the entry
    thrust (`z_i0`, as ArduPilot starts it at the current throttle). Everything outside the
    vertical channel (pitch law, cascade, pitch limit, `rotor_lag_s`) is unchanged. The defaults
    are ArduPilot master (4.8.0-dev) values, not a pinned tag (O-08). Assumptions carried:

    * `thrust_norm` is linear in rotor thrust (thr = T / T(delta_max)); `MOT_THST_EXPO` is ignored.
    * The measured acceleration is the exact vertical acceleration of the state (finite difference
      of the climb rate over the controller step): no IMU noise, bias or delay.
    * The integrator limit is max(`z_acc_imax`, W / T(delta_max)), so the hover thrust always fits.
    * Tilt compensation is 1 / cos(theta), clamped at 0.5, as in `lift_aware`.
    * `rotor_lag_s` is the lumped first-order lag applied after the throttle mapping.
    """

    def __init__(
        self, p: BackTransitionParams, ff: LiftFF | None = None, z_i0: float | None = None
    ) -> None:
        if p.ff_mode not in ("repo", "lift_aware"):
            raise ValueError(f"unknown ff_mode {p.ff_mode!r}")
        if p.ff_mode == "lift_aware" and ff is None and p.z_mode == "repo":
            raise ValueError("ff_mode 'lift_aware' needs a LiftFF")
        if p.z_mode == "ardupilot":
            if ff is None:
                raise ValueError("z_mode 'ardupilot' needs a LiftFF (its thrust map)")
            if z_i0 is None:
                raise ValueError("z_mode 'ardupilot' needs z_i0 (entry thrust over T(delta_max))")
        self.p = p
        self.ff = ff
        self.z_i0 = z_i0
        self._t_max = ff.thrust_for_delta(p.delta_max) if ff is not None else 0.0

    def _z_loop(
        self, dt: float, h: float, vh: float, theta: float, prev: dict[str, float]
    ) -> tuple[float, dict[str, float]]:
        """The vertical loop with the position P in front: climb target from the altitude error."""
        p = self.p
        v_t = min(max(p.z_pos_p * (p.h_tgt - h), -p.z_vmax), p.z_vmax)
        return self.z_loop(dt, v_t, vh, theta, prev)

    def z_loop(
        self, dt: float, v_target: float, vh: float, theta: float, prev: dict[str, float]
    ) -> tuple[float, dict[str, float]]:
        """ArduPilot's vertical loop from the climb-rate target on (C spec section 3.2).

        Velocity P, acceleration PID on the measured acceleration, thrust map and rotor lag: the
        rotor command and its memory. `v_target` is the climb-rate demand (m/s), already limited by
        the caller; `_z_loop` forms it from the altitude error, the forward transition from O-18.
        """
        p = self.p
        ff, t_max = self.ff, self._t_max
        assert ff is not None and self.z_i0 is not None  # checked in __init__
        boost = 1.0 / max(0.5, math.cos(theta))
        if "z_I" not in prev:  # first call: integrator at the entry thrust, no update
            integ, d_eff = self.z_i0, ff.delta_for_thrust(self.z_i0 * t_max)
            delta = min(max(ff.delta_for_thrust(integ * t_max * boost), p.delta_min), p.delta_max)
            cmd = d_eff if p.rotor_lag_s > 0.0 else delta
            mem = {"z_I": integ, "z_vf": 0.0, "z_ef": 0.0, "z_vh": vh, "z_d": d_eff, "z_cmd": cmd}
            return cmd, mem
        integ, vf, ef, vh_prev, d_eff = (prev[k] for k in ("z_I", "z_vf", "z_ef", "z_vh", "z_d"))
        if dt <= 0.0:  # no time has passed: nothing to update
            return prev["z_cmd"], prev
        a_meas = (vh - vh_prev) / dt  # gravity-compensated vertical acceleration
        ka = dt / (dt + 1.0 / (2.0 * math.pi * p.z_vel_flt_hz))
        vf += ka * ((v_target - vh) - vf)
        a_t = p.z_vel_p * vf
        ke = dt / (dt + 1.0 / (2.0 * math.pi * p.z_acc_flt_hz))
        ef += ke * ((a_t - a_meas) - ef)
        integ = min(max(integ + p.z_acc_i * ef * dt, 0.0), max(p.z_acc_imax, ff.weight / t_max))
        thr = min(max(integ + p.z_acc_p * ef, 0.0), 1.0)  # thrust_norm
        delta = min(max(ff.delta_for_thrust(thr * t_max * boost), p.delta_min), p.delta_max)
        cmd = delta
        if p.rotor_lag_s > 0.0:
            d_eff += (1.0 - math.exp(-dt / p.rotor_lag_s)) * (delta - d_eff)
            cmd = d_eff
        return cmd, {"z_I": integ, "z_vf": vf, "z_ef": ef, "z_vh": vh, "z_d": d_eff, "z_cmd": cmd}

    def command(
        self, t: float, x: np.ndarray, aux: dict[str, float], mode: ModeState | None
    ) -> tuple[np.ndarray, ModeState]:
        p = self.p
        if mode is None:
            mode = ModeState("position1", t_enter=t, t_last=t, memory={"integ": 0.0})
        dt = t - mode.t_last  # integrator advanced by elapsed time, not a stored step size
        h, vx, vh = float(x[1]), float(x[2]), float(x[3])
        mem: dict[str, float] = {}  # filter states, carried because the memory is rebuilt each step
        integ = mode.memory["integ"]
        if p.z_mode == "ardupilot":
            delta, mem = self._z_loop(dt, h, vh, float(x[4]), mode.memory)
        else:
            p_alt = p
            if self.ff is not None and p.ff_mode == "lift_aware":
                l_hat = p.ff_lift_scale * aux["L"]
                if p.ff_lift_tau_s > 0.0 and "L_f" in mode.memory:
                    l_f = mode.memory["L_f"]
                    l_hat = l_f + (1.0 - math.exp(-dt / p.ff_lift_tau_s)) * (l_hat - l_f)
                mem["L_f"] = l_hat
                t_ff = max(0.0, self.ff.weight - l_hat) / max(0.5, math.cos(float(x[4])))
                p_alt = replace(p, delta_hover=self.ff.delta_for_thrust(t_ff))
            delta, integ = altitude_hold(p_alt, h, vh, integ, dt)
            if p.rotor_lag_s > 0.0:
                d_prev = mode.memory.get("d_eff", delta)
                delta = d_prev + (1.0 - math.exp(-dt / p.rotor_lag_s)) * (delta - d_prev)
                mem["d_eff"] = delta
        eas = aux["V"] * math.sqrt(aux["rho"] / RHO0)
        lim = p.theta_limit(t - mode.t_enter, eas)
        if p.alpha_cap is not None and aux["V"] >= p.alpha_cap_vmin:
            lim = min(lim, max(0.0, p.alpha_cap + aux["gamma"]))
        if p.brake_at_limit:
            theta_cmd = lim
        else:
            d = p.x_tgt - float(x[0])
            d_lin = 2.0 * p.a_lim / p.K_p**2
            v_des = math.sqrt(2.0 * p.a_lim * d) if d > d_lin else p.K_p * d
            a_cmd = p.K_v * (vx - v_des)
            theta_cmd = max(0.0, min(lim, math.atan(a_cmd / G)))
        new = ModeState(mode.name, mode.t_enter, t, {"integ": integ, **mem})
        return np.array([delta, 0.0, theta_cmd]), new


@dataclass(frozen=True)
class BaselineParams:
    """Decision vector and assumed structure of the baseline (bare-minimum M5 version).

    Modelled: forward transition (wait for AIRSPEED_MIN at TKOFF_THR_MAX, then linear rotor
    fade over Q_TRANSITION_MS), a cruise stub (rate-limited pusher hand-off, speed and altitude
    hold), the switch-point test (8.11) and Position1. Not modelled: BATT_WATT_MAX, TECS energy
    blending, airbrake. Gains are ASSUMED. Q_M_SPIN_MIN is `back.delta_min`.
    """

    back: BackTransitionParams
    airspeed_min: float = 14.0  # AIRSPEED_MIN
    q_transition_s: float = 5.0  # Q_TRANSITION_MS in seconds
    tkoff_thr_max: float = 0.7  # TKOFF_THR_MAX (pusher command in AUTO)
    q_trans_fail_s: float = 0.0  # Q_TRANS_FAIL; 0 disables the timeout
    a_plan: float = 2.5  # Q_TRANS_DECEL
    speed_test: str = "ground"  # speed used in the switch test (open question O-02)
    v_cruise: float = 22.0
    delta_p_trim: float = 0.5  # cruise pusher command guess
    theta_trim: float = 0.0
    r_tecs: float = 0.5  # pusher command rate limit, 1/s
    tau_h: float = 1.0
    K_s: float = 0.05  # pusher command per m/s speed error
    K_si: float = 0.01
    K_hp: float = 0.02  # rad pitch per metre altitude error
    K_hd: float = 0.03  # rad pitch per m/s climb rate
    theta_cruise_max: float = 0.15


class BaselineController:
    """Forward transition -> cruise stub -> Position1 (modes: fwd_wait, fwd_fade, cruise,
    position1, aborted)."""

    def __init__(self, p: BaselineParams) -> None:
        self.p = p
        self._p1 = Position1Controller(p.back)

    def command(
        self, t: float, x: np.ndarray, aux: dict[str, float], mode: ModeState | None
    ) -> tuple[np.ndarray, ModeState]:
        p, b = self.p, self.p.back
        if mode is None:
            mode = ModeState("fwd_wait", t, t, {"integ": 0.0, "p_cmd": p.tkoff_thr_max})
        mem = dict(mode.memory)
        dt = t - mode.t_last
        h, vh = float(x[1]), float(x[3])
        v_air = aux["V"]
        name, t_enter = mode.name, mode.t_enter

        def switch(new: str) -> None:
            nonlocal name, t_enter
            name, t_enter = new, t
            mem["integ"] = 0.0

        if name == "fwd_wait":
            if v_air >= p.airspeed_min:
                switch("fwd_fade")
            elif p.q_trans_fail_s > 0.0 and t - t_enter > p.q_trans_fail_s:
                switch("aborted")
        if name == "fwd_fade" and t - t_enter >= p.q_transition_s:
            switch("cruise")
            mem["p_cmd"] = p.tkoff_thr_max
        if name == "cruise":
            v_ref = float(x[2]) if p.speed_test == "ground" else v_air
            if b.x_tgt - float(x[0]) <= v_ref**2 / (2.0 * p.a_plan):
                switch("position1")
        new_mem = mem

        if name in ("fwd_wait", "fwd_fade", "aborted"):
            delta, new_mem["integ"] = altitude_hold(b, h, vh, mem["integ"], dt)
            if name == "fwd_fade":
                delta *= max(0.0, 1.0 - (t - t_enter) / p.q_transition_s)  # (8.7)
            if name == "aborted":
                u = np.array([delta, 0.0, 0.0])
            else:
                u = np.array([delta, p.tkoff_thr_max, 0.0])
        elif name == "cruise":
            new_mem["integ"] = mem["integ"] + (p.v_cruise - v_air) * dt
            d_tecs = p.delta_p_trim + p.K_s * (p.v_cruise - v_air) + p.K_si * new_mem["integ"]
            d_tecs = max(0.0, min(1.0, d_tecs))
            rate = max(-p.r_tecs, min(p.r_tecs, (d_tecs - mem["p_cmd"]) / p.tau_h))
            new_mem["p_cmd"] = max(0.0, min(1.0, mem["p_cmd"] + rate * dt))
            th = p.theta_trim + p.K_hp * (b.h_tgt - h) - p.K_hd * vh
            th = max(-p.theta_cruise_max, min(p.theta_cruise_max, th))
            u = np.array([0.0, new_mem["p_cmd"], th])
        else:  # position1
            inner = ModeState("position1", t_enter, mode.t_last, {"integ": mem["integ"]})
            u, inner_new = self._p1.command(t, x, aux, inner)
            new_mem["integ"] = inner_new.memory["integ"]
        return u, ModeState(name, t_enter, t, new_mem)
