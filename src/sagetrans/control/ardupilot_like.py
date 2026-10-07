"""ArduPilot-structured baseline controller (section 8.2).

`Position1Controller` is the back-transition part: pusher off, rotor altitude hold (8.5), the
pitch envelope (8.3), and the position cascade (8.9)-(8.10) mapped to pitch by (8.4).
`BaselineController` adds the forward transition (wait for AIRSPEED_MIN, then a linear rotor
fade), a cruise stub with a rate-limited pusher hand-off, and the switch-point test (8.11).
Not modelled: BATT_WATT_MAX, TECS energy blending, the airbrake. Gains are ASSUMED placeholders.

Q_BCK_PIT_LIM blending with airspeed is unknown; theta_B is applied as a constant cap.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from sagetrans.control.base import ModeState
from sagetrans.dynamics import trim
from sagetrans.physics.params import VehicleParams

G = 9.80665


@dataclass(frozen=True)
class BackTransitionParams:
    theta_A: float = math.radians(20.0)  # Q_A_ANGLE_MAX
    theta_P: float = math.radians(25.0)  # PTCH_LIM_MAX_DEG
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

    @classmethod
    def for_vehicle(
        cls, vp: VehicleParams, rho: float, soc: float, h_tgt: float, **kw: Any
    ) -> BackTransitionParams:
        base = cls(h_tgt=h_tgt, delta_hover=trim.hover_throttle(vp, rho, soc))
        return replace(base, **kw)

    def theta_limit(self, t_rel: float) -> float:
        """theta_lim(t) of (8.3)."""
        cap = min(self.theta_A, self.theta_P)
        ramp = cap if self.T_bt <= 0.0 else cap * min(1.0, max(t_rel, 0.0) / self.T_bt)
        return ramp if self.theta_B is None else min(ramp, self.theta_B)


def altitude_hold(
    p: BackTransitionParams, h: float, vh: float, integ: float, dt: float
) -> tuple[float, float]:
    """Rotor altitude hold (8.5): returns the throttle command and the updated integral."""
    v_dem = max(-p.v_dem_max, min(p.v_dem_max, p.K_h * (p.h_tgt - h)))
    err = v_dem - vh
    integ = max(-p.i_max, min(p.i_max, integ + err * dt))
    delta = p.delta_hover + p.K_vh * err + p.K_i * integ
    return max(p.delta_min, min(p.delta_max, delta)), integ


class Position1Controller:
    """Rotor altitude hold plus pitch-limited position braking; pusher at zero."""

    def __init__(self, p: BackTransitionParams) -> None:
        self.p = p

    def command(
        self, t: float, x: np.ndarray, aux: dict[str, float], mode: ModeState | None
    ) -> tuple[np.ndarray, ModeState]:
        p = self.p
        if mode is None:
            mode = ModeState("position1", t_enter=t, t_last=t, memory={"integ": 0.0})
        dt = t - mode.t_last  # integrator advanced by elapsed time, not a stored step size
        h, vx, vh = float(x[1]), float(x[2]), float(x[3])
        delta, integ = altitude_hold(p, h, vh, mode.memory["integ"], dt)
        lim = p.theta_limit(t - mode.t_enter)
        if p.brake_at_limit:
            theta_cmd = lim
        else:
            d = p.x_tgt - float(x[0])
            d_lin = 2.0 * p.a_lim / p.K_p**2
            v_des = math.sqrt(2.0 * p.a_lim * d) if d > d_lin else p.K_p * d
            a_cmd = p.K_v * (vx - v_des)
            theta_cmd = max(0.0, min(lim, math.atan(a_cmd / G)))
        new = ModeState(mode.name, mode.t_enter, t, {"integ": integ})
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
