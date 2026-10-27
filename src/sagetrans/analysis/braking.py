"""Analysis D: Position1 braking and Q_TRANS_DECEL calibration (section 10D, eq. 10.5).

Every run starts at the switch point (M3; the cruise phase belongs to M5). The target sits
V_ref^2 / (2 a_plan) ahead, with V_ref the ground or air speed (open question O-02).
a_eq uses (V0^2 - V_f^2) / (2 x), which equals eq. (10.5) when V_f = 0.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from sagetrans import atmosphere
from sagetrans.analysis import runs
from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.control.ardupilot_like import BackTransitionParams, LiftFF, Position1Controller
from sagetrans.dynamics import events as ev
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import Result, simulate
from sagetrans.physics import aero, motor, rotor
from sagetrans.physics.params import VehicleParams
from sagetrans.physics.vehicle import CasadiVehicle

G = 9.80665
RHO0 = float(atmosphere.density(0.0))


def ideal_stopping(
    v0: float, theta_max: float, t_bt: float, v_end: float = 0.0, dt: float = 1e-4
) -> tuple[float, float, float]:
    """Planner worked example: ideal pitch, L and D neglected, braking at the limit throughout.

    Returns (stopping distance, a_eq, distance covered during the ramp).
    """
    t, v, x, x_ramp = 0.0, v0, 0.0, 0.0
    while v > v_end:
        frac = 1.0 if t_bt <= 0 else min(1.0, t / t_bt)
        a = G * math.tan(theta_max * frac)
        v_new = v - a * dt
        x += 0.5 * (v + v_new) * dt
        if t < t_bt:
            x_ramp = x
        v, t = v_new, t + dt
    return x, (v0**2 - v_end**2) / (2.0 * x), x_ramp


@dataclass
class BrakingResult:
    x_actual: float
    a_eq: float
    t_stop: float
    altitude_excursion: float
    overshoot: float | None
    sim: Result
    record: RunRecord


def _initial_state(vp: VehicleParams, scen: Scenario, v0: float, rotors: str) -> np.ndarray:
    om = trim.hover_omega(vp, scen.rho) if rotors == "hover" else 0.0
    return np.array([0.0, scen.altitude_amsl_m, v0, 0.0, 0.0, 0.0, om, 0.0, 0.0])


def entry_airspeed(
    airspeed_min: float, q_assist_speed: float, rho: float, convention: str = "tas"
) -> float:
    """True airspeed at the AIRBRAKE entry: the exit rule max(AIRSPEED_MIN - 2, Q_ASSIST_SPEED).

    The repo compares AIRSPEED_MIN as a true airspeed (`"tas"`, the rule's value is returned as
    is); `"eas"` reads it as an equivalent airspeed and converts at the scenario density (RC5).
    """
    v = max(airspeed_min - 2.0, q_assist_speed)
    if convention == "tas":
        return v
    if convention == "eas":
        return v * math.sqrt(RHO0 / rho)
    raise ValueError(f"unknown entry convention {convention!r}")


def airbrake_entry_state(
    vp: VehicleParams, scen: Scenario, bp: BackTransitionParams, v_air0: float, v_f: float = 1.0
) -> tuple[np.ndarray, float]:
    """State at the AIRBRAKE exit and the ground speed: rotors already carrying W - L.

    Pitch is `theta_fw0`, the pusher is off and the rotors are spun up to the thrust the wing does
    not supply (`hold_hover(0)`, no spool dynamics). The 10% floor keeps them turning when the wing
    alone would carry the weight; it is an assumption, not an ArduPilot value.
    """
    v0_ground = v_air0 - scen.headwind_ms  # headwind is positive against the aircraft
    if v0_ground <= v_f:
        raise ValueError(f"ground speed {v0_ground:.2f} m/s at entry is not above {v_f:g} m/s")
    vps = scen.vehicle_for(vp)
    lift0 = float(aero.forces(scen.rho, v_air0, bp.theta_fw0 + vps.wing.i_w, vps.wing)[0])
    weight = vps.m * G
    t0 = max(weight - lift0, 0.10 * weight) / max(0.5, math.cos(bp.theta_fw0))
    om0 = trim.rotor_speed_for_thrust(t0, 0.0, 0.0, scen.rho, vps)
    x0 = np.array([0.0, scen.altitude_amsl_m, v0_ground, 0.0, bp.theta_fw0, 0.0, om0, 0.0, 0.0])
    return x0, v0_ground


def rotor_speed_tau(vp: VehicleParams, rho: float, soc: float) -> float:
    """Time constant [s] of the rotor-speed response to a small throttle change at hover.

    Linearises omega_dot = (Q_motor(omega, delta, V_bus) - Q_aero(omega)) / I_rot at the hover speed
    (throttle and bus voltage from `trim.hover_state`, the bus voltage held) with a central
    difference of relative size 1e-3, and returns -1 / d(omega_dot)/d(omega). It scales linearly
    with I_rot. This is the vehicle model's own lag; `rotor_lag_s` adds to it in series.
    """
    hs = trim.hover_state(vp, soc=soc, rho=rho)
    omega = trim.hover_omega(vp, rho)
    d_eff = float(motor.effective_throttle(hs.delta_r_cmd, vp.motor_r))

    def omega_dot(w: float) -> float:
        q_m = float(motor.torque(motor.current(d_eff, hs.v_bus, w, vp.motor_r), vp.motor_r))
        q_a = float(rotor.lift_rotor(w, 0.0, 0.0, rho, vp.rotor)[1])
        return (q_m - q_a) / vp.rotor.I_rot

    h = 1e-3 * omega
    slope = (omega_dot(omega + h) - omega_dot(omega - h)) / (2.0 * h)
    return -1.0 / slope


def braking_flags(
    res: Result,
    vp: VehicleParams,
    scen: Scenario,
    bp: BackTransitionParams,
    *,
    alpha_hi: float,
    alpha_min: float | None,
    kappa: float | None,
    gate: float = 5.0,
    sink_tol: float = 0.5,
) -> dict[str, float | bool]:
    """Checks on a braking run. They only report: none changes the run or `a_eq`.

    `alpha_hi` and `alpha_min` are wing angles of attack (the C_L = 0.8 C_L,max angle and the
    lower limit); `alpha_min` None disables that check. Samples with `V` below `gate` are left out
    of the angle-of-attack integrals, where the wing carries little weight and alpha means nothing.
    """
    vps = scen.vehicle_for(vp)
    t = res.t
    dt = np.diff(t)
    weight = vps.m * G
    v_air = res["V"]
    alpha_w = res["alpha"] + vps.wing.i_w
    gated = v_air >= gate

    def span(mask: np.ndarray) -> float:
        return float(np.sum(dt * mask[:-1])) if len(t) > 1 else 0.0

    def first(mask: np.ndarray) -> int | None:
        idx = np.flatnonzero(mask)
        return int(idx[0]) if len(idx) else None

    nan = math.nan
    k08 = first(gated & (alpha_w > alpha_hi))
    ratio = res["vrs_ratio"]
    i_vrs = int(np.argmin(ratio))
    d_r = res["delta_r"]
    vh_end, dh_end = float(res["vh"][-1]), float(res["h"][-1] - res["h"][0])
    peak_i = float(np.max(res["I_rotor"]))
    min_v = float(np.min(res["V_bus"]))
    return {
        "alpha_max_deg": math.degrees(float(np.max(alpha_w))),
        "alpha_max_gated_deg": math.degrees(float(np.max(alpha_w[gated]))) if gated.any() else nan,
        "t_alpha_08_first_s": float(t[k08]) if k08 is not None else nan,
        "V_at_alpha_08_first_ms": float(v_air[k08]) if k08 is not None else nan,
        "t_above_alpha_08_s": span(gated & (alpha_w > alpha_hi)),
        "t_above_alpha_s_s": span(gated & (alpha_w > vps.wing.alpha_s)),
        "t_below_alpha_min_s": 0.0 if alpha_min is None else span(gated & (alpha_w < alpha_min)),
        "rotor_share_max": float(np.max(res["T_r"] * np.cos(res["theta"]) / weight)),
        "wing_share_min": float(np.min(res["L"][gated] / weight)) if gated.any() else nan,
        "throttle_margin_min": float(np.min((bp.delta_max - d_r) / bp.delta_max)),
        "t_throttle_sat_s": span(d_r >= bp.delta_max - 1e-3),
        "vrs_min": float(ratio[i_vrs]),
        "vrs_min_t_s": float(t[i_vrs]),
        "t_vrs_onset_s": nan if kappa is None else span(ratio <= -kappa),
        "peak_motor_current_A": peak_i,
        "current_over": peak_i > vps.motor_r.I_lim,
        "min_bus_voltage_V": min_v,
        "voltage_under": min_v < vps.battery.V_min,
        "vh_end_ms": vh_end,
        "dh_end_m": dh_end,
        "stopped": res.terminated_by == "ground_speed_reached",
        "sinking": vh_end < -sink_tol,
    }


def run_braking(
    vp: VehicleParams,
    scen: Scenario,
    v0: float,
    bp: BackTransitionParams,
    *,
    v_f: float = 1.0,
    rotors: str = "hover",
    t_end: float = 60.0,
    agl0: float = 100.0,
    vehicle: CasadiVehicle | None = None,
    x0: np.ndarray | None = None,
    refresh_hover: bool = True,
) -> BrakingResult:
    """Simulate Position1 from ground speed v0 to ground speed v_f.

    The scenario's mass is applied (`scen.vehicle_for`) to the model, to the initial rotor speed
    and, unless `refresh_hover` is false, to the hover throttle feed-forward of the altitude hold.
    A `vehicle` passed in explicitly (a test harness) is used as given.
    """
    vps = scen.vehicle_for(vp)
    veh = vehicle or runs.vehicle_for(vps)
    bp = replace(bp, h_tgt=scen.altitude_amsl_m)
    if refresh_hover:
        bp = replace(bp, delta_hover=trim.hover_throttle(vps, scen.rho, scen.soc))
    start = x0 if x0 is not None else _initial_state(vps, scen, v0, rotors)
    ff = LiftFF(vps, scen.rho, scen.soc) if bp.ff_mode == "lift_aware" else None
    res = simulate(
        veh, Position1Controller(bp, ff), scen.environment(), start, t_end,
        [ev.ground_speed_below(v_f), ev.touchdown(scen.altitude_amsl_m - agl0)],
    )  # fmt: skip
    stopped = res.terminated_by == "ground_speed_reached"
    x_act = float(res["x"][-1]) if stopped else math.nan
    a_eq = (v0**2 - v_f**2) / (2.0 * x_act) if stopped else math.nan
    return BrakingResult(
        x_act, a_eq, float(res.t[-1]), res.metrics["altitude_excursion"],
        (x_act - bp.x_tgt) if stopped else None, res, make_record(vp, scen, v0, bp, v_f, rotors),
    )  # fmt: skip


def measure_a_eq(
    vp: VehicleParams, scen: Scenario, v0: float, bp: BackTransitionParams, **kw: Any
) -> BrakingResult:
    """Worst-case braking: pitch at the envelope limit throughout (method step 1)."""
    return run_braking(vp, scen, v0, replace(bp, brake_at_limit=True), **kw)


def recommend_decel(
    vp: VehicleParams,
    scenarios: list[Scenario],
    v0_grid: list[float],
    bp: BackTransitionParams,
    percentile: float | None = None,
    **kw: Any,
) -> tuple[float, list[tuple[Scenario, float, float]]]:
    """a_rec: minimum (or the given lower percentile) of a_eq over scenarios and entry speeds."""
    rows: list[tuple[Scenario, float, float]] = []
    for sc in scenarios:
        for v0 in v0_grid:
            r = measure_a_eq(vp, sc, v0, bp, **kw)
            if not math.isnan(r.a_eq):
                rows.append((sc, v0, r.a_eq))
    vals = np.array([r[2] for r in rows])
    a_rec = float(vals.min() if percentile is None else np.percentile(vals, percentile))
    return a_rec, rows


def landing_error(
    vp: VehicleParams,
    scen: Scenario,
    v0: float,
    a_plan: float,
    bp: BackTransitionParams,
    speed_test: str = "ground",
    **kw: Any,
) -> BrakingResult:
    """Switch at d = V_ref^2 / (2 a_plan) and brake with the position cascade (method 3, 5).

    `a_plan` only places the target. The cascade never reads it and brakes with `bp.a_lim`;
    ArduPilot's Q_TRANS_DECEL does both jobs, this repo splits them.
    """
    v_air = v0 + scen.headwind_ms  # headwind is positive against the aircraft
    v_ref = v0 if speed_test == "ground" else v_air
    bps = replace(bp, x_tgt=v_ref**2 / (2.0 * a_plan), brake_at_limit=False)
    return run_braking(vp, scen, v0, bps, **kw)
