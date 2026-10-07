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

from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.control.ardupilot_like import BackTransitionParams, Position1Controller
from sagetrans.dynamics import events as ev
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import Result, simulate
from sagetrans.physics.params import VehicleParams
from sagetrans.physics.vehicle import CasadiVehicle

G = 9.80665


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
) -> BrakingResult:
    """Simulate Position1 from ground speed v0 to ground speed v_f."""
    veh = vehicle or CasadiVehicle(vp)
    bp = replace(bp, h_tgt=scen.altitude_amsl_m)
    start = x0 if x0 is not None else _initial_state(vp, scen, v0, rotors)
    res = simulate(
        veh, Position1Controller(bp), scen.environment(), start, t_end,
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
        bps = replace(bp, delta_hover=trim.hover_throttle(vp, sc.rho, sc.soc))
        for v0 in v0_grid:
            r = measure_a_eq(vp, sc, v0, bps, **kw)
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
    """Switch at d = V_ref^2 / (2 a_plan) and brake with the position cascade (method 3, 5)."""
    v_air = v0 + scen.headwind_ms  # headwind is positive against the aircraft
    v_ref = v0 if speed_test == "ground" else v_air
    bps = replace(
        bp,
        x_tgt=v_ref**2 / (2.0 * a_plan),
        delta_hover=trim.hover_throttle(vp, scen.rho, scen.soc),
        brake_at_limit=False,
    )
    return run_braking(vp, scen, v0, bps, **kw)
