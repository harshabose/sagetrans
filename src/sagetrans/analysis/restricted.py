"""Analysis C: restricted-parameter optimisation of the ArduPilot-structured baseline (10C).

Bare-minimum M5 version. Two scopes:

* ``back``: the back-transition only (Q_BACKTRANS_MS, Q_A_ANGLE_MAX, Q_M_SPIN_MIN), started from
  an idealised level cruise. Directly comparable with the free-form law of analysis B2.
* ``full``: forward transition, cruise stub and back-transition from hover, adding AIRSPEED_MIN,
  Q_TRANSITION_MS and TKOFF_THR_MAX. Cost: worst excursion of the two transitions, summed
  energy, back-transition distance.

Not in the decision vector: PTCH_LIM_MAX_DEG (fixed at 35 deg), Q_TRANS_FAIL (off), BATT_WATT_MAX
(not modelled).

Q_TRANS_DECEL is not decided by the search: each evaluation sets it from analysis D, as the
proposal asks (section 10C step 4). By default (`a_plan_source="simulated"`) it is the worst-case
a_eq that `braking.measure_a_eq` simulates for the same parameters, scenario and entry state, so
the planner is self-consistent with what the aircraft can actually do. `"ideal"` uses the
ideal-pitch planner instead (no simulation, about half the cost); that figure ignores wing drag,
lift and pitch lag, so it is neither an upper nor a lower bound on the simulated one.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from sagetrans.analysis import braking, runs
from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.analysis.ensemble import CostWeights, RunMetrics, ensemble_cost
from sagetrans.analysis.search import Bounds, SearchResult, cma_minimise
from sagetrans.control.ardupilot_like import (
    BackTransitionParams,
    BaselineController,
    BaselineParams,
    Position1Controller,
)
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import simulate
from sagetrans.physics.params import VehicleParams

THETA_P = math.radians(35.0)

BACK_BOUNDS = Bounds(
    ("T_bt", "theta_A", "spin_min"),
    (1.0, math.radians(10.0), 0.06),
    (6.0, math.radians(30.0), 0.25),
)
FULL_BOUNDS = Bounds(
    BACK_BOUNDS.names + ("airspeed_min", "q_transition", "tkoff_thr_max"),
    BACK_BOUNDS.lo + (11.0, 2.0, 0.5),
    BACK_BOUNDS.hi + (18.0, 8.0, 1.0),
)
# a mid-range start that is not the optimum (documented defaults of the placeholders)
DEFAULT_PARAMS = {
    "T_bt": 3.0, "theta_A": math.radians(20.0), "spin_min": 0.1,
    "airspeed_min": 14.0, "q_transition": 5.0, "tkoff_thr_max": 0.7,
}  # fmt: skip


def _plan_decel(v_ground: float, theta_a: float, t_bt: float, v_f: float) -> float:
    return braking.ideal_stopping(v_ground, theta_a, t_bt, v_end=v_f, dt=2e-3)[1]


def plan_decel(
    vps: VehicleParams, scen: Scenario, prm: dict[str, float], x0: np.ndarray, v_f: float,
    source: str = "simulated", agl0: float = 100.0,
    controller_overrides: dict[str, float] | None = None,
) -> float:  # fmt: skip
    """Q_TRANS_DECEL for one evaluation, from analysis D.

    `"simulated"`: the worst-case a_eq of `braking.measure_a_eq` (pitch at the envelope limit
    throughout) from the same entry state `x0`. `"ideal"`: the ideal-pitch planner. If the
    simulated run does not stop, the ideal figure is used.
    """
    ideal = _plan_decel(float(x0[2]), prm["theta_A"], prm["T_bt"], v_f)
    if source == "ideal":
        return ideal
    if source != "simulated":
        raise ValueError(f"unknown a_plan_source {source!r}")
    bp = BackTransitionParams.for_vehicle(
        vps, scen.rho, scen.soc, scen.altitude_amsl_m, T_bt=prm["T_bt"],
        theta_A=prm["theta_A"], theta_P=THETA_P, delta_min=prm["spin_min"],
        **(controller_overrides or {}),
    )  # fmt: skip
    r = braking.measure_a_eq(vps, scen, float(x0[2]), bp, v_f=v_f, agl0=agl0, x0=x0)
    return ideal if math.isnan(r.a_eq) else r.a_eq


def evaluate_back(
    vp: VehicleParams, scen: Scenario, prm: dict[str, float], v0_air: float = 22.0,
    v_f: float = 1.0, agl0: float = 100.0, controller_overrides: dict[str, float] | None = None,
    a_plan_source: str = "simulated",
) -> RunMetrics:  # fmt: skip
    """One back-transition from level cruise with the position cascade toward the planned point.

    `controller_overrides` replaces fields of BackTransitionParams (e.g. altitude-hold gains).
    `a_plan_source` selects how Q_TRANS_DECEL is set (see `plan_decel`).
    """
    vps = scen.vehicle_for(vp)
    x0 = runs.cruise_entry_state(vps, scen, v0_air)
    a_plan = plan_decel(vps, scen, prm, x0, v_f, a_plan_source, agl0, controller_overrides)
    bp = BackTransitionParams.for_vehicle(
        vps, scen.rho, scen.soc, scen.altitude_amsl_m,
        T_bt=prm["T_bt"], theta_A=prm["theta_A"], theta_P=THETA_P, delta_min=prm["spin_min"],
        x_tgt=float(x0[2]) ** 2 / (2.0 * a_plan),
        **(controller_overrides or {}),
    )  # fmt: skip
    sim = simulate(
        runs.vehicle_for(vps), Position1Controller(bp), scen.environment(), x0, 60.0,
        runs.end_events(vps, scen, v_f, agl0),
    )  # fmt: skip
    return runs.run_metrics(sim, vps, float(x0[1]))


def evaluate_full(
    vp: VehicleParams, scen: Scenario, prm: dict[str, float], v_cruise: float = 22.0,
    v_f: float = 1.0, agl0: float = 100.0, x_tgt: float = 800.0, t_end: float = 90.0,
    a_plan_source: str = "simulated",
) -> RunMetrics:  # fmt: skip
    """Hover -> forward transition -> cruise stub -> back-transition."""
    vps = scen.vehicle_for(vp)
    h0 = scen.altitude_amsl_m
    tr = trim.steady_trim(vps, scen.rho, v_cruise, 0.0, scen.soc)
    entry = runs.cruise_entry_state(vps, scen, v_cruise)  # where the back-transition starts
    a_plan = plan_decel(vps, scen, prm, entry, v_f, a_plan_source, agl0)
    back = BackTransitionParams.for_vehicle(
        vps, scen.rho, scen.soc, h0, T_bt=prm["T_bt"], theta_A=prm["theta_A"], theta_P=THETA_P,
        delta_min=prm["spin_min"], x_tgt=x_tgt,
    )  # fmt: skip
    bp = BaselineParams(
        back=back, airspeed_min=prm["airspeed_min"], q_transition_s=prm["q_transition"],
        tkoff_thr_max=prm["tkoff_thr_max"], a_plan=a_plan, v_cruise=v_cruise,
        delta_p_trim=tr.elec.delta_p_cmd, theta_trim=tr.theta,
    )  # fmt: skip
    x0 = np.array([0.0, h0, 0.0, 0.0, 0.0, 0.0, trim.hover_omega(vps, scen.rho), 0.0, 0.0])
    sim = simulate(
        runs.vehicle_for(vps), BaselineController(bp), scen.environment(), x0, t_end,
        runs.end_events(vps, scen, v_f, agl0),
    )  # fmt: skip
    modes = np.array(sim.modes)
    fwd = np.isin(modes, ("fwd_wait", "fwd_fade", "aborted"))
    bck = modes == "position1"
    fails = runs.failure_flags(sim, vps)
    if not np.any(modes == "cruise") and not np.any(bck):
        fails.append("no_transition")
    ef, en_f, _ = runs.segment_stats(sim, fwd, h0)
    eb, en_b, db = runs.segment_stats(sim, bck, h0)
    return RunMetrics(max(ef, eb), en_f + en_b, db, tuple(sorted(set(fails))))


@dataclass
class RestrictedResult:
    scope: str
    params: dict[str, float]
    cost: float
    cost_default: float
    runs: list[RunMetrics]
    search: SearchResult
    record: RunRecord


def optimise(
    vp: VehicleParams,
    scens: list[Scenario],
    scope: str = "back",
    weights: CostWeights = CostWeights(),  # noqa: B008
    seeds: tuple[int, ...] = (1, 2, 3),
    popsize: int = 8,
    maxiter: int = 15,
    a_plan_source: str = "simulated",
) -> RestrictedResult:
    """CMA-ES over the decision vector, scored with (10.1) over the ensemble.

    `result.search.spread` is the real-problem T-12 figure; pass it to
    `io.record.build_record(..., search_spreads={"C": result.search.spread})` so the release
    gate can check it.
    """
    bounds = BACK_BOUNDS if scope == "back" else FULL_BOUNDS
    ev = evaluate_back if scope == "back" else evaluate_full

    def metrics(prm: dict[str, float]) -> list[RunMetrics]:
        return [ev(vp, s, prm, a_plan_source=a_plan_source) for s in scens]

    def cost(u: np.ndarray) -> float:
        return ensemble_cost(metrics(bounds.to_physical(u)), weights)

    u0 = bounds.to_unit(DEFAULT_PARAMS)
    res = cma_minimise(cost, u0, seeds, popsize=popsize, maxiter=maxiter)
    best = bounds.to_physical(res.best_x)
    return RestrictedResult(
        scope, best, res.best_f, res.f_start, metrics(best), res,
        make_record(vp, scens, scope, weights, seeds, popsize, maxiter, a_plan_source),
    )  # fmt: skip


__all__ = ["RestrictedResult", "evaluate_back", "evaluate_full", "optimise", "plan_decel"]
