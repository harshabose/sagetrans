"""Analysis B2: speed-scheduled back-transition law tuned over the ensemble (section 10B).

Bare-minimum M5 version: back-transition only, no altitude feedback, CMA-ES from the same
search machinery and cost (10.1) as analysis C. B1 does not exist yet (M6), so the initial
knots come from analysis A's pre-spun schedule instead of B1; where A is infeasible they fall
back to fixed defaults.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sagetrans.analysis import runs
from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.analysis.ensemble import CostWeights, RunMetrics, ensemble_cost
from sagetrans.analysis.search import Bounds, SearchResult, cma_minimise
from sagetrans.analysis.zero_loss import DecelProfile, analyse
from sagetrans.control.freeform import S_KNOTS, FreeFormController, FreeFormParams
from sagetrans.dynamics.simulate import simulate
from sagetrans.physics.params import VehicleParams

N_K = len(S_KNOTS)
DELTA_MAX = 0.9
THETA_RANGE = (-0.1, 0.5)

BOUNDS = Bounds(
    tuple(f"d{i}" for i in range(N_K)) + tuple(f"t{i}" for i in range(N_K)) + ("s_on",),
    (0.0,) * N_K + (THETA_RANGE[0],) * N_K + (0.3,),
    (DELTA_MAX,) * N_K + (THETA_RANGE[1],) * N_K + (S_KNOTS[-1],),
)
FALLBACK_DELTA, FALLBACK_THETA = 0.3, 0.15


def decode(prm: dict[str, float]) -> FreeFormParams:
    return FreeFormParams(
        tuple(prm[f"d{i}"] for i in range(N_K)),
        tuple(prm[f"t{i}"] for i in range(N_K)),
        prm["s_on"],
    )


def init_from_zero_loss(
    vp: VehicleParams, scen: Scenario, v0_air: float, a_init: float = 1.0
) -> dict[str, float]:
    """Initial knots: analysis A's pre-spun schedule re-plotted against V / V_s."""
    vps = scen.vehicle_for(vp)
    vs = runs.stall_speed(vps, scen.rho)
    r = analyse(vps, scen, v0_air - scen.headwind_ms, DecelProfile(a_init), n=60)
    s = r.V / vs
    ok = np.isfinite(r.theta) & np.isfinite(r.delta_cmd)
    prm: dict[str, float] = {}
    for i, sk in enumerate(S_KNOTS):
        if ok.sum() >= 2:
            order = np.argsort(s[ok])
            d = float(np.interp(sk, s[ok][order], r.delta_cmd[ok][order]))
            th = float(np.interp(sk, s[ok][order], r.theta[ok][order]))
        else:
            d, th = FALLBACK_DELTA, FALLBACK_THETA
        prm[f"d{i}"] = float(np.clip(d, 0.0, DELTA_MAX))
        prm[f"t{i}"] = float(np.clip(th, *THETA_RANGE))
    prm["s_on"] = S_KNOTS[-1]
    return prm


def evaluate(
    vp: VehicleParams, scen: Scenario, p: FreeFormParams, v0_air: float = 22.0,
    v_f: float = 1.0, agl0: float = 100.0,
) -> RunMetrics:  # fmt: skip
    vps = scen.vehicle_for(vp)
    x0 = runs.cruise_entry_state(vps, scen, v0_air)
    ctrl = FreeFormController(p, runs.stall_speed(vps, scen.rho))
    sim = simulate(
        runs.vehicle_for(vps), ctrl, scen.environment(), x0, 60.0,
        runs.end_events(vps, scen, v_f, agl0),
    )  # fmt: skip
    return runs.run_metrics(sim, vps, float(x0[1]))


@dataclass
class FreeFormResult:
    params: FreeFormParams
    cost: float
    cost_initial: float
    runs: list[RunMetrics]
    search: SearchResult
    record: RunRecord


def optimise(
    vp: VehicleParams,
    scens: list[Scenario],
    weights: CostWeights = CostWeights(),  # noqa: B008
    init_scen: Scenario | None = None,
    v0_air: float = 22.0,
    seeds: tuple[int, ...] = (1, 2, 3),
    popsize: int = 12,
    maxiter: int = 15,
) -> FreeFormResult:
    """Tune the knots of (8.1) over the ensemble; no altitude feedback."""
    init = init_from_zero_loss(vp, init_scen or scens[0], v0_air)

    def metrics(prm: dict[str, float]) -> list[RunMetrics]:
        fp = decode(prm)
        return [evaluate(vp, s, fp, v0_air) for s in scens]

    def cost(u: np.ndarray) -> float:
        return ensemble_cost(metrics(BOUNDS.to_physical(u)), weights)

    res = cma_minimise(cost, BOUNDS.to_unit(init), seeds, popsize=popsize, maxiter=maxiter)
    best = BOUNDS.to_physical(res.best_x)
    return FreeFormResult(
        decode(best), res.best_f, res.f_start, metrics(best), res,
        make_record(vp, scens, weights, v0_air, seeds, popsize, maxiter),
    )  # fmt: skip


__all__ = ["FreeFormResult", "decode", "evaluate", "init_from_zero_loss", "optimise"]
