"""Analysis B2: speed-scheduled back-transition law tuned over the ensemble (section 10B).

Bare-minimum M5 version: back-transition only, no altitude feedback, CMA-ES from the same
search machinery and cost (10.1) as analysis C. The default starting knots come from analysis A's
pre-spun schedule (where A is infeasible they fall back to fixed defaults). The proposal starts
B2 from B1's solution replotted against V / V_s: `init_from_collocation` does that, and
`optimise(..., init_params=...)` accepts it.

Repeatability (T-12) of B2 was NOT met from the A-based start (seed spread 4-9 %); whether the
B1 start fixes it is not yet shown. Record the spread (`result.search.spread`) in the run record
(`build_record(..., search_spreads={"B2": ...})`) so the release gate can check it.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sagetrans.analysis import collocation, runs
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


def init_from_collocation(
    prob: collocation.Problem, sol: collocation.B1Solution
) -> dict[str, float]:
    """Starting knots from a B1 solution, re-plotted against V / V_s (proposal step 2 of B2).

    The rotor throttle is converted from B1's effective throttle to the ESC command, the knots
    are read off by interpolating over airspeed (the trajectory's speed falls through the knots),
    and `s_on` is the normalised speed at which B1 first switches the rotors on.
    """
    scen = prob.scen
    v_s = runs.stall_speed(prob.vp, scen.rho)
    s = np.hypot(sol.x[:, 2] + scen.headwind_ms, sol.x[:, 3]) / v_s
    dz = prob.dead_zone
    cmd = dz + (1.0 - dz) * sol.u[:, 0]
    order = np.argsort(s)
    prm: dict[str, float] = {}
    for i, sk in enumerate(S_KNOTS):
        prm[f"d{i}"] = float(np.clip(np.interp(sk, s[order], cmd[order]), 0.0, DELTA_MAX))
        prm[f"t{i}"] = float(np.clip(np.interp(sk, s[order], sol.u[order, 1]), *THETA_RANGE))
    on = np.flatnonzero(sol.u[:, 0] > 1e-3)  # first node with the rotors meaningfully on
    s_on = float(s[on[0]]) if on.size else S_KNOTS[-1]
    prm["s_on"] = float(np.clip(s_on, BOUNDS.lo[-1], BOUNDS.hi[-1]))
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
    init_params: dict[str, float] | None = None,
    sigma0: float = 0.25,
) -> FreeFormResult:
    """Tune the knots of (8.1) over the ensemble; no altitude feedback.

    The start is `init_params` if given (for example `init_from_collocation`), otherwise analysis
    A's pre-spun schedule. `sigma0` is CMA-ES's initial step size on the unit cube; a start that is
    already good (B1) wants a smaller one than a poor start.
    """
    init = init_params or init_from_zero_loss(vp, init_scen or scens[0], v0_air)

    def metrics(prm: dict[str, float]) -> list[RunMetrics]:
        fp = decode(prm)
        return [evaluate(vp, s, fp, v0_air) for s in scens]

    def cost(u: np.ndarray) -> float:
        return ensemble_cost(metrics(BOUNDS.to_physical(u)), weights)

    res = cma_minimise(
        cost, BOUNDS.to_unit(init), seeds, sigma0=sigma0, popsize=popsize, maxiter=maxiter
    )
    best = BOUNDS.to_physical(res.best_x)
    return FreeFormResult(
        decode(best), res.best_f, res.f_start, metrics(best), res,
        make_record(vp, scens, weights, v0_air, seeds, popsize, maxiter),
    )  # fmt: skip


__all__ = [
    "FreeFormResult",
    "decode",
    "evaluate",
    "init_from_collocation",
    "init_from_zero_loss",
    "optimise",
]
