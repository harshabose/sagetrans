"""Analysis C: restricted-parameter optimisation of the ArduPilot-structured baseline (10C).

**Current: `evaluate_c` / `optimise_c` / `capability_ceiling`** (docs/C_fix_spec.md). C starts at
the AIRBRAKE exit like analysis D (same entry state and entry speed rule), runs the position
cascade (`brake_at_limit = False`) with ArduPilot's vertical loop (`z_mode="ardupilot"`, O-12) and
searches four variables: Q_BACKTRANS_MS (`T_bt`), Q_A_ANGLE_MAX (`theta_A`), Q_M_SPIN_MIN
(`spin_min`) and Q_TRANS_DECEL (`q`). `q` does both jobs it does in ArduPilot: it places the
stopping point, x_tgt = V_ground^2 / (2 q) (O-02), and it is the cascade's acceleration limit
(O-11). An overshoot of the stopping point beyond `os_tol_m` and an end sink rate beyond
`sink_tol_ms` are failures, so the search cannot pick a `q` the pitch ramp cannot deliver.

**Legacy: `evaluate_back`, `plan_decel`, `evaluate_full`, `optimise`, `BACK_BOUNDS`, `FULL_BOUNDS`,
`DEFAULT_PARAMS`, `THETA_P`.** The old scopes: ``back`` starts from an idealised level cruise and
takes Q_TRANS_DECEL from a planner run (`a_plan_source`), ``full`` adds the forward transition and
AIRSPEED_MIN, Q_TRANSITION_MS and TKOFF_THR_MAX. They stay because `analysis/sensitivity.py` and
`collocation.bound_check` call them; B1 is compared with C on the AIRBRAKE-exit entry through
`evaluate_c`. The study no longer runs them.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field, replace
from functools import lru_cache
from typing import Any

import numpy as np

from sagetrans.analysis import braking, runs
from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.analysis.ensemble import CostWeights, RunMetrics, ensemble_cost
from sagetrans.analysis.search import Bounds, SearchResult, cma_minimise
from sagetrans.control.ardupilot_like import (
    BackTransitionParams,
    BaselineController,
    BaselineParams,
    LiftFF,
    Position1Controller,
)
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import Result, simulate
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
    controller_overrides: dict[str, float] | None = None, theta_p: float = THETA_P,
) -> float:  # fmt: skip
    """LEGACY (see the module docstring). Q_TRANS_DECEL for one evaluation, from analysis D.

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
        theta_A=prm["theta_A"], theta_P=theta_p, delta_min=prm["spin_min"],
        **(controller_overrides or {}),
    )  # fmt: skip
    r = braking.measure_a_eq(vps, scen, float(x0[2]), bp, v_f=v_f, agl0=agl0, x0=x0)
    return ideal if math.isnan(r.a_eq) else r.a_eq


def evaluate_back(
    vp: VehicleParams, scen: Scenario, prm: dict[str, float], v0_air: float = 22.0,
    v_f: float = 1.0, agl0: float = 100.0, controller_overrides: dict[str, float] | None = None,
    a_plan_source: str = "simulated", theta_p: float = THETA_P, v0_eas: float | None = None,
) -> RunMetrics:  # fmt: skip
    """LEGACY. One back-transition from level cruise with the position cascade to the plan.

    `controller_overrides` replaces fields of BackTransitionParams (e.g. altitude-hold gains).
    `a_plan_source` selects how Q_TRANS_DECEL is set (see `plan_decel`). `theta_p` is the fixed
    PTCH_LIM_MAX_DEG. If `v0_eas` is given the entry speed is that EQUIVALENT airspeed converted
    to true airspeed at the scenario's density (so it means the same thing at every altitude).
    """
    vps = scen.vehicle_for(vp)
    if v0_eas is not None:
        v0_air = v0_eas * math.sqrt(1.225 / scen.rho)
    x0 = runs.cruise_entry_state(vps, scen, v0_air)
    a_plan = plan_decel(
        vps, scen, prm, x0, v_f, a_plan_source, agl0, controller_overrides, theta_p
    )
    bp = BackTransitionParams.for_vehicle(
        vps, scen.rho, scen.soc, scen.altitude_amsl_m,
        T_bt=prm["T_bt"], theta_A=prm["theta_A"], theta_P=theta_p, delta_min=prm["spin_min"],
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
    a_plan_source: str = "simulated", theta_p: float = THETA_P,
    controller_overrides: dict[str, float] | None = None,
    baseline_overrides: dict[str, Any] | None = None,
) -> RunMetrics:  # fmt: skip
    """LEGACY. Hover -> forward transition -> cruise stub -> back-transition.

    `baseline_overrides` sets the BaselineParams fields the search does not decide (stub gains,
    Q_TRANS_FAIL, switch test); a `v_cruise` entry in it replaces the `v_cruise` argument.
    """
    bl_kw = dict(baseline_overrides or {})
    v_cruise = float(bl_kw.pop("v_cruise", v_cruise))
    vps = scen.vehicle_for(vp)
    h0 = scen.altitude_amsl_m
    tr = trim.steady_trim(vps, scen.rho, v_cruise, 0.0, scen.soc)
    entry = runs.cruise_entry_state(vps, scen, v_cruise)  # where the back-transition starts
    a_plan = plan_decel(
        vps, scen, prm, entry, v_f, a_plan_source, agl0, controller_overrides, theta_p
    )
    back = BackTransitionParams.for_vehicle(
        vps, scen.rho, scen.soc, h0, T_bt=prm["T_bt"], theta_A=prm["theta_A"], theta_P=theta_p,
        delta_min=prm["spin_min"], x_tgt=x_tgt, **(controller_overrides or {}),
    )  # fmt: skip
    bp = BaselineParams(
        back=back, airspeed_min=prm["airspeed_min"], q_transition_s=prm["q_transition"],
        tkoff_thr_max=prm["tkoff_thr_max"], a_plan=a_plan, v_cruise=v_cruise,
        delta_p_trim=tr.elec.delta_p_cmd, theta_trim=tr.theta, **bl_kw,
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
    at_bound: dict[str, str] = field(default_factory=dict)  # variable -> "lo" | "hi" within 10%


def optimise(
    vp: VehicleParams,
    scens: list[Scenario],
    scope: str = "back",
    weights: CostWeights = CostWeights(),  # noqa: B008
    seeds: tuple[int, ...] = (1, 2, 3),
    popsize: int = 8,
    maxiter: int = 15,
    a_plan_source: str = "simulated",
    *,
    theta_p: float = THETA_P,
    controller_overrides: dict[str, float] | None = None,
    baseline_overrides: dict[str, Any] | None = None,
    start: dict[str, float] | None = None,
    v0_eas: float | None = None,
) -> RestrictedResult:
    """LEGACY. CMA-ES over the old decision vector, scored with (10.1) over the ensemble.

    `start` is the search's starting point (default `DEFAULT_PARAMS`); the shipped config passes the
    register's own values. `v0_eas` sets the back-transition entry speed as an equivalent airspeed.
    `result.search.spread` is the real-problem T-12 figure; pass it to
    `io.record.build_record(..., search_spreads={"C": result.search.spread})` so the release
    gate can check it.
    """
    bounds = BACK_BOUNDS if scope == "back" else FULL_BOUNDS
    ev = evaluate_back if scope == "back" else evaluate_full

    common: dict[str, Any] = {
        "a_plan_source": a_plan_source, "theta_p": theta_p,
        "controller_overrides": controller_overrides,
    }
    if scope == "back":
        common["v0_eas"] = v0_eas
    else:
        common["baseline_overrides"] = baseline_overrides

    def metrics(prm: dict[str, float]) -> list[RunMetrics]:
        return [ev(vp, s, prm, **common) for s in scens]

    def cost(u: np.ndarray) -> float:
        return ensemble_cost(metrics(bounds.to_physical(u)), weights)

    u0 = bounds.to_unit({**DEFAULT_PARAMS, **(start or {})})
    res = cma_minimise(cost, u0, seeds, popsize=popsize, maxiter=maxiter)
    best = bounds.to_physical(res.best_x)
    return RestrictedResult(
        scope, best, res.best_f, res.f_start, metrics(best), res,
        make_record(vp, scens, scope, weights, seeds, popsize, maxiter, a_plan_source, start,
                    theta_p, controller_overrides, baseline_overrides, v0_eas),
    )  # fmt: skip


# ---- current: analysis C from the AIRBRAKE exit (docs/C_fix_spec.md) ---------------------------
C_BOUNDS = Bounds(
    ("T_bt", "theta_A", "spin_min", "q"),
    (1.0, math.radians(10.0), 0.06, 1.0),
    (6.0, math.radians(30.0), 0.25, 3.0),
)
C_Q_START = 2.0  # Q_TRANS_DECEL of the search's start point and of the register comparison row


@dataclass(frozen=True)
class CEval:
    """Settings of one C evaluation that the search does not decide."""

    airspeed_min: float  # AIRSPEED_MIN, m/s
    entry_convention: str = "tas"  # how AIRSPEED_MIN is read: "tas" | "eas"
    rotor_start: str = "idle"  # rotors at the AIRBRAKE exit: "idle" (O-13) | "carry" (W - L)
    os_tol_m: float = 2.0  # overshoot past x_tgt that counts as a failure (proposed)
    sink_tol_ms: float = 1.76  # end sink rate that counts as sinking (proposed)
    end_speed: float = 1.0  # ground speed at which a run ends, m/s
    agl0: float = 100.0  # height below the start at which a run counts as touched down
    t_end: float = 60.0


@dataclass(frozen=True)
class CRun:
    """One C run: the metrics of the cost and what the report needs. `sim` is the full trace."""

    metrics: RunMetrics
    v_air0: float
    v_ground0: float
    x_tgt: float
    stopped: bool
    x_end: float  # NaN unless the run stopped
    overshoot: float  # x_end - x_tgt, NaN unless the run stopped
    vh_end: float  # climb rate at the end, NaN unless the run stopped
    sim: Result


@lru_cache(maxsize=256)
def _scenario_tables(vps: VehicleParams, rho: float, soc: float) -> tuple[LiftFF, float]:
    """The thrust map and the hover throttle of one scenario (they do not depend on the search)."""
    return LiftFF(vps, rho, soc, t_max_factor=3.0, n=70), trim.hover_throttle(vps, rho, soc)


def _simulate_c(
    vp: VehicleParams, scen: Scenario, prm: dict[str, float], cfg: CEval,
    base_bp: BackTransitionParams, at_limit: bool,
) -> CRun:  # fmt: skip
    vps = scen.vehicle_for(vp)
    v_air = braking.entry_airspeed(
        cfg.airspeed_min, scen.rho, cfg.entry_convention
    )
    ff, d_hover = _scenario_tables(vps, scen.rho, scen.soc)
    q = prm["q"]
    bp = replace(
        base_bp, T_bt=prm["T_bt"], theta_A=prm["theta_A"], delta_min=prm["spin_min"], a_lim=q,
        h_tgt=scen.altitude_amsl_m, delta_hover=d_hover,
    )  # fmt: skip
    x0, v_ground = braking.airbrake_entry_state(
        vps, scen, bp, v_air, cfg.end_speed, cfg.rotor_start, ff
    )
    t0 = braking.entry_rotor_thrust(vps, scen, bp, v_air, cfg.rotor_start, ff)
    x_tgt = v_ground**2 / (2.0 * q)
    bp = replace(bp, x_tgt=x_tgt, brake_at_limit=at_limit)
    ctl = Position1Controller(bp, ff, z_i0=t0 / ff.thrust_for_delta(bp.delta_max))
    sim = simulate(
        runs.vehicle_for(vps), ctl, scen.environment(), x0, cfg.t_end,
        runs.end_events(vps, scen, cfg.end_speed, cfg.agl0),
    )  # fmt: skip
    m = runs.run_metrics(sim, vps, float(x0[1]))
    stopped = sim.terminated_by == "ground_speed_reached"
    nan = math.nan
    x_end = float(sim["x"][-1]) if stopped else nan
    vh_end = float(sim["vh"][-1]) if stopped else nan
    flags = list(m.failures)
    if stopped and x_end - x_tgt > cfg.os_tol_m:
        flags.append("overshoot")
    if stopped and vh_end < -cfg.sink_tol_ms:
        flags.append("sinking")
    metrics = RunMetrics(m.excursion, m.energy, m.distance, tuple(flags))
    return CRun(metrics, v_air, v_ground, x_tgt, stopped, x_end, x_end - x_tgt, vh_end, sim)


def run_c(
    vp: VehicleParams, scen: Scenario, prm: dict[str, float], cfg: CEval,
    base_bp: BackTransitionParams,
) -> CRun:  # fmt: skip
    """One C run from the AIRBRAKE exit with the position cascade (`brake_at_limit = False`).

    `prm` has `T_bt`, `theta_A` (rad), `spin_min` and `q`; `base_bp` carries everything else (the
    Z loop, the lag, `theta_fw0`, the pitch-limit form). Q_TRANS_DECEL `q` is also the cascade's
    acceleration limit. The failures are the usual ones plus `"overshoot"` (the aircraft stopped
    more than `cfg.os_tol_m` past x_tgt) and `"sinking"` (end climb rate below -`cfg.sink_tol_ms`);
    a run that does not stop is already a `not_stopped` failure.
    """
    return _simulate_c(vp, scen, prm, cfg, base_bp, at_limit=False)


def evaluate_c(
    vp: VehicleParams, scen: Scenario, prm: dict[str, float], cfg: CEval,
    base_bp: BackTransitionParams,
) -> RunMetrics:  # fmt: skip
    """Metrics of `run_c`: excursion, bus energy, distance and the failure flags."""
    return run_c(vp, scen, prm, cfg, base_bp).metrics


def capability_ceiling(
    vp: VehicleParams, scen: Scenario, prm: dict[str, float], cfg: CEval,
    base_bp: BackTransitionParams,
) -> float:  # fmt: skip
    """The deceleration the pitch ramp can deliver: the same run with the pitch at the ramp limit.

    a_cap = (V_ground^2 - V_end^2) / (2 x_end), the equivalent of D's `a_eq` under the same Z loop
    and lag. NaN if the run does not stop. Reporting only: it is run once per member at the optimum.
    """
    r = _simulate_c(vp, scen, prm, cfg, base_bp, at_limit=True)
    if not r.stopped:
        return math.nan
    return (r.v_ground0**2 - cfg.end_speed**2) / (2.0 * r.x_end)


def summarise_c(results: list[CRun], weights: CostWeights) -> dict[str, float]:
    """The ensemble columns of the C tables: the cost and its parts, overshoot, flag counts."""
    ms = [r.metrics for r in results]
    ovs = [r.overshoot for r in results if r.stopped]
    n_os = sum("overshoot" in m.failures for m in ms)
    n_sink = sum("sinking" in m.failures for m in ms)
    n_all = sum(len(m.failures) for m in ms)
    return {
        "J": ensemble_cost(ms, weights),
        "p95_excursion_m": float(np.percentile([m.excursion for m in ms], 95)),
        "mean_energy_kJ": float(np.mean([m.energy for m in ms])) / 1e3,
        "p95_distance_m": float(np.percentile([m.distance for m in ms], 95)),
        "max_overshoot_m": max(ovs) if ovs else math.nan,
        "n_overshoot": float(n_os), "n_sinking": float(n_sink),
        "n_other_flags": float(n_all - n_os - n_sink),
    }  # fmt: skip


def at_bound(bounds: Bounds, params: dict[str, float], frac: float = 0.1) -> dict[str, str]:
    """Variables within `frac` of their range from a bound: "lo" or "hi" (others are absent)."""
    out: dict[str, str] = {}
    for n, lo, hi in zip(bounds.names, bounds.lo, bounds.hi, strict=True):
        pos = (params[n] - lo) / (hi - lo)
        if pos <= frac:
            out[n] = "lo"
        elif pos >= 1.0 - frac:
            out[n] = "hi"
    return out


def optimise_c(
    vp: VehicleParams,
    scens: list[Scenario],
    cfg: CEval,
    base_bp: BackTransitionParams,
    weights: CostWeights,
    start: dict[str, float],
    *,
    bounds: Bounds = C_BOUNDS,
    seeds: tuple[int, ...] = (1, 2, 3),
    popsize: int = 8,
    maxiter: int = 15,
) -> RestrictedResult:
    """CMA-ES over (T_bt, theta_A, spin_min, q), scored with (10.1) over the ensemble.

    `start` holds the register values of `T_bt`, `theta_A` and `spin_min`; `q` starts at
    `C_Q_START` unless `start` has one. `result.cost_default` is the cost at that start and
    `result.search.spread` the T-12 figure for `io.record.build_record(search_spreads=...)`.
    `result.at_bound` names the variables within 10% of their range of a bound.
    """

    def metrics(prm: dict[str, float]) -> list[RunMetrics]:
        return [evaluate_c(vp, s, prm, cfg, base_bp) for s in scens]

    def cost(u: np.ndarray) -> float:
        return ensemble_cost(metrics(bounds.to_physical(u)), weights)

    u0 = bounds.to_unit({"q": C_Q_START, **start})
    res = cma_minimise(cost, u0, seeds, popsize=popsize, maxiter=maxiter)
    best = bounds.to_physical(res.best_x)
    return RestrictedResult(
        "c", best, res.best_f, res.f_start, metrics(best), res,
        make_record(vp, scens, "c", cfg, base_bp, weights, bounds, seeds, popsize, maxiter, start),
        at_bound(bounds, best),
    )  # fmt: skip


__all__ = [
    "C_BOUNDS", "C_Q_START", "CEval", "CRun", "RestrictedResult", "at_bound",
    "capability_ceiling", "evaluate_back", "evaluate_c", "evaluate_full", "optimise",
    "optimise_c", "plan_decel", "run_c", "summarise_c",
]  # fmt: skip
