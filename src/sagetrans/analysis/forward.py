"""Analysis FT: the forward transition from hover (docs/FT_spec.md), a C-style ArduPilot search.

One run starts in hover `start_agl` above the site (30 m), flies `control.forward.ForwardController`
through WAIT, the hand-over (EAS above AIRSPEED_MIN), the FADE and DONE, and stays `t_settle` after
DONE to see the recovery. Everything is an equivalent airspeed (O-14). The ensemble is the Sobol
sample of D and C (altitude is the SITE elevation, each member is shifted up to the transition
altitude), searched with CMA-ES over `AIRSPEED_MIN`, `Q_TRANSITION_MS`, `TKOFF_THR_MAX` and
`Q_TRAN_PIT_MAX` and scored with the cost of (10.1).

Flags (each at most once per run, each costs the usual penalty): `no_handover`, `stall_margin`
(EAS below `stall_margin` x the stall speed from the hand-over on), `stall`, `alt_loss`, `alt_gain`,
`proj_loss`, `underspeed`, `battery_cutoff`, `touchdown`. The projected minimum altitude is the
independent check on the observed loss: it assumes the pull-up starts at the end of the fade with
the recovery acceleration `a_rec = min(wing limit at cl_frac x C_L,max, TECS_VERT_ACC)` and is a
lower bound on the loss (TECS builds pitch over seconds).
"""

from __future__ import annotations

import itertools
import math
from collections.abc import Callable
from dataclasses import dataclass, replace
from typing import Any

import numpy as np

from sagetrans import atmosphere
from sagetrans.analysis import restricted, runs
from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.analysis.ensemble import (
    CostWeights,
    EnsembleAxes,
    RunMetrics,
    braking_ensemble,
    ensemble_cost,
)
from sagetrans.analysis.restricted import RestrictedResult, at_bound
from sagetrans.analysis.search import Bounds, cma_minimise
from sagetrans.control.forward import (
    ForwardController,
    ForwardParams,
    pitch_cap_deg,
    throttle_limits,
)
from sagetrans.dynamics import events as ev
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import Result, simulate
from sagetrans.physics import aero
from sagetrans.physics.params import VehicleParams

G = 9.80665
RHO0 = float(atmosphere.density(0.0))
FT_NAMES = ("airspeed_min", "q_transition", "tkoff_thr_max", "tran_pit_max")
FT_FLAGS = (
    "no_handover", "stall_margin", "stall", "alt_loss", "alt_gain", "proj_loss", "underspeed",
    "battery_cutoff", "touchdown",
)  # fmt: skip
# The executable values: ArduPilot takes AIRSPEED_MIN in whole m/s, Q_TRAN_PIT_MAX in whole degrees,
# TKOFF_THR_MAX in whole percent and Q_TRANSITION_MS in whole milliseconds (500 to 30000).
# Steps in the units of the search variables (m/s EAS, deg, throttle fraction); Q_TRANSITION is
# fixed per grid point and given in whole milliseconds.
LATTICE_STEP = {"airspeed_min": 1.0, "tran_pit_max": 1.0, "tkoff_thr_max": 0.01}
LATTICE_NAMES = ("airspeed_min", "tkoff_thr_max", "tran_pit_max")
A_REC_FLOOR = 0.05  # m/s^2: below it the wing cannot pull out and the projection is undefined


@dataclass(frozen=True)
class FTEval:
    """What an FT evaluation reads that the search does not decide."""

    template: ForwardParams  # the register's values for everything the search leaves alone
    t_end_s: float = 90.0  # the hand-over must happen by then
    t_settle_s: float = 10.0  # simulated after DONE
    stall_margin: float = 1.2  # EAS / stall speed from the hand-over on
    cl_frac: float = 0.8  # C_L fraction of the recovery acceleration (A's band)
    proj_is_failure: bool = True
    loss_max_m: float = 10.0
    gain_max_m: float = 10.0
    alpha_hi: float = math.nan  # rad: wing angle at the top of A's band, report only
    alpha_gate_ms: float = 5.0  # below this airspeed the wing angle is not judged


@dataclass(frozen=True)
class FTRun:
    """One run: the cost metrics, the `FT_runs` columns and the trace (`sim`)."""

    metrics: RunMetrics
    row: dict[str, Any]
    sim: Result
    t_handover: float | None
    t_done: float | None


def ft_scenario(scen: Scenario, start_agl: float) -> Scenario:
    """The member shifted from the site (its altitude) up to the transition altitude."""
    return replace(
        scen, altitude_amsl_m=scen.altitude_amsl_m + start_agl, ground_amsl_m=scen.altitude_amsl_m
    )


def ft_ensemble(
    n: int, vp: VehicleParams, seed: int, axes: EnsembleAxes, start_agl: float
) -> tuple[list[Scenario], list[Scenario]]:
    """D's Sobol sample (searched) and corners (reported), shifted to the transition altitude."""
    core, corners = braking_ensemble(n, vp, seed, axes)
    return [ft_scenario(s, start_agl) for s in core], [ft_scenario(s, start_agl) for s in corners]


def recovery_accel(
    vps: VehicleParams, rho: float, v_tas: float, cl_frac: float, vert_acc: float
) -> tuple[float, float, str]:
    """The recovery acceleration at the end of the fade: (a_wing, a_rec, binding limit).

    a_wing = g (L_avail / W - 1) with L_avail the lift at `cl_frac` x C_L,max (A's 0.8 band) at the
    true airspeed `v_tas`; a_rec = min(a_wing, TECS_VERT_ACC).
    """
    l_avail = 0.5 * rho * v_tas**2 * vps.wing.S * cl_frac * runs.cl_max(vps.wing)
    a_wing = G * (l_avail / (vps.m * G) - 1.0)
    return a_wing, min(a_wing, vert_acc), "wing" if a_wing <= vert_acc else "tecs"


def projected_loss(vh_end: float, a_rec: float) -> float:
    """Extra altitude lost after the fade if the pull-up starts at once: vh^2 / (2 a_rec).

    0 for a climb or level flight; NaN when a sinking aircraft has no recovery acceleration.
    """
    if vh_end >= 0.0:
        return 0.0
    if a_rec <= A_REC_FLOOR:
        return math.nan
    return vh_end**2 / (2.0 * a_rec)


def _interp(t: np.ndarray, y: np.ndarray, t0: float) -> float:
    return float(np.interp(t0, t, y))


def run_ft(vp: VehicleParams, scen: Scenario, prm: dict[str, float], cfg: FTEval) -> FTRun:
    """Fly one forward transition. `prm` has the four searched values (see `FT_NAMES`)."""
    vps = scen.vehicle_for(vp)
    rho = scen.rho
    ff, _ = restricted._scenario_tables(vps, rho, scen.soc)  # noqa: SLF001
    h0 = scen.altitude_amsl_m
    tpl = cfg.template
    fp = replace(
        tpl, airspeed_min_eas=prm["airspeed_min"], q_transition_s=prm["q_transition"],
        tkoff_thr_max=prm["tkoff_thr_max"], tran_pit_max_deg=prm["tran_pit_max"], h_start=h0,
        back=replace(tpl.back, h_tgt=h0),
    )  # fmt: skip
    ctl = ForwardController(fp, ff, z_i0=ff.weight / ff.thrust_for_delta(fp.back.delta_max))
    x0 = np.array([0.0, h0, 0.0, 0.0, 0.0, 0.0, trim.hover_omega(vps, rho), 0.0, 0.0])

    def settled(_t: float, _x: np.ndarray, _a: dict[str, float]) -> float:
        return -1.0 if ctl.t_done is None else _t - (ctl.t_done + cfg.t_settle_s)

    events = [
        ev.battery_cutoff(vps.battery.V_min), ev.touchdown(scen.ground_amsl_m),
        ev.Event("settled", settled, +1, True),
    ]  # fmt: skip
    t_stop = cfg.t_end_s + fp.q_transition_s + cfg.t_settle_s
    sim = simulate(runs.vehicle_for(vps), ctl, scen.environment(), x0, t_stop, events)
    return _evaluate(vps, scen, fp, cfg, ctl, sim)


def _evaluate(
    vps: VehicleParams, scen: Scenario, fp: ForwardParams, cfg: FTEval, ctl: ForwardController,
    sim: Result,
) -> FTRun:
    nan = math.nan
    t = sim.t
    h, vh, x = sim["h"], sim["vh"], sim["x"]
    h_rel = h - fp.h_start
    v_eas = sim["V"] * np.sqrt(sim["rho"] / RHO0)
    weight = vps.m * G
    v_s_eas = aero.stall_speed(vps.m, RHO0, vps.wing)
    th, td = ctl.t_handover, ctl.t_done
    term = sim.terminated_by

    loss = max(0.0, -float(np.min(h_rel)))
    gain = max(0.0, float(np.max(h_rel)))
    p_bus = sim["V_bus"] * sim["I_bus"]
    i_end = int(np.searchsorted(t, td, side="right")) if td is not None else len(t)
    i_end = max(2, min(i_end, len(t)))

    def energy(i0: int, i1: int) -> float:
        i0, i1 = max(0, min(i0, len(t) - 1)), max(0, min(i1, len(t)))
        if i1 - i0 < 2:
            return 0.0
        return float(np.sum(0.5 * (p_bus[i0 + 1:i1] + p_bus[i0:i1 - 1]) * np.diff(t[i0:i1])))

    e_total = energy(0, i_end)
    i_h = int(np.searchsorted(t, th)) if th is not None else None
    e_wait = energy(0, i_h + 1) if i_h is not None else e_total
    e_fade = energy(i_h, i_end) if i_h is not None else 0.0
    dist = float(x[min(i_end, len(t)) - 1] - x[0])

    row: dict[str, Any] = {k: nan for k in (
        "t_handover_s", "x_handover_m", "V_handover_eas_ms", "margin_handover",
        "wing_share_handover", "last_throttle", "t_done_s", "h_end_rel_m", "vh_end_ms",
        "V_end_eas_ms", "theta_end_deg", "wing_share_end", "a_wing_ms2", "a_rec_ms2",
        "proj_loss_m", "h_proj_rel_m", "stall_margin_min", "V_eas_min_fade_ms", "a_mean_ms2",
        "a_equiv_ms2",
    )}  # fmt: skip
    row["V_s_eas_ms"] = v_s_eas
    row["a_rec_binding"] = ""
    flags: list[str] = []
    if th is not None:
        mask_h = t >= th
        row.update({
            "t_handover_s": th, "x_handover_m": _interp(t, x, th),
            "V_handover_eas_ms": _interp(t, v_eas, th),
            "margin_handover": _interp(t, v_eas, th) / v_s_eas,
            "wing_share_handover": _interp(t, sim["L"], th) / weight,
            "last_throttle": _interp(t, sim["delta_r"], th - 0.02),
            "stall_margin_min": float(np.min(v_eas[mask_h])) / v_s_eas,
        })
        vx_h = _interp(t, sim["vx"], th)
        row["a_mean_ms2"] = vx_h / th if th > 0.0 else nan
        x_h = row["x_handover_m"]
        row["a_equiv_ms2"] = vx_h**2 / (2.0 * x_h) if x_h > 0 else nan
    if th is not None and td is not None:
        mask_f = (t >= th) & (t <= td)
        row["V_eas_min_fade_ms"] = float(np.min(v_eas[mask_f]))
        v_end_tas = _interp(t, sim["V"], td)
        rho_end = _interp(t, sim["rho"], td)
        a_wing, a_rec, binding = recovery_accel(
            vps, rho_end, v_end_tas, cfg.cl_frac, fp.tecs_vert_acc
        )
        vh_end = _interp(t, vh, td)
        h_end = _interp(t, h_rel, td)
        proj = projected_loss(vh_end, a_rec)
        row.update({
            "t_done_s": td, "h_end_rel_m": h_end, "vh_end_ms": vh_end,
            "V_end_eas_ms": _interp(t, v_eas, td),
            "theta_end_deg": math.degrees(_interp(t, sim["theta"], td)),
            "wing_share_end": _interp(t, sim["L"], td) / weight, "a_wing_ms2": a_wing,
            "a_rec_ms2": a_rec, "a_rec_binding": binding,
            "proj_loss_m": proj, "h_proj_rel_m": h_end - proj if not math.isnan(proj) else nan,
        })
        if cfg.proj_is_failure:
            bad_proj = (a_rec <= A_REC_FLOOR and vh_end < -0.1) or (
                not math.isnan(proj) and h_end - proj < -cfg.loss_max_m
            )
            if bad_proj:
                flags.append("proj_loss")
        if float(np.min(v_eas[t >= td])) < 0.9 * fp.airspeed_min_eas:
            flags.append("underspeed")

    # flags of the run
    if term in ("battery_cutoff", "touchdown"):
        flags.append(term)
    elif th is None or th > cfg.t_end_s:
        flags.append("no_handover")
    if th is not None and row["stall_margin_min"] < cfg.stall_margin:
        flags.append("stall_margin")
    if "stall" in runs.failure_flags(sim, vps):
        flags.append("stall")
    if loss > cfg.loss_max_m:
        flags.append("alt_loss")
    if gain > cfg.gain_max_m:
        flags.append("alt_gain")
    flags = [f for f in FT_FLAGS if f in flags]  # fixed order, once each

    # report-only columns
    aw = np.abs(sim["alpha"] + vps.wing.i_w)
    gated = sim["V"] >= cfg.alpha_gate_ms
    dt = np.diff(t, append=t[-1])
    names = sim.modes
    modes = np.array(names)

    cap = np.array([
        math.radians(pitch_cap_deg(fp, m_, float(v_)))
        for m_, v_ in zip(names, sim["vx"], strict=True)
    ])
    in_transition = np.isin(modes, ("fwd_wait", "fwd_fade"))
    hi = np.array([throttle_limits(fp, m_)[1] for m_ in names])
    i_lim = max(vps.motor_r.I_lim, vps.motor_p.I_lim)
    peak_i = float(max(np.max(sim["I_rotor"]), np.max(sim["I_pusher"])))
    row.update({
        "loss_m": loss, "gain_m": gain,
        "alpha_w_peak_deg": math.degrees(float(np.max(aw[gated]))) if gated.any() else nan,
        "t_above_alpha_band_s": nan if math.isnan(cfg.alpha_hi)
        else float(np.sum(dt[gated & (aw > cfg.alpha_hi)])),
        "t_pitch_cap_s": float(
            np.sum(dt[in_transition & (np.abs(sim["theta_cmd"]) >= cap - 1e-6)])
        ),
        "t_pusher_sat_s": float(np.sum(dt[in_transition & (sim["delta_p"] >= hi - 1e-6)])),
        "energy_kJ": e_total / 1e3, "E_wait_kJ": e_wait / 1e3, "E_fade_kJ": e_fade / 1e3,
        "distance_m": dist, "peak_motor_current_A": peak_i, "current_over": peak_i > i_lim,
        "flags": ",".join(flags),
    })  # fmt: skip
    metrics = RunMetrics(max(loss, gain), e_total, dist, tuple(flags))
    return FTRun(metrics, row, sim, th, td)


def history_rows(
    vps: VehicleParams, run: FTRun, fp: ForwardParams, scenario: str, case: str, every: int = 5
) -> list[dict[str, Any]]:
    """Time series of one run, every `every`-th sample: `FT_history` rows (angles in degrees)."""
    sim = run.sim
    weight = vps.m * G
    h0 = float(sim["h"][0])
    out: list[dict[str, Any]] = []
    for i in range(0, len(sim.t), every):
        mode = sim.modes[i]
        cap = min(pitch_cap_deg(fp, mode, float(sim["vx"][i])), fp.tecs_pitch_max_deg)
        out.append({
            "scenario": scenario, "case": case, "t": float(sim.t[i]), "mode": mode,
            "x": float(sim["x"][i]), "h_rel": float(sim["h"][i]) - h0,
            "V_eas": float(sim["V"][i] * math.sqrt(sim["rho"][i] / RHO0)),
            "v_h": float(sim["vh"][i]), "theta": math.degrees(float(sim["theta"][i])),
            "theta_cap": cap, "theta_cmd": math.degrees(float(sim["theta_cmd"][i])),
            "alpha_w_deg": math.degrees(float(sim["alpha"][i] + vps.wing.i_w)),
            "delta_r": float(sim["delta_r"][i]), "delta_p": float(sim["delta_p"][i]),
            "wing_share": float(sim["L"][i]) / weight,
            "rotor_share": float(sim["T_r"][i] * math.cos(sim["theta"][i])) / weight,
        })  # fmt: skip
    return out


def evaluate_ft(
    vp: VehicleParams, scen: Scenario, prm: dict[str, float], cfg: FTEval
) -> RunMetrics:
    """Metrics of `run_ft`: excursion, bus energy to the end of the fade, distance, flags."""
    return run_ft(vp, scen, prm, cfg).metrics


def summarise_ft(results: list[FTRun], w: CostWeights) -> dict[str, float]:
    """The ensemble columns: the cost and its terms, p95 excursion, flag counts, worst values."""
    ms = [r.metrics for r in results]
    exc = np.array([m.excursion for m in ms])
    en = np.array([m.energy for m in ms])
    dist = np.array([m.distance for m in ms])
    n_flags = sum(len(m.failures) for m in ms)
    sink = [-float(r.row["vh_end_ms"]) for r in results if not math.isnan(r.row["vh_end_ms"])]
    margins = [float(r.row["stall_margin_min"]) for r in results
               if not math.isnan(r.row["stall_margin_min"])]
    out: dict[str, float] = {
        "J": ensemble_cost(ms, w),
        "J_exc": float(w.w_h * np.percentile(exc, 95) / w.h_ref),
        "J_E": float(w.w_E * en.mean() / w.E_ref),
        "J_x": float(w.w_x * np.percentile(dist, 95) / w.x_ref),
        "J_pen": float(w.penalty * n_flags),
        "p95_excursion_m": float(np.percentile(exc, 95)),
        "mean_energy_kJ": float(en.mean()) / 1e3,
        "p95_distance_m": float(np.percentile(dist, 95)),
        "worst_sink_ms": max(sink) if sink else math.nan,
        "worst_stall_margin": min(margins) if margins else math.nan,
    }
    for f in FT_FLAGS:
        out[f"n_{f}"] = float(sum(f in m.failures for m in ms))
    out["n_flags"] = float(n_flags)
    return out


def ft_bounds(
    airspeed_min: tuple[float, float], q_transition: tuple[float, float],
    tkoff_thr_max: tuple[float, float], tran_pit_max: tuple[float, float],
    fixed: dict[str, float] | None = None,
) -> Bounds:
    """`FT_BOUNDS` for the four variables, or for the ones not in `fixed` (the scan)."""
    spec = {"airspeed_min": airspeed_min, "q_transition": q_transition,
            "tkoff_thr_max": tkoff_thr_max, "tran_pit_max": tran_pit_max}  # fmt: skip
    names = tuple(n for n in FT_NAMES if n not in (fixed or {}))
    return Bounds(names, tuple(spec[n][0] for n in names), tuple(spec[n][1] for n in names))


def _lattice_range(lo: float, hi: float, step: float) -> tuple[int, int]:
    """The first and last lattice index inside [lo, hi] (a bound off the lattice is rounded in)."""
    return math.ceil(lo / step - 1e-9), math.floor(hi / step + 1e-9)


def snap_ft(prm: dict[str, float], bounds: dict[str, tuple[float, float]]) -> dict[str, float]:
    """`prm` with AIRSPEED_MIN, TKOFF_THR_MAX and Q_TRAN_PIT_MAX on their integer steps (inside the
    bounds); every other entry is copied. Q_TRANSITION is rounded to whole milliseconds."""
    out = dict(prm)
    for n in LATTICE_NAMES:
        step = LATTICE_STEP[n]
        i_lo, i_hi = _lattice_range(*bounds[n], step)
        out[n] = round(min(max(round(prm[n] / step), i_lo), i_hi) * step, 10)
    if "q_transition" in out:
        out["q_transition"] = round(round(out["q_transition"] * 1e3) / 1e3, 10)
    return out


def on_lattice(prm: dict[str, float]) -> bool:
    """Are the three lattice variables whole steps and Q_TRANSITION whole milliseconds?"""
    ok = all(abs(prm[n] / LATTICE_STEP[n] - round(prm[n] / LATTICE_STEP[n])) < 1e-6
             for n in LATTICE_NAMES)
    q = prm.get("q_transition")
    return ok and (q is None or abs(q * 1e3 - round(q * 1e3)) < 1e-6)


def lattice_neighbours(
    prm: dict[str, float], bounds: dict[str, tuple[float, float]]
) -> list[dict[str, float]]:
    """The up to 26 points one step away on the lattice (3 x 3 x 3 without the centre), those
    inside the bounds. `prm` must be on the lattice."""
    out = []
    for delta in itertools.product((-1, 0, 1), repeat=len(LATTICE_NAMES)):
        if not any(delta):
            continue
        cand = dict(prm)
        ok = True
        for n, d in zip(LATTICE_NAMES, delta, strict=True):
            step = LATTICE_STEP[n]
            i_lo, i_hi = _lattice_range(*bounds[n], step)
            i = round(prm[n] / step) + d
            if not i_lo <= i <= i_hi:
                ok = False
                break
            cand[n] = round(i * step, 10)
        if ok:
            out.append(cand)
    return out


@dataclass(frozen=True)
class LatticeResult:
    params: dict[str, float]  # the lattice point no neighbour improves on
    cost: float
    start: dict[str, float]  # the snapped start
    cost_start: float
    rounds: int  # moves made
    evaluations: int  # distinct points evaluated


def lattice_descent(
    cost: Callable[[dict[str, float]], float], start: dict[str, float],
    bounds: dict[str, tuple[float, float]], max_rounds: int = 50,
) -> LatticeResult:
    """Snap `start` to the lattice, then move to the best of its in-bound 3 x 3 x 3 neighbours
    while one is better, so the result has no better neighbour (a local minimum on the lattice).

    `cost` scores a full parameter dict; each distinct point is evaluated once.
    """
    seen: dict[tuple[float, ...], float] = {}

    def score(p: dict[str, float]) -> float:
        key = tuple(round(p[n], 10) for n in LATTICE_NAMES)
        if key not in seen:
            seen[key] = float(cost(p))
        return seen[key]

    cur = snap_ft(start, bounds)
    first = cur
    f_cur = f_first = score(cur)
    rounds = 0
    while rounds < max_rounds:
        scored = [(score(c), c) for c in lattice_neighbours(cur, bounds)]
        if not scored:
            break
        f_best, best = min(scored, key=lambda t: t[0])
        if not f_best < f_cur - 1e-12:
            break
        cur, f_cur, rounds = best, f_best, rounds + 1
    return LatticeResult(cur, f_cur, first, f_first, rounds, len(seen))


def ft_cost(
    vp: VehicleParams, scens: list[Scenario], cfg: FTEval, prm: dict[str, float],
    weights: CostWeights,
) -> float:
    """The cost (10.1) of one full parameter set on an ensemble."""
    return ensemble_cost([evaluate_ft(vp, s, prm, cfg) for s in scens], weights)


def optimise_ft(
    vp: VehicleParams,
    scens: list[Scenario],
    cfg: FTEval,
    weights: CostWeights,
    start: dict[str, float],
    bounds: Bounds,
    *,
    fixed: dict[str, float] | None = None,
    seeds: tuple[int, ...] = (1, 2, 3),
    popsize: int = 8,
    maxiter: int = 20,
) -> RestrictedResult:
    """CMA-ES over the four FT variables (or the ones not in `fixed`), scored with (10.1).

    `start` holds the register values of all four; `fixed` pins some (the scan holds AIRSPEED_MIN).
    Returns the shape of `restricted.optimise_c`: `cost_default` is the cost at the start,
    `search.spread` the T-12 figure, `at_bound` the variables within 10 % of a bound.
    """
    pin = dict(fixed or {})

    def full(prm: dict[str, float]) -> dict[str, float]:
        return {**start, **prm, **pin}

    def metrics(prm: dict[str, float]) -> list[RunMetrics]:
        return [evaluate_ft(vp, s, full(prm), cfg) for s in scens]

    def cost(u: np.ndarray) -> float:
        return ensemble_cost(metrics(bounds.to_physical(u)), weights)

    u0 = bounds.to_unit({n: start[n] for n in bounds.names})
    res = cma_minimise(cost, u0, seeds, popsize=popsize, maxiter=maxiter)
    best = full(bounds.to_physical(res.best_x))
    record: RunRecord = make_record(
        vp, scens, "ft", cfg, weights, bounds, pin, seeds, popsize, maxiter, start
    )
    return RestrictedResult(
        "ft", best, res.best_f, res.f_start, metrics(best), res, record,
        at_bound(bounds, {n: best[n] for n in bounds.names}),
    )  # fmt: skip


def mean_energy(
    vp: VehicleParams, scens: list[Scenario], cfg: FTEval, prm: dict[str, float]
) -> float:
    """Mean bus energy of an ensemble at `prm` (J): the default `E_ref` of the cost."""
    return float(np.mean([evaluate_ft(vp, s, prm, cfg).energy for s in scens]))


__all__ = [
    "FT_FLAGS", "FT_NAMES", "LATTICE_NAMES", "LATTICE_STEP", "FTEval", "FTRun", "LatticeResult",
    "evaluate_ft", "ft_bounds", "ft_cost", "ft_ensemble", "ft_scenario", "history_rows",
    "lattice_descent", "lattice_neighbours", "mean_energy", "on_lattice", "optimise_ft",
    "projected_loss", "recovery_accel", "run_ft", "snap_ft", "summarise_ft",
]  # fmt: skip
