"""Sensitivity of the headline outputs to the uncertain inputs (FR-08, section 12).

* One-at-a-time (OAT): each input at half and full of its stated half-range (by default +-10 %
  and +-20 % of nominal), for the tornado chart.
* Global: Morris (screening) or Sobol indices through SALib over all inputs together, run at two
  sample sizes so the ranking can be shown to be stable.

Every input carries the evidence grade of the parameter register. Headline outputs are those of
the back-transition under the baseline controller (altitude excursion, distance, bus energy)
and, optionally, the maximum range of the energy budget.
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping, Sequence
from dataclasses import dataclass, field, replace
from typing import Any

import numpy as np
from SALib.analyze import morris as morris_analyze
from SALib.analyze import sobol as sobol_analyze
from SALib.sample import morris as morris_sample
from SALib.sample import sobol as sobol_sample
from scipy.stats import spearmanr

from sagetrans.analysis import energy, restricted
from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.analysis.descent import DescentLimits
from sagetrans.physics.params import VehicleParams

# weakest evidence last; used to grade a result by its dominating inputs
STATUS_ORDER = ("measured", "datasheet", "derived", "documented", "stated", "assumed", "TBD")
BASE_OUTPUTS = ("excursion_m", "distance_m", "energy_J")
RANGE_OUTPUT = "max_range_km"


@dataclass(frozen=True)
class UncertainInput:
    name: str
    kind: str  # "vehicle" | "scenario" | "control"
    path: str  # dotted attribute path, or the controller / scenario field
    nominal: float
    lo: float
    hi: float
    status: str  # evidence grade from the parameter register
    note: str = ""

    @property
    def half_range(self) -> float:
        return 0.5 * (self.hi - self.lo)


def _rel(
    name: str, kind: str, path: str, nominal: float, frac: float, status: str, note: str = ""
) -> UncertainInput:
    lo, hi = nominal * (1 - frac), nominal * (1 + frac)
    return UncertainInput(name, kind, path, nominal, lo, hi, status, note)


def default_inputs(vp: VehicleParams, scen: Scenario, v: float = 0.2) -> list[UncertainInput]:
    """The uncertain inputs of the uncertainty register (section 13), with placeholder ranges.

    Relative ranges default to +-20 %; scenario axes use absolute ranges. Statuses reflect the
    parameter register: the physics values are placeholders until the bench data arrive.
    """
    w, r, mr, b, a = vp.wing, vp.rotor, vp.motor_r, vp.battery, vp.attitude
    return [
        _rel("mass", "vehicle", "m", vp.m, v, "TBD", "meaning of '5 kg' open (O-12)"),
        _rel("wing_area", "vehicle", "wing.S", w.S, v, "TBD"),
        _rel("CL_alpha", "vehicle", "wing.CL_alpha", w.CL_alpha, v, "assumed"),
        _rel("CD0", "vehicle", "wing.CD0", w.CD0, v, "assumed"),
        _rel("alpha_stall", "vehicle", "wing.alpha_s", w.alpha_s, v, "assumed", "post-stall model"),
        _rel("CN_flat_plate", "vehicle", "wing.CN", w.CN, 0.3, "assumed", "register: +-0.3"),
        _rel("rotor_CT0", "vehicle", "rotor.CT0", r.CT0, v, "assumed", "placeholder map"),
        _rel("rotor_CQ0", "vehicle", "rotor.CQ0", r.CQ0, v, "assumed", "placeholder map"),
        _rel("rotor_kmu", "vehicle", "rotor.k_mu", r.k_mu, 0.5, "assumed", "edgewise-flow factor"),
        _rel("rotor_inertia", "vehicle", "rotor.I_rot", r.I_rot, v, "TBD", "spool-up dynamics"),
        _rel("motor_Ke", "vehicle", "motor_r.Ke", mr.Ke, v, "assumed", "K_t set equal"),
        _rel("motor_Rm", "vehicle", "motor_r.Rm", mr.Rm, v, "assumed"),
        _rel("esc_dead_zone", "vehicle", "motor_r.dead_zone", mr.dead_zone, 0.5, "TBD", "bench"),
        _rel("battery_R0", "vehicle", "battery.R0_pack", b.R0_pack, 0.5, "TBD", "ECM fit pending"),
        _rel("att_omega_n", "vehicle", "attitude.omega_n", a.omega_n, 0.3, "assumed", "pitch lag"),
        _rel("att_zeta", "vehicle", "attitude.zeta", a.zeta, v, "assumed"),
        _rel("alt_hold_Kvh", "control", "K_vh", 0.02, 0.5, "assumed", "altitude-hold gain"),
        _rel("alt_hold_Ki", "control", "K_i", 0.01, 0.5, "assumed", "altitude-hold gain"),
        UncertainInput("headwind", "scenario", "headwind_ms", scen.headwind_ms,
                       scen.headwind_ms - 5.0, scen.headwind_ms + 5.0, "assumed", "wind (O-14)"),
        UncertainInput("isa_offset", "scenario", "isa_offset_k", scen.isa_offset_k,
                       scen.isa_offset_k - 10.0, scen.isa_offset_k + 10.0, "assumed"),
        UncertainInput("soc", "scenario", "soc", scen.soc, max(0.1, scen.soc - 0.3),
                       min(1.0, scen.soc + 0.2), "stated", "pack state of charge"),
    ]  # fmt: skip


def _set_path(obj: Any, parts: Sequence[str], value: float) -> Any:
    if len(parts) == 1:
        return replace(obj, **{parts[0]: value})
    return replace(obj, **{parts[0]: _set_path(getattr(obj, parts[0]), parts[1:], value)})


def apply_inputs(
    vp: VehicleParams, scen: Scenario, inputs: Sequence[UncertainInput], values: Mapping[str, float]
) -> tuple[VehicleParams, Scenario, dict[str, float]]:
    """Return the perturbed vehicle, scenario and controller overrides."""
    ctl: dict[str, float] = {}
    for inp in inputs:
        x = values.get(inp.name, inp.nominal)
        if inp.kind == "vehicle":
            vp = _set_path(vp, inp.path.split("."), x)
            if inp.path == "motor_r.Ke":
                vp = _set_path(vp, ["motor_r", "Kt"], x)
        elif inp.kind == "scenario":
            scen = replace(scen, **{inp.path: x})
        else:
            ctl[inp.path] = x
    return vp, scen, ctl


Model = Callable[[Mapping[str, float]], dict[str, float]]


@dataclass
class HeadlineModel:
    """Maps input values to the headline outputs (back-transition, optionally the budget)."""

    vp: VehicleParams
    scen: Scenario
    inputs: Sequence[UncertainInput]
    v0_air: float = 22.0
    prm: Mapping[str, float] = field(default_factory=lambda: dict(restricted.DEFAULT_PARAMS))
    budget: tuple[energy.MissionProfile, energy.BatteryWindow, DescentLimits] | None = None

    @property
    def outputs(self) -> tuple[str, ...]:
        return (*BASE_OUTPUTS, RANGE_OUTPUT) if self.budget else BASE_OUTPUTS

    @property
    def higher_is_better(self) -> frozenset[str]:
        """Outputs where a larger value is the good direction (a failed run is the minimum)."""
        return frozenset({RANGE_OUTPUT})

    def __call__(self, values: Mapping[str, float]) -> dict[str, float]:
        vp, scen, ctl = apply_inputs(self.vp, self.scen, self.inputs, values)
        try:
            m = restricted.evaluate_back(vp, scen, dict(self.prm), self.v0_air,
                                         controller_overrides=ctl or None)  # fmt: skip
            stopped = "not_stopped" not in m.failures and "touchdown" not in m.failures
            out = {
                "excursion_m": m.excursion,
                "distance_m": m.distance if stopped else math.nan,
                "energy_J": m.energy if stopped else math.nan,
            }
        except (ValueError, RuntimeError):  # e.g. no steady trim at the entry speed
            out = dict.fromkeys(BASE_OUTPUTS, math.nan)
        if self.budget is not None:
            mp, window, limits = self.budget
            mp = replace(mp, isa_offset_k=scen.isa_offset_k, soc_mean=scen.soc)
            try:
                out[RANGE_OUTPUT] = energy.build_budget(vp, mp, window, limits).max_range_m / 1e3
            except (ValueError, RuntimeError):
                out[RANGE_OUTPUT] = math.nan
        return out


# ---- one-at-a-time -------------------------------------------------------------------------------


@dataclass(frozen=True)
class OATRow:
    input: str
    output: str
    status: str
    nominal_out: float
    lo: float  # output at the low end of the range
    hi: float
    lo_half: float  # output at half the range (+-10 % for the default +-20 % range)
    hi_half: float

    @property
    def swing(self) -> float:
        return abs(self.hi - self.lo)

    @property
    def swing_rel(self) -> float:
        return self.swing / abs(self.nominal_out) if self.nominal_out else math.inf


def oat(model: HeadlineModel) -> list[OATRow]:
    """Each input alone at +-50 % and +-100 % of its half-range (nominal elsewhere)."""
    base = model({})
    rows: list[OATRow] = []
    for inp in model.inputs:
        pts = {
            "lo": inp.nominal - inp.half_range, "hi": inp.nominal + inp.half_range,
            "lo_half": inp.nominal - 0.5 * inp.half_range,
            "hi_half": inp.nominal + 0.5 * inp.half_range,
        }  # fmt: skip
        res = {k: model({inp.name: x}) for k, x in pts.items()}
        for out in model.outputs:
            rows.append(
                OATRow(inp.name, out, inp.status, base[out], res["lo"][out], res["hi"][out],
                       res["lo_half"][out], res["hi_half"][out])
            )  # fmt: skip
    return rows


def tornado(rows: Sequence[OATRow], output: str) -> list[OATRow]:
    """The rows of one output, largest swing first."""
    return sorted((r for r in rows if r.output == output), key=lambda r: r.swing, reverse=True)


# ---- global --------------------------------------------------------------------------------------


@dataclass
class GlobalResult:
    method: str  # "morris" or "sobol"
    size: int  # trajectories (Morris) or base sample N (Sobol)
    names: list[str]
    index: dict[str, np.ndarray]  # output -> importance per input (mu_star or total-order ST)
    detail: dict[str, dict[str, np.ndarray]]  # output -> extra arrays (sigma, S1, ...)
    n_runs: int
    n_failed: int


def _problem(inputs: Sequence[UncertainInput]) -> dict[str, Any]:
    return {
        "num_vars": len(inputs),
        "names": [i.name for i in inputs],
        "bounds": [[i.lo, i.hi] for i in inputs],
    }


def _evaluate(model: HeadlineModel, x: np.ndarray) -> tuple[dict[str, np.ndarray], int]:
    names = [i.name for i in model.inputs]
    ys: dict[str, list[float]] = {o: [] for o in model.outputs}
    for row in x:
        out = model(dict(zip(names, row.tolist(), strict=True)))
        for o in model.outputs:
            ys[o].append(out[o])
    arr = {o: np.array(v) for o, v in ys.items()}
    n_failed = int(np.sum(np.isnan(np.column_stack(list(arr.values()))).any(axis=1)))
    better_high: frozenset[str] = getattr(model, "higher_is_better", frozenset())
    for o, a in arr.items():  # a failed run takes the WORST finite value seen for that output
        finite = a[np.isfinite(a)]
        worst = (finite.min() if o in better_high else finite.max()) if finite.size else 0.0
        arr[o] = np.where(np.isfinite(a), a, worst)
    return arr, n_failed


def global_morris(model: HeadlineModel, r: int, seed: int = 0, levels: int = 4) -> GlobalResult:
    """Morris elementary effects with `r` trajectories: index = mu_star."""
    prob = _problem(model.inputs)
    x = morris_sample.sample(prob, N=r, num_levels=levels, seed=seed)
    ys, n_failed = _evaluate(model, x)
    index, detail = {}, {}
    for o, y in ys.items():
        res = morris_analyze.analyze(prob, x, y, num_levels=levels, seed=seed)
        index[o] = np.asarray(res["mu_star"])
        detail[o] = {"mu": np.asarray(res["mu"]), "sigma": np.asarray(res["sigma"])}
    return GlobalResult("morris", r, prob["names"], index, detail, len(x), n_failed)


def global_sobol(model: HeadlineModel, n: int, seed: int = 0) -> GlobalResult:
    """Sobol first- and total-order indices from a Saltelli sample of base size `n` (power of 2)."""
    prob = _problem(model.inputs)
    x = sobol_sample.sample(prob, n, calc_second_order=False, seed=seed)
    ys, n_failed = _evaluate(model, x)
    index, detail = {}, {}
    for o, y in ys.items():
        res = sobol_analyze.analyze(prob, y, calc_second_order=False, seed=seed)
        index[o] = np.asarray(res["ST"])
        detail[o] = {"S1": np.asarray(res["S1"])}
    return GlobalResult("sobol", n, prob["names"], index, detail, len(x), n_failed)


def rank_stability(a: GlobalResult, b: GlobalResult, k: int = 3) -> dict[str, dict[str, float]]:
    """Spearman correlation and top-k overlap of the rankings from two sample sizes."""
    out: dict[str, dict[str, float]] = {}
    for o in a.index:
        ia, ib = a.index[o], b.index[o]
        rho = float(spearmanr(ia, ib).statistic) if np.ptp(ia) > 0 and np.ptp(ib) > 0 else math.nan
        top_a = set(np.argsort(ia)[::-1][:k].tolist())
        top_b = set(np.argsort(ib)[::-1][:k].tolist())
        out[o] = {"spearman": rho, "topk_overlap": len(top_a & top_b) / k}
    return out


def dominating(res: GlobalResult, output: str, k: int = 3, min_share: float = 0.1) -> list[str]:
    """Inputs ranked in the top k, or carrying at least `min_share` of the summed index."""
    idx = np.clip(res.index[output], 0.0, None)
    total = idx.sum()
    order = np.argsort(idx)[::-1]
    keep = [
        int(i) for j, i in enumerate(order) if j < k or (total > 0 and idx[i] / total >= min_share)
    ]
    return [res.names[i] for i in keep]


def weakest_status(statuses: Sequence[str]) -> str:
    """The weakest evidence grade among the given statuses."""
    return max(statuses, key=STATUS_ORDER.index) if statuses else "assumed"


@dataclass
class SensitivityResult:
    inputs: list[UncertainInput]
    oat: list[OATRow]
    small: GlobalResult
    large: GlobalResult
    stability: dict[str, dict[str, float]]
    record: RunRecord = field(default_factory=lambda: make_record())

    def dominating(self, output: str, k: int = 3) -> list[UncertainInput]:
        names = dominating(self.large, output, k)
        by = {i.name: i for i in self.inputs}
        return [by[n] for n in names]


def run_sensitivity(
    model: HeadlineModel, r_small: int = 6, r_large: int = 12, seed: int = 0, k: int = 3
) -> SensitivityResult:
    """OAT plus Morris at two sample sizes (the global ranking must be stable between them)."""
    rows = oat(model)
    small, large = global_morris(model, r_small, seed), global_morris(model, r_large, seed)
    return SensitivityResult(
        list(model.inputs), rows, small, large, rank_stability(small, large, k),
        make_record(model.vp, model.scen, [i.name for i in model.inputs], r_small, r_large, seed),
    )  # fmt: skip
