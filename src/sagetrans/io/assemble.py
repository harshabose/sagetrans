"""Turn the task results of a study into tables, figures, a report, a record and a gate verdict."""

from __future__ import annotations

import dataclasses
import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from sagetrans.io import figures, gate, report, results
from sagetrans.io.inputs import StudyInputs
from sagetrans.io.record import FullRunRecord, build_record

Table = list[dict[str, Any]]


def md_table(rows: Table, cols: list[str] | None = None, digits: int = 3) -> str:
    """A GitHub-flavoured markdown table of dict rows."""
    if not rows:
        return "_no data_"
    cols = cols or list(rows[0].keys())

    def fmt(v: Any) -> str:
        if isinstance(v, bool):
            return "yes" if v else "no"
        if isinstance(v, float):
            if math.isnan(v):
                return "n/a"
            return f"{v:.0f}" if abs(v) >= 1000 else f"{v:.{digits}g}"
        return str(v)

    lines = ["| " + " | ".join(cols) + " |", "| " + " | ".join("---" for _ in cols) + " |"]
    lines += ["| " + " | ".join(fmt(r.get(c, "")) for c in cols) + " |" for r in rows]
    return "\n".join(lines)


def describe_inputs(inp: StudyInputs) -> str:
    """Every register entry as used: value and unit as authored, evidence status, source."""
    rows: Table = []
    for p in inp.config:
        r = inp.config.record(p)
        rows.append({
            "parameter": p, "value": r.value if r.value is not None else (r.file or ""),
            "unit": r.unit or "", "status": r.status.value, "source": r.source,
        })  # fmt: skip
    head = [
        "# Inputs used by this run", "",
        f"Mode: **{inp.mode}**. Register hash `{inp.config.config_hash()[:12]}`, "
        f"settings hash `{inp.settings_hash[:12]}`.", "",
    ]
    if inp.draft_reasons:
        head += ["Why this is not a release-grade input set:", ""]
        head += [f"- {r}" for r in inp.draft_reasons] + [""]
    if inp.fallbacks:
        head += ["Entries that fell back to the built-in placeholder (missing from the register):",
                 ""] + [f"- `{p}`" for p in inp.fallbacks] + [""]
    return "\n".join(head) + md_table(rows, ["parameter", "value", "unit", "status", "source"], 6)


def _section_a(q: dict[str, Any], figs: dict[str, Any]) -> str | None:
    tables = q.get("tables", {})
    if "A_feasibility" not in tables:
        return None
    parts = ["Zero-altitude-loss feasibility (entry speeds are equivalent airspeed). `pre_spun` "
             "assumes the rotors already spin at the schedule's first value; `whole` includes the "
             "entry step from rest.", ""]  # fmt: skip
    for name, m in q.get("maps", {}).items():
        f = m["pre_spun"]
        grid = [{"V0_eas": v, **{f"a={a:g}": ("feasible" if f[i, j] else "no")
                                 for j, a in enumerate(m["a_grid"])}}
                for i, v in enumerate(m["v_grid_eas"])]  # fmt: skip
        parts += [f"**{name}, pre-spun rotors**", "", md_table(grid), ""]
        figs[f"A_feasibility_{name}"] = figures.feasibility_figure(
            list(m["v_grid_eas"]), list(m["a_grid"]), f)  # fmt: skip
    parts += ["Largest feasible constant deceleration (pre-spun):", "",
              md_table(tables.get("A_max_feasible_decel", [])), "",
              "**Feasibility is not monotone in the deceleration.** A higher braking pitch adds "
              "drag, which can remove the need for negative rotor thrust, until the wing stalls; "
              "so the feasible set can have gaps. The map above is the authoritative picture; the "
              "table reports the top of the highest feasible region found on a "
              "coarse scan."]  # fmt: skip
    return "\n".join(parts)


def _section_d(q: dict[str, Any]) -> str | None:
    t = q.get("tables", {})
    if "D_a_eq" not in t:
        return None
    parts = [md_table(t["D_a_eq"], ["scenario", "mass_kg", "V0_eas_ms", "a_eq_ms2", "x_actual_m",
                                    "alt_gain_m", "alt_loss_m",
                                    "peak_bus_current_A"]), ""]  # fmt: skip
    if "a_rec" in q:
        pct = (f"; {q.get('pct'):g}th percentile {q['a_rec_pct']:.2f} m/s²"
               if "a_rec_pct" in q else "")
        parts += [f"**Q_TRANS_DECEL recommendation** (minimum worst-case a_eq over the ensemble "
                  f"and entry speeds): **{q['a_rec']:.2f} m/s²**{pct}.", ""]
    parts += ["Trade curve (switch earlier = smaller a_plan = more VTOL distance and energy):", "",
              md_table(t["D_trade_curve"], ["a_plan_ms2", "a_lim_ms2", "speed_test", "headwind_ms",
                                            "x_plan_m", "x_actual_m", "overshoot_m", "energy_J"]),
              "",
              "**Reading the overshoot.** The recommendation is the worst-case a_eq (the "
              "deceleration at the pitch limit), but the position cascade does not brake at the "
              "limit: it follows a speed profile built on its own deceleration limit `a_lim` "
              "(open question O-11). When `a_lim` exceeds what the aircraft can do it starts "
              "braking late and overshoots whatever `a_plan` is; with `a_lim = a_plan` the "
              "overshoot grows as `a_plan` approaches the aircraft's capability, because the "
              "cascade then has no margin left. The Q_TRANS_DECEL number alone does not remove "
              "overshoot."]  # fmt: skip
    return "\n".join(parts)


def assemble(
    inp: StudyInputs,
    task_results: Mapping[str, dict[str, Any]],
    task_errors: Mapping[str, str],
    out_root: Path,
    verification: Mapping[str, bool] | None = None,
) -> tuple[Path, gate.GateResult, FullRunRecord]:
    """Write the run folder; returns (folder, release-gate verdict, run record)."""
    cfg = inp.config
    q: dict[str, Any] = dict(task_results.get("basics", {"tables": {}}))
    tables: dict[str, Table] = dict(q.get("tables", {}))
    figs: dict[str, Any] = {}
    sections: dict[str, str] = {}
    analyses: dict[str, Any] = {}
    spreads: dict[str, float] = {}
    errors = [f"task {t} failed: {e}" for t, e in task_errors.items()] + list(q.get("errors", []))

    a_text = _section_a(q, figs)
    if a_text:
        sections["A — zero-altitude-loss feasibility"] = a_text
    d_text = _section_d(q)
    if d_text:
        sections["D — braking and Q_TRANS_DECEL"] = d_text
    if "F_descent" in tables:
        sections["F — landing descent"] = (
            f"κ source: {inp.descent.kappa_source}\n\n" + md_table(tables["F_descent"]))

    budgets = q.get("budgets", {})
    budget_main = next(iter(budgets.values()), None)
    if "E_summary" in tables:
        parts = [md_table(tables["E_summary"]), "",
                 "Ranges shorter than the climb + descent distance (`fixed_segments_distance_km`) "
                 "do not exist for that profile and are not tabulated.", "",
                 "Budget by range:", "",
                 md_table(tables.get("E_budget", []), ["case", "range_km", "total_Wh",
                                                        "reserve_Wh", "usable_Wh", "margin_Wh",
                                                        "fits"]), "",
                 "Segments:", "",
                 md_table(tables.get("E_segments", []), ["case", "segment", "energy_Wh", "time_s",
                                                         "distance_m", "source"]), "",
                 "Hover power at end of discharge:", "", md_table(tables.get("E_hover_check", [])),
                 "", "Reserve (go-around, hold, diversion):", "",
                 md_table(tables.get("E_reserve", []))]  # fmt: skip
        sections["E — energy budget and reserve"] = "\n".join(parts)
        for k, b in budgets.items():
            analyses[f"E:{k}"] = b.record
            figs[f"E_range_margin_{k}"] = figures.range_margin_figure(b, inp.requirements_km)
    if "Q2_pusher_margin" in tables:
        sections["Q2 — pusher thrust margin at AIRSPEED_MIN"] = (
            f"Full-throttle pusher thrust against wing drag at the transition speed "
            f"({inp.mission.v_trans:g} m/s, read here as an equivalent airspeed). "
            "`margin_N < 0` means the pusher cannot reach the speed.\n\n"
            + md_table(tables["Q2_pusher_margin"], ["altitude_m", "mass_kg", "V_trans_tas_ms",
                                                    "T_max_N", "D_N", "margin_N", "accel_ms2",
                                                    "current_limited"]))  # fmt: skip

    c_res = task_results.get("C", {}).get("result")
    if c_res is not None:
        spreads["C"] = c_res.search.spread
        analyses["C"] = c_res.record
        tables["C_params"] = [{"parameter": k, "value": v} for k, v in c_res.params.items()]
        tables["C_search"] = [{"seed": s.seed, "best_cost": s.f, "evaluations": s.n_evals}
                              for s in c_res.search.seeds]  # fmt: skip
        tables["C_ensemble_runs"] = [
            {"scenario": i, "excursion_m": m.excursion, "energy_J": m.energy,
             "distance_m": m.distance, "failures": ",".join(m.failures)}
            for i, m in enumerate(c_res.runs)]  # fmt: skip
        sections["C — ArduPilot parameter optimisation (back-transition)"] = "\n".join([
            f"Cost {c_res.cost_default:.3f} (register values) → **{c_res.cost:.3f}**; three-seed "
            f"spread **{c_res.search.spread:.2%}** (T-12 limit {gate.T12_SPREAD_MAX:.0%}).", "",
            md_table(tables["C_params"]), "", md_table(tables["C_search"]), "",
            md_table(tables["C_ensemble_runs"])])  # fmt: skip

    b2 = task_results.get("B2", {})
    b2_res = b2.get("result")
    if b2_res is not None:
        spreads["B2"] = b2_res.search.spread
        analyses["B2"] = b2_res.record
        tables["B2_search"] = [{"seed": s.seed, "best_cost": s.f, "evaluations": s.n_evals}
                               for s in b2_res.search.seeds]  # fmt: skip
        tables["B2_ensemble_runs"] = [
            {"scenario": i, "excursion_m": m.excursion, "energy_J": m.energy,
             "distance_m": m.distance, "failures": ",".join(m.failures)}
            for i, m in enumerate(b2_res.runs)]  # fmt: skip
        met = b2_res.search.spread <= gate.T12_SPREAD_MAX
        sections["B2 — speed-scheduled law (started from a B1 solution)"] = "\n".join([
            f"Cost {b2_res.cost_initial:.3f} (B1-based start) → **{b2_res.cost:.3f}**; three-seed "
            f"spread **{b2_res.search.spread:.2%}**. **T-12 is {'met' if met else 'NOT met'} for "
            f"B2**{'' if met else ', so this result is not reproducible'}.", "",
            md_table(tables["B2_search"]), "", md_table(tables["B2_ensemble_runs"])])  # fmt: skip

    # B1 per scenario
    b1_parts: list[str] = []
    first_b1: dict[str, Any] | None = None
    for task, res in task_results.items():
        if not task.startswith("B1:"):
            continue
        name = res["name"]
        first_b1 = first_b1 or res
        tables[f"B1_pareto_{name}"] = res["points"]
        b1_parts += [f"**{name}**, entry {res['v0']:.1f} m/s true airspeed:", "",
                     md_table(res["points"], ["eps_h_m", "converged", "replay_verified",
                                              "excursion_m", "energy_J", "distance_m", "t_f_s",
                                              "converged_attempts"]), ""]  # fmt: skip
        unver = [p["eps_h_m"] for p in res["points"] if not p.get("replay_verified")]
        if unver:
            b1_parts += ["**Not replay-verified (do not quote): eps_h = "
                         + ", ".join(f"{u:g}" for u in unver) + " m.** A converged solution whose "
                         "replay in the simulator disagrees is not trusted.", ""]
        if res["bound"]:
            bd = res["bound"]
            tag = "replay-verified" if bd.get("b1_replay_verified") else "NOT checked"
            b1_parts += [
                f"Baseline controller at the same excursion/distance: "
                f"{bd['baseline']['energy']:.0f} J, {bd['baseline']['excursion']:.1f} m, "
                f"{bd['baseline']['distance']:.0f} m. B1 under its operational limits: "
                f"{bd['b1_energy_J']:.0f} J ({bd['energy_margin']:.0%} less; {tag}).", ""]
        errors += [f"B1 {name}: {e}" for e in res["errors"]]
        good = [p for p in res["points"] if p.get("replay_verified") and "energy_J" in p]
        if len(good) >= 2:
            figs[f"B1_pareto_{name}"] = figures.pareto_figure(
                [(p["excursion_m"], p["energy_J"], p["distance_m"]) for p in good])
    if b1_parts:
        sections["B1 — open-loop optimum (direct collocation)"] = "\n".join(b1_parts)

    # sensitivity
    s_res = task_results.get("sensitivity", {}).get("result")
    nominal: dict[str, float] = dict(task_results.get("sensitivity", {}).get("nominal", {}))
    if s_res is not None:
        analyses["sensitivity"] = s_res.record
        tables["sens_oat"] = [dataclasses.asdict(r) for r in s_res.oat]
        tables["sens_global"] = [
            {"output": o, "input": n, "morris_mu_star_large": float(s_res.large.index[o][i]),
             "morris_mu_star_small": float(s_res.small.index[o][i]),
             "status": next(x.status for x in s_res.inputs if x.name == n)}
            for o in s_res.large.index for i, n in enumerate(s_res.large.names)]  # fmt: skip
        for o in s_res.stability:
            figs[f"tornado_{o}"] = figures.tornado_figure(s_res.oat, o)

    # record, report, write
    rec = build_record(
        cfg, analyses, seeds=tuple(inp.study.C.seeds), override=inp.mode == "draft",
        search_spreads=spreads, mode=inp.mode, settings_hash=inp.settings_hash,
        fallbacks=inp.fallbacks, extra_reasons=inp.draft_reasons,
    )  # fmt: skip
    limitations = [
        "Back-transition only for B1, B2 and C; the forward transition is not optimised.",
        "The forward transition in the energy budget is a quasi-steady estimate.",
        "AIRSPEED_MIN: the simulated controller compares true airspeed; Q2 reads it as an "
        "equivalent airspeed. Which one ArduPilot uses is open (O-02).",
        *[f"Input set is not release grade: {r}" for r in inp.draft_reasons],
        *[f"ERROR: {e}" for e in errors],
    ]
    a_rec = q.get("a_rec")
    text = report.build_report(report.ReportInputs(
        f"Sage VTOL transition study — {inp.study.run_name} ({rec.label})", rec, cfg, nominal,
        sens=s_res, budget=budget_main, a_rec=a_rec, required_km=inp.requirements_km,
        results=sections, limitations=limitations))  # fmt: skip
    gate_res = gate.release_gate(
        rec, dict(verification or {}), s_res, report.flagged_assumptions(s_res) if s_res else [])
    texts = {
        "report.md": text,
        "release_gate.txt": gate_res.summary() + "\n",
        "inputs_used.md": describe_inputs(inp) + "\n",
    }
    folder = results.write_results(
        out_root, inp.study.run_name, rec, cfg, tables=tables, figures=figs, texts=texts,
        override=inp.mode == "draft")  # fmt: skip
    return folder, gate_res, rec
