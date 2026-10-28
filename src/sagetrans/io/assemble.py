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
    parts = ["An idealised quasi-steady scan, not a flight simulation: a constant deceleration "
             "from the entry speed (equivalent airspeed) to rest, level flight, no wind, pusher "
             "off, rotors already spinning. The wing's angle of attack must stay inside a band "
             "(see `study.yaml`, `A:`), so a profile that needs negative angle of attack or one "
             "past the lift limit is infeasible. The band's upper edge rests on a provisional "
             "C_L,max. `vrs_min` / `vrs_lo` / `vrs_hi` give the rotor V_c / v_h ratio (negative = "
             "flow up through the discs); it is reported only, unless `vrs_enforce` is set in "
             "`study.yaml`.", ""]  # fmt: skip
    for name, m in q.get("maps", {}).items():
        f = m["feasible"]
        grid = [{"V0_eas": v, **{f"a={a:g}": ("feasible" if f[i, j] else "no")
                                 for j, a in enumerate(m["a_grid"])}}
                for i, v in enumerate(m["v_grid_eas"])]  # fmt: skip
        parts += [f"**{name}**", "", md_table(grid), ""]
        figs[f"A_feasibility_{name}"] = figures.feasibility_figure(
            list(m["v_grid_eas"]), list(m["a_grid"]), f)  # fmt: skip
    parts += ["Feasible constant deceleration for a whole path from the entry speed to rest "
              "(`islands` = the feasible set has gaps; the span is still reported):", "",
              md_table(tables.get("A_max_feasible_decel", [])), "",
              "Feasible window at each speed (a steady-rotor check, pointwise in speed; `bind_lo` "
              "and `bind_hi` list every limit violated just outside each edge; a row without "
              "`a_lo` means no deceleration is feasible at that speed and `bind_lo` names what "
              "fails closest to feasible):", "",
              md_table(tables.get("A_window", [])), ""]  # fmt: skip
    return "\n".join(parts)


def _section_d(q: dict[str, Any]) -> str | None:
    t = q.get("tables", {})
    if "D_a_eq" not in t:
        return None
    parts = [
        "Entry is the AIRBRAKE exit: the rotors already carry what the wing does not, the pusher "
        "is off, and the pitch limit ramps from the last fixed-wing pitch to the cap over "
        "`Q_BACKTRANS_MS`. The entry air speed comes from the exit rule "
        "`max(AIRSPEED_MIN - 2, Q_ASSIST_SPEED)`; the ground speed is that minus the headwind.", "",
        md_table(t["D_a_eq"], ["scenario", "mass_kg", "V0_air_ms", "V0_ground_ms", "a_eq_ms2",
                               "x_actual_m", "alt_gain_m", "alt_loss_m", "alpha_max_gated_deg",
                               "t_above_alpha_s_s", "vh_end_ms", "sinking"]), "",
        "**Read the angle-of-attack columns first.** With the altitude held the wing angle of "
        "attack equals the pitch, so the pitch ramp is a wing angle of attack well above the stall "
        "angle "
        "for most of the braking, and the rotors carry the weight. `a_eq` here is the most the "
        "idealised controller can do in that state, not validated hardware performance: the lift "
        "and drag beyond the stall angle are a flat-plate placeholder. Setting `D.enforce_alpha` "
        "holds the angle to the top of A's band and shows the other bound (see `D_sensitivity`). "
        "`D.ff_mode: lift_aware` is an oracle at its defaults (it reads the simulated lift). "
        "Altitude excursions are lower bounds: ideal actuators, assumed gains, no gusts. "
        "`D.theta_fw0_deg` is a placeholder until a SITL run or flight log gives the pitch at the "
        "AIRBRAKE exit.", ""]  # fmt: skip
    rec = t.get("D_recommendation")
    if "a_rec" in q and rec:
        r0 = rec[0]
        parts += [
            f"**Q_TRANS_DECEL recommendation** ({r0['n']} Sobol runs, seed {r0['seed']}, "
            f"{r0['n_corners']} corners, entry `{r0['convention']}`): "
            f"**{q['a_rec_pct']:.2f} m/s²** at the {q['pct']:g}th percentile "
            f"(bootstrap 90% interval {q['a_rec_ci'][0]:.2f} to {q['a_rec_ci'][1]:.2f}); "
            f"the minimum over sample and corners is **{q['a_rec']:.2f} m/s²** "
            f"(set by {r0['min_set_by']}). {r0['n_below_vs']} of {r0['n']} sample entries are "
            "below their own 1g stall speed (flagged, not dropped).", "",
            f"These single numbers are valid only inside the ground speeds they cover: "
            f"{r0['covers']}. The curve below gives the recommendation at other ground speeds.",
            "", md_table(rec), ""]  # fmt: skip
        if t.get("D_a_rec_curve"):
            parts += ["**`a_rec` as a function of the ground speed at the AIRBRAKE exit** (the "
                      "minimum over the named scenarios x mass factors; the air speed is the "
                      "entry rule's, so the headwind is the air speed minus the ground speed; "
                      "read `Q_TRANS_DECEL` at the slowest ground speed that must be covered):",
                      "", md_table(t["D_a_rec_curve"]), ""]  # fmt: skip
    if t.get("D_sensitivity"):
        parts += ["One change at a time, first scenario at nominal mass (excursion columns: lower "
                  "bound, ideal actuators):", "", md_table(t["D_sensitivity"]), ""]
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
                 "The range is an OUTPUT: what is left after the fixed segments and the reserve "
                 "goes to cruise (the radius for a round trip, which flies twice that). The "
                 "requirements are comparison columns only. A pusher command above "
                 "`TKOFF_THR_MAX` is reported (`over_ceiling`); a segment fails only above 1 or at "
                 "another limit (`feasible`, `limit`).", "",
                 "Budget by range (a range shorter than the largest leg's fixed distance is "
                 "not tabulated):", "",
                 md_table(tables.get("E_budget", []), ["site_amsl_m", "mission", "range_km",
                                                        "cruise_Wh", "reserve_Wh", "total_Wh",
                                                        "usable_Wh", "margin_Wh", "fits"]), "",
                 "Segments:", "",
                 md_table(tables.get("E_segments", []), ["site_amsl_m", "mission", "leg",
                                                         "segment", "energy_Wh", "soc_start",
                                                         "soc_end", "pusher_cmd_max",
                                                         "over_ceiling",
                                                         "feasible", "limit"]), "",
                 "Hover power check (stress point, landing start, after the reserve):", "",
                 md_table(tables.get("E_hover_check", [])),
                 "", "Reserve (go-around, hold, diversion):", "",
                 md_table(tables.get("E_reserve", []))]  # fmt: skip
        sections["E — energy budget and reserve"] = "\n".join(parts)
        for k, b in budgets.items():
            analyses[f"E:{k}"] = b.record
            figs[f"E_range_margin_{k}"] = figures.range_margin_figure(b, inp.requirements_km)
    if "Q2_pusher_margin" in tables:
        q2c = inp.study.Q2
        sections["Q2 — pusher forward acceleration to AIRSPEED_MIN"] = (
            "Forward acceleration the pusher can deliver at `TKOFF_THR_MAX` "
            f"({inp.tkoff_thr_max:g}, through the ESC dead zone) from rest to `AIRSPEED_MIN` "
            f"({inp.airspeed_min:g} m/s). The wing carries E's share (V/V_lw)^2 of the weight, the "
            "rotors carry the rest and load the same battery bus. `a_min_ms2` is the worst over "
            "speed, `a_equiv_ms2` the constant acceleration covering the same distance. A cell "
            "with `reachable = no` cannot reach the speed (`a_min_ms2` <= 0, `v_reach_ms` says how "
            "far it gets). `AIRSPEED_MIN` is registered as a true airspeed; both readings are "
            "tabulated (DEC-1 is open) and E reads "
            f"`{q2c.feed_convention}`. The battery is a placeholder (linear open-circuit voltage, "
            "assumed resistances, no pack current limit).\n\n"
            "Worst cell in each convention:\n\n"
            + md_table(tables.get("Q2_worst", []), ["convention", "altitude_m", "mass_factor",
                                                    "soc", "v_end_tas_ms", "a_min_ms2",
                                                    "a_equiv_ms2", "t_s", "x_m", "v_reach_ms",
                                                    "reachable"])
            + "\n\nAll cells:\n\n"
            + md_table(tables["Q2_pusher_margin"], ["convention", "altitude_m", "mass_kg", "soc",
                                                    "v_end_tas_ms", "a_min_ms2", "a_equiv_ms2",
                                                    "reachable", "current_limited"]))  # fmt: skip

    c_task = task_results.get("C", {})
    c_res = c_task.get("result")
    if c_res is not None:
        spreads["C"] = c_res.search.spread
        analyses["C"] = c_res.record
        tables.update(c_task["tables"])
        cc = inp.study.C
        runs_c = tables["C_runs"]
        a_cap = sorted(float(r["a_cap_ms2"]) for r in runs_c if r["a_cap_ms2"] != "")
        q_opt = c_res.params["q"]
        cap_txt = (
            f"The pitch ramp can deliver {min(a_cap):.2f} to {max(a_cap):.2f} m/s^2 across the "
            f"ensemble (`a_cap_ms2`, median {a_cap[len(a_cap) // 2]:.2f}); the optimum asks for "
            f"`q` = {q_opt:.2f}, so " + (
                "it is above the slowest member's ceiling and the overshoot tolerance binds."
                if a_cap and q_opt > min(a_cap) else "it is below every member's ceiling."
            )
        ) if a_cap else ""
        flagged = c_res.at_bound
        sections["C — ArduPilot parameters from the AIRBRAKE exit"] = "\n".join([
            f"Cost {c_res.cost_default:.3f} (register values, q = 2) → **{c_res.cost:.3f}**; "
            f"{len(c_res.search.seeds)}-seed spread **{c_res.search.spread:.2%}** (T-12 limit "
            f"{gate.T12_SPREAD_MAX:.0%}). Entry is the AIRBRAKE exit like D; four variables are "
            "searched, `Q_TRANS_DECEL` (q) among them, which sets the stopping point and is the "
            "cascade's acceleration limit. The vertical loop is ArduPilot's structure with its "
            f"default values; rotor lag {cc.rotor_lag_s:g} s, rotors `{cc.rotor_start}` at "
            "the exit.",
            "",
            "**The overshoot tolerance decides the answer.** A run that stops more than "
            f"{cc.os_tol_m:g} m past its stopping point, or ends sinking faster than "
            f"{cc.sink_tol_ms:g} m/s, is a failure; both tolerances are proposals. "
            + cap_txt + (
                " Variables within 10% of a bound: "
                + ", ".join(f"`{k}` ({v})" for k, v in flagged.items()) + "." if flagged else ""
            ),
            "",
            "`theta_A` against its bound is a stalled-wing brake as in D (check D's alpha and VRS "
            "columns in `C_runs`); it is not a recommendation to fly at that angle. `rotor_start` "
            "idle is read from the ArduPilot source and not checked in SITL or a log (O-13); the "
            "Z-loop defaults are master 4.8.0-dev, not a pinned tag (O-08). B and C are not "
            "comparable yet: B1 and B2 still start at level cruise.",
            "",
            md_table(tables["C_params"]), "", "Search:", "", md_table(tables["C_search"]), "",
            "Energy against distance with the optimum's ramp and q varied:", "",
            md_table(tables["C_q_curve"]), "",
            "One at a time at the optimum:", "", md_table(tables["C_sens"]), "",
            "Ensemble members at the optimum:", "",
            md_table(runs_c, ["member", "altitude_m", "mass_kg", "soc", "headwind_ms",
                              "V0_ground_ms", "x_tgt_m", "x_end_m", "overshoot_m", "energy_kJ",
                              "excursion_m", "end_sink_ms", "flags", "a_cap_ms2", "q_over_a_cap",
                              "alpha_max_gated_deg", "vrs_min"]),
        ])  # fmt: skip

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

    # B1 from the AIRBRAKE exit: one table per kind over every scenario that ran
    b1_runs = [(n, task_results[f"B1:{n}"]) for n in inp.study.B1.scenarios
               if f"B1:{n}" in task_results]  # fmt: skip
    if b1_runs:
        bc = inp.study.B1
        for key, tab in (("points", "B1_front"), ("gap", "B1_gap"), ("sens", "B1_sens")):
            tables[tab] = [r for _, res in b1_runs for r in res[key]]
        tables["B1_schedule"] = b1_runs[0][1]["schedule"]
        for n, res in b1_runs:
            errors += [f"B1 {n}: {e}" for e in res["errors"]]
        front, gap, sens_rows = tables["B1_front"], tables["B1_gap"], tables["B1_sens"]
        unver = [f"{r['scenario']} {r['variant']} eps_x={r['eps_x_m']:g}" for r in front
                 if not r.get("replay_verified")]  # fmt: skip
        b1_parts = [
            "Least bus energy from the AIRBRAKE exit to the stop, as a function of the distance "
            f"cap, with the altitude held to {', '.join(f'{e:g}' for e in bc.eps_h_m)} m, "
            "under a **stated wing limit**: " + "; ".join(
                f"`{v.name}` ("
                + (f"|alpha_w| <= {v.alpha_margin:g} alpha_s" if v.alpha_max_deg is None
                   else f"|alpha_w| <= {v.alpha_max_deg:g} deg")
                + f", pitch command <= {v.theta_cmd_max_deg:g} deg)" for v in bc.variants
            ) + ". A run with no wing limit is not part of the study. **The wing limit decides "
            "the number.** B1 is clairvoyant (it knows the scenario) and open loop, and it is not "
            "a strict lower bound (B-2). The end band on the vertical speed is "
            f"{bc.end_vh_band_ms or inp.study.C.sink_tol_ms:g} m/s (C's sink tolerance; B1 bounds "
            "climb as well as sink). The entry, the sink tolerance and the rotor start are C's and "
            "D's. B2 still starts at level cruise, so B2 and B1 are not comparable yet.", "",
            "**Front** (`J_point` is C's cost of the point; `j_best` marks the least per scenario "
            "and variant):", "",
            md_table(front, ["scenario", "variant", "eps_x_m", "converged", "replay_verified",
                             "converged_attempts", "energy_kJ", "excursion_m", "distance_m",
                             "t_f_s", "below_vs", "t_rotor_on_s", "V_over_Vs_rotor_on",
                             "alpha_w_peak_above_vs_deg", "J_point", "j_best"]), "",
        ]  # fmt: skip
        if unver:
            b1_parts += ["**No verified solution (energy blank, do not quote): "
                         + "; ".join(unver) + ".** A point that did not converge from three "
                         "starts, or whose replay in the simulator disagrees, is a solver result, "
                         "not a proof that it is infeasible.", ""]  # fmt: skip
        if gap:
            b1_parts += [
                "**Same-problem gap to C** (B1 at C's own excursion and distance; "
                "B1 has no rotor lag where C has `rotor_lag_s`):", "",
                md_table(gap, ["scenario", "variant", "c_source", "c_energy_kJ", "c_excursion_m",
                               "c_distance_m", "b1_converged", "b1_replay_verified",
                               "b1_energy_kJ", "energy_margin"]), "",
            ]  # fmt: skip
        if sens_rows:
            b1_parts += [
                "**Sensitivities** (altitude band on every scenario, the rest on the first):", "",
                md_table(sens_rows, ["scenario", "sensitivity", "value", "variant", "eps_x_m",
                                     "converged", "replay_verified", "energy_kJ", "distance_m",
                                     "t_f_s"]), "",
            ]  # fmt: skip
        sections["B1 — least energy from the AIRBRAKE exit (direct collocation)"] = "\n".join(
            b1_parts)

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
