"""Turn the task results of a study into tables, figures, a report, a record and a gate verdict."""

from __future__ import annotations

import dataclasses
import json
import math
from collections.abc import Mapping
from pathlib import Path
from typing import Any

from sagetrans.analysis import braking, runs
from sagetrans.analysis.braking import RHO0
from sagetrans.io import figures, gate, recommended, report, results
from sagetrans.io.inputs import StudyInputs
from sagetrans.io.record import FullRunRecord, build_record
from sagetrans.io.stages import FT_PARAM_COLUMNS, b1_ceiling_deg, ft_bounds, grid_winner
from sagetrans.io.view.chapters import b1_reason

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


def inputs_register(inp: StudyInputs) -> Table:
    """`inputs_register.csv`: every register entry as used (the data behind `inputs_used.md`).

    `group` is the register file (or `ardupilot_defaults` for the defaults reference). No
    parameter-to-analysis map exists in the code, so there is no `used_by` column (spec 5.4).
    """
    rows: Table = []
    for path in inp.config:
        r = inp.config.record(path)
        rows.append({
            "name": path, "group": path.split(".")[0],
            "value": r.value if r.value is not None else (r.file or ""), "unit": r.unit or "",
            "grade": r.status.value, "source": r.source,
        })  # fmt: skip
    return rows


def headline_json(nominal: Mapping[str, float], tables: Mapping[str, Table]) -> str:
    """`headline.json`: each quoted nominal number with the table cell that holds it (spec 5.2)."""
    oat = tables.get("sens_oat", [])
    out: dict[str, Any] = {}
    for key, val in nominal.items():
        row = next((i for i, r in enumerate(oat) if r.get("output") == key), None)
        out[key] = {"value": float(val), "unit": report.UNITS.get(key, ""),
                    "table": "sens_oat" if row is not None else None,
                    "column": "nominal_out" if row is not None else None, "row": row}  # fmt: skip
    return json.dumps(out, indent=2, sort_keys=True) + "\n"


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
        "`AIRSPEED_MIN - 2`; the ground speed is that minus the headwind.", "",
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


NEAR_BEST = 0.02  # a grid point within this fraction of the best snapped cost is "as good"


def grid_near_best(rows: list[dict[str, Any]], frac: float = NEAR_BEST) -> list[float]:
    """The `Q_TRANSITION_MS` values (s) whose snapped cost is within `frac` of the best one."""
    costs = [(float(r["q_transition_s"]), float(r["J_snapped"])) for r in rows]
    best = min(j for _, j in costs)
    return sorted(q for q, j in costs if j <= best * (1.0 + frac) + 1e-12)


def ft_site_energy(
    runs: list[dict[str, Any]], case: str, sites: list[float]
) -> dict[float, dict[str, float]]:
    """FT's members of one case grouped by the nearest E site: mean energy of the whole simulated
    transition (hover to the end of the fade), of its wait and fade parts, and the member count.

    FT's ensemble spans altitudes 0 to 5000 m, so only the members near a site say anything about
    E at that site; the other axes (mass, ISA, headwind, SoC) still vary inside a group.
    """
    out: dict[float, dict[str, float]] = {}
    for site in sites:
        members = [
            r for r in runs
            if r.get("case") == case and isinstance(r.get("altitude_m"), int | float)
            and min(sites, key=lambda s: abs(s - float(r["altitude_m"]))) == site
            and all(isinstance(r.get(k), int | float) and math.isfinite(float(r[k]))
                    for k in ("energy_kJ", "E_wait_kJ", "E_fade_kJ"))
        ]
        if members:
            n = len(members)
            out[site] = {
                "energy_kJ": sum(float(r["energy_kJ"]) for r in members) / n,
                "wait_kJ": sum(float(r["E_wait_kJ"]) for r in members) / n,
                "fade_kJ": sum(float(r["E_fade_kJ"]) for r in members) / n,
                "members": float(n),
            }
    return out


def e_scenarios(inp: StudyInputs, rec: Mapping[str, Any]) -> Table:
    """What each scenario of scenarios.yaml is, and the state the back transition starts from.

    One row per scenario: where it is (altitude, ISA offset, mass, SoC, headwind) and, at the
    register's AIRSPEED_MIN and at the FT optimum's, the AIRBRAKE-exit state D, C and B1 start
    from: the entry airspeed (`AIRSPEED_MIN - 2`, read as an equivalent airspeed), its true and
    ground speed, and the ratio to the 1 g stall speed. The pitch and the rotor start are the
    settings of D and C.
    """
    conv = inp.study.D.entry_convention
    rows: Table = []
    for name, sc in inp.scenarios.items():
        vp_s = sc.vehicle_for(inp.vehicle)
        rho = sc.rho
        v_stall = runs.stall_speed(vp_s, rho)
        row: dict[str, Any] = {
            "scenario": name, "note": inp.scenario_notes.get(name, ""),
            "altitude_m": sc.altitude_amsl_m, "isa_offset_k": sc.isa_offset_k,
            "mass_kg": vp_s.m, "soc": sc.soc, "headwind_ms": sc.headwind_ms,
            "stall_tas_ms": v_stall,
        }  # fmt: skip
        for lab, v in (("register", inp.airspeed_min), ("recommended", rec["airspeed_min"])):
            tas = braking.entry_airspeed(float(v), rho, conv)
            row[f"airspeed_min_{lab}_ms"] = float(v)
            row[f"entry_eas_{lab}_ms"] = tas * math.sqrt(rho / RHO0)
            row[f"entry_tas_{lab}_ms"] = tas
            row[f"entry_ground_{lab}_ms"] = tas - sc.headwind_ms
            row[f"entry_over_stall_{lab}"] = tas / v_stall
        row["entry_pitch_deg"] = inp.study.D.theta_fw0_deg
        row["entry_rotor_start"] = inp.study.C.rotor_start
        rows.append(row)
    return rows


def e_cases(inp: StudyInputs, tables: Mapping[str, Table], rec: Mapping[str, Any]) -> Table:
    """E at the register and at the recommended settings, side by side, one row per case, site and
    mission. E's forward transition (the wait to AIRSPEED_MIN plus the rotor fade) is set beside
    FT's simulation of the same case, FT's members grouped by the nearest site, so the ratio is
    like-for-like by site (information, not a gate)."""
    sites = [float(s) for s in inp.study.E.sites_amsl_m]
    runs = list(tables.get("FT_runs", []))
    rows: Table = []
    for case, suffix, ft_case, v, thr in (
        ("register", "", "register", inp.airspeed_min,
         inp.tkoff_thr_max_raw if inp.tkoff_thr_max_raw is not None else inp.tkoff_thr_max),
        ("recommended", "_recommended", "optimum", rec["airspeed_min"], rec["tkoff_thr_max"]),
    ):
        fwd: dict[tuple[Any, Any], dict[str, Any]] = {}
        for r in tables.get("E_segments" + suffix, []):
            if r.get("segment") == "fwd_transition":
                fwd.setdefault((r["site_amsl_m"], r["mission"]), r)
        ft_site = ft_site_energy(runs, ft_case, sites)
        for s in tables.get("E_summary" + suffix, []):
            f = fwd.get((s["site_amsl_m"], s["mission"]), {})
            e_kj = f["energy_Wh"] * 3.6 if isinstance(f.get("energy_Wh"), int | float) else None
            fs = ft_site.get(float(s["site_amsl_m"]), {})
            ft_e = fs.get("energy_kJ")
            ratio: float | str = ""
            if e_kj is not None and ft_e:
                ratio = e_kj / ft_e
            rows.append({
                "case": case, "site_amsl_m": s["site_amsl_m"], "mission": s["mission"],
                "airspeed_min_ms": v, "tkoff_thr_max": thr,
                "max_range_km": s.get("max_range_km", ""),
                "e_fwd_kJ": "" if e_kj is None else e_kj, "e_fwd_time_s": f.get("time_s", ""),
                "e_fwd_distance_m": f.get("distance_m", ""),
                "ft_site_energy_kJ": ft_e if ft_e is not None else "",
                "ft_site_wait_kJ": fs.get("wait_kJ", ""), "ft_site_fade_kJ": fs.get("fade_kJ", ""),
                "ft_site_members": int(fs["members"]) if fs else "",
                "e_fwd_over_ft": ratio,
            })
    return rows


def ft_scenarios_context(inp: StudyInputs) -> dict[str, Any]:
    """What the FT chapter says about its history scenarios: each one's axes and note, and the
    ensemble's ranges on the same axes (the viewer compares them; it cannot read the inputs)."""
    ax = inp.ensemble_axes
    scen = {}
    for n in inp.study.FT.history_scenarios:
        s = inp.scenarios.get(n)
        if s is not None:
            scen[n] = {"altitude_m": s.altitude_amsl_m, "isa_offset_k": s.isa_offset_k,
                       "mass_factor": (s.mass_kg or inp.vehicle.m) / inp.vehicle.m, "soc": s.soc,
                       "headwind_ms": s.headwind_ms, "note": inp.scenario_notes.get(n, "")}
    return {"scenarios": scen,
            "axes": {"altitude_m": list(ax.altitude_m), "isa_offset_k": list(ax.isa_offset_k),
                     "mass_factor": list(ax.mass_factor),
                     "headwind_ms": list(inp.study.FT.headwind_ms),  # FT's own range
                     "soc": list(ax.soc)}}  # fmt: skip


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
                 "requirements are distances flown (a round trip of 100 km is a 50 km radius) "
                 "and comparison columns only. A pusher command above "
                 "`TKOFF_THR_MAX` is reported (`over_ceiling`); a segment fails only above 1 or at "
                 "another limit (`feasible`, `limit`).", "",
                 "Budget at the maximum range (`plausible`) and at each required distance "
                 "(`range_km` is the one-way distance or the radius, `flown_km` the distance "
                 "flown in all; a case shorter than the fixed segments is not tabulated):", "",
                 md_table(tables.get("E_budget", []), ["site_amsl_m", "mission", "case",
                                                        "flown_km", "range_km",
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

    ft_key = next((t for t in task_results if t.startswith("FT:")), None)
    ft_task = task_results.get(ft_key, {}) if ft_key else {}
    win = grid_winner(dict(task_results))
    ft_res = None
    ft_grid_context: dict[str, Any] = {}
    if ft_task and win is not None:
        gw = task_results[win]
        wrow = gw["row"]
        # the optimum is the snapped winner; its search is the winner's (a three-seed search
        # at the winner's fixed Q_TRANSITION_MS), its cost_default the register start's
        q_win_search = float(gw["q_transition_s"])
        ft_res = dataclasses.replace(
            gw["result"], params=dict(gw["snapped"]), cost=float(wrow["J_snapped"]),
            cost_default=float(ft_task["J_start"]))
        spreads["FT"] = ft_res.search.spread
        analyses["FT"] = ft_res.record
        tables.update(ft_task["tables"])
        grid_rows = sorted((r["row"] for t, r in task_results.items()
                            if t.startswith("FT_grid:")), key=lambda r: r["q_transition_s"])
        coarse = set(inp.study.FT.grid.q_transition_s)
        for r in grid_rows:
            r["point"] = "coarse" if r["q_transition_s"] in coarse else "fine"
            r["winner"] = r is wrow
        near = grid_near_best(grid_rows)
        for r in grid_rows:
            r["within_2pct"] = r["q_transition_s"] in near
        tables["FT_grid"] = [{"point": r["point"], "winner": r["winner"],
                              **{k: v for k, v in r.items() if k not in ("point", "winner")}}
                             for r in grid_rows]
        # the winner's search had Q_TRANSITION_MS fixed: each seed's best point in physical units
        fb = ft_bounds(inp, {"q_transition": q_win_search})
        tables["FT_search"] = [
            {"seed": sr.seed, "best_cost": sr.f, "evaluations": sr.n_evals,
             **{FT_PARAM_COLUMNS[k]: v for k, v in {"q_transition": q_win_search,
                                                    **fb.to_physical(sr.x)}.items()}}
            for sr in ft_res.search.seeds]  # fmt: skip
        # the optimum row also carries the continuous result it was snapped from
        extra = {k: wrow[k] for k in wrow if k.endswith("_continuous")}
        extra.update(J_snapped_minus_continuous=wrow["J_snapped_minus_continuous"],
                     seed_spread=wrow["seed_spread"], at_bound=wrow["at_bound"])
        for prow in tables["FT_params"]:
            for k in ("seed_spread", "at_bound", *[k for k in extra if k.endswith("continuous")]):
                prow[k] = extra[k] if prow["case"] == "optimum" else ""
            if prow["case"] == "optimum_robust":  # its cost against the optimum's, in seed spreads
                sp, j0 = float(wrow["seed_spread"]), prow.get("J_optimum_same_ensemble")
                prow["seed_spread"] = sp
                if prow.get("robust_found") and sp > 0 and j0:
                    prow["delta_J_in_seed_spreads"] = prow["delta_J"] / (sp * j0)
        t12_failed = [(r["q_transition_s"], r["seed_spread"]) for r in grid_rows
                      if not r["t12_ok"]]
        lo, hi = inp.study.FT.bounds.q_transition_s
        q_win = float(wrow["q_transition_s"])
        ft_grid_context = {"winner_q_transition_s": q_win,
                           "winner_t12_ok": bool(wrow["t12_ok"]),
                           "winner_on_q_bound": min(abs(q_win - lo), abs(q_win - hi)) < 1e-9,
                           "within_2pct_q_transition_s": near,
                           "t12_failed": [{"q_transition_s": q, "seed_spread": s}
                                          for q, s in t12_failed]}
        ft_task = {**ft_task, "notes": [*ft_task.get("notes", []),
            f"T-12 on the grid: {len(grid_rows) - len(t12_failed)} of {len(grid_rows)} points "
            f"have a three-seed spread within {gate.T12_SPREAD_MAX:.0%}"
            + (", the others (Q_TRANSITION_MS in s, spread): " + ", ".join(
                f"{q:g} ({s:.1%})" for q, s in t12_failed) if t12_failed else "")
            + f". The winner ({q_win:g} s) " + ("passes." if wrow["t12_ok"] else "FAILS it.")
            + (" The winner is on a Q_TRANSITION_MS search bound." if ft_grid_context[
                "winner_on_q_bound"] else "")
            + f" Q_TRANSITION_MS within {NEAR_BEST:.0%} of the best cost: "
            + ", ".join(f"{q:g}" for q in near) + " s."]}
    if ft_res is not None:
        scan = sorted((res["row"] for t, res in task_results.items() if t.startswith("FT_scan:")),
                      key=lambda r: r["airspeed_min_eas_ms"])  # fmt: skip
        if scan:
            tables["FT_scan"] = scan
        ftc = inp.study.FT
        n_flag_cols = ["n_" + k for k in ("no_handover", "stall_margin", "stall", "alt_loss",
                                          "alt_gain", "proj_loss", "underspeed",
                                          "battery_cutoff", "touchdown")]  # fmt: skip
        sections["FT — forward transition (TECS-lite, uncalibrated)"] = "\n".join([
            f"Cost {ft_res.cost_default:.3f} (register values) → **{ft_res.cost:.3f}**; "
            f"{len(ft_res.search.seeds)}-seed spread **{ft_res.search.spread:.2%}** (T-12 limit "
            f"{gate.T12_SPREAD_MAX:.0%}). The aircraft starts in hover "
            f"{inp.mission.clearance_agl_m:g} m above the site and flies the forward transition "
            "(WAIT, hand-over when the equivalent airspeed exceeds `AIRSPEED_MIN`, FADE, DONE); "
            "`AIRSPEED_MIN`, `Q_TRANSITION_MS`, `TKOFF_THR_MAX` and `Q_TRAN_PIT_MAX` are "
            "searched over the first members of D's Sobol sample (C uses a Latin hypercube, so the "
            "members differ from C's). Every speed is an equivalent airspeed. "
            f"A run fails on an EAS below {ftc.stall_margin:g} x the stall speed from the "
            f"hand-over on, an altitude loss beyond {inp.mission.fwd_alt_loss_max_m:g} m or gain "
            f"beyond {inp.mission.fwd_alt_gain_max_m:g} m (assumed limits), a projected minimum "
            "altitude below the loss limit, a stall, or no hand-over.", "",
            "**Model status.** The pitch and pusher come from TECS-lite, a port of ArduPilot's "
            "TECS (`Plane-4.7.2`) with its default gains; nothing is calibrated against a log or "
            f"SITL. TECS starts `{ftc.tecs_start}` (does TECS run through the VTOL hover? open). "
            "`Q_ASSIST_SPEED` is assumed 0 (O-20): nothing restarts the fade, and the minimum "
            "EAS of the fade is reported (`V_eas_min_fade_ms`) so a flown value can be checked. "
            "The height demand is held constant and the speed and height filters of the source "
            "are replaced by the true states. The projected minimum altitude is a lower bound on "
            "the loss (the pull-up is assumed to start at once). The weights of `J` are a policy "
            "choice; its terms are printed.",
            "",
            md_table(tables["FT_params"], ["case", "airspeed_min_eas_ms", "q_transition_s",
                                           "tkoff_thr_max", "tran_pit_max_deg", "J", "J_exc",
                                           "J_E", "J_pen", "J_start", "J_snapped_minus_continuous",
                                           "seed_spread", "at_bound"]),
            "", md_table(tables["FT_params"], ["case", "p95_excursion_m", "mean_energy_kJ",
                                               "worst_sink_ms", "worst_stall_margin", "n_flags",
                                               *n_flag_cols]),
            *([""] + [f"_{n}_" for n in ft_task["notes"]] if ft_task.get("notes") else []),
            "", "Search of the winning grid point:", "", md_table(tables["FT_search"]), "",
            "The grid over `Q_TRANSITION_MS` (each point searched with `Q_TRANSITION_MS` fixed, "
            "then snapped to whole m/s, degrees, percent and milliseconds and descended on the "
            "lattice):", "",
            md_table(tables["FT_grid"], ["point", "winner", "q_transition_s", "J_continuous",
                                         "seed_spread", "t12_ok", "J_snapped",
                                         "J_snapped_minus_continuous", "airspeed_min_eas_ms",
                                         "tkoff_thr_max", "tran_pit_max_deg", "n_flags"]), "",
            *(["`AIRSPEED_MIN` held fixed, the other three re-searched:", "",
               md_table(tables["FT_scan"], ["airspeed_min_eas_ms", "q_transition_s",
                                            "tkoff_thr_max", "tran_pit_max_deg", "J", "J_exc",
                                            "J_E", "J_pen", "n_flags", "entry_eas_back_ms"]), ""]
              if scan else []),
            "One at a time at the optimum:", "", md_table(tables["FT_sens"],
                                                         ["parameter", "value", "J", "J_pen",
                                                          "p95_excursion_m", "mean_energy_kJ",
                                                          "worst_sink_ms", "worst_stall_margin",
                                                          "n_flags"]),
            *(["", "`AIRSPEED_MIN` against the C_L,max scale (reporting only, no search):", "",
               md_table(tables["FT_robust"], ["airspeed_min_eas_ms", "cl_max_scale", "J",
                                              "worst_stall_margin", "n_flags", "flags"])]
              if tables.get("FT_robust") else []),
        ])  # fmt: skip

    rec_key = next((t for t in task_results if t.startswith("basics_rec:")), None)
    if rec_key is not None:
        rec_task = task_results[rec_key]
        tables.update(rec_task["tables"])
        errors += [f"recommended settings: {e}" for e in rec_task.get("errors", [])]
        tables["E_cases"] = e_cases(inp, tables, rec_task)
        tables["E_scenarios"] = e_scenarios(inp, rec_task)
        sections["E and Q2 at the recommended settings"] = (  # the register case stays the headline
            "AIRSPEED_MIN and TKOFF_THR_MAX of the FT optimum instead of the register's; the "
            "register case above stays the headline. D is re-run at the recommended entry speed. "
            "E's forward transition is the quasi-steady wait to AIRSPEED_MIN plus the rotor fade "
            "of Q_TRANSITION_MS. `ft_site_energy_kJ` is the mean energy of FT's simulated "
            "transition (hover to the end of the fade) over the FT members nearest to the "
            "site, in the same case (`ft_site_wait_kJ` and `ft_site_fade_kJ` its parts, "
            "`ft_site_members` how many); `e_fwd_over_ft` is E divided by it. The "
            "members of a group still differ in mass, ISA, headwind, SoC and by up to "
            "about 1 km of altitude: information, not a check."
            + "\n\n" + md_table(tables["E_cases"]))

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
        # the unconstrained reference and the other points of the alpha curve: their own tasks;
        # every reader of "C" above (recommended settings, E, the viewer's C chapter) takes the
        # headline task only
        ref = task_results.get("C:unconstrained")
        if ref is not None:
            tables.update({k: v for k, v in ref["tables"].items() if k != "C_alpha_curve_ref"})
        curve_rows = list(tables.get("C_alpha_curve", []))
        curve_rows += [row for k, res in sorted(task_results.items()) if k.startswith("C:alpha:")
                       for row in res["tables"]["C_alpha_curve"]]  # fmt: skip
        if ref is not None:
            curve_rows += ref["tables"]["C_alpha_curve_ref"]
        if curve_rows:
            tables["C_alpha_curve"] = sorted(
                curve_rows, key=lambda r: (r["alpha_frac"] != "", -float(r["alpha_frac"] or 0.0)))
        held_txt = (
            f"**Configuration: enforced.** The wing angle is held to |alpha_w| <= "
            f"{tables['C_params'][0].get('alpha_limit_deg', float('nan')):.2f} deg "
            f"({inp.study.margin_alpha_frac:g} x alpha_s, `margin_alpha_frac`); this is the "
            "headline and the one written to `settings_recommended`."
            if inp.study.C.enforce_alpha else
            "**Configuration: unconstrained.** The wing angle is not held: the model ignores the "
            "stall (`C.enforce_alpha` is off)."
        )
        sections["C — ArduPilot parameters from the AIRBRAKE exit"] = "\n".join([
            held_txt, "",
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
            "Z-loop defaults are master 4.8.0-dev, not a pinned tag (O-08).",
            "",
            md_table(tables["C_params"]),
            *([""] + [f"_{n}_" for n in c_task["notes"]] if c_task.get("notes") else []),
            "", "Search:", "", md_table(tables["C_search"]), "",
            "Energy against distance with the optimum's ramp and q varied:", "",
            md_table(tables["C_q_curve"]), "",
            "One at a time at the optimum:", "", md_table(tables["C_sens"]), "",
            "Ensemble members at the optimum:", "",
            md_table(runs_c, ["member", "altitude_m", "mass_kg", "soc", "headwind_ms",
                              "V0_ground_ms", "x_tgt_m", "x_end_m", "overshoot_m", "energy_kJ",
                              "excursion_m", "end_sink_ms", "flags", "a_cap_ms2", "q_over_a_cap",
                              "alpha_max_gated_deg", "vrs_min", "stall_alpha_peak_deg",
                              "stall_t_above_peak_s", "stall_t_above_limit_s",
                              "stall_wing_share_min"]),
            *(["", "The price of the margin (one C search per wing limit; `unconstrained` ignores "
               "the stall and is a reference only, never a recommendation):", "",
               md_table(tables["C_alpha_curve"])] if tables.get("C_alpha_curve") else []),
        ])  # fmt: skip

    # B1 from the AIRBRAKE exit: one table per kind over every scenario that ran
    b1_runs = [(n, task_results[f"B1:{n}"]) for n in inp.study.B1.scenarios
               if f"B1:{n}" in task_results]  # fmt: skip
    if b1_runs:
        bc = inp.study.B1
        for key, tab in (("points", "B1_front"), ("gap", "B1_gap"), ("sens", "B1_sens")):
            tables[tab] = [r for _, res in b1_runs for r in res[key]]
        tables["B1_schedule"] = b1_runs[0][1]["schedule"]
        # the same traces under the shared history name; `case` = "B1 <variant>", C's is "C <case>"
        tables["B1_history"] = [
            {"case": f"B1 {r['variant']}", **{k: v for k, v in r.items() if k != "variant"}}
            for r in tables["B1_schedule"]]  # fmt: skip
        for n, res in b1_runs:
            errors += [f"B1 {n}: {e}" for e in res["errors"]]
        front, gap, sens_rows = tables["B1_front"], tables["B1_gap"], tables["B1_sens"]
        for fr_row in front:
            fr_row["reason"] = b1_reason(fr_row)
        unver = [f"{r['scenario']} {r['variant']} eps_x={r['eps_x_m']:g}" for r in front
                 if not r.get("replay_verified")]  # fmt: skip
        b1_parts = [
            "Least bus energy from the AIRBRAKE exit to the stop, as a function of the distance "
            f"cap, with the altitude held to {', '.join(f'{e:g}' for e in bc.eps_h_m)} m, "
            "under a **stated wing limit**: " + "; ".join(
                f"`{v.name}` ("
                + (f"|alpha_w| <= {v.alpha_margin:g} alpha_s" if v.alpha_max_deg is None
                   else f"|alpha_w| <= {v.alpha_max_deg:g} deg")
                + f", pitch command <= {b1_ceiling_deg(inp, v):g} deg)" for v in bc.variants
            ) + ". A run with no wing limit is not part of the study. **The wing limit decides "
            "the number.** B1 is clairvoyant (it knows the scenario) and open loop, and it is not "
            "a strict lower bound (B-2). The end band on the vertical speed is "
            f"{bc.end_vh_band_ms or inp.study.C.sink_tol_ms:g} m/s (C's sink tolerance; B1 bounds "
            "climb as well as sink). The entry, the sink tolerance and the rotor start are C's and "
            "D's.", "",
            "**Front** (`J_point` is C's cost of the point; `j_best` marks the least per scenario "
            "and variant):", "",
            md_table(front, ["scenario", "variant", "eps_x_m", "converged", "replay_verified",
                             "replay_late_command_ok", "converged_attempts", "energy_kJ",
                             "excursion_m", "distance_m",
                             "t_f_s", "below_vs", "t_rotor_on_s", "V_over_Vs_rotor_on",
                             "alpha_w_peak_above_vs_deg", "J_point", "j_best"]), "",
        ]  # fmt: skip
        if unver:
            b1_parts += ["**No verified solution (energy blank, do not quote): "
                         + "; ".join(unver) + ".** A point that did not converge from three "
                         "starts, or whose replay in the simulator disagrees, is a solver result, "
                         "not a proof that it is infeasible.", ""]  # fmt: skip
        fragile = [f"{r['scenario']} {r['variant']} eps_x={r['eps_x_m']:g}" for r in front
                   if r.get("replay_verified") and r.get("replay_late_command_ok") is False]
        if fragile:
            b1_parts += [
                "**Verified, but not robust to a 10 ms late command: " + "; ".join(fragile)
                + ".** Their replay agrees when the schedule is applied as the optimiser means it "
                "(the step's mean command) and fails when each 20 ms step reads it at its start "
                "(`replay_late_command_ok` = no; the excursion is in "
                "`replay_late_command_excursion_m`): the optimum sits on the lift peak, a fraction "
                "of a degree below the stall cliff. "
                "Quote them as the best the model allows with exact timing, not as a flyable "
                "schedule.", ""]  # fmt: skip
        if gap:
            b1_parts += [
                "**Same-problem gap to C** (B1 at C's own excursion and distance; "
                "B1 has no rotor lag where C has `rotor_lag_s`):", "",
                md_table(gap, ["scenario", "variant", "c_source", "c_energy_kJ", "c_excursion_m",
                               "c_distance_m", "b1_converged", "b1_replay_verified",
                               "b1_late_command_ok", "b1_energy_kJ", "energy_margin"]), "",
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

    # the settings sheet: recommended values beside the register and the ArduPilot default
    tables["settings_recommended"] = recommended.settings_rows(inp, tables)
    have_defaults = any(r["default"] != "" for r in tables["settings_recommended"])
    sections["Recommended settings"] = "\n".join([
        "One row per ArduPilot parameter the study recommends or reads. A parameter no analysis "
        "has decided keeps its register value (\"not yet analysed\"). "
        + ("The ArduPilot default is read from `ardupilot_defaults.yaml` (Plane-4.7.2) and is a "
           "reference only: it is not tuned for this airframe and is never an input."
           if have_defaults else "No `ardupilot_defaults.yaml` in the config folder: no default "
           "column."), "",
        md_table(tables["settings_recommended"],
                 ["parameter", "unit", "default_text", "register", "value", "analysis", "grade",
                  "flags", "change_vs_default", "note"]),
    ])  # fmt: skip

    # record, report, write
    rec = build_record(
        cfg, analyses, seeds=tuple(inp.study.C.seeds), override=inp.mode == "draft",
        search_spreads=spreads, mode=inp.mode, settings_hash=inp.settings_hash,
        fallbacks=inp.fallbacks, extra_reasons=inp.draft_reasons,
    )  # fmt: skip
    conv_d, conv_q = inp.study.D.entry_convention, inp.study.Q2.feed_convention
    kind = {"eas": "an equivalent", "tas": "a true"}
    limitations = [
        "Back-transition: B1 and C. Forward transition: FT, when it is among the analyses; B1 "
        "does not cover it.",
        "The forward transition in the energy budget is a quasi-steady estimate.",
        f"AIRSPEED_MIN: D, C, B1 and FT read it as {kind[conv_d]} airspeed "
        f"(D.entry_convention); Q2, which feeds E, reads it as {kind[conv_q]} airspeed "
        "(Q2.feed_convention). "
        "Which one ArduPilot compares is open (O-02).",
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
    tables["inputs_register"] = inputs_register(inp)
    texts = {
        "report.md": text,
        "release_gate.txt": gate_res.summary() + "\n",
        "inputs_used.md": describe_inputs(inp) + "\n",
    }
    if nominal:
        texts["headline.json"] = headline_json(nominal, tables)
    # what the results viewer shows that is in no table: the notes, the limitations and the
    # study.yaml block of each analysis (docs/OUTPUTS_spec.md 5.5)
    rec = dataclasses.replace(rec, context={
        "limitations": limitations,
        "mission": {"fwd_alt_loss_max_m": inp.mission.fwd_alt_loss_max_m,
                    "fwd_alt_gain_max_m": inp.mission.fwd_alt_gain_max_m},
        "notes": {("FT" if n.startswith("FT:") else n): list(t["notes"])
                  for n, t in {**task_results, **({ft_key: ft_task} if ft_key else {})}.items()
                  if t.get("notes")},
        "ft_grid": ft_grid_context,
        "ft_scenarios": ft_scenarios_context(inp),
        "study": {a: getattr(inp.study, a).model_dump(mode="json")
                  for a in ("A", "D", "F", "E", "Q2", "C", "FT", "B1", "sensitivity")
                  if a in inp.study.analyses},
    })  # fmt: skip
    folder = results.write_results(
        out_root, inp.study.run_name, rec, cfg, tables=tables, figures=figs, texts=texts,
        override=inp.mode == "draft")  # fmt: skip
    return folder, gate_res, rec
