"""The chapter builders: one function per analysis, tables in, `Out` (result box, figures, tables) out.

A builder reads tables through `Ctx`, so a missing table or column makes that figure absent and
never raises (docs/OUTPUTS_spec.md 2, rule 3). Every number in a result box is a cell of a table
(`data-src="Table.column[row]"`) or an arithmetic expression of cells shown beside it.
"""

from __future__ import annotations

import math
from collections import defaultdict
from collections.abc import Callable, Sequence
from typing import Any

from sagetrans.io.view import charts as ch
from sagetrans.io.view.html import Item, Out, absent, panelbox, table_html
from sagetrans.io.view.load import Results
from sagetrans.io.view.units import fmt_value, header_label, unit_of

Row = dict[str, Any]
NAN = float("nan")


def num(v: Any) -> float:
    return float(v) if isinstance(v, int | float) and not isinstance(v, bool) else NAN


def ok(v: float) -> bool:
    return math.isfinite(v)


class Ctx:
    """The results folder as the builders see it."""

    def __init__(self, res: Results) -> None:
        self.res = res

    def rows(self, name: str) -> list[Row]:
        return self.res.rows(name)

    def has(self, name: str, *cols: str) -> bool:
        t = self.res.get(name)
        return bool(t and t.rows and t.has(*cols))

    @property
    def context(self) -> dict[str, Any]:
        c = self.res.record.get("context", {})
        return c if isinstance(c, dict) else {}

    def study(self, analysis: str, key: str, default: Any = None) -> Any:
        return self.context.get("study", {}).get(analysis, {}).get(key, default)

    def item(self, label: str, table: str, column: str, row: int, unit: str | None = None,
             note: str = "") -> Item | None:  # fmt: skip
        rows = self.rows(table)
        if not (0 <= row < len(rows)) or column not in rows[row] or rows[row][column] is None:
            return None
        u = unit_of(column)[0] if unit is None else unit
        return Item(label, fmt_value(rows[row][column], column), u, f"{table}.{column}[{row}]",
                    note=note)  # fmt: skip

    def extreme(self, table: str, column: str, pick: Callable[..., Any] = max,
                where: Callable[[Row], bool] | None = None) -> int | None:  # fmt: skip
        """Row index of the largest (or smallest) finite value of a column."""
        best: tuple[float, int] | None = None
        for i, r in enumerate(self.rows(table)):
            v = num(r.get(column))
            if ok(v) and (where is None or where(r)) and (best is None or pick(v, best[0]) == v):
                best = (v, i)
        return None if best is None else best[1]


def tip(row: Row, cols: Sequence[str], head: str = "") -> str:
    """The tooltip text of a mark: the named cells of its row, with units."""
    parts = [
        f"{c} = {fmt_value(row[c], c)}{(' ' + unit_of(c)[0]) if unit_of(c)[0] else ''}"
        for c in cols
        if c in row and row[c] is not None
    ]
    return (head + ("; " if head and parts else "") + "; ".join(parts)).strip()


def uniq(values: Sequence[Any]) -> list[Any]:
    seen: list[Any] = []
    for v in values:
        if v not in seen:
            seen.append(v)
    return seen


def need(o: Out, c: Ctx, *tables: str) -> bool:
    """True when every table exists; otherwise the missing ones are listed on the chapter."""
    missing = [t for t in tables if not c.rows(t)]
    o.absent += [f"not in this run: {t}" for t in missing]
    return not missing


def _legend(*entries: tuple[str, str, str]) -> str:
    return ch.legend(entries)


# ------------------------------------------------------------------------------------------- A
BIND_KIND = {"alpha_low": "dot", "thrust_sign": "sq", "alpha_high": "dia"}


def build_a(c: Ctx) -> Out:
    o = Out()
    mx, feas, win = c.rows("A_max_feasible_decel"), c.rows("A_feasibility"), c.rows("A_window")
    for s in uniq([r.get("scenario") for r in mx]):
        def in_scenario(r: Row, s: Any = s) -> bool:
            return bool(r.get("scenario") == s)

        i = c.extreme("A_max_feasible_decel", "max_feasible_decel_ms2", max, in_scenario)
        it = (
            c.item(
                f"Max feasible deceleration, {s}",
                "A_max_feasible_decel",
                "max_feasible_decel_ms2",
                i,
            )
            if i is not None
            else None
        )
        if it:
            o.items.append(it)
    if win:
        i = c.extreme(
            "A_window", "V0_eas_ms", min, lambda r: num(r.get("a_lo")) <= num(r.get("a_hi"))
        )
        it = (
            c.item("Lowest entry speed with a window", "A_window", "V0_eas_ms", i)
            if i is not None
            else None
        )
        if it:
            o.items.append(it)
    if feas and need(o, c, "A_feasibility"):
        panels = []
        for s in uniq([r["scenario"] for r in feas]):
            rs = [r for r in feas if r["scenario"] == s]
            cells = {}
            for r in rs:
                bad = r.get("feasible") is False
                cells[(r["V0_eas_ms"], r["decel_ms2"])] = (
                    "no" if bad else "ok", "x" if bad else "",
                    tip(r, ["altitude_m", "V0_eas_ms", "decel_ms2", "violated", "vrs_min"],
                        f"{s}, {'not feasible' if bad else 'feasible'}"))  # fmt: skip
            xs = sorted({k[0] for k in cells})
            ys = sorted({k[1] for k in cells})
            panels.append(
                (
                    s,
                    ch.cell_map(
                        xs,
                        ys,
                        cells,
                        "entry speed [m/s, EAS]",
                        "deceleration [m/s^2]",
                        f"Feasible map {s}",
                    ),
                )
            )
        o.figures.append(
            panelbox(
                "Feasible map",
                "Entry speed against deceleration. A feasible cell is shaded; a cell "
                "that is not carries x, and its tooltip names the constraint that fails.",
                ch.multiples(panels),
                _legend(("cellok", "sq", "feasible"), ("cellno", "sq", "not feasible (x)")),
            )
        )
    if win:
        panels, panels3 = [], []
        for s in uniq([r["scenario"] for r in win]):
            rs = [r for r in win if r["scenario"] == s]
            series = []
            for r in rs:
                lo, hi = str(r.get("bind_lo") or ""), str(r.get("bind_hi") or "")
                series.append(ch.Series(
                    f"{r['V0_eas_ms']:g}", "s1", [(r["V0_eas_ms"], r["a_lo"]), (r["V0_eas_ms"], r["a_hi"])],
                    [tip(r, ["V0_eas_ms", "a_lo"], f"lower edge closed by {lo}"),
                     tip(r, ["V0_eas_ms", "a_hi"], f"upper edge closed by {hi}")],
                    kinds=[BIND_KIND.get(lo.split(";")[0], "dot"), BIND_KIND.get(hi.split(";")[0], "dot")],
                ))  # fmt: skip
            panels.append(
                (
                    s,
                    ch.xy_chart(
                        series,
                        "entry speed [m/s, EAS]",
                        "deceleration [m/s^2]",
                        f"Window {s}",
                        w=340,
                        h=260,
                    ),
                )
            )
            lo_s = ch.Series(
                "at the low end",
                "s1",
                [(r["V0_eas_ms"], r["theta_lo_deg"]) for r in rs],
                [tip(r, ["V0_eas_ms", "theta_lo_deg"], "low end") for r in rs],
            )
            hi_s = ch.Series(
                "at the high end",
                "s2",
                [(r["V0_eas_ms"], r["theta_hi_deg"]) for r in rs],
                [tip(r, ["V0_eas_ms", "theta_hi_deg"], "high end") for r in rs],
            )
            panels3.append(
                (
                    s,
                    ch.xy_chart(
                        [lo_s, hi_s],
                        "entry speed [m/s, EAS]",
                        "pitch [deg]",
                        f"Pitch along the window {s}",
                        w=340,
                        h=260,
                    ),
                )
            )
        o.figures.append(
            panelbox(
                "Window at each entry speed",
                "The zero-loss deceleration window: a vertical segment per "
                "entry speed. The marker shape names the limit that closes each edge.",
                ch.multiples(panels),
                _legend(
                    ("s1", "dot", "closed by alpha_low"),
                    ("s1", "sq", "closed by thrust_sign"),
                    ("s1", "dia", "closed by alpha_high"),
                ),
            )
        )
        o.figures.append(
            panelbox(
                "Pitch used along the window",
                "Pitch at the low and the high edge of the window, against entry speed.",
                ch.multiples(panels3),
                _legend(
                    ("s1", "line", "at the low end of the window"),
                    ("s2", "line", "at the high end of the window"),
                ),
            )
        )
    else:
        o.absent.append("not in this run: A_window")
    o.tables = [("A_max_feasible_decel", None), ("A_window", None), ("A_feasibility", None)]
    return o


# ------------------------------------------------------------------------------------------ Q2
def build_q2(c: Ctx) -> Out:
    o = Out()
    rows = c.rows("Q2_pusher_margin")
    if not rows:
        return o
    i = c.extreme("Q2_pusher_margin", "a_min_ms2", min)
    if i is not None:
        it = c.item(
            "Worst pusher acceleration",
            "Q2_pusher_margin",
            "a_min_ms2",
            i,
            note=tip(rows[i], ["convention", "altitude_m", "mass_factor", "soc"]),
        )
        if it:
            o.items.append(it)
    reach = [r.get("reachable") for r in rows if "reachable" in r]
    if reach:
        n_ok = sum(1 for v in reach if v is True)
        o.items.append(
            Item(
                "Every case reaches the end speed",
                "yes" if n_ok == len(reach) else "NO",
                expr=f"{n_ok} of {len(reach)} rows have reachable = yes",
            )
        )
        if n_ok != len(reach):
            o.flags.append(f"{len(reach) - n_ok} case(s) do not reach the end speed")
    e_rows = c.rows("E_summary")
    e_i = c.extreme("E_summary", "a_fwd_out_ms2", min)
    if e_i is not None:
        it = c.item("Acceleration E assumes (lowest, out leg)", "E_summary", "a_fwd_out_ms2", e_i)
        if it:
            o.items.append(it)
    conv = [v for v in uniq([r.get("convention") for r in rows]) if v]
    first = conv[0] if conv else None
    sub = [r for r in rows if r.get("convention") == first] if first else rows
    series = []
    masses = sorted({r["mass_factor"] for r in sub if r.get("mass_factor") is not None})
    for k, m in enumerate(masses[:3]):
        rs = [r for r in sub if r.get("mass_factor") == m]
        series.append(ch.Series(f"mass x {m:g}", f"s{k + 1}", [(r["altitude_m"], r["a_min_ms2"]) for r in rs],
                                [tip(r, ["altitude_m", "soc", "a_min_ms2", "v_at_min_ms", "mass_kg"], f"mass x {m:g}")
                                 for r in rs], hollow=[r.get("reachable") is False for r in rs],
                                line=False))  # fmt: skip
    if e_rows and any("a_fwd_out_ms2" in r for r in e_rows):
        pts = [
            (r["site_amsl_m"], r["a_fwd_out_ms2"])
            for r in e_rows
            if ok(num(r.get("a_fwd_out_ms2")))
        ]
        series.append(ch.Series("E assumes (out leg)", "ink", pts,
                                [tip(r, ["site_amsl_m", "mission", "a_fwd_out_ms2"], "E_summary")
                                 for r in e_rows if ok(num(r.get("a_fwd_out_ms2")))],
                                line=False, marker="dia"))  # fmt: skip
    if series:
        o.figures.append(
            panelbox(
                "Pusher acceleration by altitude",
                f"Lowest acceleration the pusher gives up to the end speed ({first or 'all'} convention), "
                "one mark per mass factor and state of charge; E's assumed value as a diamond.",
                ch.xy_chart(
                    series, "altitude [m]", "a_min [m/s^2]", "Pusher acceleration", y_from_zero=True
                ),
                _legend(*[(s.cls, "dot" if s.cls != "ink" else "dia", s.name) for s in series]),
            )
        )
    alts = sorted({r["altitude_m"] for r in sub})
    socs = sorted({r["soc"] for r in sub})
    if alts and socs:
        vals, flg = [], []
        for a in alts:
            row_v, row_f = [], []
            for s in socs:
                cell = [r for r in sub if r["altitude_m"] == a and r["soc"] == s]
                row_v.append(min((r["a_min_ms2"] for r in cell), default=None))
                row_f.append(any(r.get("reachable") is False for r in cell))
            vals.append(row_v)
            flg.append(row_f)
        o.figures.append(
            panelbox(
                "Worst acceleration by altitude and state of charge",
                "Lowest a_min over the mass factors; the value is printed in each cell.",
                ch.heat_chart(
                    [f"SoC {s:g}" for s in socs],
                    [f"{a:g} m" for a in alts],
                    vals,
                    "Worst acceleration",
                    "state of charge",
                    "m/s^2",
                    2,
                    flg,
                ),
            )
        )
    if len(conv) >= 2:
        series = []
        for k, cv in enumerate(conv[:2]):
            by = {
                a: min(
                    r["a_min_ms2"]
                    for r in rows
                    if r.get("convention") == cv and r["altitude_m"] == a
                )
                for a in alts
            }
            pts = sorted(by.items())
            series.append(ch.Series(f"AIRSPEED_MIN read as {cv}", f"s{k + 1}", pts,
                                    [f"{cv}: altitude {a:g} m, a_min {v:.2f} m/s^2" for a, v in pts],
                                    labels={len(pts) - 1: cv}))  # fmt: skip
        o.figures.append(
            panelbox(
                "True against equivalent airspeed reading",
                "Lowest a_min at each altitude when the controller's AIRSPEED_MIN is read as true (tas) or as "
                "equivalent (eas) airspeed. The gap is the effect of the convention.",
                ch.xy_chart(
                    series,
                    "altitude [m]",
                    "worst a_min [m/s^2]",
                    "Convention effect",
                    y_from_zero=True,
                ),
                _legend(*[(s.cls, "line", s.name) for s in series]),
            )
        )
    o.tables = [("Q2_worst", None), ("Q2_pusher_margin", None)]
    return o


# ------------------------------------------------------------------------------------------- D
def build_d(c: Ctx) -> Out:
    o = Out()
    rec, curve, trade = c.rows("D_recommendation"), c.rows("D_a_rec_curve"), c.rows("D_trade_curve")
    if rec:
        for lab, col in (
            ("Recommended Q_TRANS_DECEL (percentile)", "a_rec_pct_ms2"),
            ("Ensemble minimum", "a_rec_min_ms2"),
        ):
            it = c.item(lab, "D_recommendation", col, 0)
            if it:
                o.items.append(it)
        a, b = num(rec[0].get("a_rec_pct_ms2")), num(rec[0].get("a_rec_min_ms2"))
        if ok(a) and ok(b):
            o.items.append(Item("Margin to the ensemble minimum", f"{a - b:.2f}", "m/s^2",
                                "D_recommendation.a_rec_pct_ms2[0]; D_recommendation.a_rec_min_ms2[0]",
                                f"{a:.3f} - {b:.3f}"))  # fmt: skip
        it = c.item("Members below the stall speed", "D_recommendation", "n_below_vs", 0)
        if it:
            o.items.append(it)
    if curve:
        pts = [
            (r["ground_speed_ms"], r["a_rec_min_ms2"])
            for r in curve
            if ok(num(r.get("a_rec_min_ms2")))
        ]
        series = [
            ch.Series("worst case (a_rec_min)", "s1", pts, [tip(r, ["ground_speed_ms", "a_rec_min_ms2", "set_by"])
                                                           for r in curve if ok(num(r.get("a_rec_min_ms2")))],
                      thick=True),
            ch.Series("median", "s2", [(r["ground_speed_ms"], r["a_eq_median_ms2"]) for r in curve
                                       if ok(num(r.get("a_eq_median_ms2")))], marker="none"),
            ch.Series("largest", "s3", [(r["ground_speed_ms"], r["a_eq_max_ms2"]) for r in curve
                                        if ok(num(r.get("a_eq_max_ms2")))], marker="none"),
        ]  # fmt: skip
        rules = []
        if rec and ok(num(rec[0].get("a_rec_pct_ms2"))):
            rules.append((rec[0]["a_rec_pct_ms2"], "recommended (percentile)"))
        o.figures.append(
            panelbox(
                "Achievable deceleration against ground speed",
                "The braking deceleration the ensemble can hold at each ground speed: the worst case "
                "(thick), the median and the largest.",
                ch.xy_chart(
                    series,
                    "ground speed [m/s]",
                    "deceleration [m/s^2]",
                    "Achievable deceleration",
                    hlines=rules,
                ),
                _legend(
                    ("s1", "line", "worst case"),
                    ("s2", "line", "median"),
                    ("s3", "line", "largest"),
                ),
            )
        )
    else:
        o.absent.append("not in this run: D_a_rec_curve")
    o.absent.append(
        "not in this run: D_history (the study does not write the time history of D's "
        "representative run yet)"
    )
    if trade:
        kinds = uniq([r.get("speed_test") for r in trade])[:3]
        series = []
        for k, kd in enumerate(kinds):
            rs = [r for r in trade if r.get("speed_test") == kd and ok(num(r.get("x_actual_m")))]
            rs.sort(key=lambda r: r["x_actual_m"])
            series.append(ch.Series(str(kd), f"s{k + 1}", [(r["x_actual_m"], r["energy_J"] / 1e3) for r in rs],
                                    [tip(r, ["a_plan_ms2", "headwind_ms", "x_actual_m", "energy_J",
                                             "overshoot_m"], str(kd)) for r in rs]))  # fmt: skip
        o.figures.append(
            panelbox(
                "Distance against energy",
                "Each mark is a planned deceleration; the planned value is "
                "in the tooltip. Energy is the bus energy of the brake.",
                ch.xy_chart(
                    series, "stopping distance [m]", "energy [kJ]", "Distance against energy"
                ),
                _legend(*[(s.cls, "line", f"speed test: {s.name}") for s in series]),
            )
        )
    o.tables = [
        ("D_recommendation", None),
        ("D_a_rec_curve", None),
        ("D_trade_curve", None),
        ("D_recommendation_ensemble", None),
        ("D_a_eq", None),
    ]
    return o


# ------------------------------------------------------------------------------------------- F
def build_f(c: Ctx) -> Out:
    o = Out()
    desc, prof = c.rows("F_descent"), c.rows("F_profile")
    if desc:
        i = c.extreme("F_descent", "v_cap_ms", min)
        if i is not None:
            it = c.item(
                "Descent rate used (lowest cap)",
                "F_descent",
                "v_cap_ms",
                i,
                note=f"binding: {desc[i].get('binding_limit')}",
            )
            if it:
                o.items.append(it)
        it = c.item("Source of kappa", "F_descent", "kappa_source", 0)
        if it:
            o.items.append(it)
    if prof:
        panels_r, panels_e = [], []
        for s in uniq([r["scenario"] for r in prof]):
            rs = [r for r in prof if r["scenario"] == s]
            panels_r.append(
                (
                    s,
                    ch.xy_chart(
                        [
                            ch.Series(
                                "descent rate",
                                "s1",
                                [(r["time_to_go_s"], r["descent_rate_ms"]) for r in rs],
                                [tip(r, ["h_m", "time_to_go_s", "descent_rate_ms"]) for r in rs],
                            )
                        ],
                        "time to go [s]",
                        "descent rate [m/s]",
                        f"Descent rate {s}",
                        w=340,
                        h=240,
                    ),
                )
            )
            panels_e.append(
                (
                    s,
                    ch.xy_chart(
                        [
                            ch.Series(
                                "energy to go",
                                "s2",
                                [(r["time_to_go_s"], r["energy_to_go_J"] / 1e3) for r in rs],
                                [tip(r, ["h_m", "time_to_go_s", "energy_to_go_J"]) for r in rs],
                            )
                        ],
                        "time to go [s]",
                        "energy to go [kJ]",
                        f"Energy to go {s}",
                        w=340,
                        h=240,
                    ),
                )
            )
        o.figures.append(
            panelbox(
                "Descent rate against time to go",
                "The commanded descent rate along the profile, one panel per scenario.",
                ch.multiples(panels_r, 3),
            )
        )
        o.figures.append(
            panelbox(
                "Energy to go against time to go",
                "Hover energy still to be spent, one panel per scenario.",
                ch.multiples(panels_e, 3),
            )
        )
    o.absent.append(
        "not in this run: the margin to the vortex-ring onset against time (not written)"
    )
    o.tables = [("F_descent", None), ("F_profile", None)]
    return o


# ------------------------------------------------------------------------------------------- E
def build_e(c: Ctx) -> Out:
    o = Out()
    summ, bud, seg = c.rows("E_summary"), c.rows("E_budget"), c.rows("E_segments")
    if summ:
        i = c.extreme("E_summary", "max_range_km", min)
        if i is not None:
            it = c.item(
                "Shortest maximum range",
                "E_summary",
                "max_range_km",
                i,
                note=f"site {summ[i].get('site_amsl_m'):g} m, {summ[i].get('mission')}",
            )
            if it:
                o.items.append(it)
        if any("flyable" in r for r in summ):
            n = sum(1 for r in summ if r.get("flyable") is True)
            o.items.append(
                Item(
                    "Flyable site and mission cases",
                    f"{n} of {len(summ)}",
                    expr="count of flyable = yes in E_summary",
                )
            )
        res = c.rows("E_reserve")
        for k, r in enumerate(res):
            if r.get("component") == "total":
                it = c.item("Reserve", "E_reserve", "energy_Wh", k)
                if it:
                    o.items.append(it)
    hv = c.rows("E_hover_check")
    if hv:
        n_ok = sum(1 for r in hv if r.get("ok") is True)
        o.items.append(
            Item(
                "Hover check",
                "pass" if n_ok == len(hv) else "FAIL",
                expr=f"{n_ok} of {len(hv)} points ok in E_hover_check",
            )
        )
        if n_ok != len(hv):
            o.flags.append("hover check fails")
    # whole-mission waterfall: one per site and mission, at the longest range in E_budget
    segs = ["takeoff", "fwd_transition", "climb", "descent", "back_transition", "landing"]
    if bud:
        panels = []
        for key in uniq([(r["site_amsl_m"], r["mission"]) for r in bud]):
            rs = [r for r in bud if (r["site_amsl_m"], r["mission"]) == key]
            r = max(rs, key=lambda x: num(x.get("range_km")) if ok(num(x.get("range_km"))) else -1)
            items = []
            for sg in segs:
                v = sum(
                    num(r.get(f"{d}_{sg}_Wh"))
                    for d in ("out", "ret")
                    if ok(num(r.get(f"{d}_{sg}_Wh")))
                )
                items.append((sg.replace("_", " "), v, f"{sg}: {v:.1f} Wh", "s1"))
            for col, lab in (("cruise_Wh", "cruise"), ("reserve_Wh", "reserve")):
                if ok(num(r.get(col))):
                    items.append(
                        (lab, r[col], f"{lab}: {r[col]:.1f} Wh", "s2" if lab == "reserve" else "s1")
                    )
            panels.append(
                (
                    f"site {key[0]:g} m, {key[1]}, {r['range_km']:g} km",
                    ch.bar_chart(
                        items, "energy [Wh]", f"Energy waterfall {key}", w=420, waterfall=True
                    ),
                )
            )
        o.figures.append(
            panelbox(
                "Whole-mission energy by segment",
                "Wh per segment (out and return added) at the longest range of each "
                "site and mission in E_budget; the reserve is the last bar.",
                ch.multiples(panels, 2),
                _legend(("s1", "line", "mission segments"), ("s2", "line", "reserve")),
            )
        )
    if summ:
        req = sorted({int(k.split("_at_")[1][:-2]) for r in summ for k in r if k.startswith("margin_Wh_at_")
                      and k.endswith("km")})  # fmt: skip
        items = [(f"site {r['site_amsl_m']:g} m, {r['mission']}", r["max_range_km"],
                  tip(r, ["max_range_km", "usable_Wh"] + [f"margin_Wh_at_{q}km" for q in req]),
                  "s1") for r in summ if ok(num(r.get("max_range_km")))]  # fmt: skip
        if items:
            o.figures.append(
                panelbox(
                    "Maximum range against the requirement",
                    "Maximum range with the reserves included, per site and mission; the rules are the range "
                    "requirements named by the margin columns of E_summary.",
                    ch.xy_chart(
                        [
                            ch.Series(
                                "max range",
                                "s1",
                                [(i, v) for i, (_, v, _, _) in enumerate(items)],
                                [t for _, _, t, _ in items],
                                line=False,
                            )
                        ],
                        "case (site, mission; see tooltip)",
                        "max range [km]",
                        "Maximum range",
                        y_from_zero=True,
                        vlines=[],
                        hlines=[(float(q), f"{q} km required") for q in req],
                    ),
                    _legend(("s1", "dot", "maximum range of a site and mission")),
                )
            )
    if seg:
        panels = []
        for key in uniq([(r["site_amsl_m"], r["mission"]) for r in seg]):
            rs = [r for r in seg if (r["site_amsl_m"], r["mission"]) == key]
            t, pts = 0.0, []
            for r in rs:
                if ok(num(r.get("soc_start"))):
                    pts.append((t, r["soc_start"]))
                t += num(r.get("time_s")) if ok(num(r.get("time_s"))) else 0.0
                if ok(num(r.get("soc_end"))):
                    pts.append((t, r["soc_end"]))
            if pts:
                sm = [r for r in summ if (r["site_amsl_m"], r["mission"]) == key]
                rules = (
                    [(sm[0]["soc_end_after_reserve"], "end after reserve")]
                    if sm and ok(num(sm[0].get("soc_end_after_reserve")))
                    else []
                )
                panels.append(
                    (
                        f"site {key[0]:g} m, {key[1]}",
                        ch.xy_chart(
                            [ch.Series("state of charge", "s1", pts, marker="none")],
                            "elapsed time [s]",
                            "state of charge [-]",
                            f"State of charge {key}",
                            w=420,
                            h=260,
                            hlines=rules,
                        ),
                    )
                )
        if panels:
            o.figures.append(
                panelbox(
                    "State of charge along the mission",
                    "From E_segments (segment durations added); the rule is the state "
                    "of charge the reserve leaves.",
                    ch.multiples(panels, 2),
                )
            )
    sw = c.rows("E_cruise_sweep")
    good = [r for r in sw if ok(num(r.get("Wh_per_km")))]
    if good:
        series, best = [], []
        keys = uniq([(r["scenario"], r["altitude_m"]) for r in good])[:3]
        for k, key in enumerate(keys):
            rs = [r for r in good if (r["scenario"], r["altitude_m"]) == key]
            j = min(range(len(rs)), key=lambda x: rs[x]["Wh_per_km"])
            series.append(ch.Series(f"{key[0]}, {key[1]:g} m", f"s{k + 1}", [(r["V_tas_ms"], r["Wh_per_km"])
                                                                              for r in rs],
                                    [tip(r, ["V_tas_ms", "power_W", "Wh_per_km"]) for r in rs],
                                    labels={j: "optimum"}, marker="none"))  # fmt: skip
            best.append(j)
        o.figures.append(
            panelbox(
                "Cruise energy against speed",
                "Energy per kilometre at each cruise speed; the label marks the least.",
                ch.xy_chart(series, "true airspeed [m/s]", "energy [Wh/km]", "Cruise sweep"),
                _legend(*[(s.cls, "line", s.name) for s in series]),
            )
        )
    o.tables = [
        ("E_summary", None),
        ("E_budget", None),
        ("E_segments", None),
        ("E_cruise_sweep", None),
        ("E_hover_check", None),
        ("E_reserve", None),
    ]
    cases = c.rows("E_cases")
    if cases:
        cols = ["case", "site_amsl_m", "mission", "airspeed_min_ms", "tkoff_thr_max", "max_range_km",
                "e_fwd_kJ", "e_fwd_time_s", "e_fwd_distance_m", "ft_site_energy_kJ",
                "ft_site_wait_kJ", "ft_site_fade_kJ", "ft_site_members", "e_fwd_over_ft"]
        o.figures.append(panelbox(
            "Register against recommended settings",
            "E repeated at the AIRSPEED_MIN and TKOFF_THR_MAX of the FT optimum (D re-run at that entry "
            "speed). The register case is the headline above. The last two columns are information "
            "only. e_fwd_kJ is E's forward transition: the quasi-steady wait to AIRSPEED_MIN plus "
            "the rotor fade of Q_TRANSITION_MS (the rotors fade from the throttle that carries "
            "the share of the weight the wing does not, E.fade_wing_share). ft_site_energy_kJ is "
            "the mean energy of FT's simulated transition (hover to the end of the fade) over the "
            "FT members nearest to that site in the same case, with its wait and fade parts and "
            "the member count; e_fwd_over_ft is E divided by it. The members of a group still "
            "differ in mass, ISA, headwind and SoC and in altitude by up to about 1 km, so the "
            "ratio is a like-for-like size check by site, not a validation.",
            table_html("E_cases", [k for k in cols if any(k in r for r in cases)], cases),
        ))
        o.tables += [("E_summary_recommended", None), ("E_segments_recommended", None),
                     ("E_budget_recommended", None), ("E_hover_check_recommended", None),
                     ("Q2_pusher_margin_recommended", None), ("Q2_worst_recommended", None)]
    return o


# ------------------------------------------------------------------------------- C and the default
def relative(key: str, values: list[float]) -> list[float]:
    """Altitude as the change from the first sample, so runs from different sites compare."""
    return [v - values[0] for v in values] if key == "h" and values else values


def hist_series(rows: Sequence[Row], plan: dict[str, tuple[str, str, bool, bool]],
                keys: Sequence[str]) -> list[ch.TSeries]:  # fmt: skip
    """TSeries per case of a history table; `plan[case] = (class, short name, dashed, thick)`."""
    out = []
    for case, (cls, short, dash, thick) in plan.items():
        rs = [r for r in rows if r.get("case") == case and ok(num(r.get("t")))]
        if rs:
            out.append(
                ch.TSeries(
                    case,
                    cls,
                    short,
                    [r["t"] for r in rs],
                    {k: relative(k, [num(r.get(k)) for r in rs]) for k in keys},
                    dash,
                    False,
                    thick,
                    short if cls == "ref" else "",
                )
            )
    return out


HIST_PANELS = [
    ch.Panel("V", "airspeed [m/s]", "m/s"),
    ch.Panel("h", "altitude change [m]", "m", 2, True),
    ch.Panel("theta", "pitch [deg]", "deg", 1),
    ch.Panel("delta_r", "rotor throttle [-]", "", 2),
]


def _c_cases(c: Ctx) -> list[str]:
    return uniq([r.get("case") for r in c.rows("C_params")])


def history_panels(band: float | None) -> list[ch.Panel]:
    """The four panels of C's history figures: the airspeed axis starts at 0 and the altitude panel
    draws the B1 altitude band as two thin lines."""
    lines = [(band, f"+{band:g} m"), (-band, f"-{band:g} m")] if band else []
    return [
        ch.Panel("V", "airspeed [m/s]", "m/s", ymin=0.0),
        ch.Panel("h", "altitude change [m]", "m", 2, True, hlines=lines),
        ch.Panel("theta", "pitch [deg]", "deg", 1),
        ch.Panel("delta_r", "rotor throttle [-]", "", 2),
    ]


def select_history(
    c_rows: Sequence[Row], b1_rows: Sequence[Row], view: str, headline: str | None = None
) -> tuple[dict[str, list[Row]], tuple[str, Any]]:
    """The rows of one history figure, by series, all from ONE scenario or ONE ensemble member.

    `view` is "core" (C's runs on the B1 headline scenario, with B1's replays of the same scenario)
    or "default_fails" (both C cases on the ensemble member where the default overshoots most; no B1).
    Returns `({case: rows}, ("scenario", name) | ("member", index))`. A figure never mixes
    scenarios: C rows of another scenario or member, and B1 rows of another scenario, are dropped.
    """
    cr = [r for r in c_rows if r.get("view") == view]
    if view == "core":
        name = headline or (cr[0].get("scenario") if cr else None)
        cr = [r for r in cr if r.get("scenario") == name]
        br = [r for r in b1_rows if name is not None and r.get("scenario") == name]
        ident: tuple[str, Any] = ("scenario", name)
    else:
        mem = cr[0].get("member") if cr else None
        cr = [r for r in cr if r.get("member") == mem]
        br = []
        ident = ("member", mem)
    out: dict[str, list[Row]] = {}
    for r in [*cr, *br]:
        out.setdefault(str(r.get("case")), []).append(r)
    return out, ident


def scenario_caption(r: Row) -> str:
    """Altitude, ISA offset, mass, SoC, headwind and the entry airspeed (true and equivalent)."""
    bits = [
        ("altitude", "altitude_m", "{:g} m"), ("ISA", "isa_offset_k", "{:+.1f} K"),
        ("mass", "mass_kg", "{:.1f} kg"), ("SoC", "soc", "{:.2f}"),
        ("headwind", "headwind_ms", "{:+.2f} m/s"),
    ]
    parts = [f"{lab} {fmt.format(num(r.get(col)))}" for lab, col, fmt in bits
             if ok(num(r.get(col)))]  # fmt: skip
    if ok(num(r.get("V0_tas_ms"))) and ok(num(r.get("V0_eas_ms"))):
        parts.append(f"entry airspeed {num(r['V0_tas_ms']):.1f} m/s true, "
                     f"{num(r['V0_eas_ms']):.1f} m/s equivalent")  # fmt: skip
    return ", ".join(parts)


C_HIST_STYLE = {  # case -> (class, dashed, thick)
    "C optimum": ("s1", False, True),
    "C ardupilot_default": ("ref", True, False),
    "B1 attached": ("s2", False, False),
    "B1 c_envelope": ("s3", False, False),
}


def history_figure(
    title: str, caption: str, fig_id: str, series: dict[str, list[Row]], band: float | None,
    entry: dict[str, float],
) -> str:  # fmt: skip
    """One history figure from `select_history` output; legend entries carry the entry speed."""
    ts = []
    for case, rows in series.items():
        rs = [r for r in rows if ok(num(r.get("t")))]
        if not rs:
            continue
        cls, dash, thick = C_HIST_STYLE.get(case, ("ink", False, False))
        v0 = entry.get(case)
        short = f"{case.replace('ardupilot_default', 'ArduPilot default')}" + (
            f" ({v0:.1f} m/s)" if v0 is not None and ok(v0) else "")
        ts.append(ch.TSeries(
            case, cls, short, [r["t"] for r in rs],
            {k: relative(k, [num(r.get(k)) for r in rs]) for k in ("V", "h", "theta", "delta_r")},
            dash, False, thick, "", True))  # fmt: skip
    leg = [(s.cls, "dash" if s.dash else "line", s.short) for s in ts]
    return panelbox(title, caption, ch.panel_figure(fig_id, history_panels(band), ts), _legend(*leg))


def build_c_histories(c: Ctx, o: Out) -> None:
    """C's two time-history figures: the B1 scenario, and where the default fails."""
    hist = c.rows("C_history")
    if not hist:
        o.absent.append("not in this run: C_history")
        return
    if not any("view" in r for r in hist):  # a results folder from before the two figures
        plan = {
            "C optimum": ("s1", "C optimum", False, True),
            "C ardupilot_default": ("ref", "ArduPilot default", True, False),
        }
        ts = hist_series(hist, plan, ["V", "h", "theta", "delta_r"])
        o.figures.append(panelbox(
            "Time histories",
            "Member 0 of the ensemble under C's optimum (and the ArduPilot default); "
            "an older results folder: the scenario is not recorded.",
            ch.panel_figure("c_hist", HIST_PANELS, ts),
            _legend(*[(s.cls, "dash" if s.dash else "line", s.short) for s in ts])))
        return
    b1h = [{**r, "case": r.get("case") or f"B1 {r.get('variant')}"} for r in c.rows("B1_history")]
    b1h = b1h or [{**r, "case": f"B1 {r.get('variant')}"} for r in c.rows("B1_schedule")]
    headline = b1h[0].get("scenario") if b1h else None
    front = c.rows("B1_front")
    band_vals = [num(r.get("band_eps_h_m")) for r in hist if ok(num(r.get("band_eps_h_m")))]
    band = band_vals[0] if band_vals else None

    series, (_, name) = select_history(hist, b1h, "core", headline)
    if series and name is not None:
        first = next(rows[0] for case, rows in series.items() if case.startswith("C"))
        entry = {case: num(first.get("V0_tas_ms")) for case in series}
        for case in series:  # B1's own entry speed, when its table has it
            if case.startswith("B1 "):
                fr = next((r for r in front if r.get("scenario") == name
                           and f"B1 {r.get('variant')}" == case and ok(num(r.get("V0_air_ms")))), None)
                if fr is not None:
                    entry[case] = num(fr["V0_air_ms"])
        note = ""
        if any(k.startswith("B1 ") for k in series):
            env_rows = [r for r in front if r.get("scenario") == name
                        and r.get("variant") == "c_envelope" and r.get("replay_verified") is True]
            late = [r.get("replay_late_command_ok") for r in env_rows]
            tail = ""
            if "B1 c_envelope" in series and late:
                tail = ("; c_envelope is verified for exact timing only" if False in late
                        else "; c_envelope also replays with commands 10 ms late")
            note = (" The B1 curves are open-loop bounds that use the altitude band, not "
                    "controllers" + tail + ".")
        o.figures.append(history_figure(
            f"Same scenario as B1 ({name})",
            f"Scenario {name}: {scenario_caption(first)}. C's optimum and the ArduPilot default "
            "run on the scenario that B1's headline replay uses; each curve ends with a marker "
            f"where the run stopped.{note}",
            "c_hist_core", series, band, entry))
    series, (_, mem) = select_history(hist, [], "default_fails")
    if series and mem is not None:
        first = next(iter(series.values()))[0]
        entry = {case: num(first.get("V0_tas_ms")) for case in series}
        dfl = next((rows[0] for case, rows in series.items() if "default" in case), None)
        opt_row = next((rows[0] for case, rows in series.items() if case == "C optimum"), None)
        fails = bool(first.get("default_fails"))
        d_os = num(dfl.get("run_overshoot_m")) if dfl else NAN
        o_os = num(opt_row.get("run_overshoot_m")) if opt_row else NAN
        said = (f"The default overshoots this member by {d_os:.2f} m and the optimum by {o_os:.2f} m."
                if ok(d_os) and ok(o_os) else "")
        if not fails:
            said += (" No member fails under the default; this is the one with the largest "
                     "overshoot anyway.")
        o.figures.append(history_figure(
            f"Where the default fails (ensemble member {mem:g})",
            f"Ensemble member {mem:g}: {scenario_caption(first)}. {said}",
            "c_hist_default", series, band, entry))


T12_LIMIT = 0.01  # the repeatability limit of the seeds (io.gate.T12_SPREAD_MAX)
SEED_PARAMS = (("T_bt_s", "T_bt"), ("theta_A_deg", "theta_A"), ("spin_min", "spin_min"),
               ("Q_TRANS_DECEL_ms2", "q"))  # fmt: skip


def seed_statement(srch: Sequence[Row]) -> str:
    """One sentence on the seeds, from the data: do they agree on J, and on the parameters?

    Seeds that agree on J but differ in parameters mean a flat ridge (many settings, one cost);
    agreeing on both means a single well-defined optimum; disagreeing on J means the search has
    not converged (the T-12 limit is 1 %).
    """
    js = [num(r.get("best_cost")) for r in srch if ok(num(r.get("best_cost")))]
    if len(js) < 2:
        return ""
    spread = (max(js) - min(js)) / max(abs(min(js)), 1e-12)
    if spread > T12_LIMIT:
        return (f"The seeds disagree on J (spread {spread:.2%}, the limit is {T12_LIMIT:.0%}): "
                "the search has not converged.")
    differ = []
    for col, name in SEED_PARAMS:
        v = [num(r.get(col)) for r in srch if ok(num(r.get(col)))]
        if len(v) == len(srch) and v:
            mean = sum(v) / len(v)
            if max(v) - min(v) > 0.05 * max(abs(mean), 1e-12):
                differ.append(f"{name} {min(v):.3g} to {max(v):.3g}")
    if differ:
        return (f"The seeds agree on J (spread {spread:.2%}) but differ in the parameters "
                f"({'; '.join(differ)}): a flat ridge, not a single optimum.")
    if any(ok(num(r.get(c))) for r in srch for c, _ in SEED_PARAMS):
        return (f"The seeds agree on J (spread {spread:.2%}) and on the parameters: a single "
                "well-defined optimum.")
    return ""


def bound_reason(row: Row, bound: str) -> str:
    """Where the upper bound of theta_A comes from, computed: min(30, PTCH_LIM_MAX_DEG)."""
    cfg, ptch, hi = (num(row.get(k)) for k in ("theta_A_cfg_hi_deg", "ptch_lim_deg",
                                              "theta_A_hi_deg"))
    if "theta_A:hi" not in bound or not (ok(cfg) and ok(ptch) and ok(hi)):
        return ""
    which = "the pitch limit" if ptch < cfg else "the configured bound"
    return (f" (the upper bound of theta_A is {which}: min({cfg:g}, PTCH_LIM_MAX_DEG {ptch:g}) "
            f"= {hi:g} deg)")


def sensitivity_figure(sens: Sequence[Row], js0: float) -> str:
    """C's one-at-a-time sensitivity as two dots per row: how many members fail (by type), and the
    change of J without the failure penalty. The percent change of the full J is dominated by the
    penalty (rotor lag 0.5 gives +7500 %), so it hides the smooth part. A row where members fail
    is marked with an x."""
    fails, smooth = [], []
    for r in sens:
        js = num(r.get("J_smooth"))
        if not ok(js):
            continue
        n_os, n_sink = num(r.get("n_overshoot")), num(r.get("n_sinking"))
        n_other = num(r.get("n_other_flags"))
        n_mem = num(r.get("n_failing_members"))
        n_mem = n_mem if ok(n_mem) else 0.0
        label = f"{'x ' if n_mem else ''}{r['parameter']} = {r['value']}"
        parts = ", ".join(f"{k} {v:g}" for k, v in (("overshoot", n_os), ("sinking", n_sink),
                                                   ("other", n_other)) if ok(v) and v)
        fails.append((label, n_mem, f"{n_mem:g} failing member(s)" + (f": {parts}" if parts else ""),
                      "s1" if n_mem else "ref"))
        smooth.append((label, (js / js0 - 1) * 100,
                       f"J without the penalty = {js:.3f}; change = ({js:.3f} / {js0:.3f} - 1) x 100",
                       "s1"))
    body = ch.multiples(
        [("Members that fail (by type in the tooltip)",
          ch.dot_chart(fails, "failing members of the ensemble", "C sensitivity: failing members")),
         ("Change of J without the failure penalty",
          ch.dot_chart(smooth, "change of J without the penalty [%]", "C sensitivity: smooth J"))],
        1)
    return panelbox(
        "One-at-a-time sensitivity",
        "One setting changes at a time. The change of the full J is dominated by the failure "
        "penalty, so the figure shows the failures and the smooth part of J apart; x marks a row "
        "where members fail (the tooltip has the expressions).",
        body, _legend(("s1", "dot", "a member fails"), ("ref", "dot", "no member fails")))


def spread_statement(mine: Sequence[Row], opt_row: Row) -> str:
    """What the overshoot strip shows, computed: the range, and whether any member stops short.

    The cause is read from the code (restricted._simulate_c): the stop target is
    x_tgt = V_ground^2 / (2 q), the distance of a constant deceleration q from t = 0, while the
    pitch ramp over T_bt delays the braking, so a member stops past the target.
    """
    os_v = [num(r.get("overshoot_m")) for r in mine if ok(num(r.get("overshoot_m")))]
    if not os_v:
        return ""
    short = [v for v in os_v if v <= 0.0]
    lo, hi = min(os_v), max(os_v)
    if short:
        head = (f"{len(short)} of {len(os_v)} members stop short of the target; the overshoot "
                f"ranges from {lo:.2f} to {hi:.2f} m.")
    else:
        head = (f"No member of the optimum stops short of the target: the overshoot is positive "
                f"in all {len(os_v)} and ranges from {lo:.2f} to {hi:.2f} m.")
    sink = [num(r.get("end_sink_ms")) for r in mine if ok(num(r.get("end_sink_ms")))]
    if sink:
        head += f" The end sink is at most {max(sink):.2f} m/s."
    cause = (" The stop target assumes the planned deceleration q from t = 0, while the pitch "
             "ramp over T_bt delays the braking, so the aircraft stops past it.")
    return head + cause


def margin_figure(curve: Sequence[Row], params: Sequence[Row], opt: int) -> str | None:
    """Worst overshoot and worst end sink against q: the coarse curve, the fine grid, the optimum,
    the margin row and each tolerance as a line (two small charts, the sink one as important as
    the overshoot one). None if the curve has neither column."""
    panels = []
    mrow = next((r for r in params if r.get("case") == "optimum_margin"), None)
    for col, tcol, lab, ylab in (
        ("max_overshoot_m", "os_tol_m", "worst overshoot", "worst overshoot [m]"),
        ("max_end_sink_ms", "sink_tol_ms", "worst end sink", "worst end sink [m/s]"),
    ):
        series: list[ch.Series] = []
        for grid, name, cls in (("coarse", "coarse curve", "s1"), ("fine", "fine grid", "s1")):
            rs = sorted((r for r in curve if (r.get("grid") or "coarse") == grid
                         and ok(num(r.get("q"))) and ok(num(r.get(col)))),
                        key=lambda r: num(r["q"]))  # fmt: skip
            if rs:
                series.append(ch.Series(
                    name, cls, [(num(r["q"]), num(r[col])) for r in rs],
                    [tip(r, ["q", col, "J", "n_overshoot", "n_sinking"], name) for r in rs],
                    hollow=[bool(num(r.get("n_overshoot")) or num(r.get("n_sinking"))) for r in rs],
                    line=grid == "coarse", marker="dot" if grid == "coarse" else "sq"))
        for r, name, cls, kind in ((params[opt], "optimum", "s1", "dia"),
                                   (mrow, "margin row", "s3", "dia")):
            if r is not None and ok(num(r.get("Q_TRANS_DECEL_ms2"))) and ok(num(r.get(col))):
                series.append(ch.Series(
                    name, cls, [(num(r["Q_TRANS_DECEL_ms2"]), num(r[col]))],
                    [tip(r, ["Q_TRANS_DECEL_ms2", col], name)], line=False, marker=kind))
        tol = num(params[opt].get(tcol))
        if series and ok(tol):
            panels.append((lab, ch.xy_chart(
                series, "Q_TRANS_DECEL q [m/s^2]", ylab, f"{lab} against q", w=340, h=280,
                hlines=[(tol, f"tolerance {tol:g} (PROPOSED)")])))
    if not panels:
        return None
    os_m, sk_m = num(params[opt].get("os_margin_m")), num(params[opt].get("sink_margin_ms"))
    margins = (f" At the optimum the margins are {os_m:.2f} m and {sk_m:.2f} m/s."
               if ok(os_m) and ok(sk_m) else "")
    sentence = ("The optimum relies on the overshoot and end-sink tolerances, which are PROPOSED, "
                "not on a measured limit." + margins)
    leg = _legend(("s1", "line", "coarse curve (hollow: a member fails)"), ("s1", "sq", "fine grid"),
                  ("s1", "dia", "optimum"), ("s3", "dia", "margin row (reported, not recommended)"))
    return panelbox(
        "Worst overshoot and worst end sink against q",
        "The optimum's other three values fixed; q varied. A hollow mark has a failing member.",
        ch.multiples(panels, 2) + f'<p class="cap">{ch.esc(sentence)}</p>', leg)


def build_c(c: Ctx) -> Out:
    o = Out()
    params, runs, curve = c.rows("C_params"), c.rows("C_runs"), c.rows("C_q_curve")
    if not params:
        return o
    opt = next((i for i, r in enumerate(params) if r.get("case") == "optimum"), 0)
    reg = next(
        (i for i, r in enumerate(params) if str(r.get("case", "")).startswith("register")), None
    )
    dfl = next((i for i, r in enumerate(params) if r.get("case") == "ardupilot_default"), None)
    for lab, col in (
        ("Q_BACKTRANS_MS (T_bt)", "T_bt_s"),
        ("Q_A_ANGLE_MAX (theta_A)", "theta_A_deg"),
        ("Q_M_SPIN_MIN (spin_min)", "spin_min"),
        ("Q_TRANS_DECEL (q)", "Q_TRANS_DECEL_ms2"),
        ("Cost J of the optimum", "J"),
    ):
        it = c.item(lab, "C_params", col, opt)
        if it:
            o.items.append(it)
    spread = c.res.record.get("search_spreads", {}).get("C")
    if isinstance(spread, int | float):
        o.items.append(
            Item(
                "Seed spread of the search", f"{spread:.2%}", "", "run_record.json search_spreads.C"
            )
        )
    else:
        it = c.item("Seed spread of the search", "C_params", "seed_spread", opt)
        if it:
            o.items.append(it)
    mine = [r for r in runs if r.get("case", "optimum") == "optimum"]
    nfl = sum(1 for r in mine if r.get("flags"))
    if mine:
        o.items.append(
            Item(
                "Flagged ensemble members",
                f"{nfl} of {len(mine)}",
                "",
                expr="count of non-empty flags in C_runs, case optimum",
            )
        )
        if nfl:
            o.flags.append(f"{nfl} member(s) flagged")
    for lab, vcol, tcol, unit in (
        ("Worst overshoot / tolerance", "max_overshoot_m", "os_tol_m", "m"),
        ("Worst end sink / tolerance", "max_end_sink_ms", "sink_tol_ms", "m/s"),
    ):
        v, t = num(params[opt].get(vcol)), num(params[opt].get(tcol))
        if ok(v) and ok(t):
            o.items.append(Item(lab, f"{v:.2f} / {t:g}", unit,
                                f"C_params.{vcol}[{opt}]; C_params.{tcol}[{opt}]",
                                f"{v:.2f} / {t:g}"))  # fmt: skip
    if params[opt].get("at_edge") is True:
        o.flags.append(
            "the optimum is at the edge of a tolerance: overshoot margin "
            f"{num(params[opt].get('os_margin_m')):.2f} m, end-sink margin "
            f"{num(params[opt].get('sink_margin_ms')):.2f} m/s")
    bound = str(params[opt].get("at_bound") or "")
    if bound:
        o.flags.append("at a search bound: " + bound + bound_reason(params[opt], bound))

    # 1. energy against distance: C only (B1's points are single scenarios; its chapter has them)
    series, labels = [], []
    cr = sorted((r for r in curve if (r.get("grid") or "coarse") == "coarse"
                 and ok(num(r.get("p95_distance_m"))) and ok(num(r.get("mean_energy_kJ")))
                 and ok(num(r.get("q")))), key=lambda r: num(r["q"]))  # fmt: skip
    if cr:
        series.append(ch.Series(
            "searched ramp, q varied", "s1",
            [(r["p95_distance_m"], r["mean_energy_kJ"]) for r in cr],
            [tip(r, ["q", "mean_energy_kJ", "p95_distance_m", "p95_excursion_m", "J"],
                 str(r.get("case", ""))) for r in cr],
            hollow=[bool(num(r.get("n_overshoot")) or num(r.get("n_sinking"))
                         or num(r.get("n_other_flags"))) for r in cr],
            labels={i: f"q = {r['q']:g}" for i, r in enumerate(cr)},
            label_pos={i: (-9.0, 17.0) for i in range(len(cr))}))  # fmt: skip
        labels.append(
            ("s1", "line", "C: the optimum's ramp, q varied (hollow = a member is flagged)")
        )
    cols = ["T_bt_s", "theta_A_deg", "spin_min", "Q_TRANS_DECEL_ms2", "mean_energy_kJ",
            "p95_distance_m", "J"]
    # drawn in this order, so the optimum (last) is never hidden under the default
    marks: list[tuple[int | None, str, str, str, bool, str, tuple[float, float]]] = [
        (reg, "C register (q = 2)", "s1", "sq", True, "", (0.0, 0.0)),
        (dfl, "ArduPilot default", "ref", "sq", False, "ArduPilot default", (12.0, 20.0)),
        (opt, "C optimum", "s1", "dia", False, "C optimum", (12.0, -10.0)),
    ]
    for idx, name, cls, kind, hol, lab, pos in marks:
        if idx is not None and ok(num(params[idx].get("p95_distance_m"))):
            r = params[idx]
            series.append(ch.Series(
                name, cls, [(r["p95_distance_m"], r["mean_energy_kJ"])], [tip(r, cols, name)],
                line=False, marker=kind, hollow=[hol],
                labels={0: lab} if lab else {}, label_pos={0: pos} if pos else {}))  # fmt: skip
            labels.append((cls, "hollow" if hol else kind, name))
    if series:
        o.figures.append(
            panelbox(
                "Energy against stopping distance",
                "C: ensemble mean energy against the 95th-percentile distance, the optimum's "
                "ramp with q varied (each point is labelled with its q). B1's scenario points "
                "are in the B1 chapter.",
                ch.xy_chart(
                    series, "stopping distance [m]", "bus energy [kJ]", "Energy against distance"
                ),
                _legend(*labels),
            )
        )
    else:
        o.absent.append("not in this run: C_q_curve")

    # 2. time histories
    build_c_histories(c, o)

    mf = margin_figure(curve, params, opt)
    if mf:
        o.figures.append(mf)
    # 3. convergence
    srch = c.rows("C_search")
    if srch:
        o.figures.append(
            panelbox(
                "Search convergence",
                "Best cost found by each seed; the spread of these is the T-12 check.",
                ch.dot_chart(
                    [
                        (
                            f"seed {r['seed']:g}",
                            r["best_cost"],
                            tip(r, ["seed", "best_cost", "evaluations", "T_bt_s", "theta_A_deg",
                                    "spin_min", "Q_TRANS_DECEL_ms2"]),
                            "s1",
                        )
                        for r in srch
                    ],
                    "best cost J",
                    "Search convergence",
                    zero=False,
                )
                + (f'<p class="cap">{ch.esc(seed_statement(srch))}</p>'
                   if seed_statement(srch) else ""),
            )
        )
    # 4. spread strips
    if mine:
        metrics = [
            ("energy_kJ", "energy [kJ]"),
            ("excursion_m", "excursion [m]"),
            ("end_sink_ms", "end sink [m/s]"),
            ("overshoot_m", "overshoot [m]"),
        ]
        dmem = [r for r in runs if r.get("case") == "ardupilot_default"]
        rows: list[ch.Strip] = []
        extras: list[ch.StripExtra] = []
        tols = {"overshoot_m": (num(params[opt].get("os_tol_m")), "tolerance"),
                "end_sink_ms": (num(params[opt].get("sink_tol_ms")), "tolerance")}
        for col, lab in metrics:
            if not any(col in r for r in mine):
                continue
            dots = [(num(r.get(col)), tip(r, ["member", col, "flags"]), bool(r.get("flags")), "s1")
                   for r in mine if ok(num(r.get(col)))]  # fmt: skip
            rows.append((lab, dots, None))
            tv, tl = tols.get(col, (NAN, ""))
            extras.append((
                [(num(r.get(col)), tip(r, ["member", col, "flags"], "ArduPilot default"))
                 for r in dmem if ok(num(r.get(col)))],
                (tv, f"{tl} {tv:g}") if ok(tv) else None))  # fmt: skip
        leg = [("s1", "dot", "member of the optimum"), ("hollow", "hollow", "flagged member")]
        if dmem:
            leg.append(("ref", "hollow", "member of the ArduPilot default"))
        if any(x[1] for x in extras):
            leg.append(("ref", "dash", "tolerance (PROPOSED)"))
        stat = spread_statement(mine, params[opt])
        o.figures.append(
            panelbox(
                "Ensemble spread",
                "One strip per metric, one dot per member; the ArduPilot default's members are "
                "the hollow grey dots below each strip.",
                ch.strip_chart(rows, "Ensemble spread", extras=extras)
                + (f'<p class="cap">{ch.esc(stat)}</p>' if stat else ""),
                _legend(*leg),
            )
        )
    # 5. sensitivity
    sens = c.rows("C_sens")
    j0 = num(params[opt].get("J"))
    js0 = num(params[opt].get("J_smooth"))
    if sens and ok(js0) and js0 != 0 and any("J_smooth" in r for r in sens):
        o.figures.append(sensitivity_figure(sens, js0))
    elif sens and ok(j0) and j0 != 0:  # results from before the failure counts were written
        items = [(f"{r['parameter']} = {r['value']}", (r["J"] / j0 - 1) * 100,
                  f"J = {r['J']:.3f}; change = ({r['J']:.3f} / {j0:.3f} - 1) x 100", "s1")
                 for r in sens if ok(num(r.get("J")))]  # fmt: skip
        o.figures.append(
            panelbox(
                "One-at-a-time sensitivity",
                "Change of the cost J against the optimum when one setting "
                "changes (the tooltip has the expression).",
                ch.dot_chart(items, "change of J [%]", "C sensitivity"),
            )
        )
    o.tables = [
        ("C_params", None),
        ("C_runs", None),
        ("C_q_curve", None),
        ("C_sens", None),
        ("C_search", None),
        ("C_history", None),
    ]
    return o


# ------------------------------------------------------------------------------------------ B1
LATE_WARNING = "verified for exact timing only: fails with commands 10 ms late"
VARIANT_ORDER = ("attached", "c_envelope")


def gap_items(gap: Sequence[Row]) -> list[Item]:
    """The tiles of the B1-C gap: the `attached` figures first (every verified scenario), then
    `c_envelope` beside them with its warning when its schedule fails a late command.

    Read from the existing `B1_gap` rows; the energy tiles are those of the first verified row.
    """
    order = {v: n for n, v in enumerate(VARIANT_ORDER)}
    rows = [(i, r) for i, r in enumerate(gap) if r.get("b1_replay_verified") is True
            and ok(num(r.get("b1_energy_kJ"))) and ok(num(r.get("c_energy_kJ")))]  # fmt: skip
    rows.sort(key=lambda ir: (order.get(str(ir[1].get("variant")), 9), ir[0]))
    items: list[Item] = []
    for n, (gi, r) in enumerate(rows):
        c_e, b_e = num(r["c_energy_kJ"]), num(r["b1_energy_kJ"])
        warn = LATE_WARNING if r.get("b1_late_command_ok") is False else ""
        if n == 0:
            for lab, col in (("C energy", "c_energy_kJ"), ("B1 energy", "b1_energy_kJ")):
                items.append(Item(f"{lab} ({r['scenario']}, {r['variant']})",
                                  f"{num(r[col]):.2f}", "kJ", f"B1_gap.{col}[{gi}]"))
        if c_e:
            items.append(Item(
                f"B1 below C ({r['scenario']}, {r['variant']})", f"{(1 - b_e / c_e) * 100:.0f} %",
                "", f"B1_gap.b1_energy_kJ[{gi}]; B1_gap.c_energy_kJ[{gi}]",
                f"1 - {b_e:.2f}/{c_e:.2f}", warn))  # fmt: skip
    return items


def build_b1(c: Ctx) -> Out:
    o = Out()
    front, gap = c.rows("B1_front"), c.rows("B1_gap")
    if gap:
        o.items += gap_items(gap)
        # the quoted gap can come from a schedule that does not survive a late command: say so
        late = [r for r in gap if r.get("b1_replay_verified") is True
                and r.get("b1_late_command_ok") is False]  # fmt: skip
        for r in late:
            o.flags.append(f"the B1-C gap quoted ({r['scenario']}, {r['variant']}) comes from a "
                           "schedule that fails when its commands are 10 ms late")
    if front:
        bi = c.extreme("B1_front", "energy_kJ", min, lambda r: r.get("replay_verified") is True)
        if bi is not None:
            for lab, col in (
                ("Rotors on at", "t_rotor_on_s"),
                ("V / Vs at rotor start", "V_over_Vs_rotor_on"),
                ("Peak wing angle (gated)", "alpha_w_peak_gated_deg"),
            ):
                it = c.item(
                    lab,
                    "B1_front",
                    col,
                    bi,
                    note=f"{front[bi].get('scenario')}, {front[bi].get('variant')}",
                )
                if it:
                    o.items.append(it)
        nv = sum(1 for r in front if r.get("replay_verified") is True)
        o.items.append(
            Item(
                "Replay-verified points",
                f"{nv} of {len(front)}",
                "",
                expr="count of replay_verified = yes in B1_front",
            )
        )
        if nv < len(front):
            o.flags.append(f"{len(front) - nv} point(s) without a verified solution")
        n_fragile = sum(1 for r in front if r.get("replay_verified") is True
                        and r.get("replay_late_command_ok") is False)
        if n_fragile:
            o.flags.append(
                f"{n_fragile} verified point(s) fail when the commands are 10 ms late "
                "(replay_late_command_ok = no): the optimum sits on the lift peak, a fraction of a "
                "degree below the stall cliff")
        panels, leg = [], {}
        for s in uniq([r["scenario"] for r in front]):
            series = []
            for v in uniq([r.get("variant") for r in front]):
                rs = sorted([r for r in front if r["scenario"] == s and r.get("variant") == v
                             and r.get("replay_verified") is True and ok(num(r.get("energy_kJ")))],
                            key=lambda r: r["distance_m"])  # fmt: skip
                if rs:
                    cls = {"attached": "s2", "c_envelope": "s3"}.get(v, "ink")
                    series.append(ch.Series(f"B1 {v}", cls, [(r["distance_m"], r["energy_kJ"]) for r in rs],
                                            [tip(r, ["eps_x_m", "energy_kJ", "distance_m", "excursion_m", "t_f_s"],
                                                 f"B1 {v}") for r in rs]))  # fmt: skip
                    leg[v] = (cls, "line", f"B1 {v}")
            cp = c.rows("C_params")
            ci = next((i for i, r in enumerate(cp) if r.get("case") == "optimum"), None)
            if ci is not None and ok(num(cp[ci].get("p95_distance_m"))):
                series.append(ch.Series("C optimum", "s1", [(cp[ci]["p95_distance_m"], cp[ci]["mean_energy_kJ"])],
                                        [tip(cp[ci], ["mean_energy_kJ", "p95_distance_m"], "C optimum (ensemble)")],
                                        line=False, marker="dia"))  # fmt: skip
                leg["C"] = ("s1", "dia", "C optimum (ensemble mean, p95 distance)")
            if series:
                panels.append(
                    (
                        s,
                        ch.xy_chart(
                            series, "distance [m]", "bus energy [kJ]", f"Front {s}", w=340, h=280
                        ),
                    )
                )
        if panels:
            o.figures.append(
                panelbox(
                    "Pareto front",
                    "Least energy against the distance cap, per wing-limit "
                    "variant; the C optimum is overlaid for scale.",
                    ch.multiples(panels),
                    _legend(*leg.values()),
                )
            )
    if gap:
        items = []
        for r in gap:
            if r.get("b1_replay_verified") is True and ok(num(r.get("b1_energy_kJ"))):
                lab = f"{r['scenario']} {r['variant']}"
                items.append(
                    (
                        lab + ": C",
                        r["c_energy_kJ"],
                        tip(r, ["c_energy_kJ", "c_distance_m"], lab + " C"),
                        "s1",
                    )
                )
                cls = {"attached": "s2", "c_envelope": "s3"}.get(r["variant"], "ink")
                items.append(
                    (
                        lab + ": B1",
                        r["b1_energy_kJ"],
                        tip(r, ["b1_energy_kJ", "b1_distance_m", "energy_margin"], lab + " B1"),
                        cls,
                    )
                )
        if items:
            o.figures.append(
                panelbox(
                    "B1 against C at C's own distance",
                    "Energy of the same problem in the two "
                    "analyses (B1 has no rotor lag where C has one).",
                    ch.bar_chart(items, "bus energy [kJ]", "Gap B1 to C"),
                    _legend(
                        ("s1", "line", "C"),
                        ("s2", "line", "B1 attached"),
                        ("s3", "line", "B1 c_envelope"),
                    ),
                )
            )
    sched = c.rows("B1_schedule")
    if sched:
        ts = []
        for v in uniq([r.get("variant") for r in sched]):
            rs = [r for r in sched if r.get("variant") == v]
            cls = {"attached": "s2", "c_envelope": "s3"}.get(v, "ink")
            ts.append(
                ch.TSeries(
                    f"B1 {v}",
                    cls,
                    f"B1 {v}",
                    [r["t"] for r in rs],
                    {
                        "delta_r": [num(r.get("delta_r")) for r in rs],
                        "theta_cmd": [num(r.get("theta_cmd")) for r in rs],
                    },
                    False,
                    True,
                )
            )
        o.figures.append(
            panelbox(
                "Optimal schedule",
                "The rotor throttle and the pitch command that B1 found "
                f"(scenario {sched[0].get('scenario')}, distance cap {sched[0].get('eps_x_m'):g} m).",
                ch.panel_figure(
                    "b1_sched",
                    [
                        ch.Panel("delta_r", "rotor throttle [-]", "", 2),
                        ch.Panel("theta_cmd", "pitch command [deg]", "deg", 1),
                    ],
                    ts,
                ),
                _legend(*[(s.cls, "line", s.short) for s in ts]),
            )
        )
    bh = c.rows("B1_history") or sched
    if bh:
        plan = {
            f"B1 {v}": (
                {"attached": "s2", "c_envelope": "s3"}.get(v, "ink"),
                f"B1 {v}",
                False,
                True,
            )
            for v in uniq(
                [r.get("variant") or str(r.get("case", "")).replace("B1 ", "") for r in bh]
            )
        }
        rows = [{**r, "case": r.get("case") or f"B1 {r.get('variant')}"} for r in bh]
        ts = hist_series(rows, plan, ["V", "h", "theta", "delta_r"])
        if ts:
            o.figures.append(
                panelbox(
                    "Time histories",
                    "B1's replayed solution at the headline cap, as in C's figure.",
                    ch.panel_figure("b1_hist", HIST_PANELS, ts),
                    _legend(*[(s.cls, "line", s.short) for s in ts]),
                )
            )
    sens = c.rows("B1_sens")
    if sens:
        items = [(f"{r.get('sensitivity')} = {r.get('value')} ({r.get('scenario')})", r["energy_kJ"],
                  tip(r, ["energy_kJ", "distance_m", "replay_verified"]), "s2")
                 for r in sens if ok(num(r.get("energy_kJ")))]  # fmt: skip
        if items:
            o.figures.append(
                panelbox(
                    "Sensitivity",
                    "Energy of B1 when one setting changes.",
                    ch.dot_chart(items, "bus energy [kJ]", "B1 sensitivity", zero=False),
                )
            )
    o.tables = [
        ("B1_front", None),
        ("B1_gap", None),
        ("B1_sens", None),
        ("B1_schedule", None),
        ("B1_history", None),
    ]
    return o


# ------------------------------------------------------------------------------------------ FT
FT_CASES = {
    "optimum": ("s1", "optimum", False, True),
    "register": ("s2", "register", False, False),
    "ardupilot_default": ("ref", "ArduPilot default", True, False),
}


def build_ft(c: Ctx) -> Out:
    o = Out()
    params, runs, hist = c.rows("FT_params"), c.rows("FT_runs"), c.rows("FT_history")
    if not params:
        return o
    row = {r.get("case"): i for i, r in enumerate(params)}
    opt, reg = row.get("optimum", 0), row.get("register")
    for lab, col in (
        ("AIRSPEED_MIN (EAS)", "airspeed_min_eas_ms"),
        ("Q_TRANSITION_MS", "q_transition_s"),
        ("TKOFF_THR_MAX", "tkoff_thr_max"),
        ("Q_TRAN_PIT_MAX", "tran_pit_max_deg"),
        ("Cost J of the optimum", "J"),
    ):
        it = c.item(
            lab,
            "FT_params",
            col,
            opt,
            note=(f"register {fmt_value(params[reg].get(col), col)}" if reg is not None else ""),
        )
        if it:
            o.items.append(it)
    sp = c.res.record.get("search_spreads", {}).get("FT")
    if isinstance(sp, int | float):
        o.items.append(
            Item("Seed spread of the search", f"{sp:.2%}", "", "run_record.json search_spreads.FT")
        )
    kinds = [k for k in params[opt] if k.startswith("n_") and k != "n_flags"]
    fl = [
        (k[2:], int(num(params[opt][k])))
        for k in kinds
        if ok(num(params[opt][k])) and params[opt][k]
    ]
    if fl:
        o.flags.append("flags at the optimum: " + ", ".join(f"{k} {n}" for k, n in fl))
    for lab, col in (
        ("Worst sink rate", "worst_sink_ms"),
        ("Worst stall margin", "worst_stall_margin"),
    ):
        it = c.item(lab, "FT_params", col, opt)
        if it:
            o.items.append(it)
    bound = str(params[opt].get("at_bound") or "")
    if bound:
        o.flags.append("at a search bound: " + bound)

    stall = c.study("FT", "stall_margin", 1.2)
    loss = c.context.get("mission", {}).get("fwd_alt_loss_max_m")
    gain = c.context.get("mission", {}).get("fwd_alt_gain_max_m")
    case_leg = [
        (FT_CASES[k][0], "dash" if FT_CASES[k][2] else "line", FT_CASES[k][1])
        for k in FT_CASES
        if k in row
    ]
    # 1. time histories, one figure per scenario
    for s in uniq([r.get("scenario") for r in hist])[:2]:
        hs = [r for r in hist if r.get("scenario") == s]
        ts = hist_series(
            hs,
            {k: v for k, v in FT_CASES.items()},
            ["V_eas", "h_rel", "theta", "delta_r", "delta_p"],
        )
        capped = [r for r in hs if r.get("case") == "optimum" and ok(num(r.get("theta_cap")))]
        if capped:
            ts.append(
                ch.TSeries(
                    "pitch cap (optimum)",
                    "ink",
                    "pitch cap",
                    [r["t"] for r in capped],
                    {"theta": [r["theta_cap"] for r in capped]},
                    False,
                    True,
                )
            )
        modes: list[tuple[float, str]] = []
        last = None
        for r in hs:
            if r.get("case") == "optimum" and r.get("mode") and r["mode"] != last:
                last = r["mode"]
                modes.append((r["t"], str(last).replace("fwd_", "")))
        vs = [num(r.get("V_s_eas_ms")) for r in runs if ok(num(r.get("V_s_eas_ms")))]
        eas_rules = []
        if ok(num(params[opt].get("airspeed_min_eas_ms"))):
            eas_rules.append((params[opt]["airspeed_min_eas_ms"], "AIRSPEED_MIN (optimum)"))
        if vs:
            eas_rules.append((stall * sum(vs) / len(vs), f"{stall:g} x stall speed"))
        band = (
            (-float(loss), float(gain))
            if isinstance(loss, int | float) and isinstance(gain, int | float)
            else None
        )
        panels = [
            ch.Panel("V_eas", "airspeed [m/s, EAS]", "m/s", 2, hlines=eas_rules, vticks=modes[1:]),
            ch.Panel("h_rel", "altitude change [m]", "m", 2, True, band=band),
            ch.Panel("theta", "pitch [deg]", "deg", 1),
            ch.Panel("delta_r", "rotor throttle [-]", "", 2),
            ch.Panel("delta_p", "pusher throttle [-]", "", 2),
        ]
        if ts:
            o.figures.append(
                panelbox(
                    f"Time histories, {s}",
                    "Optimum, register start and ArduPilot default through the wait, "
                    "fade and done modes. The shaded band is the accepted altitude change.",
                    ch.panel_figure(f"ft_hist_{s}", panels, ts),
                    _legend(*case_leg, ("ink", "line", "pitch cap (optimum)")),
                )
            )
    if not hist:
        o.absent.append("not in this run: FT_history")
    # 2. end of the fade
    done = [r for r in runs if ok(num(r.get("vh_end_ms"))) and ok(num(r.get("h_proj_rel_m")))]
    if done:
        series = []
        for cs in FT_CASES:
            rs = [r for r in done if r.get("case") == cs]
            if rs:
                cls, short, dash, _ = FT_CASES[cs]
                series.append(ch.Series(short, cls, [(r["h_proj_rel_m"], -r["vh_end_ms"]) for r in rs],
                                        [tip(r, ["set", "member", "vh_end_ms", "h_proj_rel_m", "proj_loss_m", "flags"],
                                             short) for r in rs], line=False,
                                        marker="sq" if cls == "ref" else "dot",
                                        hollow=[bool(r.get("flags")) for r in rs]))  # fmt: skip
        vl = [(-float(loss), "loss limit")] if isinstance(loss, int | float) else []
        o.figures.append(
            panelbox(
                "End of the fade",
                "Sink rate at the end of the fade against the projected minimum "
                "altitude change, one mark per member. A hollow mark is a flagged member.",
                ch.xy_chart(
                    series,
                    "projected minimum altitude change [m]",
                    "sink rate [m/s]",
                    "End of the fade",
                    vlines=vl,
                ),
                _legend(*case_leg, ("s1", "hollow", "flagged member")),
            )
        )
    # 3. wing share across the hand-over
    hand = [
        r
        for r in runs
        if ok(num(r.get("wing_share_handover"))) and ok(num(r.get("V_handover_eas_ms")))
    ]
    if hand:
        series = []
        for cs, (cls, short, _, _) in FT_CASES.items():
            rs = [r for r in hand if r.get("case") == cs]
            if rs:
                series.append(ch.Series(short, cls, [(r["V_handover_eas_ms"], r["wing_share_handover"]) for r in rs],
                                        [tip(r, ["set", "member", "V_handover_eas_ms", "margin_handover",
                                                 "wing_share_handover"], short) for r in rs], line=False,
                                        marker="sq" if cls == "ref" else "dot"))  # fmt: skip
        o.figures.append(
            panelbox(
                "Wing share at the hand-over",
                "Share of the weight carried by the wing when the pusher "
                "takes over, against the equivalent airspeed at that instant.",
                ch.xy_chart(
                    series,
                    "airspeed at the hand-over [m/s, EAS]",
                    "wing share of weight [-]",
                    "Wing share",
                    hlines=[(1.0, "wing carries the weight")],
                ),
                _legend(*case_leg),
            )
        )
    # 3b. the grid over Q_TRANSITION_MS: the cost of the snapped point at each value
    grid = sorted((r for r in c.rows("FT_grid") if ok(num(r.get("J_snapped")))),
                  key=lambda r: r["q_transition_s"])
    if grid:
        gtip = ["q_transition_s", "J_snapped", "J_continuous", "J_snapped_minus_continuous",
                "airspeed_min_eas_ms", "tkoff_thr_max", "tran_pit_max_deg", "seed_spread", "n_flags"]
        gseries = [ch.Series("best snapped J at each Q_TRANSITION_MS", "s1",
                             [(r["q_transition_s"], r["J_snapped"]) for r in grid],
                             [tip(r, gtip) for r in grid],
                             hollow=[r.get("t12_ok") is False for r in grid])]
        wins = [r for r in grid if r.get("winner") is True]
        if wins:
            gseries.append(ch.Series("winner", "s2", [(r["q_transition_s"], r["J_snapped"])
                                                     for r in wins], [tip(r, gtip) for r in wins]))
        failed = [f"{r['q_transition_s']:g} s ({num(r.get('seed_spread')):.1%})" for r in grid
                  if r.get("t12_ok") is False]
        near = [r["q_transition_s"] for r in grid if r.get("within_2pct") is True]
        near_text = (" Within 2 % of the best cost: Q_TRANSITION_MS "
                     + ", ".join(f"{q:g}" for q in near) + " s." if near else "")
        o.figures.append(
            panelbox(
                "Cost against Q_TRANSITION_MS",
                "The best snapped cost found with Q_TRANSITION_MS fixed at each value (the other "
                "three searched, then snapped to ArduPilot's integers). The winner is marked; a "
                "hollow mark is a point whose three-seed spread is above the T-12 limit"
                + (": " + ", ".join(failed) if failed else ", none here") + "." + near_text,
                ch.xy_chart(gseries, "Q_TRANSITION_MS [s]", "cost J",
                            "Cost against Q_TRANSITION_MS"),
                _legend(("s1", "line", "grid point"),
                        ("s1", "hollow", "T-12 spread above limit"), ("s2", "line", "winner")),
            )
        )
    # 4. scan
    scan = c.rows("FT_scan")
    if scan:
        vs = [num(r.get("V_s_eas_ms")) for r in runs if ok(num(r.get("V_s_eas_ms")))]
        pts = sorted((r["airspeed_min_eas_ms"], r["J"]) for r in scan if ok(num(r.get("J"))))
        rs = sorted(
            (r for r in scan if ok(num(r.get("J")))), key=lambda r: r["airspeed_min_eas_ms"]
        )
        series = [ch.Series("best J at each AIRSPEED_MIN", "s1", pts,
                            [tip(r, ["airspeed_min_eas_ms", "J", "q_transition_s", "tkoff_thr_max",
                                     "tran_pit_max_deg", "n_flags", "seed_spread", "at_bound"]) for r in rs],
                            hollow=[bool(num(r.get("n_flags"))) for r in rs])]  # fmt: skip
        vl = [(stall * sum(vs) / len(vs), f"{stall:g} x stall speed")] if vs else []
        o.figures.append(
            panelbox(
                "Cost against AIRSPEED_MIN",
                "The best cost found when AIRSPEED_MIN is fixed and the other "
                "three are searched. A hollow mark has flagged members.",
                ch.xy_chart(
                    series,
                    "AIRSPEED_MIN [m/s, EAS]",
                    "cost J",
                    "Cost against AIRSPEED_MIN",
                    vlines=vl,
                ),
                _legend(("s1", "line", "scan point"), ("s1", "hollow", "flags remain")),
            )
        )
    # 5. convergence and sensitivity
    srch = c.rows("FT_search")
    if srch:
        o.figures.append(
            panelbox(
                "Search convergence",
                "Best cost found by each seed.",
                ch.dot_chart(
                    [
                        (
                            f"seed {r['seed']:g}",
                            r["best_cost"],
                            tip(r, ["seed", "best_cost", "evaluations"]),
                            "s1",
                        )
                        for r in srch
                    ],
                    "best cost J",
                    "FT search convergence",
                    zero=False,
                ),
            )
        )
    sens = c.rows("FT_sens")
    # the sensitivities are evaluated on the search ensemble, so the reference is the `base` row
    # of FT_sens (the unchanged evaluation there), not the optimum's J on the final ensemble
    base = next((r for r in sens if r.get("parameter") == "base"), None)
    j0 = num(base.get("J")) if base else num(params[opt].get("J"))
    if sens and ok(j0) and j0:
        items = [(f"{r['parameter']} = {r['value']}", (r["J"] / j0 - 1) * 100,
                  f"J = {r['J']:.3f}; change = ({r['J']:.3f} / {j0:.3f} - 1) x 100", "s1")
                 for r in sens if ok(num(r.get("J"))) and r is not base]  # fmt: skip
        o.figures.append(
            panelbox(
                "One-at-a-time sensitivity",
                "Change of the cost J against the optimum when one setting changes.",
                ch.dot_chart(items, "change of J [%]", "FT sensitivity"),
            )
        )
    # the stall margin rests on a provisional C_L,max: the optimum re-evaluated with it +-10 %
    cl_rows = [r for r in sens if r.get("parameter") in ("base", "CL_max_scale")]
    if len(cl_rows) > 1:
        shown = [
            {"case": "base" if r["parameter"] == "base" else f"C_L,max x {float(r['value']):g}", **r}
            for r in cl_rows
        ]
        cols = ["case", "cl_max", "alpha_s_rad", "J", "worst_stall_margin", "n_stall_margin",
                "n_stall", "n_alt_loss", "n_proj_loss", "n_flags"]  # fmt: skip
        o.figures.append(
            panelbox(
                "Sensitivity to C_L,max",
                "The optimum's four values held fixed, with the model's C_L,max scaled through the "
                "stall angle alpha_s (the lift, the stall speed, the stall flag and the recovery "
                "follow). Evaluated on the search ensemble, like the other sensitivities.",
                table_html("FT_sens", [c for c in cols if any(c in r for r in shown)], shown),
            )
        )
    # 6. spread strips
    mine = [r for r in runs if r.get("case") == "optimum"]
    dmem = [r for r in runs if r.get("case") == "ardupilot_default"]
    if mine:
        rows: list[ch.Strip] = []
        for col, lab in (
            ("loss_m", "altitude loss [m]"),
            ("gain_m", "altitude gain [m]"),
            ("stall_margin_min", "stall margin [-]"),
            ("energy_kJ", "energy [kJ]"),
        ):
            dots = [(num(r.get(col)), tip(r, ["set", "member", col, "flags"]), bool(r.get("flags")), "s1")
                   for r in mine if ok(num(r.get(col)))]  # fmt: skip
            dv = sorted(num(r.get(col)) for r in dmem if ok(num(r.get(col))))
            rows.append((lab, dots, dv[len(dv) // 2] if dv else None))
        leg = [("s1", "dot", "member of the optimum"), ("hollow", "hollow", "flagged member")]
        if dmem:
            leg.append(("ref", "line", "ArduPilot default: median of its members"))
        o.figures.append(
            panelbox(
                "Ensemble spread",
                "One strip per metric, one dot per member.",
                ch.strip_chart(rows, "FT ensemble spread"),
                _legend(*leg),
            )
        )
    o.tables = [
        ("FT_params", None),
        ("FT_scan", None),
        ("FT_runs", None),
        ("FT_search", None),
        ("FT_sens", None),
        ("FT_history", None),
    ]
    return o


# ------------------------------------------------------------------------------- sensitivity
def build_sens(c: Ctx) -> Out:
    o = Out()
    oat, glob = c.rows("sens_oat"), c.rows("sens_global")
    if oat:
        panels = []
        for out in uniq([r["output"] for r in oat]):
            rs = [
                r
                for r in oat
                if r["output"] == out and ok(num(r.get("lo"))) and ok(num(r.get("hi")))
            ]
            rs.sort(key=lambda r: -abs(r["hi"] - r["lo"]))
            items = [
                (
                    f"{r['input']} [{r.get('status')}]",
                    abs(r["hi"] - r["lo"]),
                    f"{r['input']} ({r.get('status')}): output {r['lo']:.4g} at the low end, {r['hi']:.4g} at the "
                    f"high end; swing = |{r['hi']:.4g} - {r['lo']:.4g}|",
                    "s1",
                )
                for r in rs[:8]
            ]
            if items:
                unit = unit_of(out)[0] or ("J" if out.endswith("_J") else "")
                panels.append(
                    (out, ch.bar_chart(items, f"swing of {out} [{unit}]", f"Tornado {out}", w=420))
                )
        o.figures.append(
            panelbox(
                "One-at-a-time swings",
                "The eight inputs that move each output "
                "most over their stated range; the grade of the input is in brackets.",
                ch.multiples(panels, 2),
            )
        )
        if oat:
            o.items.append(
                Item(
                    "Outputs studied",
                    str(len(uniq([r["output"] for r in oat]))),
                    "",
                    expr="distinct outputs in sens_oat",
                )
            )
            o.items.append(
                Item(
                    "Inputs varied",
                    str(len(uniq([r["input"] for r in oat]))),
                    "",
                    expr="distinct inputs in sens_oat",
                )
            )
    if glob:
        panels = []
        for out in uniq([r["output"] for r in glob]):
            rs = [r for r in glob if r["output"] == out]
            top = sorted(rs, key=lambda r: -num(r.get("morris_mu_star_large")))[:3]
            ser = [ch.Series("inputs", "s1", [(r["morris_mu_star_large"], r["morris_mu_star_small"]) for r in rs],
                             [tip(r, ["morris_mu_star_large", "morris_mu_star_small"], f"{r['input']} ({r.get('status')})")
                              for r in rs], line=False,
                             labels={rs.index(t): t["input"] for t in top})]  # fmt: skip
            panels.append(
                (
                    out,
                    ch.xy_chart(
                        ser,
                        "mu* (large trajectory set)",
                        "mu* (small set)",
                        f"Morris {out}",
                        w=340,
                        h=280,
                        y_from_zero=True,
                    ),
                )
            )
        o.figures.append(
            panelbox(
                "Morris screening",
                "Mean absolute elementary effect with the large and the "
                "small trajectory set; points off the diagonal are not converged.",
                ch.multiples(panels, 2),
            )
        )
    o.notes.append("The sensitivity is evaluated at the register settings (AIRSPEED_MIN, TKOFF_THR_MAX, "
                   "Q_TRANS_DECEL and the other register values), not at the recommended ones.")
    o.tables = [("sens_oat", None), ("sens_global", None)]
    return o


# ------------------------------------------------------------------------------- defaults
def _pct(opt: float, dflt: float) -> tuple[str, str]:
    """The change from the default as text and the expression `1 - optimum/default`.

    A positive value is a saving, a negative one an increase (a worse cost is not a negative
    saving); nothing is printed when the default is zero or either side is missing.
    """
    if not (ok(opt) and ok(dflt)) or dflt == 0:
        return "", ""
    pct = (1 - opt / dflt) * 100
    text = f"{pct:.0f} % saving" if pct >= 0 else f"{-pct:.0f} % increase"
    return text, f"1 - {opt:.4g}/{dflt:.4g}"


def build_defaults(c: Ctx) -> Out:
    """Chapter "Defaults vs optimised": the ArduPilot default as a reference, never a recommendation."""
    o = Out()
    o.notes.append(
        "The default column is the value ArduPilot ships with. It is not tuned for this "
        "airframe and is shown only as a reference."
    )
    srows = [r for r in c.rows("settings_recommended") if r.get("analysis")]
    if srows:
        trs = []
        for r in srows:
            unit = f" {r['unit']}" if r.get("unit") not in (None, "", "-") else ""
            dtext = str(r.get("default_text") or "") or (
                fmt_value(r.get("default"), "") if r.get("default") not in (None, "") else ""
            )
            chg = r.get("change_vs_default")
            trs.append(
                f"<tr><td><span class=\"mono\">{ch.esc(r['parameter'])}</span> "
                f"<small>({ch.esc(r['analysis'])})</small></td>"
                f'<td class="num dflt">{ch.esc(dtext + unit if dtext else "")}</td>'
                f'<td class="num">{ch.esc(fmt_value(r.get("register"), "") + unit)}</td>'
                f'<td class="num">{ch.esc(fmt_value(r.get("value"), "") + unit)}</td>'
                f'<td class="num">{ch.esc(fmt_value(chg, "") + unit if chg not in (None, "") else "")}</td></tr>'
            )
        o.figures.append(
            panelbox(
                "Parameters",
                "The same cells as the settings table above; the change is the recommended "
                "value minus the default.",
                '<div class="tbl full"><table><thead><tr><th>parameter</th><th class="num">ArduPilot '
                'default</th><th class="num">register</th><th class="num">optimum</th>'
                '<th class="num">change vs default</th></tr></thead><tbody>'
                + "".join(trs)
                + "</tbody></table></div>",
            )
        )
    blocks = []
    for name, table, cols in (
        (
            "FT",
            "FT_params",
            ("J", "mean_energy_kJ", "p95_excursion_m", "worst_sink_ms", "worst_stall_margin"),
        ),
        (
            "C",
            "C_params",
            ("J", "mean_energy_kJ", "p95_excursion_m", "p95_distance_m", "max_overshoot_m"),
        ),
    ):
        rows = c.rows(table)
        if not any(r.get("case") == "ardupilot_default" for r in rows):
            continue
        blocks.append((name, table, rows, cols))
    if not blocks:
        return o
    fails = []
    for name, table, rows, cols in blocks:
        dflt = next(r for r in rows if r.get("case") == "ardupilot_default")
        opt = next((r for r in rows if r.get("case") == "optimum"), None)
        flagcols = [k for k in dflt if k.startswith("n_") and k not in ("n_flags",)]
        head = ["case", *[k for k in cols if k in dflt], *flagcols]
        trs = []
        for r in rows:
            cells = []
            for k in head:
                cells.append(esc_cell(fmt_value(r.get(k), k)))
            trs.append(
                "<tr>"
                + "".join(f'<td class="{"num" if i else ""}">{v}</td>' for i, v in enumerate(cells))
                + "</tr>"
            )
        chg = []
        if opt is not None:
            for k in cols:
                txt, expr = _pct(num(opt.get(k)), num(dflt.get(k)))
                if txt:
                    chg.append(f'<td class="num" title="{esc_cell(expr)}">{txt}</td>')
                else:
                    chg.append('<td class="num"></td>')
            trs.append(
                '<tr class="dim"><td>change from the default</td>'
                + "".join((chg[cols.index(k)] if k in cols else "<td></td>") for k in head[1:])
                + "</tr>"
            )
        th = "".join(
            f'<th class="{"num" if i else ""}">{esc_cell(header_label(k))}</th>' for i, k in enumerate(head)
        )
        blocks_html = (
            f'<div class="tbl"><table><thead><tr>{th}</tr></thead><tbody>'
            f"{''.join(trs)}</tbody></table></div>"
        )
        o.figures.append(
            panelbox(
                f"{name}: results by case",
                f"From {table}. The change is the saving 1 - optimum/default (a negative value "
                "is printed as an increase); hover it for the expression.",
                blocks_html,
            )
        )
        # where the default fails: flags it raises and the optimum does not
        runs = c.rows(f"{name}_runs")
        for case in ("ardupilot_default", "optimum"):
            cnt: dict[str, int] = defaultdict(int)
            for r in runs:
                if r.get("case", "optimum") == case:
                    for f in str(r.get("flags") or "").replace(";", ",").split(","):
                        if f.strip():
                            cnt[f.strip()] += 1
            if case == "ardupilot_default":
                dcnt = dict(cnt)
            else:
                ocnt = dict(cnt)
        # a flag the default raises that the optimum does not, or raises more often (the counts
        # are compared, not only the names)
        worse = {k: (n, ocnt.get(k, 0)) for k, n in dcnt.items() if n > ocnt.get(k, 0)}
        fails.append(
            f"{name}: "
            + (
                ", ".join(
                    f"{k} in {n} member(s)" if m == 0 else f"{k} in {n} member(s) against {m}"
                    for k, (n, m) in sorted(worse.items())
                )
                if worse
                else "no flag that the default raises more often than the optimum"
            )
        )
    o.figures.append(
        panelbox(
            "Where the default fails",
            "Flags the default case raises that the optimum does not, or raises more often "
            "(the count for the optimum is given), counted over the ensemble members.",
            "<ul>" + "".join(f"<li>{esc_cell(f)}</li>" for f in fails) + "</ul>",
        )
    )
    o.tables = [("settings_recommended", None)]
    return o


def esc_cell(s: str) -> str:
    return ch.esc(s)


__all__ = ["Ctx", "absent"]
