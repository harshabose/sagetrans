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
from sagetrans.io.view.html import Item, Out, absent, details, panelbox, table_html
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
# ------------------------------------------------------------------------------------------- E
E_SEGS = ("takeoff", "fwd_transition", "climb", "descent", "back_transition", "landing")
E_LABEL = {"takeoff": "take-off", "fwd_transition": "fwd transition", "climb": "climb",
           "descent": "descent", "back_transition": "back transition", "landing": "landing",
           "cruise": "cruise", "drop": "payload drop", "reserve": "reserve", "total": "total"}
E_MISSION = {"one_way": "one way", "round_trip": "round trip (radius)"}


def e_case_parts(row: Row) -> dict[str, float]:
    """Wh per category of one E_budget row, the legs added (the leg is `out`, `ret` or `leg`)."""
    parts: dict[str, float] = {}
    for seg in E_SEGS:
        parts[seg] = sum(num(row.get(f"{d}_{seg}_Wh")) for d in ("out", "ret", "leg")
                         if ok(num(row.get(f"{d}_{seg}_Wh"))))
    parts["cruise"] = num(row.get("cruise_Wh"))
    if ok(num(row.get("drop_Wh"))) and num(row.get("drop_Wh")) > 0.0:
        parts["drop"] = num(row["drop_Wh"])
    parts["reserve"] = num(row.get("reserve_Wh"))
    return parts


def e_verdict(rows: Sequence[Row]) -> str:
    """The maximum range against each requirement, as distance flown (a computed sentence)."""
    plaus = next((r for r in rows if r.get("case") == "plausible"), None)
    if plaus is None:
        return ""
    flown = num(plaus.get("flown_km"))
    text = f"max range {flown:.1f} km flown"
    if abs(num(plaus.get("range_km")) - flown) > 1e-9:
        text += f" (radius {num(plaus['range_km']):.1f} km)"
    bits = []
    for r in rows:
        if r.get("case") == "plausible" or not ok(num(r.get("required_km"))):
            continue
        need = num(r["required_km"])
        bits.append(f"meets {need:g} km" if flown >= need
                    else f"short of {need:g} km by {need - flown:.1f} km")
    return f"<b>{ch.esc(text)}</b>" + (": " + ch.esc("; ".join(bits)) if bits else "")


def energy_cases_figure(bud: Sequence[Row]) -> str | None:
    """Energy by segment at the maximum range, with the required distances behind it.

    One panel per site and mission, from the `case` rows of E_budget (`plausible` and one per
    requirement). The required cases are faint and outlined and appear when the checkbox is on;
    they differ from the plausible case in cruise, reserve and total only. Each mission column
    shares one energy axis, so the sites compare. A requirement is the distance flown.
    """
    if not any(r.get("case") for r in bud):
        return None
    keys = uniq([(r["site_amsl_m"], r["mission"]) for r in bud])
    missions = [m for m in E_MISSION if any(k[1] == m for k in keys)]
    sites = sorted({k[0] for k in keys})
    xmax: dict[str, float] = {}
    for m in missions:
        xmax[m] = 1.03 * max([num(r.get("total_Wh")) for r in bud if r["mission"] == m
                              and ok(num(r.get("total_Wh")))]
                             + [num(r.get("usable_Wh")) for r in bud if r["mission"] == m
                                and ok(num(r.get("usable_Wh")))] + [1.0])  # fmt: skip
    panels = []
    for site in sites:
        for m in missions:
            rows = [r for r in bud if r["site_amsl_m"] == site and r["mission"] == m]
            title = f"site {site:g} m, {E_MISSION[m]}"
            plaus = next((r for r in rows if r.get("case") == "plausible"), None)
            if plaus is None:
                panels.append((title, absent("the usable energy does not cover the fixed segments: "
                                              "no maximum range")))
                continue
            order = [r for r in rows if r.get("case") != "plausible"]
            parts = {r["case"]: e_case_parts(r) for r in rows}
            usable = num(plaus.get("usable_Wh"))
            cats = [k for k in E_LABEL if k in parts["plausible"] and k != "total"]
            spans: dict[str, dict[str, tuple[float, float]]] = {}
            for r in rows:
                run, sp = 0.0, {}
                for k in cats:
                    v = parts[r["case"]].get(k, 0.0)
                    sp[k] = (run, run + (v if ok(v) else 0.0))
                    run = sp[k][1]
                sp["total"] = (0.0, num(r.get("total_Wh")))
                spans[r["case"]] = sp
            crow = []
            for k in [*cats, "total"]:
                a, b = spans["plausible"][k]
                what = f"{E_LABEL[k]} at the maximum range ({num(plaus['flown_km']):.1f} km): "
                main_cls = "s2" if k == "reserve" else ("ink" if k == "total" else "s1")
                ghosts = []
                for gi, r in enumerate(order):
                    ga, gb = spans[r["case"]][k]
                    extra = ""
                    if k == "total":
                        over = -num(r.get("margin_Wh"))
                        extra = (f", {over:.0f} Wh over the usable energy" if over > 0
                                 else f", {-over:.0f} Wh to spare")
                    ghosts.append((gi, ga, gb, f"{E_LABEL[k]} at the required "
                                   f"{num(r.get('required_km')):g} km: {gb - ga:.1f} Wh{extra}"))
                crow.append(ch.CaseRow(E_LABEL[k], (a, b, main_cls, f"{what}{b - a:.1f} Wh"),
                                       ghosts))
            svg = ch.case_bars(crow, "energy [Wh]", f"Energy by segment, {title}", xmax[m],
                               rule=(usable, f"usable {usable:.0f}"))
            panels.append((title, f'<p class="cap sub">{e_verdict(rows)}</p>{svg}'))
    reqs = sorted({num(r["required_km"]) for r in bud if ok(num(r.get("required_km")))})
    names = " and ".join(f"{q:g}" for q in reqs)
    legend = ('<div class="legend" role="list" aria-label="Legend">'
              '<span><i class="sw s1"></i>plausible: the maximum range the energy allows</span>'
              + "".join(f'<span class="gl"><i class="sw s1 gh g{i}"></i>required {q:g} km</span>'
                        for i, q in enumerate(reqs))
              + '<span><i class="sw ink"></i>total (ends on the usable-energy rule at the '
              "maximum range)</span></div>")
    control = ('<input type="checkbox" id="e-req" class="etoggle"><label for="e-req" '
               f'class="elabel">Show the required ranges ({names} km) on top of the plausible '
               "range</label>")
    return panelbox(
        "Maximum range and the required ranges: energy by segment",
        "Energy of each segment (out and return added) at the maximum range the usable energy "
        f"allows. The checkbox adds the same budget at the required distances ({names} km flown; "
        "a round trip of 100 km is a 50 km radius), faint and outlined: they differ only in "
        "cruise, reserve and total, and they pass the usable-energy rule wherever the aircraft "
        "cannot fly them. Each column shares one axis.",
        control + legend + ch.multiples(panels, 2))


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
    # energy by segment at the maximum range with the required distances behind it (E_budget has
    # a `case` column); a folder from before it keeps the two older figures below
    cases_fig = energy_cases_figure(bud)
    if cases_fig:
        o.figures.append(cases_fig)
    segs = ["takeoff", "fwd_transition", "climb", "descent", "back_transition", "landing"]
    if bud and not cases_fig:
        panels = []
        for key in uniq([(r["site_amsl_m"], r["mission"]) for r in bud]):
            rs = [r for r in bud if (r["site_amsl_m"], r["mission"]) == key]
            r = max(rs, key=lambda x: num(x.get("range_km")) if ok(num(x.get("range_km"))) else -1)
            items = []
            for sg in segs:
                v = sum(
                    num(r.get(f"{d}_{sg}_Wh"))
                    for d in ("out", "ret", "leg")
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
    if summ and not cases_fig:
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
    "C unconstrained (reference)": ("ink", True, False),
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
    ref_h = [r for r in c.rows("C_ref_history") if r.get("scenario") == name]
    if series and ref_h:
        series["C unconstrained (reference)"] = ref_h
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
            + ("where the run stopped. The dashed curve is C without the wing-angle cap (the "
               "model ignores the stall; reference only)." if ref_h else "where the run stopped.")
            + note,
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
          ch.dot_chart(fails, "failing members of the ensemble", "C sensitivity: failing members",
                       integer=True)),
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


def flat_tail_statement(curve: Sequence[Row], runs: Sequence[Row]) -> str:
    """Why the curves go flat at high q, computed from the rows: past the capability of the pitch
    ramp the run no longer depends on q (the pitch sits at its ceiling), so the energy, the
    distance and the end sink repeat; only the overshoot, which is read against v^2 / (2 q), grows.
    Empty when no two of the highest-q rows repeat."""
    rs = sorted((r for r in curve if (r.get("grid") or "coarse") == "coarse"
                 and ok(num(r.get("q"))) and ok(num(r.get("p95_distance_m")))
                 and ok(num(r.get("max_end_sink_ms")))), key=lambda r: num(r["q"]))  # fmt: skip
    flat: list[Row] = []
    for r in reversed(rs):
        if flat and (abs(num(r["p95_distance_m"]) - num(flat[0]["p95_distance_m"])) > 1e-6
                     or abs(num(r["max_end_sink_ms"]) - num(flat[0]["max_end_sink_ms"])) > 1e-6):
            break
        flat.append(r)
    if len(flat) < 2:
        return ""
    q_lo = num(flat[-1]["q"])
    caps = [num(r.get("a_cap_ms2")) for r in runs if r.get("case", "optimum") == "optimum"
            and ok(num(r.get("a_cap_ms2")))]  # fmt: skip
    cap = (f" The pitch ramp can deliver {min(caps):.2f} m/s^2 for the weakest member and "
           f"{max(caps):.2f} for the strongest (`a_cap_ms2`)." if caps else "")
    return (f"From q = {q_lo:g} the energy, the distance and the end sink do not change with q "
            f"({num(flat[0]['max_end_sink_ms']):.2f} m/s): the pitch is at its ceiling for the "
            "whole run, so the run is the same whatever q is; only the overshoot, which is read "
            "against the target v^2 / (2 q), keeps growing." + cap)


def margin_figure(curve: Sequence[Row], params: Sequence[Row], opt: int,
                  runs: Sequence[Row] = ()) -> str | None:
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
                y_from_zero=True, hlines=[(tol, f"tolerance {tol:g} (PROPOSED)")])))
    if not panels:
        return None
    os_m, sk_m = num(params[opt].get("os_margin_m")), num(params[opt].get("sink_margin_ms"))
    margins = (f" At the optimum the margins are {os_m:.2f} m and {sk_m:.2f} m/s."
               if ok(os_m) and ok(sk_m) else "")
    sentence = ("The optimum relies on the overshoot and end-sink tolerances, which are PROPOSED, "
                "not on a measured limit." + margins)
    flat = flat_tail_statement(curve, runs)
    if flat:
        sentence += " " + flat
    leg = _legend(("s1", "line", "coarse curve (hollow: a member fails)"), ("s1", "sq", "fine grid"),
                  ("s1", "dia", "optimum"), ("s3", "dia", "margin row (reported, not recommended)"))
    return panelbox(
        "Worst overshoot and worst end sink against q",
        "The optimum's other three values fixed; q varied. A hollow mark has a failing member.",
        ch.multiples(panels, 2) + f'<p class="cap">{ch.esc(sentence)}</p>', leg)


CONFIG_COLS = ("Q_TRANS_DECEL_ms2", "T_bt_s", "theta_A_deg", "spin_min", "mean_energy_kJ",
               "p95_distance_m", "p95_stop_time_s", "n_fail_total", "max_overshoot_m",
               "max_end_sink_ms")
EXPOSURE_COLS = ("stall_alpha_peak_deg_max", "stall_members_above_peak", "stall_t_above_peak_s_max",
                 "stall_t_above_limit_s_max", "stall_wing_share_min")
LIMIT_NOTE = ("the limit caps the pitch command, so the angle can sit about a degree above it; "
              "the C_L peak is the stall")
UNCONSTRAINED = ("unconstrained: the model ignores the stall; reference only; never a "
                 "recommendation")


def exposure_cells(r: Row) -> Row:
    """The stall-exposure cells of a C_params row, with "k of n" for the members above the peak."""
    k, n = num(r.get("stall_members_above_peak")), num(r.get("stall_members"))
    return {**{c: r.get(c) for c in EXPOSURE_COLS},
            "stall_members_above_peak": f"{k:g} of {n:g}" if ok(k) and ok(n) else ""}


def configurations_figure(params: Sequence[Row], opt: int, ref: Sequence[Row]) -> str | None:
    """The enforced headline and the unconstrained reference side by side, with a sentence of
    ratios computed from the rows (energy, distance, stop time, time above the lift peak)."""
    head = params[opt]
    refo = next((r for r in ref if r.get("case") == "optimum"), None)
    if refo is None or head.get("alpha_enforced") is not True:
        return None
    lim = num(head.get("alpha_limit_deg"))
    rows = [
        {"configuration": f"enforced, |alpha_w| <= {lim:.2f} deg (headline)",
         **{c: head.get(c) for c in CONFIG_COLS}, **exposure_cells(head)},
        {"configuration": UNCONSTRAINED, **{c: refo.get(c) for c in CONFIG_COLS},
         **exposure_cells(refo)},
    ]  # fmt: skip
    sent = ""
    e1, e2 = num(head.get("mean_energy_kJ")), num(refo.get("mean_energy_kJ"))
    d1, d2 = num(head.get("p95_distance_m")), num(refo.get("p95_distance_m"))
    t1, t2 = num(head.get("p95_stop_time_s")), num(refo.get("p95_stop_time_s"))
    if all(ok(v) and v > 0 for v in (e1, e2, d1, d2, t1, t2)):
        sent = (f"Holding the wing below the limit costs {e1 / e2:.2f} x the energy, "
                f"{d1 / d2:.2f} x the stopping distance and {t1 / t2:.2f} x the stopping time of "
                "the unconstrained search; q is "
                f"{num(head.get('Q_TRANS_DECEL_ms2')):.2f} against "
                f"{num(refo.get('Q_TRANS_DECEL_ms2')):.2f} m/s^2.")
    cols = ["configuration", *CONFIG_COLS, *EXPOSURE_COLS]
    return panelbox(
        "The enforced configuration and the unconstrained reference",
        "C twice: the wing held to the limit (the headline, written to the recommended settings) "
        "and not held (the model's lift falls from its peak to about a third within a few "
        "degrees, so the unconstrained optimum flies through the stall). The exposure columns "
        "are the worst member (the least wing share is the lowest over members); they are not "
        "penalised and not failure flags. " + sent,
        table_html("C_configurations", cols, rows))


def exposure_figure(params: Sequence[Row], ref: Sequence[Row]) -> str | None:
    """Stall exposure of every case in both configurations (report only)."""
    rows = []
    for cfg, ps in (("enforced", params), ("unconstrained", ref)):
        for r in ps:
            if r.get("case") in ("optimum", "ardupilot_default") and ok(
                    num(r.get("stall_alpha_peak_deg_max"))):
                rows.append({"configuration": cfg, "case": r["case"], **exposure_cells(r)})
    if not rows:
        return None
    return panelbox(
        "Stall exposure",
        "Per case, over the ensemble members, airspeed at least D.alpha_gate_ms and t > 0: the "
        "peak |alpha_w|, how many members go above the C_L peak angle, the longest time above "
        "the peak and above the limit, and the least wing share of the weight. " + LIMIT_NOTE
        + ".",
        table_html("C_stall_exposure", ["configuration", "case", *EXPOSURE_COLS], rows))


def alpha_curve_figure(curve: Sequence[Row]) -> str | None:
    """What the margin costs: one C search per wing limit, and the unconstrained one."""
    if not curve:
        return None
    rows = []
    for r in curve:
        kind = str(r.get("kind", ""))
        label = {"headline": "headline (margin_alpha_frac)", "unconstrained": UNCONSTRAINED,
                 }.get(kind, "curve point")
        rows.append({"limit": label, **r})
    cols = ["limit", "alpha_frac", "limit_deg", "Q_TRANS_DECEL_ms2", "T_bt_s", "theta_A_deg",
            "spin_min", "p95_distance_m", "p95_stop_time_s", "mean_energy_kJ", "n_fail_total",
            "max_overshoot_m", "max_end_sink_ms"]
    return panelbox(
        "The price of the margin",
        "One C search per wing limit (a fraction of alpha_s; the limit in degrees follows). "
        "The distance and the stop time are 95th percentiles over the ensemble, the energy the "
        "mean, the failures the total count.",
        table_html("C_alpha_curve", cols, rows))


def caveats_panel(p: Row) -> str:
    """What the C result rests on, computed from the values the run used."""
    bits = []
    a_s, m_blend, clmax = num(p.get("alpha_s_deg")), num(p.get("blend_M")), num(p.get("cl_max"))
    pk = num(p.get("alpha_peak_deg"))
    if ok(a_s):
        bits.append(
            f"alpha_s = {a_s:.2f} deg is a placeholder (the CFD shows no lift peak up to 15 deg), "
            + (f"and the blend sharpness M = {m_blend:g} 1/rad is a numerical parameter that also "
               f"sets C_L,max ({clmax:.3f}, at alpha_w = {pk:.2f} deg)" if ok(m_blend) and ok(clmax)
               and ok(pk) else "")
            + ": the size of the stall effect, and so the enforced result, is provisional.")
    if p.get("rotor_constant_coeffs") is True:
        bits.append("The rotor model has constant thrust and torque coefficients and no in-plane "
                    "force (CT1, CT2, CQ1, CQ2, CH1 and k_mu are zero in the register): thrust "
                    "does not change with airspeed and the rotors add no drag in forward flight.")
    if not bits:
        return ""
    return panelbox("What this result rests on", " ".join(bits), "")


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
    held = params[opt].get("alpha_enforced")
    lim_deg = num(params[opt].get("alpha_limit_deg"))
    if held is True and ok(lim_deg):
        o.items.insert(0, Item("Configuration", f"enforced: |alpha_w| <= {lim_deg:.2f} deg", "",
                               f"C_params.alpha_limit_deg[{opt}]"))
    elif held is False:
        o.items.insert(0, Item("Configuration", "unconstrained", "",
                               f"C_params.alpha_enforced[{opt}]",
                               note="the model ignores the stall"))
        o.flags.append("C ignores the stall: " + UNCONSTRAINED)
    for idx, tag in ((opt, "optimum"), (dfl, "ArduPilot default")):
        if idx is None:
            continue
        for lab, col, unit in (
            ("Peak |alpha_w|, worst member", "stall_alpha_peak_deg_max", "deg"),
            ("Time above the C_L peak angle, worst member", "stall_t_above_peak_s_max", "s"),
            ("Time above the wing limit, worst member", "stall_t_above_limit_s_max", "s"),
            ("Least wing share of the weight", "stall_wing_share_min", ""),
        ):
            it = c.item(f"{lab} ({tag})", "C_params", col, idx, unit,
                        note=LIMIT_NOTE if col == "stall_t_above_limit_s_max" else "")
            if it:
                o.items.append(it)

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

    ref_params = c.rows("C_ref_params")
    for fig in (caveats_panel(params[opt]), configurations_figure(params, opt, ref_params),
                exposure_figure(params, ref_params), alpha_curve_figure(c.rows("C_alpha_curve"))):
        if fig:
            o.figures.append(fig)

    # 2. time histories
    build_c_histories(c, o)

    mf = margin_figure(curve, params, opt, runs)
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
            ("stall_t_above_peak_s", "time above the C_L peak [s]"),
            ("stall_wing_share_min", "least wing share of the weight"),
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
        ("C_alpha_curve", None),
        ("C_ref_params", None),
        ("C_ref_runs", None),
        ("C_ref_history", None),
    ]
    return o


# ------------------------------------------------------------------------------------------ B1
LATE_WARNING = "verified for exact timing only: fails with commands 10 ms late"
VARIANT_ORDER = ("margin", "attached", "c_envelope")
VARIANT_CLS = {"margin": "s4", "attached": "s2", "c_envelope": "s3"}  # colour follows the variant
VARIANT_ROLE = {"margin": "flyable bound", "attached": "upper bound at the lift peak",
                "c_envelope": "upper bound at the lift peak"}  # fmt: skip


def variants_in_order(names: Sequence[Any]) -> list[Any]:
    """The variants present, the flyable one first, any other (a config's own) after them."""
    rank = {v: i for i, v in enumerate(VARIANT_ORDER)}
    return sorted(uniq(list(names)), key=lambda v: rank.get(str(v), len(rank)))


REASONS = ("verified", "not converged", "replay mismatch", "fails 10 ms late")


def b1_reason(r: Row) -> str:
    """Why a B1 point is or is not quoted: the `reason` column, or derived from the flags."""
    if r.get("reason"):
        return str(r["reason"])
    if r.get("converged") is not True:
        return "not converged"
    if r.get("replay_verified") is not True:
        return "replay mismatch"
    if r.get("replay_late_command_ok") is False:
        return "fails 10 ms late"
    return "verified"


def reasons_fold(front: Sequence[Row]) -> str:
    """A collapsible table: points by scenario, variant and reason (all of B1_front)."""
    cnt: dict[tuple[str, str], dict[str, int]] = {}
    for r in front:
        key = (str(r.get("scenario")), str(r.get("variant")))
        cnt.setdefault(key, dict.fromkeys(REASONS, 0))[b1_reason(r)] += 1
    rows = [{"scenario": s, "variant": v, **c, "points": sum(c.values())}
            for (s, v), c in sorted(cnt.items())]  # fmt: skip
    return details(
        "Why points are not quoted: count by scenario, variant and reason",
        table_html("B1_front_reasons", ["scenario", "variant", *REASONS, "points"], rows))


def verified_figure(front: Sequence[Row], gap: Sequence[Row]) -> str | None:
    """One panel per scenario: the verified B1 points per variant and, where B1_gap has one, C's
    run of THAT scenario (at the register parameters with q = 2). A panel with three or more
    verified points of a variant draws them as a front; a panel with none says so."""
    matched: dict[str, Row] = {}
    for g in gap:
        if (g.get("b1_replay_verified") is True and ok(num(g.get("c_energy_kJ")))
                and ok(num(g.get("c_distance_m")))):  # fmt: skip
            matched.setdefault(g["scenario"], g)
    panels, leg, fronts, with_points = [], {}, 0, 0
    for s in uniq([r["scenario"] for r in front]):
        series, most = [], 0
        for v in variants_in_order([r.get("variant") for r in front]):
            rs = sorted([r for r in front if r["scenario"] == s and r.get("variant") == v
                         and r.get("replay_verified") is True and ok(num(r.get("energy_kJ")))],
                        key=lambda r: num(r.get("distance_m")))  # fmt: skip
            most = max(most, len(rs))
            if rs:
                cls = VARIANT_CLS.get(v, "ink")
                series.append(ch.Series(
                    f"B1 {v}", cls, [(r["distance_m"], r["energy_kJ"]) for r in rs],
                    [tip(r, ["eps_x_m", "energy_kJ", "distance_m", "excursion_m", "t_f_s"],
                         f"B1 {v}") for r in rs],
                    line=len(rs) >= 3))  # fmt: skip
                leg[v] = (cls, "line", f"B1 {v}")
        mc = matched.get(s)
        notes = ""
        if mc is not None:
            series.append(ch.Series(
                "C, this scenario", "s1", [(num(mc["c_distance_m"]), num(mc["c_energy_kJ"]))],
                [tip(mc, ["c_energy_kJ", "c_distance_m", "c_source"], f"C, {s}")],
                line=False, marker="dia"))
            leg["C"] = ("s1", "dia", "C, the same scenario (register parameters, q = 2)")
        else:
            notes = absent("no scenario-matched C run")
        if most:
            with_points += 1
            fronts += most >= 3
            panels.append((f"{s} (front)" if most >= 3 else s,
                           ch.xy_chart(series, "distance [m]", "bus energy [kJ]", f"Verified {s}",
                                       w=340, h=280) + notes))  # fmt: skip
        else:
            panels.append((s, absent("no verified solution") + notes))
    if not panels:
        return None
    title = ("Pareto front by scenario" if with_points and fronts == with_points
             else "Verified solutions by scenario")
    return panelbox(
        title,
        "Least energy against the distance cap per wing-limit variant, verified points only; "
        "a panel with three or more verified points of a variant draws them as a front. The C "
        "point is C's run of the same scenario at the register parameters with q = 2, drawn "
        "where B1_gap has one.",
        ch.multiples(panels), _legend(*leg.values()))


def _verified_gap(r: Row) -> bool:
    return (r.get("b1_replay_verified") is True and ok(num(r.get("b1_energy_kJ")))
            and ok(num(r.get("c_energy_kJ"))))  # fmt: skip


def limit_text(front: Sequence[Row], variant: str) -> str:
    """The wing limit of a variant in words, from the B1_front columns (no number if absent)."""
    lim = next((num(r["alpha_limit_deg"]) for r in front if r.get("variant") == variant
                and ok(num(r.get("alpha_limit_deg")))), NAN)  # fmt: skip
    return f"|alpha_w| up to {lim:.2f} deg" if ok(lim) else "its wing limit"


def gap_contents(front: Sequence[Row], gap: Sequence[Row], c_row: Row | None) -> str:
    """What "B1 below C" contains, written from the computed rows (the numbers are the files').

    C and B1 `margin` fly the same wing limit and the same pitch ceiling, so the pitch freedom is
    already removed from the gap between them; what is left is B1's missing rotor lag and its
    clairvoyance (it knows the scenario and is open loop, where C's four settings serve every
    ensemble member). `attached` and `c_envelope` add the lift peak: a wing limit above the margin
    (and, for `attached`, a pitch command above C's ceiling). C without the wing cap is a reference
    that ignores the stall.
    """
    lag = num(c_row.get("rotor_lag_s")) if c_row else NAN
    ptch = num(c_row.get("ptch_lim_deg")) if c_row else NAN
    lim = num(c_row.get("alpha_limit_deg")) if c_row else NAN
    held = bool(c_row and c_row.get("alpha_enforced") is True)
    out: list[str] = []
    for s in uniq([r.get("scenario") for r in gap]):
        rows = {str(r.get("variant")): r for r in gap if r.get("scenario") == s}
        base = next((r for r in rows.values() if ok(num(r.get("c_energy_kJ")))), None)
        if base is None:
            continue
        c_e, c_d = num(base["c_energy_kJ"]), num(base.get("c_distance_m"))
        txt = (f"{s}: C (register parameters, q = 2" + (f", wing held to {lim:.2f} deg" if held
               and ok(lim) else ", wing not held") + f") needs {c_e:.1f} kJ over "
               f"{c_d:.0f} m. ")
        e_m = num(rows["margin"].get("b1_energy_kJ")) if "margin" in rows and _verified_gap(
            rows["margin"]) else NAN
        if ok(e_m):
            txt += (f"B1 `margin` needs {e_m:.1f} kJ ({(1 - e_m / c_e) * 100:.0f} % less): it flies "
                    "the same wing limit and the same pitch ceiling"
                    + (f" ({ptch:g} deg)" if ok(ptch) else "")
                    + ", so the pitch freedom is already removed from that saving; what is left is "
                    "B1's missing rotor lag" + (f" (C has {lag:g} s)" if ok(lag) else "")
                    + " and its clairvoyance (B1 knows the scenario and is open loop, where C's four "
                    "settings serve every ensemble member). ")
        elif "margin" in rows:
            txt += "B1 `margin` has no verified point here, so nothing is said about it. "
        for v in ("attached", "c_envelope"):
            r = rows.get(v)
            if r is not None and not _verified_gap(r):
                txt += f"B1 `{v}` has no verified point here, so nothing is said about it. "
            if r is not None and _verified_gap(r):
                e_v = num(r["b1_energy_kJ"])
                step = f"{e_v - e_m:+.1f} kJ against `margin`, " if ok(e_m) else ""
                txt += (f"B1 `{v}` needs {e_v:.1f} kJ ({step}{(1 - e_v / c_e) * 100:.0f} % below C): "
                        f"it adds operation at the lift peak ({limit_text(front, v)}"
                        + (", and a pitch command above C's ceiling" if v == "attached" else "")
                        + "). ")
        e_u = num(base.get("c_unconstrained_energy_kJ"))
        if ok(e_u):
            txt += (f"C without the wing cap needs {e_u:.1f} kJ over "
                    f"{num(base.get('c_unconstrained_distance_m')):.0f} m because the model ignores "
                    "the stall: a reference, not a competitor.")
        out.append(txt.strip())
    if not out:
        return panelbox("What the gap between B1 and C contains",
                        "No scenario has both a C run and a verified B1 solution.", "")
    out.append("So the B1-C gap is not what a better ArduPilot parameter set could save: it is the "
               "sum of the missing rotor lag, the clairvoyance and, for `attached` and "
               "`c_envelope`, the lift-peak operation.")
    return panelbox("What the gap between B1 and C contains", " ".join(out), "")


def gap_ladder_figure(front: Sequence[Row], gap: Sequence[Row]) -> str | None:
    """The gap ladder, one panel per scenario: C (wing held), B1 `margin`, `attached` and
    `c_envelope` at C's own distance, verified points only, and C without the wing cap as a hatched
    reference. A scenario without a C run draws nothing and says so."""
    panels = []
    for s in uniq([r.get("scenario") for r in front]):
        rows = {str(r.get("variant")): r for r in gap if r.get("scenario") == s}
        base = next((r for r in rows.values() if ok(num(r.get("c_energy_kJ")))), None)
        if base is None:
            panels.append((str(s), absent("no scenario-matched C run: nothing to compare with")))
            continue
        held = base.get("c_wing_held") is True
        c_e = num(base["c_energy_kJ"])
        items = [("C" + (" (wing held)" if held else ""), c_e,
                  tip(base, ["c_energy_kJ", "c_distance_m", "c_source"], f"C, {s}"), "s1")]
        for v in variants_in_order(list(rows)):
            r = rows[str(v)]
            if _verified_gap(r):
                items.append((f"B1 {v}", num(r["b1_energy_kJ"]),
                              tip(r, ["b1_energy_kJ", "b1_distance_m", "energy_margin"],
                                  f"B1 {v}, {s}"), VARIANT_CLS.get(str(v), "ink")))
        hatched = []
        e_u = num(base.get("c_unconstrained_energy_kJ"))
        if ok(e_u):
            hatched = [len(items)]
            items.append(("C without the cap (ref.)", e_u,
                          tip(base, ["c_unconstrained_energy_kJ", "c_unconstrained_distance_m"],
                              "C unconstrained: the model ignores the stall; reference only"),
                          "ref"))
        steps = []
        for it in items[1:]:
            name = it[0]
            delta = it[1] - c_e
            if name == "B1 margin":
                steps.append(f"{name}: {delta:+.1f} kJ, the missing rotor lag and the clairvoyance "
                             "(the pitch freedom is already removed)")
            elif name.startswith("B1 "):
                steps.append(f"{name}: {delta:+.1f} kJ, adds lift-peak operation")
        steps += [f"B1 {v}: no verified point here" for v in variants_in_order(list(rows))
                  if not _verified_gap(rows[str(v)])]
        note = f'<p class="cap">{ch.esc("; ".join(steps))}.</p>' if steps else ""
        panels.append((str(s), ch.bar_chart(items, "bus energy [kJ]", f"Gap ladder {s}", w=470,
                                            hatched=hatched) + note))
    if not panels:
        return None
    return panelbox(
        "Gap ladder: C, B1 margin, attached, c_envelope",
        "Energy of the same problem at C's own distance (C at the register parameters with q = 2), "
        "verified B1 points only. The hatched bar is C without the wing cap: it ignores the stall, "
        "is a reference and never a recommendation.",
        ch.multiples(panels, 1),
        _legend(("s1", "line", "C, wing held"), ("s4", "line", "B1 margin (flyable bound)"),
                ("s2", "line", "B1 attached (upper bound at the lift peak)"),
                ("s3", "line", "B1 c_envelope (upper bound at the lift peak)"),
                ("ref", "dash", "C without the cap (reference only)")))


def attached_statement(front: Sequence[Row], variant: str = "attached") -> str:
    """Is `attached` attached? Where the wing limit of `variant` sits against the peak of the
    model's C_L, in the same angle convention (alpha_w, with the wing incidence), from the B1_front
    columns."""
    rows = [r for r in front if r.get("variant") == variant and ok(num(r.get("alpha_limit_deg")))
            and ok(num(r.get("alpha_w_clmax_deg")))]  # fmt: skip
    if not rows:
        return ""
    r = rows[0]
    lim, pk, a_s = num(r["alpha_limit_deg"]), num(r["alpha_w_clmax_deg"]), num(r.get("alpha_s_deg"))
    clmax, frac = num(r.get("cl_max")), num(r.get("cl_at_limit_over_max"))
    below = pk - lim
    text = (f"The {variant} limit is |alpha_w| <= {lim:.2f} deg"
            + (f" ({lim / a_s:.2f} x alpha_s, alpha_s = {a_s:.2f} deg)" if ok(a_s) and a_s else "")
            + f". The model's C_L peaks at alpha_w = {pk:.2f} deg"
            + (f" (C_L,max {clmax:.3f})" if ok(clmax) else "") + ", in the same convention. ")
    if below >= 0.0:
        text += f"The limit sits {below:.2f} deg below that peak"
        text += f", where C_L is {frac:.2%} of C_L,max" if ok(frac) else ""
        text += (". It leaves no stall margin in lift: the wing is flown at the top of the lift "
                 "curve, not below it" if below < 1.0 else
                 ". That is a margin in angle before the lift peak")
    else:
        text += f"The limit sits {-below:.2f} deg ABOVE that peak, in the stalled region"
    peaks = [num(x.get("alpha_w_peak_above_vs_deg")) for x in front
             if x.get("variant") == variant and x.get("replay_verified") is True
             and ok(num(x.get("alpha_w_peak_above_vs_deg")))]  # fmt: skip
    if peaks:
        span = (f"{min(peaks):.2f} deg" if f"{min(peaks):.2f}" == f"{max(peaks):.2f}"
                else f"{min(peaks):.2f} to {max(peaks):.2f} deg")
        text += f". The replayed peak wing angle of the verified points is {span}"
    return text + "."


def nominal_of(r: Row, front: Sequence[Row], head_eps: float) -> tuple[Row | None, str]:
    """The nominal point of a sensitivity row: the same scenario, variant and distance cap with
    the setting unchanged (the headline altitude band, the default rotor start and end band).
    Returns (the verified B1_front row or None, why there is none: "" | "unverified" | "none")."""
    cands = [f for f in front if f.get("scenario") == r.get("scenario")
             and f.get("variant") == r.get("variant") and f.get("eps_x_m") == r.get("eps_x_m")
             and f.get("eps_h_m") == head_eps]  # fmt: skip
    for f in cands:
        if f.get("replay_verified") is True and ok(num(f.get("energy_kJ"))):
            return f, ""
    return None, ("unverified" if cands else "none")


def b1_sensitivity_figure(sens: Sequence[Row], front: Sequence[Row]) -> str | None:
    """B1's energy when one setting changes, with the nominal energy as a hollow marker on each
    row. The row label names the scenario and the variant (the band sweep uses `c_envelope`); a row
    whose nominal point is not verified says so."""
    head = next((num(f["eps_h_m"]) for f in front if ok(num(f.get("eps_h_m")))), NAN)
    items, refs = [], []
    for r in sens:
        if not ok(num(r.get("energy_kJ"))):
            continue
        nom, why = nominal_of(r, front, head)
        label = f"{r.get('sensitivity')} = {r.get('value')} ({r.get('scenario')}, {r.get('variant')})"
        if why:
            label += " [nominal unverified]" if why == "unverified" else " [no nominal point]"
        items.append((label, num(r["energy_kJ"]), tip(r, ["energy_kJ", "distance_m", "replay_verified"]),
                      "s2"))  # fmt: skip
        refs.append((num(nom["energy_kJ"]), f"nominal: {num(nom['energy_kJ']):.2f} kJ (the setting "
                     f"unchanged; {nom.get('scenario')}, {nom.get('variant')}, cap "
                     f"{nom.get('eps_x_m'):g} m)") if nom is not None else None)  # fmt: skip
    if not items:
        return None
    swept = {str(r.get("variant")) for r in sens}
    unswept = [v for v in variants_in_order([r.get("variant") for r in front])
               if str(v) not in swept]
    skipped = (" The variant" + ("s " if len(unswept) > 1 else " ")
               + ", ".join(f"`{v}`" for v in unswept) + " is not swept here." if unswept else "")
    leg = _legend(("s2", "dot", "B1 with the setting changed"),
                  ("ref", "hollow", "nominal: the same scenario and cap, setting unchanged"))
    return panelbox(
        "Sensitivity",
        "Energy of B1 when one setting changes; the hollow marker on each row is the nominal "
        "energy of that row's scenario, variant and distance cap. A row marked [nominal "
        "unverified] has no verified nominal point to compare with." + skipped,
        ch.dot_chart(items, "bus energy [kJ]", "B1 sensitivity", zero=False, refs=refs), leg)


def gap_items(gap: Sequence[Row]) -> list[Item]:
    """The tiles of the B1-C gap, variant by variant and never averaged or picked across them:
    `margin` first ("flyable bound"; a scenario where it has no verified point says so and quotes
    nothing), then `attached` and `c_envelope` ("upper bounds at the lift peak"; `c_envelope` with
    its warning when its schedule fails a late command).

    Read from the existing `B1_gap` rows; the energy tiles are those of the first verified row.
    """
    items: list[Item] = []
    first = True
    for v in variants_in_order([r.get("variant") for r in gap]):
        role = VARIANT_ROLE.get(str(v), "")
        for gi, r in enumerate(gap):
            if r.get("variant") != v:
                continue
            label = f"B1 below C ({r['scenario']}, {v})"
            if not (r.get("b1_replay_verified") is True and ok(num(r.get("b1_energy_kJ")))
                    and ok(num(r.get("c_energy_kJ")))):  # fmt: skip
                if v == "margin":
                    items.append(Item(label, "no verified solution", "",
                                      f"B1_gap.b1_replay_verified[{gi}]",
                                      note=f"{role}: nothing is quoted for it here"))
                continue
            c_e, b_e = num(r["c_energy_kJ"]), num(r["b1_energy_kJ"])
            warn = LATE_WARNING if r.get("b1_late_command_ok") is False else ""
            if first:
                first = False
                for lab, col in (("C energy", "c_energy_kJ"), ("B1 energy", "b1_energy_kJ")):
                    items.append(Item(f"{lab} ({r['scenario']}, {r['variant']})",
                                      f"{num(r[col]):.2f}", "kJ", f"B1_gap.{col}[{gi}]"))
            if c_e:
                items.append(Item(
                    label, f"{(1 - b_e / c_e) * 100:.0f} %", "",
                    f"B1_gap.b1_energy_kJ[{gi}]; B1_gap.c_energy_kJ[{gi}]",
                    f"1 - {b_e:.2f}/{c_e:.2f}", ". ".join(x for x in (role, warn) if x)))
    return items


def build_b1(c: Ctx) -> Out:
    o = Out()
    front, gap = c.rows("B1_front"), c.rows("B1_gap")
    if gap:
        o.items += gap_items(gap)
        c_row = next((r for r in c.rows("C_params") if r.get("case") == "optimum"), None)
        o.figures.append(gap_contents(front, gap, c_row))
        ladder = gap_ladder_figure(front, gap)
        if ladder:
            o.figures.append(ladder)
        mar = attached_statement(front, "margin")
        if mar:
            o.figures.append(panelbox('Is "margin" flyable?', mar, ""))
        att = attached_statement(front)
        if att:
            o.figures.append(panelbox('Is "attached" attached?', att, ""))
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
        o.figures.append(reasons_fold(front))
        fig = verified_figure(front, gap)
        if fig:
            o.figures.append(fig)
    if gap:
        items = []
        for r in sorted(gap, key=lambda g: variants_in_order(
                [x.get("variant") for x in gap]).index(g.get("variant"))):
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
                cls = VARIANT_CLS.get(r["variant"], "ink")
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
                        ("s4", "line", "B1 margin"),
                        ("s2", "line", "B1 attached"),
                        ("s3", "line", "B1 c_envelope"),
                    ),
                )
            )
    sched = c.rows("B1_schedule")
    if sched:
        ts = []
        for v in variants_in_order([r.get("variant") for r in sched]):
            rs = [r for r in sched if r.get("variant") == v]
            cls = VARIANT_CLS.get(v, "ink")
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
                VARIANT_CLS.get(v, "ink"),
                f"B1 {v}",
                False,
                True,
            )
            for v in variants_in_order(
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
        sf = b1_sensitivity_figure(sens, front)
        if sf:
            o.figures.append(sf)
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


FT_AXES = (("altitude_m", "altitude", "m", ".0f"), ("isa_offset_k", "ISA offset", "K", "+.0f"),
           ("mass_factor", "mass factor", "", ".2f"), ("soc", "SoC", "", ".2f"),
           ("headwind_ms", "headwind", "m/s", "+.0f"))  # fmt: skip
FT_CASE_NAMES = {"optimum": "optimum", "register": "register", "ardupilot_default": "ArduPilot default"}


def heaviest_stall(runs: Sequence[Row], margin: float) -> tuple[float, str] | None:
    """The stall rule: `margin` x the stall speed of the HEAVIEST member (the largest V_s_eas_ms),
    the member that binds, not the mean of the members."""
    vs = [num(r.get("V_s_eas_ms")) for r in runs if ok(num(r.get("V_s_eas_ms")))]
    if not vs:
        return None
    return margin * max(vs), f"{margin:g} x stall speed (heaviest member)"


def ft_scenario_caption(name: str, info: dict[str, Any] | None, axes: dict[str, Any],
                     hist: Sequence[Row], runs: Sequence[Row]) -> str:  # fmt: skip
    """Which scenario a time-history figure is, why, and where it lies against the ensemble's
    range on each axis (all computed from the run record), then what the default loses there."""
    if not info:
        return f"Scenario {name}."
    parts = [f"{lab} {format(info[k], f)}{' ' + u if u else ''}" for k, lab, u, f in FT_AXES
             if isinstance(info.get(k), int | float)]  # fmt: skip
    text = f"Scenario {name}: " + ", ".join(parts)
    text += f" ({info['note']})." if info.get("note") else "."
    out, edge = [], []
    for k, lab, _, _ in FT_AXES:
        rng, v = axes.get(k), info.get(k)
        if not (isinstance(v, int | float) and isinstance(rng, list) and len(rng) == 2):
            continue
        lo, hi = float(rng[0]), float(rng[1])
        span = (hi - lo) or 1.0
        if v < lo - 1e-9 or v > hi + 1e-9:
            out.append(f"{lab} ({v:g} against {lo:g} to {hi:g})")
        elif min(v - lo, hi - v) <= 0.02 * span:
            edge.append(lab)
    if out:
        text += " It lies OUTSIDE the ensemble's range on " + ", ".join(out) + "."
    else:
        text += " It lies inside the ensemble's range on every axis"
        text += (", at the edge of the range on " + ", ".join(edge)) if edge else ""
        text += "."
    dh = [num(r.get("h_rel")) for r in hist if r.get("case") == "ardupilot_default"
          and r.get("scenario") == name and ok(num(r.get("h_rel")))]  # fmt: skip
    dl = [num(r.get("loss_m")) for r in runs if r.get("case") == "ardupilot_default"
          and ok(num(r.get("loss_m")))]  # fmt: skip
    if dh and dl:
        text += (f" The ArduPilot default loses {max(0.0, -min(dh)):.1f} m of altitude here; "
                 f"its ensemble members lose at most {max(dl):.1f} m.")
    return text


def robust_statement(robust: Sequence[Row], opt: Row | None, rob: Row | None) -> str:
    """One computed sentence on FT_robust: where the smallest AIRSPEED_MIN that is flag-free at
    every C_L,max scale sits against the optimum's, and at which scales the optimum is clean."""
    if not robust or opt is None or not ok(num(opt.get("airspeed_min_eas_ms"))):
        return ""
    scales = sorted({num(r.get("cl_max_scale")) for r in robust if ok(num(r.get("cl_max_scale")))})
    v_opt = num(opt["airspeed_min_eas_ms"])
    lab = ", ".join(f"{s:g}" for s in scales)
    clean_opt = [s for s in scales if all(
        not num(r.get("n_flags")) for r in robust if abs(num(r.get("airspeed_min_eas_ms")) - v_opt)
        < 1e-9 and abs(num(r.get("cl_max_scale")) - s) < 1e-9)]  # fmt: skip
    if rob is None or rob.get("robust_found") is not True:
        return (f"No AIRSPEED_MIN in the table is flag-free at every C_L,max scale ({lab}); the "
                f"optimum's {v_opt:g} m/s is flag-free at "
                + (", ".join(f"{s:g}" for s in clean_opt) if clean_opt else "none of them") + ".")
    v_rob = num(rob["airspeed_min_eas_ms"])
    if abs(v_rob - v_opt) < 1e-9:
        return (f"AIRSPEED_MIN {v_rob:g} m/s, the optimum, is flag-free at all {len(scales)} "
                f"C_L,max scales ({lab}); it is also the smallest value in the table that is.")
    text = (f"AIRSPEED_MIN {v_rob:g} m/s is flag-free at all {len(scales)} C_L,max scales ({lab}); "
            f"the optimum's {v_opt:g} m/s is flag-free only at "
            + (", ".join(f"{s:g}" for s in clean_opt) if clean_opt else "none of them") + ".")
    dj, sp = num(rob.get("delta_J")), num(rob.get("delta_J_in_seed_spreads"))
    if ok(dj):
        text += (f" Its cost J is {num(rob.get('J')):.3f}, {dj:+.3f} against the optimum on the same "
                 "ensemble" + (f" ({sp:+.1f} seed spreads)" if ok(sp) else "") + ".")
    return text + " It is reported, not recommended: the owner decides."


def robust_figure(robust: Sequence[Row], opt: Row | None, rob: Row | None) -> str | None:
    if not robust:
        return None
    cols = ["airspeed_min_eas_ms", "cl_max_scale", "J", "worst_stall_margin", "n_flags", "flags"]
    shown = list(robust)
    return panelbox(
        "Stall margin robustness",
        robust_statement(robust, opt, rob)
        + " The table moves AIRSPEED_MIN around the optimum with the other three settings held, "
        "each at a scaled C_L,max (the model's value is not validated); the same ensemble as the "
        "sensitivity, no search.",
        table_html("FT_robust", [c for c in cols if any(c in r for r in shown)], shown))


def median(vals: Sequence[float]) -> float:
    v = sorted(x for x in vals if ok(x))
    return NAN if not v else (v[len(v) // 2] if len(v) % 2 else 0.5 * (v[len(v) // 2 - 1] + v[len(v) // 2]))


def ft_bound_reason(row: Row, bound: str) -> str:
    """Why TKOFF_THR_MAX sits on its upper bound: 100 % is the physical limit; THR_MAX from the
    register, both computed from the FT_params columns."""
    hi, thr = num(row.get("tkoff_thr_max_hi")), num(row.get("thr_max"))
    if "tkoff_thr_max:hi" not in bound or not ok(hi):
        return ""
    text = f" (the upper bound of TKOFF_THR_MAX is {hi:.0%}, the most throttle the pusher has"
    text += f"; THR_MAX in the register is {thr:.0%}" if ok(thr) else ""
    return text + ", so the search asks for all of it)"


def grid_statement(grid: Sequence[Row], params: Sequence[Row], spread: float) -> str:
    """Cost against Q_TRANSITION_MS in words, from FT_grid: the range of J over the grid, J at the
    register's Q_TRANSITION_MS against the winner's, the plateau within the seed spread of the
    best, and whether the fade length is a second-order choice against re-tuning from the start."""
    pts = [r for r in grid if ok(num(r.get("J_snapped")))]
    if len(pts) < 2:
        return ""
    best = min(pts, key=lambda r: r["J_snapped"])
    worst = max(pts, key=lambda r: r["J_snapped"])
    jb, jw = float(best["J_snapped"]), float(worst["J_snapped"])
    text = (f"Over the {len(pts)} grid points J runs from {jb:.3f} (Q_TRANSITION_MS "
            f"{best['q_transition_s']:g} s) to {jw:.3f} ({worst['q_transition_s']:g} s), "
            f"{jw / jb - 1:.1%} above the lowest.")
    reg = next((p for p in params if p.get("case") == "register"), None)
    qr = num(reg.get("q_transition_s")) if reg else NAN
    at_reg = next((r for r in pts if ok(qr) and abs(r["q_transition_s"] - qr) < 1e-9), None)
    if at_reg is not None:
        text += (f" At the register's Q_TRANSITION_MS ({qr:g} s) J is {at_reg['J_snapped']:.3f}, "
                 f"{at_reg['J_snapped'] / jb - 1:+.1%} against the winner's.")
    elif ok(qr):
        text += f" The register's Q_TRANSITION_MS ({qr:g} s) is not on the grid."
    if ok(spread) and spread > 0:
        near = [r["q_transition_s"] for r in pts if r["J_snapped"] <= jb * (1 + spread)]
        text += (f" {len(near)} of {len(pts)} points lie within the seed spread ({spread:.2%}) of "
                 "the best: Q_TRANSITION_MS " + ", ".join(f"{q:g}" for q in near) + " s.")
    opt = next((p for p in params if p.get("case") == "optimum"), None)
    j0, j1 = (num(opt.get("J_start")), num(opt.get("J"))) if opt else (NAN, NAN)
    if ok(j0) and ok(j1) and j0 > j1:
        ratio = (jw - jb) / (j0 - j1)
        text += (f" The whole range is {ratio:.1%} of what re-tuning all four settings gains from the "
                 f"register start (J {j0:.2f} to {j1:.3f}), so with the other three re-tuned the "
                 "fade length is "
                 + ("a second-order choice." if ratio < 0.1 else "not a second-order choice."))
    return text


def sensitivity_statement(rows: Sequence[Row], j0: float, spread: float) -> str:
    """Does any input apart from C_L,max move J by more than the seed spread? Computed."""
    if not (ok(j0) and j0 and ok(spread) and spread > 0):
        return ""
    lim = spread * j0
    moves = [(r, float(r["J"]) - j0) for r in rows if ok(num(r.get("J")))]
    other = [(r, d) for r, d in moves if r.get("parameter") != "CL_max_scale"]
    if not other:
        return ""
    over = [(r, d) for r, d in other if abs(d) > lim]
    head = f"The seed spread is {spread:.2%} of J, {lim:.4f} in J. "
    big = max(other, key=lambda m: abs(m[1]))
    if not over:
        return (head + "Apart from C_L,max, no input moves J by more than that; the largest is "
                f"{big[0]['parameter']} = {big[0]['value']} at {big[1]:+.4f}.")
    names = ", ".join(f"{r['parameter']} = {r['value']} ({d:+.3f}, {int(num(r.get('n_flags')) or 0)}"
                      " failing)" for r, d in over)
    return head + f"Apart from C_L,max, these inputs DO move J by more than that: {names}."


def share_statement(dmem: Sequence[Row], opt: Sequence[Row], reg: Sequence[Row]) -> str:
    """Computed: the share of default members below stall margin 1.0 and 1.2, and the optimum's
    energy against the default's and the register's medians (the energy cost of the optimum)."""
    parts = []
    marg = [num(r.get("stall_margin_min")) for r in dmem if ok(num(r.get("stall_margin_min")))]
    if marg:
        n1, n12 = sum(m < 1.0 for m in marg), sum(m < 1.2 for m in marg)
        parts.append(f"{n1} of {len(marg)} ArduPilot-default members ({n1 / len(marg):.0%}) fly "
                     f"below a stall margin of 1.0 and {n12} ({n12 / len(marg):.0%}) below 1.2.")
    e = {k: median([num(r.get("energy_kJ")) for r in rs])
         for k, rs in (("optimum", opt), ("default", dmem), ("register", reg))}
    cmp = [f"{(e['optimum'] / e[k] - 1):+.1%} against the {k}'s {e[k]:.2f} kJ"
           for k in ("default", "register") if ok(e[k]) and e[k] and ok(e["optimum"])]
    if cmp:
        parts.append(f"The optimum's median energy is {e['optimum']:.2f} kJ: " + " and ".join(cmp)
                     + " (the median over members; the energy cost of the recommendation).")
    return " ".join(parts)


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
        o.flags.append("at a search bound: " + bound + ft_bound_reason(params[opt], bound))

    stall = c.study("FT", "stall_margin", 1.2)
    wsm = num(params[opt].get("worst_stall_margin"))
    if ok(wsm) and stall:
        o.items.append(Item("Stall margin spare", f"{wsm:.3f} vs {stall:g} ({wsm / stall - 1:.1%})",
                            "", f"FT_params.worst_stall_margin[{opt}] / FT.stall_margin"))
    loss = c.context.get("mission", {}).get("fwd_alt_loss_max_m")
    gain = c.context.get("mission", {}).get("fwd_alt_gain_max_m")
    case_leg = [
        (FT_CASES[k][0], "dash" if FT_CASES[k][2] else "line", FT_CASES[k][1])
        for k in FT_CASES
        if k in row
    ]
    # 1. time histories, one figure per scenario
    ftx = c.context.get("ft_scenarios", {})
    for s in uniq([r.get("scenario") for r in hist])[:2]:
        hs = [r for r in hist if r.get("scenario") == s]
        ts = hist_series(
            hs,
            {k: v for k, v in FT_CASES.items()},
            ["V_eas", "h_rel", "theta", "delta_r", "delta_p"],
        )
        cap_leg = []
        for cs, (cls, short, _, _) in FT_CASES.items():  # each case's own cap, in its own colour
            capped = [r for r in hs if r.get("case") == cs and ok(num(r.get("theta_cap")))]
            if capped:
                ts.append(ch.TSeries(
                    f"pitch cap ({short})", cls, f"pitch cap ({short})", [r["t"] for r in capped],
                    {"theta": [r["theta_cap"] for r in capped]}, True, True, False))
                cap_leg.append((cls, "dash", f"pitch cap ({short})"))
        for sr in ts:  # each case's own fade start and done, on its airspeed curve
            rs = [r for r in hs if r.get("case") == sr.name]
            for mode, shape, what in (("fwd_fade", "sq", "fade starts"), ("fwd_done", "dia", "done")):
                first = next((r for r in rs if r.get("mode") == mode), None)
                if first is not None:
                    sr.marks.append((first["t"], shape, f"{sr.short}: {what} at {first['t']:.1f} s"))
        modes: list[tuple[float, str]] = []
        last = None
        for r in hs:
            if r.get("case") == "optimum" and r.get("mode") and r["mode"] != last:
                last = r["mode"]
                modes.append((r["t"], str(last).replace("fwd_", "") + " (optimum)"))
        eas_rules = []
        for cs, lab in FT_CASE_NAMES.items():
            pr = next((p for p in params if p.get("case") == cs), None)
            if pr and ok(num(pr.get("airspeed_min_eas_ms"))):
                eas_rules.append((pr["airspeed_min_eas_ms"],
                                  f"AIRSPEED_MIN {lab} ({pr['airspeed_min_eas_ms']:g})"))
        sr_stall = heaviest_stall(runs, stall)
        if sr_stall:
            eas_rules.append(sr_stall)
        band = (
            (-float(loss), float(gain))
            if isinstance(loss, int | float) and isinstance(gain, int | float)
            else None
        )
        panels = [
            ch.Panel("V_eas", "airspeed [m/s, EAS]", "m/s", 2, hlines=eas_rules, vticks=modes[1:],
                     ymin=0.0, marks=True, wide=True),
            ch.Panel("h_rel", "altitude change [m]", "m", 2, True, band=band),
            ch.Panel("theta", "pitch [deg]", "deg", 1),
            ch.Panel("delta_r", "rotor throttle [-]", "", 2),
            ch.Panel("delta_p", "pusher throttle [-]", "", 2),
        ]
        if ts:
            o.figures.append(
                panelbox(
                    f"Time histories, {s}",
                    ft_scenario_caption(s, ftx.get("scenarios", {}).get(s), ftx.get("axes", {}),
                                     hs, runs)
                    + " Optimum, register start and ArduPilot default through the wait, fade and "
                    "done modes; the square on each airspeed curve is where that case's fade "
                    "starts and the diamond where it is done (the vertical lines are the "
                    "optimum's). The shaded band is the accepted altitude change.",
                    ch.panel_figure(f"ft_hist_{s}", panels, ts),
                    _legend(*case_leg, *cap_leg, ("ink", "sq", "fade starts (own, on the curve)"),
                            ("ink", "dia", "done (own, on the curve)")),
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
        xs = [p[0] for s in series for p in s.pts]
        xl = ch.limits(xs, 0.08)
        lim = -float(loss) if isinstance(loss, int | float) else None
        inside = lim is not None and xl[0] <= lim <= xl[1]
        vl = [(lim, "loss limit")] if lim is not None and inside else []
        outside = (f" The loss limit ({lim:g} m) is outside the range shown, which runs from "
                   f"{min(xs):.1f} to {max(xs):.1f} m." if lim is not None and not inside else "")
        o.figures.append(
            panelbox(
                "End of the fade",
                "Sink rate at the end of the fade against the projected minimum "
                "altitude change, one mark per member. A hollow mark is a flagged member."
                + outside,
                ch.xy_chart(
                    series,
                    "projected minimum altitude change [m]",
                    "sink rate [m/s] (negative = climbing)",
                    "End of the fade",
                    xlim=xl,
                    vlines=vl,
                ),
                _legend(*case_leg, ("s1", "hollow", "flagged member")),
            )
        )
    # 3. wing share across the hand-over: one strip per case
    hand = [
        r
        for r in runs
        if ok(num(r.get("wing_share_handover"))) and ok(num(r.get("V_handover_eas_ms")))
    ]
    if hand:
        wrows: list[ch.Strip] = []
        for cs, (cls, short, _, _) in FT_CASES.items():
            rs = [r for r in hand if r.get("case") == cs]
            if not rs:
                continue
            pr = next((p for p in params if p.get("case") == cs), None)
            av = f" (AIRSPEED_MIN {pr['airspeed_min_eas_ms']:g})" if pr and ok(
                num(pr.get("airspeed_min_eas_ms"))) else ""
            wrows.append((short + av, [(r["wing_share_handover"],
                                        tip(r, ["set", "member", "V_handover_eas_ms",
                                                "margin_handover", "wing_share_handover"], short),
                                        bool(r.get("flags")), cls) for r in rs], None))  # fmt: skip
        o.figures.append(
            panelbox(
                "Wing share at the hand-over",
                "Share of the weight the wing carries when the pusher takes over (1 would be the "
                "whole weight), one strip per case and one dot per member. Within a case the "
                "hand-over speed is the setting, so the dots differ by the wing share only.",
                ch.strip_chart(wrows, "FT wing share at the hand-over"),
                _legend(*case_leg, ("s1", "hollow", "flagged member")),
            )
        )
    # 3b. the grid over Q_TRANSITION_MS: the cost of the snapped point at each value
    grid = sorted((r for r in c.rows("FT_grid") if ok(num(r.get("J_snapped")))),
                  key=lambda r: r["q_transition_s"])
    sp = c.res.record.get("search_spreads", {}).get("FT")
    spread = float(sp) if isinstance(sp, int | float) else NAN
    if grid:
        gtip = ["q_transition_s", "J_snapped", "J_continuous", "J_snapped_minus_continuous",
                "airspeed_min_eas_ms", "tkoff_thr_max", "tran_pit_max_deg", "seed_spread", "n_flags",
                "flags"]
        gseries = [ch.Series("best snapped J at each Q_TRANSITION_MS", "s1",
                             [(r["q_transition_s"], r["J_snapped"]) for r in grid],
                             [tip(r, gtip) for r in grid],
                             hollow=[bool(num(r.get("n_flags"))) for r in grid],
                             kinds=["sq" if r.get("t12_ok") is False else "dot" for r in grid])]
        wins = [r for r in grid if r.get("winner") is True]
        if wins:
            gseries.append(ch.Series("winner", "s2", [(r["q_transition_s"], r["J_snapped"])
                                                     for r in wins], [tip(r, gtip) for r in wins]))
        failed = [f"{r['q_transition_s']:g} s ({num(r.get('seed_spread')):.1%})" for r in grid
                  if r.get("t12_ok") is False]
        flagged = [f"{r['q_transition_s']:g} s" for r in grid if num(r.get("n_flags"))]
        near = [r["q_transition_s"] for r in grid if r.get("within_2pct") is True]
        near_text = (" Within 2 % of the best cost: Q_TRANSITION_MS "
                     + ", ".join(f"{q:g}" for q in near) + " s." if near else "")
        o.figures.append(
            panelbox(
                "Cost against Q_TRANSITION_MS",
                "The best snapped cost found with Q_TRANSITION_MS fixed at each value (the other "
                "three searched, then snapped to ArduPilot's integers). The winner is marked; a "
                "hollow mark is a point whose snapped optimum still has flagged members"
                + (": " + ", ".join(flagged) if flagged else ", none here")
                + "; a square is a point whose three-seed spread is above the T-12 limit"
                + (": " + ", ".join(failed) if failed else ", none here") + ". "
                + grid_statement(grid, params, spread) + near_text,
                ch.xy_chart(gseries, "Q_TRANSITION_MS [s]", "cost J",
                            "Cost against Q_TRANSITION_MS"),
                _legend(("s1", "line", "grid point"), ("s1", "hollow", "flags remain"),
                        ("s1", "sq", "T-12 spread above limit"), ("s2", "line", "winner")),
            )
        )
    # 3c. the stall margin against the C_L,max scale
    rob = next((r for r in params if r.get("case") == "optimum_robust"), None)
    rfig = robust_figure(c.rows("FT_robust"), params[opt], rob)
    if rfig:
        o.figures.append(rfig)
    # 4. scan
    scan = c.rows("FT_scan")
    if scan:
        rs = sorted(
            (r for r in scan if ok(num(r.get("J"))) and num(r["J"]) > 0),
            key=lambda r: r["airspeed_min_eas_ms"],
        )
        pts = [(r["airspeed_min_eas_ms"], r["J"]) for r in rs]
        series = [ch.Series("best J at each AIRSPEED_MIN", "s1", pts,
                            [tip(r, ["airspeed_min_eas_ms", "J", "q_transition_s", "tkoff_thr_max",
                                     "tran_pit_max_deg", "n_flags", "flags", "seed_spread",
                                     "at_bound"]) for r in rs],
                            hollow=[bool(num(r.get("n_flags"))) for r in rs])]  # fmt: skip
        hv = heaviest_stall(runs, stall)
        vl = [hv] if hv else []
        clean = [f"{r['airspeed_min_eas_ms']:g}" for r in rs if not num(r.get("n_flags"))]
        o.figures.append(
            panelbox(
                "Cost against AIRSPEED_MIN",
                "The best cost found when AIRSPEED_MIN is fixed and the other "
                "three are searched, on a logarithmic axis (zero is not on it; it starts at the "
                "decade below the smallest cost) so neighbouring values can be told apart. A hollow "
                "mark has flagged members, named in its tooltip. Flag-free: "
                + (", ".join(clean) + " m/s." if clean else "none."),
                ch.xy_chart(
                    series,
                    "AIRSPEED_MIN [m/s, EAS]",
                    "cost J (log scale)",
                    "Cost against AIRSPEED_MIN",
                    vlines=vl,
                    ylog=True,
                ),
                _legend(("s1", "line", "scan point"), ("s1", "hollow", "flags remain")),
            )
        )
    # 5. the final cost of each seed and the sensitivity
    srch = c.rows("FT_search")
    if srch:
        ptips = ["airspeed_min_eas_ms", "q_transition_s", "tkoff_thr_max", "tran_pit_max_deg"]
        o.figures.append(
            panelbox(
                "Final cost by seed",
                "The cost each search seed ended at, with that seed's four settings in the "
                "tooltip. It is the end point of the seed, not its path: iteration data is not "
                "stored, so convergence itself is not shown.",
                ch.dot_chart(
                    [
                        (
                            f"seed {r['seed']:g}",
                            r["best_cost"],
                            tip(r, ["seed", "best_cost", "evaluations", *ptips]),
                            "s1",
                        )
                        for r in srch
                    ],
                    "final cost J",
                    "FT final cost by seed",
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
        lin = spread * j0 if ok(spread) and spread > 0 else 0.01 * j0
        items = []
        for r in sens:
            if r is base or not ok(num(r.get("J"))):
                continue
            nf = int(num(r.get("n_flags")) or 0) if ok(num(r.get("n_flags"))) else 0
            items.append((f"{r['parameter']} = {r['value']}" + (f"  [{nf} failing]" if nf else ""),
                          r["J"] - j0, f"J = {r['J']:.4f}; change = {r['J']:.4f} - {j0:.4f}; "
                          f"{nf} failing member(s)", "s1" if nf else "ref"))  # fmt: skip
        o.figures.append(
            panelbox(
                "One-at-a-time sensitivity",
                "Absolute change of the cost J against the base evaluation (J = "
                f"{j0:.4f}) when one setting changes; a row shows its failing-member count. The "
                "axis is linear within the seed spread of zero and logarithmic beyond it, so "
                "the one large row does not flatten the others. "
                + sensitivity_statement(sens, j0, spread),
                ch.dot_chart(items, "change of J (absolute; linear within the seed spread, then "
                             "logarithmic)", "FT sensitivity", sym=lin),
                _legend(("s1", "dot", "some members fail"), ("ref", "dot", "no member fails")),
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
    # 6. spread strips: the optimum, the register start and the ArduPilot default, with the limits
    mine = [r for r in runs if r.get("case") == "optimum"]
    regm = [r for r in runs if r.get("case") == "register"]
    dmem = [r for r in runs if r.get("case") == "ardupilot_default"]
    if mine:
        rows: list[ch.Strip] = []
        extras: list[ch.StripExtra] = []
        lims = {"loss_m": (float(loss), "limit") if isinstance(loss, int | float) else None,
                "gain_m": (float(gain), "limit") if isinstance(gain, int | float) else None,
                "stall_margin_min": (float(stall), f"limit {stall:g}") if stall else None}
        for col, lab in (
            ("loss_m", "altitude loss [m]"),
            ("gain_m", "altitude gain [m]"),
            ("stall_margin_min", "stall margin [-]"),
            ("energy_kJ", "energy [kJ]"),
        ):
            dots = [(num(r.get(col)), tip(r, ["set", "member", col, "flags"], "optimum"),
                     bool(r.get("flags")), "s1") for r in mine if ok(num(r.get(col)))]  # fmt: skip
            dots += [(num(r.get(col)), tip(r, ["set", "member", col, "flags"], "register"),
                      bool(r.get("flags")), "s2") for r in regm if ok(num(r.get(col)))]  # fmt: skip
            dv = sorted(num(r.get(col)) for r in dmem if ok(num(r.get(col))))
            rows.append((lab, dots, dv[len(dv) // 2] if dv else None))
            extras.append(([(num(r.get(col)), tip(r, ["set", "member", col, "flags"],
                                                  "ArduPilot default"))
                            for r in dmem if ok(num(r.get(col)))], lims.get(col)))  # fmt: skip
        leg = [("s1", "dot", "member of the optimum"), ("s2", "dot", "member of the register start"),
               ("hollow", "hollow", "flagged member")]
        if dmem:
            leg += [("ref", "hollow", "member of the ArduPilot default"),
                    ("ref", "line", "ArduPilot default: median of its members")]
        o.figures.append(
            panelbox(
                "Ensemble spread",
                "One strip per metric, one dot per member, spread vertically where they would "
                "overlap; the vertical line is the limit. "
                + share_statement(dmem, mine, regm),
                ch.strip_chart(rows, "FT ensemble spread", extras=extras),
                _legend(*leg),
            )
        )
    o.tables = [
        ("FT_params", None),
        ("FT_scan", None),
        ("FT_runs", None),
        ("FT_search", None),
        ("FT_sens", None),
        ("FT_robust", None),
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
