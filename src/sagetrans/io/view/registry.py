"""The chapter list, and the page: header, settings, chapters, inputs, run record, release gate.

`CHAPTERS` is the ordered list of (id, title, requires, build). A chapter renders if at least one of
its `requires` tables exists; every missing one prints "not in this run: <table>" (docs/OUTPUTS_spec.md
2, rule 3). An older results folder is named as such in the header, and a chapter whose tables it
lacks says so instead of failing.
"""

from __future__ import annotations

import json
from collections.abc import Callable
from dataclasses import dataclass

from sagetrans.io.view import chapters as chp
from sagetrans.io.view.chapters import Ctx
from sagetrans.io.view.charts import esc
from sagetrans.io.view.html import (
    Item,
    Out,
    absent,
    chip,
    details,
    page,
    result_box,
    table_html,
    verbatim,
)
from sagetrans.io.view.load import Results
from sagetrans.io.view.units import fmt_value

# strongest evidence first (the order of config.schema.Status); the inputs table sorts worst first
GRADES = ("measured", "datasheet", "derived", "documented", "stated", "assumed", "TBD")
VIEWER_FILES = ("report.html", "tables.xlsx")  # written by the viewer, not by the study


@dataclass(frozen=True)
class Chapter:
    id: str
    title: str
    question: str
    requires: tuple[str, ...]
    build: Callable[[Ctx], Out]
    analysis: str | None = None  # the `study.yaml` block that governed it


CHAPTERS: tuple[Chapter, ...] = (
    Chapter(
        "a",
        "A · zero-loss feasibility",
        "Which entry speeds and decelerations keep the altitude, and what closes the window?",
        ("A_feasibility", "A_max_feasible_decel", "A_window"),
        chp.build_a,
        "A",
    ),
    Chapter(
        "q2",
        "Q2 · pusher acceleration",
        "Does the pusher give the forward acceleration the energy budget assumes?",
        ("Q2_pusher_margin", "Q2_worst"),
        chp.build_q2,
        "Q2",
    ),
    Chapter(
        "d",
        "D · braking",
        "Which Q_TRANS_DECEL can the aircraft hold across the ensemble?",
        (
            "D_a_eq",
            "D_trade_curve",
            "D_recommendation_ensemble",
            "D_recommendation",
            "D_a_rec_curve",
        ),
        chp.build_d,
        "D",
    ),
    Chapter(
        "f",
        "F · landing descent",
        "How fast can the landing descent go before the vortex-ring state?",
        ("F_descent", "F_profile"),
        chp.build_f,
        "F",
    ),
    Chapter(
        "e",
        "E · energy budget",
        "Does the energy budget close for the missions, with the reserves?",
        ("E_summary", "E_budget", "E_segments", "E_cruise_sweep", "E_hover_check", "E_reserve"),
        chp.build_e,
        "E",
    ),
    Chapter(
        "c",
        "C · restricted back-transition search",
        "Which four ArduPilot settings brake the aircraft from the AIRBRAKE exit at least energy?",
        ("C_params", "C_search", "C_runs", "C_q_curve", "C_sens", "C_history", "C_alpha_curve",
         "C_ref_params", "C_ref_runs", "C_ref_history"),
        chp.build_c,
        "C",
    ),
    Chapter(
        "ft",
        "FT · forward transition",
        "Which four ArduPilot settings take the aircraft from hover to wing-borne flight "
        "without losing altitude or stalling?",
        ("FT_params", "FT_runs", "FT_search", "FT_scan", "FT_sens", "FT_robust", "FT_history"),
        chp.build_ft,
        "FT",
    ),
    Chapter(
        "sens",
        "Sensitivity",
        "Which inputs move the headline outputs, and how well are they known?",
        ("sens_oat", "sens_global"),
        chp.build_sens,
        "sensitivity",
    ),
)
DEFAULTS_CHAPTER = Chapter(
    "defaults",
    "Defaults vs optimised",
    "How far is what ArduPilot ships from what the study found, case by case?",
    ("FT_params", "C_params"),
    chp.build_defaults,
    None,
)


def rank(grade: str) -> int:
    return GRADES.index(grade) if grade in GRADES else len(GRADES)


def reason_absent(res: Results, ch: Chapter) -> str:
    sel = res.record.get("context", {}).get("study")
    if isinstance(sel, dict) and ch.analysis and ch.analysis not in sel:
        return f"analysis {ch.analysis} was not selected for this run"
    if res.legacy:
        return "results written by an older version of the study"
    return (
        "the analysis did not write its tables (it was not run, or it failed; see the run record)"
    )


# ---------------------------------------------------------------------------------- chapters
def _table_block(res: Results, name: str, cols: list[str] | None, inline: bool) -> str:
    t = res.get(name)
    if t is None or not t.rows:
        return ""
    columns = cols or t.columns
    limit = 60 if inline else 300
    body = table_html(name, columns, t.rows, limit=limit)
    n = f"{len(t.rows)} rows, {len(t.columns)} columns"
    link = f' <a href="{esc(t.source)}">{esc(t.source)}</a>' if t.source.endswith(".csv") else ""
    if inline:
        return f'<h3>{esc(name)}</h3><p class="cap">{n}.{link}</p>{body}'
    return details(f"{name} ({n})", body)


def render_chapter(ch: Chapter, c: Ctx) -> tuple[str, bool]:
    """The chapter section and whether it had any table to show."""
    res = c.res
    head = (
        f'<p class="eyebrow">{esc(ch.id.upper())}</p><h2>{esc(ch.title)}</h2>'
        f'<p class="lede">{esc(ch.question)}</p>'
    )
    present = [t for t in ch.requires if c.rows(t)]
    if not present:
        miss = "".join(absent(f"not in this run: {t}") for t in ch.requires)
        why = absent("Reason: " + reason_absent(res, ch))
        return f'<section class="chap" id="{ch.id}">{head}{why}{miss}</section>', False
    o = ch.build(c)
    parts = [head]
    edited = [t for t in present if res.get(t) and res.get(t).source in res.edited]  # type: ignore[union-attr]
    if edited:
        parts.append(
            f'<div class="flags">{chip("edited after the run: " + ", ".join(edited), "warn")}</div>'
        )
    if o.flags:
        parts.append(
            '<div class="flags">' + "".join(chip("▲ " + f, "warn") for f in o.flags) + "</div>"
        )
    parts.append(result_box(o.items))
    parts += o.figures
    missing = [t for t in ch.requires if t not in present]
    lines = list(dict.fromkeys([f"not in this run: {t}" for t in missing] + o.absent))
    parts += [absent(m) for m in lines]
    shown = [(n, cols) for n, cols in o.tables if res.get(n) and res.get(n).rows]  # type: ignore[union-attr]
    for i, (n, cols) in enumerate(shown):
        parts.append(_table_block(res, n, cols, inline=i == 0))
    # every other table of the chapter's analysis family that the loader found
    listed = {n for n, _ in o.tables}
    prefix = f"{ch.id.upper()}_" if ch.id != "sens" else "sens_"
    extra = [n for n in res.tables if n.startswith(prefix) and n not in listed]
    for n in extra:
        parts.append(_table_block(res, n, None, inline=False))
    study = res.record.get("context", {}).get("study", {})
    if ch.analysis and ch.analysis in study:
        parts.append(
            details(
                "Settings that governed this analysis (study.yaml)",
                verbatim(json.dumps(study[ch.analysis], indent=2)),
            )
        )
    elif ch.analysis:
        parts.append(absent("Settings: not in this run (results written by an older version)"))
    notes = list(o.notes) + c.context.get("notes", {}).get(ch.id.upper(), [])
    notes += (
        c.context.get("notes", {}).get(ch.analysis or "", [])
        if ch.analysis != ch.id.upper()
        else []
    )
    if notes:
        parts.append("<h3>Notes</h3>" + "".join(f"<p>{esc(n)}</p>" for n in dict.fromkeys(notes)))
    return f'<section class="chap" id="{ch.id}">{"".join(parts)}</section>', True


def render_defaults(c: Ctx) -> tuple[str, bool]:
    ch = DEFAULTS_CHAPTER
    has = any(
        r.get("case") == "ardupilot_default" for t in ("FT_params", "C_params") for r in c.rows(t)
    )
    head = (
        f'<p class="eyebrow">Reference</p><h2>{esc(ch.title)}</h2>'
        f'<p class="lede">{esc(ch.question)}</p>'
    )
    if not has:
        why = (
            "results written by an older version (before the default case existed)"
            if c.res.legacy
            else "no ardupilot_default case in the results: the defaults file was missing or an "
            "entry was not found (see the notes of C and FT)"
        )
        return (
            f'<section class="chap" id="defaults">{head}{absent("not in this run: " + why)}</section>',
            False,
        )
    o = ch.build(c)
    fixed = o.notes[0] if o.notes else ""
    parts = [head, f'<div class="banner">{esc(fixed)}</div>', *o.figures]
    for n, _cols in o.tables:
        parts.append(_table_block(c.res, n, None, inline=False))
    return f'<section class="chap" id="defaults">{"".join(parts)}</section>', True


# ----------------------------------------------------------------------------- settings table
def settings_html(c: Ctx) -> str:
    rows = c.rows("settings_recommended")
    head = (
        '<p class="eyebrow">Result</p><h2>Recommended settings</h2><p class="lede">One row per '
        "ArduPilot parameter the study reads. The default column is the value ArduPilot ships with, "
        "shown as a reference and never as a recommendation.</p>"
    )
    if not rows:
        return (
            f'<section class="chap" id="settings">{head}'
            f"{absent('not in this run: settings_recommended')}</section>"
        )
    has_default = any(r.get("default") not in (None, "") or r.get("default_text") for r in rows)
    cols = (
        ["ArduPilot parameter"]
        + (["ArduPilot default"] if has_default else [])
        + ["Register", "Recommended", "Searched range", "From", "Status", "What moves it"]
    )
    th = "".join(
        f'<th class="{"num" if k in ("ArduPilot default", "Register", "Recommended") else ""}">'
        f"{esc(k)}</th>"
        for k in cols
    )
    body = []
    for r in rows:
        unit = f" {r.get('unit')}" if r.get("unit") not in (None, "", "-") else ""
        val = fmt_value(r.get("value"), "") if r.get("value") not in (None, "") else ""
        reg = fmt_value(r.get("register"), "") if r.get("register") not in (None, "") else ""
        dflt = str(r.get("default_text") or "") or (
            fmt_value(r.get("default"), "") if r.get("default") not in (None, "") else ""
        )
        dflt_unit = (
            f" {r.get('default_unit')}"
            if dflt and r.get("default_unit") not in (None, "", "-")
            else ""
        )
        from_ = str(r.get("analysis") or "")
        pending = not from_
        flags = str(r.get("flags") or "")
        status = f"{r.get('grade', '')}" + (f", ▲ {flags}" if flags else "")
        tip = ""
        if r.get("change_vs_default") not in (None, ""):
            tip = (
                f' title="recommended minus default: {fmt_value(r.get("value"), "")} - '
                f'{dflt or fmt_value(r.get("default"), "")}"'
            )
        cells = [f'<td><span class="mono">{esc(r.get("parameter"))}</span></td>']
        if has_default:
            src = f' title="{esc(r.get("default_source"))}"' if r.get("default_source") else ""
            cells.append(f'<td class="num dflt"{src}>{esc(dflt + dflt_unit) if dflt else ""}</td>')
        cells += [
            f'<td class="num">{esc(reg + unit) if reg else ""}</td>',
            f'<td class="num"{tip}>{"" if pending else esc(val + unit)}</td>',
            f"<td>{esc(r.get('searched_range') or '')}</td>",
            f"<td>{esc(from_ or 'not yet analysed')}</td>",
            f"<td>{esc(status if not pending else '')}</td>",
            f'<td class="wrap">{esc(r.get("note") or "")}{esc(" - " + str(r.get("depends_on"))) if r.get("depends_on") else ""}</td>',
        ]
        body.append(f"<tr{' class=dim' if pending else ''}>{''.join(cells)}</tr>")
    table = f'<div class="tbl full"><table><thead><tr>{th}</tr></thead><tbody>{"".join(body)}</tbody></table></div>'
    return f'<section class="chap" id="settings">{head}{table}</section>'


# --------------------------------------------------------------------------------- the rest
def header_html(res: Results, title: str) -> str:
    r = res.record
    footer = (
        f"run record {str(r.get('config_hash', ''))[:8]} | {r.get('label', '?')} | commit "
        f"{r.get('git_commit', '?')}{'+dirty' if r.get('git_dirty') else ''} | {r.get('date', '?')}"
    )
    chips = [chip(res.label, "warn" if res.label == "DRAFT" else "mute")]
    chips.append(chip("config " + str(r.get("config_hash", ""))[:8], "mute"))
    chips.append(chip("commit " + str(r.get("git_commit", "?")), "mute"))
    if r.get("git_dirty"):
        chips.append(chip("dirty tree", "warn"))
    chips.append(chip(str(r.get("date", "")), "mute"))
    if r.get("seeds"):
        chips.append(chip("seeds " + ", ".join(str(s) for s in r["seeds"]), "mute"))
    study = r.get("context", {}).get("study", {})
    if study:
        chips.append(chip("analyses " + " ".join(study), "mute"))
    out = [
        f'<header><p class="eyebrow">Sage VTOL transition study</p><h1>{esc(title)}</h1>',
        f'<div class="chips">{"".join(chips)}</div><p class="meta">{esc(footer)}</p></header>',
    ]
    if res.label == "DRAFT":
        why = "; ".join(r.get("reasons", [])) or "no reason recorded"
        out.append(f'<div class="banner"><b>DRAFT.</b> Not for release. {esc(why)}.</div>')
    if res.legacy:
        out.append(
            '<div class="banner">Results written by an older version; some figures need a new '
            "run. Every table that exists is shown and is in the workbook.</div>"
        )
    if res.edited:
        out.append(
            '<div class="banner"><b>Edited after the run:</b> '
            + esc(", ".join(sorted(res.edited)))
            + ".</div>"
        )
    return "".join(out)


def inputs_html(c: Ctx) -> str:
    rows = sorted(
        c.rows("inputs_register"),
        key=lambda r: (rank(str(r.get("grade"))) * -1, str(r.get("name"))),
    )
    head = '<p class="eyebrow">Inputs</p><h2>Inputs and grades</h2>'
    if not rows:
        return (
            f'<section class="chap" id="inputs">{head}{absent("not in this run: inputs_register")}'
            + (
                details("inputs_used.md (verbatim)", verbatim(c.res.inputs_text))
                if c.res.inputs_text
                else ""
            )
            + "</section>"
        )
    counts: dict[str, int] = {}
    for r in rows:
        counts[str(r.get("grade"))] = counts.get(str(r.get("grade")), 0) + 1
    chips = "".join(
        chip(f"{g}: {counts[g]}", "warn" if g in ("assumed", "TBD") else "mute")
        for g in sorted(counts, key=lambda g: -rank(g))
    )
    shown = [{**r, "grade": ("▲ " if r.get("grade") in ("assumed", "TBD") else "") + str(r.get("grade"))}
             for r in rows]  # fmt: skip
    body = table_html(
        "inputs_register", ["name", "grade", "value", "unit", "source"], shown, limit=400
    )
    verb = (
        details("inputs_used.md (verbatim)", verbatim(c.res.inputs_text))
        if c.res.inputs_text
        else ""
    )
    return (
        f'<section class="chap" id="inputs">{head}<div class="flags">{chips}</div>'
        f'<p class="cap">Worst grade first. {len(rows)} register entries.</p>{body}{verb}</section>'
    )


def limitations_html(c: Ctx) -> str:
    lims = c.context.get("limitations", [])
    if not lims:
        return ""
    return (
        '<section class="chap" id="limits"><p class="eyebrow">Notes</p><h2>Limitations</h2><ul>'
        + "".join(f"<li>{esc(x)}</li>" for x in lims)
        + "</ul></section>"
    )


def files_html(res: Results) -> str:
    rows = []
    for f in res.files:
        name = f.name
        chipc = {
            "verified": chip("verified", "info"),
            "edited": chip("edited after the run", "warn"),
        }.get(f.status, chip(
                "written by the viewer"
                if name in VIEWER_FILES
                else "the record itself"
                if name == "run_record.json"
                else f.status,
                "mute",
            ))
        link = f'<a href="{esc(name)}">{esc(name)}</a>'
        rows.append(
            f'<tr><td>{link}</td><td class="num">{f.size:,}</td>'
            f'<td class="mono">{f.sha256[:12]}</td><td>{chipc}</td></tr>'
        )
    t = res.record.get("tables", {})
    note = (
        f"The record lists {len(t)} tables (schema {res.record.get('schema_version', 1)})."
        if t
        else "This record predates the table list; tables were found by their file names."
    )
    return (
        '<section class="chap" id="files"><p class="eyebrow">Provenance</p><h2>Run record and files</h2>'
        f'<p class="cap">{esc(note)} A file whose hash differs from the record has changed since the study wrote it.</p>'
        '<div class="tbl full"><table><thead><tr><th>file</th><th class="num">bytes</th><th>sha256</th>'
        f"<th>status</th></tr></thead><tbody>{''.join(rows)}</tbody></table></div></section>"
    )


def gate_html(res: Results) -> str:
    inner = (
        verbatim(res.gate_text) if res.gate_text else absent("not in this run: release_gate.txt")
    )
    return (
        '<section class="chap" id="gate"><p class="eyebrow">Release</p><h2>Release gate</h2>'
        + (details("release_gate.txt (verbatim)", inner) if res.gate_text else inner)
        + "</section>"
    )


def build_html(res: Results) -> str:
    """The whole page for a loaded results folder."""
    c = Ctx(res)
    title = f"Run {res.folder.name}: {res.label}"
    sections: list[tuple[str, str, str, bool]] = []  # (id, nav label, html, has content)
    sections.append(
        ("settings", "Recommended settings", settings_html(c), bool(c.rows("settings_recommended")))
    )
    for ch in CHAPTERS:
        html_, ok_ = render_chapter(ch, c)
        sections.append((ch.id, ch.title, html_, ok_))
    d_html, d_ok = render_defaults(c)
    sections.append(("defaults", DEFAULTS_CHAPTER.title, d_html, d_ok))
    lim = limitations_html(c)
    if lim:
        sections.append(("limits", "Limitations", lim, True))
    sections += [
        ("inputs", "Inputs and grades", inputs_html(c), bool(c.rows("inputs_register"))),
        ("files", "Run record and files", files_html(res), True),
        ("gate", "Release gate", gate_html(res), bool(res.gate_text)),
    ]
    nav_items = "".join(
        f'<li><a href="#{i}"{"" if ok_ else " class=off"}>{esc(label)}{"" if ok_ else " (not in this run)"}</a></li>'
        for i, label, _, ok_ in sections
    )
    nav = (
        f'<nav class="side" aria-label="Contents"><p class="brand">Sage VTOL study</p>'
        f'<p class="sub">{esc(res.folder.name)}</p><ol>{nav_items}</ol></nav>'
    )
    main = header_html(res, title) + "".join(h for _, _, h, _ in sections)
    return page(f"Sage VTOL study {res.folder.name}", nav, main)


__all__ = ["CHAPTERS", "Chapter", "Item", "build_html", "chp"]
