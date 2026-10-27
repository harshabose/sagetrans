"""`settings_recommended.csv`: one row per ArduPilot parameter the study recommends or reads.

The sheet a flight-test lead reads first. Each row names the analysis it came from and how far to
trust it (`grade`, the weakest evidence among the inputs the study grades, and `depends_on`, the
inputs at that grade). The ArduPilot default (docs/DEFAULTS_spec.md) is a REFERENCE: it is read
from `ardupilot_defaults.yaml`, shown next to the register and the recommended value, and never
used as an input. A parameter no analysis has decided keeps the register value, marked "not yet
analysed" (a default is never filled in as a recommendation). Values are in the parameter's own unit
(ms, deg, m/s, m/s^2, a throttle fraction).
"""

from __future__ import annotations

import math
from collections.abc import Callable, Mapping
from dataclasses import dataclass
from typing import Any

from sagetrans.analysis.sensitivity import weakest_status
from sagetrans.io.inputs import StudyInputs

Table = list[dict[str, Any]]
DEG = 180.0 / math.pi


@dataclass(frozen=True)
class _Param:
    name: str
    unit: str  # the unit shown
    scale: float  # shown = SI x scale
    register: Callable[[StudyInputs], float | None]  # SI
    c_key: str | None = None  # the key of `C_params`; shown = value x c_scale
    c_scale: float = 1.0
    ft_key: str | None = None  # the key of `FT_params`
    ft_scale: float = 1.0
    c_name: str | None = None  # the name in C's `at_bound`
    c_bound: str | None = None  # the field of `C.bounds`
    ft_name: str | None = None  # the name in FT's `at_bound`


PARAMS: tuple[_Param, ...] = (
    _Param("Q_BACKTRANS_MS", "ms", 1e3, lambda i: i.start_params["T_bt"], "T_bt_s", 1e3,
           c_name="T_bt", c_bound="T_bt"),
    _Param("Q_A_ANGLE_MAX", "deg", DEG, lambda i: i.start_params["theta_A"], "theta_A_deg", 1.0,
           c_name="theta_A", c_bound="theta_A_deg"),
    _Param("Q_M_SPIN_MIN", "-", 1.0, lambda i: i.start_params["spin_min"], "spin_min", 1.0,
           c_name="spin_min", c_bound="spin_min"),
    _Param("Q_TRANS_DECEL", "m/s^2", 1.0, lambda i: None, "Q_TRANS_DECEL_ms2", 1.0,
           c_name="q", c_bound="q"),
    _Param("AIRSPEED_MIN", "m/s", 1.0, lambda i: i.airspeed_min, None, 1.0,
           "airspeed_min_eas_ms", 1.0, ft_name="airspeed_min"),
    _Param("Q_TRANSITION_MS", "ms", 1e3, lambda i: i.start_params["q_transition"], None, 1.0,
           "q_transition_s", 1e3, ft_name="q_transition"),
    _Param("TKOFF_THR_MAX", "-", 1.0, lambda i: i.tkoff_thr_max, None, 1.0, "tkoff_thr_max", 1.0,
           ft_name="tkoff_thr_max"),
    _Param("Q_TRAN_PIT_MAX", "deg", DEG, lambda i: math.radians(i.forward.tran_pit_max_deg), None,
           1.0, "tran_pit_max_deg", 1.0, ft_name="tran_pit_max"),
)


def _fmt(v: float | None) -> float | str:
    return "" if v is None else float(v)


def _range_text(lo: float, hi: float) -> str:
    return f"{lo:g} to {hi:g}"


def _bound_flag(at_bound: str, name: str | None) -> str:
    """"at upper bound" / "at lower bound" from a table's `at_bound` string (`name:hi;...`)."""
    for part in (at_bound or "").split(";"):
        key, _, side = part.partition(":")
        if name is not None and key == name:
            return "at upper bound" if side == "hi" else "at lower bound"
    return ""


def _default_text(inp: StudyInputs, name: str, unit: str, scale: float) -> str:
    """The default as the file stores it, in words where it means "use another parameter"."""
    raw = inp.defaults.get(name)
    if raw is None:
        return ""
    if name == "TKOFF_THR_MAX" and raw == 0.0:
        return "0 (= THR_MAX)"
    return f"{raw * scale:g}"


def settings_rows(inp: StudyInputs, tables: Mapping[str, Table]) -> Table:
    """The rows of `settings_recommended.csv` from the tables already built.

    `change_vs_default` is the recommended value minus the ArduPilot default in the shown unit,
    empty when either side is missing. A parameter without any analysis keeps its register value.
    """
    statuses = inp.input_statuses()
    grade = weakest_status(list(statuses.values())) if statuses else "assumed"
    weakest = [n for n, s in statuses.items() if s == grade][:4]
    depends = ", ".join(weakest) + (" ..." if len(weakest) == 4 else "") if weakest else ""
    c_row = next((r for r in tables.get("C_params", []) if r.get("case") == "optimum"), None)
    ft_row = next((r for r in tables.get("FT_params", []) if r.get("case") == "optimum"), None)
    ft_reg = next((r for r in tables.get("FT_params", []) if r.get("case") == "register"), None)
    d_rows = tables.get("D_recommendation") or []
    d_row = d_rows[0] if d_rows else None
    rows: Table = []
    for p in PARAMS:
        reg_si = p.register(inp)
        register = None if reg_si is None else reg_si * p.scale
        dflt_si = inp.resolve_default(p.name)
        default = None if dflt_si is None else dflt_si * p.scale
        base = {
            "parameter": p.name, "unit": p.unit, "register": _fmt(register),
            "default": _fmt(default), "default_unit": p.unit if default is not None else "",
            "default_text": _default_text(inp, p.name, p.unit, p.scale),
            "default_source": (
                inp.config.record(f"ardupilot_defaults.ardupilot.{p.name}").source
                if f"ardupilot_defaults.ardupilot.{p.name}" in inp.config else ""
            ),
        }  # fmt: skip

        def add(value: float | None, analysis: str, flags: str, note: str, rng: str,
                base: dict[str, Any] = base, default: float | None = default) -> None:
            rows.append({
                **base, "value": _fmt(value), "analysis": analysis, "grade": grade,
                "flags": flags, "depends_on": depends if analysis else "", "note": note,
                "searched_range": rng,
                "change_vs_default": "" if value is None or default is None else value - default,
            })  # fmt: skip

        added = False
        if c_row is not None and p.c_key is not None and c_row.get(p.c_key) not in (None, ""):
            lo, hi = getattr(inp.study.C.bounds, p.c_bound or "")
            add(float(c_row[p.c_key]) * p.c_scale, "C",
                _bound_flag(str(c_row.get("at_bound", "")), p.c_name),
                "searched (back transition)", _range_text(lo * p.c_scale, hi * p.c_scale))
            added = True
        if p.name == "Q_TRANS_DECEL" and d_row is not None and d_row.get("a_rec_min_ms2") != "":
            add(float(d_row["a_rec_min_ms2"]), "D", "", "the minimum over D's ensemble (a_rec)",
                "not searched")
            added = True
        if ft_row is not None and p.ft_key is not None and ft_row.get(p.ft_key) not in (None, ""):
            b = inp.study.FT.bounds
            rng_si = {"airspeed_min": b.airspeed_min_eas_ms, "q_transition": b.q_transition_s,
                      "tkoff_thr_max": b.tkoff_thr_max, "tran_pit_max": b.q_tran_pit_max_deg}
            lo, hi = rng_si[p.ft_name or ""]
            shown = (lo * p.ft_scale, hi * p.ft_scale)
            n_bad = int(float(ft_reg.get("n_stall_margin") or 0)) if ft_reg else 0
            note = "searched (forward transition, equivalent airspeed)"
            if n_bad:
                note += f"; the register values fail the stall margin in {n_bad} member(s)"
            add(float(ft_row[p.ft_key]) * p.ft_scale, "FT",
                _bound_flag(str(ft_row.get("at_bound", "")), p.ft_name), note,
                _range_text(*shown))  # fmt: skip
            added = True
        if not added:  # no analysis decided it: the register value stands
            add(register, "", "", "not yet analysed", "not searched")
    return rows
