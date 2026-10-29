"""The C and B1 chapters of the results viewer: scenarios, margins, spreads, sensitivity."""

import csv
import json
import math
import re
from pathlib import Path

import numpy as np

from sagetrans.io import stages
from sagetrans.io.view import charts as ch
from sagetrans.io.view.chapters import (
    history_panels,
    scenario_caption,
    select_history,
)
from sagetrans.io.view.load import load
from sagetrans.io.view.registry import build_html

ROOT = Path(__file__).resolve().parents[2]


def write_table(folder: Path, name: str, rows: list[dict]) -> None:
    cols: list[str] = []
    for r in rows:
        cols += [c for c in r if c not in cols]
    with (folder / f"{name}.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def write_record(folder: Path, **extra) -> None:
    rec = {"label": "DRAFT", "reasons": ["72 register entries are TBD"], "config_hash": "ab" * 32,
           "git_commit": "deadbee", "git_dirty": False, "date": "2026-10-29", "seeds": [1, 2],
           "outputs": {}, "analyses": {}, "search_spreads": {}, "schema_version": 2, **extra}
    (folder / "run_record.json").write_text(json.dumps(rec), encoding="utf-8")


def section(html: str, chapter: str) -> str:
    return html.split(f'id="{chapter}"')[1].split("</section>")[0]


# ---- a synthetic run: C on two scenarios and two members, B1 on two scenarios -----------------
def hist_rows(case, view, scenario, member, v0=15.0, n=6, **extra):
    meta = {"view": view, "scenario": scenario, "altitude_m": 0.0, "isa_offset_k": 0.0, "soc": 0.8,
            "headwind_ms": 0.0, "mass_kg": 26.7, "V0_tas_ms": v0, "V0_eas_ms": v0 * 0.99,
            "band_eps_h_m": 1.0, "run_overshoot_m": 1.5 if "default" not in case else 3.1}
    return [{"case": case, "member": member, **meta, **extra, "t": 0.5 * i, "x": 10.0 * i,
             "h": 0.1 * i, "V": v0 - 2.0 * i, "v_h": 0.0, "theta": 4.0 * i, "alpha_w_deg": 3.0 * i,
             "delta_r": 0.05 * i, "theta_cmd": 4.0 * i, "omega_r": 100.0}
            for i in range(n)]


def b1_hist(variant, scenario, v0=15.0, eps_x=60.0):
    return [{"case": f"B1 {variant}", "scenario": scenario, "eps_x_m": eps_x, "t": 0.5 * i,
             "x": 9.0 * i, "h": -0.05 * i, "V": v0 - 2.2 * i, "v_h": 0.0, "theta": 5.0 * i,
             "alpha_w_deg": 3.0 * i, "delta_r": 0.04 * i, "theta_cmd": 5.0 * i, "omega_r": 90.0}
            for i in range(6)]


def synthetic_run(folder: Path, late_ok: bool = False) -> None:
    write_table(folder, "C_params", [
        {"case": "optimum", "T_bt_s": 1.6, "theta_A_deg": 19.6, "spin_min": 0.11,
         "Q_TRANS_DECEL_ms2": 2.03, "J": 1.48, "p95_distance_m": 119.0, "mean_energy_kJ": 20.0},
        {"case": "ardupilot_default", "T_bt_s": 3.0, "theta_A_deg": 30.0, "spin_min": 0.15,
         "Q_TRANS_DECEL_ms2": 2.0, "J": 61.5, "p95_distance_m": 121.0, "mean_energy_kJ": 20.3}])
    c_hist = (hist_rows("C optimum", "core", "S-01", -1)
              + hist_rows("C ardupilot_default", "core", "S-01", -1, v0=15.0)
              + hist_rows("C optimum", "core", "S-03", -1, v0=19.0)  # another scenario
              + hist_rows("C optimum", "default_fails", "ensemble member", 14, v0=19.3,
                          default_fails=True)
              + hist_rows("C ardupilot_default", "default_fails", "ensemble member", 14, v0=19.3,
                          default_fails=True)
              + hist_rows("C optimum", "default_fails", "ensemble member", 2, v0=17.0,
                          default_fails=True))  # another member
    write_table(folder, "C_history", c_hist)
    write_table(folder, "B1_history", [*b1_hist("attached", "S-01"), *b1_hist("c_envelope", "S-01"),
                                       *b1_hist("attached", "S-03", v0=19.0)])
    write_table(folder, "B1_front", [
        {"scenario": "S-01", "variant": "attached", "eps_x_m": 60.0, "eps_h_m": 1.0,
         "converged": True, "replay_verified": True, "replay_late_command_ok": True,
         "energy_kJ": 11.3, "distance_m": 60.0, "V0_air_ms": 15.0},
        {"scenario": "S-01", "variant": "c_envelope", "eps_x_m": 60.0, "eps_h_m": 1.0,
         "converged": True, "replay_verified": True, "replay_late_command_ok": late_ok,
         "energy_kJ": 10.5, "distance_m": 60.0, "V0_air_ms": 15.0}])
    write_record(folder)


def numeric(rows):
    return [{k: (float(v) if re.fullmatch(r"-?[0-9.]+", v or "x") else v) for k, v in r.items()}
            for r in rows]


# ---- C1 ---------------------------------------------------------------------------------------
def test_a_history_figure_never_mixes_scenarios_or_members(tmp_path):
    synthetic_run(tmp_path)
    c_rows = numeric(csv.DictReader((tmp_path / "C_history.csv").open(encoding="utf-8")))
    b1_rows = list(csv.DictReader((tmp_path / "B1_history.csv").open(encoding="utf-8")))
    core, ident = select_history(c_rows, b1_rows, "core", "S-01")
    assert ident == ("scenario", "S-01")
    assert set(core) == {"C optimum", "C ardupilot_default", "B1 attached", "B1 c_envelope"}
    assert {r["scenario"] for rows in core.values() for r in rows} == {"S-01"}  # not S-03
    worst, ident = select_history(c_rows, b1_rows, "default_fails")
    assert ident == ("member", 14.0) and set(worst) == {"C optimum", "C ardupilot_default"}
    assert {r["member"] for rows in worst.values() for r in rows} == {14.0}  # not member 2
    assert not any(k.startswith("B1") for k in worst)  # no B1 curves in the second figure


def test_the_c_chapter_has_two_history_figures_with_a_stated_scenario(tmp_path):
    synthetic_run(tmp_path, late_ok=False)
    c = section(build_html(load(tmp_path)), "c")
    assert "Same scenario as B1 (S-01)" in c
    assert "Where the default fails (ensemble member 14)" in c
    assert "Member 0 of the ensemble" not in c  # the old single figure is gone
    # the caption states altitude, ISA, mass, SoC, headwind and the entry speed, true and equivalent
    for text in ("altitude 0 m", "ISA +0.0 K", "mass 26.7 kg", "SoC 0.80", "headwind +0.00 m/s",
                 "entry airspeed 15.0 m/s true, 14.8 m/s equivalent"):
        assert text in c, text
    assert "C optimum (15.0 m/s)" in c and "B1 attached (15.0 m/s)" in c  # entry speed in legend
    assert "open-loop bounds that use the altitude band, not controllers" in c
    assert "c_envelope is verified for exact timing only" in c  # from replay_late_command_ok
    assert "stops at" in c  # an end marker on every curve
    assert "+1 m" in c and "-1 m" in c  # the B1 altitude band as two lines
    assert "overshoots this member by 3.10 m" in c  # the default's overshoot on that member


def test_the_b1_note_follows_the_flag_not_a_fixed_text(tmp_path):
    synthetic_run(tmp_path, late_ok=True)
    c = section(build_html(load(tmp_path)), "c")
    assert "c_envelope also replays with commands 10 ms late" in c
    assert "verified for exact timing only" not in c


def test_the_airspeed_axis_starts_at_zero_and_a_run_ends_with_a_marker():
    t = list(range(6))
    s = ch.TSeries("a", "s1", "a", [float(x) for x in t],
                   {"V": [15.0 - 2.0 * x for x in t], "h": [0.0] * 6, "theta": [0.0] * 6,
                    "delta_r": [0.0] * 6}, end_mark=True)
    svg = ch.panel_figure("f", history_panels(1.0), [s])
    v_panel = svg.split('data-pi="0"')[1].split("</svg>")[0]
    ticks = re.findall(r'class="tick"[^>]*>([^<]*)<', v_panel)
    assert "0" in ticks and all(not tk.startswith("-") for tk in ticks)  # not -10
    assert svg.count("stops at") == 4  # one marker per panel


def test_scenario_caption_skips_what_is_missing():
    assert scenario_caption({"altitude_m": 500.0}) == "altitude 500 m"
    assert "entry airspeed" not in scenario_caption({"V0_tas_ms": 15.0})


def test_the_history_rows_end_at_the_stop():
    assert stages._sample_indices(12, 5) == [0, 5, 10, 11]
    assert stages._sample_indices(11, 5) == [0, 5, 10]
    assert stages._sample_indices(1, 5) == [0]

    class Sim:
        t = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

        def __getitem__(self, k):
            return np.zeros(len(self.t))

    rows = stages._c_trace(Sim(), 0.0, "C optimum", -1, meta={"scenario": "S-01"})
    assert [r["t"] for r in rows] == [0.0, 0.5, 0.6] and rows[-1]["scenario"] == "S-01"
    assert math.isfinite(rows[0]["V"])


def test_a_default_with_no_failing_member_says_so(tmp_path):
    synthetic_run(tmp_path)
    rows = list(csv.DictReader((tmp_path / "C_history.csv").open(encoding="utf-8")))
    for r in rows:
        if r["view"] == "default_fails":
            r["default_fails"] = "False"
    write_table(tmp_path, "C_history", rows)
    c = section(build_html(load(tmp_path)), "c")
    assert "No member fails under the default; this is the one with the largest overshoot" in c


# ---- C2 ---------------------------------------------------------------------------------------
class CfgStub:
    os_tol_m, sink_tol_ms = 2.0, 1.76
    margin_flag_m, margin_flag_ms, margin_frac = 0.5, 0.3, 0.75
    q_fine_offsets_ms2 = [0.0]


def test_margins_to_both_tolerances_and_the_edge_flag():
    m = stages.margin_cols(CfgStub, 1.59, 1.24)
    assert m["os_margin_m"] == 2.0 - 1.59 and m["sink_margin_ms"] == 1.76 - 1.24
    assert m["at_edge"] is True  # the overshoot margin 0.41 m is below 0.5 m
    assert stages.margin_cols(CfgStub, 1.0, 1.0)["at_edge"] is False
    assert stages.margin_cols(CfgStub, 0.5, 1.6)["at_edge"] is True  # the SINK margin alone: 0.16
    assert stages.margin_cols(CfgStub, 0.5, 1.6)["sink_margin_ms"] < 0.3
    empty = stages.margin_cols(CfgStub, "", "")  # nothing stopped: no margin, no edge
    assert empty["os_margin_m"] == "" and empty["sink_margin_ms"] == ""
    assert empty["at_edge"] is False
    assert empty["os_tol_m"] == 2.0 and empty["sink_tol_ms"] == 1.76


def fine(q, os_, sink):
    return {"q": q, "max_overshoot_m": os_, "max_end_sink_ms": sink, "J": 1.5 + q}


def test_the_margin_row_is_the_largest_q_within_both_fractions():
    rows = [fine(1.93, 1.0, 1.0), fine(2.03, 1.3, 1.2), fine(2.08, 1.45, 1.3),
            fine(2.13, 1.6, 1.0), fine(2.23, 3.0, 1.0)]  # limits: 1.5 m and 1.32 m/s
    assert stages.pick_margin_row(rows, CfgStub)["q"] == 2.08
    # a q that meets the overshoot margin but not the sink margin is not the margin row
    rows2 = [fine(2.03, 1.0, 1.0), fine(2.08, 1.2, 1.5)]
    assert stages.pick_margin_row(rows2, CfgStub)["q"] == 2.03


def test_no_q_on_the_grid_meets_the_margin():
    rows = [fine(2.03, 1.8, 1.0), fine(2.08, 1.9, 1.0), fine(2.13, 2.5, 1.0)]
    assert stages.pick_margin_row(rows, CfgStub) is None
    assert stages.pick_margin_row([], CfgStub) is None
    assert stages.pick_margin_row([fine(2.0, "", "")], CfgStub) is None  # nothing stopped


def test_the_new_c_settings_are_refused_when_out_of_range():
    import pytest
    from pydantic import ValidationError

    from sagetrans.io.settings import CCfg

    assert CCfg().margin_frac == 0.75 and CCfg().q_fine_offsets_ms2[2] == 0.0  # the defaults
    for bad in ({"margin_frac": 0.0}, {"margin_frac": 1.5}, {"margin_flag_m": -0.1},
                {"margin_flag_ms": -1.0}, {"q_fine_offsets_ms2": [0.0, float("nan")]},
                {"q_fine_offsets_ms2": [9.0]}):
        with pytest.raises(ValidationError):
            CCfg(**bad)


def test_the_q_note_is_computed_from_the_numbers():
    from types import SimpleNamespace

    from sagetrans.io.recommended import q_note

    inp = SimpleNamespace(study=SimpleNamespace(C=CfgStub))
    c_row = {"Q_TRANS_DECEL_ms2": 2.03, "max_overshoot_m": 1.59, "max_end_sink_ms": 1.24}
    d_row = {"a_rec_min_ms2": 1.65}
    m_row = {"Q_TRANS_DECEL_ms2": 1.98}
    text = q_note(inp, c_row, d_row, m_row)
    assert "C's q is 2.03 m/s^2" in text and "D's a_rec minimum is 1.65 m/s^2" in text
    assert "C is 23 % above it" in text  # 100 * (2.03 / 1.65 - 1)
    assert "worst overshoot is 1.59 m against the tolerance 2 m (PROPOSED)" in text
    assert "worst end sink is 1.24 m/s against 1.76 m/s (PROPOSED)" in text
    assert "margin row" in text and "q = 1.98 m/s^2" in text and "75%" in text
    # D not in the run, and no q on the grid meets the margin: both stated, nothing invented
    text2 = q_note(inp, c_row, None, None)
    assert "D is not in this run" in text2 and "no q on the fine grid keeps both" in text2


def c2_run(folder: Path, edge: bool = True, with_margin: bool = True) -> None:
    opt = {"case": "optimum", "T_bt_s": 1.6, "theta_A_deg": 19.6, "spin_min": 0.11,
           "Q_TRANS_DECEL_ms2": 2.03, "J": 1.48, "p95_distance_m": 119.0, "mean_energy_kJ": 20.0,
           "max_overshoot_m": 1.59, "max_end_sink_ms": 1.24, "os_tol_m": 2.0, "sink_tol_ms": 1.76,
           "os_margin_m": 0.41, "sink_margin_ms": 0.52, "at_edge": edge}
    params = [opt, {**opt, "case": "ardupilot_default", "J": 61.5, "Q_TRANS_DECEL_ms2": 2.0,
                    "max_overshoot_m": 3.8, "os_margin_m": -1.8, "at_edge": True}]
    if with_margin:
        params.append({**opt, "case": "optimum_margin", "Q_TRANS_DECEL_ms2": 1.98,
                       "max_overshoot_m": 1.4, "os_margin_m": 0.6, "at_edge": False})
    write_table(folder, "C_params", params)
    curve = [{"case": f"q = {q:g}", "q": q, "grid": "coarse", "J": 1.5, "n_overshoot": 0.0,
              "n_sinking": 0.0, "max_overshoot_m": 0.4 * q, "max_end_sink_ms": 0.6}
             for q in (1.0, 1.5, 2.0, 2.5)]
    curve += [{"case": f"q = {q:.3f}", "q": q, "grid": "fine", "J": 1.5,
               "n_overshoot": 0.0 if q < 2.1 else 11.0, "n_sinking": 0.0,
               "max_overshoot_m": 1.59 + 5 * (q - 2.03), "max_end_sink_ms": 1.2}
              for q in (1.98, 2.03, 2.08, 2.13, 2.18)]
    write_table(folder, "C_q_curve", curve)
    write_record(folder)


def test_the_c_chapter_shows_both_margins_the_chart_and_the_proposed_caveat(tmp_path):
    c2_run(tmp_path)
    c = section(build_html(load(tmp_path)), "c")
    assert "Worst overshoot / tolerance" in c and "1.59 / 2" in c
    assert "Worst end sink / tolerance" in c and "1.24 / 1.76" in c
    assert "the optimum is at the edge of a tolerance: overshoot margin 0.41 m" in c
    assert "Worst overshoot and worst end sink against q" in c
    assert c.count("tolerance 2 (PROPOSED)") == 1 and "tolerance 1.76 (PROPOSED)" in c  # both lines
    assert "relies on the overshoot and end-sink tolerances, which are PROPOSED" in c
    assert "margin row (reported, not recommended)" in c


def test_the_margin_row_is_never_the_recommendation(tmp_path):
    from sagetrans.io.inputs import load_inputs
    from sagetrans.io.recommended import settings_rows

    inp = load_inputs(ROOT / "tests" / "data" / "placeholder_configs", only={"C"})
    base = {"T_bt_s": 1.6, "theta_A_deg": 19.6, "spin_min": 0.11, "at_bound": ""}
    tables = {"C_params": [
        {"case": "optimum", "Q_TRANS_DECEL_ms2": 2.03, **base, "max_overshoot_m": 1.59,
         "max_end_sink_ms": 1.24},
        {"case": "optimum_margin", "Q_TRANS_DECEL_ms2": 1.5, **base},
        {"case": "ardupilot_default", "Q_TRANS_DECEL_ms2": 2.0, **base}]}
    rows = settings_rows(inp, tables)
    q = next(r for r in rows if r["parameter"] == "Q_TRANS_DECEL" and r["analysis"] == "C")
    assert float(q["value"]) == 2.03  # the optimum, not the margin row
    assert "margin row" in q["note"] and "q = 1.50 m/s^2" in q["note"]  # but it is named
    assert "D is not in this run" in q["note"]  # the tiny run has no D: said, not broken


# ---- C3 ---------------------------------------------------------------------------------------
def test_a_strip_shows_the_second_case_as_hollow_dots_and_a_tolerance_rule():
    dots = [(0.8, "m0", False, "s1"), (1.2, "m1", False, "s1")]
    plain = ch.strip_chart([("overshoot [m]", dots, None)], "x")
    full = ch.strip_chart([("overshoot [m]", dots, None)], "x",
                          extras=[([(3.4, "d0"), (3.8, "d1")], (2.0, "tolerance 2"))])
    assert "mk ref hollow" not in plain
    assert full.count("mk ref hollow") == 2 and "tolerance 2" in full
    def top(svg):  # the right-hand end label of the strip's axis
        return max(float(v) for v in re.findall(r'class="tick"[^>]*>([-0-9.]+)<', svg))

    assert top(plain) < 1.5 and top(full) >= 3.8  # the axis grew to include the default's members


def test_the_spread_statement_is_computed_and_names_the_cause():
    from sagetrans.io.view.chapters import spread_statement

    rows = [{"overshoot_m": v, "end_sink_ms": 0.5 + 0.1 * i}
            for i, v in enumerate((0.77, 1.1, 1.73))]
    s = spread_statement(rows, {})
    assert "No member of the optimum stops short" in s and "positive in all 3" in s
    assert "0.77 to 1.73 m" in s and "at most 0.70 m/s" in s
    assert "planned deceleration q from t = 0" in s and "pitch ramp over T_bt" in s
    short = spread_statement([{"overshoot_m": -0.2}, {"overshoot_m": 0.5}], {})
    assert "1 of 2 members stop short" in short
    assert spread_statement([], {}) == ""


def test_the_c_chapter_strips_carry_the_default_members_and_the_tolerances(tmp_path):
    c2_run(tmp_path)
    runs = []
    for case, base in (("optimum", 1.0), ("ardupilot_default", 3.0)):
        for m in range(4):
            runs.append({"case": case, "member": m, "energy_kJ": 20.0, "excursion_m": 1.0,
                         "end_sink_ms": 0.5, "overshoot_m": base + 0.2 * m, "flags": ""})
    write_table(tmp_path, "C_runs", runs)
    c = section(build_html(load(tmp_path)), "c")
    assert c.count("mk ref hollow") == 16  # four default members on each of the four strips
    assert "tolerance 2" in c and "tolerance 1.76" in c
    assert "member of the ArduPilot default" in c
    assert "No member of the optimum stops short" in c


# ---- C4 ---------------------------------------------------------------------------------------
class FakeRun:
    def __init__(self, failures, stopped=True, vh_end=0.1, overshoot=0.5):
        self.metrics = type("M", (), {"failures": tuple(failures)})()
        self.stopped, self.vh_end, self.overshoot = stopped, vh_end, overshoot


def test_failure_counts_are_complete_and_the_smooth_cost_drops_the_penalty():
    runs = [FakeRun([]), FakeRun(["overshoot"]), FakeRun(["sinking"]),
            FakeRun(["sinking", "overshoot"]), FakeRun(["not_stopped"], stopped=False)]
    cnt = stages.failure_counts(runs)
    assert cnt == {"n_failing_members": 4.0, "n_not_stopped": 1.0, "n_fail_total": 5.0}
    assert stages.failure_counts([FakeRun([])]) == {  # a clean case still has every key, zeros
        "n_failing_members": 0.0, "n_not_stopped": 0.0, "n_fail_total": 0.0}
    w = type("W", (), {"penalty": 10.0})()
    assert stages.smooth_cost(61.5, 5.0, w) == 61.5 - 50.0
    assert stages.smooth_cost(1.48, 0.0, w) == 1.48


def sens_rows():
    base = {"n_overshoot": 0.0, "n_sinking": 0.0, "n_other_flags": 0.0, "n_failing_members": 0.0}
    return [
        {"parameter": "rotor_lag_s", "value": 0.25, **base, "J": 1.48, "J_smooth": 1.48},
        {"parameter": "rotor_lag_s", "value": 0.5, **base, "J": 111.0, "J_smooth": 1.6,
         "n_sinking": 11.0, "n_failing_members": 11.0},
        {"parameter": "z_acc_p", "value": 0.015, **base, "J": 61.0, "J_smooth": 1.5,
         "n_sinking": 6.0, "n_failing_members": 6.0},
    ]


def test_the_sensitivity_shows_the_failing_members_apart_from_the_smooth_change(tmp_path):
    c2_run(tmp_path)
    rows = sens_rows()
    write_table(tmp_path, "C_sens", rows)
    params = list(csv.DictReader((tmp_path / "C_params.csv").open(encoding="utf-8")))
    for r in params:
        r["J_smooth"] = "1.48"
    write_table(tmp_path, "C_params", params)
    c = section(build_html(load(tmp_path)), "c")
    assert "Members that fail" in c and "Change of J without the failure penalty" in c
    assert "x rotor_lag_s = 0.5" in c and "x z_acc_p = 0.015" in c  # the failing rows are marked
    assert "x rotor_lag_s = 0.25" not in c
    assert "11 failing member(s): sinking 11" in c and "6 failing member(s): sinking 6" in c
    assert "(1.600 / 1.480 - 1) x 100" in c  # the expression stays in the tooltip
    assert "+7500" not in c  # the penalty no longer sets the scale
    # an older results folder (no J_smooth) still gets the single chart
    for r in rows:
        r.pop("J_smooth")
    write_table(tmp_path, "C_sens", rows)
    for r in params:
        r.pop("J_smooth")
    write_table(tmp_path, "C_params", params)
    old = section(build_html(load(tmp_path)), "c")
    assert "Members that fail" not in old and "One-at-a-time sensitivity" in old
