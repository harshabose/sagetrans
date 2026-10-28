"""The results viewer and the workbook (docs/OUTPUTS_spec.md section 8), no browser."""

import csv
import hashlib
import json
import re
import shutil
from pathlib import Path

import pytest
from openpyxl import load_workbook

from sagetrans.io import cli, study
from sagetrans.io.view import find_results, view
from sagetrans.io.view.load import load
from sagetrans.io.view.registry import build_html
from sagetrans.io.view.units import fmt_value
from sagetrans.io.view.workbook import write_workbook

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
DEFAULTS_FILE = ROOT / "tests" / "data" / "ardupilot_defaults.yaml"
SMOKE = """
run_name: smoke
mode: draft
analyses: [A, D, F, E, Q2]
A: {scenarios: [S-01], v0_eas_ms: [12], decel_ms2: [0.5], max_decel_v0_eas_ms: [12], samples: 20}
D:
  scenarios: [S-01]
  mass_factors: [1.0]
  v0_eas_ms: [22]
  recommendation: {ensemble_n: 2, ensemble_seed: 1, percentile: 5}
  trade: {a_plan_ms2: [2.5], headwinds_ms: [0]}
F: {scenarios: [S-01]}
E: {sites_amsl_m: [0], missions: [one_way], ranges_km: [100]}
Q2: {altitudes_m: [0], mass_factors: [1.0]}
"""


def write_table(folder: Path, name: str, rows: list[dict]) -> Path:
    cols: list[str] = []
    for r in rows:
        cols += [c for c in r if c not in cols]
    p = folder / f"{name}.csv"
    with p.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)
    return p


def write_record(folder: Path, **extra) -> None:
    rec = {"label": "DRAFT", "reasons": ["72 register entries are TBD"], "config_hash": "ab" * 32,
           "git_commit": "deadbee", "git_dirty": False, "date": "2026-10-27", "seeds": [1, 2],
           "outputs": {}, "analyses": {}, "search_spreads": {}, **extra}  # fmt: skip
    (folder / "run_record.json").write_text(json.dumps(rec), encoding="utf-8")


def listing(folder: Path) -> list[str]:
    return sorted(str(p.relative_to(folder)) for p in folder.rglob("*")) if folder.exists() else []


@pytest.fixture(scope="module")
def smoke(tmp_path_factory) -> Path:
    """A real, small study written by the study writer (schema 2)."""
    base = tmp_path_factory.mktemp("viewsmoke")
    cfg = base / "configs"
    shutil.copytree(CONFIGS, cfg)
    shutil.copy(DEFAULTS_FILE, cfg / "ardupilot_defaults.yaml")
    (cfg / "study.yaml").write_text(SMOKE, encoding="utf-8")
    out = study.run_study(cfg, out_dir=base / "res", workers=1, log=lambda s: None)
    assert out.exit_code == study.EXIT_OK and out.folder is not None
    return out.folder


# ---- the writer side ---------------------------------------------------------------------------
def test_the_run_record_lists_its_tables_and_the_study_context(smoke):
    rec = json.loads((smoke / "run_record.json").read_text(encoding="utf-8"))
    assert rec["schema_version"] == 2
    t = rec["tables"]["A_feasibility"]
    assert t["rows"] > 0 and "scenario" in t["columns"]
    assert set(t) == {"rows", "columns"} or t["rows"] > 0
    assert (smoke / "inputs_register.csv").is_file() and "inputs_register" in rec["tables"]
    ctx = rec["context"]
    assert ctx["limitations"] and "A" in ctx["study"] and ctx["mission"]["fwd_alt_loss_max_m"] > 0
    assert "inputs_register.csv" in rec["outputs"]  # hashed like every other file


def test_the_limitations_name_the_conventions_that_are_set():
    from sagetrans.io.assemble import headline_json

    out = json.loads(
        headline_json(
            {"energy_J": 11000.5}, {"sens_oat": [{"output": "distance_m"}, {"output": "energy_J"}]}
        )
    )
    assert out["energy_J"] == {"value": 11000.5, "unit": "J", "table": "sens_oat",
                               "column": "nominal_out", "row": 1}  # fmt: skip


# ---- viewing -----------------------------------------------------------------------------------
def test_view_on_an_empty_or_missing_folder_returns_2_and_writes_nothing(tmp_path, capsys):
    empty = tmp_path / "empty"
    empty.mkdir()
    (empty / "notes.txt").write_text("x", encoding="utf-8")
    before = listing(tmp_path)
    assert view(empty, open_browser=False) == 2
    assert view(tmp_path / "missing", open_browser=False) == 2
    assert listing(tmp_path) == before  # nothing written, no folder made
    assert "no results in" in capsys.readouterr().err


def test_a_folder_with_only_c_params_and_c_search_renders_the_c_chapter(tmp_path):
    write_table(tmp_path, "C_params", [
        {"case": "optimum", "T_bt_s": 2.8, "theta_A_deg": 29.9, "spin_min": 0.12,
         "Q_TRANS_DECEL_ms2": 2.2, "J": 1.39, "p95_distance_m": 90.9,
         "mean_energy_kJ": 18.2}])  # fmt: skip
    write_table(tmp_path, "C_search", [{"seed": 1, "best_cost": 1.4, "evaluations": 100}])
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    assert 'id="c"' in html and "C · restricted back-transition search" in html
    assert "not in this run: C_q_curve" in html
    assert "not in this run: A_feasibility" in html  # the other chapters say so, no exception
    assert "Search convergence" in html


def test_a_legacy_folder_resolves_the_old_names_and_says_it_is_old(tmp_path):
    write_table(tmp_path, "C_params", [{"case": "optimum", "J": 1.0, "p95_distance_m": 90.0,
                                        "mean_energy_kJ": 18.0}])  # fmt: skip
    write_table(tmp_path, "C_ensemble_runs", [{"member": 0, "energy_kJ": 18.0, "flags": ""}])
    front = {"variant": "attached", "eps_x_m": 300, "energy_kJ": 8.9, "distance_m": 100.0,
             "replay_verified": True}  # fmt: skip
    write_table(tmp_path, "B1_pareto_S-01", [front])
    write_record(tmp_path)  # no schema_version, no tables list
    res = load(tmp_path)
    assert res.legacy and res.aliased == {"C_runs": "C_ensemble_runs", "B1_front": "B1_pareto_S-01"}
    assert res.rows("C_runs") and res.rows("B1_front")[0]["scenario"] == "S-01"
    html = build_html(res)
    assert "written by an older version" in html
    assert "not in this run: settings_recommended" in html  # the settings column degrades too


def test_a_file_edited_after_the_run_gets_a_warning_chip(tmp_path):
    p = write_table(tmp_path, "C_params", [{"case": "optimum", "J": 1.0}])
    good = hashlib.sha256(p.read_bytes()).hexdigest()
    write_record(tmp_path, schema_version=2, outputs={"C_params.csv": good})
    assert "edited after the run:" not in build_html(load(tmp_path))
    p.write_text(p.read_text(encoding="utf-8").replace("1.0", "0.5"), encoding="utf-8")
    html = build_html(load(tmp_path))
    assert "edited after the run: C_params" in html


def test_every_result_box_number_equals_the_table_cell_it_cites(smoke):
    res = load(smoke)
    html = build_html(res)
    cited = re.findall(r'<p class="v[^"]*" data-src="([^"]+)"[^>]*>([^<]*)', html)
    assert cited, "no result-box numbers found"
    for src, shown in cited:
        if ";" in src:  # a number made of two cells: it shows its expression in `title`
            assert 'title="' in html.split(f'data-src="{src}"')[1][:80]
            continue
        m = re.fullmatch(r"(\w+)\.(\w+)\[(\d+)\]", src)
        assert m, src  # an expression is shown with `expr`, not data-src
        table, col, row = m.group(1), m.group(2), int(m.group(3))
        cell = res.rows(table)[row][col]
        assert shown.strip() == fmt_value(cell, col), src


def test_a_default_series_is_dashed_grey_and_labelled(tmp_path):
    write_table(tmp_path, "C_params", [
        {"case": "optimum", "T_bt_s": 2.8, "J": 1.39, "p95_distance_m": 90.9,
         "mean_energy_kJ": 18.2},
        {"case": "ardupilot_default", "T_bt_s": 3.0, "J": 1.42, "p95_distance_m": 100.2,
         "mean_energy_kJ": 19.4}])  # fmt: skip
    hist = []
    for case in ("C optimum", "C ardupilot_default"):
        hist += [{"case": case, "member": 0, "t": t / 2, "V": 15 - t, "h": 100.0 - t,
                  "theta": 0.0, "delta_r": 0.1} for t in range(6)]  # fmt: skip
    write_table(tmp_path, "C_history", hist)
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    assert re.search(r'class="ln ref dash"', html)  # the dashed reference line
    assert re.search(r'<i class="sw ref line"></i>ArduPilot default', html) or "sw ref dash" in html
    assert "ArduPilot default" in html.split('id="c"')[1].split("</section>")[0]
    # the defaults chapter appears and carries the fixed sentence
    assert "It is not tuned for this airframe and is shown only as a reference." in html
    # no text takes the colour of a series: the text classes are fixed to ink tokens
    css = html.split("<style>")[1].split("</style>")[0]
    assert not re.search(r"\.(tick|axlab|dl|ptitle|cellv)\{[^}]*var\(--s[123]\)", css)


def test_a_folder_without_a_default_case_has_no_default_column_and_says_why(tmp_path):
    write_table(tmp_path, "settings_recommended", [
        {"parameter": "Q_BACKTRANS_MS", "value": 2865.0, "unit": "ms", "register": 3000.0,
         "analysis": "C", "grade": "TBD", "default": "", "default_text": "",
         "searched_range": "1 to 6"}])  # fmt: skip
    write_table(tmp_path, "C_params", [{"case": "optimum", "J": 1.0}])
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    assert "ArduPilot default</th>" not in html.split('id="settings"')[1].split("</section>")[0]
    assert "not in this run: no ardupilot_default case" in html


def test_the_settings_table_shows_the_default_column_in_words_where_needed(tmp_path):
    write_table(tmp_path, "settings_recommended", [
        {"parameter": "TKOFF_THR_MAX", "value": 1.0, "unit": "-", "register": 0.8, "analysis": "FT",
         "grade": "TBD", "default": 0.0, "default_text": "0 (= THR_MAX)", "default_source": "x",
         "searched_range": "0.5 to 1"}])  # fmt: skip
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    assert "0 (= THR_MAX)" in html and 'class="num dflt"' in html


def test_the_page_loads_nothing_from_the_network_and_stays_small(smoke):
    html = build_html(load(smoke))
    assert not re.search(r'(href|src)\s*=\s*["\']https?:', html)
    assert not re.search(r"url\(\s*[\"']?https?:", html) and "<link" not in html
    assert "@import" not in html and "googleapis" not in html
    assert len(html.encode("utf-8")) < 5_000_000


# ---- the workbook ------------------------------------------------------------------------------
def test_the_workbook_has_an_index_a_sheet_per_table_units_and_numbers(smoke, tmp_path):
    res = load(smoke)
    out = tmp_path / "t.xlsx"
    write_workbook(res, out)
    wb = load_workbook(out)
    tables = [n for n in res.tables if n not in res.aliased]
    assert wb.sheetnames[0] == "index" and len(wb.sheetnames) == len(tables) + 1
    ws = wb["A_feasibility"]
    header = [c.value for c in ws[1]]
    assert header == res.tables["A_feasibility"].columns  # the CSV header, untouched
    units = {h: ws.cell(row=2, column=i + 1).value for i, h in enumerate(header)}
    assert units["V0_eas_ms"] == "m/s" and units["altitude_m"] == "m"
    col = header.index("V0_eas_ms") + 1
    n_rows = len(res.rows("A_feasibility"))
    cells = [ws.cell(row=r, column=col).value for r in range(3, 3 + n_rows)]
    assert cells and all(isinstance(v, int | float) for v in cells)
    assert ws.freeze_panes == "A3" and ws.auto_filter.ref.startswith("A1:")
    flag = header.index("feasible") + 1
    assert isinstance(ws.cell(row=3, column=flag).value, bool)
    # every hyperlink on the index points at a sheet that exists
    links = [c.hyperlink.location for row in wb["index"].iter_rows()
             for c in row if c.hyperlink]  # fmt: skip
    assert links and all(
        re.search(r"'([^']+)'!A1", str(x)).group(1) in wb.sheetnames for x in links
    )
    assert wb["index"]["A1"].value.startswith("DRAFT")
    assert wb.properties.creator == "sagetrans"


def test_two_workbooks_from_one_folder_have_identical_sheets(smoke, tmp_path):
    res = load(smoke)
    write_workbook(res, tmp_path / "a.xlsx")
    write_workbook(res, tmp_path / "b.xlsx")
    a, b = load_workbook(tmp_path / "a.xlsx"), load_workbook(tmp_path / "b.xlsx")
    assert a.sheetnames == b.sheetnames
    for name in a.sheetnames:
        assert [[c.value for c in r] for r in a[name].iter_rows()] == [
            [c.value for c in r] for r in b[name].iter_rows()]  # fmt: skip
    assert a.properties.created == b.properties.created == a.properties.modified


# ---- command line ------------------------------------------------------------------------------
def test_view_writes_the_page_and_the_workbook_next_to_the_tables(smoke, tmp_path):
    folder = tmp_path / "copy"
    shutil.copytree(smoke, folder)
    assert cli.main(["--view", str(folder), "--no-open"]) == 0
    assert (folder / "report.html").is_file() and (folder / "tables.xlsx").is_file()
    # a second view works, and the page lists its own files without calling them edited
    assert cli.main(["--view", str(folder), "--no-open"]) == 0
    html = (folder / "report.html").read_text(encoding="utf-8")
    assert "written by the viewer" in html
    assert find_results(folder.parent) == folder  # a parent with one results folder resolves to it


def test_view_cannot_be_combined_with_a_run(tmp_path):
    for flag in (["--only", "A"], ["--force"], ["--verify"], ["--check"]):
        with pytest.raises(SystemExit) as exc:
            cli.main(["--view", str(tmp_path), *flag])
        assert exc.value.code == 2
    with pytest.raises(SystemExit):
        cli.main(["--no-open"])  # only meaningful with --view


def test_the_ft_chapter_shows_the_cl_max_sensitivity_next_to_the_base_case(tmp_path):
    write_table(tmp_path, "FT_params", [
        {"case": "optimum", "airspeed_min_eas_ms": 18.7, "q_transition_s": 9.0, "J": 0.8},
        {"case": "register", "airspeed_min_eas_ms": 17.0, "q_transition_s": 5.0,
         "J": 11.0}])  # fmt: skip
    write_table(tmp_path, "FT_sens", [
        {"parameter": "base", "value": "", "cl_max": 1.444, "J": 0.8, "worst_stall_margin": 1.29,
         "n_flags": 0},
        {"parameter": "CL_max_scale", "value": 0.9, "cl_max": 1.30, "J": 3.1,
         "worst_stall_margin": 1.18, "n_flags": 2},
        {"parameter": "CL_max_scale", "value": 1.1, "cl_max": 1.59, "J": 0.8,
         "worst_stall_margin": 1.36, "n_flags": 0},
        # a column that mixes numbers and words reads back as text, as in the real FT_sens
        {"parameter": "tecs_start", "value": "cold", "J": 0.9}])  # fmt: skip
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    ft = html.split('id="ft"')[1].split("</section>")[0]
    assert "Sensitivity to C_L,max" in ft and "C_L,max x 0.9" in ft and "C_L,max x 1.1" in ft
    assert ">base<" in ft.replace(" ", "") or "base" in ft


def test_large_numbers_are_not_printed_in_exponent_form():
    from sagetrans.io.view.units import fmt_value

    assert fmt_value(11999.8) == "12000" and fmt_value(2029.04) == "2029"
    assert fmt_value(0.0123456) == "0.01235"


def test_the_ft_chapter_shows_the_grid_with_the_winner_and_the_failed_spreads(tmp_path):
    write_table(tmp_path, "FT_params", [
        {"case": "optimum", "airspeed_min_eas_ms": 18.0, "q_transition_s": 9.0, "J": 0.8},
        {"case": "register", "airspeed_min_eas_ms": 17.0, "q_transition_s": 5.0,
         "J": 11.0}])  # fmt: skip
    write_table(tmp_path, "FT_grid", [
        {"point": "coarse", "winner": False, "q_transition_s": 4.0, "J_continuous": 1.2,
         "seed_spread": 0.002, "t12_ok": True, "J_snapped": 1.1, "within_2pct": False},
        {"point": "coarse", "winner": False, "q_transition_s": 8.0, "J_continuous": 0.9,
         "seed_spread": 0.034, "t12_ok": False, "J_snapped": 0.85, "within_2pct": False},
        {"point": "coarse", "winner": False, "q_transition_s": 10.0, "J_continuous": 0.83,
         "seed_spread": 0.004, "t12_ok": True, "J_snapped": 0.81, "within_2pct": True},
        {"point": "fine", "winner": True, "q_transition_s": 9.0, "J_continuous": 0.82,
         "seed_spread": 0.004, "t12_ok": True, "J_snapped": 0.8, "within_2pct": True}])  # fmt: skip
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    ft = html.split('id="ft"')[1].split("</section>")[0]
    assert "Cost against Q_TRANSITION_MS" in ft
    assert "8 s (3.4%)" in ft  # the point above the T-12 limit is named
    assert 'class="mk s2' in ft  # the winner has its own mark
    assert "mk s1 hollow" in ft  # and the failed point is hollow
    assert "Within 2 % of the best cost: Q_TRANSITION_MS 9, 10 s." in ft  # the flat curve


def test_the_defaults_chapter_compares_flag_counts_and_names_saving_or_increase(tmp_path):
    write_table(tmp_path, "FT_params", [
        {"case": "optimum", "J": 0.8, "mean_energy_kJ": 40.0, "worst_stall_margin": 1.2},
        {"case": "register", "J": 5.0, "mean_energy_kJ": 41.0, "worst_stall_margin": 1.0},
        {"case": "ardupilot_default", "J": 2.0, "mean_energy_kJ": 30.0, "worst_stall_margin": 0.0}])
    write_table(tmp_path, "FT_runs", [  # fmt: skip
        {"case": "optimum", "member": 0, "flags": "alt_loss"},
        {"case": "optimum", "member": 1, "flags": ""},
        {"case": "ardupilot_default", "member": 0, "flags": "alt_loss"},
        {"case": "ardupilot_default", "member": 1, "flags": "alt_loss"},
        {"case": "ardupilot_default", "member": 2, "flags": "stall_margin"}])
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    d = html.split('id="defaults"')[1].split("</section>")[0]
    # alt_loss fires in both cases but more often in the default; stall_margin only in the default
    assert "alt_loss in 2 member(s) against 1" in d and "stall_margin in 1 member(s)" in d
    # the cost 0.8 against 2.0 is a saving
    assert "60 % saving" in d
    # more energy than the default (40 against 30) is an increase, not a negative saving
    assert "33 % increase" in d and "-33" not in d


def test_the_b1_chapter_flags_verified_points_that_fail_with_late_commands(tmp_path):
    write_table(tmp_path, "B1_front", [
        {"scenario": "S-01", "variant": "c_envelope", "eps_x_m": 60.0, "converged": True,
         "replay_verified": True, "replay_late_command_ok": False, "energy_kJ": 10.5,
         "distance_m": 60.0, "excursion_m": 1.0},
        {"scenario": "S-01", "variant": "attached", "eps_x_m": 60.0, "converged": True,
         "replay_verified": True, "replay_late_command_ok": True, "energy_kJ": 11.3,
         "distance_m": 60.0, "excursion_m": 1.0}])  # fmt: skip
    write_table(tmp_path, "B1_gap", [
        {"scenario": "S-01", "variant": "c_envelope", "c_energy_kJ": 16.0, "b1_energy_kJ": 10.5,
         "b1_replay_verified": True, "b1_late_command_ok": False}])  # fmt: skip
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    b1 = html.split('id="b1"')[1].split("</section>")[0]
    assert "1 verified point(s) fail when the commands are 10 ms late" in b1
    assert "B1-C gap quoted (S-01, c_envelope) comes from a schedule that fails" in b1


def test_the_e_chapter_explains_what_the_fwd_energy_ratio_means(tmp_path):
    write_table(tmp_path, "E_summary", [
        {"site_amsl_m": 0.0, "mission": "one_way", "max_range_km": 92.0, "fixed_Wh": 100.0}])
    write_table(tmp_path, "E_cases", [
        {"case": "register", "site_amsl_m": 0.0, "mission": "one_way", "airspeed_min_ms": 17.0,
         "tkoff_thr_max": 0.8, "max_range_km": 92.0, "e_fwd_kJ": 37.4, "e_fwd_time_s": 10.0,
         "e_fwd_distance_m": 85.0, "ft_mean_energy_kJ": 62.1, "e_fwd_over_ft": 0.60},
        {"case": "register", "site_amsl_m": 5000.0, "mission": "one_way", "airspeed_min_ms": 17.0,
         "tkoff_thr_max": 0.8, "max_range_km": 94.1, "e_fwd_kJ": 124.4, "e_fwd_time_s": 36.7,
         "e_fwd_distance_m": 404.0, "ft_mean_energy_kJ": 62.1, "e_fwd_over_ft": 2.0}])  # fmt: skip
    write_record(tmp_path, schema_version=2)
    html = build_html(load(tmp_path))
    e = html.split('id="e"')[1].split("</section>")[0]
    assert "e_fwd_over_ft" in e and "0.6" in e and "2" in e
    assert "averaged over FT" in e and "rises with the site altitude" in e
    assert "not a like-for-like comparison" in e
