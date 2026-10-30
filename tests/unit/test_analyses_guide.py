"""E: the 'What each analysis is' section sits below the scenarios of the recommended settings."""

import csv
import json
from pathlib import Path

from sagetrans.io.assemble import e_scenarios
from sagetrans.io.inputs import load_inputs
from sagetrans.io.view.chapters import ANALYSES_GUIDE, analyses_panel
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


def test_the_guide_has_one_row_per_analysis_and_no_removed_one():
    names = [g[0] for g in ANALYSES_GUIDE]
    assert names == ["A", "D", "F", "Q2", "E", "FT", "C", "Sensitivity"]
    html = analyses_panel()
    assert "What each analysis is" in html and 'class="tbl full"' in html  # no inner scrolling
    for n in names:
        assert f">{n}<" in html
    assert "B1" not in html and "a_rec" in html
    assert "`" not in html  # no raw markdown


def test_the_section_follows_the_scenarios_which_follow_the_register_table(tmp_path):
    inp = load_inputs(ROOT / "configs", only={"E", "D", "C"})
    write_table(tmp_path, "E_summary", [{"site_amsl_m": 0.0, "mission": "one_way",
                                         "max_range_km": 92.0}])
    write_table(tmp_path, "E_cases", [{"case": "register", "site_amsl_m": 0.0,
                                       "mission": "one_way", "max_range_km": 92.0}])
    (tmp_path / "run_record.json").write_text(json.dumps({
        "label": "DRAFT", "reasons": [], "config_hash": "ab" * 32, "git_commit": "deadbee",
        "git_dirty": False, "date": "2026-10-30", "seeds": [1], "outputs": {},
        "analyses": {}, "search_spreads": {}, "schema_version": 2}), encoding="utf-8")
    html = build_html(load(tmp_path))
    assert "What each analysis is" not in html  # no scenarios table, no section
    write_table(tmp_path, "E_scenarios", e_scenarios(inp, {"airspeed_min": 18.0}))
    html = build_html(load(tmp_path))
    a = html.index("Register against recommended settings")
    b = html.index("Scenarios at the recommended settings")
    c = html.index("What each analysis is")
    assert a < b < c
