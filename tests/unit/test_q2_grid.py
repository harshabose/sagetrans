"""Q2: one heat grid for the convention E reads, with what E assumes."""

from pathlib import Path

from test_c_report import section, synthetic_run, write_table

from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs
from sagetrans.io.settings import Q2Cfg
from sagetrans.io.view.chapters import pusher_grid, q2_rows_fed_to_e
from sagetrans.io.view.load import load
from sagetrans.io.view.registry import build_html

ROOT = Path(__file__).resolve().parents[2]


def q2_rows(feeds=True):
    rows = []
    for conv, k in (("tas", 1.0), ("eas", 0.5)):
        for alt in (0.0, 5000.0):
            for mf in (0.85, 1.0):
                for soc in (0.8, 0.3):
                    a = k * (2.0 - alt / 5000.0 - 0.3 * mf - 0.2 * (1 - soc))
                    row = {"convention": conv, "altitude_m": alt, "mass_factor": mf, "soc": soc,
                           "a_min_ms2": a, "reachable": a > 0.0}
                    if feeds:
                        row["feeds_e"] = conv == "eas"
                    rows.append(row)
    return rows


def test_the_default_feed_convention_is_equivalent_airspeed_and_the_rows_say_so():
    assert Q2Cfg().feed_convention == "eas"
    inp = load_inputs(ROOT / "configs", only={"Q2"})
    cells = stages.q2_cells(inp, ["tas", "eas"])
    fed = {r["convention"] for r, _ in cells if r["feeds_e"]}
    assert fed == {inp.study.Q2.feed_convention} == {"eas"}
    assert all(r["feeds_e"] is (r["convention"] == "eas") for r, _ in cells)


def test_the_grid_shows_the_convention_e_reads_and_not_the_first_one_in_the_table():
    sub, conv = q2_rows_fed_to_e(q2_rows())
    assert conv == "eas" and all(r["convention"] == "eas" for r in sub)
    e_rows = [{"site_amsl_m": 0.0, "a_fwd_out_ms2": 0.9}, {"site_amsl_m": 5000.0}]
    html = pusher_grid(sub, conv, e_rows)
    assert html is not None and "an equivalent airspeed (eas: the convention E reads)" in html
    assert html.count("class=\"cell") >= 2 * 4 + 2  # 2 altitudes x (4 cases + E)
    assert "0.9" in html and ">E<" in html  # E's value at 0 m; no value at 5000 m
    # eas at 0 m, mass 0.85, SoC 0.8: 0.5 x (2 - 0.255 - 0.04) = 0.8525
    assert "0.85" in html


def test_an_older_folder_without_feeds_e_prefers_eas():
    sub, conv = q2_rows_fed_to_e(q2_rows(feeds=False))
    assert conv == "eas" and len(sub) == 8
    only_tas = [r for r in q2_rows(feeds=False) if r["convention"] == "tas"]
    assert q2_rows_fed_to_e(only_tas)[1] == "tas"


def test_unreachable_cases_are_crossed_and_nothing_is_drawn_without_rows():
    rows = [r for r in q2_rows() if r["convention"] == "eas"]
    rows[0]["a_min_ms2"], rows[0]["reachable"] = -0.08, False
    html = pusher_grid(rows, "eas", [])
    assert html is not None and "-0.08  x" in html and ">E<" not in html
    assert pusher_grid([], "eas", []) is None


def test_the_chapter_has_the_grid_and_neither_the_scatter_nor_the_convention_chart(tmp_path):
    synthetic_run(tmp_path)
    write_table(tmp_path, "Q2_pusher_margin", q2_rows())
    q2 = section(build_html(load(tmp_path)), "q2")
    assert "Pusher acceleration by altitude, mass and state of charge" in q2
    assert "True against equivalent airspeed reading" not in q2
    assert "Worst acceleration by altitude and state of charge" not in q2
    assert 'Pusher acceleration</h3>' not in q2
    # the worst-case tile follows the convention E reads: the eas minimum, not the tas one
    assert "convention = eas; altitude_m = 5000.0 m" in q2
    assert 'a_min_ms2[15]">0.28' in q2
    assert "8 of 8 rows have reachable = yes (eas reading)" in q2
