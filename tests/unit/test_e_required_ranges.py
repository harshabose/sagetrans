"""E: the requirements are distances flown, and the figure sets the maximum range beside them."""

import dataclasses
import math
import re
from pathlib import Path

import pytest
from pydantic import ValidationError

from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs
from sagetrans.io.settings import ECfg
from sagetrans.io.view import chapters as chp

ROOT = Path(__file__).resolve().parents[2]


def test_ranges_km_is_gone_and_says_why():
    with pytest.raises(ValidationError, match="ranges_km was removed"):
        ECfg(ranges_km=[50])
    assert not hasattr(ECfg(), "ranges_km")


def test_a_round_trip_requirement_is_the_distance_flown_so_its_radius_is_half():
    inp = load_inputs(ROOT / "configs", only={"E"})
    one = stages.solve_e_case(inp, 0.0, "one_way", 1.65, math.nan).mb
    two = stages.solve_e_case(inp, 0.0, "round_trip", 1.65, math.nan).mb
    assert (one.n_legs, two.n_legs) == (1, 2)
    assert two.radius_for_flown(100e3) == 50e3 and one.radius_for_flown(100e3) == 100e3
    cases = {c["case"]: c for c in two.cases(inp.requirements_km)}
    assert set(cases) == {"plausible", *(f"required_{x:g}" for x in inp.requirements_km)}
    assert cases["required_100"]["range_km"] == pytest.approx(50.0)
    assert cases["required_100"]["flown_km"] == pytest.approx(100.0)
    assert cases["required_120"]["range_km"] == pytest.approx(60.0)
    plaus = cases["plausible"]
    assert plaus["margin_Wh"] == pytest.approx(0.0, abs=1e-6) and plaus["fits"] is True
    assert plaus["range_km"] == pytest.approx(two.max_range_m / 1e3)
    assert plaus["flown_km"] == pytest.approx(2 * two.max_range_m / 1e3)
    # the conflict test is on the distance flown: the round trip's 45.8 km radius flies 91.6 km
    assert two.conflicts((100e3, 90e3)) == {100e3: True, 90e3: False}
    assert one.conflicts((100e3,)) == {100e3: one.max_range_m < 100e3}


def test_the_summary_compares_the_radius_a_round_trip_can_fly_with_half_the_requirement():
    inp = load_inputs(ROOT / "configs", only={"E"})
    e_cfg = inp.study.E.model_copy(update={"sites_amsl_m": [0.0]})
    inp = dataclasses.replace(inp, study=inp.study.model_copy(
        update={"analyses": ["E"], "E": e_cfg}))
    out = stages.task_basics(inp)["tables"]
    summ = {r["mission"]: r for r in out["E_summary"]}
    rt = summ["round_trip"]
    assert rt["radius_at_100km"] == pytest.approx(50.0)
    assert rt["conflict_100km"] == (rt["max_range_km"] < 50.0)
    assert rt["flown_total_km"] == pytest.approx(2 * rt["max_range_km"])
    assert summ["one_way"]["radius_at_100km"] == pytest.approx(100.0)
    cases = [r["case"] for r in out["E_budget"] if r["mission"] == "round_trip"]
    assert cases == ["plausible", "required_100", "required_120"]


def _case(site, mission, case, req, r_km, n, total, usable=1776.0, cruise=1400.0, leg="out"):
    row = {"site_amsl_m": site, "mission": mission, "case": case, "required_km": req,
           "range_km": r_km, "flown_km": r_km * n, "cruise_Wh": cruise, "reserve_Wh": 150.0,
           "drop_Wh": 0.0, "total_Wh": total, "usable_Wh": usable, "margin_Wh": usable - total}
    for seg in chp.E_SEGS:
        row[f"{leg}_{seg}_Wh"] = 20.0
    return row


def _budget():
    rows = []
    for m, n, rmax in (("one_way", 1, 92.0), ("round_trip", 2, 45.8)):
        leg = "leg" if m == "one_way" else "out"
        rows += [_case(0.0, m, "plausible", "", rmax, n, 1776.0, leg=leg),
                 _case(0.0, m, "required_100", 100.0, 100.0 / n, n, 1911.0 + 10 * n, leg=leg),
                 _case(0.0, m, "required_120", 120.0, 120.0 / n, n, 2249.0 + 10 * n, leg=leg)]
    return rows


def test_the_figure_has_the_checkbox_the_faint_cases_and_a_computed_verdict():
    html = chp.energy_cases_figure(_budget())
    assert html is not None
    assert 'type="checkbox" id="e-req"' in html
    assert "Show the required ranges (100 and 120 km)" in html
    assert html.count('class="ghosts"') == 2 * 9  # a group per bar row, nine rows, two panels
    assert "bar ghost g0" in html and "bar ghost g1" in html
    assert ("max range 92.0 km flown</b>: short of 100 km by 8.0 km; "
            "short of 120 km by 28.0 km") in html
    assert "max range 91.6 km flown (radius 45.8 km)" in html
    assert "short of 100 km by 8.4 km" in html  # the round trip, as distance flown
    assert "usable 1776" in html and "payload drop" not in html
    # one-way fixed segments (the `leg` columns) are drawn, not zero
    assert "take-off at the maximum range (92.0 km): 20.0 Wh" in html


def test_the_axis_is_shared_by_the_sites_of_a_mission_and_does_not_depend_on_the_checkbox():
    rows = _budget() + [dict(r, site_amsl_m=5000.0) for r in _budget()]
    html = chp.energy_cases_figure(rows)
    assert html is not None
    ticks = [t for t in html.split("<h4>") if "one way" in t.split("</h4>")[0]]
    assert len(ticks) == 2
    top = [max(float(v) for v in re.findall(r'class="tick"[^>]*>([-0-9.]+)<', t))
           for t in ticks]
    assert top[0] == top[1]


def test_a_site_whose_energy_cannot_cover_the_fixed_segments_says_so():
    rows = [r for r in _budget() if r["case"] != "plausible" or r["mission"] != "round_trip"]
    html = chp.energy_cases_figure(rows)
    assert html is not None and "no maximum range" in html


def test_an_older_folder_without_cases_keeps_the_old_figures_and_one_way_is_not_zero():
    legacy = [{k: v for k, v in r.items() if k not in ("case", "required_km", "flown_km")}
              for r in _budget() if r["case"] != "plausible"]
    assert chp.energy_cases_figure(legacy) is None


def test_the_viewer_draws_the_new_figure_and_the_old_ones_from_an_older_folder(tmp_path):
    from test_c_b1_report import section, synthetic_run, write_table

    from sagetrans.io.view.load import load
    from sagetrans.io.view.registry import build_html

    synthetic_run(tmp_path)
    summ = [{"site_amsl_m": 0.0, "mission": m, "max_range_km": r, "usable_Wh": 1776.0,
             "margin_Wh_at_100km": -135.0} for m, r in (("one_way", 92.0), ("round_trip", 45.8))]
    write_table(tmp_path, "E_summary", summ)
    write_table(tmp_path, "E_budget", _budget())
    e = section(build_html(load(tmp_path)), "e")
    assert "Maximum range and the required ranges: energy by segment" in e
    assert "Whole-mission energy by segment" not in e
    assert "Maximum range against the requirement" not in e
    legacy = [{k: v for k, v in r.items() if k not in ("case", "required_km", "flown_km")}
              for r in _budget() if r["case"] != "plausible"]
    write_table(tmp_path, "E_budget", legacy)
    e = section(build_html(load(tmp_path)), "e")
    assert "Whole-mission energy by segment" in e and "Maximum range against the requirement" in e
    # one way: the `leg` columns count (they were read as zero before)
    assert "takeoff: 20.0 Wh" in e and "takeoff: 0.0 Wh" not in e


def test_the_scenario_table_gives_each_scenario_and_its_entry_state_at_both_settings():
    from sagetrans.io.assemble import e_scenarios

    inp = load_inputs(ROOT / "configs", only={"E", "D", "C"})
    rows = {r["scenario"]: r for r in e_scenarios(inp, {"airspeed_min": 18.0})}
    assert list(rows) == list(inp.scenarios)
    s1, s2, s4, s5 = rows["S-01"], rows["S-02"], rows["S-04"], rows["S-05"]
    # AIRSPEED_MIN - 2, read as an equivalent airspeed: sea level standard day has TAS = EAS
    assert s1["airspeed_min_register_ms"] == inp.airspeed_min
    assert s1["entry_eas_register_ms"] == pytest.approx(inp.airspeed_min - 2.0)
    assert s1["entry_eas_recommended_ms"] == pytest.approx(16.0)
    assert s1["entry_tas_recommended_ms"] == pytest.approx(16.0)
    assert s2["entry_tas_register_ms"] > s2["entry_eas_register_ms"]  # thinner air at 5000 m
    # the ground speed is the true airspeed less the headwind
    assert s5["entry_ground_recommended_ms"] == pytest.approx(
        s5["entry_tas_recommended_ms"] - 5.0)
    assert s4["entry_ground_register_ms"] == pytest.approx(s4["entry_tas_register_ms"] + 5.0)
    for r in rows.values():
        assert r["entry_over_stall_recommended"] > r["entry_over_stall_register"]
        assert r["entry_pitch_deg"] == inp.study.D.theta_fw0_deg
        assert r["entry_rotor_start"] == inp.study.C.rotor_start
    assert rows["S-03"]["mass_kg"] > rows["S-01"]["mass_kg"]  # 1.15 x
    assert "sea level" in s1["note"]


def test_the_scenario_panel_follows_the_register_against_recommended_table(tmp_path):
    from test_c_b1_report import section, synthetic_run, write_table

    from sagetrans.io.assemble import e_scenarios
    from sagetrans.io.view.load import load
    from sagetrans.io.view.registry import build_html

    inp = load_inputs(ROOT / "configs", only={"E", "D", "C"})
    synthetic_run(tmp_path)
    write_table(tmp_path, "E_budget", _budget())
    write_table(tmp_path, "E_summary", [{"site_amsl_m": 0.0, "mission": "one_way",
                                         "max_range_km": 92.0}])
    write_table(tmp_path, "E_cases", [{"case": "register", "site_amsl_m": 0.0,
                                       "mission": "one_way", "max_range_km": 92.0}])
    e = section(build_html(load(tmp_path)), "e")
    assert "Scenarios at the recommended settings" not in e  # no table, no panel
    write_table(tmp_path, "E_scenarios", e_scenarios(inp, {"airspeed_min": 18.0}))
    e = section(build_html(load(tmp_path)), "e")
    assert e.index("Register against recommended settings") < e.index(
        "Scenarios at the recommended settings")
    assert "S-07" in e and "recommended one (18 m/s)" in e
    assert "AIRSPEED_MIN (17 m/s)" in e
