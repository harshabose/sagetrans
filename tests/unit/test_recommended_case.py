"""E and Q2 are repeated at the FT optimum's AIRSPEED_MIN and TKOFF_THR_MAX (`recommended`)."""

import dataclasses
import shutil
from pathlib import Path

import pytest

from sagetrans.io import assemble, stages
from sagetrans.io.inputs import load_inputs

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
GRID = """
FT:
  ensemble_n: 2
  final_ensemble_n: 2
  history_scenarios: [S-01]
  grid: {q_transition_s: [4.0, 6.0], fine_offsets_s: [], seeds: [1], maxiter: 1}
"""


def test_the_settings_are_replaced_everywhere_they_are_read():
    inp = load_inputs(CONFIGS)
    new = stages.with_settings(inp, 19.0, 0.9)
    assert new.airspeed_min == 19.0 and new.tkoff_thr_max == 0.9
    assert new.mission.v_trans == 19.0  # the loaded EAS; mission_at_site converts it once
    assert new.forward.airspeed_min_eas == 19.0 and new.forward.tkoff_thr_max == 0.9
    assert new.start_params["airspeed_min"] == 19.0 and new.start_params["tkoff_thr_max"] == 0.9
    assert inp.airspeed_min != 19.0  # the register case is untouched


def test_the_recommended_task_is_planned_from_the_ft_optimum_and_its_id_carries_it(tmp_path):
    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    with (cfg / "study.yaml").open("a", encoding="utf-8") as fh:
        fh.write(GRID)
    inp = load_inputs(cfg, only={"FT", "E"})
    snap = {"airspeed_min": 18.0, "q_transition": 5.0, "tkoff_thr_max": 0.95, "tran_pit_max": 3.0}
    grid = {i: {"q_transition_s": q, "row": {"J_continuous": 1.0, "J_snapped": 1.0},
                "snapped": {**snap, "q_transition": q}}
            for i, q in (("FT_grid:4000", 4.0), ("FT_grid:6000", 6.0))}
    assert stages.follow_up_tasks(inp, grid, {}) == ["FT:18:4000:95:3"]  # the winner's report
    done = {**grid, "FT:18:4000:95:3": {}}
    assert stages.follow_up_tasks(inp, done, {}) == ["basics_rec:18:4000:95"]
    snap2 = {**snap, "airspeed_min": 17.0, "tkoff_thr_max": 0.8}
    grid2 = {i: {**r, "snapped": {**snap2, "q_transition": r["q_transition_s"]}}
             for i, r in grid.items()}
    # another optimum is another key, so the task re-runs
    assert stages.follow_up_tasks(inp, {**grid2, "FT:17:4000:80:3": {}}, {}) == [
        "basics_rec:17:4000:80"]
    assert stages.rec_task_id(snap) == "basics_rec:18:5000:95"  # Q_TRANSITION_MS is in the id
    # without E or Q2 there is nothing to repeat
    only_ft = load_inputs(cfg, only={"FT"})
    assert stages.follow_up_tasks(only_ft, done, {}) == []


def test_e_at_the_recommended_settings_uses_them():
    inp = load_inputs(CONFIGS, only={"E", "Q2"})
    reg = stages.run_task(inp, "basics")["tables"]
    rec = stages.run_task(inp, "basics_rec:18:9000:100")
    names = set(rec["tables"])
    assert {"E_summary_recommended", "E_segments_recommended", "Q2_worst_recommended"} <= names
    assert not any(n.startswith("D_") for n in names)
    assert rec["airspeed_min"] == 18.0 and rec["tkoff_thr_max"] == 1.0
    assert rec["q_transition_s"] == 9.0
    a = {(r["site_amsl_m"], r["mission"]): r["max_range_km"] for r in reg["E_summary"]}
    b = {(r["site_amsl_m"], r["mission"]): r["max_range_km"]
         for r in rec["tables"]["E_summary_recommended"]}
    assert set(a) == set(b) and any(abs(a[k] - b[k]) > 1e-9 for k in a)  # the settings matter


def _ft_member(case, alt, wait, fade):
    return {"case": case, "altitude_m": alt, "energy_kJ": wait + fade, "E_wait_kJ": wait,
            "E_fade_kJ": fade}


def test_the_side_by_side_table_compares_e_with_ft_members_grouped_by_site():
    base = load_inputs(CONFIGS)
    e_cfg = base.study.E.model_copy(update={"sites_amsl_m": [0.0, 1500.0, 5000.0]})
    inp = dataclasses.replace(base, study=base.study.model_copy(update={"E": e_cfg}))
    tables = {
        "FT_runs": [
            _ft_member("register", 100.0, 30.0, 10.0), _ft_member("register", 600.0, 34.0, 10.0),
            _ft_member("register", 4900.0, 70.0, 10.0),  # nearest site 5000 m
            _ft_member("register", 1700.0, 44.0, 10.0),  # nearest site 1500 m
            _ft_member("optimum", 200.0, 30.0, 30.0), _ft_member("optimum", 5000.0, 60.0, 30.0),
        ],
        "E_summary": [{"site_amsl_m": s, "mission": "one_way", "max_range_km": 90.0}
                      for s in (0.0, 1500.0, 5000.0)],
        "E_segments": [{"site_amsl_m": s, "mission": "one_way", "leg": "leg",
                        "segment": "fwd_transition", "energy_Wh": e, "time_s": 12.0,
                        "distance_m": 100.0} for s, e in ((0.0, 10.0), (1500.0, 12.0),
                                                          (5000.0, 20.0))],
        "E_summary_recommended": [{"site_amsl_m": 0.0, "mission": "one_way", "max_range_km": 91.0}],
        "E_segments_recommended": [{"site_amsl_m": 0.0, "mission": "one_way", "leg": "leg",
                                    "segment": "fwd_transition", "energy_Wh": 15.0, "time_s": 9.0,
                                    "distance_m": 80.0}],
    }
    rows = assemble.e_cases(inp, tables, {"airspeed_min": 18.0, "tkoff_thr_max": 1.0})
    by = {(r["case"], r["site_amsl_m"]): r for r in rows}
    s0 = by[("register", 0.0)]  # the two members within 750 m of the site, not the 5000 m ones
    assert s0["e_fwd_kJ"] == pytest.approx(36.0)
    assert s0["ft_site_energy_kJ"] == pytest.approx(42.0) and s0["ft_site_members"] == 2
    assert s0["ft_site_wait_kJ"] == pytest.approx(32.0) and s0["ft_site_fade_kJ"] == 10.0
    assert s0["e_fwd_over_ft"] == pytest.approx(36.0 / 42.0)
    assert by[("register", 1500.0)]["ft_site_members"] == 1
    assert by[("register", 5000.0)]["e_fwd_over_ft"] == pytest.approx(72.0 / 80.0)
    # the recommended case reads FT's optimum members, and E at the recommended settings
    r0 = by[("recommended", 0.0)]
    assert r0["e_fwd_kJ"] == pytest.approx(54.0) and r0["ft_site_energy_kJ"] == 60.0
    assert r0["airspeed_min_ms"] == 18.0 and r0["max_range_km"] == 91.0
    # no members near a site: no number, never the ensemble mean
    assert assemble.ft_site_energy([_ft_member("register", 0.0, 1.0, 1.0)], "optimum", [0.0]) == {}
