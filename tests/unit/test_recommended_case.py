"""E and Q2 are repeated at the FT optimum's AIRSPEED_MIN and TKOFF_THR_MAX (`recommended`)."""

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
    assert stages.follow_up_tasks(inp, done, {}) == ["basics_rec:18:95"]
    snap2 = {**snap, "airspeed_min": 17.0, "tkoff_thr_max": 0.8}
    grid2 = {i: {**r, "snapped": {**snap2, "q_transition": r["q_transition_s"]}}
             for i, r in grid.items()}
    # another optimum is another key, so the task re-runs
    assert stages.follow_up_tasks(inp, {**grid2, "FT:17:4000:80:3": {}}, {}) == ["basics_rec:17:80"]
    assert stages.rec_task_id(snap) == "basics_rec:18:95"
    # without E or Q2 there is nothing to repeat
    only_ft = load_inputs(cfg, only={"FT"})
    assert stages.follow_up_tasks(only_ft, done, {}) == []


def test_e_at_the_recommended_settings_uses_them():
    inp = load_inputs(CONFIGS, only={"E", "Q2"})
    reg = stages.run_task(inp, "basics")["tables"]
    rec = stages.run_task(inp, "basics_rec:18:100")
    names = set(rec["tables"])
    assert {"E_summary_recommended", "E_segments_recommended", "Q2_worst_recommended"} <= names
    assert not any(n.startswith("D_") for n in names)
    assert rec["airspeed_min"] == 18.0 and rec["tkoff_thr_max"] == 1.0
    a = {(r["site_amsl_m"], r["mission"]): r["max_range_km"] for r in reg["E_summary"]}
    b = {(r["site_amsl_m"], r["mission"]): r["max_range_km"]
         for r in rec["tables"]["E_summary_recommended"]}
    assert set(a) == set(b) and any(abs(a[k] - b[k]) > 1e-9 for k in a)  # the settings matter


def test_the_side_by_side_table_carries_ft_energy_as_information_only():
    inp = load_inputs(CONFIGS)
    tables = {
        "FT_params": [{"case": "register", "mean_energy_kJ": 50.0},
                      {"case": "optimum", "mean_energy_kJ": 40.0}],
        "E_summary": [{"site_amsl_m": 0.0, "mission": "one_way", "max_range_km": 90.0}],
        "E_segments": [{"site_amsl_m": 0.0, "mission": "one_way", "leg": "leg",
                        "segment": "fwd_transition", "energy_Wh": 10.0, "time_s": 12.0,
                        "distance_m": 100.0}],
        "E_summary_recommended": [{"site_amsl_m": 0.0, "mission": "one_way", "max_range_km": 91.0}],
        "E_segments_recommended": [{"site_amsl_m": 0.0, "mission": "one_way", "leg": "leg",
                                    "segment": "fwd_transition", "energy_Wh": 8.0, "time_s": 9.0,
                                    "distance_m": 80.0}],
    }
    rows = assemble.e_cases(inp, tables, {"airspeed_min": 18.0, "tkoff_thr_max": 1.0})
    reg, rec = rows
    assert (reg["case"], rec["case"]) == ("register", "recommended")
    assert reg["e_fwd_kJ"] == pytest.approx(36.0) and reg["ft_mean_energy_kJ"] == 50.0
    assert reg["e_fwd_over_ft"] == pytest.approx(36.0 / 50.0)
    assert rec["e_fwd_kJ"] == pytest.approx(28.8) and rec["ft_mean_energy_kJ"] == 40.0
    assert rec["airspeed_min_ms"] == 18.0 and rec["max_range_km"] == 91.0
