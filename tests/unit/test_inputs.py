"""The config folder, the loader, and the one-command study runner."""

import dataclasses
import json
import math
import re
import shutil
from pathlib import Path

import pytest

from sagetrans.analysis.common import make_record
from sagetrans.analysis.energy import BatteryWindow, MissionProfile
from sagetrans.control.ardupilot_like import BackTransitionParams, BaselineParams
from sagetrans.io import cli, stages, study
from sagetrans.io.inputs import InputError, load_inputs
from sagetrans.io.record import check_outputs
from sagetrans.io.settings import ALL_ANALYSES
from sagetrans.physics.params import placeholder_vehicle

ROOT = Path(__file__).resolve().parents[2]
# frozen placeholders: the live configs/ folder holds the user's real data and changes over time
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
LIVE_CONFIGS = ROOT / "configs"


@pytest.fixture()
def cfgdir(tmp_path: Path) -> Path:
    """A private, editable copy of the shipped config folder."""
    dst = tmp_path / "configs"
    shutil.copytree(CONFIGS, dst)
    return dst


def edit(path: Path, old: str, new: str) -> None:
    text = path.read_text(encoding="utf-8")
    assert old in text, f"{old!r} not in {path.name}"
    path.write_text(text.replace(old, new, 1), encoding="utf-8")


def complete(cfgdir: Path) -> None:
    """Make the register release grade: nothing is TBD any more."""
    for name in ("vehicle", "mission", "controller"):
        p = cfgdir / f"{name}.yaml"
        p.write_text(p.read_text(encoding="utf-8").replace("status: TBD", "status: assumed"),
                     encoding="utf-8")  # fmt: skip


# ---- the shipped files reproduce the code defaults ---------------------------------------------
def test_shipped_vehicle_yaml_reproduces_the_placeholder_vehicle_exactly():
    inp = load_inputs(CONFIGS)
    assert dataclasses.asdict(inp.vehicle) == dataclasses.asdict(placeholder_vehicle())
    assert inp.fallbacks == () and inp.mode == "draft" and inp.is_draft
    assert not inp.vehicle.released  # every shipped placeholder is TBD


def test_shipped_mission_controller_and_study_match_the_code_defaults():
    inp = load_inputs(CONFIGS)
    d = dataclasses.asdict(MissionProfile())
    got = dataclasses.asdict(inp.mission)
    for k, v in d.items():
        assert got[k] == pytest.approx(v), k
    assert dataclasses.asdict(inp.window) == dataclasses.asdict(BatteryWindow())
    bt, bl = BackTransitionParams(), BaselineParams(back=BackTransitionParams())
    c = inp.controller
    for k in ("T_bt", "K_h", "K_vh", "K_i", "v_dem_max", "i_max", "K_p", "K_v", "a_lim",
              "delta_min", "delta_max"):
        assert getattr(c, k) == pytest.approx(getattr(bt, k)), k
    assert c.theta_A == pytest.approx(bt.theta_A, rel=1e-12)
    assert math.degrees(inp.theta_p) == pytest.approx(35.0)
    assert inp.start_params["airspeed_min"] == bl.airspeed_min
    assert inp.start_params["q_transition"] == pytest.approx(bl.q_transition_s)
    assert inp.baseline_overrides["r_tecs"] == bl.r_tecs
    assert set(inp.scenarios) == {"S-01", "S-02", "S-03", "S-04", "S-05", "S-06"}
    assert inp.scenarios["S-03"].mass_kg == pytest.approx(1.15 * inp.vehicle.m)
    assert inp.requirements_km == (100.0, 113.0)


# ---- units, errors, fallbacks ------------------------------------------------------------------
def test_units_are_converted_once_at_load_into_each_dataclass_native_unit(cfgdir):
    edit(cfgdir / "vehicle.yaml", 'value: 8.0, unit: "kg"', 'value: 8000.0, unit: "g"')
    edit(cfgdir / "vehicle.yaml", 'value: 0.35, unit: "rad"', 'value: 20.0, unit: "deg"')
    edit(cfgdir / "vehicle.yaml", 'value: 25.0, unit: "Ah"', 'value: 12.5, unit: "Ah"')
    edit(cfgdir / "vehicle.yaml", 'value: 0.457, unit: "m"', 'value: 457.0, unit: "mm"')
    edit(cfgdir / "controller.yaml", 'value: 20.0, unit: "deg"',
         'value: 0.3490658503988659, unit: "rad"')
    inp = load_inputs(cfgdir)
    v = inp.vehicle
    assert v.m == pytest.approx(8.0)
    assert v.wing.alpha_s == pytest.approx(math.radians(20.0))
    assert v.battery.capacity_Ah_pack == pytest.approx(12.5)  # stored in Ah, not coulombs
    assert v.battery.e_nominal_wh == pytest.approx(2 * 12.5 * 43.2)
    assert v.rotor.D == pytest.approx(0.457)
    assert inp.controller.theta_A == pytest.approx(0.3490658503988659)


def test_every_problem_is_reported_not_just_the_first(cfgdir):
    edit(cfgdir / "vehicle.yaml", "  CL_alpha:", "  CL_alfa:")  # a typo
    edit(cfgdir / "study.yaml", "scenarios: [S-01, S-02]\n  v0_eas_ms",
         "scenarios: [S-01, S-99]\n  v0_eas_ms")
    (cfgdir / "mission.yaml").unlink()
    with pytest.raises(InputError) as exc:
        load_inputs(cfgdir)
    msgs = " | ".join(exc.value.problems)
    assert "missing file: mission.yaml" in msgs
    # the typo is only visible once the other files load, so fix that file and look again
    (cfgdir / "mission.yaml").write_text((CONFIGS / "mission.yaml").read_text(encoding="utf-8"),
                                         encoding="utf-8")  # fmt: skip
    with pytest.raises(InputError) as exc2:
        load_inputs(cfgdir)
    msgs2 = " | ".join(exc2.value.problems)
    assert "unknown parameter 'vehicle.wing.CL_alfa'" in msgs2 and "CL_alpha" in msgs2
    # problems surface in stages (files, settings, names, cross-references): fix and look again
    edit(cfgdir / "vehicle.yaml", "  CL_alfa:", "  CL_alpha:")
    with pytest.raises(InputError) as exc3:
        load_inputs(cfgdir)
    assert "unknown scenario 'S-99'" in " | ".join(exc3.value.problems)


def test_unknown_study_key_and_bad_values_are_rejected(cfgdir):
    edit(cfgdir / "study.yaml", "popsize: 8", "popsise: 8")
    with pytest.raises(InputError, match="popsise"):
        load_inputs(cfgdir)
    edit(cfgdir / "study.yaml", "popsise: 8", "popsize: 8")
    edit(cfgdir / "study.yaml", "mode: draft ", "mode: sloppy ")
    with pytest.raises(InputError, match="mode"):
        load_inputs(cfgdir)


def test_a_bad_register_entry_names_its_path(cfgdir):
    edit(cfgdir / "vehicle.yaml", 'status: TBD}\n\n# Wing', 'status: probably}\n\n# Wing')
    with pytest.raises(InputError, match="vehicle.mass.m"):
        load_inputs(cfgdir)


def test_missing_entry_falls_back_in_draft_and_is_recorded(cfgdir):
    text = (cfgdir / "vehicle.yaml").read_text(encoding="utf-8").splitlines()
    kept = [ln for ln in text if not ln.strip().startswith("CD0:")]
    (cfgdir / "vehicle.yaml").write_text("\n".join(kept), encoding="utf-8")
    inp = load_inputs(cfgdir)
    assert "vehicle.wing.CD0" in inp.fallbacks
    assert inp.vehicle.wing.CD0 == placeholder_vehicle().wing.CD0  # the built-in placeholder
    assert any("missing, placeholder used" in r for r in inp.draft_reasons)


def test_release_mode_refuses_while_anything_is_tbd_or_missing(cfgdir, tmp_path):
    out = study.run_study(cfgdir, mode="release", out_dir=tmp_path / "r", log=lambda s: None)
    assert out.exit_code == study.EXIT_INPUTS and out.folder is None
    assert any("not release grade" in m for m in out.messages)
    assert not (tmp_path / "r").exists()  # nothing was started or written
    # complete the register but drop one entry: still refused (a fallback is not release grade)
    complete(cfgdir)
    lines = (cfgdir / "vehicle.yaml").read_text(encoding="utf-8").splitlines()
    (cfgdir / "vehicle.yaml").write_text(
        "\n".join(ln for ln in lines if not ln.strip().startswith("k:")), encoding="utf-8")
    out2 = study.run_study(cfgdir, mode="release", out_dir=tmp_path / "r", log=lambda s: None)
    assert out2.exit_code == study.EXIT_INPUTS
    assert any("vehicle.wing.k" in m for m in out2.messages)


def test_a_complete_register_is_release_grade_and_records_say_released(cfgdir):
    complete(cfgdir)
    inp = load_inputs(cfgdir, mode="release")
    assert not inp.is_draft and inp.vehicle.released and not inp.draft_reasons
    assert make_record(inp.vehicle).label == "RELEASED"  # the analysis-level label follows
    assert make_record(placeholder_vehicle()).label == "DRAFT"
    # and dataclasses.replace (what scenarios do with the mass) keeps the flag
    assert make_record(dataclasses.replace(inp.vehicle, m=9.0)).label == "RELEASED"


# ---- hashing, statuses, selection --------------------------------------------------------------
def test_inputs_hash_tracks_data_and_settings_but_not_naming_mode_or_parallelism(cfgdir):
    h0 = load_inputs(cfgdir).inputs_hash
    edit(cfgdir / "study.yaml", "run_name: study", "run_name: other")
    edit(cfgdir / "study.yaml", "enabled: true", "enabled: false")
    assert load_inputs(cfgdir).inputs_hash == h0  # naming and parallelism do not change numbers
    assert load_inputs(cfgdir, mode="release").inputs_hash == h0
    edit(cfgdir / "vehicle.yaml", "value: 0.6,", "value: 0.61,")
    h1 = load_inputs(cfgdir).inputs_hash
    assert h1 != h0
    edit(cfgdir / "study.yaml", "popsize: 8", "popsize: 9")
    assert load_inputs(cfgdir).inputs_hash != h1


def test_sensitivity_grades_come_from_the_register(cfgdir):
    inp = load_inputs(cfgdir)
    assert inp.input_statuses()["mass"] == "TBD" and inp.input_statuses()["CN_flat_plate"] == "TBD"
    edit(cfgdir / "vehicle.yaml",
         'source: "CAD and weighing; define mass case (nominal / max / min)", status: TBD',
         'source: "weighed", status: measured')
    edit(cfgdir / "vehicle.yaml", 'status: TBD}\n  CL0', 'status: TBD, uncertainty: 0.3}\n  CL0')
    inp2 = load_inputs(cfgdir)
    model = stages.sensitivity_model(inp2)
    by = {i.name: i for i in model.inputs}
    assert by["mass"].status == "measured"
    assert by["wing_area"].lo == pytest.approx(0.6 * 0.7)
    assert by["wing_area"].hi == pytest.approx(0.6 * 1.3)


def test_only_selects_analyses_and_partial_results_are_never_reused(cfgdir, tmp_path):
    full = load_inputs(cfgdir)
    part = load_inputs(cfgdir, only={"A", "F"})
    assert stages.plan_tasks(part) == ["basics"]
    expected = {"basics", "C", "B2", "B1:S-01", "B1:S-02", "sensitivity"}
    assert set(stages.plan_tasks(full)) == expected
    assert stages.plan_tasks(full)[0] in ("B1:S-01", "B1:S-02")  # most expensive first
    work = tmp_path / "w"
    assert study._cache_path(work, full, "basics") != study._cache_path(work, part, "basics")
    assert part.study.analyses == ["A", "F"] and set(ALL_ANALYSES) >= set(part.study.analyses)


# ---- the command line ---------------------------------------------------------------------------
def test_cli_check_valid_invalid_and_release(cfgdir, capsys, tmp_path):
    assert cli.main(["--config", str(cfgdir), "--check", "--out", str(tmp_path)]) == 0
    text = capsys.readouterr().out
    assert "inputs are valid" in text and "tasks:" in text and "NOT release grade" in text
    assert cli.main(["--config", str(cfgdir), "--check", "--mode", "release"]) == study.EXIT_INPUTS
    assert "REFUSE" in capsys.readouterr().out
    (cfgdir / "scenarios.yaml").unlink()
    assert cli.main(["--config", str(cfgdir), "--check"]) == study.EXIT_INPUTS
    assert "missing file: scenarios.yaml" in capsys.readouterr().err
    assert cli.main(["--config", str(tmp_path / "nowhere"), "--check"]) == study.EXIT_INPUTS


# ---- end to end: one command, results out ------------------------------------------------------
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


def test_one_command_produces_a_complete_traceable_result_and_reuses_the_cache(cfgdir, tmp_path):
    (cfgdir / "study.yaml").write_text(SMOKE, encoding="utf-8")
    log: list[str] = []
    out = study.run_study(cfgdir, out_dir=tmp_path / "res", workers=1, log=log.append)
    assert out.exit_code == study.EXIT_OK, out.messages
    assert out.tasks == {"basics": "ran"} and out.label == "DRAFT"
    f = out.folder
    assert f is not None and f == tmp_path / "res" / "smoke"
    for name in ("report.md", "run_record.json", "inputs_used.md", "release_gate.txt",
                 "A_feasibility.csv", "D_a_eq.csv", "F_descent.csv", "E_summary.csv",
                 "Q2_pusher_margin.csv"):  # fmt: skip
        assert (f / name).is_file(), name
    # analysis A writes the VRS ratio (report only by default) and no NaN or inf anywhere
    a_win = (f / "A_window.csv").read_text(encoding="utf-8")
    a_feas = (f / "A_feasibility.csv").read_text(encoding="utf-8")
    assert "vrs_lo,vrs_hi" in a_win and "vrs_min,vrs_min_t_s" in a_feas
    for text in (a_win, a_feas):
        assert "nan" not in text.lower() and "inf" not in text.lower()
    assert "vrs" not in a_win.splitlines()[1].split(",")[6:8]  # report only: not in bind_lo/hi
    rec = json.loads((f / "run_record.json").read_text(encoding="utf-8"))
    assert rec["mode"] == "draft" and rec["label"] == "DRAFT" and rec["settings_hash"]
    assert rec["config_hash"] and rec["outputs"] and all(check_outputs(f).values())
    report = (f / "report.md").read_text(encoding="utf-8")
    assert "DRAFT" in report and "Q2 — pusher forward acceleration" in report
    assert "TBD" in (f / "inputs_used.md").read_text(encoding="utf-8")
    # second run: unchanged inputs are served from the cache and the outputs are byte-identical
    before = rec["outputs"]
    out2 = study.run_study(cfgdir, out_dir=tmp_path / "res", workers=1, log=lambda s: None)
    assert out2.tasks == {"basics": "cached"}
    assert json.loads((f / "run_record.json").read_text(encoding="utf-8"))["outputs"] == before
    # changing one input invalidates it
    edit(cfgdir / "vehicle.yaml", "value: 0.6,", "value: 0.62,")
    out3 = study.run_study(cfgdir, out_dir=tmp_path / "res", workers=1, log=lambda s: None)
    assert out3.tasks == {"basics": "ran"}
    # --force re-runs even with unchanged inputs
    out4 = study.run_study(cfgdir, out_dir=tmp_path / "res", workers=1, force=True,
                           log=lambda s: None)  # fmt: skip
    assert out4.tasks == {"basics": "ran"}


def test_release_mode_run_on_a_complete_register_labels_the_record_released(cfgdir, tmp_path):
    complete(cfgdir)
    (cfgdir / "study.yaml").write_text(SMOKE.replace("analyses: [A, D, F, E, Q2]", "analyses: [F]"),
                                       encoding="utf-8")  # fmt: skip
    out = study.run_study(cfgdir, mode="release", out_dir=tmp_path / "r", workers=1,
                          log=lambda s: None)  # fmt: skip
    assert out.label == "RELEASED"
    # the register is release grade, but the gate still wants verification results and a
    # sensitivity analysis, so a release-mode run ends with the gate verdict, not a pass
    assert out.gate is not None and not out.gate.passed
    assert out.exit_code == study.EXIT_GATE
    rec = json.loads((out.folder / "run_record.json").read_text(encoding="utf-8"))
    assert rec["label"] == "RELEASED" and rec["override"] is False and rec["mode"] == "release"


def test_an_input_switched_off_by_decision_is_dropped_from_the_sensitivity(cfgdir):
    text = (cfgdir / "vehicle.yaml").read_text(encoding="utf-8")
    (cfgdir / "vehicle.yaml").write_text(
        re.sub(r"(  k_mu: \{value: )[^,]+,", r"\g<1>0.0,", text), encoding="utf-8")
    model = stages.sensitivity_model(load_inputs(cfgdir))
    names = {i.name for i in model.inputs}
    assert "rotor_kmu" not in names and "mass" in names
    assert all(i.hi > i.lo for i in model.inputs)  # every remaining range can be sampled


def test_the_live_configs_folder_loads_and_passes_the_preflight():
    """Whatever real data is in configs/, it must stay loadable."""
    inp = load_inputs(LIVE_CONFIGS)
    assert inp.vehicle.m > 0 and inp.scenarios and inp.study.analyses
    assert all(i.hi > i.lo for i in stages.sensitivity_model(inp).inputs)
    assert stages.plan_tasks(inp)


def test_vrs_enforce_setting_reaches_analysis_a(cfgdir, tmp_path):
    (cfgdir / "study.yaml").write_text(
        SMOKE.replace("samples: 20}", "samples: 20, vrs_enforce: true}").replace(
            "analyses: [A, D, F, E, Q2]", "analyses: [A]"
        ),
        encoding="utf-8",
    )
    out = study.run_study(cfgdir, out_dir=tmp_path / "res", workers=1, log=lambda s: None)
    assert out.exit_code == study.EXIT_OK, out.messages
    rows = (out.folder / "A_window.csv").read_text(encoding="utf-8").splitlines()
    assert any("vrs" in r.split(",")[7] for r in rows[1:])  # the upper edge is now set by it
    # enforcing needs kappa: without it the task reports an error instead of guessing
    text = (cfgdir / "mission.yaml").read_text(encoding="utf-8")
    (cfgdir / "mission.yaml").write_text(
        "\n".join(ln for ln in text.splitlines() if not ln.strip().startswith("kappa:")),
        encoding="utf-8",
    )
    out2 = study.run_study(cfgdir, out_dir=tmp_path / "res2", workers=1, log=lambda s: None)
    report = (out2.folder / "report.md").read_text(encoding="utf-8") if out2.folder else ""
    assert "vrs_enforce needs mission.descent.kappa" in report
