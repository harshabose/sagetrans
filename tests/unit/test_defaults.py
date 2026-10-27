"""The ArduPilot-defaults comparison, step 1 (docs/DEFAULTS_spec.md): register file and values."""

import math
import shutil
from pathlib import Path

import pytest

from sagetrans.io import recommended
from sagetrans.io.inputs import load_inputs, resolve_default

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
# a frozen copy of the shipped file, so a change to the live values does not move these tests
DEFAULTS_FILE = ROOT / "tests" / "data" / "ardupilot_defaults.yaml"


@pytest.fixture()
def cfgdir(tmp_path: Path) -> Path:
    """A private copy of the placeholder configs with the defaults file added."""
    dst = tmp_path / "configs"
    shutil.copytree(CONFIGS, dst)
    shutil.copy(DEFAULTS_FILE, dst / "ardupilot_defaults.yaml")
    return dst


def test_the_defaults_file_loads_with_the_register_loader_and_converts_units(cfgdir):
    inp = load_inputs(cfgdir)
    assert inp.defaults["AIRSPEED_MIN"] == 9.0
    assert inp.defaults["Q_TRANSITION_MS"] == pytest.approx(5.0)  # ms converted to s
    assert inp.defaults["Q_BACKTRANS_MS"] == pytest.approx(3.0)
    assert inp.defaults["Q_TRAN_PIT_MAX"] == pytest.approx(math.radians(3.0))  # deg to rad
    assert inp.status("ardupilot_defaults.ardupilot.AIRSPEED_MIN") == "documented"
    assert "ardupilot_defaults.ardupilot.AIRSPEED_MIN" in inp.config  # hashed with the register
    assert not any("ardupilot_defaults" in r for r in inp.draft_reasons)  # nothing TBD in it


def test_a_missing_defaults_file_is_not_an_error():
    inp = load_inputs(CONFIGS)  # the frozen placeholder folder has no defaults file
    assert inp.defaults == {} and inp.resolve_default("AIRSPEED_MIN") is None
    rows = recommended.settings_rows(inp, {})
    assert rows and all(r["default"] == "" and r["change_vs_default"] == "" for r in rows)


def test_use_another_parameter_defaults_are_resolved():
    inp_defaults = {"TKOFF_THR_MAX": 0.0, "THR_MAX": 1.0, "Q_A_ANGLE_MAX": 0.0}
    assert resolve_default(inp_defaults, "TKOFF_THR_MAX") == 1.0
    assert resolve_default({**inp_defaults, "THR_MAX": 0.9}, "TKOFF_THR_MAX") == 0.9  # follows it
    assert resolve_default(inp_defaults, "Q_A_ANGLE_MAX") == pytest.approx(math.radians(30.0))
    assert resolve_default({"TKOFF_THR_MAX": 0.7}, "TKOFF_THR_MAX") == 0.7  # a set value stands


def test_a_missing_default_gives_none_never_a_made_up_value(cfgdir):
    assert resolve_default({}, "AIRSPEED_MIN") is None
    assert resolve_default({"TKOFF_THR_MAX": 0.0}, "TKOFF_THR_MAX") is None  # THR_MAX is missing
    inp = load_inputs(cfgdir)
    assert inp.resolve_default("NOT_A_PARAMETER") is None


def test_the_settings_sheet_has_the_default_column_and_the_change_vs_default(cfgdir):
    inp = load_inputs(cfgdir)
    rows = {r["parameter"]: r for r in recommended.settings_rows(inp, {})}
    assert {"Q_BACKTRANS_MS", "Q_A_ANGLE_MAX", "Q_M_SPIN_MIN", "Q_TRANS_DECEL", "AIRSPEED_MIN",
            "Q_TRANSITION_MS", "TKOFF_THR_MAX"} <= set(rows)
    assert rows["AIRSPEED_MIN"]["default"] == 9.0 and rows["AIRSPEED_MIN"]["default_unit"] == "m/s"
    assert rows["Q_TRANSITION_MS"]["default"] == pytest.approx(5000.0)  # shown in ms
    assert rows["Q_A_ANGLE_MAX"]["default"] == pytest.approx(30.0)  # shown in degrees
    assert rows["TKOFF_THR_MAX"]["default"] == 1.0  # resolved ...
    assert rows["TKOFF_THR_MAX"]["default_text"] == "0 (= THR_MAX)"  # ... and shown in words
    assert "Plane-4.7.2" in rows["AIRSPEED_MIN"]["default_source"]
    # no analysis ran: the register value stands, marked as such, and the change is the register's
    r = rows["AIRSPEED_MIN"]
    assert r["analysis"] == "" and r["note"] == "not yet analysed"
    assert r["value"] == r["register"]
    assert r["change_vs_default"] == pytest.approx(r["register"] - 9.0)
    # Q_TRANS_DECEL has no register value: nothing to compare until an analysis decides it
    assert rows["Q_TRANS_DECEL"]["value"] == "" and rows["Q_TRANS_DECEL"]["change_vs_default"] == ""


def test_a_recommended_value_replaces_the_register_value_and_the_change_follows_it(cfgdir):
    inp = load_inputs(cfgdir)
    tables = {
        "C_params": [{"case": "optimum", "T_bt_s": 1.5, "theta_A_deg": 30.0, "spin_min": 0.12,
                      "Q_TRANS_DECEL_ms2": 2.9, "at_bound": "theta_A:hi"}],
        "D_recommendation": [{"a_rec_min_ms2": 2.4}],
    }
    rows = recommended.settings_rows(inp, tables)
    by = {(r["parameter"], r["analysis"]): r for r in rows}
    bt = by[("Q_BACKTRANS_MS", "C")]
    assert bt["value"] == pytest.approx(1500.0)
    assert bt["change_vs_default"] == pytest.approx(-1500.0)
    assert bt["searched_range"] == "1000 to 6000"
    q = by[("Q_TRANS_DECEL", "C")]
    assert q["value"] == 2.9 and q["change_vs_default"] == pytest.approx(0.9)
    assert by[("Q_A_ANGLE_MAX", "C")]["flags"] == "at upper bound"
    assert by[("Q_TRANS_DECEL", "D")]["value"] == 2.4  # D's recommendation is a second row
    assert ("Q_BACKTRANS_MS", "") not in by  # an analysed parameter has no "not yet analysed" row
