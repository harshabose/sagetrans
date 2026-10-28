"""PTCH_LIM_MAX_DEG is the ArduPilot default (20 deg) and the pitch cap follows it."""

import math
import shutil
from pathlib import Path

import pytest

from sagetrans.analysis import restricted
from sagetrans.control.ardupilot_like import BackTransitionParams
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs

ROOT = Path(__file__).resolve().parents[2]
LIVE = ROOT / "configs"
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"  # frozen: PTCH_LIM_MAX_DEG 35


def test_the_register_and_the_code_constants_are_the_ardupilot_default():
    inp = load_inputs(LIVE)
    assert math.degrees(inp.theta_p) == pytest.approx(20.0)
    assert restricted.THETA_P == pytest.approx(math.radians(20.0))
    assert BackTransitionParams().theta_P == pytest.approx(math.radians(20.0))
    assert inp.controller.theta_P == pytest.approx(math.radians(20.0))
    # the pitch cap is min(Q_A_ANGLE_MAX, PTCH_LIM_MAX_DEG): the register's 30 deg does nothing now
    assert min(inp.controller.theta_A, inp.controller.theta_P) == pytest.approx(math.radians(20.0))


def test_c_does_not_search_a_flat_region_above_the_pitch_limit():
    inp = load_inputs(LIVE)
    assert stages.c_theta_a_hi_deg(inp) == pytest.approx(20.0)  # min(30, 20)
    assert math.degrees(stages.c_bounds(inp).hi[1]) == pytest.approx(20.0)
    frozen = load_inputs(CONFIGS)  # PTCH_LIM_MAX_DEG 35: the 30 deg of DEC-7 still binds
    assert stages.c_theta_a_hi_deg(frozen) == pytest.approx(30.0)


def test_b1_c_envelope_may_not_exceed_the_pitch_limit(tmp_path):
    cfg = tmp_path / "configs"
    shutil.copytree(LIVE, cfg)
    study = cfg / "study.yaml"
    text = study.read_text(encoding="utf-8")
    old = "alpha_max_deg: 30.0, theta_cmd_max_deg: 20.0}"
    assert old in text
    study.write_text(text.replace(old, "alpha_max_deg: 30.0, theta_cmd_max_deg: 30.0}"),
                     encoding="utf-8")
    inp = load_inputs(cfg)
    with pytest.raises(ValueError, match="PTCH_LIM_MAX_DEG"):
        stages.task_b1(inp, "S-01")
    # the shipped value is the pitch limit itself
    shipped = next(v for v in load_inputs(LIVE).study.B1.variants if v.name == "c_envelope")
    assert shipped.theta_cmd_max_deg == pytest.approx(20.0)
