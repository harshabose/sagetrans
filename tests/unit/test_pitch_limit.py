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
    old = "alpha_max_deg: 30.0, theta_cmd_max_deg: null}"
    assert old in text
    study.write_text(text.replace(old, "alpha_max_deg: 30.0, theta_cmd_max_deg: 30.0}"),
                     encoding="utf-8")
    inp = load_inputs(cfg)
    with pytest.raises(ValueError, match="PTCH_LIM_MAX_DEG"):
        stages.task_b1(inp, "S-01")
    # the shipped value is the pitch limit itself: null, which follows the register
    shipped = next(v for v in load_inputs(LIVE).study.B1.variants if v.name == "c_envelope")
    assert shipped.theta_cmd_max_deg is None
    assert stages.b1_ceiling_deg(load_inputs(LIVE), shipped) == pytest.approx(20.0)


# ---- a changed register value reaches every path ---------------------------------------------
def _inp_with_pitch_limit(tmp_path, deg: float):
    cfg = tmp_path / "configs"
    shutil.copytree(LIVE, cfg)
    ctl = cfg / "controller.yaml"
    text = ctl.read_text(encoding="utf-8")
    old = "PTCH_LIM_MAX_DEG: {value: 20.0,"
    assert old in text
    ctl.write_text(text.replace(old, f"PTCH_LIM_MAX_DEG: {{value: {deg:g},"), encoding="utf-8")
    return load_inputs(cfg)


def test_a_register_pitch_limit_of_25_deg_reaches_c_d_b1_and_sensitivity(tmp_path, monkeypatch):
    from sagetrans.analysis import braking
    from sagetrans.analysis import collocation as coll

    inp = _inp_with_pitch_limit(tmp_path, 25.0)
    want = math.radians(25.0)
    assert inp.theta_p == pytest.approx(want) and inp.controller.theta_P == pytest.approx(want)

    # C: the base parameters, the controller the runs build, and the bound of Q_A_ANGLE_MAX
    cev, c_base, _ = stages.c_settings(inp)
    assert c_base.theta_P == pytest.approx(want)
    assert stages.c_theta_a_hi_deg(inp) == pytest.approx(25.0)  # min(30, 25)
    assert math.degrees(stages.c_bounds(inp).hi[1]) == pytest.approx(25.0)
    high = _inp_with_pitch_limit(tmp_path / "b", 40.0)
    assert stages.c_theta_a_hi_deg(high) == pytest.approx(30.0)  # the 30 deg of DEC-7 binds

    # D: the exit parameters, and the controller of an actual stopping run
    d_bp = stages.d_exit_params(inp)
    assert d_bp.theta_P == pytest.approx(want)
    built: list[float] = []
    for mod in (braking, restricted):
        real = mod.Position1Controller

        def spy(bp, *a, _real=real, **kw):
            built.append(bp.theta_P)
            return _real(bp, *a, **kw)

        monkeypatch.setattr(mod, "Position1Controller", spy)
    scen, vp = inp.scenarios["S-01"], inp.vehicle
    x0, v_ground = braking.airbrake_entry_state(vp, scen, d_bp, 15.0)
    braking.measure_a_eq(vp, scen, v_ground, d_bp, x0=x0)  # a D run
    prm = {**{k: inp.start_params[k] for k in ("T_bt", "theta_A", "spin_min")}, "q": 2.0}
    restricted.run_c(vp, scen, prm, cev, c_base)  # a C run
    assert built == [pytest.approx(want)] * 2

    # B1: the problem's pitch limit, and the c_envelope ceiling, which follows the register
    seen: list[float] = []

    def stop(vp, scen, base, v_air, opt, controller, theta_p, rotor_start="carry"):
        seen.append(theta_p)
        raise ValueError("stop after the first problem")

    monkeypatch.setattr(coll, "make_problem_exit", stop)
    out = stages.task_b1(inp, "S-01")
    assert seen == [pytest.approx(want)] and out["errors"]
    env = next(v for v in inp.study.B1.variants if v.name == "c_envelope")
    assert stages.b1_ceiling_deg(inp, env) == pytest.approx(25.0)
    assert stages.b1_ceiling_deg(load_inputs(LIVE), env) == pytest.approx(20.0)
    attached = next(v for v in inp.study.B1.variants if v.name == "attached")
    assert stages.b1_ceiling_deg(inp, attached) == pytest.approx(34.4)  # a number stays a number

    # the sensitivity: the model's controller and its fixed pitch limit
    model = stages.sensitivity_model(inp)
    assert model.theta_p == pytest.approx(want)
    assert model.c_base is not None and model.c_base.theta_P == pytest.approx(want)
    assert model.budget_bp is not None and model.budget_bp.theta_P == pytest.approx(want)
