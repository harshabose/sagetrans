"""Analysis FT (docs/FT_spec.md): configuration, the z_loop split and, in later steps, the model."""

from pathlib import Path

import pytest
from pydantic import ValidationError

from sagetrans import atmosphere
from sagetrans.analysis.common import Scenario
from sagetrans.control.ardupilot_like import BackTransitionParams, LiftFF, Position1Controller
from sagetrans.io.inputs import load_inputs
from sagetrans.io.settings import FTBoundsCfg, FTCfg, StudyCfg
from sagetrans.physics.params import placeholder_vehicle

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
VP = placeholder_vehicle()


# ---- settings (spec 8.10) -----------------------------------------------------------------------
def test_ft_bounds_must_be_ordered_and_the_throttle_ceiling_positive():
    with pytest.raises(ValidationError, match="airspeed_min_eas_ms"):
        FTBoundsCfg(airspeed_min_eas_ms=(20.0, 14.0))
    with pytest.raises(ValidationError, match="q_transition_s"):
        FTBoundsCfg(q_transition_s=(5.0, 5.0))
    with pytest.raises(ValidationError, match="tkoff_thr_max"):
        FTBoundsCfg(tkoff_thr_max=(0.0, 1.0))
    FTBoundsCfg()  # the shipped defaults are valid


def test_ft_settings_reject_unknown_keys_and_a_reversed_headwind_range():
    with pytest.raises(ValidationError):
        FTCfg(not_a_setting=1)
    with pytest.raises(ValidationError, match="headwind_ms"):
        FTCfg(headwind_ms=[5.0, 0.0])
    with pytest.raises(ValidationError, match="headwind_ms"):
        FTCfg(headwind_ms=[0.0])
    assert FTCfg().stall_margin == 1.2 and FTCfg().weights.w_x == 0.0


def test_ft_is_a_known_analysis_with_the_spec_defaults():
    cfg = StudyCfg()
    assert "FT" in cfg.analyses and cfg.analyses.index("FT") == cfg.analyses.index("Q2") + 1
    assert cfg.FT.bounds.q_tran_pit_max_deg == (1.0, 10.0)
    assert cfg.FT.scan.airspeed_min_eas_ms == [15.0, 16.0, 17.0, 18.0, 19.0, 20.0]
    assert cfg.FT.sens.tecs_start == ["warm", "cold"]


def test_new_mission_fields_load_and_the_unknown_key_check_passes():
    inp = load_inputs(CONFIGS)
    assert inp.mission.fwd_alt_loss_max_m == 10.0 and inp.mission.fwd_alt_gain_max_m == 10.0
    assert inp.mission.clearance_agl_m == 30.0
    for name in ("fwd_alt_loss_max_m", "fwd_alt_gain_max_m"):
        assert inp.status(f"mission.profile.{name}") == "assumed"


def test_new_controller_and_tecs_entries_are_known_register_paths():
    live = load_inputs(ROOT / "configs")
    for path in (
        "controller.ardupilot.Q_TRAN_PIT_MAX", "controller.ardupilot.TRIM_THROTTLE",
        "controller.ardupilot.TKOFF_THR_MIN", "controller.tecs.TECS_VERT_ACC",
        "controller.tecs.TECS_PITCH_MAX",
    ):
        assert path in live.config, path
    assert live.config.record("controller.tecs.TECS_TIME_CONST").si_value == 5.0


# ---- z_loop refactor (spec 4.1, 8.9) ------------------------------------------------------------
def test_z_loop_split_gives_the_same_command_as_the_altitude_error_form():
    sc = Scenario(0.0)
    rho = float(atmosphere.density(0.0))
    ff = LiftFF(VP, rho, sc.soc, t_max_factor=3.0, n=30)
    bp = BackTransitionParams(
        z_mode="ardupilot", h_tgt=10.0, delta_max=0.95, rotor_lag_s=0.1, ff_mode="repo"
    )
    t_max = ff.thrust_for_delta(bp.delta_max)
    ctl = Position1Controller(bp, ff, z_i0=ff.weight / t_max)
    h, vh, theta = 7.5, -0.4, 0.1
    v_t = min(max(bp.z_pos_p * (bp.h_tgt - h), -bp.z_vmax), bp.z_vmax)
    prev: dict[str, float] = {}
    prev2: dict[str, float] = {}
    for k in range(6):  # first call initialises, the rest run the loop; both forms stay equal
        a, prev = ctl._z_loop(0.02 if k else 0.0, h, vh + 0.01 * k, theta, prev)
        b, prev2 = ctl.z_loop(0.02 if k else 0.0, v_t, vh + 0.01 * k, theta, prev2)
        assert a == b and prev == prev2
