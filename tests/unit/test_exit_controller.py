"""The stopping run is one run: D, E's back transition and the sensitivity's budget share it."""

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest

from sagetrans.analysis import braking, energy
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs

CONFIGS = Path(__file__).resolve().parents[1] / "data" / "placeholder_configs"


def test_d_exit_params_is_d_block_on_top_of_the_register():
    inp = load_inputs(CONFIGS)
    bp = stages.d_exit_params(inp)
    c = inp.study.D
    assert bp.theta_fw0 == pytest.approx(math.radians(c.theta_fw0_deg))
    assert bp.rotor_lag_s == c.rotor_lag_s and bp.ff_mode == c.ff_mode
    assert bp.v_min_ref == inp.airspeed_min and bp.alpha_cap_vmin == c.alpha_gate_ms
    assert bp != inp.controller  # the register's controller alone is not the run D makes
    # with the cap enforced the angle cap is the top of A's band, otherwise there is none
    capped = dataclasses.replace(inp, study=inp.study.model_copy(update={
        "D": c.model_copy(update={"enforce_alpha": True})}))
    assert stages.d_exit_params(capped).alpha_cap is not None
    assert bp.alpha_cap is None if not c.enforce_alpha else bp.alpha_cap is not None


def test_e_back_transition_runs_d_controller_from_the_airbrake_exit_state(monkeypatch):
    inp = load_inputs(CONFIGS)
    seen: list[tuple[object, np.ndarray]] = []
    real = braking.landing_error

    def spy(vp, scen, v0, a_plan, bp, speed_test="ground", **kw):
        seen.append((bp, kw["x0"]))
        return real(vp, scen, v0, a_plan, bp, speed_test, **kw)

    monkeypatch.setattr(energy.braking, "landing_error", spy)
    stages.solve_e_case(inp, 0.0, "one_way", 2.5, 1.0)
    want = stages.d_exit_params(inp)
    assert seen
    for bp, x0 in seen:
        assert bp == want  # D's controller, not `inp.controller`
        assert x0[4] == pytest.approx(want.theta_fw0)  # the AIRBRAKE exit pitch, not level
        assert x0[6] > 0.0  # the rotors carry what the wing does not


def test_the_legacy_default_controller_keeps_its_hover_start(monkeypatch):
    inp = load_inputs(CONFIGS)
    mp = stages.site_mission(inp, 0.0)
    kw_seen: list[dict] = []
    real = braking.landing_error

    def spy(*a, **kw):
        kw_seen.append(kw)
        return real(*a, **kw)

    monkeypatch.setattr(energy.braking, "landing_error", spy)
    energy.back_transition_segment(inp.vehicle, mp, 15.0, 0.8, 2.5, None)
    assert kw_seen and kw_seen[0]["x0"] is None  # no bp: the old state, unchanged


def test_the_sensitivity_budget_uses_the_same_controller():
    inp = load_inputs(CONFIGS)
    model = stages.sensitivity_model(inp)
    assert model.budget_bp == stages.d_exit_params(inp)
