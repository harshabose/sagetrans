"""Analysis D addenda (docs/D_fix_spec.md sections 14 and 15): the lumped actuator lag and the
recommendation as a curve over ground speed."""

import dataclasses
from pathlib import Path
from typing import Any

import numpy as np
import pytest

from sagetrans.analysis import braking
from sagetrans.dynamics import trim
from sagetrans.io import stages
from sagetrans.io.inputs import StudyInputs, load_inputs
from sagetrans.physics.params import placeholder_vehicle
from sagetrans.physics.vehicle import CasadiVehicle

CONFIGS = Path(__file__).resolve().parents[2] / "tests" / "data" / "placeholder_configs"


def _inputs(**d_update: Any) -> StudyInputs:
    """Placeholder inputs with a small D stage (one scenario, a four-point ensemble)."""
    inp = load_inputs(CONFIGS)
    d = inp.study.D
    small = {
        "scenarios": ["S-01"], "mass_factors": [1.0], "v0_eas_ms": [16.0],
        "recommendation": d.recommendation.model_copy(
            update={"ensemble_n": 4, "corners": False, "n_boot": 20}),
        "trade": d.trade.model_copy(update={"a_plan_ms2": [2.0], "headwinds_ms": [0.0]}),
        "sens": d.sens.model_copy(update={
            "theta_fw0_deg_sweep": [], "lift_tau_s_sweep": [], "lift_scale_sweep": []}),
    }  # fmt: skip
    d2 = d.model_copy(update={**small, **d_update})
    study = inp.study.model_copy(update={"analyses": ["D"], "D": d2})
    return dataclasses.replace(inp, study=study)


def _run(**d_update: Any) -> dict[str, Any]:
    out = stages.task_basics(_inputs(**d_update))
    assert not out["errors"], out["errors"]
    return out


# ---- section 14: one lumped actuator lag --------------------------------------------------------
def test_rotor_speed_tau_matches_the_full_model_and_scales_with_i_rot():
    vp = placeholder_vehicle()
    rho, soc = 1.225, 0.8
    tau = braking.rotor_speed_tau(vp, rho, soc)
    # independent check on the full vehicle (the bus voltage follows the rotor speed there, so the
    # two agree to a few percent rather than exactly)
    veh = CasadiVehicle(vp)
    w0 = trim.hover_omega(vp, rho)
    d0 = trim.hover_state(vp, rho, soc).delta_r_cmd

    def omega_dot(w: float) -> float:
        x = np.zeros(9)
        x[6] = w
        return float(veh.derivatives(x, np.array([d0, 0.0, 0.0]), np.array([0, 0, 0, soc]))[0][6])

    assert abs(omega_dot(w0)) < 1e-2  # hover is an equilibrium
    h = 1e-3 * w0
    full = -2.0 * h / (omega_dot(w0 + h) - omega_dot(w0 - h))
    assert tau > 0.0 and tau == pytest.approx(full, rel=0.10)
    big = dataclasses.replace(vp, rotor=dataclasses.replace(vp.rotor, I_rot=3.0 * vp.rotor.I_rot))
    assert braking.rotor_speed_tau(big, rho, soc) == pytest.approx(3.0 * tau, rel=0.02)


def test_the_lag_sweep_gains_0p1_and_every_row_carries_the_time_constants():
    assert load_inputs(CONFIGS).study.D.sens.lag_s_sweep == [0.0, 0.1, 0.25, 0.5]
    rows = _run()["tables"]["D_sensitivity"]
    cases = [r["case"] for r in rows]
    assert [c for c in cases if c.startswith("rotor_lag")] == [
        "rotor_lag_0s", "rotor_lag_0.1s", "rotor_lag_0.25s", "rotor_lag_0.5s"]  # fmt: skip
    for r in rows:
        assert r["tau_total_s"] == pytest.approx(r["rotor_lag_s"] + r["tau_rotor_model_s"])
    taus = {r["tau_rotor_model_s"] for r in rows}
    assert len(taus) == 1 and taus.pop() > 0.0  # the same on every row
    lag = {r["case"]: r["rotor_lag_s"] for r in rows}
    assert lag["base"] == 0.0 and lag["rotor_lag_0.25s"] == 0.25


def test_a_lag_adds_excursion_and_a_zero_lag_is_the_default():
    rows = {r["case"]: r for r in _run()["tables"]["D_sensitivity"]}
    a0, a5 = rows["rotor_lag_0s"], rows["rotor_lag_0.5s"]
    for k in ("a_eq_ms2", "x_actual_m", "alt_gain_m", "alt_loss_m", "vh_end_ms"):
        assert a0[k] == rows["base"][k]  # a zero lag reproduces the default run bit for bit
    assert a5["a_eq_ms2"] != a0["a_eq_ms2"]
    assert a5["alt_gain_m"] + a5["alt_loss_m"] > a0["alt_gain_m"] + a0["alt_loss_m"]
