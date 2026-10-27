"""Analysis D addenda (docs/D_fix_spec.md sections 14 and 15): the lumped actuator lag and the
recommendation as a curve over ground speed."""

import dataclasses
import inspect
from pathlib import Path
from typing import Any

import numpy as np
import pytest
from pydantic import ValidationError

from sagetrans.analysis import braking
from sagetrans.dynamics import trim
from sagetrans.io import stages
from sagetrans.io.inputs import StudyInputs, load_inputs
from sagetrans.io.settings import END_SPEED_MS, DCurveCfg
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


# ---- section 15: a_rec as a curve over ground speed ---------------------------------------------
SPEEDS = [4.0, 8.0, 12.0]


@pytest.fixture(scope="module")
def curve_run() -> dict[str, Any]:
    inp = _inputs()
    d = inp.study.D
    d2 = d.model_copy(update={
        "curve": DCurveCfg(ground_speeds_ms=SPEEDS),
        "recommendation": d.recommendation.model_copy(update={"design_ground_speed_ms": 6.0}),
    })  # fmt: skip
    out = stages.task_basics(
        dataclasses.replace(inp, study=inp.study.model_copy(update={"D": d2})))
    assert not out["errors"], out["errors"]
    return out["tables"]


def test_the_curve_has_one_row_per_listed_speed_and_the_expected_columns(curve_run):
    rows = curve_run["D_a_rec_curve"]
    assert [r["ground_speed_ms"] for r in rows] == SPEEDS
    cols = {"ground_speed_ms", "headwind_ms", "V0_air_ms", "a_rec_min_ms2", "set_by",
            "a_eq_median_ms2", "a_eq_max_ms2", "x_at_min_m", "t_stop_at_min_s",
            "excursion_max_m", "n_sinking", "n_runs", "n_nan"}  # fmt: skip
    for r in rows:
        assert cols <= set(r)
        assert r["n_runs"] == 1 and r["n_nan"] == 0  # one scenario x one mass factor here
        # the wing sees the entry rule's air speed: the headwind is the air speed minus ground speed
        assert r["headwind_ms"] == pytest.approx(r["V0_air_ms"] - r["ground_speed_ms"])


def test_the_curve_rises_with_ground_speed_and_its_statistics_are_ordered(curve_run):
    rows = curve_run["D_a_rec_curve"]
    mins = [r["a_rec_min_ms2"] for r in rows]
    assert mins == sorted(mins) and len(set(mins)) == len(mins)  # strictly increasing
    for r in rows:
        assert 0.0 < r["a_rec_min_ms2"] <= r["a_eq_median_ms2"] <= r["a_eq_max_ms2"]
        assert r["x_at_min_m"] > 0.0 and r["t_stop_at_min_s"] > 0.0 and r["excursion_max_m"] >= 0.0
        # a_eq is (V0^2 - v_f^2) / (2 x) for the run that sets the minimum
        v, vf = r["ground_speed_ms"], END_SPEED_MS
        assert r["a_rec_min_ms2"] == pytest.approx((v * v - vf * vf) / (2 * r["x_at_min_m"]))


def test_the_recommendation_row_reports_what_it_covers_and_the_design_value(curve_run):
    r = curve_run["D_recommendation"][0]
    for k in ("ground_speed_min_ms", "ground_speed_max_ms", "headwind_min_ms", "headwind_max_ms",
              "covers", "a_rec_design_ms2"):  # fmt: skip
        assert k in r
    assert r["ground_speed_min_ms"] <= r["ground_speed_max_ms"]
    assert r["covers"].startswith(
        f"ground speeds {r['ground_speed_min_ms']:.1f} to {r['ground_speed_max_ms']:.1f} m/s")
    assert "headwind" in r["covers"] and "air speed" in r["covers"]
    lo, hi = (x["a_rec_min_ms2"] for x in curve_run["D_a_rec_curve"][:2])
    assert r["a_rec_design_ms2"] == pytest.approx(0.5 * (lo + hi))  # 6 m/s is halfway from 4 to 8


def test_the_design_value_is_empty_when_no_design_speed_is_set():
    rows = _run()["tables"]["D_recommendation"]
    assert rows[0]["a_rec_design_ms2"] == ""
    assert "covers" in rows[0]


def test_curve_value_interpolates_linearly_and_refuses_to_extrapolate():
    speeds, vals = [10.0, 5.0, 20.0], [2.0, 1.0, 3.0]  # unsorted on purpose
    assert braking.curve_value(speeds, vals, 5.0) == 1.0
    assert braking.curve_value(speeds, vals, 12.0) == pytest.approx(2.2)
    assert braking.curve_value(speeds, vals, 20.0) == 3.0
    for bad in (4.9, 25.0):
        with pytest.raises(ValueError, match="outside the curve"):
            braking.curve_value(speeds, vals, bad)


def test_the_config_rejects_a_design_speed_outside_the_curve_and_speeds_below_the_end_speed():
    d = load_inputs(CONFIGS).study.D
    ok = d.recommendation.model_copy(update={"design_ground_speed_ms": 12.0})
    assert d.model_validate({**d.model_dump(), "recommendation": ok.model_dump()})
    far = d.recommendation.model_copy(update={"design_ground_speed_ms": 25.0})
    with pytest.raises(ValidationError, match="outside the curve"):
        d.model_validate({**d.model_dump(), "recommendation": far.model_dump()})
    for bad in ([1.0, 5.0], [0.5], []):
        with pytest.raises(ValidationError):
            DCurveCfg(ground_speeds_ms=bad)


def test_the_end_speed_of_the_curve_check_is_the_end_speed_of_the_runs():
    assert inspect.signature(braking.run_braking).parameters["v_f"].default == END_SPEED_MS
    assert inspect.signature(braking.airbrake_entry_state).parameters["v_f"].default == END_SPEED_MS


def test_the_shipped_defaults_are_the_spec_defaults():
    from sagetrans.io.settings import DCfg

    live = load_inputs(Path(__file__).resolve().parents[2] / "configs").study.D
    assert live.curve.ground_speeds_ms == DCfg().curve.ground_speeds_ms == [
        5, 6, 8, 10, 12, 14, 15, 16, 18, 20]  # fmt: skip
    assert live.recommendation.design_ground_speed_ms is None
