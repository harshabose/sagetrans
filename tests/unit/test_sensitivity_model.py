"""The sensitivity runs the same back transition as the headline (D and C): the AIRBRAKE exit."""

import math
from pathlib import Path

import pytest

from sagetrans.analysis import restricted
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs

CONFIGS = Path(__file__).resolve().parents[1] / "data" / "placeholder_configs"


def test_the_model_baseline_is_the_c_register_run_of_the_same_scenario():
    inp = load_inputs(CONFIGS)
    model = stages.sensitivity_model(inp)
    c = inp.study.sensitivity
    scen = inp.scenarios[c.scenario] if c.scenario in inp.scenarios else next(
        iter(inp.scenarios.values()))
    cev, base, _ = stages.c_settings(inp)
    reg = {**{k: inp.start_params[k] for k in ("T_bt", "theta_A", "spin_min")},
           "q": restricted.C_Q_START}  # fmt: skip
    head = restricted.run_c(inp.vehicle, scen, reg, cev, base)
    assert head.stopped  # a baseline that does not stop would make the comparison empty
    out = model({})
    assert out["excursion_m"] == pytest.approx(head.metrics.excursion, rel=1e-9)
    assert out["distance_m"] == pytest.approx(head.metrics.distance, rel=1e-9)
    assert out["energy_J"] == pytest.approx(head.metrics.energy, rel=1e-9)
    assert model.c_eval is not None
    assert model.c_eval.entry_convention == inp.study.D.entry_convention


def test_the_inputs_belong_to_the_loop_that_is_simulated():
    inp = load_inputs(CONFIGS)
    model = stages.sensitivity_model(inp)
    names = {i.name for i in model.inputs}
    assert {"z_vel_p", "z_acc_p", "z_acc_i"} <= names
    assert not any(n.startswith("alt_hold_") for n in names)  # inert with the ArduPilot Z loop
    nominal = model({})
    z = next(i for i in model.inputs if i.name == "z_vel_p")
    moved = model({"z_vel_p": z.hi})
    assert any(
        math.isfinite(moved[k]) and moved[k] != nominal[k]
        for k in ("excursion_m", "distance_m", "energy_J")
    )  # a perturbed Z-loop gain does something
