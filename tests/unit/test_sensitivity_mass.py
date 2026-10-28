"""The sensitivity's `mass` input acts on every output, under a scenario with its own mass too."""

from pathlib import Path

import pytest

from sagetrans.analysis import sensitivity as sens
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs

CONFIGS = Path(__file__).resolve().parents[1] / "data" / "placeholder_configs"


def test_apply_inputs_scales_the_scenario_mass_with_the_vehicle_mass():
    inp = load_inputs(CONFIGS)
    scen = inp.scenarios["S-03"]  # flies at its own mass (a fraction of the vehicle's)
    assert scen.mass_kg is not None and scen.mass_kg != inp.vehicle.m
    inputs = sens.default_inputs(inp.vehicle, scen)
    mass = next(i for i in inputs if i.name == "mass")
    vp, sc, _ = sens.apply_inputs(inp.vehicle, scen, inputs, {"mass": mass.hi})
    assert vp.m == pytest.approx(mass.hi)
    assert sc.mass_kg == pytest.approx(scen.mass_kg * mass.hi / inp.vehicle.m)
    # nominal in, nominal out; a scenario without a mass is left alone
    vp0, sc0, _ = sens.apply_inputs(inp.vehicle, scen, inputs, {})
    assert sc0.mass_kg == scen.mass_kg and vp0.m == inp.vehicle.m


def test_the_mass_input_moves_the_back_transition_outputs():
    inp = load_inputs(CONFIGS, only={"sensitivity"})
    model = stages.sensitivity_model(inp)
    mass = next(i for i in model.inputs if i.name == "mass")
    base, light, heavy = model({}), model({"mass": mass.lo}), model({"mass": mass.hi})
    for out in ("excursion_m", "distance_m", "energy_J"):
        assert light[out] != base[out] and heavy[out] != base[out], out
    assert heavy["energy_J"] > light["energy_J"]  # a heavier aircraft costs more to stop
