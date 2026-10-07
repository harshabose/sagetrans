"""Guard against the YAML register and the physics placeholders drifting apart.

The physics reads `physics/params.py`, not the register (the register is not linked yet), so the
two can disagree silently. Until they are linked, every register entry that carries a value must
have a declared physics counterpart that agrees with it; adding a value to the register without
wiring it here fails this test.
"""

from pathlib import Path

import pytest

from sagetrans.config.schema import Config, Status
from sagetrans.control.ardupilot_like import BackTransitionParams, BaselineParams
from sagetrans.physics.params import placeholder_vehicle

ROOT = Path(__file__).resolve().parents[2]
VP = placeholder_vehicle()
CFG = Config.from_yaml(ROOT / "configs" / "vehicle.yaml")

# register path -> the physics value it must equal (both in SI)
COUNTERPARTS = {
    "rotors.n_lift": float(VP.rotor.n_r),
    "battery.packs_parallel": float(VP.battery.n_parallel),
    "battery.v_nominal_V": VP.battery.v_nominal,
    "battery.capacity_Ah_per_pack": VP.battery.capacity_Ah_pack * 3600.0,  # Ah -> coulombs
    "ardupilot.Q_TRANSITION_MS": BaselineParams(back=BackTransitionParams()).q_transition_s,
}


def test_every_valued_register_entry_has_a_declared_physics_counterpart():
    valued = [
        p for p in CFG if CFG.record(p).status is not Status.TBD and CFG.record(p).value is not None
    ]
    assert sorted(valued) == sorted(COUNTERPARTS), (
        "a register value has no declared physics counterpart (or a counterpart lost its value)"
    )


@pytest.mark.parametrize("path", sorted(COUNTERPARTS))
def test_register_value_agrees_with_the_physics_default(path):
    assert CFG.get(path) == pytest.approx(COUNTERPARTS[path], rel=1e-12)


def test_nominal_pack_energy_agrees():
    # 2 packs x 25 Ah x 43.2 V = 2160 Wh, computed from the register and from the physics
    from_register = (
        CFG.get("battery.packs_parallel")
        * CFG.get("battery.capacity_Ah_per_pack")  # coulombs
        * CFG.get("battery.v_nominal_V")
    )
    assert from_register == pytest.approx(VP.battery.e_nominal_wh * 3600.0)


def test_register_files_that_do_not_exist_are_reported():
    # configs/vehicle.yaml points at maps/ that are not in the repository yet
    assert CFG.missing_files() == ["battery.ecm_file", "rotors.ct_map"]
    complete = Config.from_dict(
        {"mass": {"m_kg": {"value": 8.0, "unit": "kg", "source": "scale", "status": "measured"}}}
    )
    assert complete.missing_files() == []
