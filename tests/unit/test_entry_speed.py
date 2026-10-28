"""The back transition has one entry speed: the AIRBRAKE exit, in D, C, B1 and E."""

import dataclasses
import math
from pathlib import Path

import pytest

from sagetrans import atmosphere
from sagetrans.analysis import braking, energy
from sagetrans.io import stages
from sagetrans.io.inputs import StudyInputs, load_inputs

CONFIGS = Path(__file__).resolve().parents[1] / "data" / "placeholder_configs"


def test_e_back_transition_starts_at_the_airbrake_exit(monkeypatch):
    inp = load_inputs(CONFIGS)
    seen: list[float] = []
    real = energy.back_transition_segment

    def spy(vp, mp, v0, soc, a_plan, bp=None):
        seen.append(v0)
        return real(vp, mp, v0, soc, a_plan, bp)

    monkeypatch.setattr(energy, "back_transition_segment", spy)
    for site in (0.0, 2500.0):
        seen.clear()
        stages.solve_e_case(inp, site, "one_way", 2.5, 1.0)
        rho_bt = float(atmosphere.density(site + inp.mission.h_bt_agl_m, inp.mission.isa_offset_k))
        want = braking.entry_airspeed(inp.airspeed_min, rho_bt, inp.study.D.entry_convention)
        assert seen and all(v == pytest.approx(want) for v in seen)
        if inp.study.D.entry_convention == "eas":
            by_hand = (inp.airspeed_min - 2.0) * math.sqrt(1.225 / rho_bt)
            assert want == pytest.approx(by_hand, rel=2e-3)


def test_the_register_has_no_separate_entry_speed():
    inp = load_inputs(CONFIGS)
    assert "mission.speeds.entry_eas_ms" not in inp.config
    assert "entry_eas" not in {f.name for f in dataclasses.fields(StudyInputs)}
