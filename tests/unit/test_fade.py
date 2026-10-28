"""E's forward transition includes the rotor fade after the hand-over (Q_TRANSITION_MS)."""

from dataclasses import replace

import pytest

from sagetrans.analysis import energy
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs
from sagetrans.physics.params import placeholder_vehicle

VP = placeholder_vehicle()
MP = energy.MissionProfile(v_trans=17.0, a_fwd=2.5, wing_share_handover=0.4)


def test_without_a_fade_time_the_estimate_is_the_older_one():
    plain = energy.forward_transition_estimate(VP, MP, 0.6)
    assert energy.fade_estimate(VP, MP, 0.6).time_s == 0.0
    assert plain.time_s == pytest.approx(MP.v_trans / MP.a_fwd)
    assert "rotor fade" not in plain.source


def test_the_fade_adds_its_time_energy_and_distance_to_the_segment():
    base = energy.forward_transition_estimate(VP, MP, 0.6)
    mp5 = replace(MP, q_transition_s=5.0)
    seg = energy.forward_transition_estimate(VP, mp5, 0.6)
    fade = energy.fade_estimate(VP, mp5, 0.6)
    assert fade.time_s == 5.0 and fade.energy_J > 0.0
    assert seg.time_s == pytest.approx(base.time_s + 5.0)
    assert seg.energy_J == pytest.approx(base.energy_J + fade.energy_J)
    assert seg.distance_m == pytest.approx(base.distance_m + 5.0 * MP.v_trans)
    assert "rotor fade over 5 s" in seg.source


def test_a_longer_fade_costs_more_and_a_larger_wing_share_costs_less():
    e = [energy.fade_estimate(VP, replace(MP, q_transition_s=q), 0.6).energy_J
         for q in (2.0, 5.0, 12.0)]
    assert e[0] < e[1] < e[2]
    carry = [energy.fade_estimate(VP, replace(MP, q_transition_s=5.0, wing_share_handover=s), 0.6)
             for s in (0.2, 0.4, 0.8)]
    assert carry[0].energy_J > carry[1].energy_J > carry[2].energy_J  # the rotors carry less


def test_the_register_and_the_recommended_fade_reach_e_through_the_site_mission():
    inp = load_inputs("configs", only={"E"})
    reg = stages.site_mission(inp, 0.0)
    assert reg.q_transition_s == pytest.approx(inp.start_params["q_transition"])
    assert reg.wing_share_handover == inp.study.E.fade_wing_share
    rec = stages.site_mission(stages.with_settings(inp, 18.0, 1.0, 17.0), 0.0)
    assert rec.q_transition_s == 17.0 and rec.v_trans != reg.v_trans
    # a longer fade makes E's forward segment dearer, so register vs recommended can go either way
    fe = [energy.forward_transition_estimate(inp.vehicle, m, 0.6).energy_J for m in (reg, rec)]
    assert fe[1] > energy.forward_transition_estimate(
        inp.vehicle, replace(rec, q_transition_s=0.0), 0.6).energy_J
    assert fe[0] > 0.0
