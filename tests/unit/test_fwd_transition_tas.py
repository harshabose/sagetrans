"""E's forward transition reads AIRSPEED_MIN as an equivalent airspeed and converts it once."""

import dataclasses
import math
from pathlib import Path

import pytest

from sagetrans import atmosphere
from sagetrans.analysis import energy, pusher
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs

CONFIGS = Path(__file__).resolve().parents[1] / "data" / "placeholder_configs"
RHO0 = float(atmosphere.density(0.0))


def test_e_forward_segment_at_altitude_matches_q2s_end_speed():
    inp = load_inputs(CONFIGS)
    vp, soc = inp.vehicle, inp.mission.soc_mean
    mp = energy.mission_at_site(inp.mission, 5000.0, "eas")
    rho_c = float(atmosphere.density(mp.clearance_amsl, mp.isa_offset_k))
    v_end = inp.airspeed_min * math.sqrt(RHO0 / rho_c)  # Q2's end speed: the EAS as a TAS here
    assert v_end > 1.15 * inp.airspeed_min  # 5000 m: a real conversion, not a rounding
    assert mp.v_trans == pytest.approx(v_end)
    f = pusher.forward_accel(vp, rho_c, v_end, soc, inp.tkoff_thr_max, 21)
    seg = energy.forward_transition_estimate(vp, dataclasses.replace(mp, a_fwd=f.a_min), soc)
    assert seg.time_s == pytest.approx(v_end / f.a_min)
    assert seg.distance_m == pytest.approx(v_end**2 / (2.0 * f.a_min))
    # the same segment by hand from the true airspeed: the loaded EAS would be 21 % too slow here
    old = energy.forward_transition_estimate(
        vp, dataclasses.replace(mp, v_trans=inp.airspeed_min, a_fwd=f.a_min), soc
    )
    assert old.energy_J < seg.energy_J and old.distance_m < 0.7 * seg.distance_m


def test_site_mission_follows_the_q2_feed_convention():
    inp = load_inputs(CONFIGS)
    conv = inp.study.Q2.feed_convention
    expect = energy.mission_at_site(inp.mission, 1500.0, conv).v_trans
    assert stages.site_mission(inp, 1500.0).v_trans == expect


def test_the_site_mission_converts_once_and_the_tas_convention_keeps_the_value():
    inp = load_inputs(CONFIGS)
    mp = inp.mission
    eas = energy.mission_at_site(mp, 1500.0, "eas")
    rho_c = float(atmosphere.density(1500.0 + mp.clearance_agl_m, mp.isa_offset_k))
    assert eas.v_trans == pytest.approx(mp.v_trans * math.sqrt(RHO0 / rho_c))
    assert energy.mission_at_site(mp, 1500.0, "tas").v_trans == mp.v_trans
    # a loaded profile is converted by `mission_at_site` only, never by the loader
    assert mp.v_trans == inp.airspeed_min


def test_the_acceleration_e_uses_is_q2s_at_the_same_end_speed():
    inp = load_inputs(CONFIGS)
    vp = inp.vehicle
    mp = energy.mission_at_site(inp.mission, 2500.0, "eas")
    a = stages._fwd_accel_by_leg(inp, vp, mp, {"leg": vp.m}, {"leg": mp.soc_mean}, 0.0)
    rho_c = float(atmosphere.density(mp.clearance_amsl, mp.isa_offset_k))
    f = pusher.forward_accel(
        vp, rho_c, inp.airspeed_min * math.sqrt(RHO0 / rho_c), mp.soc_mean, inp.tkoff_thr_max,
        inp.study.Q2.n_speed,
    )
    assert a["leg"] == pytest.approx(f.a_min if inp.study.E.a_fwd_metric == "min" else f.a_equiv)
