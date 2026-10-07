import math
from dataclasses import replace

import numpy as np
import pytest

from sagetrans import atmosphere
from sagetrans.analysis import braking, zero_loss
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.zero_loss import DecelProfile, analyse, solve_point
from sagetrans.control.ardupilot_like import BackTransitionParams, Position1Controller
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import simulate
from sagetrans.physics.params import (
    LiftRotorParams,
    VehicleParams,
    WingParams,
    placeholder_vehicle,
)
from sagetrans.physics.vehicle import CasadiVehicle

G = atmosphere.G0
VP = placeholder_vehicle()
SC = Scenario(0.0)
RHO = SC.rho


# ---- analysis A ------------------------------------------------------------------------------
def test_a_constant_deceleration_closed_form_no_aero():
    vp = VehicleParams(wing=WingParams(S=0.0), rotor=LiftRotorParams(CH1=0.0))
    for a in (0.5, 2.0, 5.0):
        p = solve_point(vp, RHO, 20.0, a)
        assert p.theta == pytest.approx(math.atan(a / G), abs=1e-9)
        assert p.T_r == pytest.approx(vp.m * math.hypot(a, G), rel=1e-9)


def test_a_coast_bound_gives_level_pitch_and_weight_thrust():
    # C_L = 0 and a = D/m: drag alone provides the deceleration, rotors just carry the weight
    w = WingParams(CL0=0.0, CL_alpha=0.0, k=0.0, i_w=0.0, CD0=0.05)
    vp = VehicleParams(wing=w, rotor=LiftRotorParams(CH1=0.0))
    v = 20.0
    a = 0.5 * RHO * v * v * w.S * w.CD0 / vp.m
    p = solve_point(vp, RHO, v, a)
    assert p.theta == pytest.approx(0.0, abs=1e-9)
    assert p.T_r == pytest.approx(vp.m * G, rel=1e-9)


def test_a_wing_on_matches_eq_10_3_self_consistently():
    vp = replace(VP, rotor=LiftRotorParams(CH1=0.0))
    for v, a in ((8.0, 1.0), (10.0, 2.0), (12.0, 1.5)):
        p = solve_point(vp, RHO, v, a)
        assert p.ok
        w = vp.m * G
        assert math.tan(p.theta) == pytest.approx((vp.m * a - p.D) / (w - p.L), rel=1e-7)
        assert p.T_r == pytest.approx(math.hypot(vp.m * a - p.D, w - p.L), rel=1e-7)


def test_a_profiles_constant_and_jerk_limited():
    t, v, a = DecelProfile(2.0, v_end=1.0).sample(21.0, 50)
    assert v[0] == 21.0 and v[-1] == pytest.approx(1.0) and np.all(a == 2.0)
    t, v, a = DecelProfile(2.0, jerk=4.0, v_end=1.0).sample(21.0, 400)
    assert a[0] == 0.0 and a.max() == pytest.approx(2.0, abs=1e-12)
    assert v[-1] == pytest.approx(1.0, abs=1e-9)
    assert np.trapezoid(a, t) == pytest.approx(20.0, rel=1e-3)


def test_a_entry_step_is_flagged_but_pre_spun_schedule_can_be_feasible():
    r = analyse(VP, SC, 12.0, DecelProfile(1.5))
    assert r.feasible_pre_spun
    assert not r.feasible_whole
    assert r.first_binding_whole is not None and r.first_binding_whole[1] == 0.0
    assert r.record.label == "DRAFT" and len(r.record.input_hash) == 64


def test_a_max_feasible_decel_is_consistent_with_analyse():
    a_max = zero_loss.max_feasible_decel(VP, SC, 12.0)
    assert 0.2 < a_max < 8.0
    assert analyse(VP, SC, 12.0, DecelProfile(0.95 * a_max), n=60).feasible_pre_spun
    assert not analyse(VP, SC, 12.0, DecelProfile(1.3 * a_max), n=60).feasible_pre_spun


def test_a_feasibility_map_whole_implies_pre_spun():
    m = zero_loss.feasibility_map(VP, SC, [10.0, 14.0], [0.5, 1.5, 3.0], n=40)
    assert m["whole"].shape == (2, 3)
    assert np.all(~m["whole"] | m["pre_spun"])


def test_a_aero_limit_scales_with_equivalent_airspeed_but_throttle_does_not():
    hi = Scenario(5000.0)
    v_eas = 12.0
    v_hi = v_eas * math.sqrt(SC.rho / hi.rho)
    # lift/weight and drag/weight depend on V / V_s, so the aerodynamic limit is the same ...
    a_sl = zero_loss.max_feasible_decel(VP, SC, v_eas)
    a_hi = zero_loss.max_feasible_decel(VP, hi, v_hi)
    assert a_hi == pytest.approx(a_sl, rel=0.05)
    # ... but the rotors need more throttle in thin air for the same hover-like thrust
    r_sl = analyse(VP, SC, v_eas, DecelProfile(0.8))
    r_hi = analyse(VP, hi, v_hi, DecelProfile(0.8))
    assert r_hi.delta_cmd[-1] > r_sl.delta_cmd[-1]


# ---- analysis D ------------------------------------------------------------------------------
def test_d_planner_worked_example_of_section_10d():
    x, a_eq, _ = braking.ideal_stopping(25.0, math.radians(20.0), 0.0)
    assert x == pytest.approx(87.5, abs=0.1) and a_eq == pytest.approx(3.57, abs=0.01)
    x, a_eq, x_ramp = braking.ideal_stopping(25.0, math.radians(20.0), 3.0)
    assert x == pytest.approx(124.5, abs=0.2) and a_eq == pytest.approx(2.51, abs=0.01)
    assert x_ramp == pytest.approx(69.8, abs=0.2)


def test_d_simulated_limit_case_a_eq_equals_imposed_deceleration():
    # no aero, ideal altitude hold, pitch at the limit from the start, no ramp
    r = LiftRotorParams(CT1=0.0, CT2=0.0, k_mu=0.0, CH1=0.0)
    vp = VehicleParams(wing=WingParams(S=0.0), rotor=r)
    bp = BackTransitionParams(T_bt=0.0)
    theta = min(bp.theta_A, bp.theta_P)
    rho = SC.rho
    om = 2 * math.pi * math.sqrt(vp.m * G / math.cos(theta) / (r.n_r * r.CT0 * rho * r.D**4))
    veh = CasadiVehicle(vp, frozen_states=("h", "vh", "omega_r", "omega_p", "v1"))
    x0 = np.array([0.0, 0.0, 25.0, 0.0, theta, 0.0, om, 0.0, 0.0])
    res = braking.measure_a_eq(vp, SC, 25.0, bp, vehicle=veh, x0=x0)
    assert res.a_eq == pytest.approx(G * math.tan(theta), rel=5e-3)


def test_d_simulated_braking_vs_ideal_planner_and_signed_excursion():
    sc = Scenario(100.0)
    bp = BackTransitionParams.for_vehicle(VP, sc.rho, sc.soc, 100.0)
    res = braking.measure_a_eq(VP, sc, 25.0, bp)
    _, a_ideal, _ = braking.ideal_stopping(25.0, bp.theta_A, bp.T_bt, v_end=1.0)
    assert res.sim.terminated_by == "ground_speed_reached"
    assert res.a_eq == pytest.approx(a_ideal, rel=0.10)  # drag helps; lag and lift hurt
    m = res.sim.metrics
    assert m["altitude_excursion"] == max(m["altitude_loss"], m["altitude_gain"])
    assert m["altitude_gain"] > 1.0  # documented nose-up climb on back-transition


def test_d_altitude_hold_holds_in_hover():
    sc = Scenario(100.0)
    bp = BackTransitionParams.for_vehicle(VP, sc.rho, sc.soc, 100.0, x_tgt=0.0)
    om = trim.hover_omega(VP, sc.rho)
    x0 = np.array([0.0, 100.0, 0.0, 0.0, 0.0, 0.0, om, 0.0, 0.0])
    res = simulate(CasadiVehicle(VP), Position1Controller(bp), sc.environment(), x0, 10.0)
    assert res.metrics["altitude_excursion"] < 1.0


def test_d_recommendation_is_the_minimum_over_the_grid():
    bp = BackTransitionParams.for_vehicle(VP, SC.rho, SC.soc, 0.0)
    scens = [Scenario(100.0), Scenario(3000.0, isa_offset_k=10.0)]
    a_rec, rows = braking.recommend_decel(VP, scens, [18.0, 25.0], bp)
    assert len(rows) == 4
    assert a_rec == min(r[2] for r in rows)


def test_d_earlier_switch_reduces_overshoot_and_air_speed_test_is_earlier_in_headwind():
    sc = Scenario(100.0)
    bp = BackTransitionParams.for_vehicle(VP, sc.rho, sc.soc, 100.0)
    late = braking.landing_error(VP, sc, 25.0, 3.0, bp)
    early = braking.landing_error(VP, sc, 25.0, 1.5, bp)
    assert early.overshoot is not None and late.overshoot is not None
    assert early.overshoot < late.overshoot
    wind = Scenario(100.0, headwind_ms=5.0)
    g = braking.landing_error(VP, wind, 25.0, 2.5, bp, speed_test="ground")
    a = braking.landing_error(VP, wind, 25.0, 2.5, bp, speed_test="air")
    assert a.overshoot is not None and g.overshoot is not None
    assert a.overshoot < g.overshoot  # air-speed test switches earlier in a headwind
