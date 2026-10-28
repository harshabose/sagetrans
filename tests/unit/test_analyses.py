import math
from dataclasses import replace

import numpy as np
import pytest

from sagetrans import atmosphere
from sagetrans.analysis import braking, zero_loss
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.zero_loss import DEFAULT_LIMITS, DecelProfile, analyse, solve_point
from sagetrans.control.ardupilot_like import BackTransitionParams, Position1Controller
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import simulate
from sagetrans.physics import aero, rotor
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


def test_a_profile_is_a_constant_deceleration_to_the_end_speed():
    t, v, a = DecelProfile(2.0, v_end=1.0).sample(21.0, 50)
    assert v[0] == 21.0 and v[-1] == pytest.approx(1.0) and np.all(a == 2.0)
    assert t[-1] == pytest.approx(10.0)


def test_a_violations_name_every_failing_margin_with_its_first_time():
    ok = analyse(VP, SC, 12.0, DecelProfile(0.8))
    assert ok.feasible and ok.violations == {} and zero_loss.first_binding(ok) is None
    bad = analyse(VP, SC, 12.0, DecelProfile(1.5))  # past the lift limit from the first sample
    assert not bad.feasible and bad.violations == {"alpha_high": 0.0}
    assert zero_loss.first_binding(bad) == ("alpha_high", 0.0)
    assert bad.record.label == "DRAFT" and len(bad.record.input_hash) == 64


def test_a_alpha_band_is_the_lift_fit_floor_and_a_fraction_of_cl_max():
    lo, hi = zero_loss._alpha_band(VP, zero_loss.DEFAULT_LIMITS)
    i_w = VP.wing.i_w
    assert lo == pytest.approx(-i_w)  # alpha_min = 0
    cl = float(aero.coefficients(hi + i_w, VP.wing)[0])
    assert cl == pytest.approx(0.8 * aero.cl_max(VP.wing), rel=1e-6)
    _, hi1 = zero_loss._alpha_band(VP, zero_loss.ActuatorLimits(cl_frac=1.0))
    assert hi1 > hi  # the lift peak is further out


def test_a_feasible_results_stay_inside_the_angle_band():
    lo, hi = zero_loss._alpha_band(VP, zero_loss.DEFAULT_LIMITS)
    for v0, a in ((8.0, 0.4), (10.0, 1.0), (12.0, 0.8), (14.0, 0.5)):
        r = analyse(VP, SC, v0, DecelProfile(a), n=40)
        assert r.feasible, (v0, a, r.violations)
        assert r.theta.min() >= lo - 1e-9 and r.theta.max() <= hi + 1e-9


def test_a_band_free_path_reproduces_the_unbanded_root_choice():
    # with no band, or the full scan range, solve_point is the pre-band selection exactly
    bal = zero_loss._Balance(VP)
    full = (zero_loss.THETA_LO, zero_loss.THETA_HI)
    for v, a in ((6.0, 0.5), (12.0, 1.0), (12.0, 1.5), (16.0, 1.2)):
        p0 = solve_point(VP, RHO, v, a, 0.0, None, bal)
        p1 = solve_point(VP, RHO, v, a, 0.0, None, bal, band=full)
        assert (p0.theta, p0.T_r, p0.omega) == (p1.theta, p1.T_r, p1.omega)


def test_a_banded_solver_prefers_in_band_roots():
    # at 12 m/s, a = 1.5 the only roots are outside the narrow default band: it is reported
    # (nan would hide it) but fails an angle margin; a lower deceleration has an in-band root
    lo, hi = zero_loss._alpha_band(VP, zero_loss.DEFAULT_LIMITS)
    p = solve_point(VP, RHO, 12.0, 0.8, 0.0, None, band=(lo, hi))
    assert lo <= p.theta <= hi and p.T_r >= 0.0


def test_a_feasible_decel_range_is_consistent_with_analyse():
    a_min, a_max, islands = zero_loss.feasible_decel_range(VP, SC, 12.0)
    assert 0.0 < a_min < a_max < 4.0 and not islands
    assert zero_loss.max_feasible_decel(VP, SC, 12.0) == a_max
    mid = 0.5 * (a_min + a_max)
    for a in (a_min * 1.05, mid, a_max * 0.97):
        assert analyse(VP, SC, 12.0, DecelProfile(a), n=60).feasible
    for a in (a_min * 0.8, a_max * 1.1):
        assert not analyse(VP, SC, 12.0, DecelProfile(a), n=60).feasible


def test_a_feasible_window_agrees_with_the_path_check_and_names_its_edges():
    w = zero_loss.feasible_window(VP, SC, 12.0)
    assert len(w) == 1
    a_min, a_max, _ = zero_loss.feasible_decel_range(VP, SC, 12.0)
    # the path check also needs every slower speed, so its range lies inside the window at 12 m/s
    # (to the resolution of the two bisections)
    assert w[0].a_lo <= a_min + 1e-3 and w[0].a_hi >= a_max - 1e-3
    assert w[0].bind_hi and w[0].T_r_lo > 0.0 and w[0].T_r_hi >= 0.0
    # no feasible deceleration where the wing carries more than the weight: the rotors would
    # have to push down, and the report then says which limit fails closest to feasible
    assert zero_loss.feasible_window(VP, SC, 30.0) == []
    assert zero_loss.closest_to_feasible(VP, SC, 30.0)[1] == ["thrust_sign"]


def test_a_legacy_limits_keep_the_old_non_monotone_verdict_that_b2_relies_on():
    # regression: with no band (the legacy path) the verdict at 12 m/s is feasible up to about
    # 1 m/s^2, infeasible 1.0 to 1.4, feasible again 1.6 to 2.0, infeasible above
    leg = zero_loss.LEGACY_LIMITS
    verdict = {
        a: analyse(VP, SC, 12.0, DecelProfile(a), leg, n=60).feasible for a in (0.8, 1.2, 1.8, 2.6)
    }
    assert verdict == {0.8: True, 1.2: False, 1.8: True, 2.6: False}
    assert zero_loss.max_feasible_decel(VP, SC, 12.0, leg) >= 1.8


def test_a_feasibility_map_shape_and_violations_are_consistent():
    m = zero_loss.feasibility_map(VP, SC, [10.0, 14.0], [0.5, 1.5, 3.0], n=40)
    assert m["feasible"].shape == (2, 3)
    for i in range(2):
        for j in range(3):
            assert m["feasible"][i, j] == (m["violations"][i][j] == {})
    assert m["feasible"][0, 0] and not m["feasible"][0, 2]


def test_a_limit_is_the_same_at_altitude_in_equivalent_airspeed_but_throttle_is_not():
    hi = Scenario(5000.0)
    v_eas = 12.0
    v_hi = v_eas * math.sqrt(SC.rho / hi.rho)
    # lift and drag depend on dynamic pressure, so the aerodynamic limits are the same ...
    r_sl = zero_loss.feasible_decel_range(VP, SC, v_eas)
    r_hi = zero_loss.feasible_decel_range(VP, hi, v_hi)
    assert r_sl is not None and r_hi is not None
    assert r_hi[1] == pytest.approx(r_sl[1], rel=0.01)
    # ... but the rotors need more throttle in thin air for the same hover-like thrust
    d_sl = analyse(VP, SC, v_eas, DecelProfile(0.8))
    d_hi = analyse(VP, hi, v_hi, DecelProfile(0.8))
    assert d_hi.delta_cmd[-1] > d_sl.delta_cmd[-1]


def test_a_vrs_ratio_is_always_reported_and_the_default_is_report_only():
    assert zero_loss.DEFAULT_LIMITS.vrs_limit is None and zero_loss.LEGACY_LIMITS.vrs_limit is None
    r = analyse(VP, SC, 12.0, DecelProfile(0.8), n=60)
    assert math.isfinite(r.vrs_min) and r.vrs_min < 0.0  # nose-up: flow comes up through the discs
    assert 0.0 <= r.vrs_min_t <= r.t[-1]
    assert "vrs" not in r.margins and r.feasible
    # nose-up pitch gives a negative ratio, level flight zero, and the helper is rotor.vrs_ratio
    rho = SC.rho
    assert zero_loss._vrs_ratio(VP, rho, 12.0, 0.0, 100.0) == 0.0
    assert zero_loss._vrs_ratio(VP, rho, 12.0, 0.1, 100.0) == pytest.approx(
        float(rotor.vrs_ratio(-12.0 * math.sin(0.1), 100.0, rho, VP.rotor))
    )
    assert zero_loss._vrs_ratio(VP, rho, 12.0, 0.1, 100.0) < 0.0
    assert math.isfinite(zero_loss._vrs_ratio(VP, rho, 12.0, 0.1, 0.0))  # T_r = 0: floored


def test_a_vrs_margin_has_sign_and_scale_and_enforces_only_when_a_limit_is_set():
    r0 = analyse(VP, SC, 12.0, DecelProfile(0.8), n=60)
    prof = DecelProfile(0.8)
    far = analyse(VP, SC, 12.0, prof, replace(DEFAULT_LIMITS, vrs_limit=2 * r0.vrs_min), n=60)
    near = analyse(VP, SC, 12.0, prof, replace(DEFAULT_LIMITS, vrs_limit=0.5 * r0.vrs_min), n=60)
    # margin = (ratio - limit) / |limit|: +0.5 where the ratio is half the limit
    assert far.feasible and far.margins["vrs"].min() == pytest.approx(0.5)
    assert not near.feasible and "vrs" in near.violations
    # the margin does not change any other verdict or number
    assert far.violations == r0.violations and np.array_equal(far.theta, r0.theta)
    # a ratio of exactly the limit gives 0; a ratio of 0 gives +1; an unsolved sample gives -1
    lim = replace(DEFAULT_LIMITS, vrs_limit=-0.23)
    p = solve_point(VP, RHO, 12.0, 0.5)
    one = np.array([True, False])
    m, (_, _, _, ratio) = zero_loss._margins(
        VP, SC, lim, (0.0, 0.2), np.array([0.0, 1.0]), np.full(2, 12.0), np.array([0.0, math.nan]),
        np.array([p.T_r, math.nan]), np.full(2, p.omega), one, np.zeros(2), np.zeros(2), 0.0,
    )  # fmt: skip
    assert ratio[0] == 0.0 and m["vrs"][0] == pytest.approx(1.0) and m["vrs"][1] == -1.0
    m2, _ = zero_loss._margins(
        VP, SC, replace(lim, vrs_limit=float(ratio[0]) - 1.0), (0.0, 0.2), np.array([0.0]),
        np.array([12.0]), np.array([0.0]), np.array([p.T_r]), np.array([p.omega]),
        np.array([True]), np.zeros(1), np.zeros(1), 0.0,
    )  # fmt: skip
    assert m2["vrs"][0] == pytest.approx(1.0)


def test_a_window_reports_vrs_at_its_edges_and_enforcing_trims_only_the_top():
    w = zero_loss.feasible_window(VP, SC, 12.0)[0]
    assert w.vrs_lo > w.vrs_hi and w.vrs_hi < 0.0 and math.isfinite(w.vrs_lo)
    enforced = zero_loss.feasible_window(VP, SC, 12.0, replace(DEFAULT_LIMITS, vrs_limit=-0.23))[0]
    assert enforced.a_lo == w.a_lo  # the lower edge is the drag floor, untouched
    assert enforced.a_hi < w.a_hi and enforced.bind_hi == ["vrs"]
    assert enforced.vrs_hi == pytest.approx(-0.23, abs=0.01)


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


def test_d_scenario_mass_reaches_the_model_the_hover_start_and_the_feed_forward():
    # regression: run_braking once ignored Scenario.mass_kg, so the 0.85-1.15 mass axis of the
    # ensemble was inert in the Q_TRANS_DECEL recommendation
    light, heavy = Scenario(100.0), Scenario(100.0, mass_kg=1.3 * VP.m)
    bp = BackTransitionParams.for_vehicle(VP, light.rho, light.soc, 100.0)
    r_l = braking.measure_a_eq(VP, light, 25.0, bp)
    r_h = braking.measure_a_eq(VP, heavy, 25.0, bp)
    assert abs(r_h.a_eq - r_l.a_eq) > 1e-6 and abs(r_h.x_actual - r_l.x_actual) > 1e-6
    # hover rotor speed scales with sqrt(thrust) = sqrt(mass), applied to the initial state
    ratio = float(r_h.sim["omega_r"][0] / r_l.sim["omega_r"][0])
    assert ratio == pytest.approx(1.3**0.5, rel=1e-3)
    # the recommendation and the landing error see it too
    a_rec_l, _ = braking.recommend_decel(VP, [light], [25.0], bp)
    a_rec_h, _ = braking.recommend_decel(VP, [heavy], [25.0], bp)
    assert abs(a_rec_h - a_rec_l) > 1e-6
    e_l = braking.landing_error(VP, light, 25.0, 2.5, bp)
    e_h = braking.landing_error(VP, heavy, 25.0, 2.5, bp)
    assert e_l.overshoot != e_h.overshoot
    # the hover feed-forward follows the scenario mass unless refresh is switched off
    off = braking.measure_a_eq(VP, heavy, 25.0, bp, refresh_hover=False)
    assert abs(off.a_eq - r_h.a_eq) > 1e-9
