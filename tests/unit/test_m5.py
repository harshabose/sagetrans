import math

import numpy as np
import pytest

from sagetrans.analysis import braking, freeform_opt, restricted, runs
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.ensemble import (
    CostWeights,
    RunMetrics,
    ensemble_cost,
    latin_hypercube,
)
from sagetrans.analysis.search import Bounds, cma_minimise
from sagetrans.control.ardupilot_like import (
    BackTransitionParams,
    BaselineController,
    BaselineParams,
)
from sagetrans.control.freeform import FreeFormController, FreeFormParams
from sagetrans.dynamics import events as ev
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import simulate
from sagetrans.physics.params import placeholder_vehicle

VP = placeholder_vehicle()


# ---- search machinery (T-12) ---------------------------------------------------------------------
def _penalised(u: np.ndarray) -> float:
    """Smooth bowl plus a failure-style jump, minimum 1.0 at u = 0.3."""
    return float(1.0 + np.sum((u - 0.3) ** 2) + 5.0 * (u[0] > 0.9) + 2.0 * (u[1] < 0.05))


def test_t12_three_seeds_reach_the_same_optimum_and_are_deterministic():
    u0 = np.full(4, 0.7)
    a = cma_minimise(_penalised, u0, popsize=8, maxiter=60)
    assert a.spread < 0.01
    assert a.best_f == pytest.approx(1.0, abs=0.01)
    assert np.allclose(a.best_x, 0.3, atol=0.1)
    b = cma_minimise(_penalised, u0, popsize=8, maxiter=60)  # NFR-01
    assert [s.f for s in a.seeds] == [s.f for s in b.seeds]


def test_search_caches_repeated_points_and_never_returns_worse_than_start():
    calls = []

    def f(u: np.ndarray) -> float:
        calls.append(tuple(u))
        return float(1.0 + np.sum(u**2))

    r = cma_minimise(f, np.zeros(2), seeds=(1,), popsize=4, maxiter=3)
    assert len(calls) == len(set(calls))
    assert r.best_f <= r.f_start


def test_bounds_round_trip():
    b = Bounds(("a", "b"), (1.0, -2.0), (3.0, 2.0))
    u = b.to_unit({"a": 2.0, "b": 0.0})
    assert np.allclose(u, [0.5, 0.5])
    assert b.to_physical(u) == {"a": 2.0, "b": 0.0}


# ---- ensemble and cost ---------------------------------------------------------------------------
def test_latin_hypercube_has_one_sample_per_stratum_and_is_deterministic():
    n = 10
    sc = latin_hypercube(n, VP, seed=4)
    assert sc == latin_hypercube(n, VP, seed=4)
    alt = np.array([s.altitude_amsl_m for s in sc])
    strata = np.floor(alt / 5000.0 * n).astype(int)
    assert sorted(strata.tolist()) == list(range(n))
    assert all(0.2 <= s.soc <= 1.0 and -5.0 <= s.headwind_ms <= 10.0 for s in sc)
    assert all(0.85 * VP.m <= (s.mass_kg or 0.0) <= 1.15 * VP.m for s in sc)


def test_ensemble_cost_hand_calculation():
    runs_ = [
        RunMetrics(2.0, 1000.0, 50.0),
        RunMetrics(4.0, 2000.0, 100.0),
        RunMetrics(6.0, 3000.0, 150.0, ("stall",)),
    ]
    w = CostWeights()
    # p95 of [2,4,6] = 5.8; mean E = 2000; p95 of [50,100,150] = 145; one failure
    expected = 1.0 * 5.8 / 10.0 + 0.3 * 2000.0 / 5000.0 + 0.5 * 145.0 / 150.0 + 10.0
    assert ensemble_cost(runs_, w) == pytest.approx(expected)


# ---- free-form controller ------------------------------------------------------------------------
def test_freeform_law_interpolates_in_speed_and_has_no_altitude_feedback():
    p = FreeFormParams(
        delta_r=(0.6, 0.5, 0.4, 0.3, 0.2, 0.1), theta=(0.3, 0.3, 0.2, 0.1, 0.0, 0.0), s_on=1.0
    )
    c = FreeFormController(p, v_stall=10.0)
    x_lo = np.array([0.0, 100.0, 5, 0, 0, 0, 0, 0, 0])
    u, _ = c.command(0.0, x_lo, {"V": 6.0}, None)  # s = 0.6, between knots 0.4 and 0.8
    assert u[0] == pytest.approx(0.45) and u[1] == 0.0 and u[2] == pytest.approx(0.3 - 0.05)
    u_hi, _ = c.command(0.0, x_lo, {"V": 15.0}, None)  # above s_on: rotors off
    assert u_hi[0] == 0.0
    x_other = x_lo.copy()
    x_other[1], x_other[3] = 500.0, -3.0  # altitude and climb rate change nothing
    assert np.array_equal(c.command(0.0, x_other, {"V": 6.0}, None)[0], u)


# ---- baseline controller -------------------------------------------------------------------------
def _baseline(scen: Scenario, **kw: float) -> tuple[BaselineParams, np.ndarray]:
    tr = trim.steady_trim(VP, scen.rho, 22.0, 0.0, scen.soc)
    back = BackTransitionParams.for_vehicle(
        VP, scen.rho, scen.soc, scen.altitude_amsl_m, x_tgt=800.0
    )
    bp = BaselineParams(
        back=back, delta_p_trim=tr.elec.delta_p_cmd, theta_trim=tr.theta, **kw  # type: ignore[arg-type]
    )
    x0 = np.array([0, scen.altitude_amsl_m, 0, 0, 0, 0, trim.hover_omega(VP, scen.rho), 0, 0.0])
    return bp, x0


def test_baseline_runs_the_documented_mode_sequence():
    sc = Scenario(100.0)
    bp, x0 = _baseline(sc)
    sim = simulate(
        runs.vehicle_for(VP), BaselineController(bp), sc.environment(), x0, 90.0,
        [ev.ground_speed_below(1.0)],
    )  # fmt: skip
    order = [m for i, m in enumerate(sim.modes) if i == 0 or m != sim.modes[i - 1]]
    assert order == ["fwd_wait", "fwd_fade", "cruise", "position1"]
    assert sim.terminated_by == "ground_speed_reached"
    modes = np.array(sim.modes)
    i_fade = int(np.argmax(modes == "fwd_fade"))
    # the controller sees the previous step's airspeed: it waits for AIRSPEED_MIN
    assert sim["V"][i_fade - 1] >= bp.airspeed_min > sim["V"][i_fade - 2]
    assert np.all(sim["delta_p"][modes == "fwd_wait"] == bp.tkoff_thr_max)  # TKOFF_THR_MAX in AUTO
    # (8.7): rotor throttle falls to zero over Q_TRANSITION_MS
    i_cruise = int(np.argmax(modes == "cruise"))
    assert sim.t[i_cruise] - sim.t[i_fade] == pytest.approx(bp.q_transition_s, abs=0.05)
    assert sim["delta_r"][i_cruise] == 0.0
    # the cruise stub settles near the target speed and altitude
    mid = (modes == "cruise") & (sim.t > sim.t[i_cruise] + 15.0)
    assert np.all(np.abs(sim["V"][mid] - bp.v_cruise) < 1.0)
    assert np.all(np.abs(sim["h"][mid] - 100.0) < 1.5)


def test_switch_test_fires_at_planned_distance_and_air_speed_test_switches_earlier_in_headwind():
    switch_x = {}
    for test in ("ground", "air"):
        sc = Scenario(100.0, headwind_ms=6.0)
        bp, x0 = _baseline(sc, speed_test=test)
        sim = simulate(
            runs.vehicle_for(VP), BaselineController(bp), sc.environment(), x0, 90.0,
            [ev.ground_speed_below(1.0)],
        )  # fmt: skip
        i = int(np.argmax(np.array(sim.modes) == "position1"))
        v_ref = sim["vx"][i] if test == "ground" else sim["V"][i]
        d = bp.back.x_tgt - sim["x"][i]
        assert d == pytest.approx(v_ref**2 / (2 * bp.a_plan), abs=1.5 * sim["V"][i] * 0.02 + 0.5)
        switch_x[test] = sim["x"][i]
    assert switch_x["air"] < switch_x["ground"]  # headwind: air speed > ground speed


def test_q_trans_fail_aborts_a_stuck_transition_and_keeps_the_rotors_holding():
    sc = Scenario(100.0)
    bp, x0 = _baseline(sc, tkoff_thr_max=0.1, q_trans_fail_s=4.0)  # pusher too weak to arrive
    sim = simulate(runs.vehicle_for(VP), BaselineController(bp), sc.environment(), x0, 12.0)
    assert sim.final_mode.name == "aborted"
    assert float(sim["V"].max()) < bp.airspeed_min
    assert abs(float(sim["h"][-1]) - 100.0) < 2.0  # rotors still hold altitude


# ---- run metrics ---------------------------------------------------------------------------------
def test_back_transition_defaults_complete_without_failure_and_gain_altitude():
    sc = Scenario(100.0)
    m = restricted.evaluate_back(VP, sc, restricted.DEFAULT_PARAMS)
    assert m.failures == () and m.excursion > 1.0 and m.energy > 0 and m.distance > 0


def test_c_sets_q_trans_decel_from_analysis_d_not_from_the_ideal_planner():
    sc = Scenario(100.0, mass_kg=1.1 * VP.m, headwind_ms=2.0)
    vps = sc.vehicle_for(VP)
    prm = dict(restricted.DEFAULT_PARAMS)
    x0 = runs.cruise_entry_state(vps, sc, 22.0)
    sim_a = restricted.plan_decel(vps, sc, prm, x0, 1.0, "simulated")
    ideal = restricted.plan_decel(vps, sc, prm, x0, 1.0, "ideal")
    # the simulated source IS analysis D's worst-case a_eq from the same entry state
    bp = BackTransitionParams.for_vehicle(
        vps, sc.rho, sc.soc, sc.altitude_amsl_m, T_bt=prm["T_bt"], theta_A=prm["theta_A"],
        theta_P=restricted.THETA_P, delta_min=prm["spin_min"],
    )  # fmt: skip
    d = braking.measure_a_eq(vps, sc, float(x0[2]), bp, x0=x0)
    assert sim_a == pytest.approx(d.a_eq, rel=1e-12)
    assert ideal == pytest.approx(
        braking.ideal_stopping(float(x0[2]), prm["theta_A"], prm["T_bt"], 1.0, dt=2e-3)[1]
    )
    assert abs(sim_a - ideal) > 1e-3  # the two planners are genuinely different numbers
    with pytest.raises(ValueError, match="a_plan_source"):
        restricted.plan_decel(vps, sc, prm, x0, 1.0, "nonsense")
    # both sources give a complete evaluation (the run stops)
    for src in ("simulated", "ideal"):
        m = restricted.evaluate_back(VP, sc, prm, a_plan_source=src)
        assert "not_stopped" not in m.failures and m.distance > 0


def test_failure_flags_report_a_run_that_does_not_stop():
    sc = Scenario(100.0)
    x0 = runs.cruise_entry_state(VP, sc, 22.0)
    sim = simulate(
        runs.vehicle_for(VP), BaselineController(_baseline(sc)[0]), sc.environment(), x0, 1.0
    )
    assert "not_stopped" in runs.failure_flags(sim, VP)


# ---- analysis C ----------------------------------------------------------------------------------
def test_c_full_scope_evaluates_both_transitions():
    m = restricted.evaluate_full(VP, Scenario(100.0), restricted.DEFAULT_PARAMS)
    assert m.failures == () and m.energy > 5e3 and m.distance > 0


def test_c_optimise_back_scope_improves_on_the_default_within_bounds():
    scens = latin_hypercube(3, VP, seed=1)
    r = restricted.optimise(VP, scens, "back", seeds=(1,), popsize=6, maxiter=4)
    assert r.cost <= r.cost_default
    for name, lo, hi in zip(
        restricted.BACK_BOUNDS.names, restricted.BACK_BOUNDS.lo, restricted.BACK_BOUNDS.hi,
        strict=True,
    ):  # fmt: skip
        assert lo <= r.params[name] <= hi
    assert r.record.label == "DRAFT" and len(r.runs) == 3
    assert r.cost == pytest.approx(ensemble_cost(r.runs, CostWeights()))


# ---- analysis B2 ---------------------------------------------------------------------------------
def test_b2_initial_knots_come_from_analysis_a_and_stay_in_bounds():
    prm = freeform_opt.init_from_zero_loss(VP, Scenario(100.0), 22.0)
    u = freeform_opt.BOUNDS.to_unit(prm)
    assert u.shape == (13,) and np.all((0.0 <= u) & (u <= 1.0))
    fp = freeform_opt.decode(prm)
    assert len(fp.delta_r) == len(fp.theta) == 6


def test_b2_optimise_never_returns_worse_than_its_start():
    scens = latin_hypercube(3, VP, seed=1)
    r = freeform_opt.optimise(VP, scens, seeds=(1,), popsize=8, maxiter=3)
    assert r.cost <= r.cost_initial
    assert r.cost == pytest.approx(ensemble_cost(r.runs, CostWeights()))
    # same cost definition as analysis C, so the two results are comparable
    c = restricted.optimise(VP, scens, "back", seeds=(1,), popsize=4, maxiter=1)
    assert math.isfinite(c.cost)
