from dataclasses import replace

import numpy as np
import pytest

from sagetrans.analysis import collocation as K  # noqa: N812
from sagetrans.analysis import freeform_opt
from sagetrans.analysis.common import Scenario
from sagetrans.physics.params import placeholder_vehicle

VP = placeholder_vehicle()
SC = Scenario(100.0)


@pytest.fixture(scope="module")
def prob() -> K.Problem:
    return K.make_problem(VP, SC, 22.0, K.CollocationOptions(n=40, max_iter=800))


@pytest.fixture(scope="module")
def sol10(prob: K.Problem) -> K.B1Solution:
    return K.solve_refined(prob, eps_h=10.0, eps_x=300.0, ns=(20,))


@pytest.fixture(scope="module")
def sol5(prob: K.Problem) -> K.B1Solution:
    return K.solve_refined(prob, eps_h=5.0, eps_x=300.0, ns=(20,))


# ---- mesh and controller plumbing ----------------------------------------------------------------
def test_graded_mesh_is_monotone_with_finer_ends():
    tau = K.mesh_fractions(40, 0.5)
    assert tau[0] == 0.0 and tau[-1] == pytest.approx(1.0)
    d = np.diff(tau)
    assert np.all(d > 0)
    assert d[0] == pytest.approx(0.5 / 40, rel=0.02)  # finer steps at the ends ...
    assert d[20] > 1.4 / 40  # ... coarser in the middle
    assert np.allclose(K.mesh_fractions(10, 0.0), np.linspace(0, 1, 11))


def test_open_loop_controller_maps_effective_throttle_to_the_esc_command():
    t = np.array([0.0, 1.0])
    u = np.array([[0.0, 0.1], [1.0, 0.3]])
    c = K.OpenLoopController(t, u, dead_zone=0.05)
    cmd0, _ = c.command(0.0, np.zeros(9), {}, None)
    cmd1, _ = c.command(1.0, np.zeros(9), {}, None)
    mid, _ = c.command(0.5, np.zeros(9), {}, None)
    assert cmd0[0] == pytest.approx(0.05) and cmd1[0] == pytest.approx(1.0)  # eff 0 -> dz
    assert mid[0] == pytest.approx(0.05 + 0.95 * 0.5) and mid[2] == pytest.approx(0.2)
    assert cmd0[1] == 0.0  # the pusher stays off


# ---- B1 solution, constraints and T-11 -----------------------------------------------------------
def test_b1_converges_and_satisfies_its_constraints(prob: K.Problem, sol10: K.B1Solution):
    s, o = sol10, prob.opt
    assert s.ok and s.status in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
    assert s.excursion <= 10.0 + 1e-3
    assert s.distance <= 300.0 + 1e-6
    assert s.x[-1, 2] == pytest.approx(o.v_f, abs=1e-4)  # ends at ground speed V_f
    assert abs(s.x[-1, 3]) <= o.vh_band + 1e-6
    assert np.all(s.u[:, 0] >= -1e-8) and np.all(s.u[:, 0] <= o.delta_max + 1e-8)
    assert np.all(np.abs(s.u[:, 1]) <= o.theta_cmd_max + 1e-8)
    assert np.all(s.x[:, 6] >= -1e-6)  # rotor speed non-negative
    assert np.all(np.abs(s.x[:, 3]) <= o.vh_max + 1e-6)
    assert np.all(np.diff(s.t) > 0) and s.t[0] == 0.0
    du = np.abs(np.diff(s.u, axis=0))
    dt = np.diff(s.t)
    assert np.all(du[:, 0] <= o.delta_rate * dt + 1e-6)
    assert np.all(du[:, 1] <= o.theta_rate * dt + 1e-6)
    assert s.record is not None and s.record.label == "DRAFT"


def test_t11_collocation_replayed_in_the_simulator_reproduces_the_metrics(
    prob: K.Problem, sol10: K.B1Solution
):
    chk = K.replay(prob, sol10)
    for k in ("energy", "distance", "duration"):
        assert chk.rel_error[k] <= 0.02, (k, chk.rel_error)
    assert abs(chk.metrics["excursion"] - chk.collocated["excursion"]) <= 0.3
    assert chk.passes()
    # the replay ends on the same event as the collocation: ground speed reaches V_f
    assert chk.sim.terminated_by == "ground_speed_reached"


def test_pareto_energy_rises_as_the_altitude_band_tightens(
    sol10: K.B1Solution, sol5: K.B1Solution
):
    assert sol5.ok
    assert sol5.excursion <= 5.0 + 1e-3
    assert sol5.energy > sol10.energy  # tighter band costs energy (epsilon-constraint front)
    assert sol5.t_f > sol10.t_f  # and takes longer


def test_non_convergence_is_recorded_not_raised(prob: K.Problem):
    hard = K.with_options(prob, max_iter=15)
    sol = K.solve(hard, eps_h=3.0, eps_x=40.0)  # 40 m from 22 m/s is not reachable
    assert not sol.ok
    assert sol.status and sol.status != "Solve_Succeeded"


# ---- bound, comparison, verification -------------------------------------------------------------
def test_b1_beats_the_baseline_at_the_baselines_own_excursion_and_distance(prob: K.Problem):
    bc = K.bound_check(prob)
    assert bc.b1.ok
    assert bc.b1.excursion <= bc.baseline["excursion"] * 1.02 + 1e-3
    assert bc.b1.distance <= bc.baseline["distance"] * 1.02 + 1e-3
    assert bc.energy_margin > 0.05  # under limits the baseline does not respect
    chk = K.replay(prob, bc.b1)
    assert chk.passes()
    # same scenario, same cost definitions: the comparison table
    b2 = freeform_opt.decode(freeform_opt.init_from_zero_loss(VP, SC, 22.0))
    table = K.compare(prob, bc.b1, None, b2)
    assert set(table) == {"B1", "C", "B2"}
    assert table["B1"]["energy"] < table["C"]["energy"]
    assert all(set(v) == {"excursion", "energy", "distance"} for v in table.values())


def test_replay_check_rejects_a_solution_that_cheats_between_nodes(
    prob: K.Problem, sol10: K.B1Solution
):
    chk = K.replay(prob, sol10)
    doctored = replace(chk, collocated={**chk.collocated, "excursion": 0.0})
    assert chk.passes() and not doctored.passes()  # a 0 m claim against a ~10 m replay


def test_b2_can_start_from_the_b1_solution(prob: K.Problem, sol10: K.B1Solution):
    prm = freeform_opt.init_from_collocation(prob, sol10)
    assert set(prm) == set(freeform_opt.BOUNDS.names)
    u = freeform_opt.BOUNDS.to_unit(prm)
    assert np.all((0.0 <= u) & (u <= 1.0))
    fp = freeform_opt.decode(prm)
    # the free-form law runs from this start; it is a start, not a claim that it is good
    m = freeform_opt.evaluate(VP, SC, fp, 22.0)
    assert np.isfinite([m.excursion, m.energy, m.distance]).all()
    # the rotors are off above the speed where B1 first switches them on, and the rotor knots at
    # high V/V_s carry B1's near-idle command (the ESC dead zone), not an invented value
    assert fp.s_on <= freeform_opt.S_KNOTS[-1]
    assert fp.delta_r[-1] >= prob.dead_zone - 1e-6


def test_solve_verified_and_sweep_report_all_attempts():
    small = K.make_problem(VP, SC, 22.0, K.CollocationOptions(n=20, max_iter=300))
    pts = K.pareto_sweep(small, [12.0], [300.0], guesses=[K.baseline_guess(small)])
    eh, ex, v = pts[0]
    assert (eh, ex) == (12.0, 300.0) and len(v.attempts) == 1
    assert isinstance(v.verified, bool)
    assert v.sol is v.attempts[0]
