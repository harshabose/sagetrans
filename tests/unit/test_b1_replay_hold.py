"""B1 replay: the schedule is read at the middle of each 20 ms step, as the NLP means it."""

import math
from pathlib import Path

import numpy as np
import pytest

from sagetrans.analysis import braking
from sagetrans.analysis import collocation as coll
from sagetrans.dynamics.simulate import DT_DEFAULT
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs

ROOT = Path(__file__).resolve().parents[2]


def test_the_open_loop_controller_reads_the_history_a_lead_after_the_step_start():
    t = np.array([0.0, 1.0])
    u = np.array([[0.0, 0.0], [1.0, 0.5]])  # throttle and pitch command, both linear in time
    start = coll.OpenLoopController(t, u).command(0.2, np.zeros(9), {}, None)[0]
    half = 0.5 * DT_DEFAULT
    mid = coll.OpenLoopController(t, u, lead=half).command(0.2, np.zeros(9), {}, None)[0]
    assert start[0] == pytest.approx(0.2) and mid[0] == pytest.approx(0.21)
    assert start[2] == pytest.approx(0.1) and mid[2] == pytest.approx(0.105)
    assert start[1] == 0.0 and mid[1] == 0.0  # the pusher stays off


def test_replay_rejects_an_unknown_hold():
    with pytest.raises(ValueError, match="hold"):
        coll.replay(None, None, "end")  # type: ignore[arg-type]


def test_the_c_envelope_optimum_at_a_20_deg_ceiling_replays_and_is_not_robust_to_a_late_command():
    """S-01, 60 m cap, the case that did not replay: 10.47 kJ collocated against 6.16 kJ replayed.

    The replay read the schedule at the start of each 20 ms step (a 10 ms mean delay). The optimum
    rides the lift peak 0.3 deg below the stall cliff, so that delay sank it 29 m. Read at the
    middle of the step the replay agrees; read at the start it still does not.
    """
    inp = load_inputs(ROOT / "configs", only={"B1"})
    b, c, d, vp = inp.study.B1, inp.study.C, inp.study.D, inp.vehicle
    scen = inp.scenarios["S-01"]
    base = stages.exit_params(inp, rotor_lag_s=c.rotor_lag_s)
    v_air = braking.entry_airspeed(inp.airspeed_min, scen.rho, d.entry_convention)
    opt = coll.CollocationOptions(
        n=b.intervals, max_iter=b.max_iter, vh_band=b.end_vh_band_ms or c.sink_tol_ms,
        theta_cmd_max=math.radians(20.0), alpha_max=math.radians(30.0),
    )
    prob = coll.make_problem_exit(vp, scen, base, v_air, opt, inp.controller_overrides,
                                  inp.theta_p, c.rotor_start)
    guess = coll.baseline_guess(prob, math.radians(20.0), 3.0)
    ver = coll.solve_verified(prob, 1.0, 60.0, "energy", guesses=[guess], ns=tuple(b.refine_from))
    assert ver.sol.ok and ver.verified
    ck = ver.check
    assert ck is not None and ck.rel_error["energy"] < 0.02 and ck.rel_error["distance"] < 0.02
    assert ck.metrics["excursion"] < 1.5  # claimed 1.0 m, not 29 m
    assert ver.delayed is not None and ver.delay_robust is False
    assert ver.delayed.metrics["excursion"] > 5.0  # a command 10 ms late falls over the cliff
