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


# ---- the mid-step reading belongs to the B1 replay and to nothing else ---------------------------
SRC = ROOT / "src" / "sagetrans"


def _lines_with(needle: str) -> list[str]:
    hits = []
    for p in sorted(SRC.rglob("*.py")):
        for n, line in enumerate(p.read_text(encoding="utf-8").splitlines(), start=1):
            if needle in line and not line.lstrip().startswith(("#", '"', "`")):
                hits.append(f"{p.relative_to(SRC).as_posix()}:{n}:{line.strip()}")
    return hits


def test_the_callers_of_the_mid_step_reading_are_the_b1_replay_only():
    """The path: `solve_verified` -> `replay(hold="mid")` -> `OpenLoopController(lead=dt/2)`.

    Callers (src): `OpenLoopController` is built once, in `replay`; `replay` is called twice, both
    in `solve_verified` (the mid-step read, and the start read for `Verified.delayed`);
    `solve_verified` is called by `stages.task_b1` (B1 solves) and `bound_check_c` (the B1-C gap).
    `simulate()` and its step `DT_DEFAULT` are unchanged, so D, C, E and FT never see a lead.
    """
    built = [h for h in _lines_with("OpenLoopController(") if "class " not in h]
    assert [h.split(":")[0] for h in built] == ["analysis/collocation.py"]
    assert len(built) == 1
    replays = [h for h in _lines_with("replay(prob") if "def " not in h]
    assert {h.split(":")[0] for h in replays} == {"analysis/collocation.py"} and len(replays) == 2
    assert {h.split(":")[0] for h in _lines_with("solve_verified(")
            if "def " not in h} <= {"analysis/collocation.py", "io/stages.py"}
    users = {h.split(":")[0] for h in _lines_with("DT_DEFAULT")}
    assert users == {"analysis/collocation.py", "dynamics/simulate.py"}


def test_d_c_and_ft_runs_never_build_the_open_loop_controller(monkeypatch):
    built: list[object] = []
    real = coll.OpenLoopController.__init__

    def spy(self, *a, **kw):
        built.append(1)
        real(self, *a, **kw)

    monkeypatch.setattr(coll.OpenLoopController, "__init__", spy)
    from sagetrans.analysis import restricted
    from sagetrans.analysis.common import Scenario
    from sagetrans.physics.params import placeholder_vehicle

    vp, scen = placeholder_vehicle(), Scenario(0.0)
    inp = load_inputs(ROOT / "tests" / "data" / "placeholder_configs")
    cev, base, _ = stages.c_settings(inp)
    bp = stages.d_exit_params(inp)
    x0, v_ground = braking.airbrake_entry_state(vp, scen, bp, 15.0)
    braking.measure_a_eq(vp, scen, v_ground, bp, x0=x0)  # a D run
    prm = {"T_bt": 3.0, "theta_A": 0.35, "spin_min": 0.1, "q": 2.0}
    restricted.run_c(vp, scen, prm, cev, base)
    assert built == []  # nothing but the B1 replay builds it


def test_the_replay_reads_the_middle_of_the_step_by_default_and_the_start_on_request(monkeypatch):
    leads: list[float] = []
    real = coll.OpenLoopController.__init__

    def spy(self, t, u, dead_zone=0.0, lead=0.0):
        leads.append(lead)
        real(self, t, u, dead_zone, lead)

    monkeypatch.setattr(coll.OpenLoopController, "__init__", spy)
    from sagetrans.analysis.common import Scenario
    from sagetrans.physics.params import placeholder_vehicle

    prob = coll.make_problem(placeholder_vehicle(), Scenario(100.0), 22.0,
                             coll.CollocationOptions(n=20, max_iter=400))
    sol = coll.solve(prob, eps_h=10.0, eps_x=300.0)
    coll.replay(prob, sol)
    coll.replay(prob, sol, "start")
    assert leads == [pytest.approx(0.5 * DT_DEFAULT), 0.0]
