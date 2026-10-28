"""The progress hook (`sagetrans.progress`): a no-op without a sink, never a change of result."""

import numpy as np
import pytest

from sagetrans import progress
from sagetrans.analysis.search import cma_minimise


@pytest.fixture(autouse=True)
def _no_sink_left_behind():
    progress.install(None)
    yield
    progress.install(None)


def _bowl(u: np.ndarray) -> float:
    return float(np.sum((u - 0.3) ** 2) + 0.1 * np.sin(9 * u[0]))


def test_the_hook_is_a_no_op_until_a_sink_is_installed():
    assert not progress.active()
    progress.phase("x")
    progress.step("x", 1, 2)
    progress.search_start("s", 3, 10)
    progress.search_iter(1, 1, 8, 0.5)
    progress.search_seed_done(1, 0.5, 8, 1.0)
    progress.search_done(0.5, 0.0, 1.0)
    progress.note("n")  # none of these may raise or print


def test_a_sink_gets_the_events_and_a_failing_sink_is_swallowed():
    got: list[tuple[str, dict]] = []
    progress.install(lambda kind, f: got.append((kind, f)))
    progress.phase("searching")
    progress.step("one", 1, 4)
    assert got == [("phase", {"label": "searching"}),
                   ("step", {"label": "one", "done": 1, "total": 4})]

    def broken(kind: str, fields: dict) -> None:
        raise RuntimeError("disk full")

    progress.install(broken)
    progress.phase("still fine")  # a logging fault never reaches the computation


def test_a_search_with_a_sink_is_bit_identical_to_one_without():
    u0 = np.array([0.8, 0.7, 0.2])
    plain = cma_minimise(_bowl, u0, seeds=(1, 2), popsize=6, maxiter=8)
    got: list[str] = []
    progress.install(lambda kind, f: got.append(kind))
    seen = cma_minimise(_bowl, u0, seeds=(1, 2), popsize=6, maxiter=8, name="bowl")
    assert seen.best_f == plain.best_f and seen.f_start == plain.f_start
    assert np.array_equal(seen.best_x, plain.best_x)
    assert [(s.seed, s.f, s.n_evals) for s in seen.seeds] == [
        (s.seed, s.f, s.n_evals) for s in plain.seeds]
    assert all(np.array_equal(a.x, b.x) for a, b in zip(seen.seeds, plain.seeds, strict=True))
    # and the events are there: one start, iterations, one per seed, one end
    assert got[0] == "search_start" and got[-1] == "search_done"
    assert got.count("search_seed_done") == 2 and got.count("search_iter") >= 2


def test_a_search_reports_its_seed_and_iteration_numbers():
    ev: list[tuple[str, dict]] = []
    progress.install(lambda kind, f: ev.append((kind, f)))
    cma_minimise(_bowl, np.array([0.5, 0.5]), seeds=(1, 2, 3), popsize=4, maxiter=3, name="n")
    start = ev[0][1]
    assert start == {"name": "n", "n_seeds": 3, "maxiter": 3}
    its = [f for k, f in ev if k == "search_iter"]
    assert {f["seed_idx"] for f in its} == {1, 2, 3}
    assert all(f["it"] <= 3 for f in its)
    assert [f["evals"] for f in its if f["seed_idx"] == 1] == sorted(
        f["evals"] for f in its if f["seed_idx"] == 1)
    done = [f for k, f in ev if k == "search_seed_done"]
    assert [f["seed_idx"] for f in done] == [1, 2, 3] and all(f["secs"] >= 0 for f in done)
