"""Progress events from long computations, for whoever is watching (a no-op by default).

This module imports nothing from the package, so every layer may import it. Analysis code reports
what it is doing here and never prints. Nothing happens unless a sink is installed (`install`):
a worker process installs one that writes the task's log; tests and library users install none.

A hook only tells the sink what already happened. It never calls the function being run, never
touches a random generator and never changes control flow, so a run with a sink returns the same
numbers as one without. An exception inside a sink is swallowed for the same reason: a logging
fault must not change or stop a result.
"""

from __future__ import annotations

from collections.abc import Callable
from typing import Any

Sink = Callable[[str, dict[str, Any]], None]

_sink: Sink | None = None


def install(sink: Sink | None) -> None:
    """Send every event to `sink(kind, fields)`; `None` removes it again."""
    global _sink
    _sink = sink


def active() -> bool:
    return _sink is not None


def _emit(kind: str, **fields: Any) -> None:
    sink = _sink
    if sink is None:
        return
    try:
        sink(kind, fields)
    except Exception:  # noqa: BLE001  a logging fault must never reach the computation
        pass


def phase(label: str) -> None:
    """What the task is doing now (searching, final ensemble, histories, tables)."""
    _emit("phase", label=label)


def search_start(name: str, n_seeds: int, maxiter: int) -> None:
    _emit("search_start", name=name, n_seeds=n_seeds, maxiter=maxiter)


def search_iter(seed_idx: int, it: int, evals: int, best: float) -> None:
    """One CMA-ES generation of seed `seed_idx` (1-based): its number, the evaluations so far."""
    _emit("search_iter", seed_idx=seed_idx, it=it, evals=evals, best=best)


def search_seed_done(seed_idx: int, f: float, evals: int, secs: float) -> None:
    _emit("search_seed_done", seed_idx=seed_idx, f=f, evals=evals, secs=secs)


def search_done(best: float, spread: float, secs: float) -> None:
    _emit("search_done", best=best, spread=spread, secs=secs)


def note(text: str, warn: bool = False) -> None:
    """A line for the task's log; `warn` also puts it on the terminal (a failed piece of a task)."""
    _emit("note", text=text, warn=warn)


def step(label: str, done: int, total: int) -> None:
    """`done` of `total` units of work finished; `label` names the one that just finished."""
    _emit("step", label=label, done=done, total=total)
