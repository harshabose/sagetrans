"""Run a whole study from a config folder: load, check, run tasks in parallel, assemble.

    from sagetrans.io.study import run_study
    outcome = run_study("configs")

Tasks run in separate processes (CasADi objects cannot be shared between processes, so each
worker reloads the config folder itself). Every task result is cached on disk under the run
folder, keyed by a hash of every result-relevant input, so re-running with unchanged inputs only
re-assembles the report, and changing one input re-runs only what is needed.
"""

from __future__ import annotations

import concurrent.futures as cf
import multiprocessing
import os
import pickle
import subprocess
import sys
import time
import warnings
from collections.abc import Callable
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import sagetrans
from sagetrans.config.schema import DraftConfigError
from sagetrans.io import gate, record
from sagetrans.io.assemble import assemble
from sagetrans.io.inputs import InputError, StudyInputs, load_inputs, release_blockers
from sagetrans.io.stages import TASK_COST, plan_tasks, run_task, task_selection

EXIT_OK, EXIT_TASK_FAILED, EXIT_INPUTS, EXIT_GATE = 0, 1, 2, 3


def _print(line: str) -> None:
    print(line, flush=True)  # flushed: progress must show up while a long run is going


@dataclass
class StudyOutcome:
    exit_code: int
    folder: Path | None = None
    gate: gate.GateResult | None = None
    label: str = ""
    tasks: dict[str, str] = field(default_factory=dict)  # task -> "ran" | "cached" | "failed: ..."
    seconds: float = 0.0
    messages: list[str] = field(default_factory=list)


# ---- cache -------------------------------------------------------------------------------------
def _cache_path(work: Path, inp: StudyInputs, task: str) -> Path:
    safe = task.replace(":", "_")
    sel = task_selection(inp, task)
    tag = f".{sel}" if sel else ""
    # inputs hash: the register and the settings; source hash: the code that decides what a task
    # computes. An entry written under another pair (or by a version before the source hash existed,
    # which has no third field in its name) is never looked up, so it cannot be misread.
    key = f"{inp.inputs_hash[:16]}.{sagetrans.__version__}.{record.source_hash()[:12]}"
    return work / f"{safe}{tag}.{key}.pkl"


def _load_cached(path: Path) -> dict[str, Any] | None:
    if path.is_file():
        try:
            with path.open("rb") as fh:
                data: dict[str, Any] = pickle.load(fh)
            return data
        except (pickle.UnpicklingError, EOFError, AttributeError, ImportError):
            return None  # an unreadable cache entry is just a miss
    return None


def _worker(
    config_dir: str, mode: str, only: tuple[str, ...], task: str, cache: str
) -> tuple[str, bool, float, str]:
    """Run one task in a worker process and write its result to the cache."""
    warnings.filterwarnings("ignore")
    t0 = time.time()
    try:
        inp = load_inputs(config_dir, mode, set(only) or None)
        result = run_task(inp, task)
        path = Path(cache)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        with tmp.open("wb") as fh:
            pickle.dump(result, fh)
        tmp.replace(path)
        return task, True, time.time() - t0, ""
    except Exception as exc:  # noqa: BLE001
        return task, False, time.time() - t0, f"{type(exc).__name__}: {exc}"


# ---- pre-flight --------------------------------------------------------------------------------
def preflight(inp: StudyInputs, work: Path | None = None) -> list[str]:
    """Human-readable summary of what would run, with a rough time estimate."""
    counts = inp.config.evidence_counts()
    lines = [
        f"mode: {inp.mode}    register hash: {inp.config.config_hash()[:12]}    "
        f"inputs hash: {inp.inputs_hash[:12]}",
        "register entries by evidence grade: "
        + ", ".join(f"{k} {v}" for k, v in counts.items() if v),
    ]
    if inp.draft_reasons:
        lines.append("NOT release grade:")
        lines += [f"  - {r}" for r in inp.draft_reasons]
    else:
        lines.append("inputs are release grade (nothing TBD, nothing missing)")
    tasks = plan_tasks(inp)
    lines.append("analyses: " + ", ".join(inp.study.analyses))
    lines.append("tasks:")
    total = 0.0
    for t in tasks:
        cached = bool(work and _cache_path(work, inp, t).is_file())
        est = 0.0 if cached else TASK_COST.get(t.split(":")[0], 100)
        total += est
        lines.append(f"  - {t:<14} {'cached' if cached else f'~{est:.0f} s'}")
    workers = _n_workers(inp, len(tasks), None)
    if total:
        mins = total / max(1, workers) / 60
        lines.append(f"estimated wall time: ~{mins:.0f}+ min on {workers} worker(s)")
    else:
        lines.append("estimated wall time: nothing to run (all cached)")
    return lines


def _n_workers(inp: StudyInputs, n_tasks: int, override: int | None) -> int:
    if override is not None:
        return max(1, override)
    if not inp.study.parallel.enabled:
        return 1
    cap = inp.study.parallel.max_workers or (os.cpu_count() or 1)
    return max(1, min(cap, max(1, n_tasks)))


# ---- the run -----------------------------------------------------------------------------------
def run_study(
    config_dir: str | Path,
    *,
    mode: str | None = None,
    only: set[str] | None = None,
    force: bool = False,
    workers: int | None = None,
    out_dir: str | Path | None = None,
    junit: str | Path | None = None,
    verify: bool = False,
    log: Callable[[str], None] = _print,
) -> StudyOutcome:
    """Load -> check -> run tasks -> assemble. Never raises for bad inputs; see `exit_code`."""
    t_start = time.time()
    try:
        inp = load_inputs(config_dir, mode, only)
    except InputError as exc:
        return StudyOutcome(EXIT_INPUTS, messages=list(exc.problems))
    if inp.mode == "release" and release_blockers(inp):
        return StudyOutcome(
            EXIT_INPUTS,
            messages=["release mode refuses to run: the inputs are not release grade:",
                      *[f"  - {r}" for r in release_blockers(inp)]],
        )  # fmt: skip
    root = Path(out_dir) if out_dir else Path(inp.study.output_dir)
    if not root.is_absolute():
        root = Path.cwd() / root
    work = root / inp.study.run_name / "_work"
    outcome = StudyOutcome(EXIT_OK)
    for line in preflight(inp, work):
        log(line)

    tasks = plan_tasks(inp)
    results: dict[str, dict[str, Any]] = {}
    failed: dict[str, str] = {}
    pending: list[str] = []
    for t in tasks:
        cached = None if force else _load_cached(_cache_path(work, inp, t))
        if cached is not None:
            results[t] = cached
            outcome.tasks[t] = "cached"
            log(f"[cached] {t}")
        else:
            pending.append(t)

    sel = tuple(sorted(only)) if only else ()
    n_workers = _n_workers(inp, len(pending), workers)
    if pending:
        log(f"running {len(pending)} task(s) on {n_workers} worker(s) ...")
    if pending and n_workers == 1:
        for t in pending:
            _record(_worker(str(inp.config_dir), inp.mode, sel, t, str(_cache_path(work, inp, t))),
                    inp, work, results, failed, outcome, log)
    elif pending:
        ctx = multiprocessing.get_context("spawn")
        with cf.ProcessPoolExecutor(max_workers=n_workers, mp_context=ctx) as pool:
            futs = [pool.submit(_worker, str(inp.config_dir), inp.mode, sel, t,
                                str(_cache_path(work, inp, t))) for t in pending]  # fmt: skip
            for fut in cf.as_completed(futs):
                _record(fut.result(), inp, work, results, failed, outcome, log)

    verification: dict[str, bool] = {}
    if verify:
        junit = _run_tests(root / inp.study.run_name, log)
    if junit and Path(junit).is_file():
        verification = gate.verification_from_junit(Path(junit))

    try:
        folder, gate_res, rec = assemble(inp, results, failed, root, verification)
    except DraftConfigError as exc:
        outcome.exit_code = EXIT_INPUTS
        outcome.messages.append(str(exc))
        return outcome
    outcome.folder, outcome.gate, outcome.label = folder, gate_res, rec.label
    outcome.seconds = time.time() - t_start
    if failed:
        outcome.exit_code = EXIT_TASK_FAILED
    elif inp.mode == "release" and not gate_res.passed:
        outcome.exit_code = EXIT_GATE
    return outcome


def _record(
    res: tuple[str, bool, float, str], inp: StudyInputs, work: Path,
    results: dict[str, dict[str, Any]], failed: dict[str, str], outcome: StudyOutcome,
    log: Callable[[str], None],
) -> None:
    task, ok, secs, err = res
    if ok:
        loaded = _load_cached(_cache_path(work, inp, task))
        if loaded is None:
            failed[task] = "result could not be read back from the cache"
            outcome.tasks[task] = "failed: unreadable result"
        else:
            results[task] = loaded
            outcome.tasks[task] = "ran"
            log(f"[done  ] {task} ({secs:.0f} s)")
    else:
        failed[task] = err
        outcome.tasks[task] = f"failed: {err}"
        log(f"[FAILED] {task}: {err}")


def _run_tests(run_dir: Path, log: Callable[[str], None]) -> Path | None:
    """Run the test suite and return its JUnit file (for the release gate)."""
    root = Path(sagetrans.__file__).resolve().parents[2]
    if not (root / "tests").is_dir():
        log("--verify: no tests/ folder next to the package; skipping")
        return None
    run_dir.mkdir(parents=True, exist_ok=True)
    junit = run_dir / "junit.xml"
    log("running the verification tests (a few minutes) ...")
    subprocess.run(
        [sys.executable, "-m", "pytest", "-q", f"--junitxml={junit}"], cwd=root, check=False,
        capture_output=True,
    )  # fmt: skip
    return junit
