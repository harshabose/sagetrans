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
import shutil
import subprocess
import sys
import time
import traceback
import warnings
from collections import Counter
from collections.abc import Callable
from concurrent.futures.process import BrokenProcessPool
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

import sagetrans
from sagetrans import progress
from sagetrans.config.schema import DraftConfigError
from sagetrans.io import gate, record, runlog
from sagetrans.io.assemble import assemble
from sagetrans.io.inputs import InputError, StudyInputs, load_inputs, release_blockers
from sagetrans.io.stages import TASK_COST, follow_up_tasks, plan_tasks, run_task, task_selection

EXIT_OK, EXIT_TASK_FAILED, EXIT_INPUTS, EXIT_GATE = 0, 1, 2, 3


def _print(line: str) -> None:
    print(runlog.ascii_line(line), flush=True)  # flushed: progress must show while a run is going


@dataclass
class StudyOutcome:
    exit_code: int
    folder: Path | None = None
    gate: gate.GateResult | None = None
    label: str = ""
    tasks: dict[str, str] = field(default_factory=dict)  # task -> "ran" | "cached" | "failed: ..."
    task_seconds: dict[str, float] = field(default_factory=dict)  # task -> seconds it took
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
    key = (f"{inp.inputs_hash[:16]}.{sagetrans.__version__}.{record.source_hash()[:12]}"
           f".{record.library_hash()[:8]}")
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
    config_dir: str, mode: str, only: tuple[str, ...], task: str, cache: str, logs: str = ""
) -> tuple[str, bool, float, str]:
    """Run one task in a worker process and write its result to the cache.

    With `logs` (the run's `_logs` folder) the worker installs its own progress sink: the task's
    log and state files. The sink only reports; the result is the same without it.
    """
    warnings.filterwarnings("ignore")
    t0 = time.time()
    sink = runlog.WorkerLog(Path(logs), task) if logs else None
    progress.install(sink)
    ok = False
    try:
        progress.phase("loading the inputs")
        inp = load_inputs(config_dir, mode, set(only) or None)
        result = run_task(inp, task)
        progress.phase("writing the result")
        path = Path(cache)
        path.parent.mkdir(parents=True, exist_ok=True)
        tmp = path.with_suffix(".tmp")
        with tmp.open("wb") as fh:
            pickle.dump(result, fh)
        tmp.replace(path)
        ok = True
        return task, True, time.time() - t0, ""
    except Exception as exc:  # noqa: BLE001
        if sink is not None:
            sink.traceback(traceback.format_exc().strip().splitlines()[-8:])
        return task, False, time.time() - t0, f"{type(exc).__name__}: {exc}"
    finally:
        progress.install(None)
        if sink is not None:
            sink.close(ok)


# ---- pre-flight --------------------------------------------------------------------------------
def _task_estimate(
    inp: StudyInputs, work: Path | None, tasks: list[str], workers: int
) -> tuple[float, int]:
    """(wall seconds still to go, tasks that are cached) for the planned tasks.

    The first wave takes as long as its longest task or, if there are more tasks than workers, the
    work divided by the workers. The follow-up tasks (the fine pass, the FT report, E and Q2 at
    the recommended settings) wait for it, so they add up. The costs are the observed ones
    (`stages.TASK_COST`).
    """
    pending = [t for t in tasks if not (work and _cache_path(work, inp, t).is_file())]
    n_cached = len(tasks) - len(pending)
    if not pending:
        return 0.0, n_cached
    costs = [TASK_COST.get(t.split(":")[0], 100) for t in pending]
    wall = max(max(costs), sum(costs) / max(1, workers))
    wanted = set(inp.study.analyses)
    if "FT" in wanted and any(t.startswith("FT_grid:") for t in pending):
        if inp.study.FT.grid.fine_offsets_s:
            wall += TASK_COST["FT_fine"]
        wall += TASK_COST["FT"]
        if wanted & {"E", "Q2"}:
            wall += TASK_COST["basics_rec"]
    return float(wall), n_cached


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
    for t in tasks:
        cached = bool(work and _cache_path(work, inp, t).is_file())
        est = 0.0 if cached else TASK_COST.get(t.split(":")[0], 100)
        lines.append(f"  - {t:<14} {'cached' if cached else f'~{est:.0f} s'}")
    lines += wave_plan(inp)
    workers = _n_workers(inp, len(tasks), None)
    total, _ = _task_estimate(inp, work, tasks, workers)
    if total:
        mins = total / 60
        lines.append(f"estimated wall time: ~{mins:.0f}+ min on {workers} worker(s)")
    else:
        lines.append("estimated wall time: nothing to run (all cached)")
    return lines


def wave_plan(inp: StudyInputs) -> list[str]:
    """Which tasks run together and which wait for which (later waves are planned from results)."""
    wanted = set(inp.study.analyses)
    kinds = Counter(t.split(":")[0] for t in plan_tasks(inp))
    first = ", ".join(f"{k} x{n}" if n > 1 else k for k, n in kinds.items())
    lines = [f"wave plan: wave 1 = {first} (they start together)"]
    if "FT" in wanted:
        lines.append("  wave 2 = the fine pass around the best FT_grid point "
                     "(waits for the FT_grid points of wave 1)")
        lines.append("  wave 3 = the FT report of the winner (waits for wave 2)")
        if wanted & {"E", "Q2"}:
            lines.append("  wave 4 = D, E and Q2 at the recommended settings "
                         "(waits for the FT report)")
    return lines


def banner(inp: StudyInputs, work: Path, workers: int | None) -> list[str]:
    """The start of a run: the tree, the libraries, the settings, the plan and the time estimate."""
    sha, dirty = record.git_state()
    libs = record.library_versions()
    tasks = plan_tasks(inp)
    n_work = _n_workers(inp, len(tasks), workers)
    total, n_cached = _task_estimate(inp, work, tasks, n_work)
    est = (f"estimated wall time ~{total / 60:.0f}+ min on {n_work} worker(s)"
           if total else "nothing to run: every task is cached")
    kinds = Counter(t.split(":")[0] for t in tasks)
    lines = [
        f"sagetrans {sagetrans.__version__}  git {sha}{' (dirty)' if dirty else ''}  "
        f"source hash {record.source_hash()[:12]}",
        "python " + libs["python"] + "  " + "  ".join(f"{k} {v}" for k, v in libs.items()
                                                       if k != "python"),
        f"mode {inp.mode}  workers {n_work}  register hash {inp.config.config_hash()[:12]}  "
        f"inputs hash {inp.inputs_hash[:12]}",
        f"cache folder: {work}",
        f"tasks: {len(tasks) - n_cached} to run, {n_cached} cached "
        f"({', '.join(f'{k} x{n}' if n > 1 else k for k, n in kinds.items())}); {est}",
        *wave_plan(inp),
    ]  # fmt: skip
    if inp.draft_reasons:
        lines.append(f"NOT release grade: {len(inp.draft_reasons)} reason(s); see report.md")
    return lines


def _n_workers(inp: StudyInputs, n_tasks: int, override: int | None) -> int:
    if override is not None:
        return max(1, override)
    if not inp.study.parallel.enabled:
        return 1
    cap = inp.study.parallel.max_workers or (os.cpu_count() or 1)
    return max(1, min(cap, max(1, n_tasks)))


BLAS_THREAD_VARS = ("OPENBLAS_NUM_THREADS", "OMP_NUM_THREADS", "MKL_NUM_THREADS")


def _one_blas_thread_per_worker() -> None:
    """Give each worker process one linear-algebra thread (a value the user set is kept).

    The task matrices are small. Without this every worker starts one OpenBLAS thread per core, and
    16 workers on a 24-core machine ran out of memory (BrokenProcessPool) in the acceptance run.
    Spawned workers inherit the environment, so it is set before the pool starts.
    """
    for var in BLAS_THREAD_VARS:
        os.environ.setdefault(var, "1")


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
    fresh_cache: bool = False,
    log: Callable[[str], None] = _print,
    log_level: str = "normal",
    heartbeat: float = 60.0,
) -> StudyOutcome:
    """Load -> check -> run tasks -> assemble. Never raises for bad inputs; see `exit_code`.

    `fresh_cache` deletes the task cache (`<run>/_work`, stale entries included) before the run, so
    every task is computed: the start of an acceptance run. `force` only ignores what it finds.
    `log` receives the terminal lines; `log_level` (quiet, normal, verbose) and `heartbeat`
    (seconds) only say how much is printed and how often: they are not inputs of any result and
    are not part of the cache key.
    """
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
    run_dir = root / inp.study.run_name
    work = run_dir / "_work"
    if fresh_cache and work.is_dir():
        shutil.rmtree(work)
    outcome = StudyOutcome(EXIT_OK)
    rl = runlog.RunLog(run_dir, log, level=log_level, heartbeat=heartbeat)
    est_tasks = plan_tasks(inp)
    est_total, _ = _task_estimate(inp, work, est_tasks, _n_workers(inp, len(est_tasks), workers))
    rl.start(banner(inp, work, workers), est_total or None)

    results: dict[str, dict[str, Any]] = {}
    failed: dict[str, str] = {}
    sel = tuple(sorted(only)) if only else ()
    planned: set[str] = set()
    wave = plan_tasks(inp)
    n_wave = 1
    try:
        while wave:  # a later wave is planned from the results of the earlier ones
            planned.update(wave)
            pending: list[str] = []
            for t in wave:
                cached = None if force else _load_cached(_cache_path(work, inp, t))
                if cached is not None:
                    results[t] = cached
                    outcome.tasks[t] = "cached"
                    rl.cached(t)
                else:
                    pending.append(t)
            _run_wave(inp, work, pending, sel, workers, n_wave, results, failed, outcome, rl)
            wave = [t for t in follow_up_tasks(inp, results, failed) if t not in planned]
            n_wave += 1

        verification: dict[str, bool] = {}
        if verify:
            junit = _run_tests(run_dir, rl.out)
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
        rl.summary(outcome.seconds, failed, "python run.py --view")
        return outcome
    finally:
        rl.close("failed" if failed else "finished")


def _run_wave(
    inp: StudyInputs, work: Path, pending: list[str], sel: tuple[str, ...],
    workers: int | None, n_wave: int, results: dict[str, dict[str, Any]],
    failed: dict[str, str], outcome: StudyOutcome, rl: runlog.RunLog,
) -> None:
    """Run the pending tasks of one wave, in worker processes unless there is one worker."""
    if not pending:
        return
    n_workers = _n_workers(inp, len(pending), workers)
    t_wave, n_failed = time.time(), len(failed)
    rl.wave_start(n_wave, pending, n_workers)
    rl.submit(pending)
    rl.monitor()  # a thread that tails the task logs and writes the heartbeat
    logs = str(rl.logs)
    if n_workers == 1:
        for t in pending:
            _record(_worker(str(inp.config_dir), inp.mode, sel, t,
                            str(_cache_path(work, inp, t)), logs),
                    inp, work, results, failed, outcome, rl)  # fmt: skip
    else:
        _one_blas_thread_per_worker()
        ctx = multiprocessing.get_context("spawn")
        died = False
        n_back = 0
        with cf.ProcessPoolExecutor(max_workers=n_workers, mp_context=ctx) as pool:
            futs = {
                pool.submit(_worker, str(inp.config_dir), inp.mode, sel, t,
                            str(_cache_path(work, inp, t)), logs): t for t in pending
            }  # fmt: skip
            for fut in cf.as_completed(futs):
                try:
                    res = fut.result()
                except BrokenProcessPool:
                    if not died:
                        died = True
                        rl.broken_pool(len(futs) - n_back)  # one line; the rest fail quietly
                    res = (futs[fut], False, time.time() - t_wave,
                           "the worker process died (usually out of memory)")  # fmt: skip
                except Exception as exc:  # noqa: BLE001  e.g. a result that cannot be sent back
                    res = (futs[fut], False, time.time() - t_wave, f"{type(exc).__name__}: {exc}")
                n_back += 1
                _record(res, inp, work, results, failed, outcome, rl)
    rl.wave_end(n_wave, time.time() - t_wave, len(failed) - n_failed)


def _record(
    res: tuple[str, bool, float, str], inp: StudyInputs, work: Path,
    results: dict[str, dict[str, Any]], failed: dict[str, str], outcome: StudyOutcome,
    rl: runlog.RunLog,
) -> None:
    task, ok, secs, err = res
    outcome.task_seconds[task] = secs
    if ok:
        loaded = _load_cached(_cache_path(work, inp, task))
        if loaded is None:
            failed[task] = "result could not be read back from the cache"
            outcome.tasks[task] = "failed: unreadable result"
            rl.finish(task, False, secs, failed[task])
        else:
            results[task] = loaded
            outcome.tasks[task] = "ran"
            rl.finish(task, True, secs)
    else:
        failed[task] = err
        outcome.tasks[task] = f"failed: {err}"
        rl.finish(task, False, secs, err)


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
