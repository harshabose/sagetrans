"""Run logging: the worker's files, the parent's lines, the heartbeat, `--status` and the flags."""

import json
import os
import shutil
from pathlib import Path

from sagetrans import progress
from sagetrans.io import cli, runlog, study
from sagetrans.io.runlog import RunLog, WorkerLog

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"


class Clock:
    def __init__(self, t: float = 1_800_000_000.0):
        self.t = t

    def __call__(self) -> float:
        return self.t


def _run(tmp_path, clock, level="normal", heartbeat=60.0):
    lines: list[str] = []
    rl = RunLog(tmp_path / "run", lines.append, level=level, heartbeat=heartbeat, clock=clock)
    return rl, lines


def test_a_terminal_line_is_ascii_only():
    text = "T° × – → ≤ µ café 中\r\tx"
    out = runlog.ascii_line(text)
    assert out.isascii() and "\r" not in out and "\t" not in out
    assert out.startswith("Tdeg x - -> <= u caf")


def test_state_files_are_replaced_atomically_and_parse_back(tmp_path, monkeypatch):
    path = tmp_path / "t.state.json"
    assert runlog.write_json_atomic(path, {"a": 1})
    assert runlog.read_json(path) == {"a": 1}
    assert runlog.write_json_atomic(path, {"a": 2})
    assert runlog.read_json(path) == {"a": 2}
    assert [p.name for p in tmp_path.iterdir()] == ["t.state.json"]  # no temp file left behind
    # it goes through os.replace, and a reader holding the file (PermissionError on Windows) only
    # costs a retry
    calls: list[int] = []
    real = os.replace

    def flaky(src, dst):
        calls.append(1)
        if len(calls) < 3:
            raise PermissionError("in use")
        return real(src, dst)

    monkeypatch.setattr(runlog.os, "replace", flaky)
    assert runlog.write_json_atomic(path, {"a": 3}) and len(calls) == 3
    assert runlog.read_json(path) == {"a": 3}
    assert runlog.read_json(tmp_path / "missing.json") is None
    (tmp_path / "bad.json").write_text("{not json", encoding="utf-8")
    assert runlog.read_json(tmp_path / "bad.json") is None


def test_the_worker_writes_every_event_and_the_state_at_most_every_interval(tmp_path):
    clk = Clock()
    w = WorkerLog(tmp_path, "FT_grid:12000", clock=clk, flusher=False)
    state0 = runlog.read_json(w.state_path)
    assert state0 is not None and state0["task"] == "FT_grid:12000" and state0["pid"] == os.getpid()
    progress.install(w)
    try:
        progress.phase("searching")
        progress.search_start("ft", 3, 12)
        for it in range(1, 500):  # a flood of events
            clk.t += 0.01
            progress.search_iter(2, it % 12 + 1, it * 8, 0.8 - it * 1e-4)
        progress.step("a", 3, 10)
    finally:
        progress.install(None)
    assert runlog.read_json(w.state_path) == state0  # events alone do not rewrite the state
    assert len(w.log_path.read_text(encoding="utf-8").splitlines()) >= 500
    clk.t += 5
    w.flush()
    st = runlog.read_json(w.state_path)
    assert st is not None and st["phase"] == "searching"
    assert st["search"]["seed"] == 2 and st["search"]["n_seeds"] == 3 and st["search"]["evals"] > 0
    assert st["step"]["done"] == 3 and st["step"]["total"] == 10
    assert st["elapsed_s"] >= 5 and st["status"] == "running"
    w.close(True)
    assert runlog.read_json(w.state_path)["status"] == "done"


def test_a_log_line_has_a_time_a_kind_and_json(tmp_path):
    w = WorkerLog(tmp_path, "C", clock=Clock(), flusher=False)
    w("search_seed_done", {"seed_idx": 1, "f": 1.5, "evals": 160, "secs": 12.0})
    w.close(True)
    row = w.log_path.read_text(encoding="utf-8").splitlines()[0].split("\t")
    assert len(row) == 3 and row[1] == "search_seed_done"
    assert json.loads(row[2])["evals"] == 160 and row[0].count(":") == 2


def _worker_with_search(tmp_path, clk, task="FT_grid:12000"):
    w = WorkerLog(tmp_path / "run" / "_logs", task, clock=clk, flusher=False)
    return w


def test_the_parent_prints_starts_seeds_and_ends_and_flags_a_silent_task(tmp_path):
    clk = Clock()
    rl, lines = _run(tmp_path, clk)
    rl.submit(["FT_grid:12000"])
    rl.poll()
    assert lines == []  # queued: no state file yet, so no start
    w = _worker_with_search(tmp_path, clk)
    rl.poll()
    assert any("start   FT_grid:12000" in x for x in lines)
    w("search_start", {"name": "ft", "n_seeds": 3, "maxiter": 12})
    clk.t += 60
    w("search_seed_done", {"seed_idx": 3, "f": 0.7741, "evals": 412, "secs": 1104.0})
    w.flush()
    rl.poll()
    seed = [x for x in lines if "seed 3/3 done" in x]
    assert seed and "J=0.7741" in seed[0] and "412 evals" in seed[0] and "18.4 min" in seed[0]
    n = len(lines)
    clk.t += 400  # nothing happens for 6.7 min
    w.flush()  # the flusher keeps `updated` fresh; only `last_event` shows the silence
    rl.poll()
    stale = [x for x in lines[n:] if "no update for" in x]
    assert len(stale) == 1 and "7 min" in stale[0]
    clk.t += 60
    w.flush()
    rl.poll()
    assert sum("no update for" in x for x in lines) == 1  # flagged once, not every poll
    w.close(True)
    rl.finish("FT_grid:12000", True, 1500.0)
    assert any(x.endswith("done    FT_grid:12000  1500 s (25.0 min)") for x in lines)
    assert all(x.isascii() for x in lines)
    # the file log has every line the terminal got, and each starts with HH:MM:SS
    text = rl.file_path.read_text(encoding="utf-8").splitlines()
    assert all(x in text for x in lines) and all(x[2] == ":" for x in text)


def test_the_heartbeat_with_a_running_search_has_phase_seed_best_and_a_rough_time_left(tmp_path):
    clk = Clock()
    rl, lines = _run(tmp_path, clk, level="verbose", heartbeat=60.0)
    rl.submit(["FT_grid:12000"])
    w = _worker_with_search(tmp_path, clk)
    rl.poll()
    progress.install(w)
    try:
        progress.phase("FT_grid 12 s: searching (CMA-ES)")
        progress.search_start("ft", 3, 12)
        for it in range(1, 7):  # seed 1, 6 of 12 iterations: 1/6 of the search in 300 s
            clk.t += 50
            progress.search_iter(1, it, 8 * it, 0.9 - 0.01 * it)
    finally:
        progress.install(None)
    w.flush()
    clk.t += 1
    rl.poll()
    assert any("heartbeat" in x and "1 running" in x for x in lines)
    row = next(x for x in lines if "it 6/12" in x and "searching" in x)
    assert "FT_grid:12000" in row and "seed 1/3" in row and "best 0.84" in row
    assert "~25.1 min left" in row  # 301 s for 1/6 of the work -> about 1500 s more
    st = runlog.read_json(rl.logs / "status.json")
    assert st is not None and st["counts"]["running"] == 1 and st["state"] == "running"
    shown = runlog.read_status(rl.logs.parent, now=clk.t)
    assert shown is not None and any("FT_grid:12000" in x and "~" in x for x in shown)
    w.close(True)


def test_at_normal_the_terminal_gets_a_heartbeat_block_every_fifteen_minutes(tmp_path):
    clk = Clock()
    rl, lines = _run(tmp_path, clk, level="normal", heartbeat=60.0)
    rl.submit(["C"])
    w = _worker_with_search(tmp_path, clk, "C")
    rl.poll()
    beats = 0
    for _ in range(60):  # one hour in one-minute steps
        clk.t += 60
        w("step", {"label": "x", "done": 1, "total": 40})
        w.flush()
        rl.poll()
        beats += 1
    shown = [x for x in lines if "heartbeat" in x]
    assert 3 <= len(shown) <= 5, shown  # about hourly / 15 min, not 60
    text = rl.file_path.read_text(encoding="utf-8").splitlines()
    file_beats = [x for x in text if "heartbeat" in x]
    assert len(file_beats) >= 59  # the file log has every beat
    w.close(True)


def test_a_failure_prints_the_traceback_tail_and_the_log_path(tmp_path):
    clk = Clock()
    rl, lines = _run(tmp_path, clk, level="quiet")
    rl.submit(["C"])
    w = WorkerLog(tmp_path / "run" / "_logs", "C", clock=clk, flusher=False)
    w.traceback(["Traceback (most recent call last):", "  File x.py, line 3", "ValueError: boom"])
    w.close(False)
    rl.finish("C", False, 12.0, "ValueError: boom")
    text = "\n".join(lines)
    assert "FAILED  C" in text and "ValueError: boom" in text and "C.log" in text
    rl.broken_pool(4)
    assert sum("fewer --workers" in x for x in lines) == 1


def test_a_task_that_raises_is_reported_through_the_worker(tmp_path):
    logs = tmp_path / "_logs"
    res = study._worker(str(CONFIGS), "draft", (), "nonsense:1", str(tmp_path / "x.pkl"), str(logs))
    assert res[0] == "nonsense:1" and res[1] is False and "unknown task" in res[3]
    text = (logs / "nonsense_1.log").read_text(encoding="utf-8")
    assert "\ttraceback\t" in text and "ValueError" in text
    state = runlog.read_json(logs / "nonsense_1.state.json")
    assert state is not None and state["status"] == "failed"
    assert not progress.active()  # the worker removed its sink again


def test_status_on_an_empty_folder_exits_2_and_writes_nothing(tmp_path, capsys):
    empty = tmp_path / "nothing_here"
    assert cli.main(["--status", str(empty)]) == 2
    assert f"no status in {empty}" in capsys.readouterr().out
    assert not empty.exists()
    empty.mkdir()
    assert cli.main(["--status", str(empty)]) == 2
    assert list(empty.iterdir()) == []


def test_status_cannot_be_combined_with_a_run(tmp_path, capsys):
    import pytest

    with pytest.raises(SystemExit) as exc:
        cli.main(["--status", str(tmp_path), "--only", "D"])
    assert exc.value.code == 2
    assert "--only" in capsys.readouterr().err


def test_the_log_flags_are_not_part_of_the_cache_key(tmp_path):
    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    (cfg / "study.yaml").write_text(
        "run_name: smoke\nmode: draft\nanalyses: [F]\nparallel: {enabled: false}\n"
        "F: {scenarios: [S-01]}\n", encoding="utf-8")
    out = tmp_path / "res"
    first = study.run_study(cfg, out_dir=out, workers=1, log=lambda s: None, log_level="quiet")
    assert first.exit_code == study.EXIT_OK and first.tasks == {"basics": "ran"}
    again = study.run_study(cfg, out_dir=out, workers=1, log=lambda s: None,
                            log_level="verbose", heartbeat=1.0)
    assert again.tasks == {"basics": "cached"}  # another level or heartbeat is the same key
    logs = out / "smoke" / "_logs"
    assert (logs / "status.json").is_file() and len(list(logs.glob("run_*.log"))) == 2


def test_a_full_study_stays_inside_the_terminal_line_budget(tmp_path):
    """Replay the event stream of the last full run (30 tasks) through the printer."""
    clk = Clock()
    rl, lines = _run(tmp_path, clk, level="normal", heartbeat=60.0)
    tasks = ([f"FT_grid:{q}" for q in (2, 3, 4, 5, 6, 8, 10, 12, 14, 16, 20)]
             + [f"FT_scan:{v}" for v in range(15, 21)]
             + ["C", "sensitivity", "basics", "FT_grid:11", "FT_grid:11.5", "FT:18", "basics_rec"])
    rl.start(["banner"] * 18, 3.0 * 3600)
    rl.wave_start(1, tasks, 23)
    rl.submit(tasks)
    workers = {t: WorkerLog(tmp_path / "run" / "_logs", t, clock=clk, flusher=False) for t in tasks}
    length = {t: (100 + 10 * i) * 60.0 for i, t in enumerate(tasks)}  # staggered, up to 3.5 h
    began = clk.t
    done: set[str] = set()
    tick = 60.0
    n_iter = 12
    for step_no in range(int(3.5 * 3600 / tick)):
        clk.t += tick
        for t, w in workers.items():
            if t in done:
                continue
            frac = (clk.t - began) / length[t]
            if frac >= 1:
                w.close(True)
                rl.finish(t, True, length[t])
                done.add(t)
                continue
            if t.startswith(("FT", "C")):
                if step_no == 0:
                    w("search_start", {"name": "ft", "n_seeds": 3, "maxiter": n_iter})
                k = min(int(frac * 3), 2)
                w("search_iter", {"seed_idx": k + 1, "it": int((frac * 3 - k) * n_iter) + 1,
                                  "evals": 8, "best": 0.8})
                if int(frac * 3) > int((frac - tick / length[t]) * 3) and k > 0:
                    w("search_seed_done", {"seed_idx": k, "f": 0.8, "evals": 96, "secs": 1000.0})
            else:
                w("step", {"label": "solve", "done": 1 + int(frac * 40), "total": 41})
            w.flush()
        rl.poll()
    for t in tasks:
        if t not in done:
            rl.finish(t, True, length[t])
    rl.wave_end(1, 3.5 * 3600, 0)
    rl.summary(3.5 * 3600, {}, "python run.py --view")
    assert len(lines) <= 400, len(lines)
    assert all(x.isascii() for x in lines)


def test_evaluations_keep_a_task_alive_between_generations_and_the_log_stays_short(tmp_path):
    clk = Clock()
    rl, lines = _run(tmp_path, clk)
    rl.submit(["FT_grid:17000"])
    w = _worker_with_search(tmp_path, clk, "FT_grid:17000")
    rl.poll()
    w("search_start", {"name": "ft", "n_seeds": 3, "maxiter": 12})
    for i in range(1, 21):  # 20 minutes without a generation ending, an evaluation every minute
        clk.t += 60
        w("search_eval", {"evals": i})
        w.flush()
        rl.poll()
    assert not any("no update for" in x for x in lines)  # before: flagged after 5 minutes
    st = runlog.read_json(w.state_path)
    assert st is not None and st["search"]["evals_total"] == 20
    # a burst of 1000 evaluations in one second writes one log line, not 1000
    before = len(w.log_path.read_text(encoding="utf-8").splitlines())
    for i in range(1000):
        w("search_eval", {"evals": 100 + i})
    after = len(w.log_path.read_text(encoding="utf-8").splitlines())
    assert after - before <= 1
    w.close(True)


def test_the_banner_estimate_is_the_observed_wall_time_of_a_full_study():
    from sagetrans.io.inputs import load_inputs
    from sagetrans.io.stages import TASK_COST, plan_tasks

    inp = load_inputs(ROOT / "configs")
    tasks = plan_tasks(inp)
    wall, cached = study._task_estimate(inp, None, tasks, 24)
    assert cached == 0 and 3.8 * 3600 <= wall <= 4.4 * 3600  # the clean run took 4.1 h
    assert wall >= TASK_COST["C"] + TASK_COST["FT_fine"] + TASK_COST["FT"]  # C, then the follow-ups
    few, _ = study._task_estimate(inp, None, tasks, 4)
    assert few > wall  # fewer workers than tasks: the work divided by the workers dominates
    assert study._task_estimate(inp, None, [], 24) == (0.0, 0)
    assert plan_tasks(inp)[0] == "C"  # the longest task starts first
