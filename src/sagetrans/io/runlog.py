"""Run logging: what a long study is doing, in files and on the terminal.

Each worker process installs a `WorkerLog` as the `sagetrans.progress` sink. It writes, under
``<run>/_logs/``, ``<task>.log`` (every event, with a time) and ``<task>.state.json`` (the latest
state, replaced atomically about every 5 s). The parent runs a `RunLog`: it tails the task logs,
prints plain ASCII lines ``HH:MM:SS  message`` (no colour, no carriage returns, flushed, so they
work in PyCharm, in Git Bash and in a redirected file), writes the full timestamped log to
``<run>/_logs/run_<date>_<time>.log`` and ``status.json`` at every heartbeat. `read_status` shows a
running or finished study from another terminal (`python run.py --status`).

Nothing here is imported by the analyses and nothing here changes a result.
"""

from __future__ import annotations

import json
import os
import threading
import time
from collections.abc import Callable, Mapping
from pathlib import Path
from typing import Any

LOGS = "_logs"
LEVELS = ("quiet", "normal", "verbose")
STATE_EVERY_S = 5.0  # a worker replaces its state file at most this often
STALE_S = 300.0  # no event for this long: the task is flagged
STALE_AGAIN_S = 900.0  # a flagged task is flagged again after this long
POLL_S = 2.0  # the parent looks at the task logs this often
NORMAL_BEAT_S = 900.0  # the terminal shows a heartbeat block this often at `normal` (see README)
BLOCK_LINES = 6  # task lines in a terminal heartbeat block at `normal`
RETRIES = 6

_ASCII = {
    0xB0: "deg", 0xD7: "x", 0x2013: "-", 0x2014: "-", 0x2192: "->", 0x2264: "<=", 0x2265: ">=",
    0xB5: "u", 0xB2: "2", 0xB3: "3", 0xB1: "+-", 0xB7: ".", 0x2248: "~", 0x394: "d",
    0x201C: '"', 0x201D: '"', 0x2018: "'", 0x2019: "'", 0x2026: "...", 0xA0: " ", 0x2212: "-",
}  # fmt: skip


def ascii_line(text: str) -> str:
    """One terminal line: ASCII only, no tabs, no carriage returns."""
    t = text.translate(_ASCII).replace("\t", " ").replace("\r", "")
    return t.encode("ascii", "replace").decode("ascii")


def stem(task: str) -> str:
    return task.replace(":", "_")


def clock_text(t: float) -> str:
    return time.strftime("%H:%M:%S", time.localtime(t))


def minutes_text(secs: float) -> str:
    if secs < 90:
        return f"{secs:.0f} s"
    if secs < 5400:
        return f"{secs / 60:.1f} min"
    return f"{secs / 3600:.1f} h"


# ---- files -------------------------------------------------------------------------------------
def write_json_atomic(path: Path, data: Mapping[str, Any]) -> bool:
    """Write `data` as JSON by replacing `path` in one step; False if it could not be done.

    On Windows `os.replace` raises PermissionError while a reader has the target open, so it is
    retried a few times; a reader never sees a half-written file.
    """
    tmp = path.with_name(f"{path.name}.{os.getpid()}.tmp")
    try:
        tmp.write_text(json.dumps(data, default=str), encoding="utf-8")
    except OSError:
        return False
    for i in range(RETRIES):
        try:
            os.replace(tmp, path)
            return True
        except PermissionError:
            time.sleep(0.02 * (i + 1))
        except OSError:
            break
    try:
        tmp.unlink()
    except OSError:
        pass
    return False


def read_json(path: Path) -> dict[str, Any] | None:
    """The JSON object in `path`, or None (missing, being replaced, unreadable)."""
    try:
        data = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError):
        return None
    return data if isinstance(data, dict) else None


# ---- the worker side ---------------------------------------------------------------------------
class WorkerLog:
    """The progress sink of one task: its log file and its state file."""

    def __init__(
        self, logs: Path, task: str, *, clock: Callable[[], float] = time.time,
        interval: float = STATE_EVERY_S, flusher: bool = True,
    ) -> None:  # fmt: skip
        logs.mkdir(parents=True, exist_ok=True)
        self.task, self.clock, self.interval = task, clock, interval
        self.log_path = logs / f"{stem(task)}.log"
        self.state_path = logs / f"{stem(task)}.state.json"
        self._lock = threading.Lock()
        self._last_eval_log = -1e18
        self._fh = self.log_path.open("w", encoding="utf-8", buffering=1)
        now = clock()
        self.state: dict[str, Any] = {
            "task": task, "pid": os.getpid(), "phase": "starting", "started": now,
            "last_event": now, "updated": now, "elapsed_s": 0.0, "search": None, "step": None,
            "status": "running",
        }  # fmt: skip
        self._write_state()  # the state file appearing is the parent's "start"
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        if flusher:
            self._thread = threading.Thread(target=self._flush_loop, daemon=True)
            self._thread.start()

    # state ----------------------------------------------------------------------------------
    def _write_state(self) -> None:
        now = self.clock()
        self.state["updated"], self.state["elapsed_s"] = now, now - self.state["started"]
        write_json_atomic(self.state_path, self.state)

    def _flush_loop(self) -> None:
        while not self._stop.wait(self.interval):
            with self._lock:
                self._write_state()

    def flush(self) -> None:
        with self._lock:
            self._write_state()

    # events ---------------------------------------------------------------------------------
    def __call__(self, kind: str, f: dict[str, Any]) -> None:
        now = self.clock()
        with self._lock:
            st = self.state
            st["last_event"] = now
            if kind == "search_eval":
                # one per cost evaluation: it keeps the task visibly alive between generations
                # (one generation of a 16-member ensemble takes minutes under load). The state
                # takes it at once; the log gets a line at most every `interval` seconds.
                if st["search"] is not None:
                    st["search"]["evals_total"] = f["evals"]
                if now - self._last_eval_log < self.interval:
                    return
                self._last_eval_log = now
            ms = int((now % 1) * 1000)
            self._fh.write(f"{clock_text(now)}.{ms:03d}\t{kind}\t{json.dumps(f, default=str)}\n")
            if kind == "phase":
                st["phase"] = f["label"]
            elif kind == "search_start":
                st["search"] = {"name": f["name"], "n_seeds": f["n_seeds"],
                                "maxiter": f["maxiter"], "seed": 1, "it": 0, "evals": 0,
                                "best": None, "started": now, "seeds_done": 0}  # fmt: skip
            elif kind == "search_iter" and st["search"] is not None:
                st["search"].update(seed=f["seed_idx"], it=f["it"], evals=f["evals"],
                                    best=f["best"])  # fmt: skip
            elif kind == "search_seed_done" and st["search"] is not None:
                st["search"]["seeds_done"] = f["seed_idx"]
                st["search"]["best"] = f["f"]
            elif kind == "search_done":
                st["search"] = None
            elif kind == "step":
                prev = st["step"]
                fresh = prev is None or f["total"] != prev["total"] or f["done"] < prev["done"]
                st["step"] = {"label": f["label"], "done": f["done"], "total": f["total"],
                              "started": now if fresh else prev["started"]}  # fmt: skip

    def traceback(self, lines: list[str]) -> None:
        self("traceback", {"lines": lines})

    def close(self, ok: bool) -> None:
        self._stop.set()
        with self._lock:
            self.state["status"] = "done" if ok else "failed"
            self.state["phase"] = "finished" if ok else "failed"
            self.state["last_event"] = self.clock()
            self._write_state()
            self._fh.close()


# ---- lines about one task, shared by the heartbeat and --status ---------------------------------
def progress_fraction(state: Mapping[str, Any]) -> tuple[float, float] | None:
    """(fraction done, time the measured stage started) from a state, or None if unknown."""
    s = state.get("search")
    if s:
        frac = ((s["seed"] - 1) + min(s["it"] / max(s["maxiter"], 1), 1.0)) / max(s["n_seeds"], 1)
        return frac, float(s["started"])
    st = state.get("step")
    if st and st["total"] > 0:
        return st["done"] / st["total"], float(st["started"])
    return None


def eta_minutes(state: Mapping[str, Any], now: float) -> float | None:
    """A rough time left from the observed pace of the current stage (None before 3 %)."""
    got = progress_fraction(state)
    if got is None:
        return None
    frac, t_ref = got
    if not 0.03 <= frac < 1.0:
        return None
    return (now - t_ref) * (1.0 - frac) / frac / 60.0


def task_line(state: Mapping[str, Any], now: float) -> str:
    """One line from a state: phase, seed and iteration or step, best, elapsed, ~time left."""
    bits = [f"{state['task']:<16}", str(state.get("phase", ""))[:46]]
    s, st = state.get("search"), state.get("step")
    if s:
        best = "" if s["best"] is None else f"  best {s['best']:.4g}"
        bits.append(f"seed {s['seed']}/{s['n_seeds']}  it {s['it']}/{s['maxiter']}{best}")
    elif st:
        bits.append(f"step {st['done']}/{st['total']}")
    bits.append(f"{minutes_text(now - float(state['started']))}")
    eta = eta_minutes(state, now)
    if eta is not None:
        bits.append(f"~{minutes_text(eta * 60)} left")
    quiet_for = now - float(state.get("last_event", now))
    if state.get("status", "running") == "running" and quiet_for >= STALE_S:
        bits.append(f"no update for {quiet_for / 60:.0f} min")
    return "  ".join(b for b in bits if b)


# ---- the parent side ---------------------------------------------------------------------------
class RunLog:
    """The terminal and file output of one study (thread-safe; `poll` runs in a monitor thread)."""

    def __init__(
        self, run_dir: Path, say: Callable[[str], None], *, level: str = "normal",
        heartbeat: float = 60.0, clock: Callable[[], float] = time.time,
    ) -> None:  # fmt: skip
        if level not in LEVELS:
            raise ValueError(f"log level {level!r} is not one of {LEVELS}")
        self.say, self.heartbeat, self.clock = say, heartbeat, clock
        self.level = LEVELS.index(level)
        self.logs = run_dir / LOGS
        self.logs.mkdir(parents=True, exist_ok=True)
        self.t_start = clock()
        stamp = time.strftime("%Y%m%d_%H%M%S", time.localtime(self.t_start))
        n = 1
        while True:  # two runs started in the same second must not share a file
            self.file_path = self.logs / f"run_{stamp}{'' if n == 1 else f'_{n}'}.log"
            try:
                self._file = self.file_path.open("x", encoding="utf-8", buffering=1)
                break
            except FileExistsError:
                n += 1
        self._lock = threading.RLock()
        self.active: dict[str, dict[str, Any]] = {}  # submitted and not finished
        self.finished: dict[str, tuple[str, float]] = {}  # task -> (ran | cached | failed, secs)
        self.waves: list[list[str]] = []
        self.wave = 0
        self.estimate_s: float | None = None
        self._last_beat = self._last_term_beat = self.t_start
        self._thread: threading.Thread | None = None
        self._stop = threading.Event()
        self.n_lines = 0  # terminal lines printed

    # output ---------------------------------------------------------------------------------
    def out(self, msg: str, level: str = "normal") -> None:
        """A line `HH:MM:SS  msg`: always in the run log, on the terminal if the level allows."""
        with self._lock:
            line = f"{clock_text(self.clock())}  {ascii_line(msg)}"
            self._file.write(line + "\n")
            if LEVELS.index(level) <= self.level:
                self.say(line)
                self.n_lines += 1

    def start(self, banner: list[str], estimate_s: float | None) -> None:
        self.estimate_s = estimate_s
        self.out(f"run log: {self.file_path}", "quiet")
        for b in banner:
            self.out(b, "quiet")

    def wave_start(self, n: int, tasks: list[str], workers: int) -> None:
        with self._lock:
            self.wave = n
            self.waves.append(list(tasks))
        self.out(f"wave {n} starts: {len(tasks)} task(s) on {workers} worker(s)")

    def wave_end(self, n: int, secs: float, failed: int) -> None:
        tail = f", {failed} failed" if failed else ""
        self.out(f"wave {n} ends after {minutes_text(secs)}{tail}")

    def cached(self, task: str) -> None:
        with self._lock:
            self.finished[task] = ("cached", 0.0)
        self.out(f"cached  {task}")

    def submit(self, tasks: list[str]) -> None:
        with self._lock:
            for t in tasks:
                self.active[t] = {"offset": 0, "started": False, "n_seeds": 0, "quartile": -1,
                                  "flagged": -1e18, "partial": b""}  # fmt: skip

    # monitoring thread ----------------------------------------------------------------------
    def monitor(self) -> None:
        if self._thread is None:
            self._thread = threading.Thread(target=self._loop, daemon=True)
            self._thread.start()

    def _loop(self) -> None:
        while not self._stop.wait(POLL_S):
            try:
                self.poll()
            except Exception as exc:  # noqa: BLE001  logging must never stop a study
                self.out(f"(log monitor: {type(exc).__name__}: {exc})", "verbose")

    def poll(self) -> None:
        """Look at the running tasks (start, new events, stale flags) and the heartbeat."""
        with self._lock:
            now = self.clock()
            for task, info in self.active.items():
                self._look(task, info, now)
            if now - self._last_beat >= self.heartbeat:
                self._beat(now)

    def _state(self, task: str) -> dict[str, Any] | None:
        return read_json(self.logs / f"{stem(task)}.state.json")

    def _look(self, task: str, info: dict[str, Any], now: float) -> None:
        state = self._state(task)
        if not info["started"]:
            # a state file of an earlier run (older than this run) is not this task starting
            if state is None or float(state.get("started", 0)) < self.t_start - 1:
                return
            info["started"] = True
            self.out(f"start   {task}")
        self._tail(task, info)
        if state and state.get("status") == "running":
            quiet = now - float(state.get("last_event", now))
            if quiet >= STALE_S and now - info["flagged"] >= STALE_AGAIN_S:
                info["flagged"] = now
                self.out(f"{task}  no update for {quiet / 60:.0f} min ({state.get('phase', '')})",
                         "quiet")  # fmt: skip

    def _tail(self, task: str, info: dict[str, Any]) -> None:
        path = self.logs / f"{stem(task)}.log"
        try:
            with path.open("rb") as fh:
                fh.seek(info["offset"])
                data = info["partial"] + fh.read()
        except OSError:
            return
        *lines, rest = data.split(b"\n")
        info["offset"] += len(data) - len(rest)
        info["partial"] = rest
        for raw in lines:
            parts = raw.decode("utf-8", "replace").split("\t", 2)
            if len(parts) < 3:
                continue
            try:
                f = json.loads(parts[2])
            except ValueError:
                continue
            self._event(task, info, parts[1], f)

    def _event(self, task: str, info: dict[str, Any], kind: str, f: dict[str, Any]) -> None:
        if kind == "search_start":
            info["n_seeds"] = f["n_seeds"]
        elif kind == "search_iter":
            self.out(f"{task}  seed {f['seed_idx']}/{info['n_seeds']}  it {f['it']}  "
                     f"{f['evals']} evals  best {f['best']:.5g}", "verbose")  # fmt: skip
        elif kind == "search_seed_done":
            self.out(f"{task}  seed {f['seed_idx']}/{info['n_seeds']} done  J={f['f']:.4f}  "
                     f"{f['evals']} evals  {minutes_text(f['secs'])}")  # fmt: skip
        elif kind == "search_done":
            self.out(f"{task}  search done  best J={f['best']:.4f}  seed spread "
                     f"{100 * f['spread']:.2f} %  {minutes_text(f['secs'])}")  # fmt: skip
        elif kind == "step":
            total, done = max(int(f["total"]), 1), int(f["done"])
            quartile = (4 * done) // total
            if quartile > info["quartile"] or done >= total:
                if quartile > info["quartile"]:
                    info["quartile"] = quartile
                self.out(f"{task}  {done}/{total}  {f['label']}")
            else:
                self.out(f"{task}  {done}/{total}  {f['label']}", "verbose")
        elif kind == "note":
            self.out(f"{task}  {f['text']}", "normal" if f.get("warn") else "verbose")
        elif kind == "phase":
            self.out(f"{task}  {f['label']}", "verbose")

    # heartbeat ------------------------------------------------------------------------------
    def counts(self) -> dict[str, int]:
        running = sum(1 for i in self.active.values() if i["started"])
        failed = sum(1 for s, _ in self.finished.values() if s == "failed")
        return {"done": len(self.finished) - failed, "failed": failed, "running": running,
                "waiting": len(self.active) - running}  # fmt: skip

    def status(self, now: float, state: str = "running") -> dict[str, Any]:
        running = []
        for task, info in self.active.items():
            st = self._state(task) if info["started"] else None
            if st is not None and st.get("status") == "running":
                running.append(st)
        return {
            "state": state, "started": self.t_start, "updated": now,
            "elapsed_s": now - self.t_start, "wave": self.wave, "heartbeat_s": self.heartbeat,
            "estimate_s": self.estimate_s, "log": str(self.file_path),
            "counts": self.counts(),
            "finished": {t: {"status": s, "secs": sec} for t, (s, sec) in self.finished.items()},
            "running": running,
        }  # fmt: skip

    def beat_lines(self, st: Mapping[str, Any], now: float, limit: int | None) -> list[str]:
        c = st["counts"]
        head = (f"heartbeat  {minutes_text(st['elapsed_s'])} elapsed  tasks: {c['done']} done, "
                f"{c['running']} running, {c['waiting']} waiting"
                + (f", {c['failed']} FAILED" if c["failed"] else "") + f"  (wave {st['wave']})")
        running = sorted(st["running"], key=lambda s: float(s["started"]))
        lines = [head] + ["  " + task_line(s, now) for s in running[:limit]]
        if limit is not None and len(running) > limit:
            lines.append(f"  ... and {len(running) - limit} more running (see --status)")
        return lines

    def _beat(self, now: float) -> None:
        self._last_beat = now
        st = self.status(now)
        write_json_atomic(self.logs / "status.json", st)
        for line in self.beat_lines(st, now, None):  # the run log gets everything
            self._file.write(f"{clock_text(now)}  {ascii_line(line)}\n")
        every = self.heartbeat if self.level == 2 else NORMAL_BEAT_S
        if self.level >= 1 and now - self._last_term_beat >= max(every, self.heartbeat) - 1e-9:
            self._last_term_beat = now
            for line in self.beat_lines(st, now, None if self.level == 2 else BLOCK_LINES):
                self.say(f"{clock_text(now)}  {ascii_line(line)}")
                self.n_lines += 1

    # a task ends ----------------------------------------------------------------------------
    def finish(self, task: str, ok: bool, secs: float, err: str = "") -> None:
        with self._lock:
            info = self.active.get(task)
            if info is not None:
                self._look(task, info, self.clock())
                self._tail(task, info)
            self.active.pop(task, None)
            self.finished[task] = ("ran" if ok else "failed", secs)
            if ok:
                self.out(f"done    {task}  {secs:.0f} s ({minutes_text(secs)})")
                return
            self.out(f"FAILED  {task}  after {secs:.0f} s: {err}", "quiet")
            for line in self.traceback_tail(task):
                self.out(f"          {line}", "quiet")
            self.out(f"          log: {self.logs / (stem(task) + '.log')}", "quiet")

    def traceback_tail(self, task: str, n: int = 6) -> list[str]:
        """The last lines of the traceback a worker wrote into its log (empty if none)."""
        try:
            text = (self.logs / f"{stem(task)}.log").read_text(encoding="utf-8", errors="replace")
        except OSError:
            return []
        for raw in reversed(text.splitlines()):
            parts = raw.split("\t", 2)
            if len(parts) == 3 and parts[1] == "traceback":
                try:
                    return [str(x) for x in json.loads(parts[2])["lines"]][-n:]
                except (ValueError, KeyError):
                    return []
        return []

    def broken_pool(self, n_lost: int) -> None:
        self.out("a worker process died (usually out of memory): the tasks still running or "
                 f"waiting were lost ({n_lost}); try fewer --workers", "quiet")

    def close(self, state: str = "finished") -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=5)
        with self._lock:
            now = self.clock()
            write_json_atomic(self.logs / "status.json", self.status(now, state))

    def summary(self, wall_s: float, failed: Mapping[str, str], view_cmd: str) -> None:
        """The table at the end: every task, the critical path, wall time against the estimate."""
        with self._lock:
            rows = sorted(self.finished.items(), key=lambda kv: -kv[1][1])
            ran = [(t, s, sec) for t, (s, sec) in rows if s != "cached"]
            cached = [t for t, (s, _) in rows if s == "cached"]
            self.out("summary", "quiet")
            self.out(f"  {'task':<22}{'result':<8}{'seconds':>9}", "quiet")
            for t, s, sec in ran:
                self.out(f"  {t:<22}{s:<8}{sec:>9.0f}", "quiet")
            if cached:
                self.out(f"  cached: {len(cached)} task(s): " + ", ".join(cached[:8])
                         + (" ..." if len(cached) > 8 else ""), "quiet")  # fmt: skip
            crit = 0.0
            parts = []
            for i, wave in enumerate(self.waves, start=1):
                best = max(((self.finished.get(t, ("", 0.0))[1], t) for t in wave), default=None)
                if best is not None and best[0] > 0:
                    crit += best[0]
                    parts.append(f"wave {i} {best[1]} {minutes_text(best[0])}")
            if ran:
                self.out(f"  slowest task: {ran[0][0]} ({minutes_text(ran[0][2])})", "quiet")
            if parts:
                self.out(f"  critical path ~{minutes_text(crit)}: " + "; ".join(parts), "quiet")
            est = (f", estimate ~{minutes_text(self.estimate_s)}" if self.estimate_s else "")
            self.out(f"  total wall time {minutes_text(wall_s)}{est}", "quiet")
            for t, e in failed.items():
                self.out(f"  FAILED {t}: {e}", "quiet")
                self.out(f"         log: {self.logs / (stem(t) + '.log')}", "quiet")
            self.out(f"  terminal lines of this run: {self.n_lines + 1}", "quiet")
            self.out(f"  full log: {self.file_path}", "quiet")
            self.out(f"next: {view_cmd}", "quiet")


# ---- --status ------------------------------------------------------------------------------
def read_status(folder: Path, now: float | None = None) -> list[str] | None:
    """What a running or the last study is doing, from its `_logs`; None if there is nothing.

    Reads only; never creates or changes a file.
    """
    now = time.time() if now is None else now
    logs = folder / LOGS if (folder / LOGS).is_dir() else folder
    st = read_json(logs / "status.json") if logs.is_dir() else None
    states = []
    if logs.is_dir():
        for p in sorted(logs.glob("*.state.json")):
            s = read_json(p)
            if s is not None:
                states.append(s)
    if st is None and not states:
        return None
    lines: list[str] = []
    if st is not None:
        age = now - float(st["updated"])
        if st["state"] == "running" and age > max(3 * float(st["heartbeat_s"]), 180.0):
            lines.append(f"the run looks stopped: its last update was {minutes_text(age)} ago")
        c = st["counts"]
        when = time.strftime("%Y-%m-%d %H:%M:%S", time.localtime(st["started"]))
        secs = now - st["started"] if st["state"] == "running" else st["elapsed_s"]
        lines.append(f"study {st['state']}  started {when}  elapsed {minutes_text(secs)}  "
                     f"wave {st['wave']}")
        lines.append(f"tasks: {c['done']} done, {c['running']} running, {c['waiting']} waiting, "
                     f"{c['failed']} failed" + (f"  (estimate ~{minutes_text(st['estimate_s'])})"
                                                 if st.get("estimate_s") else ""))  # fmt: skip
        lines.append(f"run log: {st['log']}")
        bad = [t for t, v in st["finished"].items() if v["status"] == "failed"]
        for t in bad:
            lines.append(f"FAILED {t}: see {logs / (stem(t) + '.log')}")
    live = [s for s in states if s.get("status") == "running"
            and (st is None or st["state"] == "running")]  # fmt: skip
    for s in sorted(live, key=lambda s: float(s["started"])):
        lines.append("  " + task_line(s, now))
    if not live and st is None:
        lines.append("no task is running; the last states:")
        lines += ["  " + task_line(s, now) for s in states]
    return [ascii_line(x) for x in lines]
