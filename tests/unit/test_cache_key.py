"""The task cache is keyed by the inputs and by the source that computes a task."""

import json
import pickle
import shutil
from pathlib import Path

import sagetrans
from sagetrans.io import record, study
from sagetrans.io.inputs import load_inputs

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
SMOKE = """
run_name: smoke
mode: draft
analyses: [F]
F: {scenarios: [S-01]}
"""


def make_pkg(base: Path) -> Path:
    """A tiny package tree: two files that decide a task and two that only write output."""
    pkg = base / "pkg"
    for rel, text in {
        "analysis/a.py": "X = 1\n", "io/stages.py": "Y = 1\n", "io/assemble.py": "Z = 1\n",
        "io/view/page.py": "V = 1\n",
    }.items():  # fmt: skip
        f = pkg / rel
        f.parent.mkdir(parents=True, exist_ok=True)
        f.write_text(text, encoding="utf-8")
    return pkg


def test_the_source_hash_follows_the_code_that_decides_a_task(tmp_path):
    pkg = make_pkg(tmp_path)
    base = record.source_hash(pkg)
    record.source_hash.cache_clear()
    (pkg / "analysis" / "a.py").write_text("X = 2\n", encoding="utf-8")
    assert record.source_hash(pkg) != base  # an analysis changed: old results are stale
    record.source_hash.cache_clear()
    (pkg / "analysis" / "a.py").write_text("X = 1\n", encoding="utf-8")
    assert record.source_hash(pkg) == base  # restored: the same key again
    record.source_hash.cache_clear()
    (pkg / "io" / "stages.py").write_text("Y = 2\n", encoding="utf-8")
    assert record.source_hash(pkg) != base  # the task functions themselves count
    record.source_hash.cache_clear()
    (pkg / "io" / "stages.py").write_text("Y = 1\n", encoding="utf-8")
    # the writer and the viewer do not: a change to them must not throw away hours of results
    (pkg / "io" / "assemble.py").write_text("Z = 2\n", encoding="utf-8")
    (pkg / "io" / "view" / "page.py").write_text("V = 2\n", encoding="utf-8")
    record.source_hash.cache_clear()
    assert record.source_hash(pkg) == base
    # line endings are not a change
    (pkg / "analysis" / "a.py").write_bytes(b"X = 1\r\n")
    record.source_hash.cache_clear()
    assert record.source_hash(pkg) == base
    record.source_hash.cache_clear()


def test_the_cache_path_carries_the_inputs_the_version_and_the_source(tmp_path, monkeypatch):
    inp = load_inputs(CONFIGS)
    work = tmp_path / "w"
    a = study._cache_path(work, inp, "basics")
    assert inp.inputs_hash[:16] in a.name and sagetrans.__version__ in a.name
    assert record.source_hash()[:12] in a.name
    monkeypatch.setattr(record, "source_hash", lambda root=None: "f" * 64)
    b = study._cache_path(work, inp, "basics")
    assert b != a and "ffffffffffff" in b.name  # other code, other entry


def test_a_code_change_or_a_register_change_invalidates_and_an_old_entry_is_ignored(
    tmp_path, monkeypatch
):
    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    (cfg / "study.yaml").write_text(SMOKE, encoding="utf-8")
    run = lambda: study.run_study(  # noqa: E731
        cfg, out_dir=tmp_path / "res", workers=1, log=lambda s: None
    )
    out = run()
    assert out.tasks == {"basics": "ran"}
    assert run().tasks == {"basics": "cached"}
    # a pickle left by the version before the source hash: same name minus the third field, and a
    # sentinel inside. It is never opened, so the task runs and the sentinel is not served.
    inp = load_inputs(cfg)
    work = out.folder / "_work"
    new = study._cache_path(work, inp, "basics")
    old = new.with_name(new.name.rsplit(".", 2)[0] + ".pkl")
    assert old != new
    old.write_bytes(pickle.dumps({"tables": {"F_descent": [{"sentinel": 1}]}, "errors": []}))
    new.unlink()
    out2 = run()
    assert out2.tasks == {"basics": "ran"}
    text = (out2.folder / "F_descent.csv").read_text(encoding="utf-8")
    assert "sentinel" not in text and "descent_time_s" in text
    # changed source: the cached entry is not used
    monkeypatch.setattr(record, "source_hash", lambda root=None: "a" * 64)
    assert run().tasks == {"basics": "ran"}
    assert run().tasks == {"basics": "cached"}
    monkeypatch.undo()
    # a register value: not used either (the inputs hash)
    veh = cfg / "vehicle.yaml"
    veh.write_text(
        veh.read_text(encoding="utf-8").replace("value: 0.6,", "value: 0.62,", 1), encoding="utf-8"
    )
    assert run().tasks == {"basics": "ran"}


def test_the_run_record_carries_the_source_hash(tmp_path):
    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    (cfg / "study.yaml").write_text(SMOKE, encoding="utf-8")
    out = study.run_study(cfg, out_dir=tmp_path / "res", workers=1, log=lambda s: None)
    rec = json.loads((out.folder / "run_record.json").read_text(encoding="utf-8"))
    assert rec["source_hash"] == record.source_hash() and len(rec["source_hash"]) == 64
