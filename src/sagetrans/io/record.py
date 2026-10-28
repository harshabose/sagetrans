"""The run record written next to every output (FR-09, NFR-07, section 11)."""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import platform
import subprocess
from collections.abc import Mapping
from dataclasses import asdict, dataclass, field
from functools import lru_cache
from importlib import metadata
from pathlib import Path
from typing import Any

import sagetrans
from sagetrans.analysis.common import RunRecord
from sagetrans.config.schema import Config

REPO_ROOT = Path(__file__).resolve().parents[3]


def git_state(root: Path = REPO_ROOT) -> tuple[str, bool]:
    """(short commit hash or 'no-commit', working tree dirty?)."""
    try:
        head = subprocess.run(
            ["git", "rev-parse", "--short", "HEAD"], cwd=root, capture_output=True, text=True,
            check=True, timeout=10,
        ).stdout.strip()  # fmt: skip
        dirty = bool(
            subprocess.run(
                ["git", "status", "--porcelain"], cwd=root, capture_output=True, text=True,
                check=True, timeout=10,
            ).stdout.strip()  # fmt: skip
        )
        return head, dirty
    except (subprocess.SubprocessError, OSError):
        return "no-commit", True


@dataclass(frozen=True)
class FullRunRecord:
    """Everything needed to reproduce an output: configuration hash plus commit, and the grade."""

    label: str  # DRAFT unless the configuration and every analysis input are release grade
    reasons: tuple[str, ...]  # why it is DRAFT, empty when RELEASED
    config_hash: str
    evidence_counts: dict[str, int]
    code_version: str
    git_commit: str
    git_dirty: bool
    date: str
    analyses: dict[str, str]  # analysis name -> input hash of its own record
    seeds: tuple[int, ...] = ()
    # three-seed cost spread of each analysis that used a stochastic search (today only C): the
    # real-problem T-12 evidence, which the release gate checks (the unit test of T-12 only
    # exercises the search machinery on an analytic cost)
    search_spreads: dict[str, float] = field(default_factory=dict)
    # how the run was configured: draft or release, the hash of the plain settings files
    # (scenarios.yaml and the result-relevant part of study.yaml), and the register paths that fell
    # back to a built-in placeholder. With config_hash these make the run reproducible.
    mode: str = ""
    settings_hash: str = ""
    fallbacks: tuple[str, ...] = ()
    override: bool = False  # the DRAFT refusal was explicitly overridden
    outputs: dict[str, str] = field(default_factory=dict)  # file name -> sha256
    # schema 2 (docs/OUTPUTS_spec.md 5.5): what the results viewer needs that is not in a table.
    # A record without `schema_version` was written by an older version and still loads.
    schema_version: int = 2
    tables: dict[str, dict[str, Any]] = field(default_factory=dict)  # name -> {rows, columns}
    # {"limitations": [...], "notes": {analysis: [...]}, "study": {analysis: settings}}
    context: dict[str, Any] = field(default_factory=dict)
    # hash of the source files that decide what a task computes (see `source_hash`); also part of
    # the task-cache key, so a code change cannot serve an old pickle
    source_hash: str = ""
    # Python and the numerical libraries that can move an optimum without a code change; also
    # part of the task-cache key (see `library_versions`)
    library_versions: dict[str, str] = field(default_factory=dict)

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True)

    def footer(self) -> str:
        """The one-line footer put on figures."""
        return (
            f"run record {self.config_hash[:8]} | {self.label} | commit {self.git_commit}"
            f"{'+dirty' if self.git_dirty else ''} | {self.date}"
        )


# Modules that only turn results into files or text. They cannot change what a task computes, so a
# change to them must not throw away a cache that took hours to build.
OUTPUT_ONLY = (
    "io/view/", "io/assemble.py", "io/cli.py", "io/study.py", "io/figures.py", "io/report.py",
    "io/gate.py", "io/recommended.py", "io/results.py", "io/record.py", "io/ci_check.py",
    "__main__.py",
)  # fmt: skip


@lru_cache(maxsize=4)
def source_hash(root: Path | None = None) -> str:
    """sha256 over the relative path and bytes of every package `.py` file that can change a task.

    `root` is the package folder (default: `sagetrans`). The register and the plain settings files
    are already in `StudyInputs.inputs_hash`; this is the code side of the task-cache key.
    """
    base = root or Path(__file__).resolve().parents[1]
    h = hashlib.sha256()
    for p in sorted(base.rglob("*.py")):
        rel = p.relative_to(base).as_posix()
        if "__pycache__" in rel or rel.startswith(OUTPUT_ONLY) or rel in OUTPUT_ONLY:
            continue
        h.update(rel.encode() + b"\0")
        data = p.read_bytes().replace(b"\r\n", b"\n")  # line endings do not count
        h.update(data + b"\0")
    return h.hexdigest()


# the libraries whose versions can change a result: pinned in pyproject.toml, recorded here and in
# the cache key
LIBRARIES = ("numpy", "scipy", "casadi", "cma", "SALib", "pydantic", "matplotlib")


@lru_cache(maxsize=1)
def library_versions() -> dict[str, str]:
    """Python and the versions of `LIBRARIES` as installed ("not installed" is recorded too)."""
    out = {"python": platform.python_version()}
    for name in LIBRARIES:
        try:
            out[name] = metadata.version(name)
        except metadata.PackageNotFoundError:
            out[name] = "not installed"
    return out


def library_hash() -> str:
    """sha256 of `library_versions`: the library part of the task-cache key."""
    blob = json.dumps(library_versions(), sort_keys=True)
    return hashlib.sha256(blob.encode()).hexdigest()


def build_record(
    config: Config,
    analyses: Mapping[str, RunRecord],
    seeds: tuple[int, ...] = (),
    override: bool = False,
    search_spreads: Mapping[str, float] | None = None,
    *,
    mode: str = "",
    settings_hash: str = "",
    fallbacks: tuple[str, ...] = (),
    extra_reasons: tuple[str, ...] = (),
) -> FullRunRecord:
    """Combine the configuration state with the records of the analyses that fed an output.

    The record is DRAFT if the configuration has any TBD parameter, or if any contributing
    analysis ran on placeholder physics (its own record is DRAFT).

    `search_spreads` maps an analysis that used a stochastic search (today only "C") to the
    three-seed cost spread of that search (`SearchResult.spread`).
    """
    reasons: list[str] = list(extra_reasons)
    if config.is_draft and not extra_reasons:
        tbd = sorted(p for p in config if config.record(p).status.value == "TBD")
        shown = ", ".join(tbd[:8]) + (f", ... ({len(tbd)} in all)" if len(tbd) > 8 else "")
        reasons.append("configuration has TBD parameters: " + shown)
    missing = config.missing_files()
    if missing and not extra_reasons:
        reasons.append("register references files that do not exist: " + ", ".join(missing))
    for name, rec in analyses.items():
        if rec.label == "DRAFT":
            reasons.append(f"analysis '{name}' ran on placeholder inputs (record is DRAFT)")
    commit, dirty = git_state()
    return FullRunRecord(
        label="DRAFT" if reasons else "RELEASED",
        reasons=tuple(reasons),
        config_hash=config.config_hash(),
        evidence_counts=config.evidence_counts(),
        code_version=sagetrans.__version__,
        git_commit=commit,
        git_dirty=dirty,
        date=dt.date.today().isoformat(),
        analyses={k: v.input_hash for k, v in analyses.items()},
        seeds=seeds,
        search_spreads=dict(search_spreads or {}),
        mode=mode,
        settings_hash=settings_hash,
        fallbacks=fallbacks,
        override=override,
        source_hash=source_hash(), library_versions=dict(library_versions()),
    )


def sha256_file(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def check_outputs(folder: Path) -> dict[str, bool]:
    """Re-hash every file named in a folder's run record: True where it still matches."""
    data: dict[str, Any] = json.loads((folder / "run_record.json").read_text(encoding="utf-8"))
    return {
        name: (folder / name).is_file() and sha256_file(folder / name) == digest
        for name, digest in data["outputs"].items()
    }
