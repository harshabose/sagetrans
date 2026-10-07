"""The run record written next to every output (FR-09, NFR-07, section 11)."""

from __future__ import annotations

import datetime as dt
import hashlib
import json
import subprocess
from collections.abc import Mapping
from dataclasses import asdict, dataclass, field
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
    override: bool = False  # the DRAFT refusal was explicitly overridden
    outputs: dict[str, str] = field(default_factory=dict)  # file name -> sha256

    def to_json(self) -> str:
        return json.dumps(asdict(self), indent=2, sort_keys=True)

    def footer(self) -> str:
        """The one-line footer put on figures."""
        return (
            f"run record {self.config_hash[:8]} | {self.label} | commit {self.git_commit}"
            f"{'+dirty' if self.git_dirty else ''} | {self.date}"
        )


def build_record(
    config: Config,
    analyses: Mapping[str, RunRecord],
    seeds: tuple[int, ...] = (),
    override: bool = False,
) -> FullRunRecord:
    """Combine the configuration state with the records of the analyses that fed an output.

    The record is DRAFT if the configuration has any TBD parameter, or if any contributing
    analysis ran on placeholder physics (its own record is DRAFT).
    """
    reasons: list[str] = []
    if config.is_draft:
        tbd = sorted(p for p in config if config.record(p).status.value == "TBD")
        reasons.append("configuration has TBD parameters: " + ", ".join(tbd))
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
        override=override,
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
