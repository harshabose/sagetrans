"""Read a results folder: the run record, every CSV table, the verbatim text files.

The viewer reads only this folder (OUTPUTS spec 2). Each column is typed from its cells (float,
bool or text; an empty cell is None) and keeps its original header: the header is the column's
identity. File hashes are checked against `run_record.json`; a mismatch is a warning, not an
error. Older folders (no `schema_version`) load too, with the old table names resolved through
`ALIASES`.
"""

from __future__ import annotations

import csv
import hashlib
import json
from dataclasses import dataclass, field
from pathlib import Path
from typing import Any

# a table the registry asks for by its current name <- the legacy names that can stand in for it
ALIASES: dict[str, tuple[str, ...]] = {"C_runs": ("C_ensemble_runs",)}
LEGACY_PREFIXES: dict[str, str] = {"B1_front": "B1_pareto_"}  # B1_pareto_<scenario> files
# a folder counts as results if it has a run record or a table with one of these prefixes
KNOWN_PREFIXES = ("A_", "Q2_", "D_", "F_", "E_", "FT_", "C_", "B1_", "sens_", "settings_")


@dataclass
class Table:
    name: str
    source: str  # the CSV file name (or several, for a merged legacy table)
    columns: list[str]
    rows: list[dict[str, Any]]

    def col(self, name: str) -> list[Any]:
        return [r.get(name) for r in self.rows]

    def has(self, *names: str) -> bool:
        return all(n in self.columns for n in names)

    def where(self, **eq: Any) -> list[dict[str, Any]]:
        return [r for r in self.rows if all(r.get(k) == v for k, v in eq.items())]


@dataclass
class FileInfo:
    name: str
    size: int
    sha256: str
    status: str  # "verified" | "edited" | "not in the record"


@dataclass
class Results:
    folder: Path
    record: dict[str, Any]
    tables: dict[str, Table]
    gate_text: str = ""
    inputs_text: str = ""
    files: list[FileInfo] = field(default_factory=list)
    edited: set[str] = field(default_factory=set)  # file names whose hash no longer matches
    legacy: bool = False  # written before `schema_version` 2
    aliased: dict[str, str] = field(default_factory=dict)  # current name -> the legacy source

    def get(self, name: str) -> Table | None:
        return self.tables.get(name)

    def rows(self, name: str) -> list[dict[str, Any]]:
        t = self.tables.get(name)
        return t.rows if t else []

    @property
    def label(self) -> str:
        return str(self.record.get("label", "UNKNOWN"))


def _typed(cells: list[str]) -> list[Any]:
    """Convert one column: float if every non-empty cell parses, else bool, else text."""
    filled = [c for c in cells if c != ""]
    if filled and all(c in ("True", "False") for c in filled):
        return [None if c == "" else c == "True" for c in cells]
    try:
        for c in filled:
            float(c)
    except ValueError:
        return [None if c == "" else c for c in cells]
    return [None if c == "" else float(c) for c in cells]


def read_csv(path: Path) -> Table:
    """Parse one CSV file into a typed table."""
    with path.open(encoding="utf-8", newline="") as fh:
        reader = csv.reader(fh)
        header = next(reader, [])
        raw = [r for r in reader if r]
    cols = [[r[i] if i < len(r) else "" for r in raw] for i in range(len(header))]
    typed = [_typed(c) for c in cols]
    rows = [{h: typed[i][k] for i, h in enumerate(header)} for k in range(len(raw))]
    return Table(path.stem, path.name, header, rows)


def _sha(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def has_results(folder: Path) -> bool:
    """True if the folder holds a run record or at least one recognised table."""
    if not folder.is_dir():
        return False
    if (folder / "run_record.json").is_file():
        return True
    return any(p.name.startswith(KNOWN_PREFIXES) for p in folder.glob("*.csv"))


def load(folder: str | Path) -> Results:
    """Load a results folder. The caller has checked `has_results`."""
    root = Path(folder)
    rec_path = root / "run_record.json"
    record: dict[str, Any] = {}
    if rec_path.is_file():
        record = json.loads(rec_path.read_text(encoding="utf-8"))
    tables: dict[str, Table] = {}
    for p in sorted(root.glob("*.csv")):
        tables[p.stem] = read_csv(p)
    res = Results(root, record, tables, legacy=int(record.get("schema_version", 1)) < 2)
    for new, olds in ALIASES.items():
        for old in olds:
            if new not in tables and old in tables:
                t = tables[old]
                tables[new] = Table(new, t.source, t.columns, t.rows)
                res.aliased[new] = old
    for new, prefix in LEGACY_PREFIXES.items():
        parts = [t for n, t in sorted(tables.items()) if n.startswith(prefix)]
        if new not in tables and parts:
            cols: list[str] = ["scenario"]
            rows: list[dict[str, Any]] = []
            for t in parts:
                scen = t.name[len(prefix) :]
                rows += [{"scenario": scen, **r} for r in t.rows]
                cols += [c for c in t.columns if c not in cols]
            tables[new] = Table(new, ", ".join(t.source for t in parts), cols, rows)
            res.aliased[new] = ", ".join(t.name for t in parts)
    outputs = record.get("outputs", {})
    for p in sorted(root.iterdir()):
        if not p.is_file():
            continue
        digest = _sha(p)
        if p.name in outputs:
            status = "verified" if outputs[p.name] == digest else "edited"
            if status == "edited":
                res.edited.add(p.name)
        else:
            status = "not in the record"
        res.files.append(FileInfo(p.name, p.stat().st_size, digest, status))
    for attr, name in (("gate_text", "release_gate.txt"), ("inputs_text", "inputs_used.md")):
        f = root / name
        if f.is_file():
            setattr(res, attr, f.read_text(encoding="utf-8"))
    return res
