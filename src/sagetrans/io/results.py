"""Writing outputs to the results folder, guarded by the DRAFT rule (T-14) and the run record."""

from __future__ import annotations

import csv
import dataclasses
from collections.abc import Mapping, Sequence
from pathlib import Path
from typing import Any

from matplotlib.figure import Figure

from sagetrans.config.schema import Config, DraftConfigError
from sagetrans.io.figures import save_figure
from sagetrans.io.record import FullRunRecord, sha256_file


def columns_of(rows: Sequence[Mapping[str, Any]]) -> list[str]:
    """The columns of a table in first-seen order."""
    cols: list[str] = []
    for row in rows:
        cols += [c for c in row if c not in cols]
    return cols


def write_csv(path: Path, rows: Sequence[Mapping[str, Any]]) -> None:
    """Machine-readable table; columns in first-seen order, newline-stable."""
    cols = columns_of(rows)
    with path.open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def require_writable(record: FullRunRecord, config: Config, override: bool) -> None:
    """Refuse DRAFT output to the report folder without an explicit override (T-14)."""
    if record.label == "DRAFT" and not override:
        why = "; ".join(record.reasons) or "DRAFT"
        raise DraftConfigError(f"output is DRAFT ({why}); pass override=True to write anyway")


def write_results(
    root: Path,
    name: str,
    record: FullRunRecord,
    config: Config,
    *,
    tables: Mapping[str, Sequence[Mapping[str, Any]]] | None = None,
    figures: Mapping[str, Figure] | None = None,
    texts: Mapping[str, str] | None = None,
    override: bool = False,
) -> Path:
    """Write tables, figures and text to `root/name/` with a `run_record.json` listing every file.

    Every file is hashed into the record, so `check_outputs` can later show that nothing was
    edited by hand. Returns the folder.
    """
    require_writable(record, config, override)
    folder = root / name
    folder.mkdir(parents=True, exist_ok=True)
    written: list[Path] = []
    shapes: dict[str, dict[str, Any]] = {}
    for stem, rows in (tables or {}).items():
        p = folder / f"{stem}.csv"
        write_csv(p, rows)
        written.append(p)
        shapes[stem] = {"rows": len(rows), "columns": columns_of(rows)}
    for stem, fig in (figures or {}).items():
        written += save_figure(fig, folder, stem, record.footer())
    for fname, text in (texts or {}).items():
        p = folder / fname
        p.write_text(text, encoding="utf-8", newline="\n")
        written.append(p)
    final = dataclasses.replace(
        record, override=override, outputs={p.name: sha256_file(p) for p in written},
        tables=shapes,
    )
    (folder / "run_record.json").write_text(final.to_json(), encoding="utf-8", newline="\n")
    return folder
