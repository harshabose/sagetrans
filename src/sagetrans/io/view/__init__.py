"""The results viewer: `python run.py --view [folder]` (docs/OUTPUTS_spec.md).

A viewer on a results folder that already exists. It never runs an analysis and never changes a
result: it writes `report.html` and `tables.xlsx` next to the tables it reads, and opens the page.
With no results in the folder it writes and opens nothing.
"""

from __future__ import annotations

import sys
import webbrowser
from pathlib import Path

import yaml

from sagetrans.io.view.html import page  # noqa: F401  (re-exported for callers building a page)
from sagetrans.io.view.load import Results, has_results, load
from sagetrans.io.view.registry import build_html
from sagetrans.io.view.workbook import write_workbook

EXIT_OK = 0
EXIT_NO_RESULTS = 2


def default_folder(config: Path, out: str | None = None) -> Path:
    """`output_dir/run_name` of `study.yaml`, read as plain YAML (a broken inputs file is not an error)."""
    study = config / "study.yaml"
    data = yaml.safe_load(study.read_text(encoding="utf-8")) if study.is_file() else {}
    data = data if isinstance(data, dict) else {}
    return Path(out or data.get("output_dir", "results")) / str(data.get("run_name", "study"))


def find_results(folder: Path) -> Path | None:
    """The folder itself, or the only sub-folder of it, that holds results; None if there are none."""
    if has_results(folder):
        return folder
    if folder.is_dir():
        kids = [p for p in sorted(folder.iterdir()) if p.is_dir() and has_results(p)]
        if len(kids) == 1:
            return kids[0]
    return None


def render(res: Results) -> tuple[str, int]:
    """The page text of a loaded folder and its size in bytes."""
    text = build_html(res)
    return text, len(text.encode("utf-8"))


def view(folder: Path, open_browser: bool = True) -> int:
    """Write `report.html` and `tables.xlsx` into the results folder and open the page.

    Returns 0, or 2 (writing and opening nothing) when the folder has no results.
    """
    where = find_results(folder)
    if where is None:
        print(f"no results in {folder}; run the study first", file=sys.stderr)
        return EXIT_NO_RESULTS
    res = load(where)
    text, size = render(res)
    html_path = where / "report.html"
    html_path.write_text(text, encoding="utf-8", newline="\n")
    write_workbook(res, where / "tables.xlsx")
    print(f"report:   {html_path}  ({size / 1e6:.2f} MB)")
    print(f"workbook: {where / 'tables.xlsx'}")
    if open_browser:
        webbrowser.open(html_path.resolve().as_uri())
    return EXIT_OK
