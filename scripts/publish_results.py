#!/usr/bin/env python3
"""Publish a results folder as a GitHub Pages branch: `results/draft-<commit>`.

    python scripts/publish_results.py [RESULTS_FOLDER] [--push] [--dry-run]

The branch is an orphan branch (no shared history) with one commit holding everything the page
links to, at the root: `index.html` and `report.html` (byte-identical copies of the viewer's
report, so every sha256 in the page stays true and the page's own link to `report.html` works),
every file the run record lists (the tables as CSV, the tornado figures, `report.md`, ...),
`run_record.json`, `tables.xlsx`, a `.nojekyll` and a `README.md`. Before it commits it checks
that every relative link in the page points at a file on the branch, so a page that would 404 is
never published. Nothing is pushed unless `--push` is given; GitHub Pages is switched on by hand
(Settings, Pages, branch, root).

Standard library only. The branch is built with git plumbing and a temporary index, so the working
tree and the checked-out branch are never touched.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
import re
import shutil
import subprocess
import sys
import tempfile
from collections.abc import Sequence
from html.parser import HTMLParser
from pathlib import Path

PAGE = "report.html"
INDEX = "index.html"
VIEWER_FILES = (PAGE, "tables.xlsx")  # written by the viewer, not by the study (not in the record)
RECORD = "run_record.json"
COMMIT_RE = re.compile(r"^[0-9a-f]{7,40}$")
EXTERNAL = re.compile(r"^(?:[a-z][a-z0-9+.-]*:|//)", re.IGNORECASE)  # http:, data:, mailto:, //host


class PublishError(Exception):
    """Something that must stop the publication; the message says what to do."""


def git(repo: Path, *args: str, env: dict[str, str] | None = None, check: bool = True) -> str:
    done = subprocess.run(
        ["git", "-C", str(repo), *args], capture_output=True, text=True,
        env={**os.environ, **(env or {})}, check=False,
    )
    if check and done.returncode != 0:
        why = done.stderr.strip() or done.stdout.strip()
        raise PublishError(f"git {' '.join(args)} failed: {why}")
    return done.stdout.strip()


def sha256(path: Path) -> str:
    h = hashlib.sha256()
    with path.open("rb") as fh:
        for chunk in iter(lambda: fh.read(1 << 20), b""):
            h.update(chunk)
    return h.hexdigest()


# -------------------------------------------------------------------------------------- the folder
def default_folder(repo: Path, config: Path) -> Path:
    """`<output_dir>/<run_name>` from study.yaml (read by line: no YAML library needed)."""
    text = (config / "study.yaml").read_text(encoding="utf-8")

    def key(name: str, default: str) -> str:
        m = re.search(rf"^{name}:\s*([^\s#]+)", text, re.MULTILINE)
        return m.group(1).strip("'\"") if m else default

    return repo / key("output_dir", "results") / key("run_name", "study")


def load_record(folder: Path) -> dict:
    path = folder / RECORD
    if not path.is_file():
        raise PublishError(f"{path} not found: run `python run.py` first")
    try:
        rec = json.loads(path.read_text(encoding="utf-8"))
    except json.JSONDecodeError as exc:
        raise PublishError(f"{path} is not valid JSON ({exc})") from exc
    if not isinstance(rec.get("outputs"), dict):
        raise PublishError(f"{path} has no `outputs` table")
    return rec


def branch_name(record: dict, allow_dirty: bool = False) -> str:
    """`results/draft-<commit>` (or `release-`), the commit the run was made at."""
    commit = str(record.get("git_commit", ""))
    if not COMMIT_RE.match(commit):
        raise PublishError(f"the run record has no usable git commit ({commit!r}): the run was "
                           "not made from a git checkout, so the branch cannot be named")
    if record.get("git_dirty") and not allow_dirty:
        raise PublishError(f"the run was made on a dirty tree at {commit}: the report cannot be "
                           "reproduced from that commit. Commit, re-run, or pass --allow-dirty")
    label = str(record.get("label", "DRAFT")).lower()
    if label not in ("draft", "release"):
        raise PublishError(f"the record's label is {record.get('label')!r}, not DRAFT or RELEASE")
    return f"results/{label}-{commit}"


def collect(folder: Path, record: dict) -> list[str]:
    """What goes on the branch: the files the record lists, the record, the viewer's two files.

    Every listed file must exist and match its sha256; a missing or edited file stops the
    publication (the page would show a table the run did not write).
    """
    if not (folder / PAGE).is_file():
        raise PublishError(f"{folder / PAGE} not found: run `python run.py --view {folder}`")
    problems = []
    for name, digest in record["outputs"].items():
        path = folder / name
        if Path(name).name != name:
            problems.append(f"{name}: not a file at the top of the folder")
        elif not path.is_file():
            problems.append(f"{name}: missing")
        elif sha256(path) != digest:
            problems.append(f"{name}: edited after the run (sha256 differs from the record)")
    if problems:
        raise PublishError("the folder does not match its run record:\n  " + "\n  ".join(problems))
    names = [*record["outputs"], RECORD]
    names += [n for n in VIEWER_FILES if (folder / n).is_file() and n not in names]
    return names


# ------------------------------------------------------------------------------------ link check
class _Links(HTMLParser):
    def __init__(self) -> None:
        super().__init__()
        self.urls: list[str] = []

    def handle_starttag(self, tag: str, attrs: list[tuple[str, str | None]]) -> None:
        for key, value in attrs:
            if value and key in ("href", "src", "poster", "data") and tag in (
                    "a", "link", "img", "script", "source", "video", "object", "iframe"):
                self.urls.append(value)


def relative_links(html: str) -> list[str]:
    """The files a page links to by relative path (no `#fragment`, no query, no external URL)."""
    parser = _Links()
    parser.feed(html)
    out = []
    for url in parser.urls:
        if url.startswith("#") or EXTERNAL.match(url):
            continue
        path = re.split(r"[?#]", url, maxsplit=1)[0]
        if path:
            out.append(path)
    return sorted(set(out))


def missing_links(html: str, published: Sequence[str]) -> list[str]:
    have = set(published)
    return [u for u in relative_links(html) if u.removeprefix("./") not in have]


# ------------------------------------------------------------------------------------------ README
def readme(record: dict, branch: str, page_sha: str, page_bytes: int, n_files: int) -> str:
    commit = str(record.get("git_commit"))
    label = str(record.get("label", "DRAFT"))
    draft = label.upper() == "DRAFT"
    reasons = record.get("reasons") or []
    py = (record.get("library_versions") or {}).get("python", "unknown")
    dirty = " (the tree was dirty)" if record.get("git_dirty") else " (clean tree)"
    head = f"# sagetrans: {label.lower()} results report ({commit})\n\n"
    if draft:
        head += (
            "**This is a draft.** `index.html` is the results viewer of one study run on assumed "
            "placeholder inputs."
            + (f" {reasons[0].split(':')[0]}." if reasons else "")
            + " None of the numbers are release-grade, and they will change when the register is "
            "filled. Do not quote them as results.\n\n")
    return head + (
        "| | |\n|---|---|\n"
        f"| page | `index.html` (and an identical `report.html`, so the links inside the page "
        f"resolve), {page_bytes / 1e6:.1f} MB |\n"
        f"| files | {n_files}: the page, the tables as CSV, `tables.xlsx`, the tornado figures, "
        "`report.md`, `run_record.json` and the rest of what the page links to |\n"
        f"| sha256 of the page | `{page_sha}` |\n"
        f"| produced by | `python run.py` then `python run.py --view` on commit `{commit}`"
        f"{dirty}, {record.get('date', 'unknown date')} |\n"
        f"| run record | label {label}, mode {record.get('mode', 'unknown')}, source hash "
        f"`{str(record.get('source_hash', ''))[:12]}`, Python {py} |\n\n"
        "How to look at it: GitHub Pages must serve this branch from its root (Settings, Pages, "
        "this branch, `/`); that is a repository setting this branch does not switch on. Without "
        "Pages, clone the branch and open `index.html` in a browser; the links to the tables work "
        "from the folder.\n\n"
        f"How to reproduce it: check out `{commit}`, `pip install -e \".[dev]\"`, "
        "`python run.py --fresh-cache`, `python run.py --view`.\n\n"
        f"Published by `scripts/publish_results.py` as `{branch}`. "
        "This branch holds only the report and shares no history with `master`.\n")


# -------------------------------------------------------------------------------------- the branch
def stage(folder: Path, names: Sequence[str], record: dict, branch: str, dest: Path) -> list[str]:
    """Copy the files into `dest`, add index.html, .nojekyll and README.md; return all names."""
    for name in names:
        shutil.copyfile(folder / name, dest / name)
    shutil.copyfile(folder / PAGE, dest / INDEX)
    (dest / ".nojekyll").write_text("", encoding="utf-8")
    page = dest / INDEX
    files = sorted({*names, INDEX, ".nojekyll", "README.md"})
    (dest / "README.md").write_text(
        readme(record, branch, sha256(page), page.stat().st_size, len(files)), encoding="utf-8")
    return files


def branch_exists(repo: Path, branch: str, remote: str | None) -> str:
    if git(repo, "rev-parse", "--verify", "--quiet", f"refs/heads/{branch}", check=False):
        return f"the branch {branch} already exists in this clone (delete it or publish another)"
    if remote and git(repo, "ls-remote", "--heads", remote, branch, check=False):
        return f"the branch {branch} already exists on {remote}"
    return ""


def commit_orphan(repo: Path, staging: Path, branch: str, message: str) -> str:
    """One parentless commit of `staging` as refs/heads/<branch>, via a temporary index."""
    with tempfile.TemporaryDirectory() as tmp:
        env = {"GIT_INDEX_FILE": str(Path(tmp) / "index"), "GIT_WORK_TREE": str(staging)}
        git(repo, "add", "--force", "--all", env=env)
        tree = git(repo, "write-tree", env=env)
    commit = git(repo, "commit-tree", tree, "-m", message)
    git(repo, "update-ref", f"refs/heads/{branch}", commit)
    return commit


def pages_url(repo: Path, remote: str) -> str:
    url = git(repo, "remote", "get-url", remote, check=False)
    m = re.search(r"github\.com[:/]([^/]+)/([^/.]+?)(?:\.git)?$", url)
    return f"https://{m.group(1).lower()}.github.io/{m.group(2)}/" if m else ""


def publish(
    folder: Path, repo: Path, *, remote: str = "origin", push: bool = False, dry_run: bool = False,
    allow_dirty: bool = False,
) -> dict[str, str]:
    record = load_record(folder)
    branch = branch_name(record, allow_dirty)
    names = collect(folder, record)
    bad = branch_exists(repo, branch, remote if push else None)
    if bad:
        raise PublishError(bad)
    with tempfile.TemporaryDirectory() as tmp:
        staging = Path(tmp)
        files = stage(folder, names, record, branch, staging)
        html = (staging / INDEX).read_text(encoding="utf-8")
        gone = missing_links(html, files)
        if gone:
            raise PublishError(
                "the page links to files that would not be on the branch (they would 404):\n  "
                + "\n  ".join(gone))
        size = sum((staging / n).stat().st_size for n in files)
        info = {"branch": branch, "files": str(len(files)), "bytes": str(size), "commit": ""}
        if dry_run:
            return info
        message = (f"results: {str(record.get('label', 'draft')).lower()} report of the study at "
                   f"{record['git_commit']}")
        info["commit"] = commit_orphan(repo, staging, branch, message)
    if push:
        git(repo, "push", remote, f"refs/heads/{branch}:refs/heads/{branch}")
        info["pushed"] = remote
    info["pages"] = pages_url(repo, remote)
    return info


def main(argv: Sequence[str] | None = None) -> int:
    p = argparse.ArgumentParser(
        description="Publish a results folder as the orphan branch results/draft-<commit>.")
    p.add_argument("folder", nargs="?", type=Path,
                   help="results folder (default: <output_dir>/<run_name> of study.yaml)")
    p.add_argument("--repo", type=Path, default=None, help="the git clone (default: this one)")
    p.add_argument("--config", type=Path, default=None, help="config folder (default: configs)")
    p.add_argument("--remote", default="origin")
    p.add_argument("--push", action="store_true", help="push the new branch (never the default)")
    p.add_argument("--dry-run", action="store_true",
                   help="run every check and say what would be published; create nothing")
    p.add_argument("--allow-dirty", action="store_true",
                   help="publish a run made on a dirty tree (the README says so)")
    args = p.parse_args(argv)
    try:
        repo = args.repo or Path(git(Path(__file__).resolve().parent, "rev-parse",
                                     "--show-toplevel"))
        folder = args.folder or default_folder(repo, args.config or repo / "configs")
        info = publish(folder, repo, remote=args.remote, push=args.push, dry_run=args.dry_run,
                       allow_dirty=args.allow_dirty)
    except (PublishError, OSError) as exc:
        print(f"publish_results: {exc}", file=sys.stderr)
        return 1
    size = int(info["bytes"]) / 1e6
    if args.dry_run:
        print(f"dry run: would create {info['branch']} with {info['files']} files ({size:.1f} MB); "
              "every link in the page resolves; nothing was created")
        return 0
    print(f"created {info['branch']} ({info['commit'][:10]}): {info['files']} files, {size:.1f} MB")
    if "pushed" in info:
        print(f"pushed to {info['pushed']}")
    else:
        print(f"not pushed: git push {args.remote} {info['branch']}  (or use --push next time)")
    print("to serve it: Settings, Pages, branch " + info["branch"] + ", folder /"
          + (f"  ->  {info['pages']}" if info.get("pages") else ""))
    return 0


if __name__ == "__main__":
    sys.exit(main())
