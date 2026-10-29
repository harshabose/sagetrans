# sagetrans: draft results report (ae83086)

**This is a draft.** `report.html` is the results viewer of one full study run on assumed placeholder inputs.
72 register entries are TBD and the release gate is BLOCKED. None of the numbers are release-grade, and they
will change when the register is filled. Do not quote them as results.

| | |
|---|---|
| file | `index.html` (the viewer's `report.html`, renamed so GitHub Pages can serve it), one self-contained page (no network references), 2.2 MB |
| sha256 | `f09340914cdf87a3479eb04f7803f3173a475e16adcab15fb6926353ea20cc8c` |
| produced by | `python run.py` then `python run.py --view` on commit `ae83086` of `master` (clean tree), 2026-10-29 |
| run record | label DRAFT, source hash `e2a9365070ea`, Python 3.14.7 |
| mode | draft (placeholder and assumed inputs) |

How to look at it: download `index.html` and open it in a browser. On github.com the file is shown as source, not rendered; to see it rendered online, GitHub Pages has to serve this branch from its root (a repository setting, not enabled by this commit).

How to reproduce it: check out `ae83086`, `pip install -e ".[dev]"`, `python run.py --fresh-cache` (about 4 hours on
24 workers), `python run.py --view`.

This branch holds only the report. It shares no history with `master`.
