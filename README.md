# sagetrans: draft results report (0e6e606)

**This is a draft.** `index.html` is the results viewer of one study run on assumed placeholder inputs. 72 register entries are TBD. None of the numbers are release-grade, and they will change when the register is filled. Do not quote them as results.

| | |
|---|---|
| page | `index.html` (and an identical `report.html`, so the links inside the page resolve), 2.9 MB |
| files | 98: the page, the tables as CSV, `tables.xlsx`, the tornado figures, `report.md`, `run_record.json` and the rest of what the page links to |
| sha256 of the page | `7d58caf1617131cc23dd0c06cdaa1a37ce7e35d813453d3b1552d97b89255446` |
| produced by | `python run.py` then `python run.py --view` on commit `0e6e606` (clean tree), 2026-10-29 |
| run record | label DRAFT, mode draft, source hash `06381b053b7a`, Python 3.14.7 |

How to look at it: GitHub Pages must serve this branch from its root (Settings, Pages, this branch, `/`); that is a repository setting this branch does not switch on. Without Pages, clone the branch and open `index.html` in a browser; the links to the tables work from the folder.

How to reproduce it: check out `0e6e606`, `pip install -e ".[dev]"`, `python run.py --fresh-cache`, `python run.py --view`.

Published by `scripts/publish_results.py` as `results/draft-0e6e606`. This branch holds only the report and shares no history with `master`.
