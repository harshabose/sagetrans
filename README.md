# sagetrans: draft results report (9a46bd4)

**This is a draft.** `index.html` is the results viewer of one study run on assumed placeholder inputs. 72 register entries are TBD. None of the numbers are release-grade, and they will change when the register is filled. Do not quote them as results.

| | |
|---|---|
| page | `index.html` (and an identical `report.html`, so the links inside the page resolve), 2.5 MB |
| files | 94: the page, the tables as CSV, `tables.xlsx`, the tornado figures, `report.md`, `run_record.json` and the rest of what the page links to |
| sha256 of the page | `20c468e4369d9a5426f8305c1ee8900006811da6465aec78c0b6149ebd3b29fb` |
| produced by | `python run.py` then `python run.py --view` on commit `9a46bd4` (clean tree), 2026-11-10 |
| run record | label DRAFT, mode draft, source hash `f15d85434b1d`, Python 3.14.7 |

How to look at it: GitHub Pages must serve this branch from its root (Settings, Pages, this branch, `/`); that is a repository setting this branch does not switch on. Without Pages, clone the branch and open `index.html` in a browser; the links to the tables work from the folder.

How to reproduce it: check out `9a46bd4`, `pip install -e ".[dev]"`, `python run.py --fresh-cache`, `python run.py --view`.

Published by `scripts/publish_results.py` as `results/draft-9a46bd4`. This branch holds only the report and shares no history with `master`.
