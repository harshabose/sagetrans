# `sagetrans.io` — outputs, run records, the report, and the release gate

Layer position: the top. It may use everything below it. It turns analysis results into files that
can be shared, and decides whether a result may be released. (The package is named `io`; inside it,
absolute imports mean it does not shadow the standard library.)

| File | Contents |
| --- | --- |
| `record.py` | `FullRunRecord`, `build_record`, `git_state`, `check_outputs`, `sha256_file` |
| `results.py` | `write_results`, `write_csv`, `require_writable` — the folder writer with the DRAFT guard |
| `figures.py` | tornado, range-margin, Pareto and feasibility figures; `save_figure` |
| `report.py` | `ReportInputs`, `build_report`, `flagged_assumptions` |
| `gate.py` | `release_gate`, `GateResult`, `verification_from_junit`, `REQUIRED_TESTS` |

## The run record (`record.py`)

Written as `run_record.json` next to every output (FR-09, NFR-07). A `FullRunRecord` holds:

| Field | Meaning |
| --- | --- |
| `label` | `DRAFT` or `RELEASED` |
| `reasons` | why it is DRAFT (empty when released) |
| `config_hash` | the parameter register's SHA-256 (`config.Config.config_hash`) |
| `evidence_counts` | how many register inputs fall in each evidence grade |
| `code_version`, `git_commit`, `git_dirty`, `date` | where the numbers came from |
| `analyses` | name → the input hash of each contributing analysis's own `RunRecord` |
| `seeds` | seeds used by stochastic searches |
| `override` | `True` if the DRAFT refusal was overridden for this write |
| `outputs` | file name → SHA-256 of every file written |

`build_record(config, analyses, seeds=(), override=False)` combines the register state with the
analyses' records. **It is DRAFT if the configuration has any `TBD` parameter, or if any
contributing analysis record is DRAFT** — and every analysis record is DRAFT while the physics runs
on placeholders. `git_state()` returns the short commit and a dirty flag (`"no-commit"` and dirty
when there is no commit, as in this repository today). `record.footer()` is the one-line text put on
figures: `run record <hash8> | DRAFT | commit … | date`.

`check_outputs(folder)` re-hashes every file the record names and returns `{file: matches}`, so a
hand-edited output is detected.

## Writing results (`results.py`)

```python
write_results(root, name, record, config, *, tables=None, figures=None, texts=None, override=False)
```

* **The DRAFT guard (T-14).** If `record.label == "DRAFT"` and `override` is false it raises
  `DraftConfigError` listing the reasons, and **writes nothing**.
* Otherwise it creates `root/name/`, writes each table as CSV (`write_csv`: columns in first-seen
  order, `\n` line endings), each figure as PNG and SVG with the record footer, and each text file;
  hashes every file; and writes `run_record.json` with the `outputs` map and the `override` flag.
* `results/` is git-ignored; every folder in it holds a `run_record.json`.

**Reproducibility is byte-for-byte.** Identical inputs give identical files. Two things made that so:
the SVG backend salts element ids randomly by default, so `svg.hashsalt` is fixed in `figures.py`;
and timestamps and software tags are stripped from PNG and SVG metadata. (A test writes the same
bundle twice and compares hashes.)

## Figures (`figures.py`)

Matplotlib with the non-interactive `Agg` backend.

| Function | Shows |
| --- | --- |
| `tornado_figure(rows, output, top=12)` | one-at-a-time sensitivity: a bar for each input from the low to the high end of its range (full range light, half range dark), coloured by evidence grade |
| `range_margin_figure(budget, required_km)` | energy margin against range, with the 100 and 113 km requirements and the maximum range |
| `pareto_figure(points)` | bus energy against altitude excursion, coloured by distance |
| `feasibility_figure(v0_grid, a_grid, feasible)` | the zero-loss feasibility map |
| `save_figure(fig, folder, stem, footer)` | writes `stem.png` and `stem.svg` with the footer |

## The report (`report.py`)

`build_report(ReportInputs(...))` returns markdown in the proposal's five-part structure (§11):

1. **Summary** — the headline numbers, each with the **weakest evidence grade among the inputs that
   dominate it**; the `Q_TRANS_DECEL` recommendation; and the range margin at 100 and 113 km with a
   flag where the requirement and usable energy conflict.
2. **Evidence grade** — counts per grade, and the list of assumed or unknown register parameters.
3. **Results by analysis** — text sections supplied by the caller.
4. **Sensitivity** — the global ranking at two sample sizes (Spearman, top-3 overlap), the dominating
   inputs with their grade and range, and the names flagged as assumptions.
5. **Limitations** — linked to the uncertainty register.

A DRAFT record produces a prominent banner with its reasons. `flagged_assumptions(sens)` lists the
dominating inputs that are not measured; the report states them as assumptions with their range.

## The release gate (`gate.py`)

A result is released only if **all** of the following hold (proposal §12); `release_gate(record,
verification, sens, flagged_assumptions, required_tests=T-01…T-15)` returns a `GateResult(passed,
reasons, warnings)` and `.summary()` prints it.

1. **The run record is not DRAFT.**
2. **Every required verification test is present and passing.** An ID with no result counts as a
   failure.
3. **The evidence-grade summary is attached** (non-zero counts).
4. **The sensitivity ranking is stable** across the two sample sizes (Spearman ≥ 0.8 and top-3
   overlap ≥ 2/3 for every output), **and every dominating input is either measured or flagged as an
   assumption with its range.** An input that is still `TBD` cannot be flagged away — it must be
   supplied. Flagged-but-unmeasured inputs pass with a warning that carries their range.

`verification_from_junit(path)` reads a pytest JUnit XML file and maps tests whose names start
`test_tNN_` to `T-NN`; an ID passes only if every test carrying it passed. The suite's tests are
named to this convention (for example `test_t11_collocation_replayed_…`).

**Status today: the gate always blocks.** The register has `TBD` mass and wing area, and every
analysis ran on placeholder physics, so the record is DRAFT; and mass and wing area dominate several
outputs. That is the intended behaviour, shown in a demo run: `RELEASE: BLOCKED` with the DRAFT
reasons and the TBD dominating inputs. CI does not yet produce a JUnit file or call the gate.

## Example

```python
import dataclasses
from sagetrans.analysis import sensitivity as S
from sagetrans.analysis.common import Scenario
from sagetrans.config.schema import Config
from sagetrans.io import figures, gate, report, results
from sagetrans.io.record import build_record
from sagetrans.physics.params import placeholder_vehicle

vp, scen = placeholder_vehicle(), Scenario(100.0)
cfg = Config.from_yaml("configs/vehicle.yaml")
model = S.HeadlineModel(vp, scen, S.default_inputs(vp, scen))
sens = S.run_sensitivity(model, r_small=6, r_large=12)
rec = build_record(cfg, {"sensitivity": sens.record}, seeds=(0,))
text = report.build_report(report.ReportInputs("Study", rec, cfg, model({}), sens=sens))
folder = results.write_results(
    "results", "run1", rec, cfg,
    tables={"oat": [dataclasses.asdict(r) for r in sens.oat]},
    figures={"tornado_excursion": figures.tornado_figure(sens.oat, "excursion_m")},
    texts={"report.md": text}, override=True)          # DRAFT: override is required
print(gate.release_gate(rec, {t: True for t in gate.REQUIRED_TESTS}, sens,
                        report.flagged_assumptions(sens)).summary())
```

## Tests (`tests/unit/test_m7.py`)

The record (DRAFT for a TBD register or placeholder analyses, RELEASED only when both are release
grade), **T-14** (refused without an override, nothing written, every file hashed, tampering
detected), byte-for-byte reproducibility and the figure footer, every gate path (passes, DRAFT,
missing or failed test, unflagged and TBD dominating inputs, unstable ranking, no sensitivity), JUnit
mapping including a real pytest run, the report's five sections and DRAFT banner, and an end-to-end
bundle on the real model.
