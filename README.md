# Sage VTOL transition and landing-approach toolkit

Simulates a 4+1 hybrid VTOL aircraft in the vertical plane and answers: how much altitude is lost or
gained in each transition, how far the back-transition needs, what `Q_TRANS_DECEL` to configure,
whether the mission fits in the battery, and how sensitive all of that is to what you don't yet know.
It runs without the autopilot.

> **Status: the physics numbers are placeholders.** `configs/` ships with illustrative values for a
> notional 8 kg aircraft, all marked `TBD`, so every result is labelled **DRAFT** and the release gate
> blocks it. Replace the values with real data (below) and the labels follow.

## Quick start

```bash
pip install -e ".[dev]"

python run.py --check      # validate configs/, show what would run and how long it takes
python run.py              # run the whole study (about 10 minutes on a multi-core laptop)
```

Results appear in `results/study/`: open **`report.md`** first. Tables are CSV, figures PNG/SVG.

## Putting in your data

Edit the YAML files in **[`configs/`](configs/README.md)** and run `python run.py` again. That is the
whole workflow: no code changes.

| File | You put in |
| --- | --- |
| `configs/vehicle.yaml` | mass, wing, rotor and pusher coefficients, motors, battery, pitch dynamics |
| `configs/mission.yaml` | mission profile, usable-battery window, reserves, descent limits, range requirements |
| `configs/controller.yaml` | the ArduPilot parameters (real names) and the assumed controller gains |
| `configs/scenarios.yaml` | the named scenarios and the ensemble ranges |
| `configs/study.yaml` | which analyses to run, effort, draft or release mode, parallelism |

Every data entry has a value, a unit (any unit; converted once on load), a source and an evidence
status (`measured`, `datasheet`, `derived`, `documented`, `stated`, `assumed`, `TBD`). The report shows
how much of each result rests on assumptions. **`configs/README.md` documents every entry.**

* **Draft mode** (default): placeholders and `TBD` entries are used and the run is labelled DRAFT.
* **Release mode** (`python run.py --mode release`): refuses to run while anything is `TBD`, missing,
  or points at a file that does not exist.

## What it does

| Analysis | Question |
| --- | --- |
| **A** | can the hardware hold altitude through a given deceleration? (feasibility map) |
| **D** | how far does the aircraft need to brake, and what `Q_TRANS_DECEL` should the planner use? |
| **E** | does the mission fit in the usable battery energy at the required ranges, with reserves? |
| **F** | how fast may it descend for landing (vortex-ring limit) and what does the hover cost? |
| **Q2** | can the pusher reach the transition speed at altitude, against mass? |
| **C** | what can the autopilot be *configured* to do? (ArduPilot-parameter optimisation) |
| **B1** | what is the best open-loop trajectory? (direct collocation, energy against altitude excursion) |
| sensitivity | which inputs drive each result, and how firmly do we know them? |

## Command line

```text
python run.py --check            validate and show the plan
python run.py                    run everything configured
python run.py --only A D E       just these analyses
python run.py --mode release     refuse unless nothing is TBD or missing
python run.py --force            ignore cached results
python run.py --serial           one process (easier to debug)
python run.py --verify           also run the test suite and feed the release gate
```

Results are cached per task and keyed by a hash of every result-relevant input, so an unchanged re-run
only rebuilds the report and changing one value only re-runs what depends on it.

## More

* **[`PROJECT_REPORT.md`](PROJECT_REPORT.md)** — what was built, what happened, what the results mean,
  deviations from the design, and what is still open.
* **[`configs/README.md`](configs/README.md)** — every input, its unit and what it affects.
* **`src/sagetrans/README.md`** and a README in each sub-package — how the code is organised.
* **[`tests/README.md`](tests/README.md)** — the verification tests (T-01 to T-15).

```bash
pytest                              # the test suite (about 3 minutes)
ruff check . && mypy && lint-imports
```
