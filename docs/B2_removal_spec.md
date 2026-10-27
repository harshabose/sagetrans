# B2 removal: spec for the implementing agent

Status: draft, 2026-10-27. Written against the repo as it stands today (B1 and C already rewritten). Line numbers are from today's files and will move; search by the names.

## 0. Decisions already taken

1. **Analysis B2 is removed from the code, the configs, the report and the docs.** Reason: it is not repeatable (seed spread 4 to 10 % against a 1 % limit), it does not beat C (2.34 against 2.26 on the same ensemble), it has no altitude feedback, it is trained on the cruise entry that C, D and B1 have left, and it only matters if a non-ArduPilot controller is ever considered. The code is not kept behind a flag.
2. **What replaces its question** ("what does a feedback-free law achieve?") is the B1-against-C gap, already in the study (`B1_gap`). A decomposition of that gap (wing limit, rotor lag, altitude loop, ramp shape) is a separate later item; do not build it here.
3. **Not removed** (other analyses use them): `analysis/search.py` (CMA-ES driver and `bootstrap_percentile`, used by C), `analysis/runs.py` (shared helpers, used by C), `analysis/ensemble.py` (`CostWeights`, `ensemble_cost`, `RunMetrics`, used by C), `collocation.make_problem` (level-cruise problem; the legacy `bound_check` and the B1 unit tests use it), the legacy functions of `restricted.py` (C spec: they stay, `sensitivity.py` and `collocation.bound_check` call them), `zero_loss.ActuatorLimits` with its `None` semantics and `solve_point(band=None)` (A's own tests and API use them).
4. **The release gate keeps its T-12 rule.** `gate.SEARCH_ANALYSES` becomes `("C",)`. The next search-based analysis (the forward transition) adds itself.
5. **No compatibility shim.** A `study.yaml` that still lists `B2` or has a `B2:` block fails validation with the existing strict-model error. The three YAML sources are changed together (section 3).
6. **History is not rewritten.** `PROJECT_REPORT.md` keeps its milestone narrative; only statements about the current state change (section 6). The `docs/*.md` analysis notes get a one-line header note, not an edit per mention.
7. Out of scope: tests for new behaviour (the owner adds them), the proposal text (`proposal.md` is not in git and keeps its B2 sections), and the next step of any other analysis.

## 1. Files to delete

| File | Content |
| --- | --- |
| `src/sagetrans/analysis/freeform_opt.py` | analysis B2: `BOUNDS`, `decode`, `init_from_zero_loss`, `init_from_collocation`, `evaluate`, `optimise`, `FreeFormResult` |
| `src/sagetrans/control/freeform.py` | `FreeFormParams`, `FreeFormController`, `S_KNOTS` |

After deleting, nothing may import `freeform_opt`, `FreeFormController`, `FreeFormParams` or `S_KNOTS` (checked in section 7).

## 2. Code edits

### 2.1 `src/sagetrans/io/stages.py`

* Remove `freeform_opt` from the `from sagetrans.analysis import (...)` list.
* Module docstring (line 4): drop ``B2`` from the task list.
* `TASK_COST`: drop `"B2": 360`.
* Section comment `# ---- tasks: C, B2, B1, sensitivity` becomes `# ---- tasks: C, B1, sensitivity`.
* Delete `_b1_problem_cruise` and `task_b2`. Nothing else calls them (B1 uses `task_b1` and the exit problem).
* `plan_tasks`: delete the `if "B2" in wanted:` block. `run_task`: delete the `if kind == "B2":` branch.

### 2.2 `src/sagetrans/io/assemble.py`

* Delete the whole B2 block (from `b2 = task_results.get("B2", {})` to the end of its `sections["B2 — ..."]` assignment): it fills `spreads["B2"]`, `analyses["B2"]`, `tables["B2_search"]`, `tables["B2_ensemble_runs"]` and one section.
* C section text: remove the sentence "B and C are not comparable yet: B1 and B2 still start at level cruise." (B1 and C are comparable: same entry.)
* B1 section text: remove "B2 still starts at level cruise, so B2 and B1 are not comparable yet."
* `limitations`: "Back-transition only for B1, B2 and C; the forward transition is not optimised." becomes "Back-transition only for B1 and C; the forward transition is not optimised."

### 2.3 `src/sagetrans/io/settings.py`

* `AnalysisName` and `ALL_ANALYSES`: remove `"B2"`.
* Delete `B2StartCfg` and `B2Cfg`, and the field `B2: B2Cfg = B2Cfg()` of the study model.
* `result_relevant()` needs no change.

### 2.4 `src/sagetrans/io/gate.py` and `record.py`

* `gate.SEARCH_ANALYSES = ("C",)`.
* `record.py`: the two comments/docstrings that say `("B2", "C")` become `("C",)` ("each analysis that used a stochastic search, today only C").

### 2.5 `src/sagetrans/analysis/collocation.py`

* `compare(prob, b1, c_params=None, b2_params=None, *, c_eval=None, c_base=None)`: remove `b2_params`, the `freeform_opt` import, and the `if b2_params is not None:` block. The function returns `{"B1": ..., "C": ...}`. Docstring: drop the B2 sentences, the "(B-9 stays open until B2 migrates)" clause, and say "B1 and C".
* Section comment `# ---- comparison with the baseline, C and B2` becomes `... the baseline and C`.
* Module docstring: line 8 "B1 keeps operational limits that B2 and C do not respect" becomes "that C does not respect"; line 10 "or the free-form law" is deleted; line 18 "(the legacy entry that B2 still starts from)" becomes "(the legacy level-cruise entry, used by `bound_check` and the unit tests)"; line 27 "comparisons with B2 and C" becomes "comparisons with C".

### 2.5b `src/sagetrans/analysis/zero_loss.py` (minimal)

* Delete `LEGACY_LIMITS` and its entry in `__all__`.
* In `analyse`, delete the `if legacy:` branch that rebuilds `om_dot`/`th_dot` with a prepended sample ("B2's start is built from this path ...") and keep only the `else` body: `om_dot, th_dot = np.gradient(om, t), np.gradient(th, t)`. The only remaining way to reach that path is an `A` config with both `alpha_min_deg: null` and `cl_frac: null`; its first-sample rates then use the same construction as the banded path (the old one halved the first rate), which changes only that unusual configuration.
* The comment on `ActuatorLimits.cl_frac` and `alpha_min` ("legacy") stays: the `None` semantics remain (A's tests call `solve_point` with `band=None`).
* Acceptance (section 7): every A table is identical to the table from before the change (default limits take the unchanged path).

### 2.6 Docstring-only mentions

* `analysis/runs.py` line 1: "(B2, C)" becomes "(C)".
* `analysis/restricted.py` module docstring line 16: "B2 and B1 are compared with C on that entry until they move" becomes "B1 is compared with C on the AIRBRAKE-exit entry through `evaluate_c`; the legacy functions are called by `sensitivity.py` and `collocation.bound_check`" (keep it to one sentence; do not change the list of legacy functions).
* `control/ardupilot_like.py`: no change.

## 3. Configs (three places, same edit)

`configs/study.yaml`, `tests/data/placeholder_configs/study.yaml`, and the string template in `scripts/make_placeholder_configs.py`:

* Comment line `# Which analyses to run. Names: A D F E Q2 C B2 B1 sensitivity` becomes `... C B1 sensitivity`.
* `analyses: [A, D, F, E, Q2, C, B2, B1, sensitivity]` becomes `analyses: [A, D, F, E, Q2, C, B1, sensitivity]`.
* Delete the whole `B2:` block (`ensemble_n`, `ensemble_seed`, `seeds`, `popsize`, `maxiter`, `sigma0`, `start`).
* In `configs/study.yaml` only, `C.ensemble_seed: 8  # same ensemble as B2 so the two can be compared` becomes `ensemble_seed: 8  # fixed seed of the 32-member ensemble`.

`tests/data/placeholder_configs/study.yaml` is generated by the script: regenerate it with the script after editing the template and check that the two agree (the drift test reads both).

## 4. Tests (delete or adapt; the owner may rewrite them later)

| File | Change |
| --- | --- |
| `tests/unit/test_m5.py` | Remove `freeform_opt` from the analysis import and the `FreeFormController, FreeFormParams` import. Delete `test_freeform_law_interpolates_in_speed_and_has_no_altitude_feedback` and the `# ---- analysis B2` section (`test_b2_initial_knots_come_from_analysis_a_and_stay_in_bounds`, `test_b2_optimise_never_returns_worse_than_its_start`). The tail of the second B2 test checks that C's cost is finite with the same cost definition: keep that check as its own small test of `restricted.optimise` if it is not covered elsewhere. |
| `tests/unit/test_m6.py` | Remove the `freeform_opt` import. In `test_b1_beats_the_baseline_at_the_baselines_own_excursion_and_distance` delete the two `b2` lines and expect `set(table) == {"B1", "C"}`; call `K.compare(prob, bc.b1)`. Delete `test_b2_can_start_from_the_b1_solution`. |
| `tests/unit/test_m7.py` | `test_gate_checks_t12_on_the_real_problem_not_only_the_analytic_unit_test`: replace `B2` by `C` in the "never shown repeatable" and "seeds disagree by 9 %" cases (a missing spread, 0.09 and NaN must each block `C`); keep the "2e-5 passes" case. For the "one repeatable, one not" case use `monkeypatch.setattr(gate, "SEARCH_ANALYSES", ("C", "FWD"))` and a record with `FWD` at 0.044. Update the comments (they quote B2's 4 to 9 %). |
| `tests/unit/test_analyses.py` | Delete `test_a_legacy_limits_keep_the_old_non_monotone_verdict_that_b2_relies_on`. In the test that asserts `zero_loss.DEFAULT_LIMITS.vrs_limit is None and zero_loss.LEGACY_LIMITS.vrs_limit is None` keep only the first operand. |
| `tests/unit/test_inputs.py` | The expected task set becomes `{"basics", "C", "B1:S-01", "B1:S-02", "sensitivity"}`. |
| `tests/README.md` | Rows for `test_m5.py` (now 13 tests, drop "free-form controllers, analyses B2 and C" to "baseline controller, analysis C") and `test_m6.py` (now 9 tests, drop "and B2 starting from a B1 solution"); recount with `pytest --collect-only -q`. |

## 5. Documentation in the package

| File | Change |
| --- | --- |
| `README.md` | Delete the `B2` row of the analyses table. |
| `configs/README.md` | `analyses` row: list without `B2`; delete the `B2` row of the settings table; in the effort-knobs sentence drop `B2.maxiter`. |
| `src/sagetrans/README.md` | Layer list: `analysis  A, B1, C, D, E, F, sensitivity, search, ensembles`; `control  controllers (baseline), mode state`. |
| `src/sagetrans/io/README.md` | `stages.py` row: tasks `basics`, `C`, `B1:<scenario>`, `sensitivity`; `search_spreads` row: `("C")` only; gate paragraph: `gate.SEARCH_ANALYSES` is `("C",)` and delete the sentence about B2's seeds and the blocked record; the example comment "analysis C or B2" becomes "analysis C". |
| `src/sagetrans/control/README.md` | Delete the `freeform.py` row, the `## FreeFormController` section (eq. 8.1, knots, tests sentence for the free-form law) and the clause "so C and B2 are unchanged" (line 70: "so C is unchanged"). |
| `src/sagetrans/analysis/README.md` | Delete the `freeform_opt.py` row and the whole `## B2 — speed-scheduled law` section. Edit the lines that mention B2 as live: the helper list (`shared by B2, C and B1` to `C and B1`), the `bootstrap_percentile` line (`C and B2 use it` to `C uses it`), the `ActuatorLimits` row and line 126 ("this is what B2's start uses": delete the clause and the `LEGACY_LIMITS` mention), line 191 (the legacy non-monotone verdict test no longer exists), line 270 and 280 (B2 in the lists of analyses that depend on the rotor-inertia lag), line 367 ("Analyses C and B2 keep the old behaviour" to "Analysis C keeps ..."), lines 658 to 660 and 719 to 720 (replace "B-9 ... stays open until B1 and B2 start at the AIRBRAKE exit" by "B-9 (B2 against C) is withdrawn: B2 was removed"), the `Comparing with C and B2` heading and its text (B1 and C only; `compare` returns two rows), the sentence that B1 is not a strict lower bound "that the baseline and B2 do not respect" (C only), and delete the "Free-form law, tuned (B2)" row of the level-cruise table. |
| `PROJECT_REPORT.md` | Section 6. |

## 6. `PROJECT_REPORT.md`

Edit the statements about the present only; leave the milestone history (M5, the T-12 table, "B2 from the B1 solution", the deviations table, the historical results) as written, and add one dated note.

* Line 27: "seven analyses (A, B1, B2, C, D, E, F)" becomes "six analyses (A, B1, C, D, E, F)"; add after the first paragraph of the summary: "Analysis B2 (speed-scheduled free-form law) was removed on 2026-10-27: it was not repeatable (seed spread 4 to 10 % against 1 %) and did not beat C. Its history below is kept as written."
* Lines 49, 93, 94, 467, 500, 539, 578 to 580, 606, 622: describe the current state without B2 (module list: drop the two files; tasks list: `basics`, `C`, `B1:<scenario>`, `sensitivity`; open items: delete "Make B2 repeatable"; the gate sentence about B2 goes).
* Section "B2 from the B1 solution" (line 330): add as its first line "(B2 was removed on 2026-10-27; this section is history.)".

## 7. Docs folder notes

At the top of each of `docs/collocation.py.md`, `docs/restricted.py.md`, `docs/zero_loss.py.md`, `docs/basics.md`, `docs/bracking.py.md`, `docs/ardupilot_like.py.md`, add: `> B2 was removed from the code on 2026-10-27. Mentions of B2 below describe the code at the time of writing.` Do not edit the bodies. In the fix specs (`A_fix_spec.md`, `B1_fix_spec.md`, `C_fix_spec.md`, `D_fix_spec.md`) add the same one-line note under the title; the statements in them that B2 "stays on the legacy cruise path", "keeps `LEGACY_LIMITS`", or "is the same ensemble as C" are superseded by this spec.

## 8. Verification

1. `rg -n "freeform|FreeForm|B2|b2_|task_b2|LEGACY_LIMITS|_b1_problem_cruise" src tests configs scripts README.md` returns no hits except: the dated note lines of section 6 and 7, and the words "B2" inside `PROJECT_REPORT.md` history. In `docs/` only the notes and the fix specs may still contain B2.
2. `pytest -q`, `ruff check`, `mypy` (strict, `src`), `lint-imports` all pass. Test count: previous total minus the 5 deleted tests plus any you added.
3. `python run.py --check` prints the plan `basics`, `C`, `B1:S-01` ... `sensitivity` (no `B2`) and "inputs are valid".
4. `python run.py --only A D F E Q2` before and after the change (zero_loss edit, section 2.5b): `A_feasibility.csv` and `A_max_feasible_decel.csv` are byte-identical (same SHA-256 in `run_record.json`).
5. A full run: `run_record.json` has no `B2` key in `analyses` or `search_spreads`; no `B2_*.csv`; `report.md` has no B2 section; the release gate lists no B2 reason. The first full run after this change recomputes every task (the settings hash changes because the `B2:` block is gone); that is expected.
6. A `study.yaml` that still contains `B2:` is rejected with a validation error naming the key (check once by hand).
7. Old result folders still on disk keep their `B2_*.csv`; nothing reads them.

## 9. Open items this leaves

* B-9 (B2 against C) is withdrawn, not answered. The B1-against-C gap (`B1_gap`) is the replacement.
* `proposal.md` (not in git) still describes B2 in its analysis and verification sections; revise it separately.
* A later decomposition of the B1 to C gap into wing limit, rotor lag, altitude loop and ramp shape is proposed, not scheduled.
