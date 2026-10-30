# Outputs: results viewer (HTML) and workbook — implementation spec

Status: DRAFT for review. Reference mock: `docs/report_mock.html` (static, hand-built from the real
C and B1 numbers; open it before reading this spec). Companion: `docs/B2_removal_spec.md`
(do that first, so the viewer never has to know about B2).

Audience of the output: the author and the lab. Technical, complete, true to the tables. Length is
not a problem; a wrong or untraceable number is.

---

## 0. Decisions already taken

1. The HTML is the **main report**. It is a **viewer**: it reads a results folder that already
   exists and renders it. It never runs an analysis, never changes a result.
2. **Launch on existing results.** `python run.py --view [folder]` (default: `output_dir` from
   `study.yaml`). If the folder has no `run_record.json` and no recognised table, print
   `no results in <folder>; run the study first` and exit 2. **Do not write or open any file in
   that case.** If results exist, write `report.html` and `tables.xlsx` into the folder and open the
   HTML in the default browser (`--no-open` to suppress).
3. A **workbook** is part of the package (`tables.xlsx`).
4. Interactivity is **minimal**: hover tooltip on marks, a crosshair on time-history panels, a
   sticky table of contents, collapsible table blocks. No filters, no toggles, no animation, no
   zoom. Anything else needs a stated reason in the PR.
5. One self-contained file: inline CSS, JS, SVG, data. No network, no CDN, no fonts fetched.
6. Light and dark themes through tokens (`prefers-color-scheme`), as in the mock.
7. DRAFT results are viewable. The banner is shown whenever `run_record.label == "DRAFT"`; the
   existing `require_writable` rule is for the study writer, not the viewer.

## 1. Command line and layering

- `cli.py`: new flags `--view [FOLDER]`, `--no-open`. `--view` is exclusive with running analyses
  (error if combined with `--only`, `--force`, `--verify`, `--check`).
- New package `sagetrans/io/view/`, top layer like the rest of `io` (it may import `io.record`,
  `io.settings`; **analysis code is not imported**). Update the import-linter contract only if it
  lists `io` submodules.

```
io/view/
  __init__.py      view(folder, open_browser) -> int      # entry point used by cli
  load.py          read run_record.json, discover tables, parse CSV -> typed columns
  registry.py      CHAPTERS: ordered list of Chapter(id, title, requires, build)
  charts.py        pure functions rows -> SVG string (own SVG, no library)
  html.py          page shell, CSS/JS strings, tokens, assemble chapters
  workbook.py      tables -> tables.xlsx (openpyxl)
  units.py         column -> (label, unit, format) lookup, from suffixes (_m, _ms, _kJ, _deg, _s, ...)
```

- New dependency: `openpyxl` in `pyproject.toml` `dependencies`. No other new dependency.
- Charts are written by hand as SVG (the mock's `build.py` is the template: linear scales, 2 px
  lines, 5 px markers, direct labels, `data-tip` attributes). No matplotlib in the viewer path.
  The existing matplotlib `figures/` PNG/SVG stay as they are, for the paper and the proposal.

## 2. Data contract (what the viewer reads)

Input is the results folder only:

| File | Use |
| --- | --- |
| `run_record.json` | title block, label (DRAFT/RELEASED) and reasons, config hash, commit, dirty flag, date, seeds, search spreads, per-analysis input hashes, file hashes |
| `*.csv` | every table. Name pattern `<ANALYSIS>_<name>.csv` or `sens_*.csv` |
| `release_gate.txt` | shown verbatim in a collapsed block |
| `inputs_used.md` | shown verbatim in a collapsed block (register inputs and grades) |
| `report.md` | not used; kept as is |

Rules for the loader:

1. Parse CSV with the stdlib; per column, try float, then bool (`True`/`False`), else string; empty
   cell is `None`. Keep the original header string: **the header is the column's identity**.
2. Verify file hashes against `run_record.json["outputs"]`. A mismatch is not an error: show a
   warning chip on that chapter ("edited after the run").
3. **Tolerance.** A chapter renders if its `requires` tables exist. A missing table prints one line
   in that chapter ("not in this run: C_q_curve") and the rest of the chapter still renders. A
   missing column makes that chart/column absent, never a crash. Unknown tables and columns still
   appear in the workbook and in the chapter's "all tables" block.
4. **Legacy folders.** The current `results/study` has `B1_pareto_S-01.csv`, `C_ensemble_runs.csv`
   and no `C_runs`/`C_q_curve`. The registry lists aliases (`C_runs` <- `C_ensemble_runs`), and the
   page header shows "results written by an older version; some figures need a new run".
5. Never invent: a number on the page is a cell of a table, or an arithmetic expression of cells
   shown next to it (e.g. the 34 % in the mock is `1 - 8.22/12.4`; show the expression in the
   tooltip). Anything from outside (a constant, a placeholder) is labelled "layout only".

## 3. Page structure

Single column, max width about 1100 px for text, charts full width of the column; sticky TOC on wide
screens, collapsed at phone width. Order:

1. **Header**: title, run label chip (DRAFT in a warning style, RELEASED neutral), config hash,
   commit (+ "dirty" chip), date, seeds, scenario count, the footer line from
   `FullRunRecord.footer()`.
2. **Recommended settings** (the first screen, as in the mock): one table, one row per ArduPilot
   parameter: parameter, value, unit, source analysis, evidence grade, flags, "what moves it".
   Source: `settings_recommended.csv` (new, section 5). Below it, "not decided by this run" for
   parameters with no recommendation and the reason.
3. **Chapters** A, Q2, D, F, E, C, B1, Forward (when it exists), Sensitivity. Same internal order in
   every chapter (section 4).
4. **Inputs and grades**: `inputs_register.csv` (new) as a table with the grade as a coloured chip
   plus text, sorted worst grade first; `inputs_used.md` collapsed below it.
5. **Run record and files**: every file, size, hash prefix, "verified/edited" chip. Link to each CSV
   (relative `href`; works from the folder).
6. **Release gate**: `release_gate.txt` verbatim, collapsed.

### 3.1 Chapter layout (identical for all)

```
H2 title            one-line question the analysis answers
Result box          3-6 numbers, each with unit and source "table.column"; flags in a row
Figures             in the order of section 4
Tables              the main table inline; "all tables" in a <details>, each with a row count
Settings & inputs   the study.yaml block that governed this analysis (from run_record / config), <details>
Notes               limitations the analysis prints (existing `limitations` of ReportInputs)
```

### 3.2 Tables on the page

- Numbers right-aligned, monospaced digits, unit in the header (`energy [kJ]`), format from
  `units.py` (kJ 1 decimal, m 1 decimal, deg 1 decimal, ms 2 decimals; otherwise 4 significant).
- Sticky header; for tables wider than the column, horizontal scroll inside the block.
- Flag cells (`flags`, `below_vs`, `overshoot`) render as text plus a small marker, never colour
  alone. A non-empty `flags` row has a left border.
- No sorting, no search. The workbook is for that.

### 3.3 Interaction (the whole list)

| Where | What | Why it earns its place |
| --- | --- | --- |
| Charts with markers | tooltip with the full row (hover and keyboard focus) | the number behind a mark is the point of a technical report |
| Time-history panels | one shared crosshair across the panels of a figure | compares the same instant in V, h, theta, throttle |
| TOC | anchor links, current chapter highlighted | long page |
| Table blocks | `<details>` | long page |
| Theme | follows the OS; no toggle | |

Nothing else. No JS framework; one small inline script (the mock's `app.js`).

## 4. Chapters: tables, figures, result box

`requires` = tables the chapter cannot render without. Column names are the ones the code writes
today unless marked NEW.

### A · zero-loss feasibility
- Tables: `A_feasibility`, `A_max_feasible_decel`, `A_window` (optional).
- Figures: (1) feasible map, entry speed x deceleration, two colours plus the limiting constraint
  as a label per boundary segment; (2) window at each entry speed, coloured by the limit that
  closes it (categorical, colour + legend + direct label); (3) wing angle used along the window
  (line per scenario).
- Result box: max feasible deceleration by scenario; lowest entry speed with a window.

### Q2 · pusher acceleration
- Tables: `Q2_pusher_margin`, `Q2_worst` (optional).
- Figures: **Pusher acceleration by altitude, mass and state of charge**: one heat grid of the
  rows that feed E (`feeds_e` = yes, the convention `Q2.feed_convention`, `eas`): a row per altitude,
  a column per mass factor and state of charge (`mass/SoC in %`), the value printed in each cell,
  a cross where the end speed is not reached, and a last column `E` with the acceleration E assumes
  for the site at that altitude (`E_summary.a_fwd_out_ms2`). It replaces the scatter against
  altitude, the heat strip by state of charge and the true-against-equivalent chart. A folder
  without `feeds_e` uses `eas` (or the only convention present). The tiles use the same rows. Both
  conventions stay in the `Q2_pusher_margin` table.
- Result box: worst margin and its case; whether E's assumption holds everywhere (yes/no + number).

### D · braking
- Tables: `D_a_eq`, `D_trade_curve`, `D_recommendation_ensemble`.
- Figures: (1) achievable deceleration against ground speed, ensemble as thin lines, recommended
  curve thick; (2) time histories of the representative run (V, h, theta, throttle; crosshair);
  (3) distance–energy trade curve.
- Result box: recommended `Q_TRANS_DECEL`, its margin to the ensemble worst case.

### F · landing descent
- Tables: `F_descent`, `F_profile`.
- Figures: (1) altitude, rate, power against time (three stacked panels); (2) margin to the
  vortex-ring onset against time.
- Result box: descent rate used, `kappa` and its source, minimum VRS margin.

### E · energy budget
- Tables: `E_summary`, `E_budget`, `E_segments`, `E_cruise_sweep`, `E_hover_check`, `E_reserve`.
- Figures: (1) **Maximum range and the required ranges: energy by segment**, one panel per site
  and mission (one way left, round trip right), from the `case` rows of `E_budget`: `plausible`
  (the maximum range the usable energy allows, solid bars; the total ends on the usable-energy
  rule) and one `required_<km>` row per requirement of `mission.yaml` (100 and 120 km), faint and
  outlined (dashed, dotted), drawn behind the plausible bars when a pure-CSS checkbox is on. A
  requirement is the distance flown: a round trip of 100 km is a 50 km radius (`range_km` is the
  one-way distance or the radius, `flown_km` the distance flown in all). The verdict line of each
  panel is computed from the rows ("short of 100 km by 8.0 km"); a column of panels shares one
  energy axis, so the checkbox does not move anything. A folder without a `case` column keeps the
  older two figures (whole-mission waterfall, maximum range against the requirement). (2)
  state-of-charge trace with the reserve band; (3) cruise sweep (range against cruise speed,
  optimum marked).
- **Scenarios at the recommended settings** (after "Register against recommended settings", only
  with `E_scenarios`, written by `assemble.e_scenarios`): one row per scenario of
  `scenarios.yaml` with its altitude, ISA offset, mass, SoC and headwind, and the AIRBRAKE-exit
  state D, C and B1 start from at the register's and the recommended `AIRSPEED_MIN` (entry
  airspeed `AIRSPEED_MIN - 2` read as EAS, its TAS, the ground speed, V/Vs). The pitch and rotor
  start are the settings of D and C.
- `E_summary` columns: `max_range_km` (one-way distance or radius), `flown_total_km`, and per
  requirement `margin_Wh_at_<km>km`, `conflict_<km>km` and `radius_at_<km>km` (the radius that
  flies the required distance).
- Result box: min range margin and where; limiting reserve; hover check pass/fail.

### C · restricted back-transition search (reference mock)
- Tables: `C_params`, `C_search`, `C_runs` (alias `C_ensemble_runs`), `C_q_curve`, `C_sens`,
  `C_history` (NEW).
- Figures (these exist in the mock — copy their construction from `mock/build.py`):
  1. **Energy against stopping distance** at each `Q_TRANS_DECEL`: register ramp (dashed) and
     searched ramp (solid) from `C_q_curve` rows (`q`, `energy_kJ`, `distance_m`); B1 front points
     overlaid when `B1_front` exists. Hollow marker where `flags` is non-empty.
  2. **Time histories** of the optimum member (V, h, theta, rotor throttle), from `C_history`;
     B1 series overlaid from `B1_history` when present. Shared crosshair.
  3. **Search convergence**: best cost per seed (`C_search`), spread printed; points at a search
     bound flagged from `C_params.at_bound`.
  4. **Ensemble spread**: `energy_kJ`, `excursion_m`, `end_sink_ms`, `overshoot_m` per member
     (strip plot, one row per metric, member index on y jitter-free), flagged members hollow.
  5. **One-at-a-time sensitivity** (`C_sens`): for each parameter the change in J relative to the
     optimum, dot plot with the parameter values labelled.
- Result box: optimum parameters (`T_bt_s`, `theta_A_deg`, `spin_min`, `Q_TRANS_DECEL_ms2`), J,
  seed spread, number of flagged members.
- **Report fixes (branch `feat/c-b1-report-fixes`)**, all read from columns, never recomputed in the
  viewer:
  - *Time histories (C1).* `C_history` rows carry `view` (`core` | `default_fails`), `scenario`,
    `member` (-1 = not an ensemble member), the scenario's axes, the entry airspeed and the B1 band.
    Figure "Same scenario as B1 (<name>)" = C optimum and the ArduPilot default on `B1.scenarios[0]`
    next to the B1 replays; figure "Where the default fails (ensemble member N)" = both C cases on the
    member with the largest default overshoot. The last sample of every run is kept and marked
    ("stops at"); the airspeed panel starts at 0. An older folder (no `view`) gets the old figure.
  - *Margins (C2).* `C_params` (every case): `os_tol_m`, `sink_tol_ms`, `max_end_sink_ms`,
    `os_margin_m`, `sink_margin_ms`, `at_edge`; row `optimum_margin` (largest fine-grid q whose worst
    overshoot AND worst end sink are within `C.margin_frac` of their tolerances; reported, never the
    recommendation); `C_q_curve` has `grid` (`coarse` | `fine`) and `max_end_sink_ms`. Tiles, an
    at-the-edge chip and a margin chart (overshoot and sink against q, tolerances as lines).
  - *Ensemble spread (C3).* The default's members as hollow grey dots, tolerance rules from
    `C_params`, one computed statement (overshoot range, largest end sink, why the overshoot is
    positive: `x_tgt = V^2 / 2q` from t = 0 while the ramp delays braking).
  - *Sensitivity (C4).* `C_sens` has `n_failing_members`, `n_not_stopped`, `n_fail_total`, `J_smooth`
    (J minus the penalty); two aligned charts: failing members (by type in the tooltip) and the
    change of `J_smooth`.
  - *Energy against distance (C5).* Marks drawn register, default, optimum (last); every coarse q
    labelled; no B1 points.
  - *Search convergence (C6).* `C_search` has `T_bt_s`, `theta_A_deg`, `spin_min`,
    `Q_TRANS_DECEL_ms2` per seed; a computed ridge / single-optimum / not-converged sentence; the
    `theta_A:hi` chip says the bound is `min(30, PTCH_LIM_MAX_DEG)` (`theta_A_hi_deg`,
    `theta_A_cfg_hi_deg`, `ptch_lim_deg` in the optimum row).

- **Wing-angle work (branch `fix/time-history-wing-angle`).** All read from columns.
  - *Stall exposure.* `C_runs`: `stall_alpha_peak_deg`, `stall_t_above_peak_s`,
    `stall_t_above_limit_s`, `stall_wing_share_min` (samples with V above 5 m/s and t > 0); every case
    row of `C_params`: `alpha_enforced`, `alpha_limit_deg`, `stall_members`,
    `stall_alpha_peak_deg_max`, `stall_members_above_peak`, `stall_t_above_peak_s_max`,
    `stall_t_above_limit_s_max`, `stall_wing_share_min`. Reported, never penalised, never a failure flag.
  - *Configurations.* When `C.enforce_alpha` is on, `C_params` is the enforced headline (written to
    `settings_recommended`) and the tables `C_ref_params`, `C_ref_runs`, `C_ref_history` are the
    unconstrained reference ("the model ignores the stall; reference only; never a recommendation").
    Figure: the two side by side with a computed ratio sentence; a stall-exposure table; the price of
    the margin from `C_alpha_curve` (`kind`, `alpha_frac`, `limit_deg`, the four variables,
    `p95_distance_m`, `p95_stop_time_s`, `mean_energy_kJ`, failures, `J`); a panel "What this result
    rests on", computed from the run's values (alpha_s is a guess, M is numerical and sets C_L,max,
    the rotor model has constant coefficients). The limit is a command-side cap, so the angle can sit
    about a degree above it; the figures say so ("the C_L peak is the stall").
  - *Settings sheet.* The `Q_TRANS_DECEL` note states the wing limit (or that it is not held), the
    stop distance and time (p95), D's `a_rec` minimum and C's excess over it, and "provisional".

### B1 · open-loop optimal control
- Tables: `B1_front`, `B1_gap`, `B1_sens`, `B1_schedule`, `B1_history` (NEW). Legacy:
  `B1_pareto_<scenario>` read as `B1_front`.
- Figures: (1) Pareto front (energy against distance, one line per variant) with the C result
  overlaid; (2) the gap B1 vs C as bars at matched distance (`B1_gap`); (3) the optimal schedule
  (throttle, theta_cmd against time; `B1_schedule`); (4) time histories as in C figure 2;
  (5) sensitivity dot plot (`B1_sens`).
- **Report fixes (branch `feat/c-b1-report-fixes`).** (a) The headline gap tiles come from every
  verified `B1_gap` row, `attached` first, `c_envelope` beside it with "verified for exact timing
  only" when `replay_late_command_ok` is false. (b) "Verified solutions by scenario": one panel per
  scenario with the verified points per variant and the scenario-matched C run of `B1_gap`
  (`c_source`) where one exists, "no verified solution" / "no scenario-matched C run" otherwise; a
  collapsible table of all `B1_front` points by scenario, variant and `reason` (new column: verified,
  not converged, replay mismatch, fails 10 ms late). (c) A computed paragraph on what the gap
  contains (pitch freedom 34.4 against `PTCH_LIM_MAX_DEG`, rotor lag, foreknowledge). (d) "Is
  `attached` attached?": `B1_front` columns `alpha_limit_deg`, `alpha_s_deg`, `wing_incidence_deg`,
  `alpha_w_clmax_deg`, `cl_max`, `cl_at_limit_over_max` and a computed paragraph on how far the limit
  sits below the lift peak. (e) Sensitivity chart with the nominal energy (same scenario, variant and
  distance cap, headline band) as a hollow marker per row and the variant in the row label; a row
  says "[nominal unverified]" or "[no nominal point]".
- **Wing-angle work (branch `fix/time-history-wing-angle`).** Variants are ordered `margin`,
  `attached`, `c_envelope` everywhere (colour series 4 for `margin`, 2 and 3 as before). (a) The gap
  tiles come from the verified `B1_gap` rows, `margin` first with "flyable bound" (a scenario where it
  has no verified point shows "no verified solution" and quotes nothing), then `attached` and
  `c_envelope` with "upper bound at the lift peak" (and the late-command warning). (b) *Gap ladder*,
  one panel per scenario: C (wing held), `margin`, `attached`, `c_envelope` at C's own distance,
  verified points only, and C without the cap as a hatched reference, from `B1_gap` columns
  `c_wing_held`, `c_unconstrained_energy_kJ`, `c_unconstrained_distance_m`; a scenario without a C run
  says so. (c) "What the gap contains" is rewritten from the rows: `margin` against C (same wing limit,
  same pitch ceiling: what is left is the rotor lag and the clairvoyance), `attached` and
  `c_envelope` against `margin` in kJ, C without the cap as a reference. (d) "Is `margin` flyable?"
  next to "Is `attached` attached?" (same columns, per variant). (e) The sensitivity caption names a
  variant that is not swept. `B1_front` has `end_balance`.
- Result box: energy at the register distance for B1 and C and the gap in % (expression shown);
  `t_rotor_on_s`, `V_over_Vs_rotor_on`, `alpha_w_peak_gated_deg`; replay-verified count.

### FT · forward transition
- Tables and figures are specified in `docs/FT_spec.md` section 6.1 and section 7 (`FT_params`, `FT_search`,
  `FT_runs`, `FT_scan`, `FT_sens`, `FT_history`). The registry entry is a normal chapter with
  `requires = [FT_params, FT_runs]`; without them the chapter is listed as "not in this run".
- **Report fixes (branch `feat/c-b1-report-fixes`).**
  - *Pitch cap (F1, defect).* `FT_history.theta_cap` is the case's own cap by mode (WAIT =
    `Q_TRAN_PIT_MAX`, FADE = (`Q_TRAN_PIT_MAX` + 1) x 2, DONE none): the history rows were built
    from the template's `ForwardParams` (the register's `Q_TRAN_PIT_MAX`), now from
    `forward.case_params`, the same construction `run_ft` flies with. `FT_runs.t_pitch_cap_s` was
    already computed from the run's own parameters. One cap line per case, in the case's colour.
  - *Time histories (F2).* Airspeed panel from 0; rule labels at the right edge, stacked, for the
    AIRSPEED_MIN of the optimum, the register and the default, and the stall rule = 1.2 x the stall
    speed of the heaviest member (also in the scan chart); each case's own fade start (square) and
    done (diamond) on its airspeed curve, the vertical lines labelled "(optimum)"; the caption states
    the scenario from `run_record.json` `context.ft_scenarios` (axes, note, the ensemble's ranges) and
    says, computed, whether it lies inside the ensemble's range on every axis, at an edge, or outside.
  - *Robustness (F3).* New table `FT_robust` (AIRSPEED_MIN = optimum + `FT.robust.airspeed_min_steps`
    at each `FT.robust.cl_max_scale`; the other three settings at the optimum; the search ensemble;
    `J`, `worst_stall_margin`, `n_flags`, the per-kind counts, `flags`); row `optimum_robust` in
    `FT_params` (the smallest flag-free-at-every-scale AIRSPEED_MIN, `robust_found`, `J`, `delta_J`,
    `delta_J_in_seed_spreads`; empty settings when none; never in `settings_recommended`); tile
    "Stall margin spare", the table and one computed sentence.
  - *Charts (F4).* AIRSPEED_MIN scan on a log axis, `FT_scan.flags` in the tooltip; Q_TRANSITION_MS
    chart hollow for `FT_grid.flags`, a square for a T-12 failure, plus a computed sentence;
    sensitivity as the absolute change of J on a symmetric log axis with failing counts and a computed
    sentence; ensemble spread with the register's members, the default's members, a beeswarm, limit
    lines and computed shares; end of the fade zoomed with the sign stated; wing share as one strip
    per case; "Final cost by seed" (`FT_search` has the four settings); the `tkoff_thr_max:hi` chip
    from `FT_params.tkoff_thr_max_hi` and `thr_max`.

### Sensitivity
- Tables: `sens_oat`, `sens_global`.
- Figures: tornado per output from `sens_oat` (same data as the existing matplotlib tornado),
  Morris mean against spread (`sens_global`).
- Table: "what would change the conclusion": inputs whose range flips a headline flag (needs
  `flips` column; if absent, omit).

### Settings sheet
- Source `settings_recommended.csv` (NEW) with columns: `parameter, value, unit, analysis, grade,
  flags, depends_on, note`. Written by the study from the analysis summaries (section 5.1); the
  viewer only displays it.

## 5. Writer-side additions (small, in `io/stages.py`, `io/assemble.py`)

These are the only changes outside `io/view/`. All are additive CSV/JSON files written through the
existing `write_results`, so the run record hashes them.

### 5.1 `settings_recommended.csv`
One row per ArduPilot parameter the study reads: `Q_BACKTRANS_MS`, `Q_A_ANGLE_MAX`, `Q_M_SPIN_MIN`,
`Q_TRANS_DECEL` (D recommendation as a row `analysis = D`; C optimum `Q_TRANS_DECEL_ms2` as a row
`analysis = C`), `AIRSPEED_MIN`, `Q_TRANSITION_MS`, `TKOFF_THR_MAX` and `Q_TRAN_PIT_MAX`. Build in `io/recommended.py`
from tables already present. A parameter whose analysis did not run keeps its register value and the row
says "not yet analysed" (the mock shows this); a default is never filled in as a recommendation.
Grade = weakest grade among the inputs the analysis lists (use `weakest_status`). The ArduPilot-default
columns are specified in `docs/DEFAULTS_spec.md` 1.4. `Q_ASSIST_SPEED` is not a study parameter
(`docs/FT_spec.md` Part 1a).

### 5.2 `headline.json`
`{ "<key>": {"value": float, "unit": str, "table": str, "column": str, "row": int|null} }` for the
numbers the proposal quotes (the existing `headline` mapping in `ReportInputs` gives the keys).
The viewer's result boxes read this file when present; otherwise compute from the tables with the
same lookups. Purpose: one traceable source of every quoted number.

### 5.3 `C_history.csv` and `B1_history.csv`
Time series for the representative runs, downsampled every 5th sample (same as the existing
`series()` in `task_b1`). Columns: `case` (`C optimum`, `B1 attached`, `B1 c_envelope`, ...),
`t, x, h, V, v_h, theta, alpha_w_deg, delta_r, theta_cmd, omega_r`. For C use the optimum member
`member = 0` unless `C_params` names another; add a `member` column. Rows are floats; no
nested JSON.

### 5.4 `inputs_register.csv`
`name, value, unit, grade, source, used_by` from the loaded inputs (the data behind
`inputs_used.md`). The viewer sorts by grade.

### 5.5 Provenance
`run_record.json` gains `schema_version: 2` and `tables: {name: {rows, columns}}`. The viewer
reads `tables` if present, else discovers by glob. Old records without it must still load.

## 6. Workbook (`tables.xlsx`)

Built from the same loaded tables (no recomputation).

1. Sheet **index** first: sheet name, source CSV, rows, columns, analysis, one-line description,
   hyperlink to the sheet. Also the run label, config hash, commit, date in the first rows.
2. One sheet per CSV table. Sheet name = file stem (truncate to 31 chars, keep unique; the index
   gives the full name).
3. Row 1 = column header exactly as in the CSV (machine identity). Row 2 = units (from
   `units.py`, empty if unknown). Data from row 3. Freeze panes below row 2, autofilter on row 1.
4. Numbers are written as numbers (never text), bools as TRUE/FALSE, `None` as an empty cell.
   Number formats by unit (kJ `0.0`, m `0.0`, deg `0.0`, others `General`).
5. Column widths from header and the first 200 rows, capped at 40.
6. DRAFT label: red text in index!A1 ("DRAFT — not for release") when applicable.
7. No formulas, no charts, no macros. The workbook is data, the HTML is the view.
8. Deterministic: fixed `docProps` (creator `sagetrans`, created = run date), sheet order stable.

## 7. Visual rules (from the dataviz review of the mock)

- Own tokens for light and dark, validated with the dataviz validator (the mock's palette passes;
  re-run if colours change). Categorical order is fixed: C = series 1, B1 attached = series 2,
  B1 wing-to-30° = series 3; the colour follows the entity, never the rank.
- Thin lines (2 px), markers 5 px minimum, 2 px surface ring on overlapping marks, grid recessive.
- Grey ink is reserved for reference series (the ArduPilot default); no entity gets it. Reference series are dashed.
- Legend whenever there are at least 2 series, plus direct labels for at most 4. Text uses text
  tokens, never the series colour.
- Status never by colour alone: DRAFT, flags, grades carry text.
- No dual axes. Time histories are separate panels with a shared time axis.
- Minimum body size 14 px; tables 13 px; tooltips 12 px.

## 8. Tests and acceptance

Unit (`tests/test_view.py`, no browser):
1. `view()` on an empty folder returns 2 and writes **nothing** (assert the folder listing is
   unchanged).
2. On a fixture folder with only `C_params.csv`, `C_search.csv`, `run_record.json`: HTML contains
   the C chapter, the "not in this run: C_q_curve" line, no exception.
3. Legacy fixture (`C_ensemble_runs.csv`, `B1_pareto_S-01.csv`): alias resolution works, the
   "older version" notice appears.
4. Hash mismatch fixture: warning chip present.
5. Every number in a result box equals the table cell it cites (parse the HTML `data-src`
   attributes; each result-box value has `data-src="Table.column[row]"`).
6. Workbook: sheet count = tables + 1, row 2 holds units, a numeric column has numeric cells,
   `index` hyperlinks resolve, two runs on the same folder give identical `sheet` contents.
7. No external URL in the HTML (`http://`/`https://` absent except inside text).
8. HTML size on the full study stays under 5 MB; downsample histories before that.

Manual (author, on a real study run):
1. Open at 1440 px and at 400 px: no horizontal page scroll; tables scroll inside their block.
2. Light and dark both readable; DRAFT banner visible.
3. Pick five numbers from the page at random, find each in the CSV/workbook.
4. Run `--view` on a folder with no results: nothing opens.

## 9. Not in this PR

- Forward-transition chapter content (separate spec).
- PDF export (the page is for screen; the matplotlib figures remain for print).
- Any filter, sort, or live recomputation.
- Settings-sheet printing layout (`settings.html`) — optional follow-up, same data.

## 10. Open items for the author

0. **ArduPilot-defaults comparison** (chosen 2026-10-27, option D in three steps): specified in
   `docs/DEFAULTS_spec.md`. The viewer parts are its section 3 (settings column, grey dashed series in the
   FT and C figures, chapter "Defaults vs optimised"); the colour rule is section 7 here (grey is reserved
   for reference series).

1. `--view` default behaviour when `report.html` already exists: overwrite (proposed) or write
   `report_<hash>.html`.
2. Whether `report.md` stays as the diff-friendly text or is generated from the same registry later.
3. Name of the settings columns shown to the lab (`depends_on`, `note`): wording only.
