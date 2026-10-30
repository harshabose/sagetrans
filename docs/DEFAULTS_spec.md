# ArduPilot defaults as a reference in the results — implementation spec

> B1 was removed (docs/B1_removal.md); mentions of B1 below are history.

Status: DRAFT for review. Written 2026-10-27. Chosen option: "D in two steps" (values column first, then
the simulated third case in FT and C, then the summary chapter). Reference mock: `docs/report_mock.html`
(settings table has the new column; the figures do not show the default yet).

Companions: `docs/FT_spec.md` (the forward analysis), `docs/OUTPUTS_spec.md` (the viewer),
`docs/ardupilot_open_questions.md` (the sources). Depends on B2 removal and on the viewer's table
registry; step 1 needs neither.

---

## 0. Decisions already taken

1. The comparison is a reference, never a recommendation. The default is drawn dashed and grey, labelled
   "ArduPilot default", and is never used for any other entity (colour rule, OUTPUTS_spec section 7).
2. The defaults come from a register file with sources, not from constants in code.
3. Like for like: only the parameters an analysis decides are set to their defaults. Everything else stays
   at the register. Entry conditions are not changed (section 3.2).
4. No toggle in the viewer. The default series is always drawn when its rows exist.
5. A default that the model cannot represent is reported as such, never silently replaced.

## 1. Step 1: the defaults register and the values column

### 1.1 New register file `configs/ardupilot_defaults.yaml`

Same format and status words as `controller.yaml`; group `ardupilot` (and `tecs` where a default is listed).
Every entry has `source` naming the file and version, and `status: documented`. Entries (values read from
`Plane-4.7.2` unless stated; re-check the ones marked *):

```yaml
# ArduPilot factory defaults of the parameters this study decides or assumes. Reference only (docs/DEFAULTS_spec.md).
ardupilot:
  AIRSPEED_MIN:     {value: 9.0,    unit: "m/s", source: "AIRSPEED_FBW_MIN in ArduPlane/config.h, Plane-4.6.3 (*check in 4.7.2)", status: documented}
  Q_TRANSITION_MS:  {value: 5000.0, unit: "ms",  source: "quadplane.cpp AP_GROUPINFO TRANSITION_MS, Plane-4.7.2", status: documented}
  Q_TRAN_PIT_MAX:   {value: 3.0,    unit: "deg", source: "quadplane.cpp TRAN_PIT_MAX, Plane-4.7.2", status: documented}
  TKOFF_THR_MAX:    {value: 0.0,    unit: "-",   source: "Parameters.cpp TKOFF_THR_MAX; 0 means THR_MAX is used (O-17)", status: documented}
  Q_BACKTRANS_MS:   {value: 3000.0, unit: "ms",  source: "quadplane.cpp BACKTRANS_MS, Plane-4.7.2", status: documented}
  Q_TRANS_DECEL:    {value: 2.0,    unit: "m/s^2", source: "quadplane.cpp TRANS_DECEL, Plane-4.7.2", status: documented}
  Q_BCK_PIT_LIM:    {value: 10.0,   unit: "deg", source: "quadplane.cpp BCK_PIT_LIM, Plane-4.7.2", status: documented}
  Q_M_SPIN_MIN:     {value: 0.15,   unit: "-",   source: "AP_MOTORS_SPIN_MIN_DEFAULT, libraries/AP_Motors/AP_MotorsMulticopter.h", status: documented}
  Q_A_ANGLE_MAX:    {value: 30.0,   unit: "deg", source: "Q_ANGLE_MAX 3000 cdeg (quadplane.cpp ANGLE_MAX); Q_A_ANGLE_MAX itself defaults to 0, meaning Q_ANGLE_MAX (*check)", status: documented}
  Q_ASSIST_SPEED:   {value: 0.0,    unit: "m/s", source: "quadplane.cpp ASSIST_SPEED; not modelled, listed for the table (O-20)", status: documented}
  TRIM_THROTTLE:    {value: 0.45,   unit: "-",   source: "THROTTLE_CRUISE 45 percent, ArduPlane/config.h", status: documented}
  THR_MAX:          {value: 1.0,    unit: "-",   source: "THROTTLE_MAX 100 percent, ArduPlane/config.h", status: documented}
  THR_MIN:          {value: 0.0,    unit: "-",   source: "THROTTLE_MIN 0 percent, ArduPlane/config.h", status: documented}
```

Names that are not decided by any analysis (`THR_SLEWRATE`, the `TECS_*` group, `PTCH_LIM_MIN_DEG`) are
not listed: the register values of controller.yaml already are the defaults (FT_spec 3.2).

`TKOFF_THR_MAX` 0 and `Q_A_ANGLE_MAX` 0 are "use another parameter" values; the file stores the
parameter as ArduPilot does and the loader resolves them (1.3).

### 1.2 Loading

`io/inputs.py`: add `ardupilot_defaults` to `REGISTER_FILES`; the file merges into the same `Config`
(so it is hashed into the run record and graded). Add its paths to `_known_paths()` using the same
`ARDUPILOT_PARAMS` table (names, SI conversion, `NATIVE_DIVISOR`) as `controller.ardupilot`, so ms is
converted exactly as for the register. `StudyInputs.defaults: dict[str, float]` holds the resolved SI
values by ArduPilot name. Missing file or missing entry is not an error: an analysis that needs a
default that is absent skips the third case and says so in its notes (never falls back to a made-up
value). In `release` mode the file's entries must be `documented` or better like any register entry.

### 1.3 Resolution rules (one function, `inputs.resolve_default(name)`)

- `TKOFF_THR_MAX` 0 resolves to `defaults["THR_MAX"]` (1.0).
- `Q_A_ANGLE_MAX` 0 or the 30 deg entry resolves to 30 deg.
- Units: `Q_TRANSITION_MS`, `Q_BACKTRANS_MS` to seconds as the register loader does.

### 1.4 Values column

- `settings_recommended.csv` (OUTPUTS_spec 5.1) gets the columns `default`, `default_unit`,
  `default_source` and `change_vs_default` (recommended minus default, in the parameter's unit;
  empty when either is missing). Rows: every parameter in the table, whether or not the study searched it.
- Viewer: the settings table has the column "ArduPilot default" between the parameter name and the
  register value, in muted italic numerals (as in the mock). A parameter whose default is "use
  another" shows it in words: `0 (= THR_MAX)`.
- `inputs_register.csv` (OUTPUTS_spec 5.4) lists the defaults file's entries with `group = ardupilot_defaults`.
- Workbook: the same columns in the `settings_recommended` sheet.

Acceptance of step 1: with the file present the column is filled for the seven parameters of the mock;
with the file removed the viewer renders without the column and without an error.

## 2. Step 2: the simulated third case

A case is `(name, parameter dict)`. The existing code evaluates two cases, `optimum` and `register`; it gets
a third, `ardupilot_default`, on the same ensemble, the same seeds of the ensemble (not of the search; no
search is run), the same flags and the same cost. One extra ensemble evaluation per analysis.

### 2.1 FT

Parameters of the default case (the four FT searches), taken from `resolve_default`:
`AIRSPEED_MIN` 9 m/s EAS, `Q_TRANSITION_MS` 5000, `TKOFF_THR_MAX` 0 resolved to `THR_MAX` (1.0),
`Q_TRAN_PIT_MAX` 3 deg. Every other parameter is the register's.

Expected, to be read as a finding of the run and not asserted in advance: `AIRSPEED_MIN` 9 m/s EAS is
below the 1 g stall speed of the aircraft (11.9 to 14.2 m/s EAS over the mass range), so the fade starts
while the wing cannot carry the weight, and the `stall_margin` flag fires in every member. The report
says what the default does on this airframe; it is not a defect of the model.

Tables (names from FT_spec 6.1, with the `case` column added where it was not there):

| Table | Change |
| --- | --- |
| `FT_params` | third row, `case = ardupilot_default`, same columns |
| `FT_runs` | column `case` (`optimum`, `register`, `ardupilot_default`); the default and register rows are written for the core sample and the corners as the optimum is |
| `FT_history` | the default case for each `history_scenarios` entry (the column `case` already exists) |

### 2.2 C

Parameters of the default case (the four C searches): `Q_BACKTRANS_MS` 3000, `Q_A_ANGLE_MAX` 30 deg,
`Q_M_SPIN_MIN` 0.15, `Q_TRANS_DECEL` 2.0. Entry: the register's `AIRSPEED_MIN` (not the default 9), so the
back transition starts from the same state in all three cases. State this line in the chapter notes.

In `task_c` (stages.py), next to `reg = {**start, "q": restricted.C_Q_START}` and `reg_runs = run_all(reg)`:
`dflt = {"T_bt": 3.0, "theta_A": radians(30), "spin_min": 0.15, "q": 2.0}` from `resolve_default`,
`dflt_runs = run_all(dflt)`, `s_dflt = restricted.summarise_c(dflt_runs, w)`.

Tables:

| Table | Change |
| --- | --- |
| `C_params` | third row `case = ardupilot_default`; the case label of the register row stays (`register at q = 2`) |
| `C_runs` | column `case` (`optimum`, `ardupilot_default`); the capability ceiling `a_cap_ms2` is computed for the default parameters too |
| `C_history` | the default case series (OUTPUTS_spec 5.3) |

Note that the C default differs from the existing register row only in `Q_M_SPIN_MIN` (0.15 against 0.10) with
the current controller.yaml. That is expected; the row stays so the viewer never has to guess which values
were used.

### 2.3 Not in this step

D (it recommends `Q_TRANS_DECEL`; its comparison with the default 2.0 is the values column only), B1 and E
(they hold no ArduPilot parameters of their own), A, F, Q2, sensitivity.

### 2.4 Plumbing

- Gate/record: the third case does not enter `search_spreads`. `FullRunRecord` needs no new field; the
  defaults file hash is in the config hash.
- Labelling: each `case` string is fixed text, exactly `optimum`, `register`, `ardupilot_default` (the
  viewer keys on them). The existing C label `register at q = 2` stays in the `case` column of `C_params`
  for backward compatibility; the viewer maps both.
- Time: one more ensemble evaluation per analysis, plus the capability ceiling of C at the default parameters.

## 3. Step 3: viewer (OUTPUTS_spec section 4.1)

### 3.1 In the existing figures (no toggle)

- FT time histories: three series. Optimum solid in series colour 1, register solid in series colour 2,
  ArduPilot default dashed in the grey reference ink with a direct label "ArduPilot default". Order and
  colours are fixed (OUTPUTS_spec 7).
- FT end-of-fade scatter: default members as square markers in grey (the hollow marker already means
  "flagged").
- C energy against distance: one grey square at the default's (distance, energy) from `C_params` with its
  value in the tooltip. C time histories: a dashed grey line.
- Ensemble spread strips: the default's median as a grey tick on each strip.
- Legend: always lists "ArduPilot default" when the series is drawn.

### 3.2 Chapter "Defaults vs optimised" (after the analysis chapters)

Requires: at least one `*_params` table with a row `case = ardupilot_default`. Content:

1. **Parameters**: one table, columns: parameter | ArduPilot default | register | optimum | change vs default
   (the same cells as the settings table, taken from `settings_recommended.csv`).
2. **Results per analysis** (FT, C): one row per case with `J`, mean energy [kJ], p95 excursion [m], flag
   counts by kind, worst sink rate and worst stall margin (FT) or p95 distance and max overshoot (C),
   and the change from the default as a percentage with the expression in the tooltip
   (`1 - optimum/default`).
3. **Where the default fails**: the list of flags the default case raises that the optimum does not, from
   the `flags` column (grouped counts), as plain text.
4. A fixed sentence at the top of the chapter: "The default column is the value ArduPilot ships with. It
   is not tuned for this airframe and is shown only as a reference."

### 3.3 Tolerance

No defaults file, or no `ardupilot_default` rows: the column and series are absent, the chapter is
listed as "not in this run" with the reason. Legacy results (written before step 2): the settings column
works, the figures and chapter do not, and the page says "written by an older version".

## 4. Tests and acceptance

Unit (`tests/unit/test_defaults.py`):
1. The file loads with the register loader; `AIRSPEED_MIN` 9, `Q_TRANSITION_MS` 5.0 s after conversion.
2. `resolve_default("TKOFF_THR_MAX")` is 1.0 and follows a changed `THR_MAX`; `Q_A_ANGLE_MAX` resolves to
   30 deg.
3. A missing entry skips the case and writes the note; no exception.
4. The C default case equals the register case in every parameter but `spin_min` for the shipped configs.
5. `FT_params` has three rows with the fixed case strings; `FT_runs.case` has three values.
6. The settings table writer: `change_vs_default` correct for a known pair; empty when a side is missing.

Viewer tests (OUTPUTS_spec 8 style): the column exists with and without the file; the grey series is
dashed (CSS class `ref`) and labelled; no text uses the series colour.

Manual: pick the default's `J` and flag counts from the chapter and find them in `FT_params` / `C_params`.

## 5. Order of work

1. Step 1 (file, loader, values column). Independent of everything else once the viewer's settings table
   exists; the writer part can be done at once.
2. Step 2 for C (small: one more `run_all`), then for FT (after FT_spec is implemented).
3. Step 3 with the viewer's chapter registry.

## 6. Open items

1. Re-check `AIRSPEED_MIN` 9 in `Plane-4.7.2` (it was read in 4.6.3's `config.h`) and the `Q_A_ANGLE_MAX`
   fall-back to `Q_ANGLE_MAX`.
2. controller.yaml labels `TKOFF_THR_MAX` 0.8 as "(DEFAULT ARDU VALUE)"; the default is 0 (meaning `THR_MAX`).
   Reword that source text when step 1 lands; the value 0.8 itself is the lab's register choice.
3. Whether the default case in B1's gap table (B1 against C) should show the default C: not requested.
