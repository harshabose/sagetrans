# B1 removal

Status: done on `draft/remove-b1-from-config` (draft PR), 2026-10-30. B1 was the open-loop optimal
control analysis (direct collocation of the vehicle, `analysis/collocation.py`, task `B1:<scenario>`,
tables `B1_front`, `B1_gap`, `B1_sens`, `B1_schedule`, `B1_history`).

## Why

1. It feeds no recommendation. `settings_recommended` comes from C, D and FT; E reads D, Q2 and F.
   B1 only answered "how far is C from a bound", and a bound that has no rotor lag and knows the
   scenario in advance cannot be set in ArduPilot.
2. Its results were fragile. In the first full run several points had no solution or a replay that
   disagreed with the collocation (S-04 at every point, S-05 and S-06 on some), `c_envelope` did not
   survive a command 10 ms late, and three wing-limit variants (`attached`, `c_envelope`, `margin`)
   had to be explained next to each other. The time-history review (docs/time_histories.md) found the
   optimum riding the edge of the lift curve and ending in free fall; the fixes (the `margin`
   variant, `end_balance`) made the chapter longer, not clearer.
3. Its question only matters if a controller other than ArduPilot's is ever considered. B2 was
   removed on the same grounds (docs/B2_removal_spec.md).

## What was removed

* Code: `analysis/collocation.py`; `stages.task_b1`, `b1_ceiling_deg`, `wing_limit_cols`,
  `margin_limit`; the `B1` task kind, its planning and its cost; the B1 tables of `assemble.py` and
  its report section; `build_b1` and everything only it used in the viewer (the gap tiles and
  ladder, the verified-points figure, the statements, the sensitivity chart, the `hatched` bars);
  the B1 chapter in the registry and the workbook; the legacy `B1_pareto_<scenario>` alias.
* Settings: `B1` as an analysis name and the whole `B1:` block (`B1Cfg`, `B1VariantCfg`, ...). A
  `study.yaml` that still lists `B1` or has a `B1:` block fails validation with the strict-model
  error. `end_balance` and the `margin` variant went with it; `margin_alpha_frac` stays (C uses it).
* Tests: `test_m6.py`, `test_b1_replay_hold.py`, the B1 parts of `test_c_b1_report.py` (now
  `test_c_report.py`), `test_view.py`, `test_pitch_limit.py`, `test_margin_alpha_frac.py`.
* Docs: `B1_fix_spec.md`, `collocation.py.md`, the B1 sections of the analysis README, the configs
  README and OUTPUTS_spec. The milestone history in `PROJECT_REPORT.md` and the per-analysis notes
  keep their B1 mentions as history.
* The verification test T-11 (a collocation result replayed in the simulator) has nothing left to
  verify: `gate.REQUIRED_TESTS` is T-01 to T-15 without T-11, and the CI check follows it.

## What changed because of it

* C owns the scenario of its history figure: `C.history_scenario` (S-01) and `C.history_band_m`
  (1.0 m) replace `B1.scenarios[0]` and `B1.eps_h_m[0]`. The figure is titled "C on one scenario".
* The report has no B1 chapter and no B1 text; the limitations text names C and FT only.
* A full run is shorter by the six B1 tasks (about 30 minutes each, not on the critical path).

## Getting it back

Nothing is rewritten in history. The last commit with B1 is the merge of PR #17 (`73fc10f`) on
`master`: `git show 73fc10f:src/sagetrans/analysis/collocation.py`, and `git log -- docs/B1_fix_spec.md`
for the spec.
