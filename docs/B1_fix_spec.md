# Analysis B1 fix specification (`collocation.py`, `stages.py`): hand-off for an implementing agent

> B2 was removed from the code on 2026-10-27. Mentions of B2 below describe the code at the time of writing.

Repo: `sagetrans` (Python, `src/sagetrans/...`). This file is self-contained. **Status: specified, not implemented.** It is written against the code that `C_fix_spec.md` produces: `braking.airbrake_entry_state(..., rotor_start=...)`, `LiftFF.thrust_for_delta`, `restricted.evaluate_c`, the shared AIRBRAKE-exit `BackTransitionParams` helper (C spec 4.2) and the `C` config block. Implement C first, or add those pieces as C spec sections 3.3, 4.1 and 4.2 describe. Branch from the tip of the C branch. The numbers below come from scratch scripts that are not in the repo (section 12); S-01 only.

## 0. Decisions already taken by the project owner (do not re-open)

1. **B1 starts at the AIRBRAKE exit, like D and C.** Same entry state (`airbrake_entry_state`), same entry speed (`entry_airspeed`: `max(AIRSPEED_MIN - 2, Q_ASSIST_SPEED)`), same `theta_fw0`. The level-cruise entry at 20.9 m/s EAS is not part of B1 any more.
2. **B1 answers one question:** what is the least bus energy for the AIRBRAKE-exit-to-hover problem, as a function of distance, under a stated wing limit? It is the energy-against-distance front to set beside C's `Q_TRANS_DECEL` curve (`C_q_curve`). It is not "the shape of the schedule" and not a strict lower bound (B-2 in `collocation.py.md`).
3. **The wing limit is a stated variant, never silent.** Two variants are run and labelled in every table: `attached` (|alpha_w| <= 0.85 alpha_s, today's constraint) and `c_envelope` (|alpha_w| <= 30 deg and pitch command <= 30 deg, the envelope C's ramp can reach: `theta_A` <= 30 deg). A run with **no** wing limit is not part of the study (section 11).
4. **Altitude band 1 m for the headline** (C holds about 0.5 to 1 m with the ArduPilot Z loop). The band is symmetric, as today. 3 m and 10 m are a sensitivity only.
5. **End state matches C.** B1's end band on the vertical speed is `C.sink_tol_ms` (1.76 m/s) instead of 0.5, plus one 0.5 m/s sensitivity row. Ground speed 1 m/s at the end, as today.
6. **Objective: energy under a distance cap, swept.** `eps_x` becomes a list, so B1 returns a front. One extra point per scenario and variant is B1 at C's own excursion and distance (the same-problem gap).
7. **B2 is not touched in this PR.** It keeps starting from the cruise-entry B1 solution through a legacy path (section 6). Migrating B2 is a later step.
8. Out of scope: tests (the owner adds them), the proposal text, B2, the VRS boundary as a constraint (B-3; reported only, section 8), the SITL check of O-13, TAS/EAS (DEC-1).

## 1. Why (short)

- **B1 and C are different problems.** B1 starts at cruise and includes the deceleration from 20.9 m/s down to the stall speed, which in ArduPilot is the AIRBRAKE leg; D and C start at the AIRBRAKE exit (15 m/s TAS with the shipped `AIRSPEED_MIN` = 17). For S-01 B1 from cruise (6.08 kJ, 83 m at 10 m of band) spends about 63 m and almost no energy before the rotors start, so its distance is not C's distance and its energy is not C's energy.
- **The 10 m band is an energy store.** B1 from the AIRBRAKE exit with a 10 m band dives 9.6 m to gain speed (15 to 17 m/s), then zoom-climbs while braking (6.26 kJ, 96 m). C holds the altitude to 0.5 m. Comparing the two says nothing about the autopilot.
- **The answer depends mostly on the wing limit.** From the AIRBRAKE exit on S-01 with a 1 m band, the same solver gives 8.92 kJ over 73 m with the 0.85 alpha_s margin, 7.75 kJ over 40 m with alpha_s allowed to 30 deg, and 7.12 kJ over 39 m with no limit (the wing at about 51 deg, a post-stall flop on the blended flat-plate model, not credible without validation). C, at the same 40 m, needs 12.68 kJ. A bound that does not say which wing limit it assumes is not interpretable.
- **`bound_check` compares against the wrong baseline** (B-1): it uses theta_A = 20 deg, T_bt = 3 s and the old altitude hold, not the register values and not C.
- **End state differs.** B1 requires |v_h| <= 0.5; C only flags a sink faster than 1.76 m/s (and ignores climb). Different end states shift the energy of the last second.

## 2. Files to change

| file | change |
| --- | --- |
| `src/sagetrans/analysis/collocation.py` | `CollocationOptions.alpha_max`; `_stall_expr`; `make_problem_exit`; `with_options` fix; `schedule_summary`; `bound_check_c`; `compare` uses `evaluate_c`; legacy notes (section 3, 4, 5) |
| `src/sagetrans/io/stages.py` | `task_b1` rewritten; `_b1_problem` split into the exit problem and a legacy cruise problem for B2 (section 6) |
| `src/sagetrans/io/settings.py` | `B1Cfg`, validator (section 7) |
| `src/sagetrans/io/assemble.py` | B1 tables and section (section 8) |
| `configs/study.yaml` | `B1:` block |
| `src/sagetrans/analysis/README.md` | B1 section (section 10) |

Not changed: `dynamics/`, `physics/`, `control/`, `freeform_opt.py`, `ensemble.py`, `restricted.py` (C's code is only called), `baseline_run`/`baseline_guess` (they stay as the initial-guess generators, section 4.3).

## 3. `collocation.py`: options and constraints

### 3.1 `CollocationOptions`

New field, default reproduces today:

```python
alpha_max: float | None = None   # rad; if set, replaces stall_margin * alpha_s as the wing-angle limit
```

`_stall_expr`: `lim = opt.alpha_max if opt.alpha_max is not None else opt.stall_margin * prob.vp.wing.alpha_s`; the rest (smooth gate that relaxes the limit below the 1g stall speed, node and collocation-point enforcement) is unchanged. `stall_constraint=False` stays available for diagnosis but `task_b1` never sets it (section 11).

The pitch-command limit is the existing `theta_cmd_max` (0.6 rad = 34.4 deg by default); the variants set it (section 7).

### 3.2 End-state band

Unchanged mechanism: `vh_band` bounds `|v_h|` at the last node. `task_b1` passes `C.sink_tol_ms` (1.76) for the headline. (C flags only sinking; B1 bounds climb too, which can only make B1 slightly less cheap than a sink-only bound. State this in the PR description.)

### 3.3 Everything else in the NLP is unchanged

Hermite-Simpson, 40 intervals refined from 20, graded mesh, slew limits (`delta_rate` 5 per s, `theta_rate` 1 rad/s), `|v_h| <= 3` at the nodes, battery voltage, 0.01 regulariser, IPOPT settings, effective-throttle controls, replay with the near-exact vehicle (T-11, 2 percent). The pusher stays at zero. There is no equivalent of C's `rotor_lag_s`; B1's rotor slew limit and rotor state play that role (caveat, section 11).

## 4. Entry state and guesses

### 4.1 `make_problem_exit`

```python
def make_problem_exit(vp, scen, base_bp, v_air, opt, controller, theta_p, rotor_start="carry") -> Problem
```

`vps = scen.vehicle_for(vp)`; `x0, _ = braking.airbrake_entry_state(vps, scen, base_bp, v_air, rotor_start=rotor_start)`; return `Problem(vps, scen, x0, opt, dict(controller), theta_p)`. `v_air = braking.entry_airspeed(inp.airspeed_min, D.q_assist_speed_ms, scen.rho, D.entry_convention)` per scenario; `base_bp` is the shared helper's output (C spec 4.2) with C's `rotor_lag_s` irrelevant here. `rotor_start` comes from `C.rotor_start`. All nine states are fixed to `x0` including the rotor speed (`prespin_max` stays 0).

If `v_ground <= opt.v_f` raise the same `ValueError` as `airbrake_entry_state`. Record per scenario whether `v_air < stall speed` (`below_vs`): at 5000 m the exit speed 15 m/s is below the 1g stall speed (17.4 m/s, S-02), where the wing limit is relaxed by the smooth gate; the row carries the flag.

### 4.2 `with_options` bug

`with_options` rebuilds the problem with `make_problem` from the ground speed, i.e. it always returns a **cruise** problem. Replace the body by `Problem(prob.vp, prob.scen, prob.x0, replace(prob.opt, **kw), prob.controller, prob.theta_p)`. For a cruise problem this gives the same `x0`, so nothing changes for B2.

### 4.3 Initial guesses

`baseline_run`, `baseline_guess` and the three `(theta_A, T_bt)` starts of `solve_verified` stay exactly as they are: they only generate guesses, they call the legacy controller from `prob.x0`, and they ran from the AIRBRAKE-exit state without change in the scratch runs (3 of 3 starts converged for the matched variant). Add to their docstrings that they are guess generators, not a baseline for comparison (B-1).

## 5. Comparison with C

### 5.1 `bound_check_c` (replaces `bound_check` in the stage)

```python
def bound_check_c(prob: Problem, c: RunMetrics, margin=1.02) -> BoundCheck
```

`c` is C's `evaluate_c` result for the same scenario at the C optimum (from the C stage if present, else at the register parameters with `q = 2.0`). Solve B1 with `eps_h = max(c.excursion * margin, 0.3)` and `eps_x = c.distance * margin`, objective energy, using the three starts of `solve_verified`; `energy_margin = (c.energy - sol.energy) / c.energy`. The old `bound_check` stays as legacy (docstring: its baseline is the 20 deg / 3 s guess controller, not C).

### 5.2 `compare`

Use `restricted.evaluate_c` for the C row (signature as in C spec section 5) and keep the B2 branch as it is (`freeform_opt.evaluate` at the cruise entry; the docstring says B2 is not on the AIRBRAKE-exit entry yet, so the B2 row is not comparable). Do not call `restricted.evaluate_back` from B1 any more.

## 6. `stages.py`

### 6.1 `task_b1(inp, scen_name)` (rewritten)

For the scenario `scen_name` (with `inp.scenarios`, mass factor 1, no ensemble):

1. Build the problem with `make_problem_exit` (section 4.1).
2. For each variant `v` in `B1.variants`: options = `CollocationOptions(n=intervals, max_iter=max_iter, vh_band=end_vh_band, theta_cmd_max=rad(v.theta_cmd_max_deg), stall_margin=v.alpha_margin or default, alpha_max=rad(v.alpha_max_deg) if given)`.
3. **Front:** for each `eps_h` in `B1.eps_h_m` (headline: [1.0]) and each `eps_x` in `B1.eps_x_m`: `solve_verified(prob, eh, ex, "energy", ns=refine_from)`. A non-converged or non-verified point is **kept in the table** with `converged`/`verified` false and its energy blank; it is not an infeasibility claim (it means the solver failed from three starts).
4. **Band sensitivity:** for `B1.band_sweep` (list of `eps_h`, run at the largest `eps_x` only, variant = `band_sweep_variant`).
5. **Same-problem gap:** `bound_check_c` against C's result (variant = each of `variants`).
6. **Other sensitivities** (first scenario in the list only): `rotor_start` flipped; `end_vh_band` = 0.5; `c_envelope` with `theta_cmd_max` 34.4 deg.
7. For every verified result compute `schedule_summary` from the replay (section 8) and the reporting-only D flags (VRS ratio, alpha time above the 0.8 C_L,max band; the same `braking.braking_flags` call as `stage_d`, `gate` from the D block).

Return `{"points": Table, "gap": Table, "sens": Table, "sols": [...], "errors": [...]}`; keep the per-point `print` of progress. Exceptions per point are caught and recorded as today.

### 6.2 B2's start (legacy path)

`task_b2` currently calls `_b1_problem(inp, scen)`. Rename the current implementation `_b1_problem_cruise` and make `task_b2` call it; `task_b2` and `freeform_opt` are otherwise unchanged. B2's numbers are bit-identical to the PR #6 outputs.

## 7. Config (`B1Cfg`, `study.yaml`)

```yaml
B1:                        # open-loop least-energy back-transition from the AIRBRAKE exit
  scenarios: [S-01, S-02, S-03, S-04, S-05, S-06]
  intervals: 40
  refine_from: [20]
  max_iter: 800
  eps_h_m: [1.0]                       # headline band (C holds 0.5 to 1 m)
  eps_x_m: [40, 50, 60, 75, 100, 300]  # distance cap swept: the front
  end_vh_band_ms: null                 # null = C.sink_tol_ms
  variants:
    - {name: attached,   alpha_margin: 0.85,  theta_cmd_max_deg: 34.4}
    - {name: c_envelope, alpha_max_deg: 30.0, theta_cmd_max_deg: 30.0}
  band_sweep: {eps_h_m: [3.0, 10.0], variant: c_envelope}
  sens: {rotor_start_flip: true, end_vh_band_ms: 0.5, c_envelope_theta_cmd_deg: 34.4}
```

`eps_x_m` changes from a scalar to a list and `eps_h_m` from `[20, 10, 6, 3]` to `[1.0]`; `_Strict` rejects old configs loudly, so the validator message points at this section. Each variant has exactly one of `alpha_margin` or `alpha_max_deg`. The rotor start, the sink tolerance and the entry-speed rule are read from the `C` and `D` blocks (single source); do not duplicate them here.

Cost: 60 to 105 s per converged point (three starts each, two meshes). The shipped block is 6 scenarios x 2 variants x 6 `eps_x` = 72 front points, plus the band sweep (6 scenarios x 2), the gap (6 x 2) and the sensitivity rows: about 100 solves, 2 to 3 hours single core. If the repo has a pattern for running scenarios in parallel, use it; otherwise leave it and report the wall time.

## 8. Outputs (`assemble.py`)

- `B1_front`: scenario, variant, `eps_h`, `eps_x`, converged, verified, attempts converged (of 3), energy (kJ), excursion, distance, `t_f`, `V0_air`, `V0_ground`, `V_s`, `below_vs`, replay errors, **`J_point`** (the C cost of this point: `w_E E/E_ref + w_x x/x_ref + w_h exc/h_ref` with C's normalisers; the point with the least `J_point` per scenario and variant is marked `j_best`).
- `B1_gap`: scenario, variant, C energy/excursion/distance (at C's optimum), B1 energy at C's excursion and distance, `energy_margin`.
- `B1_sens`: the sensitivity rows (same columns as `B1_front`).
- `B1_schedule`: for the `j_best` point of the first scenario, per variant: time series of `t, x, h, V, v_h, theta, alpha_w, delta_r, theta_cmd, omega_r` from the replay (for the plot).
- `schedule_summary(replay)` (new, in `collocation.py`): first time the effective rotor throttle exceeds idle + 0.05 (`t_rotor_on`) and the airspeed then, `V_over_Vs` at that time, peak wing angle (samples with `V` >= the gate), peak pitch, peak rotor throttle, end `v_h`. These columns go into `B1_front`.
- Reporting-only: D's VRS ratio and alpha-above-band time on the replay. They do not constrain the solve (B-3 stays open).

## 9. Interfaces

- **C and B1 share one entry state and one end tolerance.** Do not diverge: if C's `rotor_start` or `sink_tol_ms` changes, B1 follows (they are read from the C block).
- **B2** keeps its cruise-entry start and its own comparison; the README says so. B-9 (B2 against C) stays open until B2 migrates.
- **E** is not affected.

## 10. README (`analysis/README.md`, B1 section)

Rewrite the B1 paragraph: entry at the AIRBRAKE exit; what the front is; the two wing-limit variants and why there is no "no limit" variant; the 1 m band and what a larger band buys; the `bound_check_c` gap and how to read it (one scenario, scenario-clairvoyant, open-loop); non-converged is not infeasible; that B2 is not on this entry yet.

## 11. Caveats to carry into the PR description

- **The wing limit decides the number.** S-01, 1 m band: 8.92 kJ at 73 m (`attached`) against 7.75 to 8.22 kJ at 40 m (`c_envelope`) against 7.12 kJ at 39 m with no limit. The no-limit solution puts the wing at about 51 deg and relies on the blended post-stall model (eqs. 6.2 to 6.5); it is a diagnostic and is not in any table.
- **`attached` cannot go below about 73 m with a 1 m band** (the solver did not converge from three starts at `eps_x` of 40 m and 57 m). That is a statement about the solver, not proven infeasibility.
- **B1 is clairvoyant and open-loop**: it knows the scenario exactly and has no disturbance. The gap to C is therefore an upper bound on what the structure of the ArduPilot controller costs.
- **No rotor lag in B1.** C uses `rotor_lag_s` = 0.25 s as a stand-in for spool dynamics; B1 has the rotor state and a slew limit of 5 per s. The gap includes that difference.
- **Pitch-command limit**: 34.4 deg in `attached` (as today) against C's 30 deg (`theta_A` bound, DEC-7). The `c_envelope` variant matches C.
- **The end band bounds climb as well as sink** (C bounds sink only).
- **VRS is only reported** (B-3), and the exit speed at 5000 m is below the 1g stall speed (the wing limit is relaxed there).
- **Single-scenario evidence**: all scratch numbers are S-01. S-02 and S-03 are expected to behave similarly but are untested in the AIRBRAKE-exit regime.
- **Mesh 40** (proposal: 100 to 200): the replay checks agreed to under 1 percent on energy and distance.
- **C's numbers in the gap come from the C optimum**; if the C stage is not run, the gap uses the register parameters at `q` = 2.0 and says so in the table.

## 12. Acceptance checks

All S-01 (sea level, nominal mass, SoC 0.8), entry 15.0 m/s TAS, rotors carrying W - L at t = 0 (`rotor_start = carry`), `theta_fw0` = 0. Tolerances: energy +-3 percent, distance +-5 percent, `t_f` +-0.3 s. Every converged point below must be replay-verified.

1. **Legacy cruise path** (`_b1_problem_cruise`, `eps_h` 10, `eps_x` 300, end band 0.5): 6.08 kJ, 83 m, `t_f` 6.09 s, 10.0 m excursion. B2's tables are bit-identical to the PR #6 outputs.
2. **Exit entry, 10 m band, `attached`**, end band 0.5: 6.26 kJ, 96 m, `t_f` 7.42 s, excursion 9.6 m.
3. **Exit entry, 1 m band, `attached`**, `eps_x` 300:

   | end band | rotor start | energy | distance | `t_f` |
   | --- | --- | --- | --- | --- |
   | 0.5 | carry | 8.92 kJ | 73 m | 6.69 s |
   | 1.76 | carry | 8.93 kJ | 73 m | 6.69 s |
   | 0.5 | idle | 8.93 kJ | 72 m | 6.66 s |
4. **`attached`, `eps_x` 40 and 57**: not converged from three starts (rows kept, energy blank). If the implementation converges, report it; do not tune the solver to match.
5. **`c_envelope`, 1 m band**: `eps_x` 40, end band 1.76: 8.23 kJ, 39 m, `t_f` 3.84 s (end band 0.5: 8.22 kJ, same distance). With the pitch limit at 34.4 deg and `eps_x` 40: 7.75 kJ, 40 m, 3.75 s; at `eps_x` 300: 7.62 kJ, 50 m, 4.52 s.
6. **Gap to C** (scratch C evaluator at the rounded optimum of the reduced search: 12.68 kJ, 0.52 m, 40 m): B1 at the same excursion and distance gives about 35 percent less energy (`c_envelope` at a 1 m band: 8.2 kJ). With the shipped C optimum the numbers differ; expect 30 to 40 percent.
7. **Wing limits hold in the replay**: `attached` peak |alpha_w| <= 0.85 alpha_s + 0.3 deg (13.6 deg) for samples with `V` above the 1g stall speed; `c_envelope` <= 30.5 deg.
8. **Schedule shape** (S-01, `attached`, 1 m band): rotor throttle at idle (0.02 to 0.04) from the start until about 2.5 to 3 s, wing at about 13.3 deg, then the rotors come in; `t_rotor_on` is reported.
9. **Entry state**: `x0` equals `airbrake_entry_state(...)` of the same inputs; S-02 has `below_vs = True`.
10. **Legacy callers**: `sensitivity.py` and the existing `collocation.compare(..., b2_params)` path run; `bound_check` still works (legacy).
11. **Config guard**: an old `B1:` block (scalar `eps_x_m`) is rejected with the message.

## 13. How the scratch numbers were produced

A probe script around `collocation.solve_verified` (three starts, `ns=(20,)`, 40 intervals) with the entry set to `airbrake_entry_state` (15.0 m/s TAS, theta_A = 30 deg, T_bt = 3 s for the state only), run on S-01 for each row above (55 to 105 s each); the `c_envelope` rows used `stall_margin` 1.91 (30 deg) and `theta_cmd_max` 0.5236 or 0.6; the no-limit row used `stall_constraint=False`. The C row used the reduced-search optimum of `C_fix_spec.md` (rounded parameters, so it sits on the overshoot boundary). If a number in section 12 is outside its tolerance and the code follows sections 3 to 7, report it rather than tune to it.
