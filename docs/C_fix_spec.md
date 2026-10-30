# Analysis C fix specification (`restricted.py`, `ardupilot_like.py`, `braking.py`): hand-off for an implementing agent

> B1 was removed (docs/B1_removal.md); mentions of B1 below are history.

> B2 was removed from the code on 2026-10-27. Mentions of B2 below describe the code at the time of writing.

Repo: `sagetrans` (Python, `src/sagetrans/...`). This file is self-contained. **Status: specified, not implemented.** Branch from the tip of `feat/d-lag-and-curve` (PR #6) and open the PR against that branch; this work uses D's `theta_fw0`, `rotor_lag_s`, `airbrake_entry_state`, `entry_airspeed` and the D config values. Every number below was produced with scratch scripts that are not in the repo (section 13); the controller law in section 3 and the evaluation in section 5 are the authoritative definition, the scripts are not.

## 0. Decisions already taken by the project owner (do not re-open)

1. **C starts at the AIRBRAKE exit, like D.** Same entry state (`airbrake_entry_state`), same entry speed rule (`entry_airspeed`: `max(AIRSPEED_MIN - 2, Q_ASSIST_SPEED)`), t = 0 is the AIRBRAKE exit. The hover -> forward transition -> cruise stub chain and the level-cruise entry at 20.9 m/s EAS are not part of C. C has no planner simulation either.
2. **C uses the position cascade (`brake_at_limit = False`)**, because that is the ArduPilot Position1 behaviour at D's resolution. The at-limit run is used only to report the capability ceiling (section 7).
3. **`Q_TRANS_DECEL` is a searched variable** (the fourth, after `T_bt`, `theta_A`, `spin_min`). It does both jobs it does in ArduPilot: it sets the stopping point `x_tgt = V_ground^2 / (2 q)` (O-02) and the cascade's velocity-profile acceleration `a_lim = q` (O-11). A constraint (overshoot, section 5) stops the search from picking `q` above what the braking ramp can deliver.
4. **The vertical loop is ArduPilot's structure with ArduPilot's default values** (position P -> velocity P -> acceleration PID on the measured vertical acceleration; open question O-12), not the repo's `K_h`/`K_vh`/`K_i` altitude hold. The repo's hold stays available for every other caller (`z_mode = "repo"` is the dataclass default).
5. **C optimises what D could not:** altitude excursion, bus energy, distance, and (new) the overshoot of the planned stopping point and the end sink rate.
6. **No full-scope run.** `AIRSPEED_MIN`, `Q_TRANSITION_MS` and `TKOFF_THR_MAX` are no longer searched by C (Q2 and E cover part of them). The code (`evaluate_full`, `FULL_BOUNDS`, `BaselineController`) is not deleted in this PR; section 2.
7. Out of scope: B1/B2 entry-state migration, D's tables, the proposal text, tests (the owner adds them), the SITL check of O-13, pinning the ArduPilot tag (O-08).

## 0b. Proposed defaults the owner has NOT confirmed (config values, flagged in the PR description)

| Item | Proposed | Why | Where |
| --- | --- | --- | --- |
| Overshoot tolerance `os_tol_m` | 2.0 m past `x_tgt` | **binding constraint**: the optimum sits on it (section 12) | `C.os_tol_m` |
| End sink tolerance `sink_tol_ms` | 1.76 m/s | low end of F's descent cap (DEC-5 lean); D's 0.5 flagged 3 to 9 of 32 runs under lag and dominated J | `C.sink_tol_ms` |
| Normalisers `h_ref`, `E_ref`, `x_ref` | 2.0 m, 15 000 J, 100 m | the old 10 m / 5 000 J / 150 m make a sub-metre excursion irrelevant; the weights (1, 0.3, 0.5) and penalty 10 are unchanged | `C.weights` |
| Rotor lag `rotor_lag_s` | 0.25 s | D's lumped lag (D spec section 14); C uses it by default because the Z loop is fast | `C.rotor_lag_s` |
| Rotor start `rotor_start` | `idle` | O-13 (unverified): rotors are near idle at the AIRBRAKE exit; `carry` (D's assumption) is the sensitivity | `C.rotor_start` |
| Bounds | `T_bt` 1 to 6 s, `theta_A` 10 to 30 deg, `spin_min` 0.06 to 0.25, `q` 1.0 to 3.0 m/s^2 | the first three are today's `BACK_BOUNDS`; `theta_A` 30 deg is DEC-7 and is binding | `C.bounds` |
| Ensemble | today's `_ensemble(inp, 32, 8)` (same as B2) | DEC-10 open; D's `braking_ensemble` is not used here | `C.ensemble_*` |

## 1. Why (short)

- **C starts from the wrong state and is not connected to D.** It enters at level cruise (20.9 m/s EAS, 21 to 28 m/s TAS), runs a planner simulation (D's at-limit run) and then the cascade; D enters at the AIRBRAKE exit (15 m/s TAS with the shipped `AIRSPEED_MIN` = 17). The two are not comparable, and C uses the repo altitude hold, so none of D's work reaches C.
- **The two jobs of `Q_TRANS_DECEL` are split.** The planner only places `x_tgt`; the cascade then brakes along a fixed `a_lim` = 3.0 (issue C-5 in `restricted.py.md`). In ArduPilot one parameter does both (O-02, O-11).
- **The repo altitude hold is about nine times softer than ArduPilot's default vertical loop.** Equivalent velocity gain of ArduPilot's default (`VELZ_P` = 5, thrust map slope 686 N per unit throttle, 244 N vehicle): 0.18 per m/s. The repo has `K_vh` = 0.02. With the ArduPilot loop the altitude excursion is below 1 m everywhere in the tested regime, so excursion is no longer the main lever (DEC-6 largely resolves: the autopilot measures acceleration, not lift).
- **The real C trade is energy against distance, set by `q`.** At the register parameters (`T_bt` 3 s, `theta_A` 30 deg, `spin_min` 0.1) raising `q` from 1.0 to 3.0 cuts bus energy from 38.6 to 14.1 kJ and the p95 distance from 190 to 66 m, until the aircraft can no longer brake that hard and overshoots its stopping point (section 11, check 4). The pitch ramp (`T_bt`, `theta_A`) sets the ceiling; `q` below the ceiling is a free choice and `q` above it is an overshoot.
- **`q` above the capability overshoots.** The capability of the ramp depends on ground speed (the ramp is time-driven): 2.62 m/s^2 at the slowest ground speed (10 m/s), 3.54 at the fastest (check 7). One `q` for all speeds therefore overshoots at the low speeds unless the tolerance allows it.

## 2. Files to change

- `src/sagetrans/control/ardupilot_like.py`: `BackTransitionParams` fields (section 3.1); `Position1Controller` (the Z branch, section 3.2); `LiftFF` extension (section 3.3).
- `src/sagetrans/analysis/braking.py`: `airbrake_entry_state` gets one keyword (section 4.1). Nothing else.
- `src/sagetrans/analysis/restricted.py`: **new** `C_BOUNDS`, `evaluate_c`, `optimise_c`, `capability_ceiling` (sections 5, 6, 7). **Untouched, marked legacy in their docstrings:** `evaluate_back`, `plan_decel`, `_plan_decel`, `evaluate_full`, `optimise`, `BACK_BOUNDS`, `FULL_BOUNDS`, `DEFAULT_PARAMS`, `THETA_P`. `analysis/sensitivity.py` and `analysis/collocation.py` call them (`evaluate_back`, `_plan_decel`, `DEFAULT_PARAMS`, `THETA_P`); their migration is a separate step.
- `src/sagetrans/io/stages.py`: `task_c`; a small shared helper for the AIRBRAKE-exit `BackTransitionParams` (the `dataclasses.replace` chain at the top of `stage_d`, section 4.2) so C and D cannot drift. No behaviour change for D.
- `src/sagetrans/io/assemble.py`: C tables and section (section 7).
- `src/sagetrans/io/settings.py`: `CCfg` and a validator (section 8).
- `src/sagetrans/io/record.py` (`make_record`): the new options go into the record so a run can be reproduced.
- `configs/study.yaml`: `C:` block.
- `src/sagetrans/analysis/README.md`: C section (section 10).
- Not changed: `dynamics/`, `physics/`, `analysis/zero_loss.py`, `collocation.py`, `freeform_opt.py`, `ensemble.py` (`CostWeights`, `ensemble_cost`, `RunMetrics` are used as they are).

## 3. `ardupilot_like.py`: the ArduPilot vertical loop

### 3.1 `BackTransitionParams`: new fields (defaults reproduce today)

```python
z_mode: str = "repo"          # "repo" (K_h, K_vh, K_i hold) | "ardupilot" (section 3.2)
z_pos_p: float = 1.0          # Q_P_POSZ_P, 1/s
z_vel_p: float = 5.0          # Q_P_VELZ_P, (m/s^2) per (m/s)
z_acc_p: float = 0.03         # Q_P_ACCZ_P, thrust_norm per (m/s^2)
z_acc_i: float = 0.1          # Q_P_ACCZ_I, thrust_norm per (m/s^2) per s
z_acc_imax: float = 0.8       # Q_P_ACCZ_IMAX, thrust_norm
z_vmax: float = 2.5           # Q_PILOT_SPD_UP, m/s (applied symmetrically)
z_vel_flt_hz: float = 5.0     # velocity-error filter
z_acc_flt_hz: float = 10.0    # acceleration-error filter
```

The values are ArduPilot master (4.8.0-dev) Plane defaults for the QuadPlane Z loop, read on 2026-10-27 (`ardupilot_open_questions.md`, O-12). Pin the tag before relying on them (O-08). `BackTransitionParams.__post_init__` (or the controller's `__init__`) raises `ValueError` for an unknown `z_mode`; in `"ardupilot"` mode `K_h`, `K_vh`, `K_i`, `i_max`, `v_dem_max` and `delta_hover` are not read and `ff_mode` is ignored (the Z loop has no lift feed-forward; that is the point).

### 3.2 `Position1Controller`, Z branch (`z_mode == "ardupilot"`)

`Position1Controller.__init__(p, ff)` takes the `LiftFF` (now also the thrust map) as it does today; in `"ardupilot"` mode `ff` is required (`ValueError` if None). Everything outside the vertical channel (the pitch law, the cascade, the pitch limit `theta_limit`, `rotor_lag_s`) is unchanged.

State (kept in `ModeState.memory`, carried each step like `L_f`): `I` (integrator, thrust_norm), `vf` (filtered velocity error), `ef` (filtered acceleration error), `vh_prev`, `d_eff` (lagged rotor command). `t_max = ff.thrust_for_delta(p.delta_max)`.

First call (`mode is None`): `I = z_i0` and `d_eff = ff.delta_for_thrust(t0)`, where `t0` is the rotor thrust of the entry state and `z_i0 = t0 / t_max` (ArduPilot initialises the integrator to the current throttle). `t0` comes from a new helper `braking.entry_rotor_thrust(vps, scen, bp, v_air, rotor_start)` that `airbrake_entry_state` also uses (section 4.1); the caller passes `z_i0` to the constructor (`Position1Controller(p, ff, z_i0=None)`; required in `"ardupilot"` mode, `ValueError` if None). `vf = ef = 0`; `vh_prev = vh`. Output the entry command; no update (dt = 0).

Later calls, `dt = t - mode.t_last` (elapsed time, as the existing integrator does), `h`, `vh` = state, `theta` = pitch:

```
a_meas = (vh - vh_prev) / dt                       # gravity-compensated vertical acceleration
v_t    = clip(z_pos_p * (h_tgt - h), -z_vmax, z_vmax)
ka     = dt / (dt + 1/(2*pi*z_vel_flt_hz));  vf += ka * ((v_t - vh) - vf)
a_t    = z_vel_p * vf
ke     = dt / (dt + 1/(2*pi*z_acc_flt_hz));  ef += ke * ((a_t - a_meas) - ef)
I      = clip(I + z_acc_i * ef * dt, 0, max(z_acc_imax, W / t_max))
thr    = clip(I + z_acc_p * ef, 0, 1)              # thrust_norm
T      = thr * t_max / max(0.5, cos(theta))        # tilt compensation (angle boost)
delta  = clip(ff.delta_for_thrust(T), p.delta_min, p.delta_max)
if p.rotor_lag_s > 0:  d_eff += (1 - exp(-dt/rotor_lag_s)) * (delta - d_eff);  delta = d_eff
```

`vh_prev = vh` after the update. The output is `[delta, 0, theta_cmd]` as today.

### 3.3 `LiftFF` extension

`LiftFF` tabulates thrust up to 1.3 W; the Z loop needs up to the thrust at `delta_max` (about 2.06 W for scenario 0). Add a constructor keyword `t_max_factor: float = 1.3` (default unchanged, so D is bit-identical; the Z loop is built with 3.0 and 70 points) and a method `thrust_for_delta(delta)`: `np.interp(delta, delta_table, T_table)`, with `delta_table` made non-decreasing by `np.maximum.accumulate` (rotor thrust is monotone in the command; the accumulate only guards table noise). `delta_for_thrust` is unchanged.

### 3.4 Assumptions to carry (state them in the docstring)

- `thrust_norm` is taken as linear in rotor thrust (`thr = T / T(delta_max)`); ArduPilot's `MOT_THST_EXPO` curve is ignored.
- The measured acceleration is the exact vertical acceleration from the state (finite difference of `vh` over the controller step); no IMU noise, bias or delay.
- The integrator limit is `max(z_acc_imax, W / t_max)` so the hover throttle always fits (it does: W / t_max = 0.48 for scenario 0).
- Pitch tilt compensation is `1/cos(theta)` clamped at 0.5, the same clamp as `lift_aware`.
- Rotor lag is the existing lumped first-order lag applied after the throttle-to-command mapping (D spec section 14), 0.25 s in C.

## 4. Entry state

### 4.1 `braking.airbrake_entry_state(..., rotor_start="carry")`

New keyword, `"carry" | "idle"`. `"carry"` is today's behaviour (rotor thrust `max(W - L0, 0.10 W)/cos(theta_fw0)`), the default. `"idle"`: rotor thrust is `ff.thrust_for_delta(bp.delta_min)` (needs a `LiftFF` with the larger range; build it inside the helper), converted to rotor speed with `trim.rotor_speed_for_thrust`. Both branches live in the helper `entry_rotor_thrust` (section 3.2), which returns the thrust. The rest of the state is identical. `"idle"` reflects O-13: in APPROACH and AIRBRAKE ArduPilot relaxes the Z controller (`D_relax_controller(0)`), the throttle decays towards zero (time constant about 0.16 s) and the rotors sit near `spin_min` when POSITION1 starts; the target altitude is the altitude at exit. **O-13 is read from source and not checked in SITL or a log**, which is why `rotor_start` is a config value. The result is insensitive to it (idle 0.50 m against carry 0.40 m p95 excursion at `q` = 2.0, lag 0, check 9).

### 4.2 Shared base parameters

`stage_d` builds `base_bp` from `inp.controller` with `theta_fw0`, `pit_lim_mode`, `v_min_ref`, `ff_mode`, `rotor_lag_s` and the alpha settings from the D block. Extract that into one helper in `stages.py` taking the config values; C calls it with `z_mode="ardupilot"` and C's own `rotor_lag_s`. **C reads `theta_fw0_deg`, `pit_lim_mode`, `q_assist_speed_ms`, `entry_convention` from the D block** (one source: they describe the AIRBRAKE exit, not C). `v_min_ref = inp.airspeed_min`. For C: `h_tgt` = the scenario altitude, `theta_p` = `inp.theta_p` (35 deg), `delta_max` from `controller_overrides` (0.95). The cascade gains stay `K_p` = 0.5, `K_v` = 1.0 (Sage tuning values, O-11); `a_lim` is overridden by `q`.

## 5. `restricted.evaluate_c`

```python
def evaluate_c(vp, scen, prm, cfg, base_bp) -> RunMetrics
```

`prm` has `T_bt`, `theta_A` (rad), `spin_min`, `q`. Steps:

1. `vps = scen.vehicle_for(vp)`; `v_air = braking.entry_airspeed(inp.airspeed_min, q_assist, scen.rho, convention)`.
2. `bp = replace(base_bp, T_bt, theta_A, delta_min=spin_min, a_lim=q, h_tgt=scen.altitude_amsl_m, delta_hover=trim.hover_throttle(...))`; `ff = LiftFF(vps, rho, soc, t_max_factor=3.0)`.
3. `x0, v_ground = braking.airbrake_entry_state(vps, scen, bp, v_air, rotor_start=cfg.rotor_start)`; `x_tgt = v_ground**2 / (2 q)`; `bp = replace(bp, x_tgt=x_tgt, brake_at_limit=False)`.
4. `simulate(runs.vehicle_for(vps), Position1Controller(bp, ff, z_i0=...), scen.environment(), x0, 60.0, runs.end_events(vps, scen, 1.0, 100.0))`.
5. `m = runs.run_metrics(sim, vps, h0)`; flags = `m.failures` plus, only when the run stopped (`terminated_by == "ground_speed_reached"`): `"overshoot"` if `x_end - x_tgt > os_tol_m`, `"sinking"` if `vh_end < -sink_tol_ms`. Return `RunMetrics(m.excursion, m.energy, m.distance, tuple(flags))`.

The cost is `ensemble_cost(runs, CostWeights(h_ref=2.0, E_ref=15e3, x_ref=100.0))` = `w_h p95(exc)/h_ref + w_E mean(E)/E_ref + w_x p95(x)/x_ref + P * n_flags` (P = 10 per flag, so one flag in 32 runs adds 10). A run that does not stop (`not_stopped`) is already a flag. `x_end` is the distance at the end speed 1 m/s, so the aircraft normally ends about 2 m short of `x_tgt` unless it overshoots.

## 6. Search (`optimise_c`)

CMA-ES via the existing `search.cma_minimise` on `C_BOUNDS = Bounds(("T_bt", "theta_A", "spin_min", "q"), (1.0, rad(10), 0.06, 1.0), (6.0, rad(30), 0.25, 3.0))`. Start: the register (`inp.start_params`: `T_bt` 3, `theta_A` 30 deg, `spin_min` 0.1) with `q = 2.0`. `RestrictedResult` is reused with `scope="c"`. Seeds, `popsize`, `maxiter` from `CCfg`. After the search, add to the result a column `at_bound` for every variable within 10% of its range of a bound (the owner needs to see that `theta_A` at 30 deg (DEC-7) and `q` near 3.0 are against their bounds; in the scratch optimum `theta_A` is 3% and `q` 5% of the range from the bound). Cost: one ensemble evaluation takes 5 to 14 s on one core (32 runs of about 0.2 s). The shipped config (8 seeds, `popsize` 8, `maxiter` 20) is up to about 1 400 evaluations (the search caches repeated vectors), roughly 2.5 hours single-core; run the scenarios of one evaluation in parallel if the repo already has a pattern for it, otherwise leave it and report the wall time.

## 7. Outputs (`assemble.py`)

- `C_params`: the optimum (`T_bt`, `theta_A_deg`, `spin_min`, `Q_TRANS_DECEL`), `J`, `J_start`, the seed spread (T-12 gate as today), `at_bound` flags, and a second row with the register parameters at `q = 2.0` evaluated the same way. Columns: p95 excursion, mean energy (kJ), p95 distance, max overshoot, flag counts.
- `C_runs`: per ensemble member: altitude, mass, SOC, headwind, `V0_ground`, `x_tgt`, `x_end`, overshoot, energy, excursion, end sink rate, flags, **`a_cap`** (the capability ceiling) and `q / a_cap`.
- `capability_ceiling(vp, scen, prm, cfg, base_bp)`: the same run with `brake_at_limit = True` (pitch at the ramp limit throughout); `a_cap = (v_ground^2 - 1) / (2 x_end)`, the equivalent of D's `a_eq` under the same Z loop and lag. Reporting only, run once per member at the optimum (not inside the search).
- `C_q_curve`: the optimum's `T_bt`, `theta_A`, `spin_min` with `q` in `[1.0, 1.5, 2.0, 2.25, 2.5, 2.75, 3.0]`: p95 excursion, mean energy, p95 distance, max overshoot, overshoot count, `J`. This is the energy-against-distance curve, and any overshoot tolerance can be read from it.
- `C_sens`: at the optimum, one-at-a-time: `rotor_lag_s` in {0, 0.25, 0.5}; `rotor_start` in {idle, carry}; `z_vel_p` in {2.5, 5, 10}; `z_acc_p` in {0.015, 0.03, 0.06}; `theta_fw0_deg` in {0, 8}. Same columns as `C_q_curve`.
- Reporting-only extras for the optimum runs: D's `braking_flags` (alpha above the 0.8 C_L,max band, VRS ratio) with the D block's gate; they do not enter `J`.

## 8. `CCfg` and `configs/study.yaml`

```yaml
C:
  ensemble_n: 32
  ensemble_seed: 8
  seeds: [1, 2, 3, 4, 5, 6, 7, 8]
  popsize: 8
  maxiter: 20
  rotor_lag_s: 0.25          # D spec section 14 lumped lag; C default
  rotor_start: idle          # idle | carry; O-13 unverified
  os_tol_m: 2.0              # overshoot past x_tgt that counts as a failure (PROPOSED)
  sink_tol_ms: 1.76          # end sink rate that counts as sinking (PROPOSED, F's cap low end)
  weights: {h_ref: 2.0, E_ref: 15000.0, x_ref: 100.0}   # w_h 1, w_E 0.3, w_x 0.5, penalty 10 unchanged
  bounds:
    T_bt: [1.0, 6.0]
    theta_A_deg: [10.0, 30.0]
    spin_min: [0.06, 0.25]
    q: [1.0, 3.0]
  z: {pos_p: 1.0, vel_p: 5.0, acc_p: 0.03, acc_i: 0.1, acc_imax: 0.8, vmax: 2.5, vel_flt_hz: 5.0, acc_flt_hz: 10.0}
```

`a_plan_source` is removed from `CCfg`. `_Strict` forbids extra keys, so an old config fails loudly; the validator adds a message pointing at this section. `C_sens` and `C_q_curve` grids may live in the block or be constants; keep them in config.

## 9. Interfaces

- E keeps taking `a_plan` from D's recommendation; C's `q` does not feed E. C reports `q` next to the capability ceiling so the owner compares it with D's `a_rec` curve (D spec section 15) by eye.
- `analysis/sensitivity.py` and `analysis/collocation.py` keep calling the legacy `evaluate_back`. The B-to-C comparison (B-9: B2 2.341 against patched C 2.256) stays on the old entry until B1/B2 migrate to the AIRBRAKE exit; state this in the PR description and in the C section of the README.
- C's `x` is the distance from the AIRBRAKE exit to the stop. It leaves out the APPROACH/AIRBRAKE leg and the `2 * V_closing` term of the switch test (O-02), which belong to E's budget.

## 10. README (`analysis/README.md`, C section)

Rewrite the C paragraph: entry at the AIRBRAKE exit; the four variables; `q` doing both jobs; ArduPilot Z loop and where its defaults come from (O-12); the overshoot and sink flags and that their tolerances are proposals; `capability_ceiling`; what is no longer optimised (`AIRSPEED_MIN`, `Q_TRANSITION_MS`, `TKOFF_THR_MAX`); that `evaluate_back` is legacy and why it stays.

## 11. Acceptance checks

All with the config above (ensemble 32, seed 8; entry 15.0 m/s TAS for every member; lag 0.25, idle start, `os_tol` 2.0, `sink_tol` 1.76, normalisers 2 / 15 000 / 100). Tolerances: excursion +-0.05 m, energy +-0.3 kJ, p95 distance +-3 m, flag counts +-1.

1. **Bit-identical defaults.** With `z_mode="repo"`, `rotor_start="carry"`, `t_max_factor` default: the existing tests pass unchanged and D's tables are identical to the PR #6 outputs.
2. **Hover.** The Z loop started from trimmed hover (scenario 0, no braking) holds the altitude within 0.05 m over 20 s (scratch value 0.009 m).
3. **Gain equivalence (informational).** Print `z_vel_p * m / (dT/d delta)` at hover for scenario 0: 0.18 (the repo's `K_vh` is 0.02).
4. **Register parameters, `q` swept** (`T_bt` 3, `theta_A` 30 deg, `spin_min` 0.1):

   | q | J | p95 exc (m) | E (kJ) | p95 x (m) | max overshoot (m) | overshoot flags |
   | --- | --- | --- | --- | --- | --- | --- |
   | 1.0 | 2.172 | 0.90 | 38.6 | 190 | 0.6 | 0 |
   | 1.5 | 1.524 | 0.86 | 22.8 | 127 | 0.7 | 0 |
   | 2.0 | 1.289 | 0.84 | 19.5 | 96 | 0.9 | 0 |
   | 2.25 | 21.211 | 0.86 | 17.7 | 85 | 2.0 | 2 |
   | 2.5 | 121.156 | 0.89 | 16.2 | 77 | 4.2 | 12 |
   | 3.0 | 321.058 | 0.89 | 14.1 | 66 | 7.8 | 32 |

   Sink and other flags: 0 in every row.
5. **Other points** (register unless stated): `q` 2.5 with `T_bt` 1.0: J 1.139, E 16.4 kJ, p95 x 77 m, max overshoot 1.2 m, 0 flags. `q` 2.5 with `T_bt` 6.0: 32 overshoot flags (max overshoot 16.1 m). `q` 2.5 with `theta_A` 20 deg: 32 overshoot flags (10.6 m). `q` 2.0 with `spin_min` 0.06: J 1.304; with 0.25: J 1.248.
6. **Search.** Reduced scratch search (one seed, `popsize` 8, 8 generations, 435 s): `T_bt` 1.478 s, `theta_A` 29.4 deg, `spin_min` 0.119, `q` 2.896; J 1.026 (start 1.289); p95 excursion 0.80 m, mean E 14.6 kJ, p95 x 67 m, max overshoot 2.0 m, 0 flags. The full search (8 seeds, 20 generations) must reach J <= 1.04 with 0 flags and land in the same basin: `q` 2.6 to 3.0, `theta_A` >= 27 deg, `T_bt` <= 2.5 s. Do not expect the same digits. Same seeds twice give the same result.
7. **Capability at that optimum** (`capability_ceiling`): min 2.62 m/s^2 (ground speed 10.1 m/s), median 3.17, max 3.54; correlation with ground speed 0.99. `q` = 2.9 is above the minimum, which is why the overshoot tolerance binds.
8. **Flags work.** `q` = 3.0 at the register gives 32 overshoot flags (check 4); lowering `os_tol_m` to 1.0 gives at least the 2 flags that `q` = 2.25 has at 2.0 (not computed in the scratch runs; read it off `C_q_curve`).
9. **Rotor start.** At the register parameters, `q` 2.0, lag 0 and the ArduPilot Z loop, the p95 excursion is 0.50 m with `idle` and 0.40 m with `carry`; both 0 failures, minimum climb rate -1.0 and -0.9 m/s. (Scratch runs used a rotor command of `delta_min` as the lag's starting value for `carry` too; with the starting value of section 3.2 `carry` is slightly better, not worse.)
10. **Legacy callers.** `analysis/sensitivity.py` and `analysis/collocation.py` run as before; a config with `a_plan_source` is rejected with the message.
11. **Config guard.** `z_mode="ardupilot"` without a `LiftFF` raises; unknown `z_mode` or `rotor_start` raises.

## 12. Caveats to carry into the PR description

- **The overshoot tolerance decides the answer.** The optimum is on the constraint: the pitch ramp can deliver 2.62 m/s^2 at the slowest ground speed and the search picks `q` = 2.9 and accepts up to 2.0 m of overshoot there. Reduced searches (one seed, `popsize` 8, 8 generations, everything else as above) with only `os_tol_m` changed:

  | `os_tol_m` | `T_bt` (s) | `theta_A` (deg) | `spin_min` | `q` | J | E (kJ) | p95 x (m) | max overshoot (m) |
  | --- | --- | --- | --- | --- | --- | --- | --- | --- |
  | 1.0 | 2.14 | 29.3 | 0.107 | 2.10 | 1.243 | 18.7 | 91 | 1.0 |
  | 2.0 | 1.48 | 29.4 | 0.119 | 2.90 | 1.026 | 14.6 | 67 | 2.0 |
  | 5.0 | 1.80 | 26.1 | 0.156 | 2.96 | 1.028 | 14.0 | 66 | 4.9 |
  | 10.0 | 1.66 | 24.5 | 0.167 | 2.96 | 1.049 | 14.1 | 67 | 5.1 |

  Between 1 and 2 m the tolerance is worth about 22% of the energy and 26% of the distance (J 1.243 to 1.026). Above 2 m nothing more is gained: `q` is then against its upper bound of 3.0 (a larger bound might gain more; not tested), and the differences in J there are search noise. The owner decides between 1 m and 2 m (DEC-4, landing error) and whether the bound 3.0 on `q` is right (DEC-3).
- **`theta_A` is on its 30 deg bound** (DEC-7), so the optimum is a stalled-wing brake as in D (D spec section 1). Report D's alpha and VRS flags next to it; do not read the result as a recommendation to fly at 30 deg.
- **The search behind the numbers is reduced** (one seed, `popsize` 8, 8 generations). The values in checks 4 to 7 are scratch values; the owner's numbers come from the full search.
- **The Z loop is a mapping, not ArduPilot.** Defaults from master 4.8.0-dev, not a pinned tag (O-08); thrust map linear in `thrust_norm`; exact acceleration, no noise or delay; integrator start and `IMAX` units follow the source reading (O-12). `C_sens` shows the sensitivity to the gains.
- **O-13 is unverified** (idle rotors at the AIRBRAKE exit). The result is insensitive to it (check 9), but verify it in SITL or a log before the numbers go in the proposal.
- **End state is only a sink-rate check** (DEC-5 lean, 1.76 m/s); there is no landing-error criterion (DEC-4) and no check on position or heading at the stop.
- **Entry state still has placeholders:** `theta_fw0_deg` = 0 (D block), AIRSPEED_MIN read as TAS (DEC-1).
- **`AIRSPEED_MIN`, `Q_TRANSITION_MS`, `TKOFF_THR_MAX` are not optimised any more.** Q2 and E cover the forward side; nothing optimises `AIRSPEED_MIN` now. If the owner wants it back, it is a fifth variable of the same search (entry speed would then follow it), not the old full scope.
- **B and C are not comparable yet** (section 9).

## 13. How the scratch numbers were produced

Scratch scripts (not in the repo), run against the PR #6 code on the 32-member ensemble of seed 8: a subclass of `Position1Controller` with the section 3.2 law and a thrust table (`apz.py`); an `evaluate(scen, prm)` with the flags and normalisers of section 5 (`apz3.py`: check 4, check 5); a reduced CMA-ES (`apz4.py`: check 6); the at-limit capability run (`cap.py`: check 7). The thrust table there was 70 points up to 3 W, with the thrust at `delta_max` (0.95) as the full-throttle reference. If a number in section 11 is outside its tolerance and the code follows sections 3 to 5, report it rather than tune to it.
