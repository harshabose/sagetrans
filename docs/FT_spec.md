# Analysis FT: forward transition (hover to cruise) — implementation spec

Status: DRAFT for review. Written 2026-10-27. Needs: `docs/B2_removal_spec.md` done first (it removes the
other users of the legacy forward model). Sources: `docs/ardupilot_open_questions.md` O-14 to O-22
(`Plane-4.7.2`). Read O-15, O-16, O-17, O-18, O-20, O-21, O-22 before this spec.

Do not change code outside what this spec lists. The legacy `BaselineController`, `evaluate_full`
and `FULL_BOUNDS` stay untouched (a later clean-up removes them).

---

## 0. Decisions already taken

| # | Decision | Where it lands |
| --- | --- | --- |
| D-1 | The forward analysis is a C-style search: CMA-ES over the ArduPilot parameters, scored over a scenario ensemble. | section 5 |
| D-2 | Searched: `AIRSPEED_MIN`, `Q_TRANSITION_MS`, `TKOFF_THR_MAX`, `Q_TRAN_PIT_MAX`. Starting values are the register's; `Q_TRAN_PIT_MAX` starts at the ArduPilot default 3 deg. | 3.3, 5.2 |
| D-3 | **Equivalent airspeed (EAS)** everywhere in the forward model, including `AIRSPEED_MIN` (O-14). | 4, 5 |
| D-4 | Stall margin **1.2**: EAS never below 1.2 x the stall speed from the hand-over on. A failure, not a warning. | 5.4 |
| D-5 | Altitude excursion is part of the objective and of the failure flags. Start at 30 m AGL (`mission.profile.clearance_agl_m`); accepted loss below the start **10 m** and accepted gain above it **10 m** (new `mission.profile` fields). Both are failures beyond the limit. | 3.1, 5.4 |
| D-6 | Recovery acceleration `a_rec = min(wing limit at the repo's 0.8 C_L,max band, TECS_VERT_ACC)`. It feeds a projected-minimum-altitude flag. | 5.5 |
| D-7 | **`Q_ASSIST_SPEED` is removed from the study.** The forward model assumes assist is off (O-20). The minimum EAS of the fade is reported so any flown value can be checked. | Part 1 |
| D-8 | The results are written as tables for the viewer (docs/OUTPUTS_spec.md); no new matplotlib figures. | 7 |
| D-9 | FT also evaluates the ArduPilot-default settings as a third case (`ardupilot_default`), for comparison only (`docs/DEFAULTS_spec.md` 2.1). | 6.1 |

Needs the owner's OK before the agent does it (flagged, default as written): section 2 (Part 1b), the
repo's AIRSPEED_MIN becomes an EAS everywhere.

## 1. What the model rests on (short)

1. The forward transition has three phases. WAIT (`AIRSPEED_WAIT`): rotor hover Z loop, pitch cap
   `Q_TRAN_PIT_MAX` (3 deg; 0 while ground speed is below 3 m/s), pusher inside `[TRIM_THROTTLE or
   TKOFF_THR_MIN, TKOFF_THR_MAX]`. It ends when **EAS > `AIRSPEED_MIN`** (O-14). FADE (`TIMER`): rotor
   throttle = `last_throttle` x (T - t)/T with a floor of 0.01, no Z loop, pitch cap
   `(Q_TRAN_PIT_MAX + 1) x 2` (8 deg at the default). DONE: rotors shut down, caps lifted (O-16, O-18).
2. TECS holds the altitude with pitch and the speed with the pusher in WAIT and FADE, in its **VTOL
   stage**: pitch follows height only, throttle follows speed (the height error stays in the throttle only through the rate terms, about 0.01 per metre; corrected in O-21) (O-21). After DONE the stage is
   NORMAL (speed and height weighted equally). The structure is a line-by-line port target (O-22).
3. With `Q_ASSIST_SPEED` = 0 no trigger can restart the fade (O-20).
4. Why `Q_TRAN_PIT_MAX` is worth searching (hand estimate, linear lift region, register values
   `CL0` 0.4, `CL_alpha` 4.75, `S` 1.617 m^2, m 26.664 kg, sea-level density, level flight; the test in
   section 8 repeats it with the model): the wing carries this share of the weight at a pitch of 3 deg
   (WAIT cap) and 8 deg (FADE cap):

   | EAS | wing share at 3 deg | at 8 deg |
   | --- | --- | --- |
   | 15.9 m/s (1.2 x stall at nominal mass) | 0.62 | 1.02 |
   | 17.0 m/s | 0.71 | 1.16 |

   The weight must pass from the rotors to the wing during the fade, and the pitch cap decides whether
   the wing can take it. A short `Q_TRANSITION_MS` with a low cap sinks; this is the coupling a search
   over the fade time alone would miss.
5. The recovery acceleration that decides how far the aircraft sinks after the fade is speed
   dependent. With the 0.8 C_L,max band: a_wing = g (0.8 (V/V_s)^2 - 1), V_s = 13.27 m/s EAS at nominal
   mass. **Superseded by the implementation (2026-10-27):** the repo's wing model has C_L,max 1.444 (not
   1.5: `alpha_s` was set for 1.5 before the stall-blend sharpness `M` went from 100 to 70), so V_s = 13.52 m/s
   EAS and the three values below are 1.04, 2.60 and 4.10 m/s^2. The formula stands; the table is kept as
   first written.

   | EAS at the end of the fade | a_wing |
   | --- | --- |
   | 15.9 m/s | 1.5 m/s^2 |
   | 17.0 m/s | 3.1 m/s^2 |
   | 18.0 m/s | 4.6 m/s^2 |

   `TECS_VERT_ACC` is 7 m/s^2, so the wing is the binding limit at these speeds. (Not 4.3 m/s^2: that is
   the figure at full C_L,max with no margin.) Extra altitude lost = vdot^2 / (2 a_rec): 3 m/s of sink
   at 1.5 m/s^2 loses 3 m more.

## 2. Part 1: `Q_ASSIST_SPEED` removal (own commit, first)

Behaviour-neutral: every run so far uses 0. The only change in results is none; the settings hash
changes, so the first run recomputes everything.

Part 1a, remove the parameter:

| File | Change |
| --- | --- |
| `configs/study.yaml` | delete `D.q_assist_speed_ms` (line 46) and fix the comments on lines 45 and 91 (`max(AIRSPEED_MIN - 2, ...)` becomes `AIRSPEED_MIN - 2`) |
| `io/settings.py` | delete the field `q_assist_speed_ms` (line 151) |
| `analysis/braking.py` | `entry_airspeed(airspeed_min, rho, convention)`: drop the `q_assist_speed` argument; `v = airspeed_min - 2.0` |
| `analysis/restricted.py` | `CEval`: delete `q_assist_speed` (line 248) and its use in `_simulate_c` (line 285) |
| `io/stages.py` | callers at lines 351, 754, 922: drop the argument and the keyword |
| `io/assemble.py` | line 100 text: the exit rule is `AIRSPEED_MIN - 2` |
| `analysis/README.md`, `configs/README.md` | lines 245, 376, 577-578, 716 and 235: same rewording; add one line: "`Q_ASSIST_SPEED` is not modelled; the study assumes it is 0 (O-20)" |
| tests | none reference it today (grep `q_assist` in `tests`: no match); the existing `entry_airspeed` tests, if any, lose one argument |

Acceptance of Part 1a: every table from a run before and after is byte-identical (`results/study/*.csv`
diffed; `run_record.json` differs only in hashes and date).

Part 1b (needs the owner's OK; separate commit, results change at altitude): one register value cannot
be a true airspeed in D, C and B1 and an equivalent airspeed in FT. The source reads EAS at the AIRBRAKE exit as well (O-23), so `eas` is also the correct reading for the back transition: at 5000 m the current `tas` default starts D, C and B1 at 11.6 m/s EAS, below the 1 g stall speed.

- `configs/study.yaml`: `D.entry_convention: eas`.
- `configs/controller.yaml`: rewrite the `AIRSPEED_MIN` source text: "equivalent airspeed (O-14)";
  keep the value 17.0 for now (17.0 EAS = 1.28 x stall at nominal mass, 1.20 x at mass x1.15).
- Keep the `tas` option in the code (`entry_airspeed` still accepts it) so old runs reproduce.
- Re-run D, C, B1 and record in the PR which numbers moved (sea level: none; elsewhere the entry
  speed rises by `sqrt(rho0/rho)`).

## 3. Configuration

### 3.1 `configs/mission.yaml`, `profile:` group (two new fields, one reworded)

```yaml
  clearance_agl_m: {value: 30.0, unit: "m", source: "hover climb before the forward transition; also the height above ground at which analysis FT starts", status: TBD}
  fwd_alt_loss_max_m: {value: 10.0, unit: "m", source: "largest altitude loss below the forward-transition start that is accepted; a failure beyond it (analysis FT). Set by the owner 2026-10-27: 30 m start, 10 m clearance, so the floor is 20 m AGL", status: assumed}
  fwd_alt_gain_max_m: {value: 10.0, unit: "m", source: "largest altitude gain above the forward-transition start that is accepted; a failure beyond it (analysis FT). Symmetric with the loss limit by default", status: assumed}
```

`analysis/energy.py` `MissionProfile`: add `fwd_alt_loss_max_m: float = 10.0` and
`fwd_alt_gain_max_m: float = 10.0` (the loader in `io/inputs.py` reads every dataclass field of
`MissionProfile` from `mission.profile.<name>`, line 190 and 303, so nothing else is needed there;
check that the "unknown key" check passes). E does not read them.

### 3.2 `configs/controller.yaml`, `ardupilot:` group (new entries; values are ArduPilot defaults unless said)

```yaml
  Q_TRAN_PIT_MAX: {value: 3.0, unit: "deg", source: "(DEFAULT ARDU VALUE) pitch cap in the forward transition; the fade cap is (Q_TRAN_PIT_MAX + 1) x 2 (O-16)", status: TBD}
  TKOFF_THR_MIN: {value: 0.0, unit: "-", source: "(DEFAULT ARDU VALUE) 0 = use TRIM_THROTTLE as the pusher minimum in the forward transition (O-17)", status: TBD}
  TRIM_THROTTLE: {value: 0.45, unit: "-", source: "(DEFAULT ARDU VALUE) THROTTLE_CRUISE 45 percent (config.h). Set to the flown value: it is the pusher minimum during the forward transition and the TECS feed-forward", status: TBD}
  THR_MIN: {value: 0.0, unit: "-", source: "(DEFAULT ARDU VALUE)", status: TBD}
  THR_MAX: {value: 1.0, unit: "-", source: "(DEFAULT ARDU VALUE)", status: TBD}
  THR_SLEWRATE: {value: 100.0, unit: "%/s", source: "(DEFAULT ARDU VALUE)", status: TBD}
  PTCH_LIM_MIN_DEG: {value: -25.0, unit: "deg", source: "ArduPilot default as remembered; not re-read from source. Check", status: TBD}
```

and a new group (all `(DEFAULT ARDU VALUE)`, from O-19 and O-22):

```yaml
tecs:
  TECS_TIME_CONST: {value: 5.0, unit: "s", source: "(DEFAULT ARDU VALUE)", status: TBD}
  TECS_THR_DAMP:   {value: 0.5, unit: "s", source: "(DEFAULT ARDU VALUE)", status: TBD}
  TECS_INTEG_GAIN: {value: 0.3, unit: "-", source: "(DEFAULT ARDU VALUE)", status: TBD}
  TECS_VERT_ACC:   {value: 7.0, unit: "m/s^2", source: "(DEFAULT ARDU VALUE)", status: TBD}
  TECS_PTCH_DAMP:  {value: 0.3, unit: "s", source: "(DEFAULT ARDU VALUE)", status: TBD}
  TECS_CLMB_MAX:   {value: 5.0, unit: "m/s", source: "(DEFAULT ARDU VALUE)", status: TBD}
  TECS_SINK_MAX:   {value: 5.0, unit: "m/s", source: "(DEFAULT ARDU VALUE)", status: TBD}
  TECS_SINK_MIN:   {value: 2.0, unit: "m/s", source: "(DEFAULT ARDU VALUE)", status: TBD}
  TECS_PITCH_MAX:  {value: 15.0, unit: "deg", source: "(DEFAULT ARDU VALUE) 0 would use PTCH_LIM_MAX_DEG", status: TBD}
```

`AIRSPEED_CRUISE` is not a new key: derive it, the speed demand is
`cruise_stub.v_cruise x sqrt(rho(mission.profile.v_cruise_ref_amsl_m) / rho0)` (22.5 TAS at 1500 m is
20.9 m/s EAS). `io/inputs.py`: register the new names in `ARDUPILOT_PARAMS` and add a `tecs` group to the known
keys (line 201), build `ForwardParams` (section 4) from them.

### 3.3 `configs/study.yaml`, new block `FT:` and the analyses list

```yaml
analyses: [A, D, F, E, Q2, FT, C, B1, sensitivity]     # FT added after Q2

FT:                        # forward transition from hover (docs/FT_spec.md)
  # Start: mission.profile.clearance_agl_m above the site; limits mission.profile.fwd_alt_loss_max_m / fwd_alt_gain_max_m.
  ensemble_n: 32           # same Sobol sample (seed) as C, so the members compare
  ensemble_seed: 8
  headwind_ms: [0.0, 5.0]  # the aircraft turns into the wind before the transition; a tail wind would start with a negative airspeed
  seeds: [1, 2, 3, 4, 5, 6, 7, 8]
  popsize: 8
  maxiter: 20
  rotor_lag_s: 0.25        # as C
  t_end_s: 90.0            # a run that has not reached the hand-over by then is a "no_handover" failure
  t_settle_s: 10.0         # simulated after DONE, to see the recovery
  tecs_start: warm         # warm | cold: see 4.4 (open item: does TECS run in the VTOL hover?)
  stall_margin: 1.2        # EAS / stall speed from the hand-over on; failure below it
  rec_cl_frac: null        # C_L fraction for the recovery acceleration; null = A.cl_frac (0.8)
  proj_is_failure: true    # the projected minimum altitude below the loss limit counts as a failure
  weights: {h_ref: 5.0, E_ref: null, x_ref: 100.0, w_h: 1.0, w_E: 0.3, w_x: 0.0}   # E_ref null = mean energy at the start point; w_x 0: distance is not a cost here
  bounds:
    airspeed_min_eas_ms: [14.0, 20.0]   # below the cruise EAS (20.9); the 1.2 rule decides the lower end in practice
    q_transition_s: [2.0, 12.0]         # was 2 to 8 s; widened after the full run put the optimum at 8 s
    tkoff_thr_max: [0.5, 1.0]           # TECS forces a 1 % range above the minimum (O-22)
    q_tran_pit_max_deg: [1.0, 10.0]     # ArduPilot range 0 to 30; the fade cap is 4 to 22 deg
  scan:                    # FT_scan: AIRSPEED_MIN fixed on a grid, the other three re-searched
    airspeed_min_eas_ms: [15.0, 16.0, 17.0, 18.0, 19.0, 20.0]
    seeds: [1, 2, 3]
    maxiter: 12
  history_scenarios: [S-07, S-03]      # time histories at the optimum and at the register values
  sens:                    # FT_sens: one at a time at the optimum
    TECS_TIME_CONST: [3.0, 5.0, 8.0]
    TECS_INTEG_GAIN: [0.15, 0.3, 0.6]
    rotor_lag_s: [0.0, 0.25, 0.5]
    z_vel_p: [2.5, 5.0, 10.0]
    z_acc_p: [0.015, 0.03, 0.06]
    TRIM_THROTTLE: [0.35, 0.45, 0.55]
    tecs_start: [warm, cold]
    rec_cl_frac: [0.7, 0.8, 0.9]       # changes only the projection
```

`io/settings.py`: pydantic `FTCfg(_Strict)` with sub-models `FTWeightsCfg`, `FTBoundsCfg`, `FTScanCfg`,
`FTSensCfg` following `CCfg` (lines 210-290); `StudyConfig.FT: FTCfg`; `ALL_ANALYSES` gains `"FT"`.
Bounds validation: `lo < hi` for every pair; `tkoff_thr_max.lo > 0`.

Rationale of the bounds: `AIRSPEED_MIN` upper end 20 stays under the cruise EAS so TECS has a speed to
climb to; `Q_TRANSITION_MS` 2 to 12 s (the old `FULL_BOUNDS` range was 2 to 8 s; the first full run put the optimum at the 8 s bound, so the upper end was widened to 12 s, the owner's decision of 2026-10-27); `Q_TRAN_PIT_MAX` 1 to 10 deg gives
a fade cap of 4 to 22 deg, which was below the 35 deg of `PTCH_LIM_MAX_DEG` (20 deg since 2026-10-28; the fade cap does not read it) and inside the wing band checked by
the angle-of-attack flag. `at_bound` (the existing 10 % rule) names any variable that lands at a bound.

## 4. Controller model: `control/forward.py` (new file; the control layer imports no analysis)

### 4.1 Refactor first (no behaviour change)

`control/ardupilot_like.py`, `Position1Controller._z_loop(dt, h, vh, theta, prev)` computes the climb
target itself (`v_t = clamp(z_pos_p (h_tgt - h), +-z_vmax)`). Split it:

```python
def z_loop(self, dt, v_target, vh, theta, prev) -> tuple[float, dict[str, float]]   # public: the old body from "a_meas" on
```

and make the old `_z_loop` compute `v_t` and call `z_loop`. Every existing test and every existing
result must be unchanged (compare C and B1 tables before and after).

### 4.2 `ForwardParams` (frozen dataclass)

`back: BackTransitionParams` (for the Z-loop gains, `delta_min`, `delta_max`, `rotor_lag_s`),
`airspeed_min_eas`, `q_transition_s`, `tkoff_thr_max`, `tran_pit_max_deg`, `tkoff_thr_min`,
`trim_throttle`, `thr_min`, `thr_max`, `thr_slewrate` (per second), `ptch_lim_min_deg`, the nine TECS
values, `airspeed_cruise_eas`, `h_start` (the altitude to hold), `tecs_start`, `level_transition`
(False; the `Q_OPTIONS` LEVEL_TRANSITION bit, which caps the climb demand at 0; not searched).

### 4.3 `ForwardController` (implements `Controller`; modes `fwd_wait`, `fwd_fade`, `fwd_done`)

Per step, with `eas = aux["V"] x sqrt(aux["rho"] / rho0)`:

1. **Mode logic.** `fwd_wait` to `fwd_fade` the first step with `eas > airspeed_min_eas` (O-14).
   Record `t_handover` and `last_throttle` = the rotor command of the previous step. `fwd_fade` to
   `fwd_done` when `t - t_enter >= q_transition_s`. Expose `t_handover` and `t_done` as attributes
   (the `settled` event reads `t_done`, section 5.2).
2. **Rotors.** WAIT: `z_loop` with the climb demand of O-18: `v_dem = (h_start - h) / 10 s`, ramped in
   linearly over the first 2 s, clamped to `+-z_vmax`; capped at 0 above when `level_transition`.
   The integrator starts at the hover thrust (`z_i0 = W / T(delta_max)`, as C). FADE:
   `delta_r = last_throttle x max(0.01, (T - t_fade) / T)`; the Z loop is not run. DONE: `delta_r = 0`.
   The ArduPilot "throttle mix 0.5 x scale" detail is not modelled (a point mass has no differential
   thrust).
3. **TECS-lite** (4.4) gives `pitch_dem` (output `theta_cmd`, rad) and `throttle_dem` (output
   `delta_p`).
4. **Limits by mode** (re-evaluated every step, O-16 and O-17):

   | mode | pitch limits (deg) | throttle limits |
   | --- | --- | --- |
   | `fwd_wait` | `+-cap`, `cap = 0` while ground speed < 3 m/s else `Q_TRAN_PIT_MAX`; also within `TECS_PITCH_MAX` | `[max(THR_MIN, lo), TKOFF_THR_MAX]`, `lo = TKOFF_THR_MIN` if non-zero else `TRIM_THROTTLE` |
   | `fwd_fade` | `+-(Q_TRAN_PIT_MAX + 1) x 2` | same as WAIT |
   | `fwd_done` | `[PTCH_LIM_MIN_DEG, TECS_PITCH_MAX]` | `[THR_MIN, THR_MAX]` |

   If max minus min throttle is below 0.01, widen it to 0.01 as ArduPilot does (O-22). A `TKOFF_THR_MAX` of 0 means `THR_MAX` (the ArduPilot default; only the default case uses it).
5. **Stage.** WAIT and FADE use the VTOL stage, DONE the NORMAL stage (O-21).

### 4.4 `TecsLite` (a port of `AP_TECS.cpp`, `Plane-4.7.2`, not a re-derivation)

The agent reads these functions in the pinned source and ports the VTOL and NORMAL branches line by
line: `_update_speed_demand`, `_update_energies`, `_update_throttle_with_airspeed`, `_update_pitch`,
`_update_pitch_limits`, `_update_throttle_limits`, `_detect_underspeed`, `_detect_bad_descent`.
O-22 gives the equations; the source wins where they differ. Inputs per step: `dt`, true airspeed
`V = aux["V"]`, height `h`, climb rate `vh`, the pitch and throttle limits of 4.3, stage. State is carried
in `ModeState.memory` (floats only).

Ported as in the source: the speed-demand rate limit and its low-pass; `SKE_dem = 0.5 TAS_dem_adj^2`;
`SPEdot_dem = (SPE_dem - SPE_est) / tau`; the throttle law with proportional, damping, integrator and
`THR_SLEWRATE`; the pitch law with weights (VTOL: `W_ske` 0, `W_spe` 1; NORMAL: 1 and 1), integrator,
`gainInv = V g`, clamps, and the pitch-demand rate limit `TECS_VERT_ACC / V` rad/s; underspeed and bad
descent in the NORMAL stage.

Simplified (each is a listed model assumption in the notes of the report):

| Source element | Treatment |
| --- | --- |
| speed and height from sensors and the complementary filter | true states; `vel_dot` from a finite difference of `V` with a 0.1 s moving average |
| height demand chain (2-point average, rate limit, `TECS_HDEM_TCONST` lag) | the demand is constant (`h_start`), so the chain is the constant and the demanded rate is 0 |
| `_max_climb_scaler` / `_max_sink_scaler` | both 1 |
| roll compensation, land, flare, takeoff stages, glider options, synthetic airspeed, stall prevention, battery-voltage compensation | not modelled |
| speed demand | `AIRSPEED_CRUISE` as EAS, converted with the local density; limited to `[AIRSPEED_MIN, AIRSPEED_CRUISE]` |

Start state (`tecs_start`, open item in O-22): `warm` = `TAS_dem_adj` already at the demand, integrators
0, `last_throttle_dem = TRIM_THROTTLE` (TECS has run through the VTOL hover); `cold` = reset at the first
step (`TAS_dem_adj` = current TAS, integrators 0, `last_pitch_dem` = current pitch). `warm` is the default
and `cold` is a sensitivity case. Resolve the question from a flight log (TECS messages during an AUTO
VTOL hover) and set the default from it.

Calibration status: the gains are ArduPilot's defaults, the structure is the source's, nothing is
fitted to a log. Mark the chapter "TECS-lite, uncalibrated" in the report notes.

## 5. Analysis module: `analysis/forward.py` (new)

### 5.1 Scenarios

Take `ensemble.braking_ensemble(n, vp, seed, axes=EnsembleAxes(headwind_ms=FT.headwind_ms))`: the core
Sobol sample searched on, the corners reported at the optimum. The ensemble altitude is the **site**
elevation; each member is shifted to the transition altitude:

```python
s = dataclasses.replace(s, altitude_amsl_m=s.altitude_amsl_m + start_agl, ground_amsl_m=s.altitude_amsl_m)
```

with `start_agl = mission.profile.clearance_agl_m`. Named scenarios (`history_scenarios`) are shifted the
same way.

### 5.2 One run

Entry: hover at the start. `x0 = [0, h_start, 0, 0, 0, 0, trim.hover_omega(vps, rho), 0, 0]`; the
headwind gives an initial airspeed. Run with `simulate(vehicle, ForwardController, scen.environment(),
x0, t_end, events)`. Events: `battery_cutoff(V_min)`, `touchdown(ground_amsl_m)` and
`settled` = terminal, rising, `g = t - (ctl.t_done + t_settle)` while `ctl.t_done` is set and `-1` before
(the event reads the controller, so the run stops `t_settle` after DONE). Do not use
`runs.end_events` (its ground-speed event is for the back transition).

### 5.3 Metrics for the cost (`RunMetrics`)

- `excursion = max(loss, gain)` with `loss = max(0, -min(h - h_start))`, `gain = max(0, max(h - h_start))`
  over the whole run including the settle window.
- `energy` = bus energy from 0 to `t_done` (the transition cost; the settle is cruise power, not part
  of it). `distance` = ground distance from 0 to `t_done`.
- `failures` = the flags of 5.4 that fired.

### 5.4 Flags (each at most once per run; each costs the usual penalty of 10 in `ensemble_cost`)

| flag | rule |
| --- | --- |
| `no_handover` | `t_handover` not reached by `t_end_s` (and not any of the three below) |
| `stall_margin` | `min over t >= t_handover of V_eas / V_s_eas < FT.stall_margin` (1.2); `V_s_eas = sqrt(2 m g / (rho0 S C_L,max))` at the member's mass |
| `stall` | `runs.failure_flags`: wing angle above `alpha_s` for more than 0.3 s while `V >= V_s` |
| `alt_loss` | `loss > mission.fwd_alt_loss_max_m` |
| `alt_gain` | `gain > mission.fwd_alt_gain_max_m` |
| `proj_loss` | `proj_is_failure` and the projected minimum altitude (5.5) below `h_start - fwd_alt_loss_max_m`, or `a_rec <= 0.05 m/s^2` with a sink rate above 0.1 m/s |
| `underspeed` | after DONE the EAS falls below `0.9 AIRSPEED_MIN` |
| `battery_cutoff`, `touchdown` | the usual terminal events |

Report-only columns (no flag): peak wing angle against the band of analysis A (`alpha_limits`), time at the
pitch cap, time at `TKOFF_THR_MAX`, peak motor currents against `I_lim`.

### 5.5 End-of-fade state and the projected minimum altitude

Read at `t_done`: `h_end_rel`, `vh_end`, `V_eas_end`, `theta_end`, wing share `L/W`, wing angle.

```
cl_frac = FT.rec_cl_frac or A.cl_frac                       (0.8)
L_avail = 0.5 rho V_end^2 S (cl_frac C_L,max)               (V_end the true airspeed at t_done)
a_wing  = g (L_avail / W - 1)
a_rec   = min(a_wing, TECS_VERT_ACC)
proj_loss_m = vh_end^2 / (2 a_rec)   if vh_end < 0 and a_rec > 0.05 else 0 (or NaN if a_rec <= 0.05)
h_proj_rel  = h_end_rel - proj_loss_m
```

The projection is a lower bound on the loss (it assumes the pull-up starts at once; TECS builds pitch
over seconds). It does not depend on TECS-lite's gains, so it is the independent check on the observed
`loss` of the settle window. Report both, and `a_rec_binding` (`wing` or `tecs`).

### 5.6 Cost and search

`ensemble_cost` with `CostWeights(w_h, w_E, w_x, h_ref, E_ref, x_ref, penalty 10)` from `FT.weights`.
`E_ref: null` is the mean energy of the register start over the ensemble, computed once before the
search (deterministic). Weights are a policy choice like C's: print the three terms of `J` (excursion,
energy, penalty) in `FT_params` so the owner sees which dominates.

`FT_BOUNDS = Bounds(("airspeed_min", "q_transition", "tkoff_thr_max", "tran_pit_max"), ...)` from
`FT.bounds`; start = `start` dict from the register (`AIRSPEED_MIN`, `Q_TRANSITION_MS / 1000`,
`TKOFF_THR_MAX`, `Q_TRAN_PIT_MAX`). Use `cma_minimise` and `at_bound` unchanged. `optimise_ft(...)`
returns the same `RestrictedResult` shape as `optimise_c` (and `search.spread` is the T-12 figure, see 6.3).

Expected, to be confirmed by the run and not assumed: the 1.2 rule pins `AIRSPEED_MIN` near
`1.2 x V_s` of the heaviest member (about 17 m/s EAS), and the search content is the other three
variables.

### 5.7 Scan

`FT_scan`: for each `FT.scan.airspeed_min_eas_ms`, hold `AIRSPEED_MIN` fixed and re-search the other three
(scan seeds and `maxiter`). One row per grid point: the four values, `J`, the three cost terms, p95
excursion, mean energy, flag counts, `entry_eas_back_ms = AIRSPEED_MIN - 2`. Run each grid point as its own
task (parallel). Not in this PR: the back-transition cost along the scan (it needs C's optimum as an input,
which the task runner cannot pass between tasks); the column `entry_eas_back_ms` lets a later step join it.

### 5.8 Sensitivity and histories

`FT_sens`: for each key of `FT.sens`, run the ensemble at the optimum with that value and write the
summary columns (`summarise_ft`, like `summarise_c`: `J`, p95 excursion, mean energy, flag counts, worst
sink rate, worst stall margin). `FT_history`: for `history_scenarios`, the optimum and the register start,
every 5th sample (as `series()` in `task_b1`).

## 6. Study plumbing (`io/`), following `task_c`

1. `stages.py`: `task_ft` (search, reports), `task_ft_scan` (one per grid point), registered in
   `plan_tasks` and `run_task` as C is; cache key includes the FT settings, the controller register and
   `mission.profile.clearance_agl_m, fwd_alt_*`. Print per-phase timings as C does.
2. `assemble.py`: a section "FT forward transition" with the result box (optimum, register, `J`, spread,
   flag counts) and the tables; add the rows to `settings_recommended.csv` when the viewer writer exists
   (OUTPUTS_spec 5.1): `AIRSPEED_MIN` (EAS), `Q_TRANSITION_MS`, `TKOFF_THR_MAX`, `Q_TRAN_PIT_MAX`, each with
   `grade` from `weakest_status` of its inputs and `flags` = `at_bound` and the stall-margin note.
3. `gate.py`: `SEARCH_ANALYSES` gains `"FT"`; `record.search_spreads["FT"] = result.search.spread`.
4. `cli.py`: `--only FT` works through `ALL_ANALYSES`.
5. Time: measure one ensemble evaluation on the first run and report it in the PR; the budget is
   `ensemble_n x popsize x maxiter x seeds` evaluations as in C. If it exceeds C's by more than 2x, reduce
   `ensemble_n` in the config, not the model.

### 6.1 Tables written (the viewer's contract; names fixed)

| Table | One row per | Columns (beyond the obvious ids) |
| --- | --- | --- |
| `FT_params` | case (`optimum`, `register`, `ardupilot_default`) | the four values, `J`, `J_exc`, `J_E`, `J_pen`, `J_start`, `seed_spread`, `at_bound`, summary columns of `summarise_ft` |
| `FT_search` | seed | `seed`, `best_cost`, `evaluations` |
| `FT_runs` | member of each case (`case`; `set` = `core` or `corner`) | scenario axes; `t_handover_s`, `x_handover_m`, `V_handover_eas_ms`, `V_s_eas_ms`, `margin_handover`, `wing_share_handover`, `last_throttle`; `t_done_s`, `h_end_rel_m`, `vh_end_ms`, `V_end_eas_ms`, `theta_end_deg`, `wing_share_end`; `a_wing_ms2`, `a_rec_ms2`, `a_rec_binding`, `proj_loss_m`, `h_proj_rel_m`; `loss_m`, `gain_m`, `stall_margin_min`, `V_eas_min_fade_ms`; `alpha_w_peak_deg`, `t_pitch_cap_s`, `t_pusher_sat_s`; `energy_kJ`, `E_wait_kJ`, `E_fade_kJ`, `distance_m`, `a_mean_ms2`, `a_equiv_ms2`; `peak_motor_current_A`, `flags` |
| `FT_scan` | grid point | see 5.7 |
| `FT_sens` | case | `parameter`, `value`, summary columns |
| `FT_history` | sample | `scenario`, `case`, `t`, `mode`, `x`, `h_rel`, `V_eas`, `v_h`, `theta`, `theta_cap`, `theta_cmd`, `alpha_w_deg`, `delta_r`, `delta_p`, `wing_share`, `rotor_share` |

`a_mean_ms2 = vx(t_handover) / t_handover` and `a_equiv_ms2 = vx^2 / (2 x)` at the hand-over: these are
the numbers E's `a_fwd` stands for. Wiring E to read them (an `a_fwd_mode: ft`) is a follow-up; this PR
only writes them. Cross-check by eye against Q2's `a_equiv` at the same altitude, mass and SoC.

## 7. Viewer chapter (docs/OUTPUTS_spec.md section 4 reads these tables)

Chapter "FT · forward transition". Requires `FT_params`, `FT_runs`. Result box: the four recommended
values against the register; `J`; seed spread; flag counts by kind; worst sink rate and projected loss.

Figures:
1. **Time histories** from `FT_history` (panels: EAS with `AIRSPEED_MIN` and 1.2 x stall drawn as rules;
   altitude change with the +-10 m limits as a band; pitch with the cap as a stepped line; rotor throttle
   and pusher command), optimum against the register start, mode boundaries as vertical ticks.
2. **End of the fade**: for each member, sink rate against projected minimum altitude, the loss limit as a
   rule, flagged members hollow (`FT_runs`).
3. **Wing share across the hand-over**: `wing_share_handover` against `V_handover_eas_ms` per member, with
   the 3 deg and 8 deg lines of section 1 for the nominal mass.
4. **Cost against `AIRSPEED_MIN`** from `FT_scan`, with the stall-margin rule marked.
5. **Search convergence** (`FT_search`) and **one-at-a-time sensitivity** (`FT_sens`).
6. Ensemble spread strips: loss, gain, `stall_margin_min`, `energy_kJ`.

## 8. Tests and acceptance

Unit tests (`tests/unit/test_ft.py`; no search):
1. Mode machine with a scripted airspeed: WAIT until EAS exceeds `AIRSPEED_MIN`; FADE lasts
   `Q_TRANSITION_MS`; DONE after; `t_handover`, `t_done` recorded.
2. EAS comparison at 5000 m: the true airspeed at the hand-over equals `AIRSPEED_MIN / sqrt(rho / rho0)`.
3. Pitch caps: 3 deg in WAIT with ground speed 5 m/s, 0 deg below 3 m/s, 8 deg in FADE at the default;
   for `Q_TRAN_PIT_MAX = 5` the fade cap is 12 deg.
4. Pusher clamp: demand below `TRIM_THROTTLE` returns `TRIM_THROTTLE`; above `TKOFF_THR_MAX` returns
   `TKOFF_THR_MAX`; range under 0.01 is widened.
5. Fade law: `delta_r` falls linearly from `last_throttle` to 0.01 at the end; `last_throttle` is the
   last WAIT command.
6. TECS-lite: with demand equal to the state (level, at the speed demand, trim throttle) the pitch demand
   is 0 and the throttle equals `TRIM_THROTTLE` within 1 %; step in height demand gives a pitch demand
   rate-limited to `TECS_VERT_ACC / V`; VTOL stage ignores the speed error in the pitch and the height
   error in the throttle (two single-input tests).
7. `a_rec`: 15.9 m/s EAS at nominal mass gives 1.5 m/s^2 (+-3 %), 17.0 gives 3.1, 18.0 gives 4.6; the
   projection for `vh_end = -3` is `9 / (2 a_rec)`.
8. Wing share (section 1.4): at 15.9 m/s EAS, 3 deg gives 0.62 and 8 deg gives 1.02 (+-5 %) with the model's
   aero.
9. `z_loop` refactor: C and B1 tables identical before and after (compare CSVs).
10. Settings: `FTCfg` rejects `lo >= hi`; the new `mission.profile` fields load; unknown key check passes.
11. Determinism: two runs of `run_ft` on one member give identical traces.

Acceptance (author, on a real run):
1. `python run.py --only FT` completes in draft mode; every table of 6.1 exists; the record lists their
   hashes; `search_spreads["FT"]` is present.
2. The register start has a stated result in `FT_params` (its flags are a finding, not an error).
3. The optimum is not at a bound without `at_bound` naming it.
4. Pick three members and check `h_proj_rel_m` by hand from the table values.
5. The viewer chapter renders from the tables (once the viewer exists).

## 9. Order of work

1. B2 removal (own spec).
2. Part 1a (assist removal), then Part 1b after the owner's OK.
3. Section 3 configs and settings, section 4.1 refactor (results unchanged).
4. `control/forward.py`, then `analysis/forward.py`, then the stages and tables.
5. The viewer (OUTPUTS_spec) can be built before or after step 4: it tolerates the chapter being absent.

## 10. Open items (not decided here)

1. Does TECS run through the VTOL hover (default `tecs_start: warm`)? A flight log answers it.
2. The flown `TRIM_THROTTLE`, `TKOFF_THR_MIN`, `THR_MIN` and `PTCH_LIM_MIN_DEG`: the defaults are placeholders.
3. TECS-lite is not calibrated; one SITL or flight run of an AUTO forward transition (pitch, throttle,
   altitude) is the check, and a mismatch moves the numbers in section 1.
4. The height demand during a real transition is the next waypoint's altitude. FT holds `h_start`; a
   climbing first leg raises the pitch demand and changes the sink, which this model does not show.
5. Back-transition cost along the `AIRSPEED_MIN` scan (5.7).
6. E reading `a_fwd` from FT (6.1).
7. `Q_OPTIONS` LEVEL_TRANSITION is not modelled (`level_transition` stays False).
8. The weights of `J` are a policy choice; the three terms are printed so the owner can see them.

## 11. Part 4: grid search and executable values (2026-10-28)

Why. The first full runs put `Q_TRANSITION_MS` on the top of its range (8 s, then 12 s) and the eight-seed
CMA-ES of all four variables disagreed by 5.9 % (T-12 allows 1 %), so the optimum was neither interior nor
repeatable. The variables are also continuous in the search while ArduPilot takes integers: `AIRSPEED_MIN`
in 1 m/s, `Q_TRAN_PIT_MAX` in 1 deg, `TKOFF_THR_MAX` in 1 % and `Q_TRANSITION_MS` in 1 ms (500 to 30000).

Design. `Q_TRANSITION_MS` is the one variable the cost is flat or slowly rising in, so it goes on a grid and
the other three are searched at each value (a three-variable CMA-ES, as `FT_scan` does for `AIRSPEED_MIN`).

1. `FT.grid` in `study.yaml`: `q_transition_s: [2, 3, 4, 5, 6, 8, 10, 12]` (inside `FT.bounds.q_transition_s`),
   `seeds: [1, 2, 3]`, `maxiter: 12`. One task per point, `FT_grid:<Q_TRANSITION_MS in ms>`, cached on its own.
2. The fine pass. The coarse point with the lowest continuous cost gets its neighbours at `fine_offsets_s`
   (default -1, -0.5, +0.5, +1 s). A neighbour outside `FT.bounds.q_transition_s` is skipped: the range is
   not extended silently, and a winner on the bound is reported as such (`FT_grid` context in the run record).
   The reported `Q_TRANSITION_MS` is the best snapped point of all of them, in whole milliseconds.
3. The snap. After each point's continuous search, `AIRSPEED_MIN`, `Q_TRAN_PIT_MAX` and `TKOFF_THR_MAX` are
   rounded to their integer steps (inside the bounds), then a lattice descent moves to the best of the
   in-bound 3 x 3 x 3 neighbours while one is better, all on the search ensemble. The result has no better
   in-bound neighbour. `FT_grid` carries the continuous values, the cost of each stage and
   `J_snapped_minus_continuous`. Everything reported as a recommendation (the `FT_params` optimum row,
   `settings_recommended.csv`, the viewer) is the snapped value; the optimum row also carries the continuous
   values it came from.
4. T-12 per grid point, on the continuous result: `seed_spread` and `t12_ok` in `FT_grid`. The gate is not
   weakened. The run record's FT spread is the winner's; the points that fail are listed in the notes and in
   `context.ft_grid.t12_failed`. A winner that fails says so in the notes and the gate blocks the release.
5. The register start and the ArduPilot default are unchanged and are evaluated, with the winner, on the
   final `FT.final_ensemble_n` members. `ardupilot_default` values are already integers.
6. Task graph. The grid points run first; the fine pass is planned from their results (a second wave of
   `run_study`) and the report of the winner, `FT:<AIRSPEED_MIN>:<ms>:<percent>:<deg>`, from all of them (a
   third wave). The winner's values are in the task id, so they are in its cache key. The older
   four-variable search (`FT.seeds`, `FT.maxiter`) is no longer run.
7. Viewer: the FT chapter shows the snapped cost against `Q_TRANSITION_MS` with the winner marked and every
   point above the T-12 limit hollow and named.

Tests: the snapped optimum is on the lattice and its cost is not worse than any of its in-bound lattice
neighbours (re-evaluated, the in-bound subset of the 26); the fine pass skips out-of-bound neighbours; the
task ids carry the values.
