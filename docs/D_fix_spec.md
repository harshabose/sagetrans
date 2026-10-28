# Analysis D fix specification (`braking.py`, `ardupilot_like.py`): hand-off for an implementing agent

Repo: `sagetrans` (Python, `src/sagetrans/...`). This file is self-contained. **Status: implemented** (branch `feat/analysis-d-ardupilot-braking`; section 11 checks 1 to 3 and 5 to 11 were run, check 4 within about 0.05 at 15 m/s and about 0.08 at 17 m/s with 8 and 12 deg; tests are left to the owner, as in section 0). Written 26 Oct 2026 after the owner settled the points in section 0. The reference numbers in sections 1 and 11 come from my scratch scripts with a **hand-copied** vehicle (m = 26.664 kg, S = 1.617, CL0 0.4, CL_alpha 4.75, alpha_s 0.274, CD0 0.066, k 0.042, i_w 0, CN 1.5, M 70, 1500 m, SoC 0.8, controller from `controller.yaml`: Q_A_ANGLE_MAX 30 deg, T_bt 3 s, gains K_h 0.5, K_vh 0.02, K_i 0.01, a_lim 3.0); re-baseline with `load_inputs("configs", "draft", ...)` before trusting any of them. Nothing here was run against a repo change; every "expected" number is a scratch experiment that subclasses the controller.

## 0. Decisions already taken by the project owner (do not re-open)

1. **Entry state = AIRBRAKE exit.** Rotors already spun up and carrying what the wing does not (`hold_hover(0)`, no spool dynamics), pusher off after entry. t = 0 of the run is the AIRBRAKE exit, which is also when ArduPilot starts the pitch-envelope clock (`set_last_fw_pitch()`, open question O-03).
2. **Pitch limit as ArduPilot (O-04).** The upper limit ramps from `max(last_fw_pitch, 0)` to `min(Q_A_ANGLE_MAX, PTCH_LIM_MAX_DEG)` over `Q_BACKTRANS_MS`, clock from t = 0. The initial pitch is also `last_fw_pitch`. **Default `last_fw_pitch = 0`** (a config value; where the real number comes from is open, section 12).
3. **`Q_BCK_PIT_LIM`:** a constant cap by default (as today), and a **flag** that switches to ArduPilot's airspeed-scaled form (O-05).
4. **No nose-down envelope** (out of scope).
5. **No spool dynamics and no actuator lag by default.** A first-order lag on the rotor throttle exists only as a reported sensitivity, and D's altitude excursion is labelled a lower bound.
6. **Feed-forward:** a flag `ff_mode: repo | lift_aware`; the weak altitude-hold gains stay as they are.
7. **Angle-of-attack limits:** (a) reporting and (c) rotor-load checks are always on; (b) enforcement is a flag, off by default.
8. **Entry speed from the airspeed exit rule only** (`max(AIRSPEED_MIN - 2, Q_ASSIST_SPEED)`); no closing-velocity exits, no free entry-speed axis in the headline results.
9. **Better ensemble** (section 6).
10. Out of scope: A, B1, B2 and C logic (their interfaces to D are listed in section 8), tests (the owner adds them), the proposal text, any change to `AIRSPEED_MIN` vs EAS/TAS (RC5), gusts, the spool law (O-06), the nose-down envelope.

## 1. Why (short)

Numbers are for the 17 m/s case (30 deg cap, 3 s ramp, 1500 m, nominal mass) unless stated.

- **The altitude hold ignores the wing.** `delta_hover` is a constant, so at t = 0 the rotors over-thrust by about the wing lift (99 N) and then under-thrust when the wing stalls. Repo feed-forward: altitude gain 6.9 m, end sink -5.1 m/s. With a lift-aware feed-forward and instant actuators: gain 0.65 m, loss 0.16 m, end sink -0.52 m/s, `a_eq` essentially unchanged (2.65; the repo-feed-forward runs give 2.56 to 2.70).
- **Pitch limiting is not ArduPilot's** (ramp start, clock, `Q_BCK_PIT_LIM`; D-1). Entry pitch and ramp start matter: at 15 m/s, `a_eq` rises from 2.52 (start 0 deg) to 3.13 (start 12 deg), about 0.05 m/s^2 per degree.
- **The wing is deeply stalled while braking, and nothing says so.** With altitude held (gamma about 0), alpha = theta, so the 30 deg ramp is a 30 deg wing angle of attack. Lift-aware run: alpha 15.7 deg at 2.0 s, 23 deg at 2.5 s, maximum 57 deg, above 9.1 deg for 3.5 s; lift falls from 216 N at 1 s to 76 N at 2.5 s and the rotors carry the weight. With the repo feed-forward the aircraft falls and alpha reaches 109 deg at 5 m/s (gamma -79 deg), which is a falling plate, not a flight condition. The lift fit is valid to about 10 deg; beyond the stall angle the model is a flat-plate blend (CN = 1.5), so force magnitudes there are unvalidated.
- **If alpha were held to the 0.8 C_L,max ceiling (9.1 deg), `a_eq` would be about 1.3 m/s^2** (15 m/s: 1.30, x = 86 m; 17 m/s: 1.27, x = 114 m), i.e. A's window plus a ramp. So the recommended 2.5 to 2.6 is a stalled-wing number; the enforcement flag exists to show the other side.
- **The ensemble pools unlike things.** Entry speed is a ground-speed grid (16 and 20.9) the same for every sample; 16 of 64 runs (25%) start below their own 1g stall speed; the minimum is set by the slowest entry; `D_a_eq` passes `tas(ve)` as the initial ground speed although a headwind makes the air speed `V0 + w`.
- **No end-state, stall, VRS, current or voltage checks** (D-4, D-6): the run reports a pass while sinking at 5 m/s, stalled, with a VRS ratio of -0.80 against the 0.23 onset.

## 2. Files to change

- `src/sagetrans/control/ardupilot_like.py`: `BackTransitionParams` fields; `theta_limit`; `Position1Controller` (feed-forward, alpha cap, rotor lag, memory keys). Default values keep every existing caller bit-identical.
- `src/sagetrans/analysis/braking.py`: `entry_airspeed`, `airbrake_entry_state`, `braking_flags`; `run_braking` unchanged except the one item in 4.3.
- `src/sagetrans/analysis/ensemble.py`: `braking_ensemble`, `bootstrap_percentile`. **`latin_hypercube` is not touched** (C and B2 use it).
- `src/sagetrans/io/stages.py`: `stage_d`, plus the one-line `stage_e` interface change in section 8.
- `src/sagetrans/io/settings.py`: `DCfg`, `DRecCfg`, and a validator.
- `src/sagetrans/io/inputs.py`: expose `airspeed_min` (already read at line 298) as `StudyInputs.airspeed_min`; nothing else.
- `configs/study.yaml`: `D:` block.
- `src/sagetrans/analysis/README.md`: D section (section 10).
- Not changed: `dynamics/`, `physics/`, `analysis/zero_loss.py`, `restricted.py`, `collocation.py`, `freeform_opt.py`. The only external use of the alpha band is a read of `zero_loss._alpha_band` (section 3.4).

## 3. `ardupilot_like.py`

### 3.1 `BackTransitionParams`: new fields (all defaults reproduce today)

```python
theta_fw0: float = 0.0            # rad; last FW nav pitch at AIRBRAKE exit (ramp start and initial pitch)
pit_lim_mode: str = "constant"    # "constant" | "airspeed"
v_min_ref: float | None = None    # m/s; V_min in the airspeed form; set from AIRSPEED_MIN by stage_d
ff_mode: str = "repo"             # "repo" | "lift_aware"
ff_lift_tau_s: float = 0.0        # first-order filter on the lift estimate, s
ff_lift_scale: float = 1.0        # multiplier on the lift estimate (estimation error)
alpha_cap: float | None = None    # rad; theta - gamma <= alpha_cap while V >= alpha_cap_vmin; None = off
alpha_cap_vmin: float = 5.0       # m/s; below this the wing carries under 10% of the weight at C_L,max
rotor_lag_s: float = 0.0          # sensitivity only; not ArduPilot's spool law
```

`ff_mode` stays `"repo"` as the **dataclass** default so C, B2 and every test that builds `BackTransitionParams` directly are unchanged. `stage_d` sets `"lift_aware"` from the study config (section 7). Switching C and B2 is a separate, deliberate change (RC1); do not do it here.

### 3.2 `theta_limit`

```python
def theta_limit(self, t_rel: float, eas: float | None = None) -> float
```

- `cap = min(theta_A, theta_P)`; `start = min(max(theta_fw0, 0.0), cap)`.
- `ramp = cap` if `T_bt <= 0`, else `start + (cap - start) * min(1, max(t_rel, 0) / T_bt)`.
- `pit_lim_mode == "constant"`: `ramp if theta_B is None else min(ramp, theta_B)` (as today).
- `pit_lim_mode == "airspeed"` (O-05): require `theta_B` (this is `Q_BCK_PIT_LIM`, treated as theta_B0), `v_min_ref` and `eas`, else `ValueError` (no silent fallback). If `theta_B <= 0` the limit is disabled (return `ramp`). Otherwise `lim_B = min(theta_B * (max(v_min_ref, 5.0) / max(eas, 0.1))**2, theta_A)` and return `min(ramp, lim_B)`.
- `Position1Controller.command` passes `eas = aux["V"] * sqrt(aux["rho"] / rho0)` with `rho0 = atmosphere.density(0.0)` computed once at module level. The controller sees `aux` from the previous step; that is how it works today.

### 3.3 Feed-forward (`ff_mode == "lift_aware"`)

- New small class `LiftFF` in `ardupilot_like.py`: `LiftFF(vps, rho, soc)` holding the scenario vehicle (mass applied), a throttle-for-thrust table, and `delta_for_thrust(T) -> float`. The table is built once: `T` on `np.linspace(0, 1.3 * m * g, 40)`, for each `T > 0` `om = trim.rotor_speed_for_thrust(T, 0, 0, rho, vps)` then `trim.required_rotor_command(om, 0, 0, 0, rho, vps, soc).delta_cmd` (0 at `T = 0`); `np.interp` at run time. `rho` is the scenario density at the start altitude.
- `Position1Controller.__init__(self, p, ff: LiftFF | None = None)`. If `p.ff_mode == "lift_aware"` and `ff is None`, raise `ValueError`.
- Each step: `L_hat = ff_lift_scale * aux["L"]`, filtered with a first-order lag of time constant `ff_lift_tau_s` (state in `mode.memory["L_f"]`, advanced with `dt = t - mode.t_last`, initialised to the first `L_hat`; `tau = 0` means no filter). `T_ff = max(0, m*g - L_hat) / max(0.5, cos(theta))`; `delta_hover_eff = ff.delta_for_thrust(T_ff)`; run `altitude_hold` with `replace(p, delta_hover=delta_hover_eff)`. The gains `K_h, K_vh, K_i` are untouched.
- This is an **oracle** at `tau = 0, scale = 1`: it uses the simulated lift, which the real aircraft does not measure. `ff_lift_tau_s` and `ff_lift_scale` are the way to represent an estimate (section 7 runs a sensitivity). Say so in the docstring.
- `mode.memory` is rebuilt every step (`new = ModeState(..., {"integ": integ})`); carry `L_f` and `d_eff` (3.5) through, or they reset.

### 3.4 Alpha cap (`alpha_cap is not None`)

In `command`, after `lim = p.theta_limit(...)`: if `aux["V"] >= alpha_cap_vmin`, `lim = min(lim, max(0.0, alpha_cap + aux["gamma"]))`. (alpha = theta - gamma, so theta <= alpha_cap + gamma.) It applies in both the worst-case mode (`brake_at_limit`) and the cascade. `stage_d` sets `alpha_cap` to the top of A's band: `zero_loss._alpha_band(vp, ActuatorLimits(alpha_min=..., cl_frac=study.A.cl_frac))[1]` (about 9.1 deg at 0.8; this is the body-pitch band top, so it already subtracts `i_w`). Import it (promote it to a public name if you prefer); do not repeat the C_L,max search.

### 3.5 Rotor lag (`rotor_lag_s > 0`)

First-order lag on the rotor throttle only: `d_eff += (1 - exp(-dt / tau)) * (d_cmd - d_eff)`, `dt = t - mode.t_last`, state in `mode.memory["d_eff"]` (initialised to the first command); the pusher and pitch command are unchanged. Document it as a crude stand-in for rotor/ESC lag, **not** `MOT_SPOOL_TIME` (the rotors are already spooled in AIRBRAKE).

## 4. `braking.py`

### 4.1 `entry_airspeed`

```python
def entry_airspeed(airspeed_min: float, q_assist_speed: float, rho: float,
                   convention: str = "tas") -> float
```

`v = max(airspeed_min - 2.0, q_assist_speed)`. `"tas"`: return `v` (the repo compares `AIRSPEED_MIN` as true airspeed today, per the note in `controller.yaml`). `"eas"`: return `v * sqrt(rho0 / rho)`. Returns the TAS at the entry. `Q_ASSIST_SPEED` appears nowhere in the repo; the config default is 0 (ArduPilot's default disables assist), so the rule gives `AIRSPEED_MIN - 2` = 15 m/s for `AIRSPEED_MIN` = 17.

### 4.2 `airbrake_entry_state`

```python
def airbrake_entry_state(vp, scen, bp, v_air0) -> tuple[np.ndarray, float]
```

Returns `(x0, v0_ground)`. `v0_ground = v_air0 - scen.headwind_ms` (headwind positive against the aircraft; the code uses `v_air = v0 + headwind`); raise `ValueError` if `v0_ground <= v_f`. `vps = scen.vehicle_for(vp)`; `L0 = aero.forces(scen.rho, v_air0, bp.theta_fw0 + vps.wing.i_w, vps.wing)[0]`; `T0 = max(m*g - L0, 0.10*m*g) / max(0.5, cos(theta_fw0))`; `om0 = trim.rotor_speed_for_thrust(T0, 0, 0, scen.rho, vps)`. State `[0, h, v0_ground, 0, theta_fw0, 0, om0, 0, 0]`. (The rotors in `hold_hover(0)` carry what the wing does not; the 10% floor keeps them spinning when the wing alone would carry the weight, and is an assumption, not an ArduPilot value; it is inactive for `theta_fw0` <= 8 deg at 15 m/s.)

### 4.3 `run_braking`

Unchanged signature and behaviour; `stage_d` passes the state from 4.2 as `x0` and the ground speed as `v0`. The existing `_initial_state` path (pitch 0, hover-speed rotors) stays for the regression check and for C's `plan_decel`.

## 5. `braking_flags` (reporting; the D part of RC2)

```python
def braking_flags(res, vp, scen, bp, *, alpha_hi, alpha_min, kappa, gate=5.0,
                  sink_tol=0.5) -> dict[str, float | bool]
```

All from `res` columns and `res.metrics`; `alpha_w = res["alpha"] + vps.wing.i_w`. "Gated" means samples with `res["V"] >= gate`; below the gate alpha means nothing (the wing can carry under 10% of the weight at C_L,max below about 4.6 m/s at 1500 m).

- Alpha (a): `alpha_max_deg` (ungated, so the 109 deg case is still visible), `alpha_max_gated_deg`, `t_alpha_08_first_s` and `V_at_alpha_08_first_ms` (first gated sample with `alpha_w > alpha_hi`, NaN if none), `t_above_alpha_08_s`, `t_above_alpha_s_s` (`alpha_w > vps.wing.alpha_s`), `t_below_alpha_min_s` (`alpha_w < alpha_min`), all time integrals over gated samples.
- Rotor load (c): `rotor_share_max` = max of `T_r * cos(theta) / (m g)`, `wing_share_min` = min of `L / (m g)` (gated), `throttle_margin_min` = min of `(delta_max - delta_r) / delta_max`, `t_throttle_sat_s` (`delta_r >= delta_max - 1e-3`).
- Other limits: `vrs_min`, `vrs_min_t_s`, `t_vrs_onset_s` (ratio <= `-kappa`, `kappa` = `inp.descent.kappa`; report only); `peak_motor_current_A` (`max(res["I_rotor"])`) and `current_over` (> `vps.motor_r.I_lim`); `min_bus_voltage_V` and `voltage_under` (< `vps.battery.V_min`).
- End state: `vh_end_ms`, `dh_end_m`, `stopped` (`res.terminated_by == "ground_speed_reached"`), `sinking` (`vh_end < -sink_tol`; `sink_tol` = 0.5 m/s is a placeholder, in the config).
- Do not make any flag stop a run or change `a_eq` (that is what the enforcement flag does in 3.4).

## 6. Ensemble (`ensemble.py`)

```python
def braking_ensemble(n, vp, seed, axes, corners=True) -> tuple[list[Scenario], list[Scenario]]
def bootstrap_percentile(values, q, n_boot=1000, seed=0, ci=(5, 95)) -> tuple[float, float, float]
```

- **Core sample:** scrambled Sobol (`scipy.stats.qmc.Sobol(5, scramble=True, seed=seed)`, `n` a power of two, validated in settings) over the five physical axes in `inp.ensemble_axes` (altitude, ISA offset, mass factor, headwind, SoC). Mass goes through `Scenario.mass_kg`.
- **Corners:** the 8 vertices of altitude x mass x headwind (low/high), with ISA offset and SoC at the axis midpoints, returned as a second list (they are not part of the percentile).
- **Entry speed is not an axis.** For every scenario the entry air speed is `entry_airspeed(airspeed_min, q_assist, scen.rho, convention)` and the ground speed is that minus the headwind. This is what changes the answer (below).
- **Per-run columns:** altitude, ISA, mass, SoC, headwind, `V0_air_ms`, `V0_ground_ms`, `Vs_ms` (1g stall speed at the scenario's mass and density, `aero.stall_speed`), `below_vs` (`V0_air < Vs`; **flagged, not dropped**), `a_eq`, the flags of section 5.
- **Summary row** (`D_recommendation`): `n`, `seed`, `n_corners`, `convention`, `a_rec_min` (min over Sobol and corners; this is what `out["a_rec"]` carries, so E is unchanged), `a_rec_pct` (5th percentile over the Sobol sample only; the headline), its bootstrap 90% interval, `a_rec_pct_half` (same percentile over the first `n/2` points, a convergence check), `n_below_vs`, `n_nan`, the scenario that sets the minimum, the worst corner, and the Spearman correlation of `a_eq` with each axis.
- **Expected (hand-copied vehicle, lift-aware, `theta_fw0` = 0, instant actuators):** Sobol 32 plus 8 corners, entry rule 15 m/s TAS: Sobol min 2.05, 5th percentile 2.10 (bootstrap 2.05 to 2.17), median 2.52, max 2.90; 17 of 32 start below `V_s`; corner min 2.00 (altitude 0, mass x0.80, headwind +5). With the repo feed-forward: min 2.15, 5th percentile 2.21, and mean |end sink| 3.5 m/s against 0.2. With the `"eas"` convention (15 m/s EAS, TAS rises with altitude): min 2.15, 5th percentile 2.19, median 2.67, none below `V_s`, corner min 2.01. For comparison, today's ensemble: `a_rec` 2.508, `a_rec_pct` 2.521.
- **Why it moved:** the headwind effect flips sign. Today the ground speed is fixed, so a headwind adds drag and helps (Spearman +0.49). With the entry fixed in air speed, +5 m/s of headwind gives a 10 m/s ground speed, a shorter run in which the fixed 3 s ramp is a larger share, so `a_eq` falls (Spearman -1.0 in the preview). The lowest runs are the low-ground-speed ones.

## 7. `stage_d` and config

`DCfg` / `DRecCfg` additions (all in `settings.py`, `extra="forbid"` as elsewhere; update `study.yaml` in the same change):

```python
class DCfg:
    ff_mode: Literal["repo", "lift_aware"] = "lift_aware"
    ff_lift_tau_s: float = 0.0
    ff_lift_scale: float = 1.0
    theta_fw0_deg: float = 0.0
    pit_lim_mode: Literal["constant", "airspeed"] = "constant"
    q_assist_speed_ms: float = 0.0
    entry_convention: Literal["tas", "eas"] = "tas"
    enforce_alpha: bool = False
    alpha_gate_ms: float = 5.0
    sink_tol_ms: float = 0.5
    rotor_lag_s: float = 0.0
    v0_eas_ms: list[float] = [16, 17.1, 20.9]     # kept: diagnostic speed grid only (below)
    sens: DSensCfg = DSensCfg()                   # theta_fw0_deg_sweep [0,4,8,12], lag_s_sweep [0,0.25,0.5], lift_tau_s_sweep [0,0.5], lift_scale_sweep [0.8,1.2]
class DRecCfg:
    ensemble_n: int = 32          # power of two (validator)
    ensemble_seed: int = 8
    percentile: float = 5.0
    corners: bool = True
    n_boot: int = 1000
    # v0_ground_ms is removed: entry speed comes from the airspeed exit rule
```

Build `bp` for D with `dataclasses.replace(inp.controller, theta_fw0=radians(c.theta_fw0_deg), pit_lim_mode=..., v_min_ref=inp.airspeed_min, ff_mode=c.ff_mode, ff_lift_tau_s=..., ff_lift_scale=..., rotor_lag_s=..., alpha_cap=alpha_hi if c.enforce_alpha else None, alpha_cap_vmin=c.alpha_gate_ms)`. `Q_BCK_PIT_LIM` stays `inp.controller.theta_B`.

Tables:
- `D_a_eq`: named scenario x `mass_factors`, entry from the rule (not a speed grid). Columns kept; `V0_eas_ms` and `V0_ground_ms` become `V0_air_ms` and `V0_ground_ms`; add the flag columns of section 5.
- `D_a_eq_speed` (diagnostic): the old grid `v0_eas_ms` x scenarios x mass factors, with `V0_air_ms` = `tas(ve, sc)` **as an air speed** and `V0_ground_ms = V0_air_ms - headwind` (fixes the input inconsistency). This is the table to put next to A's window at the same speeds, and the source of an `a_eq` against entry-speed schedule for DEC-3. It is not used for the headline.
- `D_recommendation_ensemble` (per run) and `D_recommendation` (summary), section 6.
- `D_trade_curve`: structure unchanged; its entry becomes the rule's air speed for the first named scenario (the current `inp.entry_eas` is a different quantity).
- `D_sensitivity`: on the first named scenario at nominal mass, one row per case: base; `theta_fw0` in the sweep; `rotor_lag_s` in the sweep (lift-aware); `ff_mode = "repo"`; lift-aware with each `tau` and each `scale`; `pit_lim_mode = "airspeed"` (only if `inp.controller.theta_B` is set); `enforce_alpha = True`. Columns: `case`, `a_eq`, `x_actual`, `alt_gain_m`, `alt_loss_m`, `vh_end_ms`, `alpha_max_gated_deg`, `t_above_alpha_08_s`, `t_above_alpha_s_s`.
- Run a second headline pass with `enforce_alpha = True` only if cheap; at least report its `a_rec_pct` in `D_sensitivity`.
- Label the excursion columns "lower bound: ideal actuators" in the table notes.

## 8. Interfaces: who passes `a_plan`

- **D itself:** `D_trade_curve` loops over `D.trade.a_plan_ms2`; `landing_error` turns `a_plan` into `x_tgt = V_ref^2 / (2 a_plan)`. The Position1 cascade never reads `a_plan`; it brakes with `a_lim` (3.0). ArduPilot's `Q_TRANS_DECEL` does both jobs; the repo splits them. Say so in the `landing_error` and `BackTransitionParams` docstrings.
- **E:** `stage_e` takes `a_plan = out.get("a_rec", 2.5)`. Change only this: if `a_rec` is missing or NaN, use `inp.study.E.a_plan_fallback_ms2` (new, default 2.5), append a message to `errors`, and add an `a_plan_source` column (`"D"` or `"fallback"`) to `E_summary`. E keeps reading `a_rec` (now the minimum over Sobol and corners); whether it should read `a_rec_pct` is DEC-3.
- **C:** `restricted.plan_decel` calls `braking.measure_a_eq` with `BackTransitionParams.for_vehicle(...)` and an explicit `x0`; with the dataclass defaults above it is unchanged. **B:** `collocation._plan_decel` uses `ideal_stopping`; unchanged.
- C and B2 therefore keep the old behaviour until someone switches them. Their `a_plan` still comes from a stalled-wing worst-case `a_eq` when `a_plan_source = "simulated"`; note that in the C and B2 docs.

## 9. Performance

A run takes about 0.1 s here, so the whole D stage (about 18 named runs, 40 ensemble runs, 16 x 3 x 2 trade runs, about 20 sensitivity runs) is small. The `LiftFF` table avoids a root solve per control step; check that interpolation agrees with a direct solve to 1e-3 in throttle.

## 10. README (`analysis/README.md`, D section)

Rewrite: entry is the AIRBRAKE exit; the ramp and its clock; the two `Q_BCK_PIT_LIM` forms; the three feed-forward settings and that `lift_aware` is an oracle at default; that the wing is stalled for most of the braking in the default results and what the alpha flags and the enforcement flag mean; that altitude excursion is a lower bound (ideal actuators, assumed gains); the new ensemble and its three headline numbers; the `a_plan` chain of section 8.

## 11. Acceptance checks

1. **Regression (exact):** with `ff_mode="repo"`, `theta_fw0=0`, `pit_lim_mode="constant"`, `alpha_cap=None`, `rotor_lag_s=0` and the old `_initial_state` entry, `measure_a_eq`, the old `D_a_eq` columns and C's `plan_decel` output are bit-identical to before.
2. **Ramp:** `theta_fw0 = 8 deg`, cap 30 deg, `T_bt` 3 s: `theta_limit` is 8, 19 and 30 deg at t = 0, 1.5 and 3 s; with `theta_fw0 = 40 deg` it is 30 deg throughout.
3. **Airspeed form:** `theta_B = 10 deg`, `v_min_ref = 17`: EAS 17 gives 10.0 deg; 15 gives 12.8; 12 gives 20.1; 10 gives 28.9; 5 gives 30 (the `theta_A` cap); 25 gives 4.6; `theta_B <= 0` returns the ramp; a missing `eas` raises.
4. **Start-pitch sweep** (lift-aware, instant actuators, 15 m/s ground speed, 1500 m): `theta_fw0` = 0, 4, 8, 12 deg gives `a_eq` = 2.52, 2.70, 2.89, 3.13; x = 44.4, 41.5, 38.8, 35.8 m; peak alpha 41, 40, 44, 53 deg; time above 9.1 deg 3.1, 3.2, 3.4, 3.7 s. At 17 m/s: `a_eq` 2.65, 2.78, 2.93, 3.13. Tolerance about 0.05 m/s^2.
5. **Feed-forward at 17 m/s, theta_fw0 = 0:** `repo`: gain about 6.9 m, end sink about -5.1 m/s, peak alpha about 109 deg ungated, time above `alpha_s` about 2.2 s. `lift_aware`: gain 0.65 m, loss 0.16 m, end sink -0.52 m/s, `a_eq` 2.65, x = 54.4 m, peak alpha 57 deg. Lift-aware with `rotor_lag_s` = 0.5: gain 2.1 m, loss 3.0 m, end sink -2.3 m/s, `a_eq` 2.57. Lift-aware with a 0.5 s lift filter (no lag): gain about 1.9 m, end sink about -1.6 m/s.
6. **Alpha cap:** `enforce_alpha` with the 9.1 deg band top: gated peak alpha at most about 9.3 deg; `a_eq` 1.30 (15 m/s) and 1.27 (17 m/s); x = 86 and 114 m; end sink about +0.1 m/s. The ungated peak alpha is larger (about 28 deg) because the aircraft is below the 5 m/s gate at the end.
7. **Flags:** on the repo-feed-forward baseline the flags report the fall (`sinking` true, `alpha_max_deg` about 109, `vrs_min` about -0.8, `t_vrs_onset_s` > 0); on the lift-aware run `sinking` is false at tolerance 0.5.
8. **Entry:** at 1500 m with `AIRSPEED_MIN` 17, `entry_airspeed` returns 15.0 (`"tas"`) and `15 * sqrt(rho0/rho)` (`"eas"`, about 16.1); with headwind +5 the ground speed is 10.0; `theta_fw0` = 0 gives initial rotor speed from `T0 = m g - L0` (about 185 N at 15 m/s).
9. **Ensemble:** with the preview settings (32 Sobol, seed 8, 8 corners, `"tas"`), `a_rec_pct` within about 0.05 of 2.10, `a_rec_min` within about 0.05 of 2.00, `n_below_vs` about 17, `n_nan` = 0; `"eas"`: `a_rec_pct` about 2.19, `n_below_vs` = 0. `latin_hypercube(n, vp, seed)` output unchanged.
10. **Interface:** with D disabled, `stage_e` uses the fallback, records an error, and `E_summary` shows `a_plan_source = "fallback"`; with D run, `"D"`.
11. **Input fix:** `D_a_eq_speed` has `V0_ground_ms = V0_air_ms - headwind` for the headwind scenario.

## 12. Caveats to carry into the PR description

- **The default results describe a stalled wing.** The recommended deceleration is reached with alpha well above the stall angle for 2.5 to 3.5 s and the rotors carrying the weight. Treat it as an upper bound of what the idealised controller can do, not as validated hardware performance; the lift and drag beyond the stall angle are a flat-plate placeholder. `enforce_alpha` shows the other bound (about 1.3 m/s^2).
- **`lift_aware` is an oracle at default** and changes what D describes: a controller with a lift-aware altitude hold, not necessarily ArduPilot's (I have not verified what AIRBRAKE and Position1 do with wing lift). Keep `ff_mode: repo` available and show both in `D_sensitivity`.
- **Altitude excursions are lower bounds:** assumed gains, instant actuators (`I_rot` is TBD), no wind gusts.
- **`theta_fw0` is a placeholder.** Default 0 is a guess: AIRBRAKE clamps pitch at 0 or above and TECS probably asks for nose-down as the speed falls, which would put it at the clamp, but this is not verified. The real number is the TECS/navigation pitch demand at AIRBRAKE exit from a SITL run or flight log. Each degree adds about 0.05 m/s^2 to `a_eq` and lengthens the stalled time.
- **Entry convention (RC5) moves the ensemble:** `"tas"` puts 17 of 32 samples below their stall speed, `"eas"` none; the 5th percentile moves from 2.10 to 2.19. Keep it a config value and report the one used.
- **Entry speed is no longer an axis.** The old result that entry speed dominates `a_eq` is now carried by headwind through the ground speed; `D_a_eq_speed` keeps a speed grid for comparison with A.
- **`a_rec` is a worst-case braking measure** (pitch at the limit throughout); the cascade overshoots by metres even at `a_plan = a_rec`. The trade curve is the place to read the planning margin.
- **`airspeed` pitch-limit form** needs an EAS and a `V_min` in the same convention as `AIRSPEED_MIN` (RC5); until that is settled it is a flag, not a result.
- This changes the meaning of the D tables; downstream reading notes (`bracking.py.md`) need a separate update.

## 13. How the scratch numbers were produced

Scratch scripts (not for the repo), all on the hand-copied vehicle above: a subclass of `Position1Controller` that replaces `delta_hover` by the throttle for `max(0, W - L)/cos(theta)` using the simulated lift (the lift-aware case, with optional filter and scale); a `theta_limit` override that starts the ramp at `theta_fw0` and an initial state with `theta = theta_fw0` (the sweep in check 4); a subclass that limits `lim` to `alpha_cap + gamma` above 5 m/s (check 6); and a loop over 32 scrambled-Sobol scenarios plus 8 corners with entry `V_air0 = 15 m/s` and ground speed `V_air0 - headwind`, mass applied through `Scenario.mass_kg` and a per-scenario feed-forward built from that scenario's mass, density and SoC (check 9).
