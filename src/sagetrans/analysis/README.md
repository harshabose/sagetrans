# `sagetrans.analysis` — what the toolkit computes

Layer position: above `control`, below `io`. Each analysis is a function from a vehicle, a
scenario and options to a results object. Analysis letters follow the proposal's section 10.

> All results quoted below were obtained with the **placeholder vehicle** and are `DRAFT`. They
> show that the analyses run and behave sensibly, and what they found *in the model*; they are not
> conclusions about Sage.

| Module | Proposal | Question answered |
| --- | --- | --- |
| `zero_loss.py` | **A** — quasi-steady zero-altitude-loss feasibility | For which constant decelerations at which speeds can the hardware hold altitude, inside an angle-of-attack band? What fails at each edge? |
| `collocation.py` | **B1** — open-loop optimum by direct collocation | What is the best trajectory under the constraints? (energy against excursion and distance) |
| `freeform_opt.py` | **B2** — speed-scheduled law tuned over an ensemble | What does a robust, feedback-free speed law achieve? |
| `restricted.py` | **C** — ArduPilot-parameter optimisation | What can the autopilot be *configured* to do? |
| `braking.py` | **D** — Position1 braking and `Q_TRANS_DECEL` | How far does the aircraft need to brake, and what deceleration should the planner assume? |
| `energy.py` | **E** — mission energy budget and reserve | Does the mission fit in the usable pack energy at 100 and 113 km (Q1, Q7)? |
| `pusher.py` | **Q2** — pusher forward acceleration | What forward acceleration can the pusher deliver at `TKOFF_THR_MAX` from rest to `AIRSPEED_MIN`? |
| `descent.py` | **F** — landing descent profile | How fast may it descend, and what does the landing hover cost (Q6)? |
| `sensitivity.py` | FR-08 | Which uncertain inputs drive each headline result? |
| `common.py`, `ensemble.py`, `search.py`, `runs.py` | shared | scenarios, run records, ensembles and cost, CMA-ES wrapper, run helpers |

---

## Shared plumbing

### `common.py`

* **`Scenario(altitude_amsl_m, isa_offset_k=0, soc=0.8, headwind_ms=0, ground_amsl_m=0, mass_kg=None)`**
  — one scenario. `altitude_amsl_m` is the *flight* altitude (it sets the density); `ground_amsl_m`
  is the terrain elevation. `.rho` is the density, `.vehicle_for(vp)` returns the vehicle with the
  scenario's mass if set, and `.environment()` builds the `dynamics.Environment`.
* **`RunRecord(input_hash, code_version, date, label="DRAFT")`** and **`make_record(*parts)`** —
  the per-analysis traceability record. The hash is a SHA-256 over a JSON of the vehicle
  parameters, scenario and options passed in. Every analysis result carries one. It is **not** the
  register's `Config` hash (the register is not connected to the physics yet), and the label is
  always `DRAFT`. `io.record.build_record` combines these with the register state.

### `ensemble.py`

* **`latin_hypercube(n, vp, seed, axes)`** — `n` scenarios with exactly one sample per stratum
  on every axis, deterministic for a given seed. Axes (`EnsembleAxes`, placeholders pending the
  mission set, O-14): altitude 0–5000 m, ISA offset −10…+20 K, mass factor 0.85–1.15 times the
  vehicle mass, headwind −5…+10 m/s, state of charge 0.2–1.0.
* **`braking_ensemble(n, vp, seed, axes, corners=True)`** — analysis D's ensemble: a scrambled
  Sobol sample over the five axes (`n` a power of two) and, as a second list, the 8 corners
  (altitude x mass x headwind, the other axes at their midpoints). `latin_hypercube` is untouched
  (C and B2 use it). **`bootstrap_percentile(values, q, n_boot, seed, ci)`** — a percentile with a
  bootstrap interval.
* **`RunMetrics(excursion, energy, distance, failures)`** — one run's headline numbers and failure
  flags (`stall`, `battery_cutoff`, `touchdown`, `not_stopped`, `no_transition`).
* **`CostWeights`** and **`ensemble_cost(runs, w)`** — eq. (10.1):

```
J = w_h p95(dh)/h_ref + w_E mean(E)/E_ref + w_x p95(x)/x_ref + P * (number of failures)
```

  Defaults `w_h=1, w_E=0.3, w_x=0.5, h_ref=10 m, E_ref=5 kJ, x_ref=150 m, P=10`. The weights are a
  policy choice; the proposal asks reports to show at least three weightings (not yet done).
  Because failures add jumps, `J` is discontinuous and a gradient-free search is used.

### `search.py`

* **`cma_minimise(fn, u0, seeds=(1,2,3), sigma0=0.25, popsize=8, maxiter=30, tolfun=1e-6)`** —
  CMA-ES (the `cma` package) on the unit cube `[0,1]^n`. Evaluations are cached by the rounded
  parameter vector, each seed runs independently, and the best point ever evaluated is tracked, so
  the result is never worse than the start. Deterministic for a seed (NFR-01).
* **`SearchResult.spread`** is `(max − min)/best` over the per-seed best costs: the **T-12**
  measure ("same optimum from three seeds within 1 %").
* **`Bounds(names, lo, hi)`** maps between physical values and the unit cube.
* Evaluations run **serially**: CasADi functions cannot be pickled across processes, so the
  proposal's parallel evaluation is not implemented.

### `runs.py`

Helpers shared by B2, C and B1: a cached `vehicle_for(vp)`; `stall_speed`;
`cruise_entry_state(vp, scen, v_air)` (level steady cruise from `steady_trim`, rotors stopped,
pusher at its trim speed, ground speed `V_air − headwind`); `end_events`; `failure_flags`;
`segment_stats`; `run_metrics`.

**The stall flag** counts time with `|alpha_w| > alpha_s` **only while `V` is at or above the 1g
stall speed**. A stalled wing matters only while it still carries load; below the 1g stall speed the
rotors carry the weight by design. Without this rule every back-transition was flagged, because the
nose-up aircraft briefly stalls its wing at low speed with the rotors already holding it. A run is
a stall failure if that time exceeds 0.3 s.

---

## A — quasi-steady zero-altitude-loss feasibility (`zero_loss.py`)

**Question.** In an idealised case, can the hardware hold altitude through a constant deceleration,
and for which decelerations at which speeds?

**Scope — an idealised scan, not a simulation.** A constant deceleration `a` from a free entry speed
`V0` down to `v_end` (1 m/s), `gamma = 0`, no wind, **pusher off** (`t_p = 0`; the argument is kept
but is not a config axis). The rotors are always treated as **pre-spun**: their speed at the first
sample is the first schedule value. There is no entry step and no jerk limit. The argument for
treating the rotor start speed as a decision variable now lives in B1 (`prespin_max`) and D.

**Method.** At each speed the force balance leaves pitch as the one unknown. Eliminating the rotor
thrust gives one equation in pitch:

```
R(theta) = (T_p - H) + (m a - D(theta)) cos(theta) - (W - L(theta)) sin(theta) = 0
```

which reduces to eq. (10.3), `tan(theta) = (m a - D)/(W - L)`, `T_r = sqrt((m a - D)^2 + (W - L)^2)`,
when `H = T_p = 0`, and to `tan(theta) = a/g` at low speed. (`H` is the rotors' in-plane force; it
depends on the rotor speed, so it is iterated.) The rotor map is then inverted for the rotor speed
that gives `T_r` (`dynamics.trim`), and the motor equation for the throttle **including the
rotor-acceleration term** `I_rot * omega_dot`.

**The angle-of-attack band.** The wing's angle of attack must stay inside `[alpha_min, alpha_hi]`:
`alpha_min = 0` is the lower end of the range the lift fit was made on (0 to 10 deg), and `alpha_hi`
is the angle where `C_L = cl_frac x C_L,max` on the rising side (proposal rule 8.6; `cl_frac = 0.8`,
about 9.1 deg for the current wing; 1.0 is the lift peak). Both are settings (`alpha_min_deg`,
`cl_frac` under `A:` in `study.yaml`). `C_L,max` is provisional (`alpha_s` in `vehicle.yaml`), so
`alpha_hi` is too. A profile that needs negative angle of attack, where the lift model is an
extrapolation, or one past the band, is infeasible.

**Root choice.** `R(theta)` has several roots (at 17 m/s and `a = 1.04` there are roots at 4.2, 8.1
and 14.9 deg). Roots are found on the coarse 61-point grid over the whole range and, with a band, on
a dense 121-point grid inside it, because near a fold the two low roots are less than 1 deg apart.
The choice is: in-band roots with `T_r >= 0` (nearest the previous pitch), else in-band roots with
`T_r < 0` (these fail `thrust_sign`), else the out-of-band root nearest the band (these fail an angle
margin). With no band the pre-band selection is reproduced exactly (this is what B2's start uses).

**Checks** (each a normalised margin, feasible when `>= 0` everywhere): "pitch solution exists",
thrust sign (`T_r >= 0`), rotor throttle ceiling (`ActuatorLimits.delta_r_max` 0.95), motor current,
bus voltage floor, torque floor (the ESC cannot brake the rotor), pitch limit (0.6 rad, never binds),
pitch rate against `q_max`, the two angle margins `alpha_low` and `alpha_high` (the legacy
symmetric `stall` margin when `cl_frac` is `None`), and, **only when `ActuatorLimits.vrs_limit` is
set**, the vortex-ring margin `vrs`. **Every** violated margin is reported with the
first time it goes negative, earliest first, as a `violations` dict (`first_binding(result)` gives the
old single name).

**Vortex-ring ratio (report only by default).** At the rotors `V_c = -V sin(theta)` (gamma = 0, so a
nose-up pitch gives a **negative** ratio: the flow comes up through the discs) and
`v_h = sqrt(T_r / (2 rho A_tot))`, as in `rotor.vrs_ratio`. The ratio `V_c / v_h` is always
reported: `ZeroLossResult.vrs_min` and `vrs_min_t` (most negative value on the path and when),
`Window.vrs_lo` / `vrs_hi` (at the feasible point on each edge), and the `vrs_min` columns of
`A_feasibility` and `A_window`. It does not change any verdict unless enforced: with
`vrs_limit = -kappa` (set by `stage_a` from `mission.yaml` `descent.kappa` when
`vrs_enforce: true`) the margin is `(ratio - vrs_limit) / |vrs_limit|`, so a ratio of 0 gives +1 and
a ratio of the limit gives 0. Enforcing trims only the **top** of each window (the ratio is about 0 at
the lower edge and goes strongly negative toward the upper edge, where `T_r -> 0`): about 0.04 to
0.18 m/s² on the real-data draft. It is report only by default because `kappa = 0.23` is from
near-zero-forward-speed data whereas A runs at 12 to 22 m/s, where the rotors are at several times
their hover induced velocity and onset is a low-advance-ratio phenomenon, so an axial-only criterion
is probably over-conservative here; the ratio also diverges as `T_r -> 0`, so near the upper edge it
flags a barely loaded rotor. Confirm `kappa` against NASA/TP-2005-213477 before relying on it.

**The window.** At each speed the feasible deceleration forms a window `[a_lo(V), a_hi(V)]`
(`feasible_window`). The lower edge is the wing's drag floor at `alpha_min` (below it the wing
would have to be at negative angle of attack to give so little drag). The upper edge is where the
low-theta root pair of the force balance disappears (a fold): beyond it only the stalled root near
15 deg exists and `thrust_sign` fails. `bind_lo` and `bind_hi` list every margin violated just
outside each edge. The window scan is pointwise with a **steady** rotor (`omega_dot = 0`, battery RC
state 0); the constant-`a` verdict from `analyse` also includes the rotor acceleration, battery
state and pitch rate and is the authoritative one for a whole path.

**API.**

| Name | Purpose |
| --- | --- |
| `DecelProfile(a, v_end=1.0).sample(v0, n)` | constant deceleration; returns `t, V, a(t)` |
| `ActuatorLimits(..., alpha_min, cl_frac, vrs_limit)` | the limits; `LEGACY_LIMITS` (all `None`) = the pre-band behaviour; `vrs_limit=None` = report the vortex-ring ratio only |
| `solve_point(vp, rho, V, a, t_p, prev_theta, bal, band)` | one pointwise solution: `theta`, `T_r`, `H_r`, `omega`, `L`, `D`, `ok` |
| `analyse(vp, scen, v0, profile, limits, t_p, n=120, v1_0=0)` | the full result (`ZeroLossResult`: `margins`, `feasible`, `violations`, `vrs_min`, `vrs_min_t`, ...) |
| `feasible_window(vp, scen, V, limits, ...)` | windows `[a_lo, a_hi]` at one speed, with `Window.bind_lo/bind_hi` |
| `closest_to_feasible(...)` | when there is no window: the deceleration that fails least, and what fails |
| `feasible_decel_range(vp, scen, v0, limits, ...)` | `(a_min, a_max, islands)` for the whole path from `v0` to rest, or `None` |
| `max_feasible_decel(...)` | `a_max` of the above (0 if none) |
| `feasibility_map(...)` | `{"feasible": bool[nV, na], "violations": [[dict]], "vrs_min", "vrs_min_t"}` over a grid of `(V0, a)` |

`feasible_decel_range` scans 48 values of `a` (spacing about 0.08 m/s²), takes the first and last
feasible ones and bisects each edge. A feasible range narrower than the spacing can be missed;
`islands = True` flags a feasible set with gaps (the full span is still returned).

**What it found (real-data draft, 1500 m, pusher off).** The window narrows with speed: about
0.34 to 1.28 m/s² at 12 m/s, 0.60 to 1.09 at 16 m/s, 0.76 to 1.09 at 18 m/s, 0.93 to 1.17 at
20 m/s, and 1.18 to 1.32 at 22.5 m/s. For a whole constant-`a` path from `V0` to rest the upper edge
is set by the 16 to 18 m/s part, so about 0.9 to 1.1 m/s² is feasible from 20 m/s and **nothing** is
feasible from 22.5 m/s. This is a property of the idealised pusher-off, rotors-never-push-down case,
not a hardware limit. The aerodynamic limits depend only on dynamic pressure, so they are the same at
sea level and 5,000 m at equal equivalent airspeed (tested); the throttle demand rises in thin air.

**Tests** (`test_analyses.py`): closed forms (`theta = atan(a/g)`, `T_r = m sqrt(a^2+g^2)`), coast
bound, eq. (10.3) self-consistency with the wing on, the profile, the angle band, root preference
and the unbanded selection, the violation list, the window against the path check, an empty window,
the legacy non-monotone verdict that B2's start relies on, and the equivalent-airspeed scaling.

--- | --- |
| `DecelProfile(a, jerk=None, v_end=1.0).sample(v0, n)` | constant or jerk-limited deceleration; returns `t, V, a(t)` |
| `EntryState(omega_r=0, theta=None, v1=0)` | the state before the first sample (`theta=None` = level trim) |
| `solve_point(vp, rho, V, a, ...)` | one pointwise solution: `theta`, `T_r`, `H_r`, `omega`, `L`, `D`, `ok` |
| `analyse(vp, scen, v0, profile, limits, entry, t_p, n=120)` | the full result (`ZeroLossResult`) |
| `max_feasible_decel(...)` | the largest feasible constant deceleration (0 if none): a coarse scan over `a`, then bisection at the top of the highest feasible region (feasibility is **not monotone** in `a`, see below) |
| `feasibility_map(...)` | both verdicts and the first binding constraint over a grid of `(V0, a)` |

**What it found (placeholder).** At 12 m/s and 1.5 m/s² the pre-spun schedule is feasible while the
whole segment is not. At higher speed the first binding constraint is *thrust sign*, not propulsion:
the pitch needed to brake pushes the wing's lift above the weight, so the rotors would have to push
down. That limit depends only on `V/V_s`, so it is the same at sea level and 5,000 m at equal
equivalent airspeed (tested); the throttle demand, in contrast, rises in thin air.

**Feasibility is not monotone in the deceleration.** At 12 m/s the placeholder aircraft is feasible up
to about 1 m/s², infeasible from 1.0 to 1.4 (thrust sign), feasible again from 1.6 to 2.0, and
infeasible above about 2.1 (stall); the pattern is the same at 60 and 120 samples, so it is not a mesh
artefact. A higher braking pitch adds drag, which removes the need for negative rotor thrust until the
wing stalls. `max_feasible_decel` was originally a plain bisection that assumed monotonicity: it
returned 0.98 m/s² at 12 m/s, missing the 1.6–2.0 region. It now scans 24 values of `a` first
(spacing about 0.34 m/s², so a feasible island narrower than that can still be missed) and bisects only
at the top of the highest feasible region; it returns 2.43, 2.02, 0.82 and 0.76 m/s² at 10, 12, 14 and
16 m/s. `feasibility_map` remains the authoritative picture. A regression test covers it.

**Tests** (`test_analyses.py`): closed forms (`theta = atan(a/g)`, `T_r = m sqrt(a^2+g^2)`), coast
bound (`C_L = 0`, `a = D/m` gives level pitch and `T_r = W`), eq. (10.3) self-consistency with the
wing on, profile sampling, the entry-step verdicts, consistency of `max_feasible_decel`,
whole-implies-pre-spun, and the equivalent-airspeed scaling.

---

## D — Position1 braking and `Q_TRANS_DECEL` (`braking.py`)

**Question.** The autopilot switches to VTOL "Position1" when the distance to the landing point is
within `V^2 / (2 * Q_TRANS_DECEL)`. What value makes the aircraft stop where intended?

```
x_plan = V0^2/(2 a_plan),   a_eq = (V0^2 - V_f^2)/(2 x_actual),   overshoot = x_actual - x_plan,
a_rec = a low percentile (and the minimum) of a_eq over the ensemble                       (10.5)
```

`a_eq` uses `V0^2 - V_f^2` (with the end speed `V_f = 1 m/s`) so it equals eq. (10.5) when `V_f = 0`.

### What a run is

* **Entry is the AIRBRAKE exit.** `t = 0` is when ArduPilot leaves AIRBRAKE, which is also when it
  starts the pitch-envelope clock (`set_last_fw_pitch()`, open question O-03). The rotors are
  already spun up and carry what the wing does not (`hold_hover(0)`; no spool dynamics), the pusher
  is off, and the pitch is the last fixed-wing pitch `theta_fw0` (**placeholder 0**: the real value
  is the TECS/navigation pitch demand at the AIRBRAKE exit, from a SITL run or a flight log; each
  degree adds about 0.05 m/s² to `a_eq`). `airbrake_entry_state` builds that state.
* **The entry speed comes from the airspeed exit rule only**: `entry_airspeed` returns
  `max(AIRSPEED_MIN - 2, Q_ASSIST_SPEED)` as a true airspeed. `entry_convention: tas` reads
  `AIRSPEED_MIN` as a true airspeed (as the controller does today), `eas` as an equivalent one
  (open question RC5). The ground speed is the air speed minus the headwind.
* **The pitch limit is ArduPilot's** (O-04): it ramps from `max(theta_fw0, 0)` to
  `min(Q_A_ANGLE_MAX, PTCH_LIM_MAX_DEG)` over `Q_BACKTRANS_MS`, clock from `t = 0`; the initial
  pitch is `theta_fw0`. `Q_BCK_PIT_LIM` is a constant cap by default; `pit_lim_mode: airspeed`
  switches to ArduPilot's airspeed-scaled form (O-05; it needs an EAS and a `V_min` in the same
  convention as `AIRSPEED_MIN`, so it is a flag, not a result, until RC5 is settled). There is no
  nose-down envelope.
* **Altitude-hold feed-forward** (`ff_mode`): `repo` is the constant hover throttle of the original
  controller, so the rotors over-thrust by about the wing lift at entry and under-thrust when the
  wing stalls (17 m/s: the aircraft gains 6.9 m and ends sinking at 5 m/s). `lift_aware` (the
  default here) commands the throttle for `W - L`. **At `ff_lift_tau_s = 0` and
  `ff_lift_scale = 1` it is an oracle**: it reads the simulated lift, which the aircraft does not
  measure. The tau and the scale stand in for an estimate. D then describes a controller with a
  lift-aware altitude hold, not necessarily ArduPilot's; both are shown in `D_sensitivity`. The
  altitude-hold gains are unchanged.
* **No spool dynamics and no actuator lag** by default. `rotor_lag_s` is the one actuator-lag
  parameter of D: a lumped first-order lag on the rotor throttle command. It stands for everything
  between ArduPilot's rotor output and the rotor-speed response that the vehicle model does not
  already contain (ESC slew, output filtering, throttle smoothing, rotor or motor inertia beyond the
  placeholder `I_rot`). It is not `MOT_SPOOL_TIME` (the rotors are already spooled). Its value is
  not fixed: `D.sens.lag_s_sweep` (0, 0.1, 0.25, 0.5 s) is a sensitivity, and the altitude excursion
  stays a lower bound until a bench value is set.
  * `vehicle.rotor.I_rot` is separate and stays: the vehicle model integrates the rotor speed with it
    and analyses A, B1 and B2 depend on it. It is a physical parameter, not a D setting, and D does
    not sweep it. `braking.rotor_speed_tau` is the model's own time constant at hover (about 0.023 s
    at the placeholder, linear in `I_rot`); it adds in series with `rotor_lag_s`, so the lag
    dominates for any value worth sweeping. `D_sensitivity` shows `rotor_lag_s`,
    `tau_rotor_model_s` and `tau_total_s` on every row.
  * **How to set it:** bench-test one lift motor with its ESC and propeller, step the throttle about
    the hover command (for example +10%), record rotor speed (or thrust) and fit a first-order
    `tau_meas`. Then `rotor_lag_s = max(0, tau_meas - tau_rotor_model_s)`, so the model's lag is not
    counted twice. If a measured `I_rot` arrives, enter it in `vehicle.yaml` and reduce `rotor_lag_s`
    by the model time constant it adds.
  * B1 and B2 have only `I_rot`, not this lag; C uses the controller and gets it. Compare B with C
    at `rotor_lag_s = 0`.

### The wing is stalled while braking

With the altitude held, the wing angle of attack equals the pitch, so the 30° ramp is a wing angle
of attack well beyond the stall angle for 2 to 3.5 s: the lift falls and the rotors carry the
weight. The lift fit is valid to about 10°; beyond the stall angle the model is a flat-plate blend,
so forces there are unvalidated. **The default `a_eq` (about 2.5 m/s² at 15 m/s) is the most the
idealised controller can do in that state, not validated hardware performance.** If the angle of
attack were held to the 0.8 C_L,max band top of analysis A (about 9.1°), `a_eq` would be about
1.3 m/s², which is A's window plus a ramp. `enforce_alpha: true` shows that bound.

`braking_flags` reports, per run and without changing it:

| Group | Columns |
| --- | --- |
| Angle of attack (a) | `alpha_max_deg` (ungated), `alpha_max_gated_deg`, `t_alpha_08_first_s`, `V_at_alpha_08_first_ms`, `t_above_alpha_08_s`, `t_above_alpha_s_s` (above the stall angle), `t_below_alpha_min_s`; the integrals use samples with `V >= alpha_gate_ms` |
| Rotor load (c) | `rotor_share_max`, `wing_share_min`, `throttle_margin_min`, `t_throttle_sat_s` |
| Other limits | `vrs_min`, `vrs_min_t_s`, `t_vrs_onset_s` (ratio below `-kappa`; report only), `peak_motor_current_A`, `current_over`, `min_bus_voltage_V`, `voltage_under` |
| End state | `vh_end_ms`, `dh_end_m`, `stopped`, `sinking` (`vh_end < -sink_tol_ms`, placeholder 0.5 m/s) |

`enforce_alpha` is the one switch that changes the run: it limits the pitch command to
`alpha_cap + gamma` while `V >= alpha_gate_ms`, with `alpha_cap` the top of A's band (body pitch;
it already subtracts the wing incidence).

### The ensemble and the recommendation

`ensemble.braking_ensemble` draws a scrambled Sobol sample (`ensemble_n` a power of two) over the
five axes of `scenarios.yaml`, plus 8 corners (altitude x mass x headwind, the other axes at their
midpoints) reported separately. **Entry speed is not an axis**: every scenario enters at the rule's
air speed and the ground speed follows from its headwind. That moves the result: the headwind
effect changes sign. It used to add drag at a fixed ground speed and help; with the entry fixed in
air speed, +5 m/s of headwind means a 10 m/s ground speed, a shorter run in which the fixed 3 s
ramp is a larger share, so `a_eq` falls (Spearman −1.0 with headwind). The lowest runs are the
low-ground-speed ones.

`D_recommendation` carries three numbers: `a_rec_pct_ms2` (5th percentile over the Sobol sample
only; the headline), its bootstrap interval (`bootstrap_percentile`) and `a_rec_min_ms2` (minimum
over Sobol and corners; this is the `a_rec` that analysis E reads). `a_rec_pct_half_ms2` (the same
percentile over the first half) is a convergence check. `n_below_vs` counts entries below their own
1g stall speed (flagged, not dropped; at 1500 m with `tas` that is 17 of 32, with `eas` none).
At the register values (32 points, seed 8, `tas`): 5th percentile 2.09 (interval 2.05 to 2.17),
minimum 1.99 (the corner at altitude 0, mass ×0.8, headwind +5), median 2.5; with `eas`, 5th
percentile 2.18. Before this change the ensemble gave 2.52 (minimum 2.51).

**`a_rec` is a function of the ground speed.** In the default runs `a_eq` is set by the ground speed
at the AIRBRAKE exit and by almost nothing else: the pitch limit ramps with time from the exit and
the rotors carry `W - L`, so the deceleration is about `g tan(theta)` times `(1 - L/W)`; mass
cancels and the density enters only through the small wing terms. A slow aircraft stops while the
ramp is still low, so its distance-equivalent `a_eq` is low. With the entry air speed fixed, only the
headwind moves the ground speed, so `a_rec_pct` is in effect a percentile of the headwind range.
`D_a_rec_curve` therefore reports the recommendation as a curve, one row per ground speed in
`D.curve.ground_speeds_ms`: the air speed is the rule's, the headwind is `V_air0 - V_ground` (a
negative value is a tailwind), and every named scenario x mass factor runs at each speed.
`a_rec_min_ms2` is the minimum over those runs and `set_by` names the run; read `Q_TRANS_DECEL` off
the curve at the slowest ground speed that has to be covered, and look at `a_eq_median_ms2` beside
it (the minimum can be an entry-state artefact: the 10% rotor floor in the lightest run). The single
numbers of `D_recommendation` are valid only inside `covers` (`ground_speed_min_ms` to
`ground_speed_max_ms`, with the headwind range). `D.recommendation.design_ground_speed_ms` gives
`a_rec_design_ms2`, the curve value at that speed by linear interpolation; a speed outside the
curve is an error. Analysis E keeps reading the single conservative `a_rec_min`. E enters faster
than the curve ends (its entry EAS is 21 to 27 m/s TAS against 20 m/s), where `a_eq` is higher, so
`a_rec_min` is conservative there; using the curve at E's entry speed is an open decision.

### Tables

| Table | Contents |
| --- | --- |
| `D_a_eq` | named scenarios x mass factors, entry from the rule; `a_eq`, distance, excursions, the flags |
| `D_a_eq_speed` | diagnostic: the old speed grid `v0_eas_ms`, `V0_air_ms` = TAS of the grid EAS, `V0_ground_ms = V0_air_ms - headwind`; set it beside A's windows at the same speeds |
| `D_recommendation_ensemble` | one row per run (Sobol and corners): inputs, `V0_air_ms`, `V0_ground_ms`, `Vs_ms`, `below_vs`, `a_eq`, the flags |
| `D_recommendation` | the summary row above, plus the scenario that sets the minimum, the worst corner, the Spearman correlation of `a_eq` with each axis, the ground-speed and headwind range it covers (`covers`) and `a_rec_design_ms2` |
| `D_a_rec_curve` | `a_rec` as a function of the ground speed at the exit: per speed, the minimum and median over the named scenarios x mass factors, what sets the minimum, its distance and time, the largest excursion and the number of runs still sinking |
| `D_trade_curve` | first named scenario; entry is the rule's air speed, ground speed is that minus the wind |
| `D_sensitivity` | first named scenario at nominal mass, one change at a time: `theta_fw0`, rotor lag, `ff_mode: repo`, lift filter and scale, the airspeed pitch-limit form (only if `Q_BCK_PIT_LIM` is set), `enforce_alpha`; the base, repo and enforce cases also give the ensemble 5th percentile |

**Altitude excursions are lower bounds**: ideal actuators, assumed altitude-hold gains, no gusts
(`I_rot` is TBD).

### Who passes `a_plan`

`D_trade_curve` loops over `D.trade.a_plan_ms2`; `landing_error` turns `a_plan` into
`x_tgt = V_ref^2 / (2 a_plan)`. The Position1 cascade never reads `a_plan`; it brakes with `a_lim`.
ArduPilot's `Q_TRANS_DECEL` does both jobs, this repo splits them. Analysis E takes
`a_plan = a_rec` (the minimum), and falls back to `E.a_plan_fallback_ms2` with an error message and
`a_plan_source = "fallback"` in `E_summary` when D gave none (whether E should read the percentile
instead is open). Analyses C and B2 keep the old behaviour: C's `plan_decel` still calls
`measure_a_eq` from its own entry state with the dataclass defaults (`ff_mode="repo"`), so its
`a_plan` comes from a stalled-wing worst-case `a_eq`.

**API.**

| Name | Purpose |
| --- | --- |
| `ideal_stopping(v0, theta_max, t_bt, v_end=0, dt=1e-4)` | the planner worked example: ideal pitch, no lift/drag, braking at the limit throughout; returns `(distance, a_eq, ramp distance)` |
| `entry_airspeed(airspeed_min, q_assist_speed, rho, convention)` | the airspeed exit rule, as a true airspeed |
| `airbrake_entry_state(vp, scen, bp, v_air0)` | the state at the AIRBRAKE exit and the ground speed |
| `run_braking(vp, scen, v0, bp, ..., x0=None)` | simulate Position1 from ground speed `v0` to `V_f`; builds the `LiftFF` table when `bp.ff_mode == "lift_aware"`; returns a `BrakingResult` |
| `measure_a_eq(...)` | the worst case: pitch at the envelope limit throughout |
| `braking_flags(res, vp, scen, bp, ...)` | the report-only checks above |
| `recommend_decel(vp, scenarios, v0_grid, bp, percentile=None)` | the older recommendation over a speed grid (kept for callers and tests) |
| `landing_error(vp, scen, v0, a_plan, bp, speed_test)` | switch at `d = V_ref^2/(2 a_plan)` and brake with the position cascade; `speed_test` is `"ground"` or `"air"` |

**Defaults keep the old behaviour.** `BackTransitionParams` defaults (`ff_mode="repo"`,
`theta_fw0=0`, `pit_lim_mode="constant"`, no alpha cap, no lag) with the old `x0` (pitch 0,
hover-speed rotors) give bit-identical results; only `stage_d` switches to the AIRBRAKE entry and
`lift_aware`.

**What it found (placeholder).** `ideal_stopping(25 m/s, 20°)` gives 87.5 m and `a_eq = 3.57 m/s²`
without the ramp, and 124.5 m / 2.51 m/s² with a 3 s ramp (69.8 m during the ramp), matching the
proposal's worked example. The ideal figure is not a bound on `a_eq` in either direction: lag
lowers the real `a_eq`, wing drag raises it. Even at `a_plan = a_rec` the cascade overshoots by
metres, because `a_rec` is a worst-case braking measure (pitch at the limit throughout); read the
planning margin from the trade curve.

**Scenario mass is applied.** `run_braking` uses `scen.vehicle_for(vp)` for the model, for the
initial rotor speed, and for the hover-throttle feed-forward of the altitude hold
(`refresh_hover=True`; set it false to keep a caller's `delta_hover`).

**Tests:** the planner example, the simulated limit case where `a_eq` equals `g tan(theta)` within
0.5 % (frozen-state harness, no ramp), the signed excursion, the altitude hold in hover,
`recommend_decel` being the minimum, earlier switching reducing overshoot, the air-speed test
switching earlier in a headwind, and the scenario-mass regression. New tests for this change
(ramp start, airspeed form, feed-forward, alpha cap, entry, ensemble, the E fallback) are left to
the owner.

---

## F — descent profile (`descent.py`)

```
v_h = sqrt(T / (2 rho A_tot)),     V_z,max = kappa * v_h                                     (10.7)
```

`descent_profile(vp, scen, h_start, limits)` returns descent rate, time-to-go and hover energy
against height. The descent rate is capped by the **tightest of** the vortex-ring limit
`kappa * v_h`, a payload-fragility limit (default 3 m/s), and an optional sensor limit; below
`h_slow` (5 m) it ramps linearly down to `v_final` (0.5 m/s). Time is the integral of `dh/v`;
energy is the hover power times that time (thrust taken equal to weight).

**`kappa` has no default, on purpose.** The vortex-ring boundary must come from Johnson's NASA
report; the working figure of a quarter to a third is unsourced and "must not appear in a result".
Calling with `kappa=None` raises `ValueError`. The `DescentLimits.kappa_source` string is carried
into the result and the segment source text. The `binding` field says which limit sets the rate.

**Tests:** the sourced-`kappa` rule, `v_h` against the formula, the binding limit in each case, the
descent time against a closed-form hand calculation (`(h0 − h_s)/v_c + h_s ln(v_c/v_f)/(v_c − v_f)`,
rel 1e-4) and energy `= P_hover * t`, monotone time-to-go, and thin air raising both the allowed
rate and the hover energy.

---

## E — mission energy budget and reserve (`energy.py`)

```
E_total = E_to + E_ft + E_climb + E_cruise + E_desc + E_bt + E_land + E_reserve
          <= eta_usable * SoH * f(T) * E_nom                                              (10.6)
```

### What E computes

* **Two missions per site.** `round_trip`: a loaded outbound leg, the payload drop, an empty
  return leg (`vehicle mass - payload_kg`); each leg has its own take-off, forward transition,
  climb, cruise, glide, back transition and landing, so every fixed segment appears twice and the
  reserve is counted once (the larger of the two landings' reserves). `one_way`: a single loaded
  leg. `MissionBudget` composes the legs (`BudgetResult` is one leg and is unchanged).
* **The range is an output.** Whatever energy is left after the fixed segments and the reserve goes
  to cruise (`max_range_m`, in closed form: the energy is linear in the distance). It is the
  one-way distance, or the radius of a round trip (which flies twice that). The 100 and 120 km
  requirements are comparison columns (`margin_Wh_at_<R>km`, `conflict_<R>km`); they never enter the
  solve.
* **Site and climb height.** The swept quantity is the airfield elevation (`E.sites_amsl_m`);
  take-off and landing are at the same elevation and the cruise altitude is the site plus
  `mission.profile.climb_height_m` (200 m, so the climb is 170 m above the 30 m clearance). The
  5000 m site is a design-envelope check, not a delivery site.
* **Speeds follow the dynamic pressure.** `v_cruise` is the design TAS at
  `v_cruise_ref_amsl_m` (22.5 m/s at 1500 m, an EAS of 20.9); at any other altitude the TAS follows
  from the density (`design_speeds`, `mission_at_site`). Climb and glide use the same speed unless
  `v_climb` or `v_glide` are given.
* **Forward-transition acceleration** comes from Q2 per site, per leg mass and per leg SoC
  (`E.a_fwd_mode`: `case` default, `worst`, `mission`; `E.a_fwd_metric`: `min` or `equiv`). In
  `case` mode a second pass takes each leg's SoC at the start of its forward transition from the
  trace. A non-positive acceleration gives an error row and no budget (there is no fallback).
* **Back transition** uses the configured controller (as D does) and enters at D's entry EAS
  converted to TAS at the site.

### `soc_trace.py`: the SoC along the mission and the feasibility of each segment

Segment energies use the mean SoC (`soc_mean`, now the mean of the trace). The trace starts at
`soc_start` (a full pack, 1.0) and falls with the cumulative energy over the nominal capacity; the
usable energy is `eta_usable` of nominal, so it ends at 0.2 after the reserve (`window_ok` reports
that this stays above `soc_eod`, the stress point of the hover check). Every segment is checked at
**both** its start and end SoC and the worse is reported: the bus voltage falls with the charge, so
the same thrust needs a higher command (a sea-level climb at 2.5 m/s needs 0.80 at SoC 1.0 and 0.95
at 0.2). A segment **fails** only when its operating point is infeasible: a pusher or rotor command
above 1, a motor current limit, a bus voltage below `V_min`, a pack that cannot deliver the power,
or (forward transition) a pusher that cannot reach `AIRSPEED_MIN`. A pusher command above
`TKOFF_THR_MAX` (the ceiling) is only **reported** (`over_ceiling`); `roc_at_ceiling_ms` is the best
rate of climb at the ceiling. At the defaults nothing fails; every climb is above the 0.8 ceiling,
and the 5000 m return climb is within about 0.01 of the limit of 1. With `soc_start` 0.8 that climb
fails (command 1.04) and `window_ok` is False.

Tables: `E_summary` (one row per site and mission), `E_budget`, `E_segments` (with the checks),
`E_hover_check` (stress point, landing start, after the reserve), `E_reserve`, `E_cruise_sweep`.

| Segment | How it is computed |
| --- | --- |
| Take-off | hover power times (pre-lift-off hover + vertical climb to clearance) plus `m g h` divided by the electrical-to-lift efficiency |
| Forward transition | **a quasi-steady estimate** (constant acceleration `a_fwd`, rotors carrying `W − L`, pusher supplying `m a + D`); `a_fwd` comes from Q2 (`E.a_fwd_mode`: `case` is the pusher at `TKOFF_THR_MAX` at E's own site, `worst` the minimum over the Q2 grid, `mission` the value in `mission.yaml`); to be replaced by the simulated transition |
| Climb | steady-climb trim with the pusher model; power × time |
| Cruise | level trim at the best-range speed (minimum energy per metre); energy per metre × distance |
| Descent | pusher-off glide trim; avionics power × time |
| Back-transition | the analysis-D simulation's bus energy |
| Landing | analysis F hover energy |
| Reserve | go-around hover + hold + diversion cruise + extra transition pairs (Q7) |

**`BudgetResult`** holds the segments, the cruise energy per metre, the reserve and the usable
energy. `table(ranges_m)` gives the budget by range in Wh with margin and "fits"; `max_range_m`
solves for the range at which the margin reaches zero; `conflicts(required_m)` flags where a range
requirement and the usable energy conflict. `from_segments` builds a budget from plain numbers (used
for the hand-calculated reference case).

**`hover_check`** confirms hover power can be delivered at end of discharge:
`P_hover,req <= V_bus(eod) * I_available`, plus the motor-level throttle and current check.

**Assumptions to know about:** every segment's energy uses one **mean state of charge**
(`soc_mean`); the trace and the checks use the actual SoC; the
usable-energy window (`BatteryWindow`: `eta_usable 0.8`, `SoH 1`, temperature factor 1) is assumed
(open question O-15); the forward transition is an estimate.

**What it found (placeholder).** Hover is about 1.2 kW (it was 3.5 kW before the ESC bus-current
correction — see `physics/README.md`). The placeholder budget comes out at several hundred km maximum
range, which is a property of the placeholder wing and pusher, not of Sage; it demonstrates the
machinery only.

**Tests** (`test_energy_descent.py`): the usable energy of the nominal pack (1728 Wh), the
**hand-calculated reference case** (the M4 gate: maximum range 135.33 km, margins 424 Wh at 100 km
and 268 Wh at 113 km), hover and cruise trims reproducing zero acceleration and the same bus voltage
and current in the simulated vehicle, hover power above the momentum-theory ideal, climb energy above
the potential energy, the glide balance, best-range speed minimising energy per metre, reserve
scaling, and an end-to-end budget.

---

## Q2 — pusher forward acceleration (`pusher.py`)

**Question.** The controller matches the rotors to whatever lift the wing does not provide until
`AIRSPEED_MIN` is reached, however the wing produces it. So what forward acceleration can the pusher
deliver at `TKOFF_THR_MAX`, from rest to `AIRSPEED_MIN`, at each site (altitude, state of charge,
mass)? It replaces the old "full throttle, L = W, one speed" thrust margin.

**Model** (`forward_accel(vp, rho, v_end, soc, cmd, n)`), at each speed on `linspace(0, v_end, n)`:

* the wing carries the share `(V / V_lw)^2` of the weight, `V_lw = V_stall,1g / sqrt(0.8)`, **E's own
  rule** (`energy.rotor_share`), so Q2 and E agree; the rotors carry the rest, at forward-flight
  inflow `V`, and load the same battery bus (a fixed-point on the bus voltage);
* the pusher command `TKOFF_THR_MAX` goes through the ESC dead zone (0.8 is an effective 0.789); its
  speed solves motor torque = propeller torque at the loaded bus voltage;
* net acceleration `(T_p - D - H_r) / m`, with wing drag for the lift the wing carries.

Quasi-steady: level flight, no spool dynamics, no wind. It is the available acceleration, not a
trajectory. `FwdAccel` gives the profile, `a_min` (worst over speed, where E and the worst-case table
read), `a_equiv` (constant acceleration covering the same distance), time and distance, `v_reach`
(how far the pusher gets with positive acceleration), `reachable`, and flags for rotor saturation and
current limits. An unreachable cell keeps a row with `a_min <= 0` and NaN `a_equiv`, `t_s`, `x_m`.

**Tables.** `Q2_pusher_margin` (one row per altitude x mass factor x SoC x convention), `Q2_worst`
(the smallest `a_min` per convention), and `Q2_profile` (`Q2.write_profile`, off by default). Every row
records `tkoff_thr_max` and its status; if the register entry is absent the 0.7 built-in is used and
the row says `fallback (placeholder)`.

**Airspeed convention.** `AIRSPEED_MIN` is registered as a true airspeed, so `tas` is the default feed
to E, but the old Q2 read it as equivalent; both are tabulated until DEC-1 is settled. At 5000 m the
two differ by 29 percent (17 EAS is 21.9 TAS).

**What it found (register values, `TKOFF_THR_MAX` 0.8, 17 m/s).** At E's own site (30 m, SoC 0.6,
nominal mass) `a_min` is 1.24 m/s² (distance-equivalent 1.86, 7.9 s, 78 m); the command matters a
great deal: 2.68 / 1.94 / 1.24 / 0.59 at 1.0 / 0.9 / 0.8 / 0.7. Over the grid with TAS the global
worst is 0.48 m/s² (5000 m, mass x1.15, SoC 0.3). With EAS that cell cannot reach 21.9 m/s TAS (the
acceleration reaches zero near 20.3 m/s). Neither current limit nor rotor saturation is active
anywhere on the grid. In E the choice of `a_fwd` moves the forward-transition energy by a factor of
2.4 (10.0 to 26.5 Wh) and the 1500 m maximum range by under 1 km; the distance it needs (78 to
304 m) matters more. `worst` is pessimistic on purpose: a constant minimum overstates the time and
distance of the real profile.

**Caveats.** The battery is a placeholder (linear open-circuit voltage, assumed R0/R1/C1, no pack
current limit; the pusher and rotor `I_lim` are the only current limits). `TKOFF_THR_MAX` is a TBD
register value and the result is very sensitive to it. The lift share is E's rule, not the wing polar.

---

## C — restricted-parameter optimisation (`restricted.py`)

**Question.** What can the autopilot be *configured* to do, and how far is that from the bound?

**Entry.** C starts at the AIRBRAKE exit, like D: the same entry state (`braking.airbrake_entry_state`),
the same entry speed rule (`braking.entry_airspeed`, `max(AIRSPEED_MIN - 2, Q_ASSIST_SPEED)`) and
t = 0 at the exit. `theta_fw0_deg`, `pit_lim_mode`, `q_assist_speed_ms` and `entry_convention` are
read from the **D block** (they describe the exit, not C); `io.stages.exit_params` builds the
controller parameters for both so they cannot drift. There is no forward transition, no cruise and
no planner simulation. The position cascade runs (`brake_at_limit = False`); the at-limit run is
used only to report a ceiling (below).

**Four variables** (`restricted.C_BOUNDS`, widened or narrowed in `C.bounds`):

| Name | ArduPilot parameter | Range |
| --- | --- | --- |
| `T_bt` | `Q_BACKTRANS_MS` | 1–6 s |
| `theta_A` | `Q_A_ANGLE_MAX` | 10°–30° (30° is DEC-7 and binding) |
| `spin_min` | `Q_M_SPIN_MIN` | 0.06–0.25 |
| `q` | `Q_TRANS_DECEL` | 1.0–3.0 m/s² |

**`q` does both jobs it does in ArduPilot.** It places the stopping point, `x_tgt = V_ground² /
(2 q)` (O-02), and it is the cascade's velocity-profile acceleration limit `a_lim = q` (O-11). The
old split (a planner run placing the target, a fixed `a_lim` braking along it) is gone. Left alone the
search would pick a `q` above what the pitch ramp can deliver, so two flags are added to the usual
failures (cost penalty 10 each): **`overshoot`** (the run stopped more than `C.os_tol_m` past
`x_tgt`) and **`sinking`** (the end climb rate is below `-C.sink_tol_ms`). Both tolerances are
proposals, and **the overshoot tolerance decides the answer**: the optimum sits on it. The ramp is
time-driven, so what it can deliver depends on the ground speed; `capability_ceiling` runs the same
case with the pitch at the ramp limit and reports `a_cap = (V_ground² - 1) / (2 x_end)`, D's `a_eq`
under the same Z loop and lag, once per ensemble member at the optimum. `C_runs` has `a_cap_ms2` and
`q_over_a_cap` so `q` can be read against D's `a_rec` curve (`D_a_rec_curve`) by eye; C's `q` does
not feed E, which keeps taking D's recommendation.

**The vertical loop is ArduPilot's structure with ArduPilot's default values** (`z_mode =
"ardupilot"`, open question O-12; the repo's `K_h`, `K_vh`, `K_i` hold stays the dataclass default
for every other caller): position P → velocity P → an acceleration PID on the *measured*
vertical acceleration, giving a normalised thrust that is turned into a rotor command through a
thrust table (`LiftFF.thrust_for_delta`) with tilt compensation. It has no lift or hover
feed-forward, so `ff_mode` is ignored. Its equivalent velocity gain is about 0.16 to 0.18 throttle
per m/s, eight to nine times the repo hold's `K_vh` (0.02). The defaults are ArduPilot master (4.8.0-dev)
Plane values read on 2026-10-27, **not a pinned tag** (O-08). It is a mapping, not ArduPilot:
`thrust_norm` is linear in rotor thrust (no `MOT_THST_EXPO`), the acceleration is the exact vertical
acceleration of the state (no noise, bias or delay), and the integrator starts at the entry thrust.
`C_sens` shows the sensitivity to the two gains that matter (`z_vel_p`, `z_acc_p`).

**Rotors at the exit.** `C.rotor_start = "idle"` starts the rotors at the thrust of
`Q_M_SPIN_MIN` (O-13: ArduPilot relaxes the Z controller in APPROACH and AIRBRAKE, so the throttle has
decayed towards idle by the time POSITION1 starts). **O-13 is read from source and not checked in SITL
or a log**; `"carry"` (D's assumption, W − L) is the sensitivity. The result barely depends on it.
`braking.airbrake_entry_state(..., rotor_start=)` and `braking.entry_rotor_thrust` implement both;
D's default stays `"carry"`.

**Rotor lag.** `C.rotor_lag_s` (0.25 s) is D's one lumped actuator lag (D, section 14), C's own value
because the Z loop is fast.

**Cost.** (10.1) with the normalisers `h_ref` 2 m, `E_ref` 15 kJ, `x_ref` 100 m (`C.weights`; the old
10 m / 5 kJ / 150 m make a sub-metre excursion irrelevant). The weights (1, 0.3, 0.5) and the
penalty 10 are unchanged. `ensemble.ensemble_cost` and `RunMetrics` are used as they are.

**Search.** `optimise_c(vp, scens, cev, base_bp, weights, start, bounds=…, seeds, popsize, maxiter)`
is CMA-ES from the register values with `q = 2`. `result.at_bound` names every variable within 10 %
of its range of a bound (`theta_A` and `q` are the ones to watch). Evaluations are serial (CasADi
functions are not picklable across processes) and cached per vector; one ensemble evaluation of 32 runs
takes 5 to 14 s, so the shipped search (8 seeds, `popsize` 8, `maxiter` 20) is a few hours on one core.
Record `result.search.spread` as `search_spreads={"C": …}` so the release gate can check it.

**Tables.**

| Table | Content |
| --- | --- |
| `C_params` | the optimum and the register values at `q = 2`: the four variables, `J`, `J_start`, seed spread, `at_bound`, p95 excursion, mean energy (kJ), p95 distance, max overshoot, flag counts |
| `C_search` | per seed: best cost, evaluations |
| `C_runs` | per ensemble member at the optimum: inputs, `x_tgt`, `x_end`, overshoot, energy, excursion, end sink rate, flags, `a_cap_ms2`, `q_over_a_cap`, and D's `braking_flags` (alpha above the 0.8 C_L,max band, VRS ratio; reporting only, not in `J`) |
| `C_q_curve` | the optimum's `T_bt`, `theta_A`, `spin_min` with `q` varied (`C.q_curve`): the energy-against-distance curve, from which any overshoot tolerance can be read |
| `C_sens` | one at a time at the optimum: `rotor_lag_s`, `rotor_start`, `z_vel_p`, `z_acc_p`, `theta_fw0_deg` |

**What is no longer optimised:** `AIRSPEED_MIN`, `Q_TRANSITION_MS` and `TKOFF_THR_MAX` (Q2 and E
cover part of the forward side; nothing optimises `AIRSPEED_MIN` now, and if it comes back it is a
fifth variable of this search, with the entry speed following it). Also not in the vector:
`PTCH_LIM_MAX_DEG` (fixed at 35°), `Q_TRANS_FAIL` (off), `BATT_WATT_MAX` (not modelled).

**Legacy.** `evaluate_back`, `plan_decel`, `evaluate_full`, `optimise`, `BACK_BOUNDS`, `FULL_BOUNDS`,
`DEFAULT_PARAMS` and `THETA_P` are untouched and marked legacy in their docstrings.
`analysis/sensitivity.py` and `analysis/collocation.py` still call `evaluate_back`, `plan_decel`,
`DEFAULT_PARAMS` and `THETA_P`; their migration is a separate step. **B and C are not comparable yet**:
the B-to-C comparison (B-9: B2 2.341 against patched C 2.256) stays on the old level-cruise entry until
B1 and B2 start at the AIRBRAKE exit. C's `x` is the distance from the AIRBRAKE exit to the stop; it
leaves out the APPROACH/AIRBRAKE leg and the `2 * V_closing` term of the switch test (O-02), which belong
to E's budget.

**Limits.** The end state is only a sink-rate check (DEC-5 lean, 1.76 m/s): there is no landing-error
criterion (DEC-4) and no check on position or heading at the stop. The entry state still has
placeholders (`theta_fw0_deg` = 0; `AIRSPEED_MIN` read as a true airspeed, DEC-1). `theta_A` against its
30° bound is a stalled-wing brake as in D: read D's alpha and VRS flags next to it, and do not read the
result as a recommendation to fly at 30°.

---

## B2 — speed-scheduled law (`freeform_opt.py`)

Tunes the 13 parameters of the free-form law (6 rotor-throttle knots, 6 pitch knots, `s_on`) over an
ensemble with the same cost and search as C, **back-transition only**. There are two starts:

* `init_from_zero_loss` (the default): analysis A's pre-spun schedule re-plotted against `V/V_s`, computed with `LEGACY_LIMITS` (no angle band) so B2's start does not depend on A's band;
  where A is infeasible the knots fall back to fixed defaults.
* `init_from_collocation(prob, sol)`: a B1 solution re-plotted against `V/V_s` (the proposal's
  step 2), with the rotor throttle converted from B1's effective throttle to the ESC command and
  `s_on` read from where B1 first switches the rotors on. Pass it as `optimise(..., init_params=…)`.

`optimise` also takes `sigma0`, CMA-ES's initial step size on the unit cube (default 0.25).

**What it found (placeholder, the same 4-scenario ensemble as C; 180 evaluations per seed unless
stated).**

| Start | Start cost | Best cost | Seed spread | T-12 (≤ 1 %) |
| --- | --- | --- | --- | --- |
| A-based, `sigma0` 0.25 | 8.68 | 1.94 | 4.4 % | fails |
| A-based, `sigma0` 0.25, 540 evaluations | 8.68 | 1.60 | 9 % | fails |
| B1-based, `sigma0` 0.25 | **1.90** | 1.75 | 8.7 % | fails |
| B1-based, `sigma0` 0.10 | 1.90 | **1.69** | 5.0 % | fails |
| B1-based, `sigma0` 0.04 | 1.90 | 1.75 | 5.3 % | fails |
| *C for comparison* | 2.09 | 2.00 | 0.10 % | passes |

The B1 start is far better as a *starting point* (cost 1.90 against 8.68 — already below C's best,
because it begins near a good open-loop trajectory) and reaches lower costs at the same budget, but
**B2's optimum is still not repeatable from three seeds, so T-12 is not satisfied for B2.** With the
default step size one seed never left the start; smaller steps help (spread 8.7 % → 5 %) but not
enough. The landscape (13 dimensions, no altitude feedback, discontinuous failure penalties) probably
needs a much larger budget, a different search setup, or fewer free parameters. The B2 free-form law
reached an excursion of about 8–10 m against C's 12–14 m, so the result is lower-cost than C but not
reproducible. **The release gate blocks any run record that includes `"B2"` without a spread of at
most 1 %** (see `io/README.md`).

---

## B1 — open-loop optimum by direct collocation (`collocation.py`)

**Question.** The best trajectory — the physical bound — for one scenario, as a Pareto surface of
energy, altitude excursion and distance.

**Formulation.** Hermite–Simpson collocation of the single CasADi vehicle (with smoothing), free final
time, solved with IPOPT through CasADi's `Opti`. States are the 9 vehicle states; the unknowns are the
rotor-throttle and pitch-command histories and the final time. The pusher stays off.

```
minimise  E = integral V_bus I_bus dt   (+ a small control-rate regulariser)
subject to dynamics (HS defects), |h - h_ref| <= eps_h at nodes AND midpoints,
           ground distance <= eps_x,   vx(T_f) = V_f,  |vh(T_f)| <= 0.5,
           0 <= delta_r <= 0.95, |theta_cmd| <= 0.6, throttle and pitch slew limits,
           V_bus >= V_min, |vh| <= 3 m/s, rotor speed >= 0,
           stall margin |alpha_w| <= 0.85 alpha_s while V >= V_s (nodes AND midpoints)
```

The second objective, `"excursion"`, minimises the maximum altitude deviation instead (the
`epsilon_h*` of the Pareto front).

### What it took to make it work — read this before changing it

The first versions converged poorly, and then converged to solutions that **did not survive replay in
`simulate()`**. Each issue below was diagnosed, not guessed:

1. **Speed.** An exact Hessian through the MX graph took minutes. `expand=True` (SX expansion) cut a
   solve from 42 s to 3.8 s. Limited-memory Hessians were fast but stalled.
2. **Mesh refinement.** Large meshes converge poorly from a rough guess; `solve_refined` solves on a
   coarse mesh first (default 20 intervals) and warm-starts the finer mesh.
3. **Graded mesh** (`grading=0.5`): finer steps at both ends, where the rotors spool and the pusher
   speed decays.
4. **ESC dead-zone dither.** The optimiser kept the rotor throttle inside the dead zone, where the
   smoothed corner leaks a little torque that Hermite–Simpson integrates differently from the
   simulator. **Fix: the NLP's rotor control is the *effective* throttle** (dead zone removed from the
   NLP vehicle); the replay maps back with `command = dz + (1 − dz) * effective`.
5. **Stall cliff.** The wing's stall blend is about 0.01 rad wide, and the optimiser flew a pitch-up
   through it between nodes, where Hermite–Simpson cannot see it; replays disagreed by metres.
   **Fix: a stall margin (0.85 of the stall angle), enforced at collocation midpoints as well as
   nodes**, and the altitude band is enforced at midpoints too.
6. **Pitch pumping and flat optima.** `|vh| <= 3 m/s` and a small control-rate regulariser
   (`reg=1e-2`) removed violent pumping and conditioned the energy-flat optimum.
7. **Smoothing leakage.** The current-clip smoothing width biases energy; at 0.5 A the replay energy
   differed by about 2 %, at the default 0.02 A by 0.4–0.7 %.

### Replay and verification (T-11)

`replay(prob, sol)` re-simulates the controls with the near-exact vehicle and RK4 at 0.02 s and compares
energy, distance, duration and excursion (`ReplayCheck.passes`): energy, distance and duration within
2 %, and the excursion within 2 % **or 0.3 m** (a small difference of large integrals that agrees only
to about 0.25 m at N = 40; this is a deliberate deviation from a flat 2 %). A solution that fails its
replay is not trusted. `solve_verified` tries several initial guesses (baseline-controller runs with
different pitch limits and ramp times) and returns the best result that both **converged and replays**;
every attempt is kept, and non-convergence is recorded, never raised or hidden. `pareto_sweep` builds
the epsilon-constraint front this way; `min_excursion` is the least reliable mode (a converged
excursion-objective solution once reported 0.0 m at the nodes while the replay showed several metres)
and its result must be checked via `Verified.verified`.

### Comparing with the baseline, C and B2

`bound_check(prob)` solves B1 at the baseline's own excursion and distance **under B1's operational
limits** and reports the energy margin; `compare(prob, b1, c_params, b2_params)` tabulates B1, C and B2.

**B1 is *not* a strict lower bound.** It keeps limits the baseline and B2 do not respect (`|vh|`, stall
margin, slew limits), so a controller that breaks them can be cheaper at the same excursion — and
tuned B2 was, marginally. Relaxing the limits makes the NLP much harder (none of three retries
converged). The claim that *can* be made is one-sided: if B1 under its tighter limits already beats the
baseline, the true bound is at least that good.

**What it found (placeholder, 22 m/s, sea level).**

| Case | Excursion | Energy | Distance |
| --- | --- | --- | --- |
| B1, 20 m band (nearly unconstrained) | 16 m | 2.11 kJ | 83 m |
| B1, 10 m band | 10 m | 2.34 kJ | 129 m |
| B1, 5 m band | 5 m | 2.56 kJ | 176 m |
| Baseline, tuned (C) | 13.4 m | 3.21 kJ | 93 m |
| Free-form law, tuned (B2) | 8.1 m | 2.37 kJ | 168 m |
| Baseline, default, vs B1 at the same excursion/distance | 12.4 m | 4.53 kJ vs 2.23 kJ | 121 m vs 108 m |

The cost of a tighter altitude band is visible in the first three rows. The baseline spends about twice
the energy of B1 at the same excursion and distance.

**Known weaknesses.** Convergence is fragile, especially for infeasible caps (a 100 m or 80 m stopping
distance from 22 m/s). Solves at 100–120 intervals work with refinement but take 140–160 s. Not
implemented: a sourced vortex-ring exclusion, five initial guesses per point (three are used), and the
free rotor start is an option (`prespin_max`) whose pre-entry spin-up energy is **not** counted.

**Tests** (`test_m6.py`): the graded mesh, the dead-zone mapping in the open-loop controller, that the
solution satisfies every constraint, **T-11**, the Pareto trend (a tighter band costs energy and time),
non-convergence being recorded, the baseline comparison, the replay check rejecting a doctored
solution, and that sweeps keep every attempt.

---

## `sensitivity.py` — what drives the answers

**Inputs.** `default_inputs(vp, scen)` returns 21 `UncertainInput`s, each with a nominal value, a
`[lo, hi]` range and the register's **evidence grade**: mass and wing area (TBD), the wing coefficients,
rotor thrust and torque coefficients, rotor inertia, motor constants, ESC dead zone and pack resistance
(mostly TBD or assumed), the pitch-lag parameters, the altitude-hold gains, and the wind, ISA offset and
state-of-charge scenario axes. Relative ranges default to ±20 %; scenario axes use absolute ranges.

**Outputs.** `HeadlineModel` maps input values to the back-transition's `excursion_m`, `distance_m`,
`energy_J` and, optionally, `max_range_km` from the energy budget.

**Methods.**

* `oat(model)` — one-at-a-time at half and full range (±10 % and ±20 % by default), for the tornado
  chart; `tornado(rows, output)` sorts by swing.
* `global_morris(model, r)` — Morris screening; `global_sobol(model, n)` — Sobol first- and total-order
  indices (SALib). Both fill a **failed** run with the *worst* finite value of that output in its own
  direction (largest for excursion/distance/energy, smallest for range). An earlier version used the
  largest value for every output, which would have treated an infeasible run as the longest range.
* `run_sensitivity(model, r_small, r_large)` — OAT plus Morris at two sizes, with `rank_stability`
  (Spearman correlation and top-k overlap between the two rankings) and `dominating(...)` (the top-k
  inputs, plus any carrying at least 10 % of the summed index).
* `weakest_status(statuses)` grades a result by its dominating inputs' weakest evidence.

**What it found (placeholder, all 21 inputs, Morris at 6 and 12 trajectories; re-run after the
review fixes, about 105 s).** Rankings were stable: Spearman 0.96–0.99 and top-3 overlap 1.0 for
every output. Dominating inputs: for **excursion**, wing area, the lift-curve slope, `CD0` and the
altitude-hold gain; for **distance**, headwind (it changes the ground speed at entry), wing area and
mass, with the stall angle close behind; for **energy**, mass, headwind and the rotor thrust
coefficient; for **range** (measured in the earlier run with the energy budget attached), wing area,
mass and `CD0`. Mass and wing area are TBD in the register, so the weakest grade among the dominating
inputs is TBD for every output. Since the review fixes, the planner inside `evaluate_back` is the
simulated `a_eq`, which roughly doubles the cost of each run (0.23 s against 0.13 s).

**Tests** (`test_m7.py`): Morris recovers a linear model's coefficients exactly; Sobol total-order
indices match the analytic variance shares (0.2 and 0.8) within 0.05; rank stability; OAT rows match direct
evaluation; the failed-run fill direction; the real model responds in the physical direction; and the
**FR-08 ranking stability** on a 7-input subset.

---

## Run records

Every result object carries `record: RunRecord` (`input_hash`, `code_version`, `date`, `label`). The
hash covers the vehicle parameters, scenario and all options, so two results with the same hash and
code version are reproducible from each other. All records are `DRAFT` until the register feeds the
physics. See `io/README.md` for how they are combined into the file written next to an output.
