# `sagetrans.analysis` — what the toolkit computes

Layer position: above `control`, below `io`. Each analysis is a function from a vehicle, a
scenario and options to a results object. Analysis letters follow the proposal's section 10.

> All results quoted below were obtained with the **placeholder vehicle** and are `DRAFT`. They
> show that the analyses run and behave sensibly, and what they found *in the model*; they are not
> conclusions about Sage.

| Module | Proposal | Question answered |
| --- | --- | --- |
| `zero_loss.py` | **A** — zero-altitude-loss reference and feasibility | Can the hardware hold altitude through a given deceleration? What binds first? |
| `collocation.py` | **B1** — open-loop optimum by direct collocation | What is the best trajectory under the constraints? (energy against excursion and distance) |
| `freeform_opt.py` | **B2** — speed-scheduled law tuned over an ensemble | What does a robust, feedback-free speed law achieve? |
| `restricted.py` | **C** — ArduPilot-parameter optimisation | What can the autopilot be *configured* to do? |
| `braking.py` | **D** — Position1 braking and `Q_TRANS_DECEL` | How far does the aircraft need to brake, and what deceleration should the planner assume? |
| `energy.py` | **E** — mission energy budget and reserve | Does the mission fit in the usable pack energy at 100 and 113 km (Q1, Q7)? |
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

## A — zero-altitude-loss reference and feasibility (`zero_loss.py`)

**Question.** If the aircraft must hold altitude exactly while decelerating, what throttle and
pitch does that take, and can the actuators deliver it?

**Method.** Prescribe the airspeed history from `V0` and a deceleration profile with `vh = 0`,
`gamma = 0`, no wind, pusher thrust fixed (default zero). Eliminating the rotor thrust from the
force balance leaves one equation in pitch:

```
R(theta) = (T_p - H) + (m a - D(theta)) cos(theta) - (W - L(theta)) sin(theta) = 0
```

which reduces to eq. (10.3), `tan(theta) = (m a - D)/(W - L)`, `T_r = sqrt((m a - D)^2 + (W - L)^2)`,
when `H = T_p = 0`, and to `tan(theta) = a/g` at low speed. (`H` is the rotors' in-plane force; it
depends on the rotor speed, so it is iterated.) The rotor map is then inverted for the rotor speed
that gives `T_r` (`dynamics.trim`), and the motor equation for the throttle **including the
rotor-acceleration term** `I_rot * omega_dot`. Roots of `R` are found on a 61-point grid then refined;
continuity of `theta` along the profile picks the root, preferring those with `T_r >= 0`.

**Entry discontinuity — the important design point.** At entry the wing carries the weight and the
rotors are at rest, so `T_r` and `omega` must jump from their entry values to the schedule's first
values, and `theta` must jump from level trim. If the schedule were evaluated pointwise and
differentiated, this jump would be invisible and the torque, current and pitch-rate checks would
pass wrongly. So every result carries **two verdicts**:

* **whole segment, including the entry step** (`feasible_whole`) — almost always infeasible, by
  design: it is the argument for treating the rotor start speed `V_on` as a decision variable;
* **with the rotors pre-spun** to the schedule's first value (`feasible_pre_spun`) — the
  meaningful hardware-limit verdict, and the one `max_feasible_decel` uses.

**Checks** (each a normalised margin, feasible when `>= 0` everywhere): rotor throttle ceiling
(`ActuatorLimits.delta_r_max` 0.95), motor current limit, bus voltage floor, torque floor (the ESC
cannot brake the rotor), thrust sign (`T_r >= 0`), pitch limit (0.6 rad), pitch rate against
`q_max`, wing angle of attack against stall, and "pitch solution exists". The first binding
constraint and its time are reported.

**API.**

| Name | Purpose |
| --- | --- |
| `DecelProfile(a, jerk=None, v_end=1.0).sample(v0, n)` | constant or jerk-limited deceleration; returns `t, V, a(t)` |
| `EntryState(omega_r=0, theta=None, v1=0)` | the state before the first sample (`theta=None` = level trim) |
| `solve_point(vp, rho, V, a, ...)` | one pointwise solution: `theta`, `T_r`, `H_r`, `omega`, `L`, `D`, `ok` |
| `analyse(vp, scen, v0, profile, limits, entry, t_p, n=120)` | the full result (`ZeroLossResult`) |
| `max_feasible_decel(...)` | bisection on `a` for the largest feasible constant deceleration (0 if none) |
| `feasibility_map(...)` | both verdicts and the first binding constraint over a grid of `(V0, a)` |

**What it found (placeholder).** At 12 m/s and 1.5 m/s² the pre-spun schedule is feasible while the
whole segment is not. At higher speed the first binding constraint is *thrust sign*, not propulsion:
the pitch needed to brake pushes the wing's lift above the weight, so the rotors would have to push
down. That limit depends only on `V/V_s`, so it is the same at sea level and 5,000 m at equal
equivalent airspeed (tested); the throttle demand, in contrast, rises in thin air.

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
a_rec = min over scenarios and V0 of a_eq                                                   (10.5)
```

`a_eq` uses `V0^2 - V_f^2` (with the end speed `V_f = 1 m/s`) so it equals eq. (10.5) when `V_f = 0`.

**Why planner and aircraft disagree.** The planner assumes constant deceleration from the first
instant; the pitch envelope starts at zero and ramps, so the aircraft's available deceleration
`g tan(theta_lim(t))` is below the plan early on and it covers more distance.

**API.**

| Name | Purpose |
| --- | --- |
| `ideal_stopping(v0, theta_max, t_bt, v_end=0, dt=1e-4)` | the planner worked example: ideal pitch, no lift/drag, braking at the limit throughout; returns `(distance, a_eq, ramp distance)` |
| `run_braking(vp, scen, v0, bp, ...)` | simulate Position1 from ground speed `v0` to `V_f`; returns a `BrakingResult` |
| `measure_a_eq(...)` | the worst case: pitch at the envelope limit throughout |
| `recommend_decel(vp, scenarios, v0_grid, bp, percentile=None)` | `a_rec` as the minimum (or a lower percentile) over the grid |
| `landing_error(vp, scen, v0, a_plan, bp, speed_test)` | switch at `d = V_ref^2/(2 a_plan)` and brake with the position cascade; `speed_test` is `"ground"` or `"air"` |

**Scope decision.** Each run **starts at the switch point**; there is no cruise phase here (that is
analysis C's `full` scope). The target is placed `V_ref^2/(2 a_plan)` ahead. `V_ref` is the ground
or the air speed (open question O-02): in a headwind the air-speed test switches earlier.

**What it found (placeholder).** `ideal_stopping(25 m/s, 20°)` gives 87.5 m and `a_eq = 3.57 m/s²`
without the ramp, and 124.5 m / 2.51 m/s² with a 3 s ramp (69.8 m during the ramp) — matching the
proposal's worked example. The simulated worst-case `a_eq` was about 2.6 m/s², 3 % *above* the
ideal figure, because wing drag helps. The proposal calls the ideal figure an upper bound on `a_eq`;
with this wing that is not strictly true. The altitude excursion is a **gain** of about 18 m at
25 m/s (nose-up lift), no loss. At `a_plan = 2.5 m/s²` the 25 m/s case overshoots by about 22 m;
switching earlier reduces it.

**Tests:** the planner example (87.5/124.5/69.8 m), the simulated limit case where `a_eq` equals
`g tan(theta)` within 0.5 % (frozen-state harness, no ramp), the signed excursion, the altitude
hold in hover, `recommend_decel` being the minimum, earlier switching reducing overshoot, and the
air-speed test switching earlier in a headwind.

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

| Segment | How it is computed |
| --- | --- |
| Take-off | hover power times (pre-lift-off hover + vertical climb to clearance) plus `m g h` divided by the electrical-to-lift efficiency |
| Forward transition | **a quasi-steady estimate** (constant acceleration `a_fwd`, rotors carrying `W − L`, pusher supplying `m a + D`); to be replaced by the simulated transition |
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

**Assumptions to know about:** one leg at one mass (a return leg is a second call); every segment
uses one **mean state of charge** (`soc_mean`) rather than tracking the open-circuit voltage; the
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

## C — restricted-parameter optimisation (`restricted.py`)

**Question.** What can the autopilot be *configured* to do, and how far is that from the bound?

Decision vector, with bounds:

| Name | ArduPilot parameter | Range |
| --- | --- | --- |
| `T_bt` | `Q_BACKTRANS_MS` | 1–6 s |
| `theta_A` | `Q_A_ANGLE_MAX` | 10°–30° |
| `spin_min` | `Q_M_SPIN_MIN` | 0.06–0.25 |
| `airspeed_min` (full scope) | `AIRSPEED_MIN` | 11–18 m/s |
| `q_transition` (full scope) | `Q_TRANSITION_MS` | 2–8 s |
| `tkoff_thr_max` (full scope) | `TKOFF_THR_MAX` | 0.5–1.0 |

**Two scopes.** `back` evaluates the back-transition only from an idealised level cruise (directly
comparable with B2). `full` flies the whole sequence from hover (`evaluate_full`): forward
transition, cruise stub, back-transition, with the cost taken as the worst excursion of the two
transitions, summed energy, and the back-transition distance.

**Not in the decision vector:** `PTCH_LIM_MAX_DEG` (fixed at 35° so `Q_A_ANGLE_MAX` binds),
`Q_TRANS_FAIL` (off), `BATT_WATT_MAX` (not modelled). **`Q_TRANS_DECEL` is not decided:** each
evaluation sets it from `braking.ideal_stopping` (the ideal-pitch planner). The proposal asks for
the *simulated* `a_eq` from analysis D; the ideal planner is a cheap stand-in and the real
figure is slightly different.

`optimise(vp, scens, scope, weights, seeds, popsize, maxiter)` runs CMA-ES from a mid-range default
and returns the best parameters, the cost, the default's cost, per-scenario metrics and the search.
`evaluate_back` also takes `controller_overrides` (used by the sensitivity analysis to vary
controller gains).

**What it found (placeholder, 4-scenario ensemble).** The cost fell from 2.05 to 1.96. The optimiser
drove `T_bt` and `spin_min` to their lower bounds; the altitude excursion stayed near 12–14 m, i.e.
the parameters the autopilot offers barely move it. **Three seeds agreed to a spread of 2e-5** (T-12
satisfied for C). A 3-seed run took about 2 minutes.

---

## B2 — speed-scheduled law (`freeform_opt.py`)

Tunes the 13 parameters of the free-form law (6 rotor-throttle knots, 6 pitch knots, `s_on`) over an
ensemble with the same cost and search as C, **back-transition only**. B1 does not exist at the time
of the initial design, so the starting knots come from analysis A's pre-spun schedule re-plotted
against `V/V_s` (`init_from_zero_loss`); where A is infeasible they fall back to fixed defaults.

**What it found (placeholder).** The initial knots cost 8.7, driven by excursions of 70 m or more in two of the four scenarios. After 180 evaluations
per seed the best cost was 1.94 and after 540 it was 1.60, against C's 1.96 on the same ensemble: the
free-form law reached an excursion of about 8–10 m against C's 12–14 m. **But the seeds did not agree:
the spread was 4.4 % at 180 evaluations and 9 % at 540. T-12 is therefore *not* satisfied for B2.**
The landscape (13 dimensions, no feedback, discontinuous penalties, a start far from the optimum) needs
a much larger budget or a better start such as B1's solution.

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

**What it found (placeholder, all 21 inputs, Morris at 6 and 12 trajectories).** Rankings were stable:
Spearman 0.96–0.99 and top-3 overlap 1.0 for every output. Dominating inputs: for **excursion**, the
lift-curve slope, wing area, `CD0` and the altitude-hold gain; for **distance**, headwind (it changes the
ground speed at entry), wing area and stall angle; for **energy**, mass, headwind and the rotor thrust
coefficient; for **range**, wing area, mass and `CD0`. Mass and wing area are TBD in the register, so the
weakest grade among the dominating inputs is TBD for every output.

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
