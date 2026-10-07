# Project report — Sage VTOL transition and landing-approach toolkit

Status at the time of writing: milestones **M0–M7 are implemented**; M8 (calibration against
iron-bird data and software-in-the-loop) is not started because it needs data that does not exist
yet. **144 tests pass**, and `ruff`, `mypy --strict` and the `import-linter` layering contract are
clean. M0–M7 are in commit `d9f9966`; the review follow-up described in section 4a is in the commit
after it (`Fix review findings…`). Both are on `origin/master`.

This document explains what was done, in what order, what went wrong and how it was fixed, what the
numbers mean, where the code departs from the design (`proposal.md`), and what remains. Each package
also has its own `README.md` with the detail of its modules (`src/sagetrans/README.md` and one in
each sub-package, plus `tests/README.md`).

> **`proposal.md` is not in the repository.** It is listed in `.gitignore`, so it is kept locally
> and a fresh clone will not contain it. Equation and section numbers cited here and in the READMEs
> (for example "eq. 6.11", "§10D") refer to that local file.

---

## 1. Executive summary

**What exists.** A Python package, `sagetrans`, that simulates a 4+1 VTOL aircraft in the vertical
plane and answers the seven questions of the proposal's section 1: whether the mission energy fits
(Q1, Q7), whether the pusher can reach the transition speed (Q2, partially), how much altitude is lost
or gained and what the electrical peaks are (Q3), how far the back-transition needs (Q4), what
`Q_TRANS_DECEL` and related autopilot parameters to set (Q5), and what descent rate vortex-ring-state
avoidance imposes (Q6). It has seven analyses (A, B1, B2, C, D, E, F), a sensitivity analysis, a
report generator, run records with file hashes, and a release gate.

**The single most important caveat.** *Every number the toolkit produces is derived from placeholder
physics.* The vehicle parameters (`physics/params.py`) are illustrative assumed values for a notional
8 kg aircraft — not Sage's data — and the parameter register (`configs/vehicle.yaml`) is not yet
connected to the physics. Everything is therefore labelled `DRAFT`, and the release gate blocks it by
design. What this work delivers is *validated machinery*: models that reproduce analytical limits,
analyses that agree with hand calculations, and a pipeline that knows what it does not know.

**What the placeholder model says** (only as a demonstration; see section 7):

* The back-transition **gains** altitude (about 12–14 m for the baseline controller) rather than
  losing it, from nose-up lift while braking — consistent with the proposal's "documented nose-up
  climb on back-transition".
* An open-loop optimum needs about **half the energy** of the baseline controller at the same
  excursion and stopping distance, and the cost of holding the altitude tighter is visible: roughly
  2.1 kJ at a 16 m excursion, 2.3 kJ at 10 m, 2.6 kJ at 5 m.
* The quantities that dominate the headline outputs are mass, wing area, the wing's lift-curve slope
  and zero-lift drag, the rotor thrust coefficient, headwind, and the altitude-hold gains. **Mass and
  wing area are still TBD** in the register — they are the first inputs to pin down.

**Where it is weaker than the proposal.** Optimisation of the free-form law (B2) did **not** meet the
three-seed repeatability test (T-12); the open-loop optimiser (B1) converges less reliably than I would
like and is not a strict lower bound; several proposal items were reduced in scope (section 6). One
equation in the proposal was physically wrong and was corrected (section 5.4).

---

## 2. Background and how the work was organised

The design is `proposal.md` (about 1,000 lines). It specifies the physics, the controllers, nine
analyses and nine milestones M0–M8, each ending at a test gate. The request was to implement the
milestones in order. M0 (repository, configuration schema, atmosphere, continuous integration) was
**already in the repository** when this work began; I read it and built on it but did not write it.

Working method: for each milestone I read the relevant proposal sections, wrote the code, wrote tests
against the proposal's stated gates, ran the tests, `ruff`, `mypy --strict` and `import-linter`,
investigated every surprise rather than adjusting a test to pass, and recorded the findings. For M5 the
instruction was explicit: *"take a basic implementation… just bare minimum with descent assumptions is
fine"*, which is why M5 is reduced in scope. I consulted a second reviewer once, before designing M3;
its advice shaped the entry-discontinuity handling and the decision not to build a cruise phase for
analysis D (section 4, M3).

Two process notes for honesty: a few multi-file shell commands failed to parse in this environment and
I wrote those files individually instead; and some of the headline numbers below (the three-seed runs,
the 21-input sensitivity run, the B1 Pareto table) came from scripts run in a scratch directory outside
the repository, using only the public APIs. They are reproducible (section 10) but are not unit tests.

---

## 3. Repository map

```
sagetrans/
  PROJECT_REPORT.md        this document
  proposal.md              the design (one equation edited, see 5.4)
  pyproject.toml           dependencies, ruff, mypy --strict, import-linter layering contract
  configs/vehicle.yaml     the parameter register (5 TBD entries)
  .github/workflows/ci.yml ruff, mypy, lint-imports, pytest on 3.11 and 3.13
  src/sagetrans/           4,693 lines in 34 files
    README.md              overview, conventions, atmosphere
    atmosphere.py          ISA
    config/                parameter register, units, hashing                 (README)
    physics/               wing, rotors, motor/ESC, battery, attitude, vehicle  (README)
    dynamics/              simulate(), events, steady-flight helpers           (README)
    control/               baseline controller, free-form law                   (README)
    analysis/              analyses A, B1, B2, C, D, E, F, sensitivity, search   (README)
    io/                    run record, results writer, figures, report, gate    (README)
  tests/                   144 tests                                           (README)
```

Layering (lower layers never import higher ones; enforced in CI):
`io → analysis → control → dynamics → physics → atmosphere → config`.

Dependencies added to `pyproject.toml` during this work: `cma` (CMA-ES), `SALib` (sensitivity),
`matplotlib` (figures); `casadi`, `numpy`, `scipy`, `pydantic`, `pyyaml` were already present.

---

## 4. Milestone by milestone

Test counts are cumulative.

### M0 — repository, configuration, atmosphere, CI  *(pre-existing; 40 tests)*

Gate: T-01, T-13, T-14. Present on arrival: the schema with the evidence-grade vocabulary, the unit
converter, the ISA atmosphere that accepts floats, arrays and CasADi expressions, and CI. I confirmed
the 40 tests passed before starting.

### M1 — physics components  *(50 tests)*

**Built.** `physics/`: wing with the smooth stall blend, lift rotors and pusher with placeholder
polynomial coefficient maps, motor and ESC, battery equivalent circuit with a bus-voltage solve, the
pitch-tracking surrogate, and `CasadiVehicle`, the single compiled vehicle function; plus a NumPy
reference implementation (in the tests only) and a speed check.

**Gate.** T-02 (wing limits), T-03 (hover power with figure of merit 1), T-07 (closed-form versus
iterative bus voltage), T-08 (motor steady state), T-15 (NumPy against CasADi to 1e-9) — all pass; one
60 s segment steps in under one second, so the "CasADi only" decision (O-17) is confirmed and the NumPy
fallback was not needed.

**What happened.**

* Two tests failed on the first run — both were *my test's* mistakes (the hover test started at 100 m,
  where density is 1.213 not 1.225; a bus-voltage monotonicity test had every current saturated). I
  fixed the tests, not the model.
* **Stall-blend sharpness.** T-02's 1e-9 tolerance at zero angle of attack requires the sigmoid weight
  to be about 1e-15 there, which needs a sharpness `M` of about 100 per radian with a 0.35 rad stall
  angle. This makes the stall a near-cliff — about 0.01 rad wide — which caused real trouble in M6.
* The clipped bus-voltage solve uses 12 unrolled Newton iterations, validated against a bracketing
  solver on 200 random clipped cases.

### M2 — dynamics and `simulate()`  *(57 tests)*

**Built.** `dynamics/simulate.py` (fixed 0.02 s controller step, compiled RK4 with 4 sub-steps, adaptive
LSODA reference integrator, event detection with linear interpolation, metrics) and `dynamics/events.py`.
The vehicle gained a `frozen_states` test harness used by the limit-case tests.

**Gate.** T-04 (constant-deceleration stopping distance within 0.5 %), T-05 (coast distance), T-06
(energy conserved to 1e-6 over 60 s), T-09 (RK4 against the adaptive reference), T-10 (Cartesian against
flight-path-angle form) — all pass, first time.

**Design choices worth knowing.** The controller sees the *previous* step's auxiliary outputs (a 20 ms
lag, because outputs depend on the command). The controller interface was kept structural so `dynamics`
does not import `control` (the layering rule).

### M3 — analyses A and D, first controller  *(71 tests)*

**Built.** `dynamics/trim.py` (rotor-map inversion, throttle for a given acceleration, hover),
`analysis/zero_loss.py` (A), `analysis/braking.py` (D), `control/` (the `ModeState` interface and the
Position1 controller), `analysis/common.py` (scenarios, run records).

**Before designing M3 I consulted a reviewer, and followed its advice:**

1. *Expose the entry discontinuity in A.* Pointwise solving then differencing would hide the jump in
   rotor speed and pitch at entry and make the torque checks pass wrongly. A therefore reports two
   verdicts per case: the whole segment (almost always infeasible, by design) and the segment with the
   rotors pre-spun.
2. *Do not build a cruise phase for D.* D starts each run at the switch point and places the target
   `V_ref²/(2 a_plan)` ahead; that covers the method, including the ground-versus-air speed comparison,
   without fixed-wing gain tuning. (The reviewer also suggested removing the unused `headwind`
   argument of `ground_speed_below`; I left it in and documented that it is unused.)
3. Concrete gate tests with checked numbers, the `alpha_w = theta + i_w` trap, and integrators that
   advance by elapsed time rather than a stored step.
4. A minimal run record on results, and to tell the user the pusher placeholder was retuned.

**Gate.** A reproduces the analytical bounds (`theta = atan(a/g)`, `T_r = m√(a²+g²)`, the coast case,
eq. 10.3 self-consistency); in the limit case D's `a_eq` equals the imposed deceleration within 0.5 %;
and the planner reproduces the proposal's worked example exactly (87.5 m and 3.57 m/s² without the ramp;
124.5 m, 2.51 m/s² and 69.8 m in the ramp with it).

**What happened and what it found.**

* The placeholder pusher gave no thrust at cruise speed (its thrust coefficient went negative before
  25 m/s), so it was retuned. (It was retuned twice more in M5.)
* One test assumed that thin air shrinks the zero-loss feasible region. The data said otherwise: the
  limiting constraint is aerodynamic and depends only on `V/V_s`, so it is the same at sea level and
  5,000 m at equal equivalent airspeed, while the *throttle demand* does rise. I replaced the test with
  what the physics actually shows.
* **Finding:** the braking simulation shows a signed altitude **gain** of about 18 m at 25 m/s, no loss.
  Simulated `a_eq` is about 3 % *higher* than the ideal planner's, because wing drag helps — so the
  proposal's claim that the ideal figure is an upper bound does not hold exactly with this wing.

### M4 — analyses E and F  *(86 tests)*

**Built.** `analysis/energy.py` (mission budget and reserve), `analysis/descent.py` (descent profile) and
the steady-flight helpers in `trim.py` (steady climb and glide trims, electrical operating points).

**Gate.** A hand-calculated reference case (maximum range 135.33 km, margins 424 Wh at 100 km and 268 Wh
at 113 km) is reproduced; cruise and hover trims reproduce zero acceleration in the simulated vehicle;
the descent time matches a closed-form integral.

**What happened — a bug in the proposal.** The first budget gave a hover power of 3.46 kW for an 8 kg
aircraft; a hand estimate from the rotor torque gave about 1 kW. Tracing it showed that eq. (6.11) took
the **bus current equal to the sum of the motor currents**. The ESC chops the bus: the motor sees
`delta × V_bus`, so the bus carries `delta × I_m`. As printed, the model overstated battery draw by about
`1/delta` — roughly 3× at hover. I corrected the model (`battery.py`, `vehicle.py`, `trim.py`, the test
reference), edited the equation in `proposal.md`, and all earlier tests still passed. Hover power is now
about 1.2 kW. **If you intended the original form, this is the change to revisit** (section 5.4).

`kappa` (the vortex-ring fraction of induced velocity) has no default: the proposal says the working
figure of a quarter to a third is unsourced and must not appear in a result, so the code refuses to run
without an explicit, sourced value.

### M5 — controller, analyses C and B2  *(101 tests)*

**Scope, as instructed: bare minimum.** The baseline controller models the forward transition (wait at
`TKOFF_THR_MAX`, linear rotor fade), a cruise stub, the switch-point test, and Position1. B2 and C are
compared on the **back-transition only** from an idealised cruise; C also has a `full` scope that flies
the whole sequence from hover. `Q_TRANS_DECEL` is set inside each evaluation from the ideal-pitch planner
(a cheap stand-in for D's simulated `a_eq`). Dropped from the decision vector: `PTCH_LIM_MAX_DEG`,
`Q_TRANS_FAIL`, `BATT_WATT_MAX`. Evaluation is serial (CasADi objects cannot be pickled across
processes).

**What happened.**

* The first baseline run overshot to 45 m/s because the placeholder pusher was far too strong; I retuned
  its thrust and torque coefficients. A weaker pusher then needed consistent torque coefficients (a prop
  efficiency of about 70 % at cruise); the third retune did that.
* **The stall flag was wrong.** Every back-transition was flagged as stalled. The cause: the nose-up
  aircraft briefly stalls its wing around 6–10 m/s *while the rotors already carry the weight*, which
  is harmless. Stall now counts only while `V` is at or above the 1g stall speed.
* After braking past the target the Position1 controller drifts backward, because it can only pitch
  nose-up. This is the controller's documented scope (it brakes), and analysis D ends at the first time
  the ground speed reaches `V_f`; but it is why full-sequence runs also end on that event.

**Gate (T-12, repeatability from three seeds) — mixed result.** On an analytic cost with a jump the
machinery passes (all three seeds within 1 %). On the real problems, with a 4-scenario ensemble:

| Analysis | Seeds' spread | Verdict |
| --- | --- | --- |
| C (3 parameters) | **2e-5** | passes |
| B2 (13 parameters), 180 evaluations per seed | 4.4 % | **fails** |
| B2, 540 evaluations per seed | 9 % | **fails** |

B2 reached lower cost than C (1.60 against 1.96) but the seeds did not agree, so its optimum is not
reliably reproduced; it needs a much larger budget or a better starting point (B1's solution, now that
B1 exists).

### M6 — analysis B1, direct collocation  *(110 tests)*

**Built.** `analysis/collocation.py`: Hermite–Simpson collocation of the single CasADi vehicle, free final
time, IPOPT; minimum-energy and minimum-excursion objectives; mesh refinement; baseline-run initial
guesses; replay in the simulator; verified solves; Pareto sweep; comparison with the baseline, C and B2.

**Gate (T-11).** A collocation result replayed in `simulate()` reproduces energy, distance and duration
within 2 % (typically 0.4–2 % energy, 0.5 % distance, 0.3 % duration) and the excursion within 0.3 m.
(I made the excursion tolerance absolute because a 2 % bound on a 3 m excursion is smaller than the
mesh's accuracy.)

**What happened — this milestone took the most diagnosis.** The first solves did not converge, and then
converged to answers that *did not survive replay*: a 3 m collocated excursion replayed as 15 m. The root
causes, found one at a time with node-by-node comparisons of the collocated and simulated states:

1. *Speed.* An exact Hessian through the symbolic graph took minutes per solve; expanding to SX made it
   10× faster. Limited-memory Hessians stalled.
2. *Convergence.* Mesh refinement (solve a coarse mesh, warm-start a finer one) and a graded mesh (finer
   steps at both ends).
3. *Exploiting the ESC dead zone.* The optimiser dithered the throttle inside the dead zone, which
   Hermite–Simpson integrates differently from the simulator. Fix: the NLP's rotor control is the
   **effective** throttle, with the dead zone removed from the NLP vehicle and re-applied in the replay.
4. *Exploiting the stall cliff.* The 0.01 rad-wide stall blend let the optimiser fly a pitch-up through
   it between nodes. Fix: a stall margin enforced at nodes and collocation midpoints.
5. *Violent pitch pumping and flat optima.* An operational `|v_h| ≤ 3 m/s` limit and a small
   control-rate regulariser.
6. *Smoothing leakage.* The current-clip smoothing biased replay energy by 2 % at 0.5 A; at 0.02 A it is
   under 1 %.
7. *Unreliable minimum-excursion mode.* One converged solution claimed 0.0 m at the nodes but replayed at
   several metres. So no collocation result is trusted until it replays: `solve_verified` keeps only
   attempts that both converge and pass the replay check.

**Findings.** B1 is **not a strict lower bound**: it keeps operational limits (`|v_h|`, stall margin, slew
limits) that the baseline and B2 do not, so a controller that breaks them can be cheaper — and tuned B2
marginally was. Relaxing B1's limits to make it a bound made the NLP stop converging. The defensible,
one-sided claim is that B1 *under its tighter limits* already needs about half the baseline's energy at the
same excursion and distance. Convergence also remains fragile for infeasible caps (an 80 m or 100 m
stopping distance from 22 m/s).

### M7 — sensitivity, report generator, release gate  *(130 tests)*

**Built.** `analysis/sensitivity.py` (21 uncertain inputs with evidence grades, one-at-a-time and global
Morris/Sobol through SALib at two sample sizes, rank-stability measure) and the `io` package (run record,
results writer, figures, report, release gate, JUnit reader).

**Gate.** The global ranking is stable across two sample sizes (Spearman 0.96–0.99, top-3 overlap 1.0
for all four outputs on the full 21-input run); every output carries a run record with file hashes; DRAFT
output is refused without an override (T-14); and the release gate's four conditions are each tested.

**What happened.**

* **Bug found by reading the demo output:** failed runs in the global analysis were filled with the
  *largest* finite value of each output. For maximum range, that treated an infeasible run as the *best*
  result. Failed runs are now filled with the worst value in each output's own direction (59 of 264 runs
  failed once the budget was included).
* **Reproducibility gap:** SVG figures were not byte-for-byte reproducible because matplotlib salts element
  ids randomly; fixed with a constant salt and by stripping timestamps.
* **The demonstration:** a bundle (tornado charts, range-margin plot, CSV tables, report, run record) was
  generated for the placeholder model, and the gate correctly answered `RELEASE: BLOCKED`, naming the DRAFT
  reasons and the TBD dominating inputs (mass and wing area).

---

## 4a. Review follow-up (after the first commit)

A review of the committed code raised the points below. Each was checked against the code first;
the first two were real defects of mine and are the most important findings of this whole project
after the bus-current error. The follow-up is committed on top of `d9f9966`; the test suite (144
tests) passes with it.

### Confirmed and fixed

| Finding | What was wrong | Fix |
| --- | --- | --- |
| **Analysis D ignored scenario mass** | `run_braking` built `CasadiVehicle(vp)` and called `hover_omega`/`hover_throttle` with the nominal vehicle, never `scen.vehicle_for(vp)`. A run at mass ×1.3 gave an identical `a_eq` and stopping distance (difference exactly 0.0). The ensemble's 0.85–1.15 mass axis was therefore inert in the `Q_TRANS_DECEL` recommendation. | `run_braking` applies the scenario mass to the model, the initial rotor speed and the hover feed-forward; `recommend_decel` and `landing_error` rely on it. A regression test **fails on the old code and passes on the new**. Measured effect at 25 m/s: worst-case `a_eq` 2.70 / 2.59 / 2.50 / 2.43 m/s² at mass ×0.85 / 1.0 / 1.15 / 1.3, so the recommendation (the minimum) is set by the heavy end. The other analyses (C, B2, B1) already used `vehicle_for`. |
| **The T-12 gate could pass falsely** | `release_gate` matches tests by name, and the only `test_t12_*` runs on an analytic cost. B2, whose three seeds disagree by 4–9 %, would have satisfied the gate. | The run record carries a `search_spreads` entry for each analysis that used a stochastic search (`"C"`, `"B2"`); the gate blocks a record that lists one of them without a spread of at most 1 % (missing and NaN also block). Tested, including the exact case of B2 at 9 %. |
| **C and D were not aligned** | C set `Q_TRANS_DECEL` from the ideal planner while D measures it by simulation. | C now defaults to `a_plan_source="simulated"` (D's worst-case `a_eq`, same parameters, scenario and entry state); `"ideal"` remains selectable. A test checks the two C sources equal D's and the ideal planner. Real-problem T-12 for C re-measured: **spread 0.10 %, still passes** (cost 2.09 → 2.00). |
| **"Ideal is an upper bound on `a_eq`" was wrong** | the design doc said so; the simulation gave a worst-case `a_eq` 3 % *above* the ideal. | `proposal.md` §10D corrected locally (it is not tracked): the ideal figure is not a bound in either direction. The READMEs say the same. |
| **Register and physics could drift apart** | no link, and `configs/vehicle.yaml` points at `maps/ct_lift.csv` and `maps/ecm_fit_v1.npz`, which do not exist. | A drift test requires every register value to be paired with its physics default (they agree today); `Config.missing_files()` lists the missing files and the run record counts them as a reason for DRAFT. This is a guard, **not** the link itself. |
| **No JUnit output; no CI step for the gate** | `verification_from_junit` was never fed. | CI now runs `pytest --junitxml=junit.xml`, then `python -m sagetrans.io.ci_check junit.xml` (fails if any of T-01…T-15 is missing or failing), and uploads the JUnit file. Checked against a real run: all 15 IDs present and passing. CI still does not call `release_gate` itself (that needs a run record built from real analysis results). |
| **Unused `ground_speed_below(headwind=…)`** | the parameter did nothing. | Removed (no caller used it). |
| **Report errors** | it said there were no commits (`d9f9966` exists) and cited `proposal.md`, which is untracked. | Corrected at the top of this document and in the package README: `proposal.md` is local-only because `.gitignore` lists it. |

### B2 from the B1 solution — implemented, but did not fix repeatability

`freeform_opt.init_from_collocation` re-plots a B1 solution against `V/V_s` as B2's starting knots, and
`optimise` accepts it (and a step size, `sigma0`). Measured on the real problem, same ensemble:

| B2 start | Start cost | Best cost | Seed spread |
| --- | --- | --- | --- |
| analysis A, `sigma0` 0.25 | 8.68 | 1.94 | 4.4 % |
| B1, `sigma0` 0.25 | 1.90 | 1.75 | 8.7 % |
| B1, `sigma0` 0.10 | 1.90 | 1.69 | 5.0 % |
| B1, `sigma0` 0.04 | 1.90 | 1.75 | 5.3 % |

The B1 start is a far better starting point and reaches lower costs, but **T-12 is still not met for
B2**; the gate will block any record that includes it. The review's suggestion was right as far as
it goes, and it is not sufficient at this budget.

### Acknowledged, and what the code does about each

* **Stall blend cliff (`M = 100 /rad`).** Not changed — it is the model. B1 works around it with a
  stall margin enforced at nodes and midpoints (this is why replay mismatches stopped). It stays a
  physically questionable feature to revisit with wing data.
* **`gamma = atan2(vh, va)` at hover.** Genuinely unaddressed: its derivative is undefined at `V = 0`.
  Simulation never differentiates it and B1 stays at about 1 m/s or more, **except** that a tailwind
  equal to the end ground speed would bring airspeed near zero; B1 has not been run there.
* **`if_else` attitude rate limit.** Non-smooth in every mode. Inactive in the B1 solutions (largest
  pitch rate about 1.0 rad/s against a 1.5 rad/s limit); trouble is expected if a problem drives it.
* **Motor torque step at zero current.** Already smoothed in collocation (`tanh`); exact `sign` for
  simulation. (So this one is addressed where gradients matter.)
* **Controller behaviours** (Position1 only brakes and drifts backward after an overshoot; the
  low-speed sink at 6–10 m/s from slow assumed gains; the air-speed switch test firing early in a
  headwind), **events already past zero at the first sample never fire**, and **serial evaluation**:
  all accurate and all documented in the READMEs; none changed.

### Not done from the review

* **The register → physics link.** Only the drift guard and the missing-file check; `VehicleParams` is
  still built from placeholder dataclasses.
* **A B2 that is repeatable.** Needs a different approach or budget (see the table above).
* **Real-problem T-12 as a unit test.** Three-seed optimisations take minutes; the requirement is
  enforced through the gate and the run record instead.

---

## 5. Cross-cutting design decisions and corrections

### 5.1 One vehicle model, written once, in CasADi
Used both by `simulate()` (numeric) and by collocation (symbolic). It removes a whole class of
simulation-versus-optimisation drift, and the analysis in M6 is why that mattered: replay disagreements
were diagnosed *as* modelling differences between the NLP and the simulator, and could be isolated because
they share one definition.

### 5.2 Smoothing as a parameter
Exact clips for simulation, rounded clips for the NLP, with the difference measured (replay agreement)
rather than assumed.

### 5.3 Honesty mechanisms built into the code
Every parameter carries a grade; every result carries a `DRAFT` run record; `kappa` cannot be defaulted;
non-convergence is a recorded status, not an exception or a silent fallback; collocation results are
replay-verified; the release gate refuses TBD dominating inputs even if flagged.

### 5.4 Deviation from the proposal's equations: bus current (needs your confirmation)
The proposal's eq. (6.11) took `I_bus = Σ I_m`. The corrected model uses `I_bus = n δ_r I_m,r + δ_p I_m,p
+ I_av`. This is energy-conserving and lowers every battery-related number by roughly the throttle ratio
(about 3× at hover). It is the one place I changed the proposal's physics rather than only its scope.

---

## 6. Deviations from the proposal (complete list)

| Area | Proposal | What was done | Why |
| --- | --- | --- | --- |
| Eq. (6.11) | bus current = motor current | bus current = `δ × I_m` | printed form violates energy balance |
| Parameter register | physics reads the register | physics reads placeholder dataclasses; the register supplies grade/hash only | register has no data yet; linking is the next step |
| M5 scope | full baseline, B2 and C over forward + back | back-transition only for B2 and C comparison; `full` scope exists for C | instruction: bare minimum |
| `Q_TRANS_DECEL` in C | set from D's simulated `a_eq` | originally the ideal-pitch planner; **now the simulated worst-case `a_eq` by default** (`a_plan_source="ideal"` still selectable) | aligned with D after review (section 4a); costs about twice the simulation per evaluation |
| C decision vector | 9 parameters | 3 (back) / 6 (full) | `PTCH_LIM_MAX_DEG`, `Q_TRANS_FAIL`, `BATT_WATT_MAX` fixed or not modelled |
| Search | parallel evaluation, cached by config hash | serial, cached by parameter vector | CasADi objects cannot be pickled |
| B2 start | fit to B1 | analysis A's pre-spun schedule by default; `init_from_collocation` fits it to B1 | B1 did not exist when B2 was built; the B1 start was added later and did not make B2 repeatable |
| Analysis D | includes cruise and switch | starts at the switch point | advice taken in M3; cruise is in C `full` |
| Stall failure | `alpha_w > alpha_s` for a set time | only while `V ≥` the 1g stall speed | below it the rotors carry the weight by design |
| B1 constraints | listed in (10.4) | plus `|v_h| ≤ 3 m/s`, stall margin 0.85, regulariser, effective-throttle control | convergence and replay fidelity |
| B1 guesses | five per point | three | time |
| B1 mesh | 100–200 intervals | 40–60 for sweeps (refined from 20); 100–120 shown to work | solve time |
| T-11 | 2 % on every metric | 2 % on energy/distance/duration; excursion 2 % *or* 0.3 m | excursion is a small difference of large integrals |
| B1 as a bound | "the bound" | best under its operational limits; not a strict lower bound | relaxation would not converge |
| B1 acceptance | "reproduces A when unconstrained" | not tested | wording ambiguous (A is the zero-loss reference); tested the bound property and T-11 instead |
| Reference integrator | LSODA or Radau | LSODA only | sufficient for T-09 |
| `Q_BCK_PIT_LIM` | blends with airspeed | constant cap | blending is undocumented |
| Sobol | global method | implemented and verified on analytic functions; Morris used on the real model | cost |
| Layout | `scenarios.py`, `cli.py`, `maps/`, `benchmarks/`, `regression/` | not created | out of the bare-minimum scope |
| Proposal text | revise each milestone's section | only §6.11 edited | not requested; to do |

---

## 7. Results (all placeholder-based; none is a statement about Sage)

| Result | Value |
| --- | --- |
| Hover electrical power (8 kg placeholder) | about 1.2 kW |
| Back-transition altitude change, baseline controller, 22–25 m/s | **gain** of 12–18 m; no loss |
| Planner worked example (25 m/s, 20°) | 87.5 m / 3.57 m/s² (no ramp); 124.5 m / 2.51 m/s² (3 s ramp) — matches the proposal |
| Simulated worst-case `a_eq` vs ideal | about 3 % higher (wing drag helps) |
| `Q_TRANS_DECEL` recommendation (demo, 18 and 22 m/s) | about 2.1 m/s² |
| Zero-loss feasibility | limited by the wing's lift at the braking pitch (thrust sign), depends on `V/V_s`; throttle demand rises in thin air |
| B1 energy against altitude band (sea level, 22 m/s) | 2.11 kJ (16 m) → 2.34 kJ (10 m) → 2.56 kJ (5 m) |
| Baseline vs B1 at equal excursion and distance | about 4.5 kJ vs 2.2 kJ |
| Tuned C / tuned B2 / default baseline | 13.4 m, 3.2 kJ / 8.1 m, 2.4 kJ / 12.4 m, 4.5 kJ |
| Maximum range of the placeholder budget | several hundred km (a property of the placeholder wing and pusher) |
| Dominating inputs (re-run after the review fixes; ranking Spearman 0.96–0.99) | excursion: wing area, lift-curve slope, `CD0`, altitude-hold gain; distance: headwind, wing area, mass (stall angle close); energy: mass, headwind, rotor `CT0`; range: wing area, mass, `CD0` |

---

## 8. Verification status

| ID | Status |
| --- | --- |
| T-01 … T-11, T-13, T-14, T-15 | pass |
| T-12 | passes on an analytic cost and on analysis C (0.10 % spread, re-measured with the simulated `Q_TRANS_DECEL`); **fails for analysis B2** (4.4–9 % from the A start; 5.0–8.7 % from the B1 start). The unit test covers only the analytic cost; the real-problem figure is enforced by the gate (`search_spreads`) |
| FR-01…FR-09 | implemented; FR-06 (derivative-free repeatability) met by C, not by B2 |
| Release gate | implemented and tested, including the real-problem T-12 rule; blocks every current result (DRAFT) |
| NFR-01 (determinism) | met (seeded searches, byte-identical outputs) |
| NFR-02 (SI units) | met at the input boundary |
| NFR-03 (speed) | 60 s segment under 1 s; 100-scenario ensemble not timed |
| NFR-05 (layering) | enforced by `import-linter` |
| NFR-08 (one vehicle definition) | met |

CI runs `ruff`, `mypy --strict`, `lint-imports` and `pytest --junitxml`, then `ci_check` (fails if any of
T-01…T-15 is missing or failing) and uploads the JUnit file. It does **not** call `release_gate`: that
needs a run record built from real analysis results.

---

## 9. What is placeholder, missing, or at risk

**Placeholder:** all vehicle parameters (mass, wing, rotors, motors, pusher, battery, attitude);
the rotor coefficient maps (the proposal's largest uncertainty); controller gains; the usable-energy window
(O-15); the ensemble ranges (O-14); the forward transition in the energy budget (a quasi-steady estimate
with an assumed acceleration); `kappa` for the vortex-ring limit (unsourced).

**Not done:**

* Linking `VehicleParams` to the `Config` register, so a result's grade comes from real inputs. (A drift
  guard and a missing-file check exist; the link does not.)
* M8: calibration against iron-bird data and SITL (V-01…V-07).
* Closing the ArduPilot questions O-01…O-11 (no source reading was done; the baseline's behaviour is the
  documented structure with assumed shapes and gains).
* A `cli.py`; the named scenario set S-01…S-06 as YAML; three cost weightings in reports; figures
  beyond the four built. (The JUnit step in CI now exists.)
* Revising the proposal's milestone sections to match the code.
* A sourced vortex-ring-state exclusion in B1, and a free rotor start that counts its pre-entry energy.

**Technical risks:** B1's convergence is fragile and its results must be replay-verified; B2 is not
repeatable (a B1 start helps but does not fix it); the stall blend's sharpness makes the lift
cliff-like, which is physically questionable and must be revisited with wind-tunnel or flight data;
`gamma = atan2(vh, va)` has no defined derivative at hover and the attitude rate limit is non-smooth
(both documented in `physics/README.md`); the rotor torque is even in speed; sensitivity rankings
depend on the ranges chosen for the inputs; and the Q_TRANS_DECEL recommendation is only as good as
the mass range of the ensemble now that mass is applied.

---

## 10. How to run and reproduce

```bash
pip install -e ".[dev]"
pytest                               # 144 tests, about 3 minutes
ruff check . && mypy && lint-imports
```

Reproducing the headline numbers uses only public APIs (scripts were run from a scratch directory):

```python
from sagetrans.physics.params import placeholder_vehicle
from sagetrans.analysis.common import Scenario
vp, sc = placeholder_vehicle(), Scenario(100.0)

# T-12 on analysis C (about 5 min):  restricted.optimise(vp, latin_hypercube(4, vp, seed=1), "back",
#                                                          popsize=8, maxiter=15)  -> .search.spread
# T-12 on analysis B2 (about 5 min): freeform_opt.optimise(vp, scens, popsize=12, maxiter=15,
#                                      init_params=freeform_opt.init_from_collocation(prob, sol),
#                                      sigma0=0.1)   -> .search.spread   (A start: omit init_params)
# B1 Pareto points:                  collocation.make_problem(vp, sc, 22.0, CollocationOptions(n=40,
#                                      max_iter=800)); solve_refined(prob, eps_h, 300.0, ns=(20,))
# Sensitivity (about 70 s):          S.run_sensitivity(S.HeadlineModel(vp, sc, S.default_inputs(vp, sc)),
#                                                       r_small=6, r_large=12)
```

---

## 11. Recommended next steps, in order

1. **Connect the register to the physics.** Build `VehicleParams` from a `Config`, so a result's grade,
   hash and DRAFT status reflect real inputs. This is what lets the gate ever pass.
2. **Fill the TBDs the sensitivity analysis flags:** mass (and the meaning of "5 kg", O-12), wing area,
   the wing's lift-curve slope and drag, then the rotor thrust and torque coefficients.
3. **Confirm the corrected bus-current model** (section 5.4) — or tell me to revert it.
4. **Start M8 data collection:** rotor thrust against throttle and pack voltage, spool-up steps, ESC dead
   zone, battery pulses; they replace the highest-ranked placeholders.
5. **Make B2 repeatable.** The B1 start was tried and is not enough (section 4a). Options: a much
   larger budget, fewer free knots, a cheaper evaluation so more seeds fit, or a different optimiser.
   Until the spread is at most 1 %, the gate blocks any record that includes B2.
6. **Close the ArduPilot source questions** (firmware version, switch-test speed, ramp shapes), starting
   with O-08 and O-01.
7. Add the remaining layout items (`scenarios.py`, named scenario YAML, `cli.py`) and update the
   proposal's milestone sections. (The JUnit step in CI is done.)

---

## 12. Appendix: file inventory

| File | Lines | Purpose |
| --- | --- | --- |
| `analysis/collocation.py` | 585 | B1 |
| `analysis/energy.py` | 378 | E |
| `analysis/zero_loss.py` | 342 | A |
| `analysis/sensitivity.py` | 330 | sensitivity |
| `dynamics/trim.py` | 262 | inversion, steady flight, electrical state |
| `dynamics/simulate.py` | 210 | time stepping |
| `control/ardupilot_like.py` | 192 | baseline controllers |
| `config/schema.py` | 190 | parameter register |
| `analysis/restricted.py` | 157 | C |
| `physics/params.py` | 139 | parameters and placeholders |
| `analysis/braking.py` | 138 | D |
| `physics/vehicle.py` | 129 | the vehicle |
| `config/units.py` | 125 | units |
| `io/report.py` | 124 | report |
| `analysis/freeform_opt.py` | 119 | B2 |
| `io/record.py` | 111 | run record |
| `io/figures.py` | 102 | figures |
| `io/gate.py` | 101 | release gate |
| `analysis/descent.py` | 97 | F |
| `analysis/search.py` | 92 | CMA-ES |
| `analysis/runs.py` | 90 | run helpers |
| `physics/battery.py` | 76 | battery |
| `analysis/ensemble.py` | 76 | ensembles and cost |
| `physics/rotor.py` | 70 | rotors |
| `io/results.py` | 70 | results writer |
| `analysis/common.py` | 69 | scenarios, run record |
| `physics/motor.py` | 62 | motor |
| `atmosphere.py` | 53 | ISA |
| `physics/aero.py` | 47 | wing |
| `dynamics/events.py` | 47 | events |
| `control/freeform.py` | 40 | free-form law |
| `control/base.py` | 25 | controller interface |
| `physics/attitude.py` | 23 | pitch surrogate |
