# `configs/` — where your data goes

Everything the toolkit needs is in five YAML files here. Edit them, then run **one command** from
the repository root:

```bash
python run.py --check     # validate everything, show what would run and how long it takes
python run.py             # run the whole study; results appear in results/<run_name>/
```

Nothing else needs touching: no code, no Python edits. The files shipped here contain
**placeholder** values for a notional 8 kg aircraft so the first run works out of the box; replace
them with real data as you get it.

## The five files

| File | Kind | What you put in it |
| --- | --- | --- |
| `vehicle.yaml` | register | the 59 physical parameters: mass, wing, rotors, pusher, both motors, battery, pitch dynamics |
| `mission.yaml` | register | the mission profile, usable-battery window, reserves, descent limits, range requirements |
| `controller.yaml` | register | the ArduPilot parameters (by their real names) and the assumed controller gains |
| `scenarios.yaml` | plain | the named scenarios (S-01…S-06) and the ensemble ranges |
| `study.yaml` | plain | which analyses to run, how much effort, run name, draft/release mode, parallelism |

**Register files** are the *data*: every entry carries a value, a unit, a source and an evidence
status, and the toolkit tracks how much of every result rests on assumptions. **Plain files** are
*choices* (which scenarios, how many seeds); they have no evidence grades but are hashed into every
run record, because they change the numbers.

## An entry in a register file

```yaml
wing:
  S: {value: 0.6, unit: "m^2", source: "CAD, rev C", status: measured}
  alpha_s: {value: 20, unit: "deg", source: "wind tunnel run 14", status: measured}
```

* **`unit`** can be any unit the converter knows (`m mm ft km`, `kg g`, `s ms min`, `deg rad rpm`,
  `Ah mAh`, `V A ohm F`, `N Pa hPa`, `J Wh kWh W kW`, `K`, `degC`, `%`, `-` for dimensionless,
  compounds such as `m/s`, `m^2`, `kg*m^2`, `V*s/rad`). It is converted to SI **once**, on load, so
  type the number in whatever unit you have. An unknown unit is an error that names the entry.
* **`status`** is the evidence grade: `measured` (your own bench or flight data), `datasheet`,
  `derived`, `documented` (for example ArduPilot's docs), `stated` (given in the brief), `assumed`,
  or `TBD`.
* **`source`** is free text: where the number came from. It is shown in the report.
* Optional **`uncertainty`** (a fraction, e.g. `0.3`) sets the half-range used for that input in the
  sensitivity analysis.
* A few entries are **optional**: leave them out and the toolkit chooses automatically
  (`mission.profile.v_cruise` = best-range speed, `mission.descent.v_sensor` = no limit,
  `Q_BCK_PIT_LIM` = no cap). The files have a commented example.

### Draft mode and release mode

| | `draft` (default) | `release` |
| --- | --- | --- |
| an entry marked `TBD` | used as written | **refused** |
| an entry missing from the file | the built-in placeholder is used and recorded | **refused** |
| a referenced file that does not exist | recorded | **refused** |
| results | labelled **DRAFT** | labelled RELEASED only if the above are all clear |

Set it in `study.yaml` (`mode: draft`) or on the command line (`python run.py --mode release`). The
shipped placeholders are all marked `TBD` on purpose: change an entry to `measured`, `datasheet`,
… when it is real, and the run moves toward release grade one entry at a time. `python run.py --check`
lists exactly what is still TBD.

A typo in an entry name is always an error, never a silent default: the loader reports the unknown
name and suggests the closest valid one.

## What to provide first

The sensitivity analysis ranks which inputs move the headline results most. On the placeholder model
the order was: **mass** and **wing area** (both still TBD), then the wing's **lift-curve slope**
(`CL_alpha`) and **zero-lift drag** (`CD0`), the **rotor thrust and torque coefficients**
(`CT0`, `CQ0`), the **altitude-hold gains**, and the **wind**. Pin these down first; the rest matter
less. Re-run the sensitivity on your own data to get your own ranking (it is part of `python run.py`).

## `vehicle.yaml` reference

Rotor and pusher aerodynamics are **polynomial** coefficient maps (`CT = CT0 + CT1 J + CT2 J²`,
and so on). Entries marked *numerical* are not physical: leave them unless you know why.

#### `mass` — total take-off mass (what does "5 kg" mean? open question O-12)

| Entry | Unit | Placeholder | Meaning / where it comes from |
| --- | --- | --- | --- |
| `m` | kg | 8 | CAD and weighing; define the mass case (nominal / max / min) |

#### `wing` — area, linear lift, stall and drag; flat-plate post-stall model

| Entry | Unit | Placeholder | Meaning / where it comes from |
| --- | --- | --- | --- |
| `S` | m^2 | 0.6 | CAD or flight-mechanics sheet |
| `CL0` | - | 0.25 | lift coefficient at zero wing angle of attack |
| `CL_alpha` | rad^-1 | 5 | lift-curve slope; flight-mechanics sheet or wind tunnel |
| `alpha_s` | rad | 0.35 | stall angle of attack |
| `CD0` | - | 0.03 | zero-lift drag coefficient |
| `k` | - | 0.05 | induced-drag factor, CD = CD0 + k CL² |
| `CN` | - | 1.5 | flat-plate normal-force coefficient (1.2 to 2 in the literature); a sourced value is needed |
| `i_w` | rad | 0.03 | wing incidence relative to the body x-axis |
| `M` | rad^-1 | 100 | stall-blend sharpness: *numerical*; 100 is needed for the wing test T-02 |

#### `rotor` — four lift rotors (identical in the point-mass model)

| Entry | Unit | Placeholder | Meaning / where it comes from |
| --- | --- | --- | --- |
| `n_r` | - | 4 | 4+1 configuration (stated) |
| `D` | m | 0.457 | propeller diameter; propulsion trade study (O-13) |
| `I_rot` | kg*m^2 | 0.0015 | rotor + motor inertia; sets spool-up |
| `CT0` | - | 0.1 | static thrust coefficient, T = CT ρ n² D⁴; thrust bench |
| `CT1` | - | -0.1 | slope in the axial advance ratio J; BEMT or published data |
| `CT2` | - | 0 | quadratic term in J |
| `CQ0` | - | 0.00669 | static torque coefficient, Q = CQ ρ n² D⁵; thrust bench |
| `CQ1` | - | -0.00335 | torque-coefficient slope in J |
| `CQ2` | - | 0 | quadratic term in J |
| `CH1` | - | 0.01 | in-plane force coefficient per unit edgewise ratio μ |
| `k_mu` | - | 1 | edgewise-flow factor (1 + k_mu μ²) on CT and CQ |
| `J_min`, `J_max`, `mu_max` | - | -0.5, 1, 1 | validity range of the maps (inputs are clipped to it) |
| `omega_floor` | rad/s | 1 | *numerical* guard keeping J and μ finite at standstill |

The largest single uncertainty of the project is rotor behaviour at high edgewise flow and negative
axial inflow (`CT1`, `CT2`, `CQ1`, `CQ2`, `k_mu`, `CH1`): the static bench tests do not measure it.

#### `pusher` — propeller, polynomial maps in J_p = V cos(α) / (n D)

| Entry | Unit | Placeholder | Meaning / where it comes from |
| --- | --- | --- | --- |
| `D` | m | 0.4 | propeller diameter |
| `I_p` | kg*m^2 | 0.0002 | propeller + motor inertia |
| `CT0` | - | 0.05 | thrust coefficient at J_p = 0 |
| `CT1` | - | -0.03 | slope in J_p (thrust goes negative at high J_p: windmilling drag) |
| `CT2` | - | 0 | quadratic term |
| `CQ0` | - | 0.0055 | torque coefficient at J_p = 0 |
| `CQ1` | - | -0.0018 | slope in J_p |
| `J_min`, `J_max` | - | -0.5, 2 | validity range |
| `omega_floor` | rad/s | 1 | *numerical* guard |

#### `motor_lift` and `motor_pusher` — one motor + ESC each (the four lift motors are assumed identical)

| Entry | Unit | Placeholder (lift / pusher) | Meaning / where it comes from |
| --- | --- | --- | --- |
| `Ke` | V*s/rad | 0.03 / 0.04 | back-EMF constant (from Kv, or bench) |
| `Kt` | N*m/A | 0.03 / 0.04 | torque constant (equal to `Ke` in SI) |
| `Rm` | ohm | 0.08 / 0.05 | winding resistance |
| `I0` | A | 1.5 / 1.0 | no-load current |
| `I_lim` | A | 60 / 80 | motor/ESC current limit |
| `dead_zone` | - | 0.05 / 0.05 | ESC dead-zone fraction of throttle; bench motor test |

#### `battery` — two packs in parallel through a power distribution board

| Entry | Unit | Placeholder | Meaning / where it comes from |
| --- | --- | --- | --- |
| `n_parallel` | - | 2 | packs in parallel (stated) |
| `cells_series` | - | 12 | 12S (stated) |
| `R0_pack`, `R1_pack` | ohm | 0.03, 0.02 | series and RC-branch resistance of **one** pack; HPPC/GITT fit |
| `C1_pack` | F | 2000 | RC-branch capacitance of **one** pack |
| `I_av` | A | 2 | avionics current drawn from the bus |
| `V_min` | V | 33 | bus cut-off; battery-management limit (O-15) |
| `capacity_Ah_pack` | Ah | 25 | datasheet, per pack |
| `v_nominal` | V | 43.2 | datasheet |

The open-circuit voltage is a placeholder linear curve in the code (`12 × (3.3 + 0.9 × SoC)` V);
a measured table is not supported yet (see "Not supported yet").

#### `attitude` — pitch-tracking surrogate (full rotational dynamics are out of scope)

| Entry | Unit | Placeholder | Meaning / where it comes from |
| --- | --- | --- | --- |
| `omega_n` | rad/s | 8 | pitch-loop natural frequency; SITL or flight logs |
| `zeta` | - | 0.8 | pitch-loop damping ratio |
| `q_max` | rad/s | 1.5 | pitch-rate limit |

## `mission.yaml` reference

| Group.entry | Unit | Placeholder | Meaning |
| --- | --- | --- | --- |
| `profile.cruise_amsl_m`, `ground_amsl_m` | m | 1000, 0 | cruise altitude and take-off/landing elevation |
| `profile.isa_offset_k` | K | 0 | temperature offset for the budget |
| `profile.clearance_agl_m`, `vz_takeoff`, `t_hover_pre` | m, m/s, s | 30, 2, 10 | take-off: clearance height, climb rate, hover before lift-off |
| `profile.a_fwd` | m/s² | 1.5 | forward-transition acceleration (an estimate until analysis C replaces it) |
| `profile.v_climb`, `roc` | m/s | 18, 2.5 | climb true airspeed and rate of climb |
| `profile.v_cruise` | m/s | *omitted* | cruise speed; omitted = best-range speed |
| `profile.v_glide` | m/s | 16 | pusher-off glide speed in descent |
| `profile.h_bt_agl_m` | m | 30 | height above ground where the back-transition starts |
| `profile.soc_mean` | - | 0.6 | one mean state of charge for all segment powers |
| `profile.t_go_around`, `t_hold`, `d_diversion_m`, `n_diversion_transitions` | s, s, m, - | 60, 0, 5000, 0 | reserve (go-around, hold, diversion) |
| `battery_window.eta_usable`, `soh`, `temp_factor` | - | 0.8, 1, 1 | usable energy = η × SoH × f(T) × nominal (O-15) |
| `battery_window.soc_eod` | - | 0.1 | state of charge at end of discharge, for the hover-power check |
| `descent.kappa` | - | 0.25 | vortex-ring fraction of induced velocity. **Unsourced: must come from a sourced boundary (Johnson, NASA).** Analysis F refuses to run without it |
| `descent.v_fragility`, `v_sensor`, `h_slow`, `v_final` | m/s, m/s, m, m/s | 3, *omitted*, 5, 0.5 | payload-fragility and sensor limits; the final slow descent |
| `requirements.range_primary_km`, `range_p95_km` | km | 100, 113 | range requirements the budget is checked against |
| `speeds.entry_eas_ms` | m/s | 22 | equivalent airspeed at the start of the back-transition |

## `controller.yaml` reference

The `ardupilot` group uses the **real parameter names**.

| Entry | Unit | Placeholder | Meaning |
| --- | --- | --- | --- |
| `AIRSPEED_MIN` | m/s | 14 | transition speed. The simulated controller compares *true* airspeed; whether ArduPilot compares EAS or TAS is open (O-02) |
| `Q_TRANSITION_MS` | ms | 5000 | rotor fade time after `AIRSPEED_MIN` (documented default) |
| `Q_BACKTRANS_MS` | ms | 3000 | back-transition pitch-limit ramp |
| `Q_A_ANGLE_MAX` | deg | 20 | maximum VTOL lean angle |
| `PTCH_LIM_MAX_DEG` | deg | 35 | fixed-wing pitch limit (fixed in analysis C, not searched) |
| `Q_BCK_PIT_LIM` | deg | *omitted* | back-transition pitch cap; omitted = none |
| `TKOFF_THR_MAX` | - | 0.7 | pusher throttle in AUTO during the forward transition |
| `Q_TRANS_FAIL` | ms | 0 | stuck-transition timeout; 0 disables it |
| `Q_M_SPIN_MIN`, `Q_M_SPIN_MAX` | - | 0.1, 0.95 | rotor throttle floor and ceiling |
| `Q_TRANS_DECEL` | m/s² | *omitted* | normally an **output** (analysis D); add it only to fix the planner value |
| `altitude_hold.K_h`, `K_vh`, `K_i`, `v_dem_max`, `i_max` | | 0.5, 0.02, 0.01, 3, 20 | rotor altitude-hold gains (assumed) |
| `position.K_p`, `K_v`, `a_lim` | | 0.5, 1, 3 | position-cascade gains and deceleration limit (assumed; `a_lim` is open question O-11) |
| `cruise_stub.v_cruise`, `r_tecs`, `tau_h`, `K_s`, `K_si`, `K_hp`, `K_hd`, `theta_cruise_max` | | 22, 0.5, 1, 0.05, 0.01, 0.02, 0.03, 0.15 rad | fixed-wing hold between the transitions (assumed) |

Analysis C **starts** its search from the `ardupilot` values you give here, and searches
`Q_BACKTRANS_MS`, `Q_A_ANGLE_MAX` and `Q_M_SPIN_MIN` (plus `AIRSPEED_MIN`, `Q_TRANSITION_MS` and
`TKOFF_THR_MAX` in its `full` scope). The other entries stay fixed at your values.

## `scenarios.yaml`

`core` lists the named scenarios that appear in every report (S-01…S-06 from the design). Each has
`altitude_amsl_m`, `isa_offset_k`, `soc`, `headwind_ms` (positive = against the aircraft),
`mass_factor` (times the vehicle mass) and optional `ground_amsl_m`. `ensemble.axes` gives the
ranges of the Latin-hypercube ensemble used by the optimisations and the `Q_TRANS_DECEL`
recommendation. Add scenarios by adding lines; refer to them by name in `study.yaml`.

## `study.yaml`

| Entry | Meaning |
| --- | --- |
| `run_name`, `output_dir` | results go to `<output_dir>/<run_name>/` |
| `mode` | `draft` (default) or `release`; `--mode` on the command line overrides it |
| `parallel` | `enabled`, and `max_workers` (0 = one per CPU core) |
| `analyses` | which of `A D F E Q2 C B2 B1 sensitivity` to run; `--only` overrides it |
| `switch_test` | `ground` or `air`: the speed used in the switch-point test (O-02) |
| `A`, `D`, `F`, `E`, `Q2` | grids and scenario lists for the quick analyses. `A` also takes `alpha_min_deg` and `cl_frac`: the wing angle-of-attack band the feasibility scan must stay inside (lower end of the lift fit; the angle where C_L = `cl_frac` x C_L,max, provisional because C_L,max is) |
| `C`, `B2` | ensemble size and seed, search seeds, population, iterations (`C` also `a_plan_source`) |
| `B1` | scenarios, the altitude bands to sweep, the distance cap, the collocation mesh |
| `sensitivity` | the scenario, the two Morris sizes, and whether to include the energy budget |

Effort knobs worth knowing: `C.maxiter`, `B2.maxiter`, `B1.intervals` and `sensitivity.r_large`
dominate the run time. Lower them for a quick look, raise them for a final run.

## What `python run.py` produces

`results/<run_name>/`:

| File | Contents |
| --- | --- |
| `report.md` | headline numbers with the evidence grade next to each, per-analysis results, sensitivity, limitations |
| `*.csv` | every table (feasibility maps, budgets, trade curves, Pareto points, sensitivity indices …) |
| `*.png`, `*.svg` | figures, each with the run record in the footer |
| `inputs_used.md` | every register entry as used: value, unit, status, source; what fell back to a placeholder |
| `release_gate.txt` | the release-gate verdict and exactly what blocks it |
| `run_record.json` | config hash, settings hash, code version, git commit, evidence counts, seeds, and a SHA-256 of every output file |

Results are cached per task, keyed by the hash of every result-relevant input. Run again with
unchanged inputs and only the report is rebuilt; change one value and only the affected tasks
re-run. `python run.py --force` ignores the cache.

## Command line

```text
python run.py                        # run everything in configs/
python run.py --check                # validate and show the plan, then stop
python run.py --config other/        # another config folder
python run.py --mode release         # refuse unless nothing is TBD or missing
python run.py --only A D E           # just these analyses
python run.py --force                # ignore cached results
python run.py --serial               # one process (easier to debug)
python run.py --verify               # also run the test suite and feed the release gate
```

Exit codes: `0` ok, `1` a task failed, `2` the inputs are unusable (or release mode refused),
`3` release mode finished but the release gate blocked the result.

## Not supported yet

* **Measured coefficient tables.** Rotor and pusher aerodynamics are polynomial coefficients only.
  Table-driven maps (CSV over J and μ) and a measured open-circuit-voltage / equivalent-circuit table
  need B-spline interpolants in the model and are the next step; the register already accepts `file:`
  entries, but nothing reads them.
* **A forward-transition optimisation** and a simulated forward transition in the energy budget.
* **Per-scenario vehicle changes** other than mass (a hot-day derating of the motors, say).
