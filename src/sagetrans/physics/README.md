# `sagetrans.physics` — the vehicle model

The physical sub-models and the one compiled function that assembles them. Layer position: above
`atmosphere` and `config`, below `dynamics`. **Nothing here knows about controllers, time
stepping, or analyses.**

| File | Models | Proposal |
| --- | --- | --- |
| `params.py` | frozen dataclasses of every physical parameter, and the placeholder vehicle | §6, §9 |
| `aero.py` | wing lift and drag with a smooth stall blend | §6.2, eqs. 6.2–6.5 |
| `rotor.py` | lift-rotor and pusher thrust, torque, in-plane force; vortex-ring diagnostic | §6.3, eqs. 6.6–6.7 |
| `motor.py` | motor and ESC electrical/mechanical model | §6.4, eqs. 6.8–6.9 |
| `battery.py` | equivalent-circuit pack, bus-voltage solution | §6.5, eqs. 6.10–6.11 |
| `attitude.py` | second-order pitch-tracking surrogate | §6.6, eq. 6.12 |
| `vehicle.py` | `CasadiVehicle`: all of the above plus the equations of motion | §7, eqs. 7.1–7.4 |

## The central design decision: one model, written once, in CasADi (NFR-08, O-17)

Collocation (analysis B1) needs the vehicle as a *symbolic* expression; simulation needs fast
*numeric* code. Writing both invites drift, so the vehicle is written **once**, as CasADi
expressions, and compiled into a single `casadi.Function`. `simulate()` calls it with numbers;
the collocation NLP calls it with symbols.

* Branches are written with `fmax`, `fmin`, `if_else`, `sign` — never Python `if` on a value.
  (Python `if` on a *configuration* constant, such as "smoothing on or off", is fine and used.)
* A NumPy reference implementation of the wing, rotor, motor and battery functions exists **only
  in the tests** (`tests/unit/physics_reference.py`) to check this model (T-15).
* **Speed check (M1 gate, NFR-03):** one 60 s segment (3000 compiled RK4 steps) takes about **0.63 s
  on the development laptop**, against the requirement of under 1 s "on a laptop". A full 12 s
  braking simulation including the controller takes about 0.15 s. The fallback of a NumPy model with
  an equivalence test was therefore not needed. The test (`test_physics.py`) takes the best of three
  repeats and applies the strict 1.0 s limit everywhere except when the `CI` environment variable is
  set, where the limit is multiplied by 3: a shared GitHub runner measured **1.37 s** for the same
  work (about 2.2× slower, a hardware difference, not a model one), which an absolute wall-clock limit
  cannot distinguish from a regression. The requirement is unchanged; only the CI allowance is
  stated, and the test still fails when the time is exceeded.
* **Smoothing is a parameter.** `CasadiVehicle(params, smooth=0.0)`. At `0` the clips are exact
  (used for simulation). At `smooth > 0` the clips are rounded with hyperbolic functions so a
  gradient-based solver can use them (used by collocation). The current-clip width is `smooth`
  amps; the dimensionless clips (rotor inflow ratios, ESC dead zone) use the fixed
  `SMOOTH_ND = 0.02`. The size of the difference is measured and reported (see
  `analysis/README.md`, B1).

## `params.py` — parameter containers and the placeholder vehicle

All dataclasses are `frozen=True` (hashable, so they can key caches). `placeholder_vehicle()`
returns the default `VehicleParams`.

> These are **illustrative assumed numbers**, chosen so that the model behaves like a plausible
> 8 kg 4+1 VTOL. They are *not* Sage's data. The pusher in particular was retuned three times
> during development because earlier values gave no thrust at cruise speed or far too much.

| Group | Parameter | Placeholder value |
| --- | --- | --- |
| Mass | `m` | 8.0 kg (the meaning of "5 kg" is open, O-12) |
| Wing | `S`, `CL0`, `CL_alpha`, `alpha_s`, `CD0`, `k`, `CN`, `i_w`, `M` | 0.6 m², 0.25, 5.0 /rad, 0.35 rad, 0.03, 0.05, 1.5, 0.03 rad, 100 /rad |
| Lift rotors (4) | `D`, `I_rot` | 0.457 m, 1.5e-3 kg m² |
| | `CT0`, `CT1`, `CT2` | 0.10, −0.10, 0 |
| | `CQ0`, `CQ1`, `CQ2` | static figure of merit 0.6, so `ideal_cq0(0.10)/0.6`; `CQ1 = −0.5 CQ0`; 0 |
| | `CH1`, `k_mu` | 0.01, 1.0 |
| | clips | `J` in [−0.5, 1], `mu` in [0, 1] |
| Pusher | `D`, `I_p` | 0.40 m, 2e-4 kg m² |
| | `CT0`, `CT1` | 0.05, −0.03 (thrust goes negative above `J = 1.67`) |
| | `CQ0`, `CQ1` | 0.0055, −0.0018 |
| Lift motors | `Ke = Kt`, `Rm`, `I0`, `I_lim`, `dead_zone` | 0.03 V s/rad, 0.08 Ω, 1.5 A, 60 A, 0.05 |
| Pusher motor | `Ke = Kt`, `Rm`, `I0`, `I_lim` | 0.04, 0.05 Ω, 1.0 A, 80 A |
| Battery | 12S, two packs in parallel | `R0_pack` 0.030 Ω, `R1_pack` 0.020 Ω, `C1_pack` 2000 F, `I_av` 2 A, `V_min` 33 V |
| | capacity, nominal voltage | 25 Ah per pack, 43.2 V (datasheet) |
| Attitude | `omega_n`, `zeta`, `q_max` | 8 rad/s, 0.8, 1.5 rad/s |

Derived properties: `LiftRotorParams.R`, `.A_tot` (total disc area of all four rotors);
`BatteryParams.R0, R1, C1` (two packs in parallel: resistances halved, capacitance doubled),
`.e_nominal_wh` (= 2 × 25 × 43.2 = 2160 Wh, *nominal, not usable*), `.voc(soc)` (placeholder
linear open-circuit voltage `12 (3.3 + 0.9 soc)` V); `ideal_cq0(ct0)` gives the static torque
coefficient for a figure of merit of 1 (used by test T-03).

## `aero.py` — wing

Linear aerodynamics below stall, flat-plate behaviour above, joined by a sigmoid so gradients
exist (Cory and Tedrake for the flat plate; Beard and McLain for the blend — the blend's form is
**still to be checked against the book**, as the proposal says).

```
sigma(a) = (1 + e^{-M(a - a_s)} + e^{ M(a + a_s)}) / ((1 + e^{-M(a - a_s)}) (1 + e^{ M(a + a_s)}))     (6.2)
CL = (1 - sigma)(CL0 + CL_alpha a) + sigma CN sin a cos a                                              (6.3)
CD = (1 - sigma)(CD0 + k CL_lin^2) + sigma (CD0 + CN sin^2 a),   L,D = 0.5 rho V^2 S (CL, CD)        (6.4)
```

* `a` is the wing angle of attack `alpha + i_w`.
* **Sharpness `M = 100 /rad`.** With a smaller `M` the blend weight at `a = 0` is not small enough
  for test T-02's 1e-9 tolerance; `M = 100` with `alpha_s = 0.35` gives `sigma(0) ~ 1e-15`. The
  consequence is a stall "cliff" about 0.01 rad wide, which mattered a great deal for collocation
  (see `analysis/README.md`).
* `cl_max(p)` finds the peak of `CL` numerically (about 1.79 for the placeholder wing);
  `stall_speed(m, rho, p)` is the 1g stall speed `sqrt(2 W / (rho S CL_max))` (eq. 6.5).

## `rotor.py` — lift rotors and pusher

Coefficient maps over the axial advance ratio `J` and edgewise ratio `mu`:

```
w_s = sqrt(w^2 + w_floor^2)      n_s = w_s / 2 pi      n = w / 2 pi
J = clip(Vc / (n_s D), J_min, J_max)      mu = clip(Ve / (w_s R), 0, mu_max)         (6.6)
CT = (CT0 + CT1 J + CT2 J^2)(1 + k_mu mu^2)      CQ similarly      CH = CH1 mu
T = CT rho n^2 D^4,   Q = CQ rho n^2 D^5,   H = CH rho n^2 D^4                         (6.7)
```

`lift_rotor(omega, Vc, Ve, rho, p)` returns `(T_total, Q_per_rotor, H_total)` with the totals
multiplied by the number of rotors. The `w_floor` (1 rad/s) is used only inside `J` and `mu` to
keep them finite at standstill; the forces use the true `n^2`, so thrust is exactly zero at
`omega = 0`.

* **These are placeholder polynomials** (the proposal's "placeholder form until real data
  exists"). Rotor behaviour at high edgewise flow and negative axial inflow is flagged as the
  largest single uncertainty of the project.
* `pusher(omega, V_axial, rho, p)` uses a one-dimensional map in `J_p = V cos(alpha) / (n D)`.
  With these coefficients `C_T` goes negative at high `J_p`, so a windmilling pusher at zero
  throttle produces braking drag.
* `vrs_ratio(Vc, T_r, rho, p)` returns `V_c / v_h` with `v_h = sqrt(T_r / (2 rho A_tot))`. It is a
  *diagnostic* only: the vortex-ring avoidance boundary must be sourced from Johnson's NASA
  report, and no constraint uses an unsourced number.
* `ideal_hover_power(T, rho, A)` is momentum theory, `T^1.5 / sqrt(2 rho A)` (test T-03).
* The optional `smooth` argument replaces the `fmin/fmax` clips with a hyperbolic rounding.

## `motor.py` — motor and ESC

```
delta_eff = clip((delta - dz) / (1 - dz), 0, 1)                    ESC dead zone
I_m = clip((delta_eff V_bus - Ke w) / Rm, 0, I_lim)                                  (6.8)
Q_m = Kt (I_m - I0)   if I_m > 0, else 0
I_rot w_dot = Q_m - Q_i(J, mu, rho, w)                                               (6.9)
```

* Because the load torque `Q_i` depends on density, at fixed throttle a lower density unloads the
  motor and the thrust falls by *less* than the density ratio — with no fitted exponent.
* `clipped_current(raw, I_lim, smooth)` returns the clipped current **and its slope**, which the
  battery's Newton solve needs.
* `torque` uses `sign(I_m)` (exact) or `tanh(I_m / smooth)` (smooth) for the `I_m > 0` switch.
* `no_load_speed` is `(V_m − I0 Rm) / Ke` (test T-08).

## `battery.py` — equivalent-circuit pack and bus voltage

First-order circuit: open-circuit voltage minus a series resistance drop minus an RC-branch
voltage `v1`:

```
V_bus = V_oc(SoC) - R0 I_bus - v1,        v1_dot = -v1 / (R1 C1) + I_bus / C1        (6.10)
```

In the proposal the pack model is fitted offline (PyBaMM) and only a small table is loaded at run
time. Here the table is the placeholder `voc()` and constant `R0, R1, C1`.

### A correction to the proposal's eq. (6.11) — important

The proposal's printed closed form took the **bus current equal to the sum of motor currents**.
That is not energy-conserving: the ESC chops the bus, so the motor sees `delta * V_bus` and the
bus carries `delta * I_m`. As printed, the model drew about `1/delta` too much battery current —
roughly **3 times** at hover (3.5 kW instead of about 1.2 kW). It was found in milestone 4 when the
hover power looked wrong against a hand estimate. The model now uses

```
I_bus = n_r delta_r I_m,r + delta_p I_m,p + I_av
V_bus = ( V_oc - v1 + R0 ( n_r delta_r Ke w_r / Rm + delta_p Ke,p w_p / Rp - I_av ) )
        / ( 1 + R0 ( n_r delta_r^2 / Rm + delta_p^2 / Rp ) )                          (6.11, corrected)
```

(`delta` here is the *effective* throttle after the dead zone.) The proposal text in §6.11 was
edited to match. This changes every battery-related number and all tests still pass; if you
intended the original form, `battery.py`, `vehicle.py`, `dynamics/trim.py` and the test reference
are the places to revert.

* `closed_form_bus_voltage(...)` is valid only while no current is clipped or zero.
* `solve_bus_voltage(...)` handles clipping: Newton's method on the monotone scalar equation
  `g(V) = V - (V_oc - v1 - R0 I_bus(V))`, whose slope is at least 1, starting from the unloaded
  voltage and clamped to `[0, V_oc - v1]`. Twelve iterations are unrolled into the CasADi graph.
  Test T-07 checks the closed form against a bracketing root-finder to 1e-9 V where both apply,
  and 200 random clipped cases agree to 1e-6 V.

## `attitude.py` — pitch surrogate

```
theta_dot = q,    q_dot = wn^2 (theta_cmd - theta) - 2 zeta wn q,
|theta_cmd| <= theta_lim,    q is not accelerated further once |q| >= q_max               (6.12)
```

Full rotational dynamics are out of scope; this lag is a stand-in until `omega_n` and `zeta` are
calibrated from SITL or flight logs. `theta_lim` is an input supplied per scenario.

## `vehicle.py` — `CasadiVehicle`

`CasadiVehicle(params, smooth=0.0, frozen_states=())` builds `self.f(x, u, p) -> (xdot, aux)`.

* **State** `x = [x, h, vx, vh, theta, q, omega_r, omega_p, v1]`, **input**
  `u = [delta_r, delta_p, theta_cmd]`, **parameters** `p = [d_isa, headwind, theta_lim, soc]`.
* **Aux outputs** (`AUX_NAMES`): `L, D, T_r, T_p, H_r, I_bus, V_bus, alpha, V, gamma, vrs_ratio,
  I_rotor, I_pusher, rho, C_L`.
* **Cartesian dynamics** (ground-frame velocity components), because the flight-path-angle form
  divides by `V` and is singular at the end of the back-transition:

```
va = vx + headwind     V = sqrt(va^2 + vh^2 + 1e-9)     gamma = atan2(vh, va)     alpha = theta - gamma
Vc = vh cos(theta) - va sin(theta)       Ve = va cos(theta) + vh sin(theta)               (7.4)
m vx_dot = T_p cos(th) - T_r sin(th) - H_r cos(th) - L sin(g) - D cos(g)                  (7.2)
m vh_dot = T_p sin(th) + T_r cos(th) - H_r sin(th) + L cos(g) - D sin(g) - m g            (7.3)
```

  The `1e-9` keeps the square root differentiable at `V = 0`. Test T-10 checks the Cartesian form
  against the flight-path-angle form at 100 random states to 1e-6.
* `derivatives(x, u, p)` is a NumPy wrapper returning `(xdot, aux_dict)`.
* `rk4_step(dt, substeps=4)` returns a compiled RK4 step (cached per `(dt, substeps)`), which is
  what `simulate()` uses.
* **`frozen_states`** zeroes the named state derivatives. It is a **test harness only** (for
  example "vertical motion held at zero", used by T-04, T-05, T-06 and the braking limit-case
  test); analyses never use it.

## Known limitations and modelling caveats

* **Torque even in speed.** `Q_i` and `T` are proportional to `n^2`, so they are even in the
  rotor speed. A rotor driven to slightly negative speed is accelerated further negative by the
  load torque instead of being restored. In practice the speed approaches zero from above and never
  crosses it in continuous time; with a fixed-step integrator it can dip fractionally below zero.
  Collocation imposes `omega >= 0` as a constraint. A proper `n|n|` formulation is not implemented.
* **`mu` is clipped at zero**, so reverse edgewise flow produces no in-plane force.
* **`V` is a magnitude**, so when the aircraft moves backward the pusher still sees "forward"
  axial flow. This only matters if a controller lets the aircraft drift backward.
* **No motor thermal limit, demagnetisation, ESC switching loss beyond the duty-cycle balance, or
  rotor-wing interference** (recorded as limitations).
* **Placeholder battery:** linear `V_oc`, constant resistances, constant avionics current, no
  temperature or ageing dependence.
* **The J/mu clips and the ESC dead zone are corners.** They are smoothed for collocation; at
  `smooth = 0` they are exact.

### Non-smooth or singular points, and what is and is not done about them

These matter for gradient-based use (analysis B1) and were pointed out in review:

| Item | Status |
| --- | --- |
| **Stall blend** (`M = 100 /rad`, a cliff about 0.01 rad wide) | **Not smoothed.** It is the model. Collocation *works around* it with a stall margin (`|alpha_w| <= 0.85 alpha_s` while the wing carries load), enforced at nodes and midpoints; without that the optimiser flew through the cliff between nodes and its solutions failed replay. |
| **`gamma = atan2(vh, va)`** | **Not regularised, and its derivative is undefined at `V = 0`.** `V` itself is protected (`sqrt(... + 1e-9)`) but `gamma` is not. Simulation never differentiates it, and B1 stays away from hover in still air or a headwind (the end condition is ground speed 1 m/s and `|vh| <= 0.5 m/s`, so airspeed is about 1 m/s or more at the last node). **A tailwind equal to the end ground speed would put the airspeed near zero at the last node and approach the singularity**; B1 has not been run there. A formulation that must reach hover exactly needs a regularised flight-path angle. |
| **Attitude rate limit** (`if_else` on `q >= q_max`) | **Not smoothed** (it is non-smooth in both exact and smooth modes). It is inactive in the B1 solutions: the largest pitch rate seen was about 1.0 rad/s against `q_max = 1.5`. If a problem drives `q` to the limit, expect gradient trouble there. |
| **Motor torque switch at zero current** | **Smoothed when `smooth > 0`** (`tanh(I_m / smooth)`); exact `sign` for simulation. |
| **Current clip and rotor-inflow clips** | Smoothed when `smooth > 0` (hyperbolic rounding). |
| **ESC dead zone** | Smoothed when `smooth > 0`; B1 avoids it entirely by using the *effective* throttle as its control. |

## Tests

`tests/unit/test_physics.py` (10 tests): **T-02** wing limits and continuity; **T-03** hover
power with figure of merit 1 within 0.1 %; **T-07** closed form versus iterative solve, plus 200
clipped random cases; **T-08** motor steady state and no-load speed; **T-15** NumPy reference
against CasADi at 100 random inputs to 1e-9 relative; Hypothesis checks (thrust rises with speed
and density, drag non-negative, bus voltage falls as current rises); the vehicle's finiteness and
the 60 s speed check; smooth-versus-exact agreement.
