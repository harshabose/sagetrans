# `sagetrans.dynamics` — time stepping, events, and steady-flight helpers

Layer position: above `physics`, below `control`. It joins a vehicle model to a controller in
time (`simulate`), defines the events that end or mark a segment (`events`), and provides numeric
inversion and steady-state helpers that the analyses and controllers need (`trim`).

| File | Contents |
| --- | --- |
| `simulate.py` | `Environment`, `Result`, `EventRecord`, `compute_metrics`, `simulate` |
| `events.py` | `Event` and the standard event factories |
| `trim.py` | rotor/pusher inversion, electrical operating points, steady and glide trim, hover |

## `simulate()` — what it does, step by step

```python
simulate(vehicle, controller, env, x0, t_end, events=None, *, mode0=None,
         dt=0.02, substeps=4, integrator="rk4") -> Result
```

* **Fixed controller step** `dt = 0.02 s` (an assumed 50 Hz); commands are **held constant**
  over each step (zero-order hold).
* **Default integrator:** the compiled CasADi RK4 from `vehicle.rk4_step`, with 4 sub-steps per
  controller step (5 ms). Deterministic and fast (NFR-01, NFR-03).
* **Reference integrator** (`integrator="reference"`): `scipy.integrate.solve_ivp` with LSODA,
  `rtol=1e-6`, `atol=1e-9`, integrated over each controller step with the command held. It
  exists to confirm the default step is converged (T-09). The proposal also suggested Radau; only
  LSODA is implemented.

Each iteration of the loop:

1. `u, mode = controller.command(t, x, aux_prev, mode)`. **The controller sees the auxiliary
   outputs of the *previous* step** (`aux_prev`), because they depend on the command. This is a
   one-step (20 ms) lag; for example a speed-triggered mode switch happens one sample after the
   speed was reached.
2. The vehicle is evaluated once with `(x, u, p)` to obtain this sample's `aux` outputs.
3. Each event function is evaluated; a sign change since the previous sample is detected
   (below).
4. The sample is recorded, then `x` is advanced one step with RK4.

`Result.t` and `Result.columns` hold the histories. Columns are the 9 states, the 3 inputs and the
15 auxiliary outputs, by name (for example `res["h"]`, `res["V_bus"]`). `Result.modes` is the
controller's mode name at each sample (empty strings if the mode state has no `name`).
`Result.final_mode` is the mode state after the last command; `terminated_by` is the name of the
terminal event, or `None` if the run reached `t_end`.

### `Environment`

Frozen, per-scenario, constant over a segment.

| Field | Meaning |
| --- | --- |
| `altitude_amsl_m` | terrain elevation under the track (so `h_AGL = h - altitude_amsl_m`) |
| `isa_offset_k` | ISA temperature offset |
| `headwind_ms` | positive **against** the aircraft |
| `gust` | optional `t -> m/s`, **added to the headwind and evaluated once per step** |
| `soc` | pack state of charge, constant within a segment (default 0.8) |
| `theta_max_rad` | pitch-command limit passed to the attitude surrogate (default 1.2) |

`param_vector(t)` gives the `[d_isa, wind, theta_max, soc]` vector the vehicle expects.

### Events

An `Event(name, fn, direction, terminal)` has `fn(t, x, aux) -> float`; it fires when `fn`
crosses zero. `direction` is `+1` (rising), `-1` (falling) or `0` (either). Detection compares
consecutive samples: a rising crossing is `g_prev < 0 <= g`, a falling one is `g_prev > 0 >= g`.
The crossing time and state are found by **linear interpolation** within the step. A terminal event
truncates the run at the interpolated point (the sample that detected it is replaced by the
interpolated one, with the previous command and interpolated aux outputs).

Subtleties:

* An event whose function is already past zero at the first sample **never fires** (there is no
  previous sample on the other side).
* Several events can fire in one step; only the first terminal one stops the run.

Standard factories in `events.py`:

| Factory | Fires when | Terminal |
| --- | --- | --- |
| `airspeed_reached(v_min)` | `V >= v_min` (rising) | optional |
| `battery_cutoff(v_min)` | `V_bus < v_min` (falling) | yes |
| `touchdown(ground_amsl_m)` | `h` falls to the given elevation | yes |
| `ground_speed_below(v_f)` | `vx` falls to `v_f` (end of the back-transition) | yes |
| `vrs_entry(limit)` | `V_c/v_h` falls below `limit` (a **sourced** boundary is required) | no |

(`ground_speed_below` has a `headwind` argument that is unused: `vx` is already a ground speed.)

### Metrics (`compute_metrics`)

Computed from the histories of one segment, relative to the starting altitude `h_ref`:

| Metric | Definition |
| --- | --- |
| `altitude_loss` | `max(0, h_ref - min h)` |
| `altitude_gain` | `max(0, max h - h_ref)` |
| `altitude_excursion` | `max(loss, gain)` — the **signed** excursion of the proposal's decision 4 |
| `distance` | `x_end - x_start` |
| `duration` | `t_end - t_start` |
| `energy_J` | trapezoidal integral of `V_bus * I_bus` |
| `peak_bus_current`, `min_bus_voltage` | extremes over the segment |

## `trim.py` — inversion and steady flight

These call the **same** CasADi physics as the vehicle, so there is no second copy of the models.

| Function | Purpose |
| --- | --- |
| `rotor_speed_for_thrust(T, Vc, Ve, rho, vp)` | lift-rotor speed giving a total thrust (bracketing root-find; 0 for `T <= 0`) |
| `pusher_speed_for_thrust(T, v_axial, rho, vp)` | same for the pusher |
| `required_rotor_command(omega, omega_dot, ...)` | throttle that produces a given rotor acceleration (eq. 6.9 inverted), by fixed-point iteration with the bus voltage; returns a `RotorDemand` |
| `hover_omega`, `hover_throttle`, `hover_power`, `hover_state` | steady hover: rotor speed, command (before the dead zone), electrical power, full operating point |
| `electrical_state(vp, soc, v1, omega_r, q_rotor, omega_p, q_pusher)` | bus voltage, current, power and throttles for given speeds and load torques |
| `steady_trim(vp, rho, V, gamma, soc)` | level or steady-climb trim with the pusher; rotors off |
| `glide_trim(vp, rho, V, soc)` | pusher-off glide |

### `electrical_state` — why bus voltage is explicit

Motor current is fixed by torque alone, `I = Q/Kt + I0`. The ESC passes motor power to the bus,
`I_bus = P_m / V_bus + I_av`, with `P_m = n (Rm I + Ke w) I` summed over motors. Substituting into
`V_bus = V_oc - v1 - R0 I_bus` gives a quadratic with an explicit root:

```
V_bus = ( B + sqrt(B^2 - 4 R0 P_m) ) / 2,      B = V_oc - v1 - R0 I_av
```

If the discriminant is negative the pack cannot deliver that motor power at all, and the result
is flagged infeasible. The throttle follows afterwards, `delta = (Rm I + Ke w) / V_bus`, converted
to a command with the dead zone. `feasible` is true when currents are within limits, throttles are
at most 1, the bus voltage is above the cut-off, and the power is transferable.

### `steady_trim` — the exact force balance

With the pusher active and rotors off, level or climbing at flight-path angle `gamma`, the force
balance (7.2)–(7.3) reduces to

```
T_p = (D + W sin gamma) / cos alpha          L + (D + W sin gamma) tan alpha = W cos gamma
```

solved for `alpha` by bracketing (`ValueError` if the wing cannot carry the weight at that speed).
Returned: `alpha`, `theta = alpha + gamma`, pusher thrust and speed, lift, drag, and the electrical
state. The cruise-trim test builds the vehicle state from this trim and checks the simulated
accelerations are zero to 1e-6 of the weight — so the trim is exactly consistent with the model.

`glide_trim` solves `D = -W sin gamma`, `L = W cos gamma` with `T_p = 0` for `alpha` (and then
`gamma`) at a given airspeed.

## Tests

`tests/unit/test_dynamics.py` (7 tests):

| ID | Test |
| --- | --- |
| T-04 | constant-deceleration limit: stopping distance `V^2 / (2 g tan theta)` within 0.5 % |
| T-05 | coast limit: distance `L_d ln(V0/V)` with `L_d = 2m / (rho S CD)` within 0.5 % |
| T-06 | mechanical energy conserved to 1e-6 relative over 60 s with thrust and drag off |
| T-09 | RK4 against the adaptive reference: excursion within 1 % or 0.05 m |
| T-10 | Cartesian dynamics against the flight-path-angle form at 100 random states, 1e-6 |
| — | events and metrics, determinism (NFR-01), terminal-event truncation |

The steady-flight helpers are tested in `tests/unit/test_energy_descent.py` against the simulated
vehicle (hover and cruise trims reproduce zero acceleration and the same bus voltage/current).
