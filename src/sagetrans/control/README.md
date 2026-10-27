# `sagetrans.control` — controllers

Layer position: above `dynamics`, below `analysis`. A controller maps what it can observe to the
input vector `u = [delta_r, delta_p, theta_cmd]` (rotor throttle command, pusher throttle command,
commanded pitch). **Controllers know nothing of analyses** (NFR-05).

| File | Contents |
| --- | --- |
| `base.py` | `ModeState` and the `Controller` protocol |
| `ardupilot_like.py` | the ArduPilot-structured baseline: `Position1Controller`, `BaselineController` and their parameter classes |
| `freeform.py` | the speed-scheduled free-form law used by analysis B2 |

## The interface (`base.py`)

```python
class Controller(Protocol):
    def command(self, t, x, aux, mode) -> tuple[np.ndarray, ModeState]: ...
```

* `x` is the full 9-state vector; `aux` is the **previous step's** auxiliary outputs as a dict
  (so a controller reading `aux["V"]` sees a 20 ms-old airspeed).
* `ModeState` is an immutable dataclass: `name`, `t_enter`, `t_last` and a `memory` dict of floats
  (integrators and the like). Controllers return a *new* state rather than mutating, so a run is
  reproducible and a controller object can be reused.
* **Integrators advance by elapsed time** (`t - mode.t_last`), not by a stored step size. This was
  a deliberate fix: a `dt` held inside the controller could silently disagree with `simulate`'s
  step and mis-scale the integral.
* On the first call `mode` is `None`; the controller creates its initial state.

## `Position1Controller` — the back-transition

This reproduces the *documented structure* of ArduPilot's back-transition, not its source code.
Behaviours marked "assumed" in the proposal are placeholders until SITL or flight logs calibrate
them. Pusher command is always 0 (documented: the pusher stops immediately).

### Rotor altitude hold (eq. 8.5)

```
v_dem = clip(K_h (h_tgt - h), +-v_dem_max)
delta_r = clip( delta_hover + K_vh (v_dem - vh) + K_i * integral(v_dem - vh), delta_min, delta_max )
```

`delta_hover` is a feed-forward from `dynamics.trim.hover_throttle`; `delta_min` is the
`Q_M_SPIN_MIN` rotor floor (rotors idle there rather than at zero). Defaults: `K_h = 0.5 /s`,
`K_vh = 0.02`, `K_i = 0.01`, `v_dem_max = 3 m/s`, `delta_max = 0.95`. **These gains are assumed**;
sensitivity analysis ranks the first two among the dominating inputs for altitude excursion.

### Pitch envelope (eq. 8.3)

```
ramp(t) = start + (cap - start) * min(1, t / T_bt),   cap = min(theta_A, theta_P),
start = clip(theta_fw0, 0, cap)
theta_lim(t) = min(ramp, theta_B)                                        pit_lim_mode "constant"
theta_lim(t) = min(ramp, theta_A, theta_B * (V_min / EAS)^2)             pit_lim_mode "airspeed"
```

`theta_A` is `Q_A_ANGLE_MAX`, `theta_P` is `PTCH_LIM_MAX_DEG`, `T_bt` is `Q_BACKTRANS_MS` (3 s), and
`theta_B` is `Q_BCK_PIT_LIM` (no cap by default). The ramp starts at the last fixed-wing pitch
`theta_fw0` (default 0) with the clock at the AIRBRAKE exit (O-04); at 0 the limit starts at zero,
so early in the back-transition the aircraft cannot pitch and brake, which is why the planner's
constant deceleration is optimistic (analysis D). The airspeed form (O-05) needs `theta_B`,
`v_min_ref` and the equivalent airspeed (the controller passes it from `aux`), and raises
`ValueError` otherwise; `theta_B <= 0` disables it. It is off by default.

### Feed-forward, alpha cap and rotor lag (analysis D)

* `ff_mode="lift_aware"` (with a `LiftFF` table built from the scenario vehicle) replaces the
  constant `delta_hover` by the throttle for `(W - L_hat) / cos(theta)`; `L_hat` is the simulated
  lift times `ff_lift_scale`, filtered with time constant `ff_lift_tau_s`. At tau 0 and scale 1 it
  is an oracle. The default of the dataclass stays `"repo"`, so C and B2 are unchanged.
* `alpha_cap` limits the pitch command to `alpha_cap + gamma` while `V >= alpha_cap_vmin`.
* `rotor_lag_s` is the one lumped actuator lag of D, first order on the rotor throttle (ESC slew,
  filtering, inertia beyond `I_rot`); a sensitivity, not the spool law.
* Filter states live in `ModeState.memory` (`L_f`, `d_eff`) and are carried through each step.

### Position cascade (eqs. 8.9, 8.4, 8.10)

```
d = x_tgt - x                  d_lin = 2 a_lim / K_p^2
v_des = sqrt(2 a_lim d)  if d > d_lin   else   K_p d
a_cmd = K_v (vx - v_des)       (positive = braking)
theta_cmd = clip( atan(a_cmd / g), 0, theta_lim(t) )
```

With `brake_at_limit = True` the controller commands `theta_cmd = theta_lim(t)` throughout; this is
analysis D's **worst case**. `BackTransitionParams.for_vehicle(vp, rho, soc, h_tgt, **overrides)`
builds a parameter set with the hover feed-forward computed for that vehicle and conditions.

### Known behaviour to be aware of

* **It only brakes.** `theta_cmd >= 0`; there is no nose-down command. If the aircraft overshoots
  the target it will pitch to the limit to pull back and then drift backward with no way to
  recover. Analysis D therefore ends every run at the first time the ground speed reaches `V_f`.
* **Altitude gain.** Pitching up while the wing still carries lift climbs the aircraft; the
  placeholder runs show a gain of roughly 12–14 m (the "documented nose-up climb on
  back-transition").
* **Low-speed sink.** With rotors starting from rest and slow altitude-hold gains, the aircraft
  can sink at several m/s around 6–10 m/s, with the wing stalled while the rotors catch up.

## `BaselineController` — forward transition, cruise stub, back-transition

Built from a `BaselineParams` (which wraps a `BackTransitionParams`). Modes, in order:

| Mode | Behaviour |
| --- | --- |
| `fwd_wait` | pusher at `TKOFF_THR_MAX` (the documented AUTO behaviour); rotors in altitude hold with the `Q_M_SPIN_MIN` floor; level pitch. Leaves when airspeed reaches `AIRSPEED_MIN`. If `Q_TRANS_FAIL > 0` and the wait exceeds it, goes to `aborted` (rotors keep holding altitude; pusher off). |
| `fwd_fade` | rotor throttle `= (1 - (t - t0)/T_fade)^+ * delta_hold` (eq. 8.7, a **linear** fade of assumed shape over `Q_TRANSITION_MS`); pusher stays at `TKOFF_THR_MAX` until the fade completes. |
| `cruise` | rotors off. Pusher command hands off from `TKOFF_THR_MAX` to a speed-hold demand through a rate limiter (`r_tecs`, `tau_h`; eq. 8.8, form assumed); pitch holds altitude with a PD law. Leaves when the switch test fires. |
| `position1` | the `Position1Controller` logic above. |

**Switch-point test (eq. 8.11):** enter `position1` when `x_tgt - x <= V_ref^2 / (2 * a_plan)`,
with `a_plan = Q_TRANS_DECEL`. `V_ref` is the ground speed or the air speed per
`speed_test` (`"ground"`/`"air"`), because which one ArduPilot uses is open question O-02.
In a headwind the air-speed test fires earlier (safe, but costs VTOL energy); the tests verify
exactly this.

**Not modelled:** `BATT_WATT_MAX`, TECS energy blending, the airbrake phase, the rotor spool ramp
on mode switch, `Q_OPTIONS` level-transition. **Gains are assumed.** The cruise stub holds speed
and altitude within about 1 m/s and 1.5 m in the placeholder model.

## `FreeFormController` (`freeform.py`) — eq. 8.1

For analysis B2. Rotor throttle and pitch command are **piecewise-linear in normalised airspeed
`V / V_s`**, with `V_s` the scenario's 1g stall speed:

```
delta_r = f(V/V_s)   if V/V_s <= s_on  else 0          theta_cmd = k(V/V_s)         pusher = 0
```

* Knots are shared by both channels at `V/V_s = 0.1, 0.4, 0.8, 1.2, 1.6, 2.4`; `FreeFormParams`
  holds the knot values and `s_on` (the rotor start speed, `V_on / V_s`, a decision variable).
* It is robust to mass and density because it reacts to the speed actually flown.
* **It has no altitude feedback**, so it cannot correct a disturbance it did not anticipate; and
  `V/V_s` does not absorb the throttle-to-thrust dependence on density and pack voltage.

## Tests

`tests/unit/test_m5.py` (the controller parts): the free-form law's interpolation, rotors-off above
`s_on`, and independence from altitude and climb rate; the baseline's mode sequence
`fwd_wait → fwd_fade → cruise → position1`, the wait for `AIRSPEED_MIN` (allowing for the one-step
lag), `TKOFF_THR_MAX` during the wait, the fade duration and zero rotor throttle at its end, the
cruise stub's settling, the switch test firing at the planned distance, the air-speed test
switching earlier in a headwind, and `Q_TRANS_FAIL` aborting a stuck transition while the rotors
keep holding altitude. `tests/unit/test_analyses.py` covers the altitude hold in hover and the
braking behaviour.
