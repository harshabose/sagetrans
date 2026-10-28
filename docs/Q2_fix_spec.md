# Analysis Q2 fix specification (pusher acceleration, feed to E): hand-off for an implementing agent

Repo: `sagetrans` (Python, `src/sagetrans/...`). This file is self-contained. **Status: implemented** (branch `feat/q2-pusher-accel`; section 11 checks 1 to 12 were run, tests are left to the owner) (written 26 Oct 2026). The numbers below come from scratch scripts run on the repo's real configs in draft mode (`vehicle.yaml`, `study.yaml` Q2 grid, `controller.yaml`); they are expected values for the acceptance checks, not repo changes. See section 12 for how they were made.

**Hand-off order.** This spec is self-contained and goes first. It leaves E's structure as it is today (one leg, the existing cruise-altitude loop, one mean SoC). A separate redesign of E (`E_fix_spec.md`: airfield-elevation sweep, round-trip and one-way missions, two leg masses, SoC per leg) is written afterwards against the code this spec produces; it reuses `pusher.forward_accel` unchanged and only changes how often, and with which mass and SoC, `stage_e` calls it (see the scope note in section 6).

## 0. Decisions already taken by the project owner (do not re-open)

1. **What Q2 asks.** The controller matches the rotors to whatever lift the wing does not provide, until `AIRSPEED_MIN` is reached, however the wing produces it. So the question reduces to: **what forward acceleration can the pusher deliver at `TKOFF_THR_MAX`, from rest to `AIRSPEED_MIN`, at each site** (altitude, state of charge, mass). The old "full throttle, L = W, one speed" margin is not that question.
2. **Pusher command is `TKOFF_THR_MAX`** (register value 0.8, status TBD), not full throttle. It passes through the ESC dead zone (`motor.effective_throttle`), so 0.8 is an effective 0.789.
3. **Lift model is E's own**: the wing carries the share `(V / V_lw)^2` of the weight, `V_lw = stall_speed_1g / sqrt(0.8)` (the speed where the wing carries the weight at 0.8 of `C_L,max`); the rotors carry the rest and load the same battery bus. Keep this model in one place and use it from both Q2 and E.
4. **E takes its forward-transition acceleration from Q2.** The value used is the **worst over speed** (the minimum of the acceleration profile, which sits at `AIRSPEED_MIN`), not a mean. Two sources, switchable (`E.a_fwd_mode`):
   - `case` (default): Q2's model evaluated at **E's own site** (clearance altitude AMSL, `mp.isa_offset_k`, `mp.soc_mean`, nominal mass). This is the "matched case".
   - `worst`: the **global worst over the Q2 grid** (altitude x mass factor x SoC).
   - `mission` keeps today's behaviour (`mp.a_fwd` from `mission.yaml`, 1.5 m/s^2) so the old numbers can be reproduced.
   Q2 always reports both numbers; E carries whichever mode is set and records all three in `E_summary`.
   In this PR E keeps its current structure; the redesign of E is a later, separate spec (see the header).
   *Reading note:* "case matched with global worst as worst" was read as "case-matched value for E, global worst as the worst-case figure". If the intent was the reverse default, change only the default of `E.a_fwd_mode`.
5. **Airspeed convention.** `AIRSPEED_MIN` is registered as TAS in `controller.yaml`; the old Q2 read it as EAS. **Both are kept**: Q2 reports every row in both conventions (`convention` column). The value fed to E uses `Q2.feed_convention` (default `tas`, because E treats `v_trans` as TAS). DEC-1 (the project-wide TAS/EAS decision) is still open and changes these numbers at altitude; see section 10.

## 1. Why (short)

- Q2 today computes the pusher thrust at **full throttle (effective 1.0)** minus wing drag at **L = W**, at one speed, with **the rotors off the bus**. At `AIRSPEED_MIN` the wing does carry nearly the weight, but on the way up the rotors carry most of it and draw current from the same pack, and the controller does not command full throttle: `TKOFF_THR_MAX` is 0.8.
- The throttle matters a great deal. At E's site (30 m, SoC 0.6, nominal mass) the minimum acceleration is 2.68 at command 1.0, 1.94 at 0.9, **1.24 at 0.8**, 0.59 at 0.7 m/s^2.
- Q2 has no link to E. E's forward-transition acceleration is `a_fwd = 1.5` in `mission.yaml` ("ASSUMED until analysis C"), so E cannot say whether the hardware can deliver it.
- The speed is read as EAS in `stage_q2` but as TAS in E and in the register; at 5000 m the two differ by 29 percent (17 EAS = 21.9 TAS).

## 2. Files to change

| file | change |
| --- | --- |
| `src/sagetrans/analysis/pusher.py` | **new**, section 3 |
| `src/sagetrans/analysis/energy.py` | factor out two helpers, use the Q2 value in the budget, section 4 |
| `src/sagetrans/io/stages.py` (`stage_q2`, `stage_e`) | sections 5 and 6 |
| `src/sagetrans/io/settings.py` (`Q2Cfg`, `ECfg`) | section 7 |
| `src/sagetrans/io/inputs.py` (`StudyInputs`) | expose `tkoff_thr_max` and `airspeed_min`, section 7 |
| `configs/study.yaml` (`Q2:` and `E:` blocks) | section 7 |
| `src/sagetrans/analysis/README.md` (E and Q2 text) | section 9 |

`pusher_margin` in `stages.py` stays as a legacy function (nothing else imports it; checked with grep): do not delete it, do not call it from `stage_q2` any more.

Out of scope: any change to `physics/*`, `dynamics/*`, `control/*`; the battery model itself (linear OCV, no pack current limit, open item O-15; section 10); the TAS/EAS decision DEC-1; analyses A, B, C, D, F; tests (the owner adds them).

## 3. New module `analysis/pusher.py`

One public function and one result class. The prototype below was run against the repo and reproduces every number in section 8; copy it, then change only imports and style.

```python
"""Forward acceleration the pusher can deliver from rest to a target airspeed (analysis Q2)."""
from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from sagetrans.analysis import energy
from sagetrans.dynamics import trim
from sagetrans.physics import battery, motor, rotor
from sagetrans.physics.params import VehicleParams

G = 9.80665


@dataclass(frozen=True)
class FwdAccel:
    V: np.ndarray            # true airspeed grid, 0 .. v_end [m/s]
    a: np.ndarray            # net forward acceleration [m/s^2]
    a_min: float             # minimum over the grid (in practice at v_end)
    v_at_min: float
    a_equiv: float           # constant a that covers the same distance: v_end^2 / (2 x); NaN unless reachable
    t_s: float               # time from rest to v_end along the profile; NaN unless reachable
    x_m: float               # distance, same
    v_reach: float           # highest grid speed reached with a > 0 continuously from rest
    reachable: bool          # a > 0 over the whole grid
    rotor_saturated: bool    # lift rotors hit trim.OMEGA_MAX somewhere
    current_limited: bool    # pusher or rotor current limit active somewhere


def _point(vp: VehicleParams, rho: float, V: float, soc: float, cmd: float):
    """Net forward acceleration at airspeed V: pusher at `cmd`, rotors carrying W - L."""
    b, mr, mp = vp.battery, vp.motor_r, vp.motor_p
    w_n = vp.m * G
    share = energy.rotor_share(vp, rho, V)               # fraction of the weight the rotors carry
    t_r = share * w_n
    om_r = trim.rotor_speed_for_thrust(t_r, 0.0, V, rho, vp) if t_r > 0.0 else 0.0
    sat = t_r > 0.0 and om_r >= trim.OMEGA_MAX - 1e-9
    if om_r > 0.0:
        _, q_r, h_r = (float(x) for x in rotor.lift_rotor(om_r, 0.0, V, rho, vp.rotor))
        i_r = q_r / mr.Kt + mr.I0                         # per-rotor current
    else:
        h_r = i_r = 0.0
    drag = energy.wing_polar_drag(vp, rho, V, (1.0 - share) * w_n) if V > 0.5 else 0.0
    d_p = float(motor.effective_throttle(cmd, mp))        # ESC dead zone: 0.8 -> 0.789
    voc = float(b.voc(soc))

    def bus(w_p: float) -> float:
        vb = voc
        for _ in range(8):                                # rotor duty depends on the bus voltage
            d_r = (mr.Ke * om_r + mr.Rm * i_r) / max(vb, 1.0) if om_r > 0.0 else 0.0
            vb = float(battery.solve_bus_voltage(
                voc, 0.0, b, vp.rotor.n_r, mr, mp, d_r, d_p, om_r, w_p))
        return vb

    def resid(w_p: float) -> float:
        i_p = float(motor.current(d_p, bus(w_p), w_p, mp))
        return float(motor.torque(i_p, mp)) - float(rotor.pusher(w_p, V, rho, vp.pusher)[1])

    w_p = brentq(resid, 1.0, 4000.0, xtol=1e-9)
    t_p = float(rotor.pusher(w_p, V, rho, vp.pusher)[0])
    i_p = float(motor.current(d_p, bus(w_p), w_p, mp))
    lim = i_p >= mp.I_lim - 1e-6 or i_r >= mr.I_lim - 1e-6
    return (t_p - drag - h_r) / vp.m, sat, lim


def forward_accel(vp, rho, v_end, soc, cmd, n=41) -> FwdAccel:
    """Acceleration profile over `np.linspace(0, v_end, n)` (v_end is a TRUE airspeed)."""
    V = np.linspace(0.0, v_end, n)
    a = np.empty(n)
    sat = lim = False
    for i, v in enumerate(V):
        a[i], s, l = _point(vp, rho, float(v), soc, cmd)
        sat |= s
        lim |= l
    i_min = int(np.argmin(a))
    ok = bool((a > 0.0).all())
    if ok:
        t = float(np.trapezoid(1.0 / a, V))
        x = float(np.trapezoid(V / a, V))
        a_eq = v_end**2 / (2.0 * x)
    else:
        t = x = a_eq = math.nan
    bad = np.flatnonzero(a <= 0.0)
    v_reach = v_end if ok else (float(V[bad[0] - 1]) if bad[0] > 0 else 0.0)
    return FwdAccel(V, a, float(a[i_min]), float(V[i_min]), a_eq, t, x, v_reach, ok, sat, lim)
```

Notes for the implementer:

- The model, per speed `V` on the grid: wing lift share by E's rule; `T_r = share * W` from the rotors at zero axial inflow and forward-flight inflow `V` (as E does: `rotor_speed_for_thrust(T_r, 0, V, rho, vp)`); wing drag `q S (C_D0 + k C_L^2)` for the remaining lift (zero below 0.5 m/s); pusher speed from motor torque = propeller torque at the **loaded** bus voltage, with the rotor current on the same bus (hence the fixed point, 8 passes); net acceleration `(T_p - D - H_r) / m`, where `H_r` is the rotor H-force as `lift_rotor` returns it (a few tenths of a newton; its sign convention is the model's, keep it).
- `a_min` is the value E and the worst-case table use. It lies at `v_end` in every case checked, but take the true minimum over the grid.
- `a_equiv` (distance-equivalent) and `t_s`, `x_m` are reported for information and for the E metric option (section 6). A non-positive acceleration anywhere sets them to NaN and `reachable = False`; `v_reach` then says how far the pusher gets.
- Cost: one `forward_accel` call is about 0.3 s (41 speeds). The Q2 grid with both conventions is 72 calls, about 25 s; E adds 2 more.
- Do not use `smooth` arguments; this is a plain numerical evaluation, not part of any collocation problem.

## 4. `energy.py`

Two small refactors with **no change in numbers** (check 1):

1. New public helper `rotor_share(vp, rho, V) -> float` returning `max(0, 1 - (V / v_lw)**2)` with `v_lw = stall_speed_1g(vp, rho) / sqrt(0.8)`. `forward_transition_estimate` calls it instead of its inline expression. (The `0.8` is E's existing assumption; leave it a module constant `LIFT_FRACTION_AT_V_LW = 0.8` if you like, but keep the value.)
2. Rename `_wing_polar_drag` to `wing_polar_drag` (keep a `_wing_polar_drag = wing_polar_drag` alias; no other module imports it, checked with grep).

`forward_transition_estimate` itself is unchanged: it still takes `mp.a_fwd`. The new acceleration reaches it through `dataclasses.replace(mp, a_fwd=a_used)` in `stage_e` (section 6). Update its docstring: "a_fwd comes from analysis Q2 (`E.a_fwd_mode`) or the mission file".

Do not make `energy.py` import `pusher.py` (it would create an import cycle through `trim`/`rotor` for no gain); `pusher.py` imports `energy`.

## 5. `stage_q2` in `io/stages.py`

Replace the body. For each `(altitude, mass factor, SoC, convention)`:

```python
c = inp.study.Q2
cmd = inp.tkoff_thr_max                       # TKOFF_THR_MAX from the register
v_reg = inp.airspeed_min                      # AIRSPEED_MIN
for alt in c.altitudes_m:
    sc = Scenario(alt, isa_offset_k=c.isa_offset_k)
    for mf in c.mass_factors:
        vps = dataclasses.replace(vp, m=vp.m * mf)
        for conv in c.conventions:             # ["tas", "eas"]
            v_end = v_reg if conv == "tas" else tas(v_reg, sc)
            for soc in c.soc:                  # [0.8, 0.6, 0.3]
                f = pusher.forward_accel(vps, sc.rho, v_end, soc, cmd, c.n_speed)
                rows.append({...})
```

Tables written (replace the old `Q2_pusher_margin`):

- `Q2_pusher_margin` (one row per cell): `convention, altitude_m, rho_kg_m3, mass_kg, soc, tkoff_thr_max, v_end_tas_ms, a_min_ms2, v_at_min_ms, a_equiv_ms2, t_s, x_m, v_reach_ms, reachable, rotor_saturated, current_limited`.
- `Q2_worst` (one row per convention): the cell with the smallest `a_min_ms2`, with its altitude, mass factor, SoC and the same metrics. The row for `Q2.feed_convention` is the one E reads for `worst` mode.
- `Q2_profile` (optional, `Q2.write_profile: bool = False`): `convention, altitude_m, mass_kg, soc, V_tas_ms, a_ms2` for every cell. Off by default (72 x 41 rows).

An unreachable cell (`reachable = False`) still gets a row: `a_min_ms2` negative or zero, `a_equiv_ms2`, `t_s`, `x_m` NaN. Never drop it and never clip `a_min` at zero. Q2 stays a table; it does not raise.

## 6. `stage_e`: take the acceleration from Q2

`stage_e` runs after `stage_d` and before `stage_q2` in `task_basics`; reorder so **Q2 runs before E** (both are quick; the `wanted` handling stays). If Q2 is not in `inp.study.analyses` and the mode needs it, compute the cells on the fly through a shared helper `_q2_cells(inp)` (cached on the function object or passed through `out`), so E never silently falls back.

```python
mode = inp.study.E.a_fwd_mode                     # "case" | "worst" | "mission"
mp0 = inp.mission
rho_c = float(atmosphere.density(mp0.clearance_amsl, mp0.isa_offset_k))
v_end = mp0.v_trans if feed_conv == "tas" else mp0.v_trans * sqrt(RHO0 / rho_c)
case = pusher.forward_accel(vp, rho_c, v_end, mp0.soc_mean, inp.tkoff_thr_max, c.n_speed)
a_case = case.a_min
a_worst = <a_min of the Q2_worst row for feed_convention>
a_used = {"case": a_case, "worst": a_worst, "mission": mp0.a_fwd}[mode]
```

- `E.a_fwd_metric`: `"min"` (default, the worst over speed, as decided) or `"equiv"` (distance-equivalent `a_equiv`). Applies to `case` and `worst`.
- If `a_used <= 0` (or NaN because the target speed is not reachable), **do not fall back**: append an error to `errors`, write an `E_summary` row `{"case": "forward_transition", "error": ...}` and skip the budgets. A silent fallback to 1.5 is the failure this change removes.
- Each budget then uses `mp = dataclasses.replace(inp.mission, cruise_amsl_m=alt, a_fwd=a_used)`.
- `E_summary` gets four columns per row: `a_fwd_used_ms2`, `a_fwd_case_ms2`, `a_fwd_worst_ms2`, `a_fwd_mode`. `E_segments` is unchanged (its `source` text for the forward transition becomes `"quasi-steady estimate, a_fwd from Q2 (<mode>)"`, or the old text in `mission` mode).
- The case altitude is the clearance altitude (ground + `clearance_agl_m`), the same one `forward_transition_estimate` uses for density, so Q2's case and E's segment see the same air. It does **not** depend on the cruise altitude, so compute `a_case` once.

**Scope note.** "Compute once" holds only for E as it is today, where the airfield stays at `mission.yaml`'s `ground_amsl_m` while the cruise altitude is swept. The planned E redesign sweeps the airfield elevation and flies two legs at different masses and SoCs, so it will call `pusher.forward_accel` once per site and per leg (same function, different `rho`, `vp.m`, `soc`). Do not build a cache or a signature in this PR that assumes a single site, mass or SoC: pass them as arguments, as `forward_accel` already does.
- E's `v_trans` stays a TAS target inside `forward_transition_estimate`. With `feed_convention = "eas"` the case speed is `v_trans * sqrt(RHO0 / rho_c)`, which at 30 m ISA is 0.2 percent above `v_trans` (17.03 against 17.00), so the two E models agree to rounding. At the case altitude the convention hardly matters; at altitude it matters a lot (section 10).

## 7. Config and inputs

`settings.py`:

```python
class Q2Cfg(_Strict):
    altitudes_m: list[float] = [0.0, 1000.0, 2500.0, 5000.0]
    mass_factors: list[float] = [0.85, 1.0, 1.15]
    soc: list[float] = [0.8, 0.6, 0.3]
    conventions: list[Literal["tas", "eas"]] = ["tas", "eas"]
    feed_convention: Literal["tas", "eas"] = "tas"
    isa_offset_k: float = 0.0
    n_speed: int = Field(41, ge=11)
    write_profile: bool = False

class ECfg(_Strict):
    cruise_amsl_m: ...                      # unchanged
    ranges_km: ...                          # unchanged
    a_fwd_mode: Literal["case", "worst", "mission"] = "case"
    a_fwd_metric: Literal["min", "equiv"] = "min"
```

`feed_convention` must be one of `conventions` (validator). Build the scenario as `Scenario(alt, isa_offset_k=c.isa_offset_k)` (it has that argument), so the density and `tas()` follow the same offset.

`configs/study.yaml`: the current `Q2:` block is `altitudes_m: [0, 1500, 2500, 5000]`, `mass_factors: [0.80, 1.0, 1.15]`. Keep both lists as they are (they override the defaults above) and add:

```yaml
Q2:
  altitudes_m: [0, 1500, 2500, 5000]
  mass_factors: [0.80, 1.0, 1.15]
  soc: [0.8, 0.6, 0.3]
  conventions: [tas, eas]
  feed_convention: tas
E:
  # ... existing keys ...
  a_fwd_mode: case          # case | worst | mission
  a_fwd_metric: min         # min | equiv
```

`inputs.py`: `StudyInputs` gets `tkoff_thr_max: float` and `airspeed_min: float`, both read where `start_params` is built (they are already there as `start["tkoff_thr_max"]` and `start["airspeed_min"]`; copy them to the new fields so Q2 and E do not reach into `start_params`). `TKOFF_THR_MAX` has the register default `0.8` and status TBD in `controller.yaml`; the built-in fallback in `BaselineParams` is 0.7: if the register entry is absent the fallback is used, and `stage_q2` must add that to `notes`/`fallbacks` so the table is marked as running on a placeholder. Record `tkoff_thr_max` and its status in every Q2 table row (column above).

## 8. Expected results (check against these)

Repo vehicle (nominal mass 26.66 kg), `TKOFF_THR_MAX = 0.8` (effective 0.789), `AIRSPEED_MIN = 17`, 41-speed grid. Cells are `a_min / a_equiv` in m/s^2. The study grid is altitude (rows) x mass factor, columns SoC 0.8 / 0.6 / 0.3.

**TAS convention (V_end = 17 m/s at every altitude)**

| altitude, mass | SoC 0.8 | SoC 0.6 | SoC 0.3 |
| --- | --- | --- | --- |
| 0 m, 0.80 (21.3 kg) | 2.02 / 2.76 | 1.72 / 2.43 | 1.28 / 1.96 |
| 0 m, 1.00 | 1.48 / 2.12 | 1.24 / 1.86 | 0.89 / 1.47 |
| 0 m, 1.15 (30.7 kg) | 1.19 / 1.81 | 0.97 / 1.58 | 0.67 / 1.23 |
| 1500 m, 0.80 | 1.79 / 2.47 | 1.51 / 2.17 | 1.11 / 1.74 |
| 1500 m, 1.00 | 1.27 / 1.91 | 1.05 / 1.67 | 0.73 / 1.31 |
| 1500 m, 1.15 | 1.03 / 1.65 | 0.84 / 1.44 | 0.56 / 1.12 |
| 2500 m, 0.80 | 1.62 / 2.28 | 1.36 / 2.00 | 0.98 / 1.59 |
| 2500 m, 1.00 | 1.13 / 1.79 | 0.92 / 1.56 | 0.63 / 1.22 |
| 2500 m, 1.15 | 0.98 / 1.55 | 0.80 / 1.35 | 0.54 / 1.06 |
| 5000 m, 0.80 | 1.23 / 1.88 | 1.01 / 1.64 | 0.70 / 1.30 |
| 5000 m, 1.00 | 0.97 / 1.49 | 0.80 / 1.30 | 0.55 / 1.03 |
| 5000 m, 1.15 | 0.84 / 1.29 | 0.69 / 1.13 | **0.48 / 0.89** |

**Global worst (TAS): 0.476 m/s^2 at 5000 m, mass factor 1.15, SoC 0.3** (distance-equivalent 0.886, 16.1 s, 163 m to 17 m/s). The pusher never hits its current limit and the lift rotors never saturate anywhere on the grid.

**EAS convention** (V_end = 17, 18.29, 19.24, 21.93 TAS at 0, 1500, 2500, 5000 m). The 0 m rows are identical to the table above. Other rows, `a_min / a_equiv`:

| altitude, mass | SoC 0.8 | SoC 0.6 | SoC 0.3 |
| --- | --- | --- | --- |
| 1500 m, 1.00 | 1.15 / 1.77 | 0.93 / 1.53 | 0.61 / 1.16 |
| 2500 m, 1.00 | 0.94 / 1.54 | 0.73 / 1.30 | 0.44 / 0.95 |
| 5000 m, 0.80 | 0.72 / 1.35 | 0.51 / 1.09 | 0.21 / 0.68 |
| 5000 m, 1.00 | 0.44 / 0.96 | 0.27 / 0.73 | 0.03 / 0.27 |
| 5000 m, 1.15 | 0.28 / 0.76 | 0.13 / 0.52 | **-0.08 / NaN** |

At 5000 m, mass factor 1.15, SoC 0.3 the pusher at 0.8 **cannot reach** 21.93 m/s TAS: acceleration reaches zero at about 20.3 m/s (`v_reach`). So `Q2_worst` for the EAS convention has a negative `a_min` and `reachable = False`. That is a real finding about the hardware (and about reading 17 as EAS at altitude), not an error; section 6 makes `E` refuse to use it.

**E's own case** (30 m AMSL, ISA, SoC 0.6, nominal mass, TAS 17): `a_min` **1.237** (at 17 m/s), `a_equiv` 1.858, 7.9 s and 78 m along the profile. Profile: 3.08 at 0 m/s, 2.88 at 4, 2.51 at 8, 1.97 at 12, 1.34 at 16, 1.24 at 17 m/s; `V_lw` = 15.1 m/s. Command sensitivity of `a_min` at this case: 2.68 (cmd 1.0), 1.94 (0.9), 1.24 (0.8), 0.59 (0.7).

**What E does with it** (forward-transition segment and 1500 m budget; the max range is the E budget's `max_range`):

| `a_fwd` used | source | segment energy | time | distance | max range |
| --- | --- | --- | --- | --- | --- |
| 1.500 | `mission` (today) | 11.2 Wh | 11.3 s | 96 m | 91.6 km |
| 1.237 | `case`, metric `min` (new default) | 12.5 Wh | 13.7 s | 117 m | 91.6 km |
| 1.858 | `case`, metric `equiv` | 10.0 Wh | 9.1 s | 78 m | 91.7 km |
| 0.476 | `worst` (TAS) | 26.5 Wh | 35.7 s | 304 m | 90.9 km |

The transition is a small part of the budget: the choice moves the forward-transition energy by a factor of 2.4 across the range but the maximum range by under 1 km. The distance it needs (96 to 304 m) is the more important change for the mission. (`worst` is pessimistic on purpose: a constant 0.476 over the whole speed range overstates time and distance, because the real acceleration is higher at low speed; for that worst cell the true profile takes 16.1 s and 163 m.)

## 9. README (`analysis/README.md`)

- E section: replace the line "Forward transition | a quasi-steady estimate (constant acceleration `a_fwd`...)" with the same estimate plus: "`a_fwd` comes from Q2 (`E.a_fwd_mode`: `case` is the pusher at `TKOFF_THR_MAX` at E's own site, `worst` the minimum over the Q2 grid, `mission` the value in `mission.yaml`)".
- Add a Q2 section: question, the model in section 3 (one paragraph and the assumptions list: rotors carry `W - L`, E's lift share, loaded bus, `TKOFF_THR_MAX` through the ESC dead zone), the tables, and the DEC-1 convention note.
- State in the Q2 section that the battery is a placeholder (linear open-circuit voltage, assumed R0/R1/C1, no pack current limit); the pusher and rotor `I_lim` are the only current limits.

## 10. Caveats to carry into the PR description

- **Battery model.** Linear OCV (`voc = cells_series * (3.3 + 0.9 SoC)`), assumed R0/R1/C1, minimum voltage 33 V TBD (O-15), no pack current limit. The SoC trend (1.48 at SoC 0.8 against 0.89 at SoC 0.3 at sea level, nominal mass) is the model's, not a measurement.
- **`TKOFF_THR_MAX` is a TBD register value** (ArduPilot default 0.8). The result is very sensitive to it (2.68 / 1.94 / 1.24 / 0.59 for 1.0 / 0.9 / 0.8 / 0.7). If the real value differs, the table must be re-run; consider adding it to the sensitivity inputs (register item Q2-6 in the consolidated issues document).
- **Lift model is E's, not the wing polar's.** At low speed the real wing may carry more (or less) than `(V / V_lw)^2` of the weight; the rotors would carry the difference. This changes the rotor load on the bus slightly and the wing drag slightly. Q2 uses E's rule so that Q2 and E agree.
- **Quasi-steady.** No rotor or pusher spool dynamics, no pitch transient, level flight (gamma = 0), no wind. This is the available acceleration, not a predicted trajectory.
- **TAS and EAS (DEC-1).** Default feed is TAS, as the register says. Under EAS the 5000 m rows need 21.9 m/s TAS; one grid cell cannot reach it. Whatever DEC-1 decides, Q2 reports both until it is settled.
- **E's `worst` mode** applies a constant minimum acceleration over the whole speed range, so it overstates transition time and distance (35.7 s and 304 m, against 16.1 s and 163 m along the true profile of that worst cell). Use the `equiv` metric if a constant-acceleration stand-in that matches the distance is wanted.
- **Not covered.** Whether the rotors can supply the lift the wing does not (they never saturate on this grid, and `rotor_saturated` flags it if they do), the propeller advance-ratio clip inside `rotor.pusher` at high speed, and how the `fwd_wait` state of the baseline controller holds `TKOFF_THR_MAX` (see `control/README.md`).

## 11. Acceptance checks

1. With `E.a_fwd_mode: mission`, every E table is bit-for-bit what it is today (the helper refactors in section 4 change nothing).
2. `energy.rotor_share` equals the old inline expression: `rotor_share(vp, rho, 0) = 1.0`, `rotor_share(vp, rho, V_lw) = 0.0`, and `rotor_share(vp, rho, 8.0)` at 30 m ISA, nominal mass equals `1 - (8.0 / 15.1)**2 = 0.719` (tolerance 0.002).
3. `forward_accel(vp0, rho(30 m), 17.0, 0.6, 0.8)`: `a_min = 1.237 +/- 0.01`, `a_equiv = 1.858 +/- 0.01`, `t_s = 7.87 +/- 0.1`, `x_m = 77.8 +/- 1`, `reachable`, not current-limited, not saturated.
4. `forward_accel` at cmd 1.0 / 0.9 / 0.7 for the same case: `a_min` = 2.68 / 1.94 / 0.59, each within 0.02.
5. TAS grid: all 36 `a_min` values within 0.02 of the table in section 8 (a_equiv within 0.03). Global worst = 0.476 +/- 0.01 at (5000 m, 1.15, SoC 0.3).
6. EAS grid: the 0 m rows equal the TAS rows exactly; the 5000 m / 1.15 / 0.3 cell has `reachable = False`, `a_min = -0.08 +/- 0.02`, `v_reach` between 20.0 and 20.6, and `a_equiv`, `t_s`, `x_m` NaN.
7. Monotonicity (a property, not a number): `a_min` falls with altitude, with mass and with falling SoC in every row and column of both grids (the tables above satisfy this).
8. The default `E.a_fwd_mode: case` run gives `E_summary.a_fwd_used_ms2 = 1.237 +/- 0.01`, `a_fwd_case_ms2` the same, `a_fwd_worst_ms2 = 0.476 +/- 0.01`, `a_fwd_mode = "case"`; the forward-transition segment shows about 12.5 Wh, 13.7 s, 117 m, and the 1500 m max range is within 0.2 km of 91.6.
9. `a_fwd_mode: worst` gives 0.476 and about 26.5 Wh / 35.7 s / 304 m; `a_fwd_metric: equiv` with `case` gives 1.858.
10. With `feed_convention: eas` and `a_fwd_mode: worst` the stage records an error (unreachable cell), writes the `E_summary` error row, and writes no budget; it must not fall back to 1.5.
11. Q2 runs before E in `task_basics`; with `analyses: [E]` only, E still obtains its Q2 inputs (section 6).
12. Removing `TKOFF_THR_MAX` from the register makes Q2 run at the 0.7 fallback and mark the tables (`fallbacks` / `notes`); it does not raise.
13. Run time of `stage_q2` with both conventions and the default grid: about 25 s (72 calls of about 0.3 s). Do not parallelise unless it is over 60 s.
14. The old `Q2_pusher_margin` columns (`T_max_N`, `omega_p_rad_s`, `I_pusher_A`, `CL_required`, `CL_max`, `D_N`, `margin_N`, `accel_ms2`) are gone from the table; anything in the repo or the report generator that reads them must be updated (grep for `margin_N` and `accel_ms2`).

## 12. How the scratch numbers were made (not for the repo)

`load_inputs` on a temporary copy of the config folder (`study.yaml` without the two A keys that the older `settings.py` forbids), draft mode, the real `vehicle.yaml`; a prototype of section 3 run over the grids with `np.linspace(0, v_end, 41)`; E's segment energy and the 1500 m `max_range` from `energy.forward_transition_estimate` and `energy.build_budget` with `a_fwd` replaced. The bus fixed point converges well inside 8 passes at every grid point; the pusher root is bracketed on [1, 4000] rad/s.
