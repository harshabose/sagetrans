# Analysis A fix specification (`zero_loss.py`): hand-off for an implementing agent

Repo: `sagetrans` (Python, `src/sagetrans/...`). This file is self-contained. Nothing here has been implemented yet; every number in "Expected results" comes from scratch scripts and must be re-baselined against the real configs (section 9).

## 0. Decisions already taken by the project owner (do not re-open)

1. A is a **purely quasi-steady, idealised feasibility scan**: constant deceleration `a` from a free entry speed `V0` down to `v_end` (1 m/s), gamma = 0, no wind, **pusher off** (`t_p = 0`; keep the `t_p` argument in the function signatures but do not add a config axis for it).
2. The rotors are always treated as **pre-spun** (rotor speed at the first sample is the first schedule value). **Delete the "whole segment / rotors from rest" verdict and everything that exists only to support it.**
3. **No jerk limit.**
4. The wing angle of attack is limited to a band **[alpha_min, alpha_hi]**: `alpha_min = 0` (lower end of the range the lift fit was made on, 0 to 10 deg) and `alpha_hi` = the angle where C_L = **0.8 x C_L,max** on the rising side (about 9.1 deg for the current wing; the proposal's rule 8.6). C_L,max is provisional (`vehicle.yaml` marks `alpha_s` PROVISIONAL), so make the fraction configurable.
5. **Rotor lead-time output and any spool/throttle-slew model are deferred.** Do not implement them.
6. Out of scope: B1/B2/C/D/E/F logic, ArduPilot AIRBRAKE modelling, tests (the owner will add tests separately), the proposal text.

## 1. Why (short)

- Today A reports many "feasible" constant-a profiles that use **negative angle of attack** (theta down to -5 deg at a = 0.2 at 22.5 m/s) where the lift model is an extrapolation, plus a **false feasible island** at 16 m/s TAS, a = 1.5 (theta 15.3 deg, past the lift peak at 13.4 deg, T_r = 3.7 N, passes by 0.4 deg).
- With theta >= 0 imposed, **no constant-a profile is feasible at 22.5 m/s** (pusher off): the wing's drag floor at alpha = 0 (about 29 to 31 N, about 1.1 m/s^2) sits above what the wing can deliver without needing rotor thrust below zero.
- Per speed there is a **window** [a_lo(V), a_hi(V)]: lower edge = drag floor at alpha = 0; upper edge = the low-theta root pair of the force balance disappearing (a fold), after which only the stalled root near 15 deg exists and `thrust_sign` fails.
- The "entry jump" checks measure a pitch/rotor step from a reference state (`a = 0`, pusher off: theta = -5.6 deg, L = -29 N) that is not a state the aircraft is ever in. Dropping them is the owner's decision.
- `first_binding` names only the most negative normalised margin at the first bad sample; margins are normalised differently, so that compares unlike things and hides ties.

## 2. Files to change

| file | change |
| --- | --- |
| `src/sagetrans/analysis/zero_loss.py` | main work, section 3 |
| `src/sagetrans/io/settings.py` (`ACfg`) | two new fields, section 4 |
| `configs/study.yaml` (`A:` block) | set them, section 4 |
| `src/sagetrans/io/stages.py` (`stage_a`) | tables and limits wiring, section 5 |
| `src/sagetrans/analysis/freeform_opt.py` (line ~56) | one-line guard so B2's start does not change, section 6 |
| `src/sagetrans/analysis/README.md` (A section) | rewrite, section 8 |

No other module imports `zero_loss` apart from `stages.py` and `freeform_opt.py` (checked with grep). `freeform_opt.init_from_zero_loss` uses only `r.V`, `r.theta`, `r.delta_cmd` of `analyse(...)`: keep those three field names.

## 3. `zero_loss.py`

### 3.1 Delete

- `EntryState`, `DEFAULT_ENTRY`, the `entry` parameter of `analyse`. Replace by a keyword `v1_0: float = 0.0` (initial battery RC state; today `entry.v1`, default 0).
- The `th_entry` block, `dt0`, the `whole = build(...)` call, and `ok_w, fb_w`.
- `DecelProfile.jerk` and the jerk branches in `sample`; the `jerk` argument of `max_feasible_decel`.
- The `stall` margin (replaced by 3.3). Keep it only inside the legacy path of 3.3.
- In `ZeroLossResult`: remove `margins_whole`, `feasible_whole`, `first_binding_whole`. Rename `margins_pre_spun -> margins`, `feasible_pre_spun -> feasible`, `first_binding_pre_spun -> violations` (type in 3.4). Keep `t, V, theta, T_r, omega_r, delta_cmd, I_m, V_bus, record`.
- In `analyse`, the module docstring and class comments that describe the entry discontinuity (rewrite to the quasi-steady description in section 1).

### 3.2 `ActuatorLimits`: add two fields

```python
alpha_min: float | None = 0.0   # rad; None = no lower band (legacy)
cl_frac: float | None = 0.8     # alpha_hi where C_L = cl_frac * C_L,max (rising side); 1.0 = lift peak; None = legacy symmetric alpha_s check
```

Keep `delta_r_max`, `theta_max` (0.6 rad; it will never bind because theta = alpha <= 9.1 deg at gamma = 0, leave it), `vrs_limit`.

Add private `_alpha_band(vp, limits) -> tuple[float, float]` returning the **theta** band (alpha minus `vp.wing.i_w`):

- `alpha_lo = limits.alpha_min` if not None else `THETA_LO + i_w`.
- `alpha_hi`: if `cl_frac is None`, use `THETA_HI + i_w`; else find alpha on `[0, alpha_peak]` with `C_L(alpha) = cl_frac * aero.cl_max(wing)` by `brentq` (use `aero.coefficients(alpha, wing)`; `alpha_peak` = argmax of C_L on a fine grid, as `aero.cl_max` does; `cl_frac = 1.0` returns alpha_peak). Do not edit `physics/aero.py`.
- Return `(alpha_lo - i_w, alpha_hi - i_w)`.

### 3.3 Root search and `solve_point`

Problem to fix: the force balance `R(theta) = 0` has several roots (example, V = 17 m/s TAS, a = 1.04: roots at 4.2, 8.1 and 14.9 deg; a = 1.08: 5.7, 6.6 and 15.0 deg; a = 1.10: only 15.0 deg). Today the code takes the root nearest the previous theta among those with `T_r >= 0`, so it can jump to the 15 deg branch.

1. `_Balance`: make the scan resolution configurable. Keep `self.scan = resid.map(61)` for the coarse out-of-band diagnostic grid; add a cached `map(N)` helper and a **dense in-band grid of about 121 points** over the band `[theta_lo, theta_hi]` (spacing about 0.08 deg for a 9 deg band). Reason: near the fold the two low roots are less than 1 deg apart, and the coarse 1.6 deg cell can miss the pair and report the upper window edge too low.
2. `_roots(...)`: accept an explicit grid (band bounds and N).
3. `solve_point(..., band: tuple[float, float] | None = None)`. Inside the existing 30-iteration `h_r` loop, build candidate roots from the dense in-band scan plus the coarse full-range scan, then choose with this priority (within a group, nearest `prev_theta`, default reference 0):
   1. in-band roots with `T_r >= 0`;
   2. in-band roots with `T_r < 0` (these fail `thrust_sign`);
   3. out-of-band roots, nearest to the band (these fail `alpha_low` or `alpha_high`).
   With `band=None` or `band = (THETA_LO, THETA_HI)` this must reproduce the **current** selection exactly (groups 1 and 2 = today's `pos` / `all`). That identity is the regression test in section 9.
4. Everything else in `solve_point` (the `h_r` fixed-point, `rotor_speed_for_thrust`, `Point`) is unchanged.

### 3.4 Margins and violations

1. Lift the inner `build()` out of `analyse` into a module-level `_margins(vp, scen, limits, band_alpha, t, V, th, t_r, om, solved, om_dot, th_dot, v1_0)` so the path check and the window share it. Inside it:
   - keep `pitch_solution`, `thrust_sign`, `throttle`, `current`, `voltage`, `torque_floor`, `pitch`, `pitch_rate` exactly as today;
   - add `alpha_low = (alpha - alpha_lo)/(alpha_hi - alpha_lo)` and `alpha_high = (alpha_hi - alpha)/(alpha_hi - alpha_lo)` with `alpha = theta + i_w`;
   - if `limits.alpha_min is None` omit `alpha_low`; if `limits.cl_frac is None` use the old `stall` margin `(alpha_s - abs(alpha))/alpha_s` instead of `alpha_high` (legacy path).
2. In `analyse`, compute `om_dot` and `th_dot` as `np.gradient(om, t)` and `np.gradient(th, t)` directly on the profile samples (today the code concatenates a "previous" sample, which halves the first rate). Unsolved samples stay NaN -> margin -1, as today.
3. Replace `_first_binding` by `_violations(margins, t) -> dict[str, float]`: for each margin that goes below `-1e-9` anywhere, the first time it does, sorted by time (ties preserved). `ZeroLossResult.violations` holds this dict; `feasible = (len(violations) == 0)`. Provide a helper `first_binding(result) -> (name, time) | None` returning the earliest entry (for any caller wanting the old shape).
4. `make_record(vp, scen, profile, limits, v1_0, t_p, v0)` (no `entry`).

### 3.5 New `feasible_window`

```python
def feasible_window(vp, scen, V, limits=DEFAULT_LIMITS, t_p=0.0,
                    a_max=3.0, da=0.02, bisect=9) -> list[Window]
```

- For each `a` on `arange(0, a_max, da)`: `solve_point(..., band=_alpha_band(...))`, then `_margins(...)` on length-1 arrays with **om_dot = 0, th_dot = 0, v1_0 = 0** (steady rotor; state this in the docstring). Feasible if all margins >= 0.
- Find contiguous feasible runs; refine each edge by bisection against its infeasible neighbour.
- `Window` (dataclass): `a_lo, a_hi, theta_lo, theta_hi, T_r_lo, T_r_hi`, `bind_lo`, `bind_hi` = the **list of all violated margin names** just outside each edge (not only the worst). If no feasible `a`, return an empty list; the caller records the violated names at the closest `a` instead.

### 3.6 `feasible_decel_range` and `max_feasible_decel`

- New `feasible_decel_range(vp, scen, v0, limits, n=60, a_lo=0.05, a_hi=4.0, scan_points=24, iterations=9) -> tuple[float, float, bool] | None` returning `(a_min, a_max, islands)`. Method: scan `a` with `analyse` (authoritative, includes omega-dot, battery and rate margins), take the first and last feasible grid points, bisect each edge against its infeasible neighbour; `islands = True` if the feasible grid points are not contiguous (then report the full span and set the flag; do not hide it).
- `max_feasible_decel(...)` stays as a thin wrapper returning `a_max` (0.0 if none). Rewrite its docstring: the old "feasibility is not monotone / islands" paragraph is obsolete.
- `feasibility_map(...)`: returns `{"feasible": bool[nV, na], "violations": [[dict]]}`; drop `"whole"`.

## 4. Config

`io/settings.py` `ACfg` add:

```python
alpha_min_deg: float | None = 0.0
cl_frac: float | None = 0.8
```

`configs/study.yaml` `A:` block add `alpha_min_deg: 0.0` and `cl_frac: 0.8` (with a comment: lift fit range lower end; proposal rule 8.6; C_L,max provisional). Keep `v0_eas_ms`, `decel_ms2`, `max_decel_v0_eas_ms`, `samples` as they are: `v0_eas_ms` is the free entry-speed axis.

## 5. `stage_a` in `io/stages.py`

- Build `lim = ActuatorLimits(alpha_min=math.radians(c.alpha_min_deg) if c.alpha_min_deg is not None else None, cl_frac=c.cl_frac)` and pass it to `feasibility_map`, `feasible_window`, `feasible_decel_range` (today `stage_a` passes no limits).
- `A_feasibility` rows: drop `feasible_whole`; rename `feasible_pre_spun -> feasible`; replace `first_binding_pre_spun` / `first_binding_time_s` by one `violated` string such as `thrust_sign@1.52;alpha_high@0.00` (all violated names, sorted by time).
- New table `A_window`: `scenario, altitude_m, V0_eas_ms, V0_tas_ms, a_lo, a_hi, bind_lo, bind_hi, theta_lo_deg, theta_hi_deg` (one row per window; a row with empty `a_lo`/`a_hi` and the violating names when none exists), evaluated at each `v0` in `c.v0_eas_ms`, pusher off.
- `A_max_feasible_decel`: columns `min_feasible_decel_ms2`, `max_feasible_decel_ms2`, `islands` (rename from `max_feasible_decel_ms2_pre_spun`; no other consumer of the old name was found in `src`).
- `out["maps"]`: keep the structure, using the new keys.

## 6. `freeform_opt.py`

`init_from_zero_loss` calls `analyse(vps, scen, v0_air - scen.headwind_ms, DecelProfile(a_init), n=60)`. Change to pass `limits=ActuatorLimits(alpha_min=None, cl_frac=None)` (import it) so B2's initial schedule is bit-for-bit what it is today. Do not change B's behaviour as part of this task (B1 has its own negative-alpha issue, tracked separately).

## 7. Performance note

The dense in-band grid adds cost to every `solve_point`. Budget check: `stage_a` runs 4 scenarios x 6 V0 x 15 a x 120 samples. If the run time grows by more than about 3x, do the dense scan only when the coarse scan shows no root in band or an odd number of sign changes near the band, and report what was done.

## 8. README (`analysis/README.md`, A section)

Rewrite: remove the "Entry discontinuity" paragraph and the two-verdicts text; describe the quasi-steady scan, the angle band, the window output and the violation list; update the API table (no `EntryState`, no `jerk`, new `feasible_window`, `feasible_decel_range`); keep the physics (force balance, rotor map inversion) unchanged. Note that the argument for treating the rotor start speed as a decision variable now lives in B1 (`prespin_max`) and D.

## 9. Acceptance checks

The reference numbers below come from my scratch scripts, which used a **hand-copied** vehicle (m = 26.664 kg, S = 1.617, CL0 0.4, CL_alpha 4.75, alpha_s 0.274, CD0 0.066, k 0.042, i_w 0, `Scenario(altitude_amsl_m=1500, soc=0.8)`, W = 261.5 N, rho = 1.058). **Re-baseline with `load_inputs("configs", "draft", ...)` first**; expect agreement within about 0.03 m/s^2 on window edges (about 0.05 near the fold, since the dense grid moves it slightly), pusher off.

1. **Regression (exact):** with `ActuatorLimits(alpha_min=None, cl_frac=None)`, the new `analyse` reproduces the old pre-spun `theta, T_r, omega_r, delta_cmd` and the same feasible/infeasible set on the existing grid.
2. **Window at 22.5 m/s TAS:** about 1.20 to 1.32 m/s^2 (theta 0.2 to 2.4 deg).
3. **Window by speed (pointwise, theta in [0, 9.1 deg], T_r >= 0):**

   | V (m/s) | a_lo to a_hi |
   | --- | --- |
   | 22.5 | 1.20 to 1.32 |
   | 20 | 0.94 to 1.14 |
   | 18 | 0.76 to 1.08 |
   | 16 | 0.60 to 1.08 |
   | 14 | 0.46 to 1.16 |
   | 12 | 0.34 to 1.28 |
   | 10 | 0.24 to 1.36 |
   | 6 | 0.10 to 1.48 |

4. **Constant-a feasible range from V0 down to 1 m/s:** V0 = 14: 0.46 to 1.16; V0 = 16: 0.60 to 1.08; V0 = 17: 0.68 to 1.08; V0 = 20: 0.94 to 1.08; **V0 = 22.5: none**. (Upper edge is set by the 16 to 18 m/s cross-section.)
5. **Fold at 17 m/s:** a = 1.04 feasible, a = 1.12 infeasible. Expected binding at the upper edge: `thrust_sign` (via the 15 deg root) or `alpha_high`; report whichever appears, and check it is stable under a finer `da`.
6. **False island gone:** V0 = 16 m/s TAS, a = 1.5 is infeasible (was feasible at theta 15.3 deg).
7. **Negative alpha gone:** no feasible result has `min(theta) < 0`.
8. **Sensitivity:** `cl_frac = 1.0` (lift peak, 13.4 deg) changes the window only below about 14 m/s (e.g. a_hi about 1.48 vs 1.28 at 12 m/s); `cl_frac = 0.8` is the default.
9. **Violations:** for a = 1.4 at 22.5 m/s the `violated` string lists every margin that fails, with first times; the old single name is the first entry.
10. **B2 unchanged:** `init_from_zero_loss` output identical before and after (guard in section 6).

## 10. Caveats to carry into the PR description

- With pusher off the headline becomes about 1.1 m/s^2 over most entry speeds, set by the 16 to 18 m/s speed range where the wing carries almost all the weight. It is a property of the idealised pusher-off, rotors-never-push-down case, not a hardware limit.
- `alpha_hi` rests on a provisional C_L,max (about 1.44 here; the register says the CFD has no peak up to 15 deg). `V_0.8 proportional to 1/sqrt(C_L,max)`.
- The window uses steady rotors (omega-dot = 0, v1 = 0); the constant-a verdict from `analyse` includes omega-dot, battery state and pitch rate and is the authoritative one.
- `I_rot` (1.5e-3 kg m^2) and `q_max` are TBD in `vehicle.yaml`; throttle/current/torque margins depend on them.
- This changes the meaning of every A table and figure; downstream reading notes will need updating separately.

## 11. Reference script for the pointwise preview (scratch, not for the repo)

```python
import math, numpy as np
bal = zl._Balance(vp)                       # vp, scen as in section 9
def feas(V, a, hi_deg=9.1):
    p = zl.solve_point(vp, scen.rho, V, a, 0.0, None, bal)
    if math.isnan(p.theta): return False
    th = math.degrees(p.theta)
    return p.T_r >= 0 and 0.0 <= th <= hi_deg
agrid = np.arange(0.0, 4.01, 0.02)
for V in (6, 10, 14, 18, 22.5):
    ok = [a for a in agrid if feas(V, a)]
    print(V, (ok[0], ok[-1]) if ok else None)
```

(This uses the old coarse 61-point root grid, so near the upper edge it may differ slightly from the new dense-grid result.)
