# Analysis E fix specification (`energy.py`, `stage_e`): hand-off for an implementing agent

Repo: `sagetrans` (Python, `src/sagetrans/...`). This file is self-contained. **Status: implemented** (branch `feat/e-mission-budget`; section 10 checks 1 to 13 were run, tests are left to the owner) (written 26 Oct 2026). **Order:** implement `Q2_fix_spec.md` first (it adds `analysis/pusher.py`, `StudyInputs.tkoff_thr_max` and `airspeed_min`, the `E.a_fwd_mode` and `E.a_fwd_metric` keys and the Q2-before-E ordering); this spec is written against that code and replaces its temporary `stage_e` hook. The expected numbers below come from a scratch prototype of this design run on the repo's real configs in draft mode; section 13 says how. They are values for the acceptance checks, not repo changes.

## 0. Decisions already taken by the project owner (do not re-open)

1. **Two missions are run for every site**, and the range is an **output** (whatever energy is left after everything else goes to cruise):
   - `round_trip`: loaded outbound leg, payload drop, empty return leg. Each leg has its own take-off, forward transition, climb, cruise, glide, back transition and landing, so every fixed segment appears twice. The reserve is counted once.
   - `one_way`: a single loaded leg, as E does today.
   The 100 and 113/120 km requirements are **comparison columns only**; they never enter the solve.
2. **Geometry.** A fixed climb height (`climb_height_m`, 200, config). Take-off and landing are at the same elevation (the owner's assumption). The swept quantity is the **site (airfield) elevation**; cruise altitude is site + climb height.
3. **Cruise design speed.** `v_cruise` = 22.5 m/s is the design TAS at the 1500 m reference altitude, which is 20.9 EAS. At any other altitude the TAS follows from the density (constant dynamic pressure). Climb and glide use the same speed by default (optional overrides).
4. **The throttle check is report-only against `TKOFF_THR_MAX`** (0.8, still the ceiling for this analysis). A segment **fails** only when its operating point is infeasible: pusher or rotor command above 1, a motor current limit, bus voltage below `V_min`, or a pack that cannot deliver the power. Failures are shown as failures; the energy is still computed and tabulated.
5. **SoC.** Segment energies keep the mean SoC (`soc_mean`). A SoC trace is built from the cumulative energy, starting at `soc_start` (default 1.0, a full pack; configurable). Every segment is checked at **both** its start and end SoC and the worse result is reported. The trace feeds the forward-transition acceleration and the hover check as well.
6. **Forward-transition acceleration** comes from the Q2 model, per site, per leg mass and per leg SoC (`E.a_fwd_mode`: `case` default, `worst`, `mission`; metric `min`/`equiv`; all as in `Q2_fix_spec.md`).
7. **Reserve** is counted once, at the end, as the larger of the two landings' reserve. `n_diversion_transitions` stays 0 (the owner takes the 5 km diversion as already covering transitions); do not change the reserve model.
8. **Out of scope:** analyses A, B, C, D, F and Q2 logic; the OCV curve and any battery current limit (O-15); the trim-bracket defect (A section 7.11); wind, route factor, temperature derating; payload-drop procedure beyond one optional hover time; tests (the owner adds them).

9. **Usable-energy window.** The usable energy is a fraction of the nominal capacity taken from a **full pack** (`eta_usable` x `soh` x `temp_factor` x nominal, unchanged): `soc_start` is 1.0 and the trace ends at `1 - eta_usable` = 0.2 once the usable energy is spent. `soc_eod` (0.1) stays as a stress point for the hover check; `window_ok` reports that the trace stays above it.

## 1. Why (short)

- E-2: `stage_e` sweeps the cruise altitude with the airfield fixed at 0 m, so the 5000 m case climbs 5 km (1351 Wh, 76 percent of the usable energy) and "5000 m" is a mountain-crossing mission nobody asked for.
- The cruise speed is fixed at 22.5 TAS at every altitude, which is an EAS 7.6 percent above the design speed at 0 m and 16.5 percent below it at 5000 m (17.5 against 20.9). Most of the apparent altitude benefit (18.3 to 14.4 Wh/km) comes from that: at constant EAS the energy per kilometre is 16.9 to 16.2 Wh/km.
- E budgets one loaded leg. "Range" in the requirement is not defined, so the code answers only one of its readings.
- E-1: the back-transition segment builds a controller from code defaults (20/25 deg), not the configured one (30/35 deg) that D uses, and it enters at the cruise speed (22.5 m/s) rather than at D's entry EAS (20.9).
- E-4: `a_plan` falls back silently to 2.5 when D is missing.
- `build_budget` ignores the trim's `feasible` flag: at 5000 m a climb that needs a pusher command of 1.03 is reported as 2.8 kW of ordinary power.
- A single mean SoC says nothing about feasibility: the pack's bus voltage falls from 50 V to 41 V as SoC drops, so the same thrust needs a higher command (climb at 2.5 m/s at sea level: 0.80 at SoC 1.0, 0.87 at 0.6, 0.95 at 0.2).

## 2. Files to change

| file | change |
| --- | --- |
| `src/sagetrans/analysis/energy.py` | profile fields, speed conversion, `LegBudget`/`MissionBudget`, segment tweaks, section 4 |
| `src/sagetrans/analysis/soc_trace.py` | **new**: SoC trace and the segment feasibility check, section 5 |
| `src/sagetrans/io/stages.py` (`stage_e`) | rewrite, section 6 |
| `src/sagetrans/io/settings.py` (`ECfg`) | section 3 |
| `src/sagetrans/io/inputs.py` | loader keys for the moved and new inputs, section 3 |
| `configs/mission.yaml`, `configs/study.yaml` | section 3 |
| `src/sagetrans/analysis/README.md` (E section) | section 9 |

Keep `BudgetResult` and its tests (the M4 hand-calculated case) working unchanged; `MissionBudget` composes `BudgetResult` objects. `hover_check` and `cruise_sweep` keep their signatures except where section 7 says.

## 3. Config: what lives where

Rule: **`mission.yaml` holds facts about the mission and the aircraft's operation; `study.yaml` holds what to sweep and what to report. No key lives in both.**

`configs/mission.yaml`, `profile:`

| key | change |
| --- | --- |
| `cruise_amsl_m`, `ground_amsl_m` | **remove** (E derives them per site). In `inputs.py` add `MISSION_FROM_STUDY = {"cruise_amsl_m", "ground_amsl_m"}` and skip them exactly as `MISSION_FROM_CONTROLLER` is skipped; the dataclass keeps the fields with their defaults. Remove the two keys from the yaml and from whatever known-key list the loader builds, otherwise the strict loader rejects them. |
| `climb_height_m` | **new**, 200, m, "climb above the airfield; cruise altitude = site + this", status TBD |
| `v_cruise` | keep 22.5; reword source: "design TAS at `v_cruise_ref_amsl_m`; EAS 20.9" |
| `v_cruise_ref_amsl_m` | **new**, 1500, m, status TBD |
| `v_climb`, `v_glide` | **optional** overrides (design TAS at the reference altitude). Absent means the cruise speed. Delete the current values 20 and 20 from the yaml; they were fixed TAS and fail the glide trim at 5000 m. |
| `payload_kg` | **new**, 5.0, kg, "mass delivered at the destination; the return leg flies at vehicle mass minus this" (O-12; check that 1 - payload/mass equals scenario S-06's 0.8125) |
| `t_drop_s` | **new**, 0.0, s, "hover at the destination for the drop, at the loaded mass", TBD |
| kept | `isa_offset_k`, `clearance_agl_m`, `vz_takeoff`, `t_hover_pre`, `roc`, `h_bt_agl_m`, `soc_mean`, `t_go_around`, `t_hold`, `d_diversion_m`, `n_diversion_transitions`, `a_fwd` (used only by `a_fwd_mode: mission`) |

`mission.yaml`, `battery_window:` add `soc_start`, 1.0, "state of charge at take-off (full pack); the usable energy is eta_usable of nominal, so the mission ends at 1 - eta_usable", TBD. Keep `eta_usable`, `soh`, `temp_factor`, `soc_eod` as they are.

`mission.yaml`, `requirements:` unchanged. (`range_p95_km` says 120 while its source text says about 113: the owner's value; not part of this change.)

`configs/study.yaml`, `E:`

```yaml
E:
  sites_amsl_m: [0, 1500, 2500, 5000]    # airfield elevations; cruise = site + climb_height_m
  missions: [round_trip, one_way]
  ranges_km: [50, 100, 150]              # report points (one-way distance, or radius for round_trip)
  a_fwd_mode: case                       # case | worst | mission   (Q2 spec)
  a_fwd_metric: min                      # min | equiv
  a_plan_fallback_ms2: 2.5               # D spec section 8
```

Remove `cruise_amsl_m` from `E:`. `ECfg` follows (`sites_amsl_m: list[float]`, `missions: list[Literal["round_trip","one_way"]]`). `controller.yaml` is unchanged (`AIRSPEED_MIN` still becomes `mp.v_trans`). `controller.cruise_stub.v_cruise` duplicates `mission.profile.v_cruise`: keep both, and make `load_inputs` raise an `InputError` if they differ.

`MissionProfile` (frozen dataclass) gains `climb_height_m: float = 200.0`, `v_cruise_ref_amsl_m: float = 1500.0`, `payload_kg: float = 5.0`, `t_drop_s: float = 0.0`; `v_climb` and `v_glide` become `float | None = None`. `BatteryWindow` gains `soc_start: float = 1.0`.

## 4. `energy.py`

### 4.1 Speeds and the site

```python
def design_speeds(mp: MissionProfile, altitude_amsl_m: float) -> tuple[float, float, float]:
    """(cruise, climb, glide) TAS at `altitude_amsl_m`: the design TAS scaled to the same dynamic pressure."""
    k = math.sqrt(density(mp.v_cruise_ref_amsl_m, mp.isa_offset_k) / density(altitude_amsl_m, mp.isa_offset_k))
    vc = mp.v_cruise * k          # if mp.v_cruise is None the caller uses best_range_speed
    return vc, (mp.v_climb * k if mp.v_climb is not None else vc), (mp.v_glide * k if mp.v_glide is not None else vc)

def mission_at_site(mp: MissionProfile, site_amsl_m: float) -> MissionProfile:
    cruise = site_amsl_m + mp.climb_height_m
    vc, vcl, vg = design_speeds(mp, cruise)
    return replace(mp, ground_amsl_m=site_amsl_m, cruise_amsl_m=cruise, v_cruise=vc, v_climb=vcl, v_glide=vg)
```

`density` is `atmosphere.density`. When `mp.v_cruise` is `None` (best-range speed) skip the scaling for cruise and let `build_budget` pick the best-range speed at the cruise density; climb and glide then follow it. The existing segment functions then run **unchanged** on the converted profile: they read `mp.ground_amsl_m`, `mp.cruise_amsl_m`, `mp.v_climb`, `mp.v_glide`. The climb is `climb_height_m - clearance_agl_m` = 170 m.

### 4.2 Back transition (E-1)

`back_transition_segment(vp, mp, v0, soc, a_plan, bp)` gets two changes: the controller `bp` is passed in (stage uses `inp.controller`, as `stage_d` does; `run_braking` sets `h_tgt` and `delta_hover` itself, so do not call `for_vehicle` here) and `v0` is the caller's entry speed. `stage_e` passes `v0 = inp.entry_eas * sqrt(RHO0 / rho)` with `rho` the density at `ground + h_bt_agl_m` (the `tas()` helper in `stages.py` does this). The glide hands over to the back transition at that entry speed.

### 4.3 Legs and the mission

```python
@dataclass
class MissionBudget:
    kind: str                                        # "round_trip" | "one_way"
    legs: list[tuple[str, float, BudgetResult]]       # (name, mass_kg, leg budget with reserve_J = 0)
    reserve_J: float                                 # counted once
    reserve_parts: dict[str, dict[str, float]]       # per landing: reserve_energy() output
    drop_J: float                                    # t_drop_s * hover power at the loaded mass, round_trip only
    usable_J: float
    record: RunRecord
    notes: dict[str, float]

    # r_m = the one-way distance every leg covers (climb and descent included); round_trip flies 2 r_m.
    def total_J(self, r_m):  # sum_i [fixed_i + (r_m - d_i) e_i] + drop + reserve ; ValueError if r_m < max d_i
    def margin_J(self, r_m): ...
    @property
    def max_range_m(self):
        # (usable - sum fixed - reserve - drop + sum d_i e_i) / sum e_i ; 0 if that is below max d_i
    def table(self, ranges_m): ...   # as BudgetResult.table; columns prefixed by leg name, e.g. out_takeoff_Wh
    def conflicts(self, required_m): ...
```

`build_mission(vp, mp_site, window, limits, a_plan, kind, a_fwd_by_leg, bp, v0_entry)` returns a `MissionBudget`:

- Leg masses: `vp.m` for the outbound leg (and the `one_way` leg); `vp.m - mp.payload_kg` for the return leg. Build each leg with `dataclasses.replace(vp, m=...)` and `replace(mp_site, a_fwd=a_fwd_by_leg[leg])`, and the existing segment functions at `soc_mean`; the cruise energy per metre is `steady_trim(...)` power over the cruise speed, as `build_budget` computes it now. Name the legs `"out"` and `"ret"` (`"leg"` for `one_way`).
- Reserve: `reserve_energy(...)` for each landing (the loaded outbound landing, the empty home landing; the single landing for `one_way`); `reserve_J` is the larger of the two, and `notes["reserve_binding_leg"]` says which.
- `drop_J = mp.t_drop_s * hover_power(vp_loaded, rho_site, soc_mean)` (zero by default).
- Hover power, the back transition (D simulation at the leg mass) and F's landing profile all use the leg's vehicle.
- Raise nothing for an infeasible trim; return the numbers. The feasibility is judged in section 5.

`build_budget` stays for single-leg callers (grep for its callers before changing it, and keep them working); give it the same `bp` and `v0` keyword-only arguments, default `None` meaning today's behaviour.

## 5. New module `analysis/soc_trace.py`

```python
FLIGHT_ORDER = ["takeoff", "fwd_transition", "climb", "cruise", "descent", "back_transition", "landing"]

@dataclass(frozen=True)
class SegmentCheck:
    leg: str; segment: str
    soc_start: float; soc_end: float; energy_Wh: float
    feasible: bool                 # at both ends
    limit: str                     # "" when feasible; else the first limit hit, see limit_reason()
    pusher_cmd_max: float          # max over the two SoC ends; 0 for segments without the pusher
    rotor_cmd_max: float
    over_ceiling: bool             # pusher_cmd_max > tkoff_thr_max  (REPORT ONLY)
    roc_at_ceiling_ms: float       # climb rows: best rate of climb with pusher command = ceiling at the worse SoC; NaN when level flight already exceeds it
```

- `trace(mb, r_m, soc_start, e_nom_J)`: walk the legs in `FLIGHT_ORDER`; the cruise energy of a leg is `(r_m - d_leg) * e_cruise_leg`; the drop energy sits between the legs; SoC falls by `energy / e_nom_J` with `e_nom_J = vp.battery.e_nominal_wh * 3600` (2220 Wh). Return the rows, the SoC before the reserve and the SoC after it.
- `check_segment(vp_leg, mp_leg, name, soc, ceiling)` at one SoC:
  - `takeoff`, `landing`: `trim.hover_state(vp_leg, rho_site, soc)`; rotor command is `delta_r_cmd`.
  - `climb`: `trim.steady_trim(vp_leg, rho(h_mean), v_climb, asin(roc / v_climb), soc)` with `h_mean` as in `climb_segment`.
  - `cruise`: `trim.steady_trim(vp_leg, rho(cruise), v_cruise, 0, soc)`.
  - `descent`: pusher off; feasible when `glide_trim` exists (as `descent_segment` already requires).
  - `fwd_transition`: `pusher.forward_accel(vp_leg, rho(site + clearance_agl), mp.v_trans, soc, ceiling)`; infeasible when `reachable` is False (`limit` = `"pusher cannot reach AIRSPEED_MIN at TKOFF_THR_MAX (v_reach=...)"`) or `rotor_saturated`. Here the pusher command is the ceiling by design, so `over_ceiling` is False.
  - `back_transition`: feasible when the D simulation completed (`terminated_by == "ground_speed_reached"`); today a failure raises `RuntimeError` in the segment, keep that.
- A segment is feasible at a SoC when `elec.feasible` is True. `limit_reason(vp, es)` names the first limit, in this order: `"pusher command > 1"`, `"rotor command > 1"`, `"pusher current limit"`, `"rotor current limit"`, `"bus voltage < V_min"`, otherwise `"pack cannot deliver this power"`. Command above the **ceiling** but at most 1 is **not** a failure.
- `roc_at_ceiling_ms`: `brentq` on gamma in `[0, asin(5 / v_climb)]` solving `delta_p_cmd(gamma) = ceiling` at the worse of the two SoC ends; NaN if `delta_p_cmd(0) > ceiling`.
- Mission-level: `flyable = all(rows.feasible)`; `n_infeasible`, `first_infeasible` (leg, segment, limit); `n_over_ceiling`.

## 6. `stage_e`

For each site in `E.sites_amsl_m`, each mission in `E.missions`:

1. `mp = mission_at_site(inp.mission, site)`; entry speed `v0`; `a_plan` as in D spec section 8 (D's `a_rec`; if missing or NaN use `E.a_plan_fallback_ms2`, append to `errors`, and record `a_plan_source`).
2. **Two passes for the acceleration.** Pass 1: every leg's SoC is `soc_mean`. Compute `a_fwd` per leg (mode `case`: `pusher.forward_accel(vp_leg, rho(site + clearance_agl), mp.v_trans, soc_leg, inp.tkoff_thr_max).a_min`, or `.a_equiv` for metric `equiv`; mode `worst`: the Q2 global-worst value for `feed_convention` for every leg; mode `mission`: `mp.a_fwd`), build the mission, solve `r_max = max_range_m`, run the trace. Pass 2: set each leg's `soc_leg` to the traced SoC at the start of its forward transition, recompute `a_fwd`, rebuild, solve, trace again. Report pass 2. (The second pass changes `r_max` by about 0.02 km or less; it exists so the reported acceleration belongs to the SoC the leg actually has.)
3. If any `a_fwd` is NaN or not above 0: no fallback. Append the error, write an `E_summary` row `{"site": ..., "mission": ..., "error": ...}` and go on to the next case.
4. Check every segment at `r_max` (section 5). Write the tables below.

Tables (replace the current E tables; keep `E_cruise_sweep`, `E_reserve` for the first case):

- `E_summary`, one row per (site, mission): `site_amsl_m, mission, cruise_amsl_m, v_cruise_tas_ms, v_cruise_eas_ms, max_range_km` (the radius for `round_trip`), `flown_total_km, fixed_Wh, cruise_Wh, reserve_Wh, reserve_binding_leg, drop_Wh, usable_Wh, soc_start, soc_end_before_reserve, soc_end_after_reserve, window_ok` (`soc_end_after_reserve >= window.soc_eod`; 0.20 against 0.1 at the defaults), `flyable, n_infeasible, first_infeasible, n_over_ceiling, a_fwd_mode, a_fwd_out_ms2, a_fwd_ret_ms2, a_plan_ms2, a_plan_source`, and for each requirement R (100, 120): `margin_Wh_at_<R>km` and `conflict_<R>km` (computed with `r_m = R`, i.e. the one-way distance or the radius; for a total-flown reading read the table at R/2).
- `E_budget`, one row per (site, mission, range): leg-prefixed segment Wh, `cruise_Wh, reserve_Wh, drop_Wh, total_Wh, usable_Wh, margin_Wh, margin_pct, fits`. Ranges shorter than the largest leg's fixed distance are skipped.
- `E_segments`, one row per (site, mission, leg, segment) plus the cruise: `mass_kg, energy_Wh, time_s, distance_m, source` and the `SegmentCheck` fields.
- `E_hover_check`, per site, at three SoC points: the stress point `window.soc_eod`, the SoC at the final landing's start, and the SoC after the reserve (floor 0). Columns as today plus `soc` and `point`. `hover_check` takes `soc` as an argument (it read `window.soc_eod`) and uses `mp.ground_amsl_m` as the site.

If the mission is not `flyable` the row still shows `max_range_km` (the energy answer); `flyable = False` and `first_infeasible` say it cannot be flown as configured.

## 7. Performance

One case runs two passes: about 2 x (legs x (segment functions + one D simulation + one `forward_accel`) + about 30 trims for the checks). Prototype: a few seconds per (site, mission). Eight cases is under a minute. The quick basics task stays quick; do not parallelise.

## 8. Expected results (check against these)

Repo vehicle (26.664 kg, 21.664 kg after the 5 kg drop), `TKOFF_THR_MAX` 0.8, `soc_start` 1.0, `soc_mean` 0.6, 1776 Wh usable, `a_plan` 2.5, entry 20.9 EAS, configured controller, `a_fwd_mode: case`, metric `min`. Cruise TAS is 21.11, 22.73, 23.9 and 27.27 m/s at cruise altitudes 200, 1700, 2700, 5200 m (EAS 20.91).

| site | round-trip radius (flown) | one-way | fixed Wh out / return | reserve | SoC end, before / after reserve | `a_fwd` out / return |
| --- | --- | --- | --- | --- | --- | --- |
| 0 m | 45.9 km (91.8) | 92.1 km | 118 / 95 | 150 Wh | 0.268 / 0.200 | 1.71 / 1.69 |
| 1500 m | 46.3 km (92.7) | 93.1 km | 123 / 99 | 153 Wh | 0.269 / 0.200 | 1.48 / 1.48 |
| 2500 m | 46.6 km (93.2) | 93.7 km | 126 / 102 | 155 Wh | 0.270 / 0.200 | 1.33 / 1.33 |
| 5000 m | 47.1 km (94.1) | 95.1 km | 137 / 111 | 161 Wh | 0.272 / 0.200 | 1.14 / 1.00 |

Energy per kilometre in cruise (loaded / empty): 16.90 / 15.87 (0 m), 16.65 / 15.63 (1500 m), 16.12 / 15.12 (5000 m) Wh/km. Segments at 1500 m, outbound: take-off 33.1, forward transition 12.2, climb 49.7, descent 2.1, back transition 4.3, landing 21.3 Wh. The leg SoC at the start of the forward transition is about 0.98 to 0.99 (outbound) and 0.61 (return) at every site; that is why the return `a_fwd` is not much higher than the outbound one even though the aircraft is lighter.

**Pusher command against the ceiling** (report-only; maximum over both ends of the segment):

| site | outbound climb | return climb | outbound cruise | return cruise |
| --- | --- | --- | --- | --- |
| 0 m | 0.80 (over by 0.004) | 0.83 over | 0.69 | 0.73 |
| 1500 m | 0.84 over | 0.87 over | 0.73 | 0.78 |
| 2500 m | 0.87 over | 0.90 over | 0.76 | 0.81 over |
| 5000 m | 0.95 over | 0.99 over | 0.85 over | 0.91 over |

"over" means above 0.8, which is flagged but not a failure. **Nothing is infeasible at the defaults** in either mission. The one-way cruise runs lower in SoC than the round-trip outbound one and reads 0.74, 0.79, 0.83 and 0.92 at 0, 1500, 2500 and 5000 m (over only at 2500 and 5000 m). The tightest margin is the 5000 m return climb at 0.988 against the limit of 1 (about 0.01): the flag could go either way across platforms.

To exercise the failure path (acceptance check 7) run the same case with `soc_start` 0.8: the return climb at 5000 m in the round trip becomes infeasible (command 1.04 at SoC 0.40 to 0.38, `limit = "pusher command > 1"`) and every other segment stays feasible.

## 9. README (`analysis/README.md`, E section)

Rewrite to say: the two missions and what a leg is; that range is an output and the requirements are comparison columns; the site sweep and the fixed climb height; the constant-dynamic-pressure speed rule; the SoC trace (energies at `soc_mean`, checks at both ends of each segment); that a throttle above `TKOFF_THR_MAX` is reported and above 1 fails; the `a_fwd` source per leg; the reserve rule. Replace the "one leg at one mass (a return leg is a second call)" assumption text and the hand-calculated reference-case paragraph only if the tests change (they should not).

## 10. Acceptance checks

1. **Regression:** `BudgetResult`, its tests (including the M4 hand-calculated case: 135.33 km, 424 and 268 Wh margins) and `hover_check` at its old SoC are unchanged.
2. **Speed rule:** `design_speeds(mp, 1500)` returns (22.5, 22.5, 22.5) and EAS 20.91; at 0, 2700 and 5200 m the cruise TAS is 21.11 (200 m), 23.9 and 27.27 within 0.05; with `v_climb` set to 20 the climb TAS at 1500 m is 20.0 and at 5200 m is 24.2 within 0.1.
3. **Geometry:** for site 1500 the profile has `ground_amsl_m` 1500, `cruise_amsl_m` 1700, and the climb segment's `dh` is 170 m; the 5000 m case no longer climbs 5 km (climb about 53 Wh, not 1351).
4. **Ranges:** the table in section 8, within 0.3 km (radius, flown total and one-way); fixed Wh within 2; reserve within 2 Wh.
5. **Consistency:** `MissionBudget.total_J(max_range_m)` equals `usable_J` within 1 J; for the `one_way` mission it equals a single `BudgetResult` with the same reserve.
6. **Trace:** the first row starts at `soc_start`; the last SoC equals `soc_start - total_J(r_max) / e_nom` within 1e-9; SoC is non-increasing.
7. **Throttle semantics:** a segment with command 0.9 is feasible with `over_ceiling = True`; a command of 1.04 is infeasible with `limit = "pusher command > 1"`; the section 8 table is reproduced (nothing infeasible at the defaults); with `soc_start` 0.8 the only infeasible segment is the round-trip return climb at 5000 m (1.04).
8. **Climb diagnostic:** at 1500 m, SoC 0.6, loaded mass, `roc_at_ceiling_ms` about 0.88 m/s (about 1.91 at SoC 1.0), within 0.05, with the leg's cruise speed (22.73 m/s) and the density at the climb's mean altitude; NaN where level cruise is already above the ceiling.
9. **Forward transition:** `a_fwd_mode: case` gives the section 8 values within 0.02; `worst` gives the Q2 global worst for every leg; `mission` gives 1.5; a non-positive value produces an error row and no budget (no fallback).
10. **Controller:** the back-transition energy at site 1500, outbound, is about 4.3 Wh (3.8 at 0 m, 5.9 at 5000 m), not the old 4.5 Wh at a 22.5 m/s entry; changing `Q_A_ANGLE_MAX` in `controller.yaml` changes it.
11. **`a_plan` source:** with D not in the run, `E_summary.a_plan_source` is `"fallback"` and an error is appended.
12. **Config:** the loader rejects `cruise_amsl_m` in `mission.yaml` or `E:` as unknown; it raises when `cruise_stub.v_cruise` differs from `profile.v_cruise`; `mission.yaml` without `v_climb` and `v_glide` loads and uses the cruise speed.
13. **Window flag:** with the defaults `window_ok` is True in every row (SoC after the reserve is 0.200 against `soc_eod` 0.1); with `soc_start` 0.8 it is False (about 0.0).

## 11. What changes in the numbers (for the PR description)

Against today's E: the altitude sweep is flat instead of rising (one-way 92 to 95 km across 0 to 5000 m, not 85 to 94 km on the old geometry and 85 to 107 km with the altitude fix alone); a round trip reaches about 46 km radius; every climb runs above the 0.8 ceiling (reported) and the 5000 m return climb is within about 0.01 of the limit of 1; the forward transition takes 0.8 to 1.5 m/s^2 depending on site and leg (not the assumed 1.5) and costs a little more; the back transition uses the configured controller.

## 12. Caveats to carry into the PR description

1. **The usable-energy window is read as a fraction of the nominal capacity from a full pack** (owner's decision): `soc_start` 1.0, the trace ends at 0.2, and `soc_eod` (0.1) is a stress point, not the end of the mission. If the battery-management cut-off turns out to be 0.1, the same 1776 Wh then leaves a 0.1 SoC floor unused, and `eta_usable` should read 0.9 (O-15). A start below a full pack (the scenarios use 0.8) would spend the whole pack: with `soc_start` 0.8 the trace ends at 0.0 and `window_ok` is False.
2. **`soc_mean` (0.6) is now the mean of the trace** (1.0 to 0.2), so the energies and the trace agree. Energies move by about 0.5 km between 0.4 and 0.8, so this is not sensitive either way.
3. **The ceiling is applied to climb and cruise too** (owner's decision). In ArduPilot a different parameter (`THR_MAX`) normally governs cruise; confirm. The report columns let either reading be read off.
4. **Climb at the cruise speed** is not the best-climb speed; it costs about 3 to 4 Wh more on the 170 m climb and is what makes the 5000 m return climb infeasible. A separate `v_climb` override is possible (section 3).
5. **The prototype `a_plan` was 2.5**; with the D spec merged `a_rec` is about 2.0 to 2.1 and the back transition costs a little more. Whether E should read `a_rec_pct` instead is DEC-3.
6. **The 5000 m site is a design-envelope check**, not a delivery site (Maharashtra terrain tops out near 1.6 km; check against the mission set, O-14).
7. **Battery model:** linear OCV, assumed R0/R1/C1, `V_min` 33 V TBD, no pack current limit. The SoC-dependent commands in section 8 are this model's.
8. **Trim bracket (A section 7.11):** the glide and climb trims still search only up to the stall angle; they work at the converted speeds on this grid but can fail at other speeds or masses. That shows up as an infeasible segment, not a crash.
9. **Not modelled:** wind, route factor, re-climb after a go-around, temperature derating, a contingency margin on cruise, payload-drop procedure beyond `t_drop_s`.

## 13. How the scratch numbers were made (not for the repo)

`load_inputs` on a temporary copy of the config folder (without the two A keys the older `settings.py` forbids), draft mode, the real `vehicle.yaml`, `mission.yaml` and `controller.yaml`. A prototype of sections 4 to 6 (legs at the two masses, the configured controller from `inp.controller`, entry at `inp.entry_eas`, `a_plan` 2.5, `pusher.forward_accel` from the Q2 prototype at `TKOFF_THR_MAX`, SoC trace from energy over 2220 Wh starting at 1.0, trims checked at both ends of each segment) run for sites 0, 1500, 2500 and 5000 m; the `soc_start` 0.8 variant gives the failure case in section 8. The 5000 m climb command at SoC 0.6 (1.03) and the best rate of climb at the ceiling come from the same trims run separately.
