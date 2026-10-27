# ruff: noqa: E501  (data tables with long source notes)
"""Regenerate the shipped placeholder config files in configs/ from the current defaults.

    python scripts/make_placeholder_configs.py

The files are meant to be EDITED by hand with real data afterwards; this script exists so the
shipped placeholders stay exactly equal to the code defaults (a test checks the round trip) and
so a clean set can be recreated. It overwrites configs/*.yaml, so do not run it over real data.
"""

from __future__ import annotations

import dataclasses
import math
from pathlib import Path
from typing import Any

from sagetrans.analysis.energy import BatteryWindow, MissionProfile
from sagetrans.control.ardupilot_like import BackTransitionParams, BaselineParams
from sagetrans.control.forward import ForwardParams
from sagetrans.physics.params import VehicleParams

OUT = Path(__file__).resolve().parents[1] / "configs"

# (group in the yaml, attribute path in VehicleParams or None for the top level, field) ->
# (unit, source note). Units are the ones a person would type; they are converted to SI on load.
VEHICLE_GROUPS: list[tuple[str, str | None, str]] = [
    ("mass", None, "Total take-off mass. What does '5 kg' mean (payload or MTOW)? Open question O-12."),
    ("wing", "wing", "Wing: area, linear lift, stall and drag. Flat-plate post-stall model."),
    ("rotor", "rotor", "Four lift rotors (identical in the point-mass model), polynomial maps."),
    ("pusher", "pusher", "Pusher propeller, polynomial maps in J_p = V cos(alpha) / (n D)."),
    ("motor_lift", "motor_r", "Lift motor and ESC (one motor; all four assumed identical)."),
    ("motor_pusher", "motor_p", "Pusher motor and ESC."),
    ("battery", "battery", "Two packs in parallel through a PDB; equivalent-circuit model."),
    ("attitude", "attitude", "Pitch-tracking surrogate (full rotational dynamics are out of scope)."),
]

# field -> (unit, source). Fields not listed fall back to ("-", "placeholder").
FIELD_META: dict[tuple[str, str], tuple[str, str]] = {
    ("mass", "m"): ("kg", "CAD and weighing; define mass case (nominal / max / min)"),
    ("wing", "S"): ("m^2", "CAD or flight-mechanics sheet"),
    ("wing", "CL0"): ("-", "lift coefficient at zero wing angle of attack (incl. incidence effects)"),
    ("wing", "CL_alpha"): ("rad^-1", "lift-curve slope; flight-mechanics sheet or wind tunnel"),
    ("wing", "alpha_s"): ("rad", "stall angle of attack"),
    ("wing", "CD0"): ("-", "zero-lift drag coefficient"),
    ("wing", "k"): ("-", "induced-drag factor, CD = CD0 + k CL^2"),
    ("wing", "CN"): ("-", "flat-plate normal-force coefficient (Cory and Tedrake 1.2 to 2); sourced or measured value needed"),
    ("wing", "i_w"): ("rad", "wing incidence relative to the body x-axis"),
    ("wing", "M"): ("rad^-1", "stall-blend sharpness: a NUMERICAL parameter, not physical; 100 needed for T-02"),
    ("rotor", "n_r"): ("-", "4+1 configuration (stated)"),
    ("rotor", "D"): ("m", "propeller diameter; propulsion trade study (O-13)"),
    ("rotor", "I_rot"): ("kg*m^2", "rotor + motor inertia; sets spool-up"),
    ("rotor", "CT0"): ("-", "static thrust coefficient T = CT rho n^2 D^4; thrust bench"),
    ("rotor", "CT1"): ("-", "thrust-coefficient slope in axial advance ratio J; BEMT or published data"),
    ("rotor", "CT2"): ("-", "quadratic term in J"),
    ("rotor", "CQ0"): ("-", "static torque coefficient Q = CQ rho n^2 D^5; thrust bench"),
    ("rotor", "CQ1"): ("-", "torque-coefficient slope in J"),
    ("rotor", "CQ2"): ("-", "quadratic term in J"),
    ("rotor", "CH1"): ("-", "in-plane force coefficient per unit edgewise ratio mu"),
    ("rotor", "k_mu"): ("-", "edgewise-flow factor (1 + k_mu mu^2) on CT and CQ"),
    ("rotor", "J_min"): ("-", "map validity: lowest axial advance ratio (negative = descent)"),
    ("rotor", "J_max"): ("-", "map validity: highest axial advance ratio"),
    ("rotor", "mu_max"): ("-", "map validity: highest edgewise advance ratio"),
    ("rotor", "omega_floor"): ("rad/s", "NUMERICAL guard keeping J and mu finite at standstill"),
    ("pusher", "D"): ("m", "propeller diameter"),
    ("pusher", "I_p"): ("kg*m^2", "propeller + motor inertia"),
    ("pusher", "CT0"): ("-", "thrust coefficient at J_p = 0"),
    ("pusher", "CT1"): ("-", "slope in J_p (goes negative at high J_p: windmilling drag)"),
    ("pusher", "CT2"): ("-", "quadratic term in J_p"),
    ("pusher", "CQ0"): ("-", "torque coefficient at J_p = 0"),
    ("pusher", "CQ1"): ("-", "slope in J_p"),
    ("pusher", "J_min"): ("-", "map validity"),
    ("pusher", "J_max"): ("-", "map validity"),
    ("pusher", "omega_floor"): ("rad/s", "NUMERICAL guard"),
    ("battery", "n_parallel"): ("-", "Y-connector to PDB (stated)"),
    ("battery", "cells_series"): ("-", "MPower 12S (stated)"),
    ("battery", "R0_pack"): ("ohm", "series resistance of ONE pack; HPPC/GITT fit"),
    ("battery", "R1_pack"): ("ohm", "RC-branch resistance of ONE pack; HPPC/GITT fit"),
    ("battery", "C1_pack"): ("F", "RC-branch capacitance of ONE pack; HPPC/GITT fit"),
    ("battery", "I_av"): ("A", "avionics current drawn from the bus"),
    ("battery", "V_min"): ("V", "bus cut-off; battery-management limit (O-15)"),
    ("battery", "capacity_Ah_pack"): ("Ah", "MPower datasheet, per pack"),
    ("battery", "v_nominal"): ("V", "MPower 12S datasheet"),
    ("attitude", "omega_n"): ("rad/s", "pitch-loop natural frequency; SITL or flight logs"),
    ("attitude", "zeta"): ("-", "pitch-loop damping ratio"),
    ("attitude", "q_max"): ("rad/s", "pitch-rate limit"),
}
for g in ("motor_lift", "motor_pusher"):
    FIELD_META.update({
        (g, "Ke"): ("V*s/rad", "back-EMF constant (Kv-derived or bench)"),
        (g, "Kt"): ("N*m/A", "torque constant (equals Ke in SI)"),
        (g, "Rm"): ("ohm", "winding resistance; datasheet or bench"),
        (g, "I0"): ("A", "no-load current"),
        (g, "I_lim"): ("A", "motor/ESC current limit"),
        (g, "dead_zone"): ("-", "ESC dead-zone fraction of throttle; bench motor test"),
    })

# values kept at a grade better than TBD because the brief or a datasheet gives them
GRADED = {
    ("rotor", "n_r"): "stated",
    ("battery", "n_parallel"): "stated",
    ("battery", "cells_series"): "stated",
    ("battery", "capacity_Ah_pack"): "datasheet",
    ("battery", "v_nominal"): "datasheet",
}


def num(v: Any) -> str:
    if isinstance(v, bool):
        return "true" if v else "false"
    if isinstance(v, int):
        return str(v)
    return repr(float(v))


def rec(value: Any, unit: str, source: str, status: str) -> str:
    src = source.replace('"', "'")
    return f'{{value: {num(value)}, unit: "{unit}", source: "{src}", status: {status}}}'


def vehicle_yaml() -> str:
    vp = VehicleParams()
    lines = [
        "# Vehicle parameter register. EDIT THIS FILE with real data.",
        "#",
        "# Every entry: value, unit, source, status. Units are converted to SI once, on load, so",
        "# write them in whatever unit is natural (kg, mm, deg, rpm, Ah ...).",
        "# status is one of: measured | datasheet | derived | documented | stated | assumed | TBD",
        "#",
        "# The values below are PLACEHOLDERS: illustrative numbers for a notional 8 kg 4+1 VTOL,",
        "# not Sage's data, and so every one is marked TBD. In 'draft' mode (the default) a TBD",
        "# entry is used as written and the whole run is labelled DRAFT; in 'release' mode any TBD",
        "# entry is refused. Replace a value, set its unit/source, and change TBD to measured,",
        "# datasheet, ... once it is real. An entry left out (or with no value) falls back to the",
        "# built-in placeholder in draft mode and is refused in release mode.",
        "",
    ]
    for group, attr, note in VEHICLE_GROUPS:
        obj = vp if attr is None else getattr(vp, attr)
        lines += [f"# {note}", f"{group}:"]
        for f in dataclasses.fields(obj):
            if attr is None and f.name != "m":
                continue
            if not f.init or f.name == "released" or dataclasses.is_dataclass(getattr(obj, f.name)):
                continue
            unit, source = FIELD_META.get((group, f.name), ("-", "placeholder"))
            status = GRADED.get((group, f.name), "TBD")
            lines.append(f"  {f.name}: {rec(getattr(obj, f.name), unit, source, status)}")
        lines.append("")
    return "\n".join(lines)


def mission_yaml() -> str:
    mp, bw = MissionProfile(), BatteryWindow()
    r = lambda v, u, s, st="TBD": rec(v, u, s, st)  # noqa: E731
    return "\n".join([
        "# Mission, usable-energy window, reserves and descent limits. EDIT THIS FILE.",
        "# Same format and status words as vehicle.yaml. TBD = placeholder (draft only).",
        "",
        "profile:        # the idealised flight used by the energy budget (analysis E)",
        f"  climb_height_m: {r(mp.climb_height_m, 'm', 'climb above the airfield; cruise altitude = site + this')}",
        f"  isa_offset_k: {r(mp.isa_offset_k, 'K', 'ISA temperature offset for the budget')}",
        f"  clearance_agl_m: {r(mp.clearance_agl_m, 'm', 'hover climb before the forward transition; also the height above ground at which analysis FT starts')}",
        f"  fwd_alt_loss_max_m: {rec(mp.fwd_alt_loss_max_m, 'm', 'largest altitude loss below the forward-transition start that is accepted; a failure beyond it (analysis FT)', 'assumed')}",
        f"  fwd_alt_gain_max_m: {rec(mp.fwd_alt_gain_max_m, 'm', 'largest altitude gain above the forward-transition start that is accepted; a failure beyond it (analysis FT)', 'assumed')}",
        f"  vz_takeoff: {r(mp.vz_takeoff, 'm/s', 'vertical climb rate in take-off')}",
        f"  t_hover_pre: {r(mp.t_hover_pre, 's', 'hover on the pad before lift-off')}",
        f"  a_fwd: {r(mp.a_fwd, 'm/s^2', 'forward-transition acceleration; ASSUMED until analysis C')}",
        f"  roc: {r(mp.roc, 'm/s', 'rate of climb')}",
        "  # v_cruise: omit to use the best-range speed, or add {value, unit, source, status}",
        f"  v_cruise_ref_amsl_m: {r(mp.v_cruise_ref_amsl_m, 'm', 'altitude where the design speeds hold')}",
        f"  payload_kg: {r(mp.payload_kg, 'kg', 'mass delivered at the destination; the return leg is lighter')}",
        f"  t_drop_s: {r(mp.t_drop_s, 's', 'hover at the destination for the drop, at the loaded mass')}",
        "  # v_climb, v_glide: omit to use the cruise speed, or add {value, unit, source, status}",
        f"  h_bt_agl_m: {r(mp.h_bt_agl_m, 'm', 'height above ground where the back-transition starts')}",
        f"  soc_mean: {r(mp.soc_mean, '-', 'one mean state of charge used for all segment powers')}",
        f"  t_go_around: {r(mp.t_go_around, 's', 'go-around hover time (reserve)')}",
        f"  t_hold: {r(mp.t_hold, 's', 'hold at the landing zone (reserve)')}",
        f"  d_diversion_m: {r(mp.d_diversion_m, 'm', 'diversion distance (reserve)')}",
        f"  n_diversion_transitions: {r(mp.n_diversion_transitions, '-', 'extra forward+back transition pairs')}",
        "",
        "battery_window:  # usable energy = eta_usable * soh * temp_factor * nominal energy (O-15)",
        f"  eta_usable: {r(bw.eta_usable, '-', 'usable fraction of nominal energy; BMS and cut-off limits')}",
        f"  soh: {r(bw.soh, '-', 'state of health (1 = new)')}",
        f"  temp_factor: {r(bw.temp_factor, '-', 'temperature derating (1 = 25 degC)')}",
        f"  soc_start: {r(bw.soc_start, '-', 'state of charge at take-off (a full pack)')}",
        f"  soc_eod: {r(bw.soc_eod, '-', 'state of charge at end of discharge, for the hover power check')}",
        "",
        "descent:         # analysis F",
        f"  kappa: {r(0.25, '-', 'UNSOURCED placeholder. Must come from a sourced vortex-ring-state boundary (Johnson, NASA). Do not quote results until replaced.')}",
        f"  v_fragility: {r(3.0, 'm/s', 'payload-fragility descent-rate limit')}",
        "  # v_sensor: omit for none, or add {value, unit, source, status} (rangefinder limit)",
        f"  h_slow: {r(5.0, 'm', 'below this height the descent slows to v_final')}",
        f"  v_final: {r(0.5, 'm/s', 'final descent rate; rangefinder and ground effect')}",
        "",
        "requirements:    # the range requirements the budget is checked against",
        f"  range_primary_km: {rec(100.0, 'km', 'specified range requirement', 'stated')}",
        f"  range_p95_km: {rec(113.0, 'km', 'about 113 km needed for the 95th-percentile mission', 'stated')}",
        "",
        "speeds:          # entry and transition speeds used by the studies",
        f"  entry_eas_ms: {r(22.0, 'm/s', 'equivalent airspeed at the start of the back-transition (cruise speed)', 'assumed')}",
        "",
    ])


def controller_yaml() -> str:
    bt, bl = BackTransitionParams(), BaselineParams(back=BackTransitionParams())
    fd = ForwardParams(back=BackTransitionParams(z_mode="ardupilot"))
    r = lambda v, u, s, st="TBD": rec(v, u, s, st)  # noqa: E731
    doc = "Plane docs"
    return "\n".join([
        "# ArduPilot-structured baseline controller (QuadPlane). EDIT THIS FILE.",
        "# Names in the `ardupilot` group are the real ArduPilot parameter names. Gains in the other",
        "# groups are ASSUMED structure/values until SITL or flight logs calibrate them.",
        "",
        "ardupilot:",
        f"  AIRSPEED_MIN: {r(bl.airspeed_min, 'm/s', 'transition speed. Compared as TRUE airspeed in the simulated controller; EAS vs TAS in ArduPilot is open (O-02)')}",
        f"  Q_TRANSITION_MS: {rec(bl.q_transition_s * 1000.0, 'ms', doc + ' default 5000', 'documented')}",
        f"  Q_BACKTRANS_MS: {r(bt.T_bt * 1000.0, 'ms', 'pitch-limit ramp time on back-transition')}",
        f"  Q_A_ANGLE_MAX: {r(round(math.degrees(bt.theta_A), 12), 'deg', 'max VTOL lean angle')}",
        "  # Q_BCK_PIT_LIM: omit for no cap, or add {value, unit, source, status}",
        f"  TKOFF_THR_MAX: {r(bl.tkoff_thr_max, '-', 'pusher throttle in AUTO during the forward transition')}",
        f"  Q_TRANS_FAIL: {r(bl.q_trans_fail_s * 1000.0, 'ms', '0 disables the stuck-transition timeout (documented)')}",
        f"  Q_M_SPIN_MIN: {r(bt.delta_min, '-', 'rotor throttle floor in altitude hold; set above the ESC dead zone')}",
        f"  Q_M_SPIN_MAX: {r(bt.delta_max, '-', 'rotor throttle ceiling')}",
        f"  PTCH_LIM_MAX_DEG: {r(35.0, 'deg', 'fixed-wing pitch limit (not a decision variable in analysis C)')}",
        f"  Q_TRAN_PIT_MAX: {r(fd.tran_pit_max_deg, 'deg', '(DEFAULT ARDU VALUE) pitch cap in the forward transition; the fade cap is (Q_TRAN_PIT_MAX + 1) x 2 (O-16)')}",
        f"  TKOFF_THR_MIN: {r(fd.tkoff_thr_min, '-', '(DEFAULT ARDU VALUE) 0 = use TRIM_THROTTLE as the pusher minimum in the forward transition (O-17)')}",
        f"  TRIM_THROTTLE: {r(fd.trim_throttle, '-', '(DEFAULT ARDU VALUE) THROTTLE_CRUISE 45 percent (config.h)')}",
        f"  THR_MIN: {r(fd.thr_min, '-', '(DEFAULT ARDU VALUE)')}",
        f"  THR_MAX: {r(fd.thr_max, '-', '(DEFAULT ARDU VALUE)')}",
        f"  THR_SLEWRATE: {r(fd.thr_slewrate * 100.0, '%/s', '(DEFAULT ARDU VALUE)')}",
        f"  PTCH_LIM_MIN_DEG: {r(fd.ptch_lim_min_deg, 'deg', 'ArduPilot default as remembered; not re-read from source. Check')}",
        "  # Q_TRANS_DECEL: normally an ANALYSIS OUTPUT (analysis D). Add it only to fix the planner",
        "  # value used when an analysis does not compute it.",
        "",
        "tecs:            # TECS defaults for the forward-transition model (analysis FT)",
        f"  TECS_TIME_CONST: {r(fd.tecs_time_const, 's', '(DEFAULT ARDU VALUE)')}",
        f"  TECS_THR_DAMP:   {r(fd.tecs_thr_damp, 's', '(DEFAULT ARDU VALUE)')}",
        f"  TECS_INTEG_GAIN: {r(fd.tecs_integ_gain, '-', '(DEFAULT ARDU VALUE)')}",
        f"  TECS_VERT_ACC:   {r(fd.tecs_vert_acc, 'm/s^2', '(DEFAULT ARDU VALUE)')}",
        f"  TECS_PTCH_DAMP:  {r(fd.tecs_ptch_damp, 's', '(DEFAULT ARDU VALUE)')}",
        f"  TECS_CLMB_MAX:   {r(fd.tecs_clmb_max, 'm/s', '(DEFAULT ARDU VALUE)')}",
        f"  TECS_SINK_MAX:   {r(fd.tecs_sink_max, 'm/s', '(DEFAULT ARDU VALUE)')}",
        f"  TECS_SINK_MIN:   {r(fd.tecs_sink_min, 'm/s', '(DEFAULT ARDU VALUE)')}",
        f"  TECS_PITCH_MAX:  {r(fd.tecs_pitch_max_deg, 'deg', '(DEFAULT ARDU VALUE) 0 would use PTCH_LIM_MAX_DEG')}",
        "",
        "altitude_hold:   # rotor altitude hold, eq. 8.5 (gains assumed)",
        f"  K_h: {r(bt.K_h, 's^-1', 'altitude error to demanded climb rate')}",
        f"  K_vh: {r(bt.K_vh, 's/m', 'throttle per m/s of climb-rate error')}",
        f"  K_i: {r(bt.K_i, 'm^-1', 'integral gain: throttle per metre of accumulated altitude-rate error')}",
        f"  v_dem_max: {r(bt.v_dem_max, 'm/s', 'climb-rate demand limit')}",
        f"  i_max: {r(bt.i_max, 'm', 'integral clamp')}",
        "",
        "position:        # position cascade, eq. 8.9 (open question O-11)",
        f"  K_p: {r(bt.K_p, 's^-1', 'position error to demanded speed (linear zone)')}",
        f"  K_v: {r(bt.K_v, 's^-1', 'speed error to commanded deceleration')}",
        f"  a_lim: {r(bt.a_lim, 'm/s^2', 'deceleration limit of the speed profile; what it is in Plane is open (O-11)')}",
        "",
        "cruise_stub:     # fixed-wing hold between the transitions",
        f"  v_cruise: {r(bl.v_cruise, 'm/s', 'cruise speed target (true airspeed)')}",
        f"  r_tecs: {r(bl.r_tecs, 's^-1', 'pusher-command rate limit; TECS_THR_ERATE meaning open (O-10)')}",
        f"  tau_h: {r(bl.tau_h, 's', 'pusher hand-off time constant')}",
        f"  K_s: {r(bl.K_s, 's/m', 'pusher command per m/s speed error')}",
        f"  K_si: {r(bl.K_si, 'm^-1', 'speed integral gain')}",
        f"  K_hp: {r(bl.K_hp, 'rad/m', 'pitch per metre of altitude error')}",
        f"  K_hd: {r(bl.K_hd, 'rad*s/m', 'pitch per m/s of climb rate')}",
        f"  theta_cruise_max: {r(bl.theta_cruise_max, 'rad', 'cruise pitch limit')}",
        "",
    ])


def scenarios_yaml() -> str:
    return """# Scenarios (proposal section 9). EDIT THIS FILE. Plain settings: no evidence grades.
# Units are fixed by the key suffix: _m metres, _k kelvin, _ms metres/second, soc is 0..1,
# mass_factor multiplies the vehicle mass. Headwind is positive AGAINST the aircraft.
#
# These are the named core scenarios that appear in every report. The values are placeholders
# (mission set O-14); S-05's elevation is "to set".

core:
  - {name: S-01, altitude_amsl_m: 0,    isa_offset_k: 0,  soc: 0.8, headwind_ms: 0,  mass_factor: 1.0,  note: "sea level, nominal"}
  - {name: S-02, altitude_amsl_m: 5000, isa_offset_k: 0,  soc: 0.8, headwind_ms: 0,  mass_factor: 1.0,  note: "5,000 m design point, nominal"}
  - {name: S-03, altitude_amsl_m: 5000, isa_offset_k: 10, soc: 0.5, headwind_ms: 0,  mass_factor: 1.15, note: "high, hot, heavy, low pack"}
  - {name: S-04, altitude_amsl_m: 0,    isa_offset_k: 20, soc: 0.8, headwind_ms: -5, mass_factor: 1.15, note: "hot, heavy, 5 m/s tailwind"}
  - {name: S-05, altitude_amsl_m: 500,  isa_offset_k: 0,  soc: 0.8, headwind_ms: 5,  mass_factor: 1.0,  note: "landing elevation to be set; 5 m/s headwind"}
  - {name: S-06, altitude_amsl_m: 5000, isa_offset_k: 0,  soc: 0.3, headwind_ms: 0,  mass_factor: 0.85, note: "payload delivered, low pack"}

# Latin-hypercube ensemble used by the robust optimisations and the recommendation.
ensemble:
  axes:
    altitude_m: [0, 5000]
    isa_offset_k: [-10, 20]
    mass_factor: [0.85, 1.15]
    headwind_ms: [-5, 10]
    soc: [0.2, 1.0]
"""


def study_yaml() -> str:
    return """# What to run and how much effort to spend. EDIT THIS FILE.
# Plain settings: no evidence grades. Everything here changes results, so this file is hashed
# into the run record.

run_name: study            # output goes to <output_dir>/<run_name>/
output_dir: results        # git-ignored
mode: draft                # draft: placeholders allowed, results labelled DRAFT (default)
                           # release: refuse to run while anything is TBD or missing

parallel:
  enabled: true
  max_workers: 0           # 0 = one per CPU core (capped by the number of tasks)

# Which analyses to run. Names: A D F E Q2 C B1 sensitivity
analyses: [A, D, F, E, Q2, C, B1, sensitivity]

switch_test: ground        # speed used in the Position1 switch test: ground | air  (O-02)

# ---- analysis settings -------------------------------------------------------------------------
A:                         # zero-altitude-loss feasibility
  scenarios: [S-01, S-02]
  v0_eas_ms: [10, 12, 14, 16, 18, 20]
  decel_ms2: [0.5, 1.0, 1.5, 2.0, 3.0]
  max_decel_v0_eas_ms: [10, 12, 14, 16]
  samples: 60

D:                         # braking and Q_TRANS_DECEL
  scenarios: [S-01, S-02]
  mass_factors: [0.85, 1.0, 1.15]
  v0_eas_ms: [16, 22]
  recommendation: {ensemble_n: 8, ensemble_seed: 3, percentile: 5}
  trade: {a_plan_ms2: [1.5, 2.0, 2.5, 3.0, 3.5], headwinds_ms: [0, 5]}

F:                         # landing descent profile
  scenarios: [S-01, S-02]

E:                         # energy budget
  sites_amsl_m: [0, 5000]          # airfield elevations
  ranges_km: [50, 100, 113, 150]   # ranges shorter than the climb+descent distance are skipped

Q2:                        # pusher thrust margin at AIRSPEED_MIN
  altitudes_m: [0, 1000, 2500, 5000]
  mass_factors: [0.85, 1.0, 1.15]

C:                         # ArduPilot-parameter optimisation (back-transition)
  ensemble_n: 4
  ensemble_seed: 1
  seeds: [1, 2, 3]
  popsize: 8
  maxiter: 15

B1:                        # open-loop least-energy back-transition from the AIRBRAKE exit (docs/B1_fix_spec.md)
  # The entry (rotor start, sink tolerance) comes from the C block, the entry speed rule from the D block.
  scenarios: [S-01, S-02]
  intervals: 40
  refine_from: [20]
  max_iter: 800
  eps_h_m: [1.0]                       # headline altitude band (C holds about 0.5 to 1 m)
  eps_x_m: [40, 50, 60, 75, 100, 300]  # distance caps swept: the energy-against-distance front
  end_vh_band_ms: null                 # null = C.sink_tol_ms
  variants:                            # the wing limit is stated, never silent; no "no limit" variant
    - {name: attached,   alpha_margin: 0.85,  theta_cmd_max_deg: 34.4}   # |alpha_w| <= 0.85 alpha_s
    - {name: c_envelope, alpha_max_deg: 30.0, theta_cmd_max_deg: 30.0}   # what C's ramp can reach
  band_sweep: {eps_h_m: [3.0, 10.0], variant: c_envelope}   # sensitivity at the largest distance cap
  sens: {rotor_start_flip: true, end_vh_band_ms: 0.5, c_envelope_theta_cmd_deg: 34.4}   # first scenario only
  gap_c: null                          # C parameters for the gap: {T_bt, theta_A_deg, spin_min, q}; null = register at q = 2

sensitivity:
  scenario: S-01                   # the scenario the sensitivity is run in
  r_small: 6
  r_large: 12
  include_budget: true             # adds maximum range as an output
"""


def main() -> None:
    OUT.mkdir(exist_ok=True)
    files = {
        "vehicle.yaml": vehicle_yaml(),
        "mission.yaml": mission_yaml(),
        "controller.yaml": controller_yaml(),
        "scenarios.yaml": scenarios_yaml(),
        "study.yaml": study_yaml(),
    }
    for name, text in files.items():
        (OUT / name).write_text(text, encoding="utf-8", newline="\n")
        print("wrote", OUT / name)


if __name__ == "__main__":
    main()
