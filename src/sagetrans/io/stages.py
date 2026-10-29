"""The analysis stages of a study, each a function of the loaded inputs.

A *task* is the unit of parallel work: ``basics`` (analyses A, D, F, E and Q2, which are quick and
share results), ``C``, ``B1:<scenario>`` (one per scenario) and ``sensitivity``. Every task
returns a plain dict of tables and objects that can be pickled, so it can be cached and run in a
separate process. Nothing here reads a file: all data arrives through `StudyInputs`.
"""

from __future__ import annotations

import dataclasses
import math
import time
import traceback
from collections.abc import Callable
from functools import partial
from typing import Any

import numpy as np
from scipy import stats
from scipy.optimize import brentq

from sagetrans import atmosphere, progress
from sagetrans.analysis import (
    braking,
    descent,
    energy,
    forward,
    pusher,
    restricted,
    runs,
    soc_trace,
    zero_loss,
)
from sagetrans.analysis import collocation as coll
from sagetrans.analysis import sensitivity as sens
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.ensemble import (
    CostWeights,
    bootstrap_percentile,
    braking_ensemble,
    latin_hypercube,
)
from sagetrans.analysis.search import Bounds
from sagetrans.control.ardupilot_like import BackTransitionParams
from sagetrans.io.gate import T12_SPREAD_MAX
from sagetrans.io.inputs import StudyInputs
from sagetrans.io.settings import END_SPEED_MS
from sagetrans.physics import aero, battery, motor, rotor
from sagetrans.physics.params import VehicleParams

RHO0 = float(atmosphere.density(0.0))
Table = list[dict[str, Any]]

# rough wall-clock seconds on a laptop, for scheduling and for the pre-flight estimate
# Seconds per task as observed in the last full clean run (2026-10-29: 24 workers on a 24-core
# machine, all busy, so they include the slowdown of running side by side; one worker alone is
# about 1.7 times faster). The run took 4.1 h: C 10269 s, FT_grid 6.7 to 8.6 ks, FT_scan 6.4 to
# 9.1 ks, B1 1.0 to 1.7 ks, the fine-pass FT_grid points 3.9 to 4.0 ks, basics 848 s.
# "FT_fine" is the fine pass (a FT_grid point planned after the first wave).
TASK_COST = {"basics": 850, "C": 10300, "FT": 370, "FT_grid": 7700, "FT_fine": 4000,
             "FT_scan": 7700, "B1": 1400, "sensitivity": 420, "basics_rec": 110}


def tas(eas: float, scen: Scenario) -> float:
    """True airspeed for an equivalent airspeed at the scenario's density."""
    return eas * math.sqrt(RHO0 / scen.rho)


def _num(x: float) -> float | str:
    """A table cell: the number, or empty if it is not finite (no NaN or inf in written files)."""
    return float(x) + 0.0 if math.isfinite(x) else ""  # + 0.0: no "-0.0" in the files


def _attempt(label: str, fn: Callable[[], Any], errors: list[str]) -> Any:
    """Run one piece of a task; a failure is recorded in `errors` with its traceback tail."""
    t = time.time()
    progress.phase(label)
    try:
        out = fn()
        progress.note(f"ok   {label} ({time.time() - t:.0f} s)")
        return out
    except Exception as exc:  # noqa: BLE001
        tb = traceback.format_exc().strip().splitlines()[-3:]
        errors.append(f"{label}: {type(exc).__name__}: {exc}")
        progress.note(f"FAIL {label}: {exc}\n      " + "\n      ".join(tb), warn=True)
        return None


def _ensemble(inp: StudyInputs, n: int, seed: int) -> list[Scenario]:
    return latin_hypercube(n, inp.vehicle, seed=seed, axes=inp.ensemble_axes)


def exit_params(
    inp: StudyInputs, *, rotor_lag_s: float, alpha_cap: float | None = None, **extra: Any
) -> BackTransitionParams:
    """Controller parameters at the AIRBRAKE exit: the register's values plus the D block's.

    The exit state belongs to D's block (`theta_fw0_deg`, `pit_lim_mode`, the lift feed-forward,
    the alpha gate), so D and C build their parameters here and cannot drift. `rotor_lag_s` is the
    caller's own (D's or C's) and `extra` any other field (C passes the Z-loop ones).
    """
    c = inp.study.D
    return dataclasses.replace(
        inp.controller, theta_fw0=math.radians(c.theta_fw0_deg), pit_lim_mode=c.pit_lim_mode,
        v_min_ref=inp.airspeed_min, ff_mode=c.ff_mode, ff_lift_tau_s=c.ff_lift_tau_s,
        ff_lift_scale=c.ff_lift_scale, rotor_lag_s=rotor_lag_s, alpha_cap=alpha_cap,
        alpha_cap_vmin=c.alpha_gate_ms, **extra,
    )  # fmt: skip


def d_exit_params(inp: StudyInputs) -> BackTransitionParams:
    """The controller analysis D runs at the AIRBRAKE exit (its rotor lag, its angle cap).

    E's back transition and the sensitivity's budget use it too, so no stopping run in the study
    has other controller settings than D's.
    """
    lim, _ = alpha_limits(inp)
    cap = zero_loss._alpha_band(inp.vehicle, lim)[1]  # body pitch at the top of A's band
    return exit_params(
        inp, rotor_lag_s=inp.study.D.rotor_lag_s,
        alpha_cap=cap if inp.study.D.enforce_alpha else None,
    )


# ---- Q2: pusher thrust margin ------------------------------------------------------------------
def pusher_margin(vp: VehicleParams, rho: float, v_tas: float, soc: float) -> dict[str, float]:
    """Full-throttle pusher thrust against the wing drag at the transition speed.

    The pusher runs at full effective throttle; its speed solves motor torque = propeller torque
    at the loaded bus voltage. Drag assumes the wing carries the whole weight (lift = W), which
    needs C_L <= C_L,max; below that speed the rotors still help and the margin is not defined.
    """
    b, mp = vp.battery, vp.motor_p
    voc = float(b.voc(soc))

    def bus(w: float) -> float:
        return float(
            battery.solve_bus_voltage(voc, 0.0, b, vp.rotor.n_r, vp.motor_r, mp, 0.0, 1.0, 0.0, w)
        )

    def resid(w: float) -> float:
        i_m = float(motor.current(1.0, bus(w), w, mp))
        return float(motor.torque(i_m, mp)) - float(rotor.pusher(w, v_tas, rho, vp.pusher)[1])

    w = float(brentq(resid, 1.0, 4000.0, xtol=1e-9))
    t_max = float(rotor.pusher(w, v_tas, rho, vp.pusher)[0])
    i_m = float(motor.current(1.0, bus(w), w, mp))
    q_s = 0.5 * rho * v_tas**2 * vp.wing.S
    cl = vp.m * 9.80665 / q_s
    cl_max = runs.cl_max(vp.wing)
    drag = q_s * (vp.wing.CD0 + vp.wing.k * cl**2) if cl <= cl_max else math.nan
    return {
        "T_max_N": t_max, "omega_p_rad_s": w, "I_pusher_A": i_m,
        "current_limited": float(i_m >= mp.I_lim - 1e-6), "CL_required": cl, "CL_max": cl_max,
        "D_N": drag, "margin_N": t_max - drag, "accel_ms2": (t_max - drag) / vp.m,
    }  # fmt: skip


# ---- Q2: forward acceleration of the pusher ------------------------------------------------------
Q2Cell = tuple[dict[str, Any], pusher.FwdAccel]


def q2_cells(inp: StudyInputs, conventions: list[str]) -> list[Q2Cell]:
    """Q2's grid: altitude x mass factor x SoC for each airspeed convention.

    Each cell is its table row and the full acceleration profile. `AIRSPEED_MIN` is read as a
    true airspeed (`tas`, as the register does) or as an equivalent one (`eas`, the old Q2).
    """
    c = inp.study.Q2
    vp = inp.vehicle
    key = "controller.ardupilot.TKOFF_THR_MAX"
    status = "fallback (placeholder)" if key in inp.fallbacks else inp.status(key)
    cells: list[Q2Cell] = []
    for conv in conventions:
        for alt in c.altitudes_m:
            sc = Scenario(alt, isa_offset_k=c.isa_offset_k)
            v_end = inp.airspeed_min if conv == "tas" else tas(inp.airspeed_min, sc)
            for mf in c.mass_factors:
                vps = dataclasses.replace(vp, m=vp.m * mf)
                for soc in c.soc:
                    f = pusher.forward_accel(vps, sc.rho, v_end, soc, inp.tkoff_thr_max, c.n_speed)
                    row = {
                        "convention": conv, "altitude_m": alt, "rho_kg_m3": sc.rho,
                        "mass_factor": mf, "mass_kg": vps.m, "soc": soc,
                        "tkoff_thr_max": inp.tkoff_thr_max, "tkoff_status": status,
                        "v_end_tas_ms": v_end, "a_min_ms2": _num(f.a_min),
                        "v_at_min_ms": f.v_at_min, "a_equiv_ms2": _num(f.a_equiv),
                        "t_s": _num(f.t_s), "x_m": _num(f.x_m), "v_reach_ms": f.v_reach,
                        "reachable": f.reachable, "rotor_saturated": f.rotor_saturated,
                        "current_limited": f.current_limited,
                    }  # fmt: skip
                    cells.append((row, f))
    return cells


def q2_worst(cells: list[Q2Cell], conv: str) -> Q2Cell:
    """The cell with the smallest minimum acceleration in one convention (never clipped at 0)."""
    return min((c for c in cells if c[0]["convention"] == conv), key=lambda c: c[1].a_min)


def site_mission(inp: StudyInputs, site: float) -> energy.MissionProfile:
    """The mission at a site, with `v_trans` as a true airspeed (Q2.feed_convention) and the
    rotor fade of the register's (or the recommended) `Q_TRANSITION_MS`."""
    mp = energy.mission_at_site(inp.mission, site, inp.study.Q2.feed_convention)
    return dataclasses.replace(
        mp, q_transition_s=float(inp.start_params["q_transition"]),
        wing_share_handover=inp.study.E.fade_wing_share,
    )


# ---- E: the mission budget per site -------------------------------------------------------------
@dataclasses.dataclass
class ECase:
    """One solved (site, mission): the budget, its range, the SoC trace and the checks."""

    mb: energy.MissionBudget
    r_max: float
    rows: list[soc_trace.TraceRow]
    soc_before: float
    soc_after: float
    checks: list[soc_trace.SegmentCheck]
    a_fwd: dict[str, float]


def _fwd_accel_by_leg(
    inp: StudyInputs, vp: VehicleParams, mp: energy.MissionProfile, masses: dict[str, float],
    socs: dict[str, float], worst: float,
) -> dict[str, float]:
    """Forward-transition acceleration of each leg: Q2 at the site, mass and SoC of the leg
    (`case`), the global worst over the Q2 grid (`worst`) or the mission file (`mission`)."""
    ec, q2c = inp.study.E, inp.study.Q2
    if ec.a_fwd_mode == "mission":
        return dict.fromkeys(masses, mp.a_fwd)
    if ec.a_fwd_mode == "worst":
        return dict.fromkeys(masses, worst)
    rho_c = float(atmosphere.density(mp.clearance_amsl, mp.isa_offset_k))
    v_end = mp.v_trans  # already a true airspeed (`energy.mission_at_site`)
    out: dict[str, float] = {}
    for leg, m in masses.items():
        f = pusher.forward_accel(
            dataclasses.replace(vp, m=m), rho_c, v_end, socs[leg], inp.tkoff_thr_max, q2c.n_speed
        )
        out[leg] = f.a_min if ec.a_fwd_metric == "min" else f.a_equiv
    return out


def solve_e_case(
    inp: StudyInputs, site: float, kind: str, a_plan: float, worst: float
) -> ECase:
    """Solve one site and mission: the range, the SoC trace and the segment checks.

    Pass 1 takes every leg's forward-transition acceleration at the mean SoC; pass 2 at the SoC the
    trace gives at the start of that leg's forward transition (only `case` mode depends on it).
    Raises ValueError for a non-positive acceleration: there is no fallback to the mission value.
    """
    vp, ec = inp.vehicle, inp.study.E
    mp = site_mission(inp, site)
    masses = {"out": vp.m, "ret": vp.m - mp.payload_kg} if kind == "round_trip" else {"leg": vp.m}
    rho_bt = float(atmosphere.density(site + mp.h_bt_agl_m, mp.isa_offset_k))
    # the stopping run starts at the AIRBRAKE exit, as in D, C and B1 (AIRSPEED_MIN - 2)
    v0 = braking.entry_airspeed(inp.airspeed_min, rho_bt, inp.study.D.entry_convention)
    e_nom = vp.battery.e_nominal_wh * 3600.0
    src = None
    if ec.a_fwd_mode != "mission":
        src = f"quasi-steady estimate, a_fwd from Q2 ({ec.a_fwd_mode})"
    socs = dict.fromkeys(masses, mp.soc_mean)
    for n_pass in (1, 2):
        a = _fwd_accel_by_leg(inp, vp, mp, masses, socs, worst)
        bad = {k: v for k, v in a.items() if not (math.isfinite(v) and v > 0.0)}
        if bad:
            raise ValueError(
                f"a_fwd = {', '.join(f'{k} {v:.3g}' for k, v in bad.items())} m/s2 "
                f"({ec.a_fwd_mode}, {ec.a_fwd_metric}): the pusher cannot reach AIRSPEED_MIN at "
                f"TKOFF_THR_MAX = {inp.tkoff_thr_max:g}"
            )
        mb = energy.build_mission(vp, mp, inp.window, inp.descent, a_plan, kind, a,
                                  bp=d_exit_params(inp), v0_entry=v0, a_fwd_source=src)  # fmt: skip
        r_max = mb.max_range_m
        if r_max <= 0.0:
            raise ValueError("the usable energy does not cover the fixed segments and the reserve")
        rows, s_before, s_after = soc_trace.trace(mb, r_max, inp.window.soc_start, e_nom)
        if n_pass == 2 or ec.a_fwd_mode != "case":
            break
        socs = {r.leg: r.soc_start for r in rows if r.segment == "fwd_transition"}
    checks = soc_trace.check_mission(mb, rows, inp.tkoff_thr_max)
    return ECase(mb, r_max, rows, s_before, s_after, checks, a)


# ---- task: basics (A, D, F, E, Q2) -------------------------------------------------------------
def task_basics(inp: StudyInputs) -> dict[str, Any]:
    wanted = set(inp.study.analyses)
    errors: list[str] = []
    out: dict[str, Any] = {"tables": {}, "errors": errors}
    vp = inp.vehicle
    q2_memo: dict[str, list[Q2Cell]] = {}

    def q2_grid(conventions: list[str]) -> list[Q2Cell]:
        """Q2's cells, computed once per convention and shared by Q2 and E."""
        for conv in conventions:
            if conv not in q2_memo:
                q2_memo[conv] = q2_cells(inp, [conv])
        return [cell for conv in conventions for cell in q2_memo[conv]]

    def stage_a() -> None:
        c = inp.study.A
        kappa = inp.descent.kappa
        if c.vrs_enforce and kappa is None:
            raise ValueError("A.vrs_enforce needs mission.descent.kappa, which is not set")
        lim = zero_loss.ActuatorLimits(
            alpha_min=None if c.alpha_min_deg is None else math.radians(c.alpha_min_deg),
            cl_frac=c.cl_frac,
            vrs_limit=-kappa if c.vrs_enforce and kappa is not None else None,
        )
        maps: dict[str, Any] = {}
        rows: Table = []
        win: Table = []
        for k_sc, name in enumerate(c.scenarios, start=1):
            progress.step(f"A: feasibility map of {name}", k_sc - 1, len(c.scenarios))
            sc = inp.scenarios[name]
            vps = sc.vehicle_for(vp)
            v_grid = [tas(v, sc) for v in c.v0_eas_ms]
            fm: dict[str, Any] = dict(
                zero_loss.feasibility_map(vps, sc, v_grid, c.decel_ms2, lim, n=c.samples)
            )
            maps[name] = {"v_grid_eas": c.v0_eas_ms, "a_grid": c.decel_ms2, **fm}
            for i, ve in enumerate(c.v0_eas_ms):
                for j, a in enumerate(c.decel_ms2):
                    viol = fm["violations"][i][j]
                    rows.append({
                        "scenario": name, "altitude_m": sc.altitude_amsl_m, "V0_eas_ms": ve,
                        "V0_tas_ms": v_grid[i], "decel_ms2": a,
                        "feasible": bool(fm["feasible"][i, j]),
                        "violated": ";".join(f"{k}@{t:.2f}" for k, t in viol.items()),
                        "vrs_min": _num(fm["vrs_min"][i, j]),
                        "vrs_min_t_s": _num(fm["vrs_min_t"][i, j]),
                    })  # fmt: skip
                base = {"scenario": name, "altitude_m": sc.altitude_amsl_m, "V0_eas_ms": ve,
                        "V0_tas_ms": v_grid[i]}  # fmt: skip
                windows = zero_loss.feasible_window(vps, sc, v_grid[i], lim)
                for w in windows:
                    win.append({**base, "a_lo": w.a_lo, "a_hi": w.a_hi,
                                "bind_lo": ";".join(w.bind_lo), "bind_hi": ";".join(w.bind_hi),
                                "theta_lo_deg": math.degrees(w.theta_lo),
                                "theta_hi_deg": math.degrees(w.theta_hi),
                                "vrs_lo": _num(w.vrs_lo), "vrs_hi": _num(w.vrs_hi)})  # fmt: skip
                if not windows:  # no feasible deceleration: say what fails closest to one
                    _, names = zero_loss.closest_to_feasible(vps, sc, v_grid[i], lim)
                    win.append({**base, "a_lo": "", "a_hi": "", "bind_lo": ";".join(names),
                                "bind_hi": "", "theta_lo_deg": "", "theta_hi_deg": "",
                                "vrs_lo": "", "vrs_hi": ""})  # fmt: skip
        out["tables"]["A_feasibility"] = rows
        out["tables"]["A_window"] = win
        out["maps"] = maps
        mx: Table = []
        for name in c.scenarios:
            sc = inp.scenarios[name]
            for ve in c.max_decel_v0_eas_ms:
                rng = zero_loss.feasible_decel_range(
                    sc.vehicle_for(vp), sc, tas(ve, sc), lim, n=c.samples
                )
                mx.append({"scenario": name, "altitude_m": sc.altitude_amsl_m, "V0_eas_ms": ve,
                           "min_feasible_decel_ms2": "" if rng is None else rng[0],
                           "max_feasible_decel_ms2": 0.0 if rng is None else rng[1],
                           "islands": "" if rng is None else rng[2]})  # fmt: skip
        out["tables"]["A_max_feasible_decel"] = mx

    def stage_d() -> None:
        c = inp.study.D
        rec = c.recommendation
        if c.pit_lim_mode == "airspeed" and inp.controller.theta_B is None:
            raise ValueError("D.pit_lim_mode 'airspeed' needs controller.ardupilot.Q_BCK_PIT_LIM")
        # the wing-angle band of analysis A: its top is the enforcement cap and the 0.8 C_L,max flag
        lim = zero_loss.ActuatorLimits(alpha_min=0.0 if inp.study.A.alpha_min_deg is None
                                       else math.radians(inp.study.A.alpha_min_deg),
                                       cl_frac=inp.study.A.cl_frac or 0.8)  # fmt: skip
        alpha_cap = zero_loss._alpha_band(vp, lim)[1]  # body pitch; the wing angle adds i_w
        alpha_hi = alpha_cap + vp.wing.i_w
        kappa = inp.descent.kappa
        base_bp = d_exit_params(inp)

        def entry(sc: Scenario) -> float:
            return braking.entry_airspeed(inp.airspeed_min, sc.rho, c.entry_convention)

        Run = tuple[braking.BrakingResult, dict[str, Any], float]
        d_part = ["headline table"]  # progress: the part of D that is running and its run count
        d_runs = [0]

        def run(sc: Scenario, bp: Any, v_air: float) -> Run:
            d_runs[0] += 1
            progress.phase(f"D: {d_part[0]} (run {d_runs[0]})")
            x0, v_ground = braking.airbrake_entry_state(vp, sc, bp, v_air)
            r = braking.measure_a_eq(vp, sc, v_ground, bp, x0=x0)
            fl = braking.braking_flags(
                r.sim, vp, sc, bp, alpha_hi=alpha_hi, alpha_min=lim.alpha_min, kappa=kappa,
                gate=c.alpha_gate_ms, sink_tol=c.sink_tol_ms,
            )  # fmt: skip
            return r, fl, v_ground

        def cells(fl: dict[str, Any], skip: tuple[str, ...] = ()) -> dict[str, Any]:
            return {k: v if isinstance(v, bool) else _num(v)
                    for k, v in fl.items() if k not in skip}  # fmt: skip

        # -- headline table: the named scenarios x mass factors, entry from the airspeed exit rule
        rows: Table = []
        for name in c.scenarios:
            for mf in c.mass_factors:
                sc = dataclasses.replace(inp.scenarios[name], mass_kg=vp.m * mf)
                v_air = entry(sc)
                r, fl, v_ground = run(sc, base_bp, v_air)
                m = r.sim.metrics
                rows.append({
                    "scenario": name, "altitude_m": sc.altitude_amsl_m, "mass_kg": sc.mass_kg,
                    "V0_air_ms": v_air, "V0_ground_ms": v_ground, "a_eq_ms2": _num(r.a_eq),
                    "x_actual_m": _num(r.x_actual), "t_stop_s": r.t_stop,
                    "altitude_excursion_m": r.altitude_excursion,
                    "alt_gain_m": m["altitude_gain"], "alt_loss_m": m["altitude_loss"],
                    "peak_bus_current_A": m["peak_bus_current"],
                    "min_bus_voltage_V": m["min_bus_voltage"],
                    **cells(fl, skip=("min_bus_voltage_V",)),
                })  # fmt: skip
        out["tables"]["D_a_eq"] = rows

        # -- diagnostic: the old speed grid, air speed = TAS of the grid EAS, ground = air - wind
        d_part[0] = "speed grid"
        grid: Table = []
        for name in c.scenarios:
            for mf in c.mass_factors:
                sc = dataclasses.replace(inp.scenarios[name], mass_kg=vp.m * mf)
                for ve in c.v0_eas_ms:
                    v_air = tas(ve, sc)
                    base = {"scenario": name, "altitude_m": sc.altitude_amsl_m,
                            "mass_kg": sc.mass_kg, "V0_eas_ms": ve, "V0_air_ms": v_air,
                            "V0_ground_ms": v_air - sc.headwind_ms}  # fmt: skip
                    try:
                        r, fl, _ = run(sc, base_bp, v_air)
                    except ValueError as exc:  # entry at or below the end speed: no run
                        grid.append({**base, "note": str(exc)})
                        continue
                    grid.append({**base, "a_eq_ms2": _num(r.a_eq), "x_actual_m": _num(r.x_actual),
                                 **cells(fl)})  # fmt: skip
        out["tables"]["D_a_eq_speed"] = grid

        # -- recommendation: Sobol ensemble + corners, entry fixed in air speed per scenario
        d_part[0] = "recommendation ensemble"

        def ensemble(bp: Any) -> tuple[Table, list[float], list[float]]:
            core, corners = braking_ensemble(rec.ensemble_n, vp, rec.ensemble_seed,
                                             inp.ensemble_axes, rec.corners)  # fmt: skip
            erows: Table = []
            for kind, scens in (("sobol", core), ("corner", corners)):
                for s in scens:
                    v_air = entry(s)
                    vs = aero.stall_speed(s.mass_kg or vp.m, s.rho, vp.wing)
                    r, fl, v_ground = run(s, bp, v_air)
                    erows.append({
                        "set": kind, "altitude_m": s.altitude_amsl_m,
                        "isa_offset_k": s.isa_offset_k, "mass_kg": s.mass_kg, "soc": s.soc,
                        "headwind_ms": s.headwind_ms,
                        "V0_air_ms": v_air, "V0_ground_ms": v_ground, "Vs_ms": vs,
                        "below_vs": v_air < vs, "a_eq_ms2": _num(r.a_eq), **cells(fl),
                    })  # fmt: skip
            done = [x for x in erows if x["a_eq_ms2"] != ""]
            core_a = [float(x["a_eq_ms2"]) for x in done if x["set"] == "sobol"]
            return erows, core_a, [float(x["a_eq_ms2"]) for x in done]

        erows, core_a, all_a = ensemble(base_bp)
        out["tables"]["D_recommendation_ensemble"] = erows
        sobol = [x for x in erows if x["set"] == "sobol"]
        corner = [x for x in erows if x["set"] == "corner"]
        if not core_a:
            raise ValueError("D: no ensemble run reached the end speed")
        a_min = min(all_a)
        pct, lo, hi = bootstrap_percentile(core_a, rec.percentile, rec.n_boot, rec.ensemble_seed)
        half = [float(x["a_eq_ms2"]) for x in sobol[: len(sobol) // 2] if x["a_eq_ms2"] != ""]

        def where(x: dict[str, Any]) -> str:
            return (f"alt {x['altitude_m']:.0f} m, mass {x['mass_kg']:.1f} kg, headwind "
                    f"{x['headwind_ms']:+.1f} m/s, ISA {x['isa_offset_k']:+.1f} K, "
                    f"SoC {x['soc']:.2f}")

        def by_a(x: dict[str, Any]) -> float:
            return float(x["a_eq_ms2"])

        ok = [x for x in erows if x["a_eq_ms2"] != ""]
        ok_corner = [x for x in corner if x["a_eq_ms2"] != ""]
        spear: dict[str, float | str] = {}
        for ax, key in (("altitude", "altitude_m"), ("isa", "isa_offset_k"), ("mass", "mass_kg"),
                        ("headwind", "headwind_ms"), ("soc", "soc")):
            xs = [float(x[key]) for x in sobol if x["a_eq_ms2"] != ""]
            ys = [float(x["a_eq_ms2"]) for x in sobol if x["a_eq_ms2"] != ""]
            varies = len(set(xs)) > 1 and len(set(ys)) > 1
            rho_s = float(stats.spearmanr(xs, ys).statistic) if varies else math.nan
            spear[f"spearman_{ax}"] = _num(rho_s)
        gs = [float(x["V0_ground_ms"]) for x in erows]
        hw = [float(x["headwind_ms"]) for x in erows]
        va = [float(x["V0_air_ms"]) for x in erows]
        v_lo, v_hi = min(va), max(va)
        air_txt = f"{v_lo:.1f}" if v_hi - v_lo < 0.05 else f"{v_lo:.1f} to {v_hi:.1f}"
        covers = (f"ground speeds {min(gs):.1f} to {max(gs):.1f} m/s (air speed {air_txt}, "
                  f"headwind {min(hw):+.1f} to {max(hw):+.1f})")  # fmt: skip
        out["tables"]["D_recommendation"] = [{
            "n": rec.ensemble_n, "seed": rec.ensemble_seed, "n_corners": len(corner),
            "convention": c.entry_convention, "percentile": rec.percentile,
            "a_rec_min_ms2": a_min, "a_rec_pct_ms2": pct, "a_rec_pct_ci_lo_ms2": lo,
            "a_rec_pct_ci_hi_ms2": hi,
            "a_rec_pct_half_ms2": float(np.percentile(half, rec.percentile)) if half else "",
            "n_below_vs": sum(1 for x in sobol if x["below_vs"]),
            "n_nan": sum(1 for x in erows if x["a_eq_ms2"] == ""),
            "min_set_by": where(min(ok, key=by_a)),
            "worst_corner": where(min(ok_corner, key=by_a)) if ok_corner else "",
            **spear,
            "ground_speed_min_ms": min(gs), "ground_speed_max_ms": max(gs),
            "headwind_min_ms": min(hw), "headwind_max_ms": max(hw), "covers": covers,
            "a_rec_design_ms2": "",
        }]  # fmt: skip
        out["a_rec"], out["a_rec_pct"], out["pct"] = a_min, pct, rec.percentile
        out["a_rec_ci"] = (lo, hi)

        # -- a_rec as a curve over the ground speed at the AIRBRAKE exit: the air speed is the
        # entry rule's, so the headwind is the air speed minus the ground speed (a negative value
        # is a tailwind); every named scenario x mass factor runs at each speed
        curve: Table = []
        d_part[0] = "a_rec curve"
        for vg in c.curve.ground_speeds_ms:
            done: list[tuple[str, float, float, braking.BrakingResult, dict[str, Any]]] = []
            n_runs = 0
            for name in c.scenarios:
                for mf in c.mass_factors:
                    sc = dataclasses.replace(inp.scenarios[name], mass_kg=vp.m * mf)
                    v_air = entry(sc)
                    sc = dataclasses.replace(sc, headwind_ms=v_air - vg)
                    r, fl, _ = run(sc, base_bp, v_air)
                    n_runs += 1
                    if not math.isnan(r.a_eq):
                        done.append((f"{name} x{mf:g}", sc.headwind_ms, v_air, r, fl))
            row: dict[str, Any] = {"ground_speed_ms": vg, "n_runs": n_runs,
                                   "n_nan": n_runs - len(done)}  # fmt: skip
            if not done:
                curve.append({**row, "note": "no run reached the end speed"})
                continue
            a_all = [d[3].a_eq for d in done]
            who, hw_min, v_air_min, r_min, _ = min(done, key=lambda d: d[3].a_eq)
            curve.append({
                **row, "headwind_ms": hw_min, "V0_air_ms": v_air_min, "a_rec_min_ms2": r_min.a_eq,
                "set_by": who, "a_eq_median_ms2": float(np.median(a_all)),
                "a_eq_max_ms2": max(a_all), "x_at_min_m": _num(r_min.x_actual),
                "t_stop_at_min_s": r_min.t_stop,
                "excursion_max_m": max(max(d[3].sim.metrics["altitude_gain"],
                                           d[3].sim.metrics["altitude_loss"]) for d in done),
                "n_sinking": sum(1 for d in done if d[4]["sinking"]), "note": "",
            })  # fmt: skip
        out["tables"]["D_a_rec_curve"] = curve
        design = rec.design_ground_speed_ms
        if design is not None:
            have = [r for r in curve if r.get("a_rec_min_ms2", "") != ""]
            out["tables"]["D_recommendation"][0]["a_rec_design_ms2"] = braking.curve_value(
                [float(r["ground_speed_ms"]) for r in have],
                [float(r["a_rec_min_ms2"]) for r in have], design)  # fmt: skip

        # -- trade curve: first named scenario, entry from the airspeed rule, ground = air - wind
        sc0 = inp.scenarios[c.scenarios[0]]
        v_air0 = entry(sc0)
        trade: Table = []
        d_part[0] = "trade curve"
        for a_plan in c.trade.a_plan_ms2:
            for wind in c.trade.headwinds_ms:
                for test in ("ground", "air") if wind > 0 else ("ground",):
                    progress.phase(f"D: trade curve (a_plan {a_plan:g}, headwind {wind:g}, {test})")
                    for a_lim in sorted({base_bp.a_lim, a_plan}):
                        scw = dataclasses.replace(sc0, headwind_ms=wind)
                        bpw = dataclasses.replace(base_bp, a_lim=a_lim)
                        x0, v0g = braking.airbrake_entry_state(vp, scw, bpw, v_air0)
                        r = braking.landing_error(vp, scw, v0g, a_plan, bpw, speed_test=test, x0=x0)
                        v_ref = v0g if test == "ground" else v_air0
                        trade.append({
                            "a_plan_ms2": a_plan, "a_lim_ms2": a_lim, "speed_test": test,
                            "headwind_ms": wind, "V0_air_ms": v_air0, "V0_ground_ms": v0g,
                            "x_plan_m": v_ref**2 / (2.0 * a_plan),
                            "x_actual_m": _num(r.x_actual),
                            "overshoot_m": "" if r.overshoot is None else r.overshoot,
                            "energy_J": r.sim.metrics["energy_J"],
                            "altitude_excursion_m": r.altitude_excursion,
                        })  # fmt: skip
        out["tables"]["D_trade_curve"] = trade

        # -- sensitivity: first named scenario at nominal mass, one change at a time
        scn = dataclasses.replace(sc0, mass_kg=vp.m)
        v_air_n = entry(scn)
        deg = math.radians
        cases: list[tuple[str, dict[str, Any]]] = [("base", {})]
        cases += [(f"theta_fw0_{d:g}deg", {"theta_fw0": deg(d)})
                  for d in c.sens.theta_fw0_deg_sweep]  # fmt: skip
        cases += [(f"rotor_lag_{s:g}s", {"ff_mode": "lift_aware", "rotor_lag_s": s})
                  for s in c.sens.lag_s_sweep]  # fmt: skip
        cases += [("ff_mode_repo", {"ff_mode": "repo"})]
        cases += [(f"lift_filter_{s:g}s", {"ff_mode": "lift_aware", "ff_lift_tau_s": s})
                  for s in c.sens.lift_tau_s_sweep]  # fmt: skip
        cases += [(f"lift_scale_{s:g}", {"ff_mode": "lift_aware", "ff_lift_scale": s})
                  for s in c.sens.lift_scale_sweep]  # fmt: skip
        if inp.controller.theta_B is not None:
            cases += [("pit_lim_airspeed", {"pit_lim_mode": "airspeed"})]
        cases += [("enforce_alpha", {"alpha_cap": alpha_cap})]
        with_ensemble = {"base", "ff_mode_repo", "enforce_alpha"}
        sens_rows: Table = []
        # the vehicle model's own rotor lag at this density and SoC; rotor_lag_s adds in series
        tau_model = braking.rotor_speed_tau(scn.vehicle_for(vp), scn.rho, scn.soc)
        d_part[0] = "sensitivities"
        for case, over in cases:
            bps = dataclasses.replace(base_bp, **over)
            r, fl, _ = run(scn, bps, v_air_n)
            m = r.sim.metrics
            srow: dict[str, Any] = {
                "case": case, "a_eq_ms2": _num(r.a_eq), "x_actual_m": _num(r.x_actual),
                "alt_gain_m": m["altitude_gain"], "alt_loss_m": m["altitude_loss"],
                **cells({k: fl[k] for k in ("vh_end_ms", "alpha_max_gated_deg",
                                            "t_above_alpha_08_s", "t_above_alpha_s_s")}),
                "a_rec_pct_ensemble_ms2": "", "rotor_lag_s": bps.rotor_lag_s,
                "tau_rotor_model_s": tau_model, "tau_total_s": bps.rotor_lag_s + tau_model,
            }  # fmt: skip
            if case in with_ensemble:
                _, ca, _ = ensemble(bps)
                if ca:
                    srow["a_rec_pct_ensemble_ms2"] = float(np.percentile(ca, rec.percentile))
            sens_rows.append(srow)
        out["tables"]["D_sensitivity"] = sens_rows

    def stage_f() -> None:
        rows: Table = []
        for name in inp.study.F.scenarios:
            sc = inp.scenarios[name]
            p = descent.descent_profile(sc.vehicle_for(vp), sc, inp.mission.h_bt_agl_m,
                                        inp.descent)  # fmt: skip
            rows.append({"scenario": name, "altitude_m": sc.altitude_amsl_m, "v_h_ms": p.v_h,
                         "v_cap_ms": p.v_cap, "binding_limit": p.binding,
                         "descent_time_s": p.time, "hover_power_W": p.hover_power,
                         "hover_energy_J": p.energy_J, "kappa_source": p.kappa_source})  # fmt: skip
            out["tables"].setdefault("F_profile", []).extend(
                {"scenario": name, "h_m": float(h), "descent_rate_ms": float(v),
                 "time_to_go_s": float(t), "energy_to_go_J": float(e)}
                for h, v, t, e in zip(p.h[::20], p.v[::20], p.t_to_go[::20], p.e_to_go[::20],
                                      strict=True)
            )  # fmt: skip
        out["tables"]["F_descent"] = rows

    def stage_e() -> None:
        ec, q2c = inp.study.E, inp.study.Q2
        a_rec = out.get("a_rec")
        if a_rec is None or not math.isfinite(a_rec):
            a_plan, a_source = ec.a_plan_fallback_ms2, "fallback"
            errors.append(f"E: no recommendation from D, a_plan = {a_plan:g} m/s2 (fallback)")
        else:
            a_plan, a_source = float(a_rec), "D"
        # the global worst of the Q2 grid, for `worst` mode and as a reference column
        worst = math.nan
        if ec.a_fwd_mode == "worst" or "Q2" in wanted:
            w_f = q2_worst(q2_grid([q2c.feed_convention]), q2c.feed_convention)[1]
            worst = w_f.a_min if ec.a_fwd_metric == "min" else w_f.a_equiv
        tabs = out["tables"]
        budgets: dict[str, Any] = {}
        ranges = sorted({*ec.ranges_km, *inp.requirements_km})
        window = inp.window
        for site in ec.sites_amsl_m:
            for kind in ec.missions:
                name = f"{site:.0f}m_{kind}"
                try:
                    cs = solve_e_case(inp, site, kind, a_plan, worst)
                except (ValueError, RuntimeError) as exc:
                    errors.append(f"E {name}: {exc}")
                    tabs.setdefault("E_summary", []).append(
                        {"site_amsl_m": site, "mission": kind, "error": str(exc)})
                    continue
                mb, r = cs.mb, cs.r_max
                budgets[name] = mb
                mp_site = site_mission(inp, site)
                rho_c = float(atmosphere.density(mp_site.cruise_amsl_m, mp_site.isa_offset_k))
                v_c = mb.profiles[mb.legs[0][0]].v_cruise or math.nan
                reqs = {f"{x:g}": x * 1e3 for x in inp.requirements_km}
                summ = soc_trace.summarise(cs.checks)
                row: dict[str, Any] = {
                    "site_amsl_m": site, "mission": kind, "cruise_amsl_m": mp_site.cruise_amsl_m,
                    "v_cruise_tas_ms": v_c, "v_cruise_eas_ms": v_c * math.sqrt(rho_c / RHO0),
                    "max_range_km": r / 1e3,
                    "flown_total_km": r * len(mb.legs) / 1e3,
                    "fixed_Wh": mb.fixed_J / 3600.0,
                    **{f"fixed_{n}_Wh": b.fixed_J / 3600.0 for n, _, b in mb.legs},
                    "cruise_Wh": (mb.total_J(r) - mb.fixed_J - mb.reserve_J - mb.drop_J) / 3600.0,
                    "reserve_Wh": mb.reserve_J / 3600.0,
                    "reserve_binding_leg": mb.notes["reserve_binding_leg"],
                    "drop_Wh": mb.drop_J / 3600.0, "usable_Wh": mb.usable_J / 3600.0,
                    "soc_start": window.soc_start, "soc_end_before_reserve": cs.soc_before,
                    "soc_end_after_reserve": cs.soc_after,
                    "window_ok": cs.soc_after >= window.soc_eod - 1e-9, **summ,
                    "a_fwd_mode": ec.a_fwd_mode,
                    "a_fwd_out_ms2": cs.a_fwd.get("out", cs.a_fwd.get("leg")),
                    "a_fwd_ret_ms2": cs.a_fwd.get("ret", ""), "a_fwd_worst_ms2": _num(worst),
                    "a_plan_ms2": a_plan, "a_plan_source": a_source,
                }  # fmt: skip
                for lab, rm in reqs.items():
                    ok = rm >= mb.fixed_distance_m
                    row[f"margin_Wh_at_{lab}km"] = mb.margin_J(rm) / 3600.0 if ok else ""
                    row[f"conflict_{lab}km"] = mb.max_range_m < rm
                tabs.setdefault("E_summary", []).append(row)
                ok_ranges = [x * 1e3 for x in ranges if x * 1e3 >= mb.fixed_distance_m]
                for brow in mb.table(ok_ranges):
                    tabs.setdefault("E_budget", []).append(
                        {"site_amsl_m": site, "mission": kind, **brow})
                by_leg = {n: (m, b) for n, m, b in mb.legs}
                for chk in cs.checks:
                    mass, b = by_leg[chk.leg]
                    if chk.segment == "cruise":
                        dist = r - b.fixed_distance_m
                        t_s, src = dist / (v_c or math.nan), "level trim, pusher model"
                    else:
                        sr = b.segments[chk.segment]
                        dist, t_s, src = sr.distance_m, sr.time_s, sr.source
                    tabs.setdefault("E_segments", []).append({
                        "site_amsl_m": site, "mission": kind, "leg": chk.leg,
                        "segment": chk.segment,
                        "mass_kg": mass, "energy_Wh": chk.energy_Wh, "time_s": t_s,
                        "distance_m": dist, "source": src, "soc_start": chk.soc_start,
                        "soc_end": chk.soc_end, "feasible": chk.feasible, "limit": chk.limit,
                        "pusher_cmd_max": _num(chk.pusher_cmd_max),
                        "rotor_cmd_max": _num(chk.rotor_cmd_max),
                        "over_ceiling": chk.over_ceiling,
                        "roc_at_ceiling_ms": _num(chk.roc_at_ceiling_ms),
                    })  # fmt: skip
                last = mb.legs[-1][0]
                land_start = next(x.soc_start for x in reversed(cs.rows)
                                  if x.leg == last and x.segment == "landing")  # fmt: skip
                for point, soc in (("stress (soc_eod)", window.soc_eod),
                                   ("landing start", land_start),
                                   ("after reserve", max(cs.soc_after, 0.0))):  # fmt: skip
                    tabs.setdefault("E_hover_check", []).append({
                        "site_amsl_m": site, "mission": kind, "point": point, "soc": soc,
                        **energy.hover_check(mb.vehicles[last], mb.profiles[last], window, soc),
                    })  # fmt: skip
        out["budgets"] = budgets
        first = next(iter(budgets.values()), None)
        if first is not None:
            binding = str(first.notes["reserve_binding_leg"])
            tabs["E_reserve"] = [{"component": k, "energy_Wh": v / 3600.0}
                                 for k, v in first.reserve_parts[binding].items()]  # fmt: skip
        for name in inp.study.F.scenarios:
            sc = inp.scenarios[name]
            speeds = np.linspace(1.1 * energy.stall_speed_1g(sc.vehicle_for(vp), sc.rho), 40.0, 30)
            sw = energy.cruise_sweep(sc.vehicle_for(vp), sc.rho, inp.mission.soc_mean, speeds)
            for v, pw, j in zip(sw["V"], sw["power_W"], sw["J_per_m"], strict=True):
                tabs.setdefault("E_cruise_sweep", []).append({
                    "scenario": name, "altitude_m": sc.altitude_amsl_m, "V_tas_ms": float(v),
                    "power_W": float(pw), "Wh_per_km": float(j) * 1e3 / 3600.0})  # fmt: skip

    def stage_q2() -> None:
        c = inp.study.Q2
        cells = q2_grid(list(c.conventions))
        out["tables"]["Q2_pusher_margin"] = [row for row, _ in cells]
        out["tables"]["Q2_worst"] = [q2_worst(cells, conv)[0] for conv in c.conventions]
        if c.write_profile:
            out["tables"]["Q2_profile"] = [
                {"convention": row["convention"], "altitude_m": row["altitude_m"],
                 "mass_kg": row["mass_kg"], "soc": row["soc"], "V_tas_ms": float(v),
                 "a_ms2": float(a)}
                for row, f in cells for v, a in zip(f.V, f.a, strict=True)
            ]  # fmt: skip

    if "A" in wanted:
        _attempt("A zero-loss feasibility", stage_a, errors)
    if "D" in wanted:
        _attempt("D braking and Q_TRANS_DECEL", stage_d, errors)
    if "F" in wanted:
        _attempt("F descent", stage_f, errors)
    if "Q2" in wanted:
        _attempt("Q2 pusher acceleration", stage_q2, errors)
    if "E" in wanted:
        _attempt("E energy budget", stage_e, errors)
    return out


# ---- tasks: C, FT, B1, sensitivity -------------------------------------------------------------
def c_settings(inp: StudyInputs) -> tuple[restricted.CEval, BackTransitionParams, CostWeights]:
    """C's evaluation settings, base controller parameters and cost normalisers from the config.

    The entry (`theta_fw0_deg`, `pit_lim_mode`, `entry_convention`) is read
    from the D block: it describes the AIRBRAKE exit, not C.
    """
    c, d = inp.study.C, inp.study.D
    if d.pit_lim_mode == "airspeed" and inp.controller.theta_B is None:
        raise ValueError("D.pit_lim_mode 'airspeed' needs controller.ardupilot.Q_BCK_PIT_LIM")
    z = c.z
    base = exit_params(
        inp, rotor_lag_s=c.rotor_lag_s, z_mode="ardupilot", z_pos_p=z.pos_p, z_vel_p=z.vel_p,
        z_acc_p=z.acc_p, z_acc_i=z.acc_i, z_acc_imax=z.acc_imax, z_vmax=z.vmax,
        z_vel_flt_hz=z.vel_flt_hz, z_acc_flt_hz=z.acc_flt_hz,
    )  # fmt: skip
    cev = restricted.CEval(
        airspeed_min=inp.airspeed_min, entry_convention=d.entry_convention,
        rotor_start=c.rotor_start, os_tol_m=c.os_tol_m, sink_tol_ms=c.sink_tol_ms,
        end_speed=END_SPEED_MS,
    )  # fmt: skip
    w = CostWeights(h_ref=c.weights.h_ref, E_ref=c.weights.E_ref, x_ref=c.weights.x_ref)
    return cev, base, w


def alpha_limits(inp: StudyInputs) -> tuple[zero_loss.ActuatorLimits, float]:
    """The wing-angle band of analysis A and the wing angle at its top (D's 0.8 C_L,max flag)."""
    vp = inp.vehicle
    lim = zero_loss.ActuatorLimits(
        alpha_min=0.0 if inp.study.A.alpha_min_deg is None
        else math.radians(inp.study.A.alpha_min_deg),
        cl_frac=inp.study.A.cl_frac or 0.8,
    )  # fmt: skip
    return lim, zero_loss._alpha_band(vp, lim)[1] + vp.wing.i_w


def b1_ceiling_deg(inp: StudyInputs, variant: Any) -> float:
    """The pitch-command ceiling of a B1 variant in degrees; `null` in the config is the
    register's PTCH_LIM_MAX_DEG, so the `c_envelope` variant follows it."""
    if variant.theta_cmd_max_deg is None:
        return math.degrees(inp.theta_p)
    return float(variant.theta_cmd_max_deg)


def c_theta_a_hi_deg(inp: StudyInputs) -> float:
    """Upper bound of `Q_A_ANGLE_MAX` in degrees: `min(C.bounds.theta_A_deg[1], PTCH_LIM_MAX_DEG)`.

    The pitch cap is `min(Q_A_ANGLE_MAX, PTCH_LIM_MAX_DEG)`, so an angle above the pitch limit
    changes nothing and the search would wander on a flat region. The 30 deg of DEC-7 is now
    only the pitch limit when `PTCH_LIM_MAX_DEG` is the smaller.
    """
    lo, hi = inp.study.C.bounds.theta_A_deg
    eff = min(hi, math.degrees(inp.theta_p))
    if eff <= lo:
        raise ValueError(f"C.bounds.theta_A_deg {lo:g} is not below PTCH_LIM_MAX_DEG "
                         f"({math.degrees(inp.theta_p):g} deg): nothing left to search")
    return eff


def c_bounds(inp: StudyInputs) -> Bounds:
    b = inp.study.C.bounds
    return Bounds(
        restricted.C_BOUNDS.names,
        (b.T_bt[0], math.radians(b.theta_A_deg[0]), b.spin_min[0], b.q[0]),
        (b.T_bt[1], math.radians(c_theta_a_hi_deg(inp)), b.spin_min[1], b.q[1]),
    )  # fmt: skip


def worst_end_sink(results: list[restricted.CRun]) -> float:
    """The largest sink rate at the stop over the members that stopped (NaN if none did)."""
    sinks = [-r.vh_end for r in results if r.stopped and math.isfinite(r.vh_end)]
    return max(sinks) if sinks else math.nan


def failure_counts(results: list[restricted.CRun]) -> dict[str, float]:
    """Members with a failure and failures by type, every key always present (zeros filled).

    `n_overshoot` and `n_sinking` count the C-specific failures, `n_not_stopped` the runs that did
    not stop; `n_other_flags` (from `summarise_c`) is everything that is neither overshoot nor
    sinking and so includes `n_not_stopped`.
    """
    fails = [r.metrics.failures for r in results]
    return {
        "n_failing_members": float(sum(1 for f in fails if f)),
        "n_not_stopped": float(sum("not_stopped" in f for f in fails)),
        "n_fail_total": float(sum(len(f) for f in fails)),
    }


def smooth_cost(j: float, n_fail_total: float, w: CostWeights) -> float:
    """J without the failure penalty: the part of (10.1) that moves smoothly with the setting."""
    return j - w.penalty * n_fail_total


def _c_row(label: str, results: list[restricted.CRun], w: CostWeights) -> dict[str, Any]:
    s = restricted.summarise_c(results, w)
    cnt = failure_counts(results)
    return {"case": label, **{k: _num(v) for k, v in s.items()}, **cnt,
            "J_smooth": _num(smooth_cost(s["J"], cnt["n_fail_total"], w)),
            "max_end_sink_ms": _num(worst_end_sink(results))}  # fmt: skip


def _is_num(x: Any) -> bool:
    return isinstance(x, int | float) and not isinstance(x, bool) and math.isfinite(x)


def margin_cols(c: Any, max_overshoot: Any, max_end_sink: Any) -> dict[str, Any]:
    """The tolerances, the margins to them and `at_edge` for one case (reporting only).

    A margin is the tolerance minus the worst value over the members; `at_edge` is set when the
    overshoot margin is below `C.margin_flag_m` or the end-sink margin below `C.margin_flag_ms`.
    An empty cell (no member stopped) has no margin.
    """
    os_m = c.os_tol_m - max_overshoot if _is_num(max_overshoot) else ""
    sk_m = c.sink_tol_ms - max_end_sink if _is_num(max_end_sink) else ""
    edge = (_is_num(os_m) and os_m < c.margin_flag_m) or (_is_num(sk_m) and sk_m < c.margin_flag_ms)
    return {"os_tol_m": c.os_tol_m, "sink_tol_ms": c.sink_tol_ms,
            "max_end_sink_ms": max_end_sink if _is_num(max_end_sink) else "",
            "os_margin_m": os_m, "sink_margin_ms": sk_m, "at_edge": bool(edge)}  # fmt: skip


def pick_margin_row(fine: Table, c: Any) -> dict[str, Any] | None:
    """The `optimum_margin` q: the largest q on the fine grid whose worst overshoot is at most
    `margin_frac` x `os_tol_m` AND whose worst end sink is at most `margin_frac` x `sink_tol_ms`.
    None if no q on the grid does (a row with no number would only look like a result)."""
    ok_rows = [
        r for r in fine
        if _is_num(r.get("q")) and _is_num(r.get("max_overshoot_m"))
        and r["max_overshoot_m"] <= c.margin_frac * c.os_tol_m
        and (not _is_num(r.get("max_end_sink_ms"))
             or r["max_end_sink_ms"] <= c.margin_frac * c.sink_tol_ms)
    ]  # fmt: skip
    return max(ok_rows, key=lambda r: r["q"]) if ok_rows else None


def default_case(
    inp: StudyInputs, names: dict[str, str], convert: dict[str, Any] | None = None
) -> tuple[dict[str, float] | None, str]:
    """The parameters of an analysis's `ardupilot_default` case, from the defaults register.

    `names` maps the analysis's variable to the ArduPilot name. Returns (None, note) when any
    default is missing: the analysis then skips the case and says so (never a made-up value).
    `convert` turns an SI default into the variable's own unit (FT: radians to degrees).
    """
    out: dict[str, float] = {}
    missing: list[str] = []
    for var, name in names.items():
        v = inp.resolve_default(name)
        if v is None:
            missing.append(name)
        else:
            out[var] = (convert or {}).get(var, lambda x: x)(v)
    if missing:
        return None, ("ArduPilot default case not run: no default for " + ", ".join(missing)
                      + " in ardupilot_defaults.yaml")  # fmt: skip
    return out, ""


def _sample_indices(n: int, every: int = 5) -> list[int]:
    """Every `every`-th sample and always the last one, so a curve ends where the run stopped."""
    idx = list(range(0, n, every))
    if idx and idx[-1] != n - 1:
        idx.append(n - 1)
    return idx


def _hist_meta(
    view: str, name: str, scen: Scenario, vp: VehicleParams, run: restricted.CRun, band: float,
    **extra: Any,
) -> dict[str, Any]:  # fmt: skip
    """What the caption of a history figure states: the scenario and the entry speed."""
    return {
        "view": view, "scenario": name, "altitude_m": scen.altitude_amsl_m,
        "isa_offset_k": scen.isa_offset_k, "soc": scen.soc, "headwind_ms": scen.headwind_ms,
        "mass_kg": vp.m if scen.mass_kg is None else scen.mass_kg, "V0_tas_ms": run.v_air0,
        "V0_eas_ms": run.v_air0 * math.sqrt(scen.rho / RHO0), "band_eps_h_m": band,
        "run_overshoot_m": _num(run.overshoot), **extra,
    }  # fmt: skip


def _c_trace(
    sim: Any, i_w: float, case: str, member: int, every: int = 5,
    meta: dict[str, Any] | None = None,
) -> Table:  # fmt: skip
    """`C_history` rows of one run: every `every`-th sample (and the last), angles in degrees (as
    B1's series). `meta` is the scenario description every row of the run carries."""
    cols = {"t": sim.t, "x": sim["x"], "h": sim["h"], "V": sim["V"], "v_h": sim["vh"],
            "theta": np.degrees(sim["theta"]), "alpha_w_deg": np.degrees(sim["alpha"] + i_w),
            "delta_r": sim["delta_r"], "theta_cmd": np.degrees(sim["theta_cmd"]),
            "omega_r": sim["omega_r"]}  # fmt: skip
    return [{"case": case, "member": member, **(meta or {}),
             **{k: float(v[i]) for k, v in cols.items()}}
            for i in _sample_indices(len(sim.t), every)]  # fmt: skip


def task_c(inp: StudyInputs) -> dict[str, Any]:
    """Analysis C: search the four ArduPilot variables from the AIRBRAKE exit, then report.

    Tables: `C_params` (the optimum, the register at q = 2 and the ArduPilot default), `C_search`,
    `C_runs` (one row per ensemble member and case, with the capability ceiling and D's alpha and
    VRS checks), `C_q_curve` (the optimum's ramp with q varied), `C_sens` (one-at-a-time
    changes at the optimum) and `C_history` (the time series of member 0 in each
    case). The ArduPilot-default case (docs/DEFAULTS_spec.md 2.2) sets the four searched
    parameters to their defaults and keeps the register's entry (`AIRSPEED_MIN`), so the back
    transition starts from the same state in all three cases; it is a reference only.
    """
    c, d, vp = inp.study.C, inp.study.D, inp.vehicle
    scens = _ensemble(inp, c.ensemble_n, c.ensemble_seed)
    cev, base, w = c_settings(inp)
    bounds = c_bounds(inp)
    start = {k: inp.start_params[k] for k in ("T_bt", "theta_A", "spin_min")}
    theta_hi = c_theta_a_hi_deg(inp)
    theta_note = (
        f"Q_A_ANGLE_MAX is searched up to {theta_hi:g} deg = min({c.bounds.theta_A_deg[1]:g}, "
        f"PTCH_LIM_MAX_DEG {math.degrees(inp.theta_p):g}): the pitch cap is "
        f"min(Q_A_ANGLE_MAX, PTCH_LIM_MAX_DEG), so "
        + ("the 30 deg limit (DEC-7) is now the pitch limit." if theta_hi < c.bounds.theta_A_deg[1]
           else "the pitch limit does not bind the search.")
    )
    progress.phase("C: searching (CMA-ES)")
    r = restricted.optimise_c(
        vp, scens, cev, base, w, start, bounds=bounds, seeds=tuple(c.seeds),
        popsize=c.popsize, maxiter=c.maxiter,
    )  # fmt: skip
    best = r.params
    reg = {**start, "q": restricted.C_Q_START}
    dflt, dflt_note = default_case(
        inp, {"T_bt": "Q_BACKTRANS_MS", "theta_A": "Q_A_ANGLE_MAX", "spin_min": "Q_M_SPIN_MIN",
              "q": "Q_TRANS_DECEL"}
    )  # fmt: skip

    def run_all(
        prm: dict[str, float], ev: restricted.CEval = cev, bp: BackTransitionParams = base
    ) -> list[restricted.CRun]:
        out: list[restricted.CRun] = []
        for i, s in enumerate(scens, start=1):
            out.append(restricted.run_c(vp, s, prm, ev, bp))
            progress.step(f"C runs: member {i}", i, len(scens))
        return out

    def deg(prm: dict[str, float]) -> dict[str, Any]:
        return {"T_bt_s": prm["T_bt"], "theta_A_deg": math.degrees(prm["theta_A"]),
                "spin_min": prm["spin_min"], "Q_TRANS_DECEL_ms2": prm["q"]}  # fmt: skip

    t0 = time.time()
    progress.phase("C: final ensemble (optimum, register, default)")
    opt = run_all(best)
    reg_runs = run_all(reg)
    s_opt, s_reg = restricted.summarise_c(opt, w), restricted.summarise_c(reg_runs, w)
    dflt_runs = run_all(dflt) if dflt is not None else None
    params: Table = [
        {"case": "optimum", **deg(best), "J": r.cost, "J_start": r.cost_default,
         "seed_spread": r.search.spread,
         "at_bound": ";".join(f"{k}:{v}" for k, v in r.at_bound.items()),
         **{k: _num(v) for k, v in s_opt.items() if k != "J"}},
        {"case": f"register at q = {restricted.C_Q_START:g}", **deg(reg), "J": s_reg["J"],
         "J_start": "", "seed_spread": "", "at_bound": "",
         **{k: _num(v) for k, v in s_reg.items() if k != "J"}},
    ]  # fmt: skip
    if dflt is not None and dflt_runs is not None:
        s_dflt = restricted.summarise_c(dflt_runs, w)
        params.append({
            "case": "ardupilot_default", **deg(dflt), "J": s_dflt["J"], "J_start": "",
            "seed_spread": "", "at_bound": "",
            **{k: _num(v) for k, v in s_dflt.items() if k != "J"},
        })  # fmt: skip
    # each seed's best point in physical units (the stored unit-cube point mapped through the
    # bounds; no search is run), so seeds that agree on J can be told apart by their parameters
    search = []
    for sr in r.search.seeds:
        pt = bounds.to_physical(sr.x)
        search.append({"seed": sr.seed, "best_cost": sr.f, "evaluations": sr.n_evals,
                       "T_bt_s": pt["T_bt"], "theta_A_deg": math.degrees(pt["theta_A"]),
                       "spin_min": pt["spin_min"], "Q_TRANS_DECEL_ms2": pt["q"]})  # fmt: skip
    # what the "at a search bound" chip needs to say where the upper bound of theta_A comes from
    params[0].update({"theta_A_hi_deg": theta_hi, "theta_A_cfg_hi_deg": c.bounds.theta_A_deg[1],
                      "ptch_lim_deg": math.degrees(inp.theta_p),
                      "rotor_lag_s": c.rotor_lag_s})  # fmt: skip

    # per member and case: the run, the capability ceiling, D's alpha and VRS checks (report only)
    progress.phase("C: per-member tables")
    lim, alpha_hi = alpha_limits(inp)
    run_rows: Table = []

    def member_rows(case: str, prm: dict[str, float], results: list[restricted.CRun]) -> None:
        for i, (s, cr) in enumerate(zip(scens, results, strict=True)):
            vps = s.vehicle_for(vp)
            bp_i = dataclasses.replace(base, T_bt=prm["T_bt"], theta_A=prm["theta_A"],
                                       delta_min=prm["spin_min"])  # fmt: skip
            fl = braking.braking_flags(
                cr.sim, vps, s, bp_i, alpha_hi=alpha_hi, alpha_min=lim.alpha_min,
                kappa=inp.descent.kappa, gate=d.alpha_gate_ms, sink_tol=c.sink_tol_ms,
            )  # fmt: skip
            a_cap = restricted.capability_ceiling(vp, s, prm, cev, base)
            progress.step(f"C tables, {case}: member {i}", i + 1, len(scens))
            run_rows.append({
                "case": case, "member": i, "altitude_m": s.altitude_amsl_m,
                "isa_offset_k": s.isa_offset_k, "mass_kg": s.mass_kg, "soc": s.soc,
                "headwind_ms": s.headwind_ms, "V0_air_ms": cr.v_air0,
                "V0_ground_ms": cr.v_ground0, "x_tgt_m": cr.x_tgt,
                "x_end_m": _num(cr.x_end), "overshoot_m": _num(cr.overshoot),
                "energy_kJ": cr.metrics.energy / 1e3, "excursion_m": cr.metrics.excursion,
                "end_sink_ms": _num(-cr.vh_end), "flags": ",".join(cr.metrics.failures),
                "a_cap_ms2": _num(a_cap), "q_over_a_cap": _num(prm["q"] / a_cap),
                **{k: v if isinstance(v, bool) else _num(v) for k, v in fl.items()},
            })  # fmt: skip

    member_rows("optimum", best, opt)
    if dflt is not None and dflt_runs is not None:
        member_rows("ardupilot_default", dflt, dflt_runs)

    progress.phase("C: histories and the q curve")
    # Two views (member = -1 is "not an ensemble member"): the B1 headline scenario with C's
    # optimum and the ArduPilot default on it, and the ensemble member on which the default
    # overshoots most with both cases. Every row of a run says which scenario it is.
    history: Table = []
    band = float(inp.study.B1.eps_h_m[0])
    core_name = inp.study.B1.scenarios[0] if inp.study.B1.scenarios else ""
    hist_notes: list[str] = []
    if core_name in inp.scenarios:
        core = inp.scenarios[core_name]
        try:
            for case, prm in (("C optimum", best), ("C ardupilot_default", dflt)):
                if prm is None:
                    continue
                cr = restricted.run_c(vp, core, prm, cev, base)
                history += _c_trace(cr.sim, vp.wing.i_w, case, -1,
                                    meta=_hist_meta("core", core_name, core, vp, cr, band))
        except (ValueError, RuntimeError) as exc:
            hist_notes.append(f"C on the B1 scenario {core_name} could not be run: {exc}")
    if dflt_runs is not None:
        over = [r.overshoot if math.isfinite(r.overshoot) else -math.inf for r in dflt_runs]
        worst = max(range(len(over)), key=over.__getitem__)
        fails = over[worst] > c.os_tol_m
        for case, runs_i in (("C optimum", opt), ("C ardupilot_default", dflt_runs)):
            history += _c_trace(
                runs_i[worst].sim, vp.wing.i_w, case, worst,
                meta=_hist_meta("default_fails", "ensemble member", scens[worst], vp,
                                runs_i[worst], band, default_fails=bool(fails)))

    q_curve = [_c_row(f"q = {q:g}", run_all({**best, "q": q}), w) | {"q": q, "grid": "coarse"}
               for q in c.q_curve]  # fmt: skip
    # a fine grid around the optimum, the other three variables fixed. q is not clipped to the
    # search bound (it is a report, not a search); only a q that is not positive is dropped.
    q_fine: Table = []
    for off in c.q_fine_offsets_ms2:
        q_f = best["q"] + off
        if q_f <= 0.0:
            continue
        runs_q = opt if off == 0.0 else run_all({**best, "q": q_f})
        q_fine.append(_c_row(f"q = {q_f:.3f}", runs_q, w) | {"q": q_f, "grid": "fine"})
    q_curve += q_fine
    # the tolerances and margins of every case, and the reported margin row (never the
    # recommendation)
    by_case = {"optimum": opt, str(params[1]["case"]): reg_runs}
    if dflt_runs is not None:
        by_case["ardupilot_default"] = dflt_runs
    for prow in params:
        sink_w = worst_end_sink(by_case[prow["case"]])
        prow.update(margin_cols(c, prow.get("max_overshoot_m"), sink_w))
        cnt = failure_counts(by_case[prow["case"]])
        prow.update(cnt)
        prow["J_smooth"] = _num(smooth_cost(float(prow["J"]), cnt["n_fail_total"], w))
    mrow = pick_margin_row(q_fine, c)
    if mrow is not None:
        prm_m = {**best, "q": mrow["q"]}
        params.append({
            "case": "optimum_margin", **deg(prm_m), "J": mrow["J"], "J_start": "",
            "seed_spread": "", "at_bound": "",
            **{k: v for k, v in mrow.items()
               if k not in ("case", "J", "q", "grid", "max_end_sink_ms")},
            **margin_cols(c, mrow["max_overshoot_m"], mrow["max_end_sink_ms"]),
        })  # fmt: skip

    sens_rows: Table = []
    progress.phase("C: one-at-a-time sensitivities")

    def sens_case(name: str, value: Any, ev: restricted.CEval = cev,
                  bp: BackTransitionParams = base) -> None:  # fmt: skip
        row = _c_row("", run_all(best, ev, bp), w)
        sens_rows.append({"parameter": name, "value": value, **row})

    sc_ = c.sens
    for v in sc_.rotor_lag_s:
        sens_case("rotor_lag_s", v, bp=dataclasses.replace(base, rotor_lag_s=v))
    for rs in sc_.rotor_start:
        sens_case("rotor_start", rs, ev=dataclasses.replace(cev, rotor_start=rs))
    for v in sc_.z_vel_p:
        sens_case("z_vel_p", v, bp=dataclasses.replace(base, z_vel_p=v))
    for v in sc_.z_acc_p:
        sens_case("z_acc_p", v, bp=dataclasses.replace(base, z_acc_p=v))
    for v in sc_.theta_fw0_deg:
        sens_case("theta_fw0_deg", v, bp=dataclasses.replace(base, theta_fw0=math.radians(v)))
    for row in sens_rows:
        row.pop("case", None)
    progress.note(f"C reports: {time.time() - t0:.0f} s")
    return {
        "result": r, "scens": scens,
        "notes": [theta_note, *([dflt_note] if dflt_note else []), *hist_notes],
        "tables": {"C_params": params, "C_search": search, "C_runs": run_rows,
                   "C_q_curve": q_curve, "C_sens": sens_rows, "C_history": history},
    }  # fmt: skip


def ft_settings(inp: StudyInputs) -> tuple[forward.FTEval, dict[str, float]]:
    """FT's evaluation settings and the register start (the four searched values).

    The controller template is `inp.forward` (the register plus C's Z-loop gains); the accepted
    loss and gain are the mission profile's; the recovery C_L fraction is `FT.rec_cl_frac` or A's.
    """
    f, m, fp = inp.study.FT, inp.mission, inp.forward
    _, alpha_hi = alpha_limits(inp)
    fev = forward.FTEval(
        template=fp, t_end_s=f.t_end_s, t_settle_s=f.t_settle_s, stall_margin=f.stall_margin,
        cl_frac=f.rec_cl_frac or inp.study.A.cl_frac or 0.8, proj_is_failure=f.proj_is_failure,
        loss_max_m=m.fwd_alt_loss_max_m, gain_max_m=m.fwd_alt_gain_max_m, alpha_hi=alpha_hi,
        alpha_gate_ms=inp.study.D.alpha_gate_ms,
    )  # fmt: skip
    start = {
        "airspeed_min": fp.airspeed_min_eas, "q_transition": fp.q_transition_s,
        "tkoff_thr_max": fp.tkoff_thr_max, "tran_pit_max": fp.tran_pit_max_deg,
    }  # fmt: skip
    return fev, start


def ft_scenarios(
    inp: StudyInputs, n: int | None = None
) -> tuple[list[Scenario], list[Scenario]]:
    """The Sobol sample and the corners, at the transition altitude.

    `n` members of the sample (default `FT.ensemble_n`, the search ensemble). A larger `n` is a
    prefix extension of a smaller one, so the search members are the first members of the final set.
    """
    f = inp.study.FT
    axes = dataclasses.replace(inp.ensemble_axes, headwind_ms=(f.headwind_ms[0], f.headwind_ms[1]))
    return forward.ft_ensemble(
        n or f.ensemble_n, inp.vehicle, f.ensemble_seed, axes, inp.mission.clearance_agl_m
    )


def ft_bounds(inp: StudyInputs, fixed: dict[str, float] | None = None) -> Bounds:
    b = inp.study.FT.bounds
    return forward.ft_bounds(
        b.airspeed_min_eas_ms, b.q_transition_s, b.tkoff_thr_max, b.q_tran_pit_max_deg, fixed
    )


def ft_weights(
    inp: StudyInputs, scens: list[Scenario], fev: forward.FTEval, start: dict[str, float]
) -> CostWeights:
    """The cost of FT; `E_ref: null` is the mean energy at the register start (computed once)."""
    w = inp.study.FT.weights
    e_ref = w.E_ref or forward.mean_energy(inp.vehicle, scens, fev, start)
    return CostWeights(w_h=w.w_h, w_E=w.w_E, w_x=w.w_x, h_ref=w.h_ref, E_ref=e_ref, x_ref=w.x_ref)


def _ft_member_row(s: Scenario, r: forward.FTRun, case: str, kind: str, i: int) -> dict[str, Any]:
    """One `FT_runs` row: the member's axes, then the run's columns."""
    row: dict[str, Any] = {
        "case": case, "set": kind, "member": i, "altitude_m": s.ground_amsl_m,
        "isa_offset_k": s.isa_offset_k, "mass_kg": s.mass_kg, "soc": s.soc,
        "headwind_ms": s.headwind_ms,
    }  # fmt: skip
    for k, v in r.row.items():
        row[k] = v if isinstance(v, bool | str) else _num(v)
    return row


FT_PARAM_COLUMNS = {
    "airspeed_min": "airspeed_min_eas_ms", "q_transition": "q_transition_s",
    "tkoff_thr_max": "tkoff_thr_max", "tran_pit_max": "tran_pit_max_deg",
}  # fmt: skip


def _ft_sens_eval(fev: forward.FTEval, key: str, value: Any) -> forward.FTEval:
    """The evaluation settings with one `FT.sens` entry changed."""
    tp = fev.template
    if key == "rec_cl_frac":  # changes only the projection
        return dataclasses.replace(fev, cl_frac=value)
    if key in ("rotor_lag_s", "z_vel_p", "z_acc_p"):
        back = dataclasses.replace(tp.back, **{key: value})
        return dataclasses.replace(fev, template=dataclasses.replace(tp, back=back))
    own = {"TECS_TIME_CONST": "tecs_time_const", "TECS_INTEG_GAIN": "tecs_integ_gain",
           "TRIM_THROTTLE": "trim_throttle", "tecs_start": "tecs_start"}  # fmt: skip
    return dataclasses.replace(fev, template=dataclasses.replace(tp, **{own[key]: value}))


def vehicle_with_cl_max(vp: VehicleParams, scale: float) -> VehicleParams:
    """The vehicle with its wing's C_L,max scaled, the least invasive way: only `alpha_s` moves.

    The stall angle `alpha_s` is the provisional input of the lift peak (vehicle.yaml); it is found
    by bisection so that `runs.cl_max` is `scale` times the model's value (C_L,max rises with
    alpha_s). The stall speed, the stall flag, the recovery acceleration and the simulated lift all
    read the wing, so they follow. Nothing else of the wing (C_L0, C_L,alpha, C_N, M, drag) changes.
    """
    target = scale * runs.cl_max(vp.wing)
    lo, hi = 0.5 * vp.wing.alpha_s, 2.0 * vp.wing.alpha_s
    for _ in range(40):
        mid = 0.5 * (lo + hi)
        wing = dataclasses.replace(vp.wing, alpha_s=mid)
        if runs.cl_max(wing) < target:
            lo = mid
        else:
            hi = mid
    return dataclasses.replace(vp, wing=dataclasses.replace(vp.wing, alpha_s=0.5 * (lo + hi)))


# ---- FT: the grid over Q_TRANSITION_MS, the snap to ArduPilot's integers, the report ----------
def ft_lattice_bounds(inp: StudyInputs) -> dict[str, tuple[float, float]]:
    """The search bounds of the three lattice variables, by `forward.LATTICE_NAMES`."""
    b = inp.study.FT.bounds
    return {"airspeed_min": b.airspeed_min_eas_ms, "tkoff_thr_max": b.tkoff_thr_max,
            "tran_pit_max": b.q_tran_pit_max_deg}  # fmt: skip


def ft_task_id(prm: dict[str, float]) -> str:
    """`FT:<AIRSPEED_MIN m/s>:<Q_TRANSITION_MS>:<TKOFF_THR_MAX %>:<Q_TRAN_PIT_MAX deg>` of a lattice
    point. The values are in the task id, so they are part of its cache key."""
    return "FT:{}:{}:{}:{}".format(
        round(prm["airspeed_min"]), round(prm["q_transition"] * 1e3),
        round(prm["tkoff_thr_max"] * 100), round(prm["tran_pit_max"]),
    )


def parse_ft_id(arg: str) -> dict[str, float]:
    v, q_ms, thr, pit = (int(x) for x in arg.split(":"))
    return {"airspeed_min": float(v), "q_transition": q_ms / 1e3, "tkoff_thr_max": thr / 100.0,
            "tran_pit_max": float(pit)}  # fmt: skip


def grid_id(q_s: float) -> str:
    return f"FT_grid:{round(q_s * 1e3)}"


def task_ft_grid(inp: StudyInputs, q_ms_arg: str) -> dict[str, Any]:
    """One `FT_grid` point (FT_spec Part 4): Q_TRANSITION_MS fixed, the other three searched.

    The continuous CMA-ES result is snapped to ArduPilot's integers (AIRSPEED_MIN 1 m/s,
    Q_TRAN_PIT_MAX 1 deg, TKOFF_THR_MAX 1 %), then a lattice descent moves to the best in-bound
    3 x 3 x 3 neighbour until none is better, all on the search ensemble. The T-12 spread is the
    three-seed spread of the continuous search. The row carries both results and the cost change.
    """
    f, vp = inp.study.FT, inp.vehicle
    q = int(q_ms_arg) / 1e3
    fev, start = ft_settings(inp)
    core, _ = ft_scenarios(inp)
    w = ft_weights(inp, core, fev, start)
    fixed = {"q_transition": q}
    progress.phase(f"FT_grid {q:g} s: searching (CMA-ES)")
    r = forward.optimise_ft(
        vp, core, fev, w, {**start, **fixed}, ft_bounds(inp, fixed), fixed=fixed,
        seeds=tuple(f.grid.seeds), popsize=f.popsize, maxiter=f.grid.maxiter,
    )  # fmt: skip
    cont = r.params
    progress.phase(f"FT_grid {q:g} s: lattice descent")

    def cost(p: dict[str, float]) -> float:
        return forward.ft_cost(vp, core, fev, {**p, **fixed}, w)

    lat = forward.lattice_descent(
        cost, cont, ft_lattice_bounds(inp), max_rounds=f.grid.descent_max_rounds
    )
    snapped = {**lat.params, **fixed}
    progress.phase(f"FT_grid {q:g} s: summary of the snapped point")
    sm = forward.summarise_ft(forward_runs(vp, core, snapped, fev), w)
    row = {
        "q_transition_s": q,
        **{FT_PARAM_COLUMNS[k] + "_continuous": cont[k] for k in forward.FT_NAMES},
        "J_continuous": r.cost, "seed_spread": r.search.spread,
        "t12_ok": r.search.spread <= T12_SPREAD_MAX,
        "at_bound": ";".join(f"{k}:{x}" for k, x in r.at_bound.items()),
        **{FT_PARAM_COLUMNS[k]: snapped[k] for k in forward.FT_NAMES},
        "J_snap": lat.cost_start, "J_snapped": lat.cost,
        "J_snapped_minus_continuous": lat.cost - r.cost,
        "descent_moves": lat.rounds, "descent_evaluations": lat.evaluations,
        "ensemble_n": len(core), **{k: _num(x) for k, x in sm.items() if k != "J"},
    }  # fmt: skip
    return {"result": r, "row": row, "q_transition_s": q, "snapped": snapped}


def grid_coarse_ids(inp: StudyInputs) -> list[str]:
    return [grid_id(q) for q in inp.study.FT.grid.q_transition_s]


def grid_fine_ids(inp: StudyInputs, results: dict[str, dict[str, Any]]) -> list[str]:
    """The fine pass: the best coarse point (lowest continuous `J_continuous`, the `J_search` of the
    brief) plus the offsets, inside the search bounds, not already on the coarse grid."""
    g = inp.study.FT.grid
    coarse = [(results[i]["row"]["J_continuous"], results[i]["q_transition_s"])
              for i in grid_coarse_ids(inp) if i in results]  # fmt: skip
    if not coarse:
        return []
    q_best = min(coarse)[1]
    lo, hi = inp.study.FT.bounds.q_transition_s
    out = []
    for off in g.fine_offsets_s:
        q = round(q_best + off, 3)
        if lo - 1e-9 <= q <= hi + 1e-9 and q not in g.q_transition_s:
            out.append(grid_id(q))
    return out


def grid_winner(results: dict[str, dict[str, Any]]) -> str | None:
    """The grid task with the lowest snapped cost (`J_snapped`) among those that ran."""
    rows = {t: r["row"]["J_snapped"] for t, r in results.items() if t.startswith("FT_grid:")}
    return min(rows, key=lambda t: (rows[t], t)) if rows else None


def follow_up_tasks(
    inp: StudyInputs, results: dict[str, dict[str, Any]], failed: dict[str, str]
) -> list[str]:
    """Tasks that can only be planned once earlier ones are done (a later phase of the run).

    FT: the fine pass after the coarse grid, then the report of the winner. The task id carries
    the winner's values, so they are part of the cache key.
    """
    out: list[str] = []
    if "FT" not in inp.study.analyses:
        return out
    done = set(results) | set(failed)
    coarse = grid_coarse_ids(inp)
    if not all(i in done for i in coarse):
        return out
    fine = grid_fine_ids(inp, results)
    out += [i for i in fine if i not in done]
    if all(i in done for i in [*coarse, *fine]):
        win = grid_winner(results)
        if win is not None:
            tid = ft_task_id(results[win]["snapped"])
            if tid not in done:
                out.append(tid)
    # the recommended settings, once the FT report exists: E and Q2 are repeated at them
    ft_done = next((t for t in results if t.startswith("FT:")), None)
    if ft_done is not None and set(inp.study.analyses) & {"E", "Q2"}:
        rec = rec_task_id(parse_ft_id(ft_done.split(":", 1)[1]))
        if rec not in done:
            out.append(rec)
    return out


def task_ft(inp: StudyInputs, arg: str) -> dict[str, Any]:
    """Analysis FT, the report of the grid winner `arg` (see `ft_task_id`).

    Tables: `FT_params` (the optimum, the register start and the ArduPilot default), `FT_runs`
    (every member of each case, Sobol core and corners), `FT_sens` (one at a time at the optimum)
    and `FT_history` (time series of the named scenarios, each case). The sensitivities, `base` and
    `J_start` use the first `FT.ensemble_n` members; the three cases of `FT_params`, `FT_runs` and
    `FT_history` are evaluated on the first `FT.final_ensemble_n` (`ensemble_n` columns say which,
    and the notes repeat it). The `ardupilot_default` case (docs/DEFAULTS_spec.md 2.1) sets the
    four variables to their defaults, everything else to the register; no search is run for it.
    The search itself is `task_ft_grid`; `assemble` joins its row to the optimum's.
    """
    f, vp = inp.study.FT, inp.vehicle
    best = parse_ft_id(arg)
    fev, start = ft_settings(inp)
    core, corners = ft_scenarios(inp)
    t0 = time.time()
    progress.phase("FT report: reference energy and start cost")
    w = ft_weights(inp, core, fev, start)
    j_start = forward.ft_cost(vp, core, fev, start, w)
    progress.note(f"FT E_ref {w.E_ref:.0f} J ({time.time() - t0:.0f} s, {len(core)} runs)")
    t1 = time.time()
    cases = {"optimum": best, "register": start}
    dflt, dflt_note = default_case(
        inp, {"airspeed_min": "AIRSPEED_MIN", "q_transition": "Q_TRANSITION_MS",
              "tkoff_thr_max": "TKOFF_THR_MAX", "tran_pit_max": "Q_TRAN_PIT_MAX"},
        convert={"tran_pit_max": math.degrees},
    )  # fmt: skip
    if dflt is not None:
        cases["ardupilot_default"] = dflt
    final, final_corners = ft_scenarios(inp, f.final_ensemble_n)
    progress.phase("FT report: final ensemble (core and corners, each case)")
    core_runs: dict[str, Any] = {}
    corner_runs: dict[str, Any] = {}
    for i, (c, prm) in enumerate(cases.items(), start=1):
        core_runs[c] = forward_runs(vp, final, prm, fev)
        corner_runs[c] = forward_runs(vp, final_corners, prm, fev)
        progress.step(f"final ensemble, case {c}", i, len(cases))
    params: Table = []
    for c, prm in cases.items():
        opt = c == "optimum"
        params.append({
            "case": c, **{FT_PARAM_COLUMNS[k]: prm[k] for k in forward.FT_NAMES},
            "ensemble_n": len(final), "J_start": j_start if opt else "",
            **{k: _num(v) for k, v in forward.summarise_ft(core_runs[c], w).items()},
        })  # fmt: skip
    run_rows: Table = []
    for c in cases:
        for kind, scens, results in (
            ("core", final, core_runs[c]), ("corner", final_corners, corner_runs[c])
        ):
            for i, (sc, rr) in enumerate(zip(scens, results, strict=True)):
                run_rows.append(_ft_member_row(sc, rr, c, kind, i))
    notes = [
        "FT_params, FT_runs and FT_history evaluate the optimum, the register start and the "
        f"ArduPilot default on the first {len(final)} members of the Sobol sample and the "
        f"{len(final_corners)} corners. The grid search, FT_grid, FT_sens and J_start use the "
        f"first {len(core)} members. The optimum is the best snapped point of FT_grid: whole m/s, "
        "whole "
        "degrees, whole percent and whole milliseconds, the values ArduPilot can take."
    ]
    # one at a time at the optimum, on the search ensemble; `base` is the unchanged evaluation there
    sens_rows: Table = []
    progress.phase("FT report: one-at-a-time sensitivities")
    n_sens = sum(len(v) for v in f.sens.model_dump().values())
    base_sm = forward.summarise_ft(forward_runs(vp, core, best, fev), w)
    sens_rows.append({"parameter": "base", "value": "", "ensemble_n": len(core),
                      "cl_max": runs.cl_max(vp.wing), "alpha_s_rad": vp.wing.alpha_s,
                      **{k: _num(x) for k, x in base_sm.items()}})  # fmt: skip
    for key, values in f.sens.model_dump().items():
        for v in values:
            if key == "CL_max_scale":  # a different wing: the stall flag reads the changed C_L,max
                vp_s, fev_s = vehicle_with_cl_max(vp, v), fev
            else:
                vp_s, fev_s = vp, _ft_sens_eval(fev, key, v)
            sm = forward.summarise_ft(forward_runs(vp_s, core, best, fev_s), w)
            sens_rows.append({"parameter": key, "value": v, "ensemble_n": len(core),
                              "cl_max": runs.cl_max(vp_s.wing), "alpha_s_rad": vp_s.wing.alpha_s,
                              **{k: _num(x) for k, x in sm.items()}})  # fmt: skip
            progress.step(f"sensitivity {key}={v}", len(sens_rows) - 1, n_sens)
    progress.phase("FT report: histories")
    hist: Table = []
    for name in f.history_scenarios:
        sc = forward.ft_scenario(inp.scenarios[name], inp.mission.clearance_agl_m)
        for c, prm in cases.items():
            rr = forward.run_ft(vp, sc, prm, fev)
            hist += forward.history_rows(sc.vehicle_for(vp), rr, fev.template, name, c)
    progress.note(f"FT reports: {time.time() - t1:.0f} s")
    return {
        "optimum": best, "J_start": j_start, "scens": core, "E_ref": w.E_ref,
        "notes": [*notes, *([dflt_note] if dflt_note else [])],
        "tables": {"FT_params": params, "FT_runs": run_rows, "FT_sens": sens_rows,
                   "FT_history": hist},
    }  # fmt: skip


def with_settings(
    inp: StudyInputs, airspeed_min: float, tkoff_thr_max: float, q_transition_s: float | None = None
) -> StudyInputs:
    """The inputs with `AIRSPEED_MIN`, `TKOFF_THR_MAX` and (if given) `Q_TRANSITION_MS` replaced
    everywhere they are read."""
    q = inp.start_params["q_transition"] if q_transition_s is None else q_transition_s
    return dataclasses.replace(
        inp, airspeed_min=airspeed_min, tkoff_thr_max=tkoff_thr_max,
        tkoff_thr_max_raw=tkoff_thr_max,
        mission=dataclasses.replace(inp.mission, v_trans=airspeed_min),
        forward=dataclasses.replace(inp.forward, airspeed_min_eas=airspeed_min,
                                    tkoff_thr_max=tkoff_thr_max),
        start_params={**inp.start_params, "airspeed_min": airspeed_min,
                      "tkoff_thr_max": tkoff_thr_max, "q_transition": q},
    )


def rec_task_id(optimum: dict[str, float]) -> str:
    """`basics_rec:<AIRSPEED_MIN m/s>:<Q_TRANSITION_MS>:<TKOFF_THR_MAX %>`: D, E and Q2 at the
    recommended settings (E's rotor fade reads `Q_TRANSITION_MS`).

    The FT optimum's values are in the id, so a changed optimum is a new cache key (a re-run).
    """
    return (f"basics_rec:{round(optimum['airspeed_min'])}:{round(optimum['q_transition'] * 1e3)}"
            f":{round(optimum['tkoff_thr_max'] * 100)}")


def task_basics_rec(inp: StudyInputs, arg: str) -> dict[str, Any]:
    """E and Q2 (and D, whose recommendation E's back transition reads) at the recommended
    `AIRSPEED_MIN` and `TKOFF_THR_MAX` of the FT optimum: the same code as `task_basics`, on
    inputs with those two values replaced. The tables carry the suffix `_recommended`; the
    `register` case stays the headline. The sensitivity is not repeated: it stays at the
    register values.
    """
    v, q_ms, thr = (int(x) for x in arg.split(":"))
    keep = [a for a in inp.study.analyses if a in ("D", "E", "Q2")]
    study = inp.study.model_copy(update={"analyses": keep})
    inp2 = dataclasses.replace(with_settings(inp, float(v), thr / 100.0, q_ms / 1e3), study=study)
    out = task_basics(inp2)
    tables = {f"{k}_recommended": rows for k, rows in out["tables"].items()
              if k.startswith(("E_", "Q2_"))}
    return {"tables": tables, "errors": out.get("errors", []), "airspeed_min": float(v),
            "q_transition_s": q_ms / 1e3, "tkoff_thr_max": thr / 100.0,
            "notes": [f"E and Q2 at the FT optimum's AIRSPEED_MIN {v} m/s, Q_TRANSITION_MS "
                       f"{q_ms} and TKOFF_THR_MAX {thr} %: D is re-run at that entry speed, so E's "
                       "back transition and its planned deceleration follow the recommended "
                       "settings"]}


def forward_runs(
    vp: VehicleParams, scens: list[Scenario], prm: dict[str, float], fev: forward.FTEval
) -> list[forward.FTRun]:
    return [forward.run_ft(vp, sc, prm, fev) for sc in scens]


def task_ft_scan(inp: StudyInputs, v_arg: str) -> dict[str, Any]:
    """One `FT_scan` grid point: AIRSPEED_MIN fixed, the other three variables re-searched."""
    f, vp = inp.study.FT, inp.vehicle
    v = float(v_arg)
    fev, start = ft_settings(inp)
    core, _ = ft_scenarios(inp)
    w = ft_weights(inp, core, fev, start)
    fixed = {"airspeed_min": v}
    progress.phase(f"FT_scan {v:g} m/s: searching (CMA-ES)")
    r = forward.optimise_ft(
        vp, core, fev, w, {**start, **fixed}, ft_bounds(inp, fixed), fixed=fixed,
        seeds=tuple(f.scan.seeds), popsize=f.popsize, maxiter=f.scan.maxiter,
    )  # fmt: skip
    progress.phase(f"FT_scan {v:g} m/s: summary")
    sm = forward.summarise_ft(forward_runs(vp, core, r.params, fev), w)
    row = {
        "airspeed_min_eas_ms": v, **{FT_PARAM_COLUMNS[k]: r.params[k] for k in forward.FT_NAMES},
        "J_search": r.cost, "seed_spread": r.search.spread,
        "at_bound": ";".join(f"{k}:{x}" for k, x in r.at_bound.items()),
        "entry_eas_back_ms": v - 2.0, **{k: _num(x) for k, x in sm.items()},
    }  # fmt: skip
    return {"result": r, "row": row, "airspeed_min": v}


def wing_lift_peak(wing: Any) -> tuple[float, float]:
    """(alpha_w at the peak of the model's C_L [rad], that C_L), in the angle convention of the B1
    wing limits (alpha_w = body angle of attack + wing incidence). A 0.01 deg scan to 40 deg."""
    grid = np.radians(np.arange(0.0, 40.0001, 0.01))
    cl = np.array([float(aero.coefficients(float(a), wing)[0]) for a in grid])
    k = int(np.argmax(cl))
    return float(grid[k]), float(cl[k])


def wing_limit_cols(wing: Any, variant: Any) -> dict[str, float]:
    """Where a B1 wing limit sits against the lift peak (columns of every B1_front row)."""
    a_pk, cl_pk = wing_lift_peak(wing)
    lim = (math.radians(variant.alpha_max_deg) if variant.alpha_max_deg is not None
           else variant.alpha_margin * wing.alpha_s)  # fmt: skip
    cl_lim = float(aero.coefficients(lim, wing)[0])
    return {
        "alpha_limit_deg": math.degrees(lim), "alpha_s_deg": math.degrees(wing.alpha_s),
        "wing_incidence_deg": math.degrees(wing.i_w), "alpha_w_clmax_deg": math.degrees(a_pk),
        "cl_max": cl_pk, "cl_at_limit_over_max": cl_lim / cl_pk,
    }


def task_b1(inp: StudyInputs, scen_name: str) -> dict[str, Any]:
    """Analysis B1 for one scenario: least-energy back-transition from the AIRBRAKE exit.

    Per wing-limit variant, the front (energy against the distance cap at the headline band), the
    altitude-band sweep, the same-problem gap to C and, for the first scenario, the sensitivities.
    A point that did not converge or replay stays in the table with its energy blank: it means the
    solver failed from three starts, not that the point is infeasible.
    """
    b, c, d, vp = inp.study.B1, inp.study.C, inp.study.D, inp.vehicle
    scen = inp.scenarios[scen_name]
    cev, c_base, w = c_settings(inp)
    base = exit_params(inp, rotor_lag_s=c.rotor_lag_s)  # the entry state reads theta_fw0, delta_min
    v_air = braking.entry_airspeed(inp.airspeed_min, scen.rho, d.entry_convention)
    end_band = b.end_vh_band_ms or c.sink_tol_ms
    lim, alpha_hi = alpha_limits(inp)
    errors: list[str] = []
    out: dict[str, Any] = {
        "points": [], "gap": [], "sens": [], "schedule": [], "sols": [], "errors": errors,
        "v0": v_air, "scen": scen, "name": scen_name,
    }  # fmt: skip
    v_s = runs.stall_speed(scen.vehicle_for(vp), scen.rho)
    variants = {v.name: v for v in b.variants}
    ceiling = variants.get("c_envelope")
    if ceiling is not None and b1_ceiling_deg(inp, ceiling) > math.degrees(inp.theta_p) + 1e-9:
        raise ValueError(
            f"B1 variant c_envelope: pitch command ceiling {b1_ceiling_deg(inp, ceiling):g} deg "
            f"is above PTCH_LIM_MAX_DEG ({math.degrees(inp.theta_p):g} deg); it describes what "
            "C's ramp can reach"
        )
    ex_max = max(b.eps_x_m)

    def options(v: Any, band: float, theta_deg: float | None = None) -> coll.CollocationOptions:
        kw: dict[str, Any] = {
            "n": b.intervals, "max_iter": b.max_iter, "vh_band": band,
            "theta_cmd_max": math.radians(theta_deg or b1_ceiling_deg(inp, v)),
        }  # fmt: skip
        if v.alpha_max_deg is not None:
            kw["alpha_max"] = math.radians(v.alpha_max_deg)
        else:
            kw["stall_margin"] = v.alpha_margin
        return coll.CollocationOptions(**kw)

    def problem(opt: coll.CollocationOptions, rotor_start: str) -> coll.Problem:
        return coll.make_problem_exit(
            vp, scen, base, v_air, opt, inp.controller_overrides, inp.theta_p, rotor_start
        )

    def series(prob: coll.Problem, chk: coll.ReplayCheck) -> Table:
        sim = chk.sim
        aw = np.degrees(sim["alpha"] + prob.vp.wing.i_w)
        cols = {"t": sim.t, "x": sim["x"], "h": sim["h"], "V": sim["V"], "v_h": sim["vh"],
                "theta": np.degrees(sim["theta"]), "alpha_w_deg": aw, "delta_r": sim["delta_r"],
                "theta_cmd": np.degrees(sim["theta_cmd"]), "omega_r": sim["omega_r"]}  # fmt: skip
        return [{k: float(v[i]) for k, v in cols.items()}
                for i in _sample_indices(len(sim.t), 5)]

    def row(
        prob: coll.Problem, ver: coll.Verified, variant: str, eh: float, ex: float, rs: str,
        band: float, theta_deg: float,
    ) -> dict[str, Any]:  # fmt: skip
        s = ver.sol
        r: dict[str, Any] = {
            "scenario": scen_name, "variant": variant, "eps_h_m": eh, "eps_x_m": ex,
            **wing_limit_cols(vp.wing, variants[variant]),
            "end_vh_band_ms": band, "rotor_start": rs, "theta_cmd_max_deg": theta_deg,
            "converged": s.ok, "replay_verified": ver.verified,
            "converged_attempts": sum(a.ok for a in ver.attempts), "status": s.status,
            "V0_air_ms": v_air, "V0_ground_ms": float(prob.x0[2]), "Vs_ms": v_s,
            "below_vs": v_air < v_s,
        }  # fmt: skip
        if not (s.ok and ver.verified and ver.check is not None):
            return r
        chk = ver.check
        if ver.delayed is not None:  # does the schedule survive commands that are 10 ms late?
            r["replay_late_command_ok"] = ver.delay_robust
            r["replay_late_command_excursion_m"] = ver.delayed.metrics["excursion"]
        fl = braking.braking_flags(
            chk.sim, vp, scen, base, alpha_hi=alpha_hi, alpha_min=lim.alpha_min,
            kappa=inp.descent.kappa, gate=d.alpha_gate_ms, sink_tol=c.sink_tol_ms,
        )  # fmt: skip
        sm = coll.schedule_summary(prob, chk, d.alpha_gate_ms)
        j = (w.w_E * s.energy / w.E_ref + w.w_x * s.distance / w.x_ref
             + w.w_h * s.excursion / w.h_ref)  # fmt: skip
        r.update({
            "energy_kJ": s.energy / 1e3, "excursion_m": s.excursion, "distance_m": s.distance,
            "t_f_s": s.t_f, **{f"replay_err_{k}": e for k, e in chk.rel_error.items()},
            "J_point": j, **{k: _num(v) for k, v in sm.items()},
            **{k: _num(fl[k]) for k in ("vrs_min", "t_above_alpha_08_s", "alpha_max_gated_deg")},
        })  # fmt: skip
        return r

    n_front = len(b.variants) * len(b.eps_h_m) * len(b.eps_x_m)
    n_total = (n_front + len(b.band_sweep.eps_h_m) + len(b.variants)
               + (3 if scen_name == b.scenarios[0] else 0))  # an upper bound: the first scenario
    n_done = [0]  # also has the rotor-start, band and ceiling sensitivities

    def solved(
        kind: str, what: str, ok: bool, verified: bool, e_kj: float | None, t0: float
    ) -> None:  # fmt: skip
        n_done[0] += 1
        state = "verified" if verified else ("converged, replay differs" if ok else "no solution")
        e = f" E={e_kj:.2f} kJ" if e_kj is not None else ""
        progress.step(f"{scen_name} {kind} {what}: {state}{e} ({time.time() - t0:.0f} s)",
                      n_done[0], max(n_total, n_done[0]))  # fmt: skip

    def solve_point(
        v: Any, eh: float, ex: float, *, rs: str | None = None, band: float | None = None,
        theta_deg: float | None = None,
    ) -> tuple[dict[str, Any], Table, coll.B1Solution | None]:  # fmt: skip
        rs, band = rs or c.rotor_start, band or end_band
        theta = theta_deg or b1_ceiling_deg(inp, v)
        t0 = time.time()
        progress.phase(f"B1 {scen_name}: solving {v.name} eps_h={eh:g} eps_x={ex:g}")
        prob = problem(options(v, band, theta), rs)
        ver = coll.solve_verified(prob, eh, ex, "energy", ns=tuple(b.refine_from))
        rw = row(prob, ver, v.name, eh, ex, rs, band, theta)
        e = rw.get("energy_kJ")
        solved("solve", f"{v.name} eps_h={eh:g} eps_x={ex:g}", ver.sol.ok, ver.verified, e, t0)
        ser = series(prob, ver.check) if ver.verified and ver.check is not None else []
        return rw, ser, ver.sol

    def attempt(label: str, fn: Callable[[], Any]) -> Any:
        try:
            return fn()
        except Exception as exc:  # noqa: BLE001
            errors.append(f"{label}: {type(exc).__name__}: {exc}")
            return None

    try:
        problem(options(b.variants[0], end_band), c.rotor_start)  # the entry must exist
    except ValueError as exc:
        errors.append(str(exc))
        return out

    # -- the front: per variant, the headline band(s) against the distance caps
    kept: dict[str, list[tuple[dict[str, Any], Table]]] = {v.name: [] for v in b.variants}
    for v in b.variants:
        for eh in b.eps_h_m:
            for ex in b.eps_x_m:
                label = f"{v.name} eps_h={eh:g} eps_x={ex:g}"
                got = attempt(label, partial(solve_point, v, eh, ex))
                if got is None:
                    out["points"].append({
                        "scenario": scen_name, "variant": v.name, "eps_h_m": eh, "eps_x_m": ex,
                        "converged": False, "replay_verified": False, "status": errors[-1],
                    })  # fmt: skip
                    continue
                rw, ser, sol = got
                out["points"].append(rw)
                out["sols"].append((v.name, eh, ex, sol))
                if "J_point" in rw:
                    kept[v.name].append((rw, ser))
    for v in b.variants:  # the point with the least C cost per variant
        if kept[v.name]:
            best = min(kept[v.name], key=lambda t: t[0]["J_point"])
            best[0]["j_best"] = True
            out["schedule"] += [{"scenario": scen_name, "variant": v.name,
                                 "eps_x_m": best[0]["eps_x_m"], **r} for r in best[1]]  # fmt: skip
    for r in out["points"]:
        r.setdefault("j_best", False)

    # -- sensitivities: the altitude band (every scenario), the rest on the first scenario
    def sens(kind: str, value: Any, **kw: Any) -> None:
        v = kw.pop("variant")
        eh = kw.pop("eps_h")
        got = attempt(f"sens {kind}={value}", partial(solve_point, v, eh, ex_max, **kw))
        if got is None:
            out["sens"].append({"scenario": scen_name, "sensitivity": kind, "value": value,
                                "variant": v.name, "converged": False, "replay_verified": False,
                                "status": errors[-1]})  # fmt: skip
            return
        out["sens"].append({"sensitivity": kind, "value": value, **got[0]})

    bv = variants[b.band_sweep.variant]
    for eh in b.band_sweep.eps_h_m:
        sens("eps_h_m", eh, variant=bv, eps_h=eh)
    if scen_name == b.scenarios[0]:
        v0, eh0 = b.variants[0], b.eps_h_m[0]
        if b.sens.rotor_start_flip:
            other = "idle" if c.rotor_start == "carry" else "carry"
            sens("rotor_start", other, variant=v0, eps_h=eh0, rs=other)
        if b.sens.end_vh_band_ms is not None:
            sens("end_vh_band_ms", b.sens.end_vh_band_ms, variant=v0, eps_h=eh0,
                 band=b.sens.end_vh_band_ms)  # fmt: skip
        abs_v = next((v for v in b.variants if v.alpha_max_deg is not None), None)
        if abs_v is not None and b.sens.c_envelope_theta_cmd_deg is not None:
            sens("theta_cmd_max_deg", b.sens.c_envelope_theta_cmd_deg, variant=abs_v, eps_h=eh0,
                 theta_deg=b.sens.c_envelope_theta_cmd_deg)  # fmt: skip

    # -- the same-problem gap: B1 at C's own excursion and distance
    if b.gap_c is None:
        gp = {"T_bt": inp.start_params["T_bt"], "theta_A": inp.start_params["theta_A"],
              "spin_min": inp.start_params["spin_min"], "q": restricted.C_Q_START}  # fmt: skip
        src = f"register parameters at q = {restricted.C_Q_START:g}"
    else:
        gc = b.gap_c
        gp = {"T_bt": gc.T_bt, "theta_A": math.radians(gc.theta_A_deg), "spin_min": gc.spin_min,
              "q": gc.q}  # fmt: skip
        src = "B1.gap_c (the C optimum, pasted from C_params)"
    cm = attempt("C at the gap parameters", lambda: restricted.evaluate_c(
        vp, scen, gp, cev, c_base))  # fmt: skip
    if cm is not None and "not_stopped" in cm.failures:
        errors.append(f"C at the gap parameters did not stop ({','.join(cm.failures)}): no gap")
        cm = None
    if cm is not None:
        for v in b.variants:
            t0 = time.time()

            def gap_solve(v: Any = v) -> coll.BoundCheck:
                prob = problem(options(v, end_band), c.rotor_start)
                return coll.bound_check_c(prob, cm, ns=tuple(b.refine_from))

            res = attempt(f"gap {v.name}", gap_solve)
            g: dict[str, Any] = {
                "scenario": scen_name, "variant": v.name, "c_source": src,
                "c_energy_kJ": cm.energy / 1e3, "c_excursion_m": cm.excursion,
                "c_distance_m": cm.distance, "c_flags": ",".join(cm.failures),
            }  # fmt: skip
            if res is not None:
                s_ = res.b1
                g.update({"b1_converged": s_.ok, "b1_replay_verified": res.verified,
                          "b1_late_command_ok": res.delay_robust,
                          "eps_h_m": s_.eps_h, "eps_x_m": s_.eps_x})  # fmt: skip
                if res.verified:
                    g.update({
                        "b1_energy_kJ": s_.energy / 1e3, "b1_excursion_m": s_.excursion,
                        "b1_distance_m": s_.distance, "energy_margin": res.energy_margin,
                    })  # fmt: skip
                solved("gap", v.name, s_.ok, res.verified, None, t0)
            out["gap"].append(g)
    return out


def sensitivity_model(inp: StudyInputs) -> sens.HeadlineModel:
    c = inp.study.sensitivity
    scen = inp.scenarios[c.scenario] if c.scenario in inp.scenarios else next(
        iter(inp.scenarios.values()))
    inputs = sens.default_inputs(
        inp.vehicle, scen, statuses=inp.input_statuses(), fractions=inp.input_fractions(),
        controller_nominals=inp.controller_nominals(),
    )  # fmt: skip
    # the back transition is the AIRBRAKE-exit run of D and C (its vertical loop is the ArduPilot
    # one), so the altitude-hold gains of the other loop are replaced by the Z-loop gains
    inputs = [i for i in inputs if not i.name.startswith("alt_hold_")]
    inputs += sens.z_loop_inputs(inp.study.C.z.model_dump())
    cev, base, _ = c_settings(inp)
    mp_site = site_mission(inp, scen.ground_amsl_m)
    budget = (mp_site, inp.window, inp.descent) if c.include_budget else None
    return sens.HeadlineModel(
        inp.vehicle, scen, inputs, prm=dict(inp.start_params), budget=budget,
        controller=dict(inp.controller_overrides), theta_p=inp.theta_p,
        c_eval=cev, c_base=base, budget_bp=d_exit_params(inp),
    )  # fmt: skip


def task_sensitivity(inp: StudyInputs) -> dict[str, Any]:
    c = inp.study.sensitivity
    progress.phase("sensitivity: building the model")
    model = sensitivity_model(inp)
    res = sens.run_sensitivity(model, r_small=c.r_small, r_large=c.r_large)
    progress.phase("sensitivity: nominal run")
    return {"result": res, "nominal": model({}), "outputs": model.outputs}


# ---- planning ----------------------------------------------------------------------------------
def plan_tasks(inp: StudyInputs) -> list[str]:
    """Task ids to run for the selected analyses (`inp.study.analyses`), most expensive first."""
    wanted = set(inp.study.analyses)
    tasks: list[str] = []
    if wanted & {"A", "D", "F", "E", "Q2"}:
        tasks.append("basics")
    if "C" in wanted:
        tasks.append("C")
    if "FT" in wanted:
        tasks += grid_coarse_ids(inp)  # the report of the winner follows (`follow_up_tasks`)
        tasks += [f"FT_scan:{v:g}" for v in inp.study.FT.scan.airspeed_min_eas_ms]
    if "B1" in wanted:
        tasks += [f"B1:{n}" for n in inp.study.B1.scenarios]
    if "sensitivity" in wanted:
        tasks.append("sensitivity")
    return sorted(tasks, key=lambda t: -TASK_COST.get(t.split(":")[0], 100))


def task_selection(inp: StudyInputs, task: str) -> str:
    """Which analyses a task covers: part of its cache key, so a partial run is never reused."""
    if task.split(":")[0] == "basics":
        return "+".join(a for a in ("A", "D", "F", "E", "Q2") if a in inp.study.analyses)
    return ""


def run_task(inp: StudyInputs, task: str) -> dict[str, Any]:
    kind, _, arg = task.partition(":")
    if kind == "basics":
        return task_basics(inp)
    if kind == "C":
        return task_c(inp)
    if kind == "basics_rec":
        return task_basics_rec(inp, arg)
    if kind == "FT":
        return task_ft(inp, arg)
    if kind == "FT_grid":
        return task_ft_grid(inp, arg)
    if kind == "FT_scan":
        return task_ft_scan(inp, arg)
    if kind == "B1":
        return task_b1(inp, arg)
    if kind == "sensitivity":
        return task_sensitivity(inp)
    raise ValueError(f"unknown task {task!r}")
