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

from sagetrans import atmosphere
from sagetrans.analysis import (
    braking,
    descent,
    energy,
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
from sagetrans.io.inputs import StudyInputs
from sagetrans.io.settings import END_SPEED_MS
from sagetrans.physics import aero, battery, motor, rotor
from sagetrans.physics.params import VehicleParams

RHO0 = float(atmosphere.density(0.0))
Table = list[dict[str, Any]]

# rough wall-clock seconds on a laptop, for scheduling and for the pre-flight estimate
TASK_COST = {"basics": 60, "C": 330, "B1": 400, "sensitivity": 220}


def tas(eas: float, scen: Scenario) -> float:
    """True airspeed for an equivalent airspeed at the scenario's density."""
    return eas * math.sqrt(RHO0 / scen.rho)


def _num(x: float) -> float | str:
    """A table cell: the number, or empty if it is not finite (no NaN or inf in written files)."""
    return float(x) + 0.0 if math.isfinite(x) else ""  # + 0.0: no "-0.0" in the files


def _attempt(label: str, fn: Callable[[], Any], errors: list[str]) -> Any:
    """Run one piece of a task; a failure is recorded in `errors` with its traceback tail."""
    t = time.time()
    try:
        out = fn()
        print(f"    ok   {label} ({time.time() - t:.0f} s)", flush=True)
        return out
    except Exception as exc:  # noqa: BLE001
        tb = traceback.format_exc().strip().splitlines()[-3:]
        errors.append(f"{label}: {type(exc).__name__}: {exc}")
        print(f"    FAIL {label}: {exc}\n      " + "\n      ".join(tb), flush=True)
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
    v_end = mp.v_trans if q2c.feed_convention == "tas" else mp.v_trans * math.sqrt(RHO0 / rho_c)
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
    mp = energy.mission_at_site(inp.mission, site)
    masses = {"out": vp.m, "ret": vp.m - mp.payload_kg} if kind == "round_trip" else {"leg": vp.m}
    rho_bt = float(atmosphere.density(site + mp.h_bt_agl_m, mp.isa_offset_k))
    v0 = inp.entry_eas * math.sqrt(RHO0 / rho_bt)  # D's entry EAS as a true airspeed
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
                                  bp=inp.controller, v0_entry=v0, a_fwd_source=src)  # fmt: skip
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
        for name in c.scenarios:
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
        base_bp = exit_params(
            inp, rotor_lag_s=c.rotor_lag_s, alpha_cap=alpha_cap if c.enforce_alpha else None
        )

        def entry(sc: Scenario) -> float:
            return braking.entry_airspeed(inp.airspeed_min, sc.rho, c.entry_convention)

        Run = tuple[braking.BrakingResult, dict[str, Any], float]

        def run(sc: Scenario, bp: Any, v_air: float) -> Run:
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
        for a_plan in c.trade.a_plan_ms2:
            for wind in c.trade.headwinds_ms:
                for test in ("ground", "air") if wind > 0 else ("ground",):
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
                mp_site = energy.mission_at_site(inp.mission, site)
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


# ---- tasks: C, B1, sensitivity -------------------------------------------------------------
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


def c_bounds(inp: StudyInputs) -> Bounds:
    b = inp.study.C.bounds
    return Bounds(
        restricted.C_BOUNDS.names,
        (b.T_bt[0], math.radians(b.theta_A_deg[0]), b.spin_min[0], b.q[0]),
        (b.T_bt[1], math.radians(b.theta_A_deg[1]), b.spin_min[1], b.q[1]),
    )  # fmt: skip


def _c_row(label: str, results: list[restricted.CRun], w: CostWeights) -> dict[str, Any]:
    return {"case": label, **{k: _num(v) for k, v in restricted.summarise_c(results, w).items()}}


def task_c(inp: StudyInputs) -> dict[str, Any]:
    """Analysis C: search the four ArduPilot variables from the AIRBRAKE exit, then report.

    Tables: `C_params` (the optimum and the register at q = 2), `C_search`, `C_runs` (one row per
    ensemble member, with the capability ceiling and D's alpha and VRS checks), `C_q_curve`
    (the optimum's ramp with q varied) and `C_sens` (one-at-a-time changes at the optimum).
    """
    c, d, vp = inp.study.C, inp.study.D, inp.vehicle
    scens = _ensemble(inp, c.ensemble_n, c.ensemble_seed)
    cev, base, w = c_settings(inp)
    bounds = c_bounds(inp)
    start = {k: inp.start_params[k] for k in ("T_bt", "theta_A", "spin_min")}
    r = restricted.optimise_c(
        vp, scens, cev, base, w, start, bounds=bounds, seeds=tuple(c.seeds),
        popsize=c.popsize, maxiter=c.maxiter,
    )  # fmt: skip
    best = r.params
    reg = {**start, "q": restricted.C_Q_START}

    def run_all(
        prm: dict[str, float], ev: restricted.CEval = cev, bp: BackTransitionParams = base
    ) -> list[restricted.CRun]:
        return [restricted.run_c(vp, s, prm, ev, bp) for s in scens]

    def deg(prm: dict[str, float]) -> dict[str, Any]:
        return {"T_bt_s": prm["T_bt"], "theta_A_deg": math.degrees(prm["theta_A"]),
                "spin_min": prm["spin_min"], "Q_TRANS_DECEL_ms2": prm["q"]}  # fmt: skip

    t0 = time.time()
    opt = run_all(best)
    reg_runs = run_all(reg)
    s_opt, s_reg = restricted.summarise_c(opt, w), restricted.summarise_c(reg_runs, w)
    params: Table = [
        {"case": "optimum", **deg(best), "J": r.cost, "J_start": r.cost_default,
         "seed_spread": r.search.spread,
         "at_bound": ";".join(f"{k}:{v}" for k, v in r.at_bound.items()),
         **{k: _num(v) for k, v in s_opt.items() if k != "J"}},
        {"case": f"register at q = {restricted.C_Q_START:g}", **deg(reg), "J": s_reg["J"],
         "J_start": "", "seed_spread": "", "at_bound": "",
         **{k: _num(v) for k, v in s_reg.items() if k != "J"}},
    ]  # fmt: skip
    search = [{"seed": sr.seed, "best_cost": sr.f, "evaluations": sr.n_evals}
              for sr in r.search.seeds]  # fmt: skip

    # per member: the run, the capability ceiling, D's alpha and VRS checks (reporting only)
    lim, alpha_hi = alpha_limits(inp)
    run_rows: Table = []
    for i, (s, cr) in enumerate(zip(scens, opt, strict=True)):
        vps = s.vehicle_for(vp)
        bp_i = dataclasses.replace(base, T_bt=best["T_bt"], theta_A=best["theta_A"],
                                   delta_min=best["spin_min"])  # fmt: skip
        fl = braking.braking_flags(
            cr.sim, vps, s, bp_i, alpha_hi=alpha_hi, alpha_min=lim.alpha_min,
            kappa=inp.descent.kappa, gate=d.alpha_gate_ms, sink_tol=c.sink_tol_ms,
        )  # fmt: skip
        a_cap = restricted.capability_ceiling(vp, s, best, cev, base)
        run_rows.append({
            "member": i, "altitude_m": s.altitude_amsl_m, "isa_offset_k": s.isa_offset_k,
            "mass_kg": s.mass_kg, "soc": s.soc, "headwind_ms": s.headwind_ms,
            "V0_air_ms": cr.v_air0, "V0_ground_ms": cr.v_ground0, "x_tgt_m": cr.x_tgt,
            "x_end_m": _num(cr.x_end), "overshoot_m": _num(cr.overshoot),
            "energy_kJ": cr.metrics.energy / 1e3, "excursion_m": cr.metrics.excursion,
            "end_sink_ms": _num(-cr.vh_end), "flags": ",".join(cr.metrics.failures),
            "a_cap_ms2": _num(a_cap), "q_over_a_cap": _num(best["q"] / a_cap),
            **{k: v if isinstance(v, bool) else _num(v) for k, v in fl.items()},
        })  # fmt: skip

    q_curve = [_c_row(f"q = {q:g}", run_all({**best, "q": q}), w) | {"q": q}
               for q in c.q_curve]  # fmt: skip

    sens_rows: Table = []

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
    print(f"    C reports: {time.time() - t0:.0f} s", flush=True)
    return {
        "result": r, "scens": scens,
        "tables": {"C_params": params, "C_search": search, "C_runs": run_rows,
                   "C_q_curve": q_curve, "C_sens": sens_rows},
    }  # fmt: skip


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
    ex_max = max(b.eps_x_m)

    def options(v: Any, band: float, theta_deg: float | None = None) -> coll.CollocationOptions:
        kw: dict[str, Any] = {
            "n": b.intervals, "max_iter": b.max_iter, "vh_band": band,
            "theta_cmd_max": math.radians(theta_deg or v.theta_cmd_max_deg),
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
        return [{k: float(v[i]) for k, v in cols.items()} for i in range(0, len(sim.t), 5)]

    def row(
        prob: coll.Problem, ver: coll.Verified, variant: str, eh: float, ex: float, rs: str,
        band: float, theta_deg: float,
    ) -> dict[str, Any]:  # fmt: skip
        s = ver.sol
        r: dict[str, Any] = {
            "scenario": scen_name, "variant": variant, "eps_h_m": eh, "eps_x_m": ex,
            "end_vh_band_ms": band, "rotor_start": rs, "theta_cmd_max_deg": theta_deg,
            "converged": s.ok, "replay_verified": ver.verified,
            "converged_attempts": sum(a.ok for a in ver.attempts), "status": s.status,
            "V0_air_ms": v_air, "V0_ground_ms": float(prob.x0[2]), "Vs_ms": v_s,
            "below_vs": v_air < v_s,
        }  # fmt: skip
        if not (s.ok and ver.verified and ver.check is not None):
            return r
        chk = ver.check
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

    def solve_point(
        v: Any, eh: float, ex: float, *, rs: str | None = None, band: float | None = None,
        theta_deg: float | None = None,
    ) -> tuple[dict[str, Any], Table, coll.B1Solution | None]:  # fmt: skip
        rs, band = rs or c.rotor_start, band or end_band
        theta = theta_deg or v.theta_cmd_max_deg
        t0 = time.time()
        prob = problem(options(v, band, theta), rs)
        ver = coll.solve_verified(prob, eh, ex, "energy", ns=tuple(b.refine_from))
        rw = row(prob, ver, v.name, eh, ex, rs, band, theta)
        e = rw.get("energy_kJ")
        print(f"    {scen_name} {v.name} eps_h={eh:g} eps_x={ex:g}: ok={ver.sol.ok} "
              f"verified={ver.verified}" + (f" E={e:.2f} kJ" if e is not None else "")
              + f" ({time.time() - t0:.0f} s)", flush=True)  # fmt: skip
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
                          "eps_h_m": s_.eps_h, "eps_x_m": s_.eps_x})  # fmt: skip
                if res.verified:
                    g.update({
                        "b1_energy_kJ": s_.energy / 1e3, "b1_excursion_m": s_.excursion,
                        "b1_distance_m": s_.distance, "energy_margin": res.energy_margin,
                    })  # fmt: skip
                print(f"    {scen_name} gap {v.name}: ok={s_.ok} verified={res.verified} "
                      f"({time.time() - t0:.0f} s)", flush=True)  # fmt: skip
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
    mp_site = energy.mission_at_site(inp.mission, scen.ground_amsl_m)
    budget = (mp_site, inp.window, inp.descent) if c.include_budget else None
    return sens.HeadlineModel(
        inp.vehicle, scen, inputs, prm=dict(inp.start_params), budget=budget,
        controller=dict(inp.controller_overrides), theta_p=inp.theta_p,
        a_plan_source="simulated", v0_eas=inp.entry_eas,
    )  # fmt: skip


def task_sensitivity(inp: StudyInputs) -> dict[str, Any]:
    c = inp.study.sensitivity
    model = sensitivity_model(inp)
    res = sens.run_sensitivity(model, r_small=c.r_small, r_large=c.r_large)
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
    if kind == "B1":
        return task_b1(inp, arg)
    if kind == "sensitivity":
        return task_sensitivity(inp)
    raise ValueError(f"unknown task {task!r}")
