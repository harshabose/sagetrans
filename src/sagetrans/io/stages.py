"""The analysis stages of a study, each a function of the loaded inputs.

A *task* is the unit of parallel work: ``basics`` (analyses A, D, F, E and Q2, which are quick and
share results), ``C``, ``B2``, ``B1:<scenario>`` (one per scenario) and ``sensitivity``. Every task
returns a plain dict of tables and objects that can be pickled, so it can be cached and run in a
separate process. Nothing here reads a file: all data arrives through `StudyInputs`.
"""

from __future__ import annotations

import dataclasses
import math
import time
import traceback
from collections.abc import Callable
from typing import Any

import numpy as np
from scipy import stats
from scipy.optimize import brentq

from sagetrans import atmosphere
from sagetrans.analysis import (
    braking,
    descent,
    energy,
    freeform_opt,
    pusher,
    restricted,
    runs,
    soc_trace,
    zero_loss,
)
from sagetrans.analysis import collocation as coll
from sagetrans.analysis import sensitivity as sens
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.ensemble import bootstrap_percentile, braking_ensemble, latin_hypercube
from sagetrans.io.inputs import StudyInputs
from sagetrans.physics import aero, battery, motor, rotor
from sagetrans.physics.params import VehicleParams

RHO0 = float(atmosphere.density(0.0))
Table = list[dict[str, Any]]

# rough wall-clock seconds on a laptop, for scheduling and for the pre-flight estimate
TASK_COST = {"basics": 60, "C": 330, "B2": 360, "B1": 400, "sensitivity": 220}


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
        base_bp = dataclasses.replace(
            inp.controller, theta_fw0=math.radians(c.theta_fw0_deg), pit_lim_mode=c.pit_lim_mode,
            v_min_ref=inp.airspeed_min, ff_mode=c.ff_mode, ff_lift_tau_s=c.ff_lift_tau_s,
            ff_lift_scale=c.ff_lift_scale, rotor_lag_s=c.rotor_lag_s,
            alpha_cap=alpha_cap if c.enforce_alpha else None, alpha_cap_vmin=c.alpha_gate_ms,
        )  # fmt: skip

        def entry(sc: Scenario) -> float:
            return braking.entry_airspeed(inp.airspeed_min, c.q_assist_speed_ms, sc.rho,
                                          c.entry_convention)  # fmt: skip

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
        }]  # fmt: skip
        out["a_rec"], out["a_rec_pct"], out["pct"] = a_min, pct, rec.percentile
        out["a_rec_ci"] = (lo, hi)

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


# ---- tasks: C, B2, B1, sensitivity -------------------------------------------------------------
def task_c(inp: StudyInputs) -> dict[str, Any]:
    c = inp.study.C
    scens = _ensemble(inp, c.ensemble_n, c.ensemble_seed)
    r = restricted.optimise(
        inp.vehicle, scens, "back", seeds=tuple(c.seeds), popsize=c.popsize, maxiter=c.maxiter,
        a_plan_source=c.a_plan_source, theta_p=inp.theta_p,
        controller_overrides=inp.controller_overrides, start=inp.start_params,
        v0_eas=inp.entry_eas,
    )  # fmt: skip
    return {"result": r, "scens": scens}


def _b1_problem(inp: StudyInputs, scen: Scenario) -> coll.Problem:
    c = inp.study.B1
    opts = coll.CollocationOptions(n=c.intervals, max_iter=c.max_iter)
    return coll.make_problem(
        inp.vehicle, scen, tas(inp.entry_eas, scen), opts, inp.controller_overrides, inp.theta_p
    )


def task_b2(inp: StudyInputs) -> dict[str, Any]:
    c = inp.study.B2
    scens = _ensemble(inp, c.ensemble_n, c.ensemble_seed)
    prob = _b1_problem(inp, scens[c.start.scenario_index % len(scens)])
    ver = coll.solve_verified(
        prob, c.start.eps_h_m, c.start.eps_x_m, "energy", ns=tuple(inp.study.B1.refine_from)
    )
    init = freeform_opt.init_from_collocation(prob, ver.sol)
    r = freeform_opt.optimise(
        inp.vehicle, scens, seeds=tuple(c.seeds), popsize=c.popsize, maxiter=c.maxiter,
        init_params=init, sigma0=c.sigma0, v0_eas=inp.entry_eas,
    )  # fmt: skip
    return {"result": r, "scens": scens, "b1_start": (ver.sol.ok, ver.verified, ver.sol.energy)}


def task_b1(inp: StudyInputs, scen_name: str) -> dict[str, Any]:
    c = inp.study.B1
    scen = inp.scenarios[scen_name]
    prob = _b1_problem(inp, scen)
    points: Table = []
    sols: list[Any] = []
    errors: list[str] = []
    for eh in c.eps_h_m:
        t = time.time()
        try:
            ver = coll.solve_verified(prob, eh, c.eps_x_m, "energy", ns=tuple(c.refine_from))
            s = ver.sol
            points.append({
                "scenario": scen_name, "eps_h_m": eh, "eps_x_m": c.eps_x_m, "converged": s.ok,
                "replay_verified": ver.verified, "status": s.status,
                "attempts": len(ver.attempts),
                "converged_attempts": sum(a.ok for a in ver.attempts),
                "excursion_m": s.excursion, "energy_J": s.energy, "distance_m": s.distance,
                "t_f_s": s.t_f,
                **({f"replay_err_{k}": e for k, e in ver.check.rel_error.items()}
                   if ver.check else {}),
            })  # fmt: skip
            sols.append(ver)
            print(f"    {scen_name} eps_h={eh}: ok={s.ok} verified={ver.verified} "
                  f"E={s.energy:.0f} J ({time.time() - t:.0f} s)", flush=True)
        except Exception as exc:  # noqa: BLE001
            errors.append(f"eps_h={eh}: {type(exc).__name__}: {exc}")
            points.append({"scenario": scen_name, "eps_h_m": eh, "converged": False,
                           "replay_verified": False, "status": str(exc)})  # fmt: skip
    bound = None
    try:
        bc = coll.bound_check(prob)
        chk = coll.replay(prob, bc.b1) if bc.b1.ok else None
        bound = {
            "baseline": bc.baseline, "b1_energy_J": bc.b1.energy,
            "b1_excursion_m": bc.b1.excursion, "b1_distance_m": bc.b1.distance,
            "b1_ok": bc.b1.ok, "energy_margin": bc.energy_margin,
            "b1_replay_verified": bool(chk.passes()) if chk else False,
        }  # fmt: skip
    except Exception as exc:  # noqa: BLE001
        errors.append(f"bound_check: {type(exc).__name__}: {exc}")
    return {"points": points, "sols": sols, "bound": bound, "errors": errors,
            "v0": tas(inp.entry_eas, scen), "scen": scen, "name": scen_name}  # fmt: skip


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
        a_plan_source=inp.study.C.a_plan_source, v0_eas=inp.entry_eas,
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
    if "B2" in wanted:
        tasks.append("B2")
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
    if kind == "B2":
        return task_b2(inp)
    if kind == "B1":
        return task_b1(inp, arg)
    if kind == "sensitivity":
        return task_sensitivity(inp)
    raise ValueError(f"unknown task {task!r}")
