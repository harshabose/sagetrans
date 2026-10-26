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
        for case, over in cases:
            bps = dataclasses.replace(base_bp, **over)
            r, fl, _ = run(scn, bps, v_air_n)
            m = r.sim.metrics
            srow: dict[str, Any] = {
                "case": case, "a_eq_ms2": _num(r.a_eq), "x_actual_m": _num(r.x_actual),
                "alt_gain_m": m["altitude_gain"], "alt_loss_m": m["altitude_loss"],
                **cells({k: fl[k] for k in ("vh_end_ms", "alpha_max_gated_deg",
                                            "t_above_alpha_08_s", "t_above_alpha_s_s")}),
                "a_rec_pct_ensemble_ms2": "",
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
        a_rec = out.get("a_rec")
        if a_rec is None or not math.isfinite(a_rec):
            a_plan, a_source = inp.study.E.a_plan_fallback_ms2, "fallback"
            errors.append(f"E: no recommendation from D, a_plan = {a_plan:g} m/s2 (fallback)")
        else:
            a_plan, a_source = float(a_rec), "D"
        # forward-transition acceleration from Q2: E's own site ("case"), the worst over the Q2
        # grid ("worst") or the mission file; all three are reported, one is used
        ec, q2c = inp.study.E, inp.study.Q2
        mp0 = inp.mission
        rho_c = float(atmosphere.density(mp0.clearance_amsl, mp0.isa_offset_k))
        feed_tas = q2c.feed_convention == "tas"
        v_end = mp0.v_trans if feed_tas else mp0.v_trans * math.sqrt(RHO0 / rho_c)
        case = pusher.forward_accel(vp, rho_c, v_end, mp0.soc_mean, inp.tkoff_thr_max, q2c.n_speed)
        worst_row, worst = q2_worst(q2_grid([q2c.feed_convention]), q2c.feed_convention)

        def pick(f: pusher.FwdAccel) -> float:
            return f.a_min if ec.a_fwd_metric == "min" else f.a_equiv

        a_case, a_worst = pick(case), pick(worst)
        a_used = {"case": a_case, "worst": a_worst, "mission": mp0.a_fwd}[ec.a_fwd_mode]
        fwd_cols = {"a_fwd_used_ms2": _num(a_used), "a_fwd_case_ms2": _num(a_case),
                    "a_fwd_worst_ms2": _num(a_worst), "a_fwd_mode": ec.a_fwd_mode}  # fmt: skip
        fwd_src = None if ec.a_fwd_mode == "mission" else (
            f"quasi-steady estimate, a_fwd from Q2 ({ec.a_fwd_mode})")  # fmt: skip
        budgets: dict[str, Any] = {}
        ranges = sorted({*inp.study.E.ranges_km, *inp.requirements_km})
        usable = math.isfinite(a_used) and a_used > 0.0
        v_target = worst_row["v_end_tas_ms"] if ec.a_fwd_mode == "worst" else v_end
        if not usable:  # no fallback to the mission value: that silent default is what Q2 removes
            msg = (f"a_fwd = {a_used:.3g} m/s2 ({ec.a_fwd_mode}, {ec.a_fwd_metric}): the pusher "
                   f"cannot reach {v_target:.1f} m/s at TKOFF_THR_MAX = {inp.tkoff_thr_max:g}")
            errors.append(f"E forward transition: {msg}")
            out["tables"].setdefault("E_summary", []).append(
                {"case": "forward_transition", "error": msg, **fwd_cols}
            )
        for alt in inp.study.E.cruise_amsl_m if usable else []:
            name = f"cruise_{alt:.0f}m"
            mp = dataclasses.replace(inp.mission, cruise_amsl_m=alt, a_fwd=a_used)
            try:
                b = energy.build_budget(vp, mp, inp.window, inp.descent, a_plan, fwd_src)
            except Exception as exc:  # noqa: BLE001
                errors.append(f"E budget {name}: {type(exc).__name__}: {exc}")
                out["tables"].setdefault("E_summary", []).append({"case": name, "error": str(exc)})
                continue
            budgets[name] = b
            ok_ranges = [r * 1e3 for r in ranges if r * 1e3 >= b.fixed_distance_m]
            for row in b.table(ok_ranges):
                out["tables"].setdefault("E_budget", []).append({"case": name, **row})
            conf = b.conflicts(tuple(r * 1e3 for r in inp.requirements_km))
            out["tables"].setdefault("E_summary", []).append({
                "case": name, "v_cruise_ms": b.notes["v_cruise"],
                "fixed_segments_distance_km": b.fixed_distance_m / 1e3,
                "p_cruise_W": b.notes["p_cruise_W"],
                "e_cruise_Wh_per_km": b.e_cruise_J_per_m * 1e3 / 3600.0,
                "max_range_km": b.max_range_m / 1e3, "usable_Wh": b.usable_J / 3600.0,
                "reserve_Wh": b.reserve_J / 3600.0,
                **{f"conflict_{r:g}km": c for r, c in zip(inp.requirements_km, conf.values(),
                                                          strict=True)},
                "a_plan_ms2": a_plan, "a_plan_source": a_source, **fwd_cols,
            })  # fmt: skip
            for seg, s in b.segments.items():
                out["tables"].setdefault("E_segments", []).append({
                    "case": name, "segment": seg, "energy_Wh": s.energy_J / 3600.0,
                    "time_s": s.time_s, "distance_m": s.distance_m,
                    "source": s.source})  # fmt: skip
        out["budgets"] = budgets
        hc: Table = []
        for soc_eod in sorted({inp.window.soc_eod, min(1.0, inp.window.soc_eod + 0.1)}):
            w = dataclasses.replace(inp.window, soc_eod=soc_eod)
            for g in sorted({inp.mission.ground_amsl_m, inp.mission.ground_amsl_m + 2000.0}):
                mp = dataclasses.replace(inp.mission, ground_amsl_m=g, cruise_amsl_m=g + 1000.0)
                hc.append({"soc_end_of_discharge": soc_eod, "landing_elevation_m": g,
                           **energy.hover_check(vp, mp, w)})  # fmt: skip
        out["tables"]["E_hover_check"] = hc
        first = next(iter(budgets.values()), None)
        if first is not None:
            res = energy.reserve_energy(
                vp, inp.mission, inp.mission.soc_mean, first.e_cruise_J_per_m,
                first.segments["fwd_transition"].energy_J,
                first.segments["back_transition"].energy_J,
            )  # fmt: skip
            out["tables"]["E_reserve"] = [{"component": k, "energy_Wh": v / 3600.0}
                                          for k, v in res.items()]  # fmt: skip
        for name in inp.study.F.scenarios:
            sc = inp.scenarios[name]
            speeds = np.linspace(1.1 * energy.stall_speed_1g(sc.vehicle_for(vp), sc.rho), 40.0, 30)
            sw = energy.cruise_sweep(sc.vehicle_for(vp), sc.rho, inp.mission.soc_mean, speeds)
            for v, pw, j in zip(sw["V"], sw["power_W"], sw["J_per_m"], strict=True):
                out["tables"].setdefault("E_cruise_sweep", []).append({
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
    budget = (inp.mission, inp.window, inp.descent) if c.include_budget else None
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
