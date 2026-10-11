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
from scipy.optimize import brentq

from sagetrans import atmosphere
from sagetrans.analysis import (
    braking,
    descent,
    energy,
    freeform_opt,
    restricted,
    runs,
    zero_loss,
)
from sagetrans.analysis import collocation as coll
from sagetrans.analysis import sensitivity as sens
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.ensemble import latin_hypercube
from sagetrans.io.inputs import StudyInputs
from sagetrans.physics import battery, motor, rotor
from sagetrans.physics.params import VehicleParams

RHO0 = float(atmosphere.density(0.0))
Table = list[dict[str, Any]]

# rough wall-clock seconds on a laptop, for scheduling and for the pre-flight estimate
TASK_COST = {"basics": 60, "C": 330, "B2": 360, "B1": 400, "sensitivity": 220}


def tas(eas: float, scen: Scenario) -> float:
    """True airspeed for an equivalent airspeed at the scenario's density."""
    return eas * math.sqrt(RHO0 / scen.rho)


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


# ---- task: basics (A, D, F, E, Q2) -------------------------------------------------------------
def task_basics(inp: StudyInputs) -> dict[str, Any]:
    wanted = set(inp.study.analyses)
    errors: list[str] = []
    out: dict[str, Any] = {"tables": {}, "errors": errors}
    vp = inp.vehicle

    def stage_a() -> None:
        c = inp.study.A
        maps: dict[str, Any] = {}
        rows: Table = []
        for name in c.scenarios:
            sc = inp.scenarios[name]
            vps = sc.vehicle_for(vp)
            v_grid = [tas(v, sc) for v in c.v0_eas_ms]
            fm: dict[str, Any] = dict(
                zero_loss.feasibility_map(vps, sc, v_grid, c.decel_ms2, n=c.samples)
            )
            maps[name] = {"v_grid_eas": c.v0_eas_ms, "a_grid": c.decel_ms2, **fm}
            for i, ve in enumerate(c.v0_eas_ms):
                for j, a in enumerate(c.decel_ms2):
                    fb = fm["first_binding_pre_spun"][i][j]
                    rows.append({
                        "scenario": name, "altitude_m": sc.altitude_amsl_m, "V0_eas_ms": ve,
                        "V0_tas_ms": v_grid[i], "decel_ms2": a,
                        "feasible_whole": bool(fm["whole"][i, j]),
                        "feasible_pre_spun": bool(fm["pre_spun"][i, j]),
                        "first_binding_pre_spun": fb[0] if fb else "",
                        "first_binding_time_s": fb[1] if fb else "",
                    })  # fmt: skip
        out["tables"]["A_feasibility"] = rows
        out["maps"] = maps
        mx: Table = []
        for name in c.scenarios:
            sc = inp.scenarios[name]
            for ve in c.max_decel_v0_eas_ms:
                a = zero_loss.max_feasible_decel(
                    sc.vehicle_for(vp), sc, tas(ve, sc), pre_spun=True, n=c.samples
                )
                mx.append({"scenario": name, "altitude_m": sc.altitude_amsl_m,
                           "V0_eas_ms": ve, "max_feasible_decel_ms2_pre_spun": a})  # fmt: skip
        out["tables"]["A_max_feasible_decel"] = mx

    def stage_d() -> None:
        c = inp.study.D
        bp = inp.controller
        rows: Table = []
        for name in c.scenarios:
            for mf in c.mass_factors:
                sc = dataclasses.replace(inp.scenarios[name], mass_kg=vp.m * mf)
                for ve in c.v0_eas_ms:
                    v0 = tas(ve, sc)
                    r = braking.measure_a_eq(vp, sc, v0, bp)
                    m = r.sim.metrics
                    rows.append({
                        "scenario": name, "altitude_m": sc.altitude_amsl_m, "mass_kg": sc.mass_kg,
                        "V0_eas_ms": ve, "V0_ground_ms": v0, "a_eq_ms2": r.a_eq,
                        "x_actual_m": r.x_actual, "t_stop_s": r.t_stop,
                        "altitude_excursion_m": r.altitude_excursion,
                        "alt_gain_m": m["altitude_gain"], "alt_loss_m": m["altitude_loss"],
                        "peak_bus_current_A": m["peak_bus_current"],
                        "min_bus_voltage_V": m["min_bus_voltage"],
                    })  # fmt: skip
        out["tables"]["D_a_eq"] = rows
        rec = c.recommendation
        scens = _ensemble(inp, rec.ensemble_n, rec.ensemble_seed)
        a_rec, rec_rows = braking.recommend_decel(vp, scens, rec.v0_ground_ms, bp)
        out["a_rec"] = a_rec
        if rec.percentile is not None:
            a_pct, _ = braking.recommend_decel(
                vp, scens, rec.v0_ground_ms, bp, percentile=rec.percentile
            )
            out["a_rec_pct"], out["pct"] = a_pct, rec.percentile
        out["tables"]["D_recommendation_ensemble"] = [
            {"altitude_m": s.altitude_amsl_m, "mass_kg": s.mass_kg, "headwind_ms": s.headwind_ms,
             "V0_ground_ms": v0, "a_eq_ms2": a}
            for s, v0, a in rec_rows
        ]  # fmt: skip
        sc0 = inp.scenarios[c.scenarios[0]]
        v0g = tas(inp.entry_eas, sc0)
        trade: Table = []
        for a_plan in c.trade.a_plan_ms2:
            for wind in c.trade.headwinds_ms:
                for test in ("ground", "air") if wind > 0 else ("ground",):
                    for a_lim in sorted({bp.a_lim, a_plan}):
                        scw = dataclasses.replace(sc0, headwind_ms=wind)
                        bpw = dataclasses.replace(bp, a_lim=a_lim)
                        r = braking.landing_error(vp, scw, v0g, a_plan, bpw, speed_test=test)
                        v_ref = v0g if test == "ground" else v0g + wind
                        trade.append({
                            "a_plan_ms2": a_plan, "a_lim_ms2": a_lim, "speed_test": test,
                            "headwind_ms": wind, "x_plan_m": v_ref**2 / (2.0 * a_plan),
                            "x_actual_m": r.x_actual, "overshoot_m": r.overshoot,
                            "energy_J": r.sim.metrics["energy_J"],
                            "altitude_excursion_m": r.altitude_excursion,
                        })  # fmt: skip
        out["tables"]["D_trade_curve"] = trade

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
        a_plan = float(out.get("a_rec", 2.5))
        budgets: dict[str, Any] = {}
        ranges = sorted({*inp.study.E.ranges_km, *inp.requirements_km})
        for alt in inp.study.E.cruise_amsl_m:
            name = f"cruise_{alt:.0f}m"
            mp = dataclasses.replace(inp.mission, cruise_amsl_m=alt)
            try:
                b = energy.build_budget(vp, mp, inp.window, inp.descent, a_plan)
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
                "a_plan_ms2": a_plan,
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
        rows: Table = []
        eas = inp.mission.v_trans  # AIRSPEED_MIN, read here as an equivalent airspeed (O-02)
        for alt in c.altitudes_m:
            for mf in c.mass_factors:
                sc = Scenario(alt)
                vps = dataclasses.replace(vp, m=vp.m * mf)
                m = pusher_margin(vps, sc.rho, tas(eas, sc), 0.8)
                rows.append({"altitude_m": alt, "rho_kg_m3": sc.rho, "mass_kg": vps.m,
                             "V_trans_eas_ms": eas, "V_trans_tas_ms": tas(eas, sc),
                             **m})  # fmt: skip
        out["tables"]["Q2_pusher_margin"] = rows

    if "A" in wanted:
        _attempt("A zero-loss feasibility", stage_a, errors)
    if "D" in wanted:
        _attempt("D braking and Q_TRANS_DECEL", stage_d, errors)
    if "F" in wanted:
        _attempt("F descent", stage_f, errors)
    if "E" in wanted:
        _attempt("E energy budget", stage_e, errors)
    if "Q2" in wanted:
        _attempt("Q2 pusher margin", stage_q2, errors)
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
