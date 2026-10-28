"""State-of-charge trace along a mission and the feasibility of each segment (analysis E).

The segment energies of `energy.MissionBudget` use one mean state of charge. Here the SoC is
walked along the mission from the cumulative energy, and every segment is checked at BOTH its start
and end SoC (the bus voltage falls with the charge, so the same thrust needs a higher command) with
the worse result reported. A segment FAILS only when its operating point is infeasible: a pusher or
rotor command above 1, a motor current limit, a bus voltage below `V_min`, or a pack that cannot
deliver the power. A pusher command above `TKOFF_THR_MAX` (the ceiling) is REPORTED, not a failure.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scipy.optimize import brentq

from sagetrans import atmosphere
from sagetrans.analysis import energy, pusher
from sagetrans.dynamics import trim
from sagetrans.physics.params import VehicleParams

FLIGHT_ORDER = [
    "takeoff", "fwd_transition", "climb", "cruise", "descent", "back_transition", "landing",
]  # fmt: skip
ROC_SEARCH_MS = 5.0  # upper end of the rate-of-climb search


@dataclass(frozen=True)
class TraceRow:
    leg: str
    segment: str
    energy_J: float
    soc_start: float
    soc_end: float


@dataclass(frozen=True)
class SegmentCheck:
    leg: str
    segment: str
    soc_start: float
    soc_end: float
    energy_Wh: float
    feasible: bool  # at both ends
    limit: str  # "" when feasible; else the first limit hit
    pusher_cmd_max: float  # max over the two SoC ends; 0 for segments without the pusher
    rotor_cmd_max: float  # NaN where it is not evaluated (the transitions)
    over_ceiling: bool  # pusher_cmd_max > tkoff_thr_max  (REPORT ONLY)
    roc_at_ceiling_ms: float  # climb rows: best rate of climb at the ceiling; NaN if none


def limit_reason(vp: VehicleParams, es: trim.ElectricalState) -> str:
    """The first limit an infeasible electrical state hits."""
    if es.delta_p_cmd > 1.0:
        return "pusher command > 1"
    if es.delta_r_cmd > 1.0:
        return "rotor command > 1"
    if es.i_pusher > vp.motor_p.I_lim:
        return "pusher current limit"
    if es.i_rotor > vp.motor_r.I_lim:
        return "rotor current limit"
    if es.v_bus < vp.battery.V_min:
        return "bus voltage < V_min"
    return "pack cannot deliver this power"


def trace(
    mb: energy.MissionBudget, r_m: float, soc_start: float, e_nom_J: float
) -> tuple[list[TraceRow], float, float]:
    """Walk the legs in flight order; returns the rows, the SoC before and after the reserve.

    The cruise energy of a leg is `(r_m - d_leg) e_cruise`; the drop hover sits between the legs.
    """
    rows: list[TraceRow] = []
    soc = soc_start

    def add(leg: str, seg: str, e: float) -> None:
        nonlocal soc
        rows.append(TraceRow(leg, seg, e, soc, soc - e / e_nom_J))
        soc -= e / e_nom_J

    for i, (name, _, b) in enumerate(mb.legs):
        for seg in FLIGHT_ORDER:
            if seg == "cruise":
                add(name, seg, (r_m - b.fixed_distance_m) * b.e_cruise_J_per_m)
            else:
                add(name, seg, b.segments[seg].energy_J)
        if i == 0 and mb.drop_J > 0.0 and len(mb.legs) > 1:
            add(name, "drop", mb.drop_J)
    return rows, soc, soc - mb.reserve_J / e_nom_J


def _check_at(
    vp: VehicleParams, mp: energy.MissionProfile, name: str, soc: float, ceiling: float
) -> tuple[bool, str, float, float]:
    """(feasible, limit, pusher command, rotor command) of one segment at one SoC."""
    rho_site = float(atmosphere.density(mp.ground_amsl_m, mp.isa_offset_k))
    try:
        if name in ("takeoff", "landing"):
            es = trim.hover_state(vp, rho_site, soc)
        elif name == "climb":
            dh = mp.cruise_amsl_m - mp.clearance_amsl
            if dh <= 0.0:
                return True, "", 0.0, 0.0
            v = energy.climb_speed(mp)
            rho = float(atmosphere.density(0.5 * (mp.cruise_amsl_m + mp.clearance_amsl),
                                           mp.isa_offset_k))  # fmt: skip
            es = trim.steady_trim(vp, rho, v, math.asin(mp.roc / v), soc).elec
        elif name == "cruise":
            rho = float(atmosphere.density(mp.cruise_amsl_m, mp.isa_offset_k))
            assert mp.v_cruise is not None
            es = trim.steady_trim(vp, rho, mp.v_cruise, 0.0, soc).elec
        elif name == "descent":
            if mp.cruise_amsl_m - (mp.ground_amsl_m + mp.h_bt_agl_m) <= 0.0:
                return True, "", 0.0, 0.0
            rho = float(atmosphere.density(0.5 * (mp.cruise_amsl_m + mp.ground_amsl_m),
                                           mp.isa_offset_k))  # fmt: skip
            es = trim.glide_trim(vp, rho, energy.glide_speed(mp), soc).elec
        elif name == "fwd_transition":
            rho = float(atmosphere.density(mp.clearance_amsl, mp.isa_offset_k))
            f = pusher.forward_accel(vp, rho, mp.v_trans, soc, ceiling, 21)
            if not f.reachable:
                why = f"pusher cannot reach AIRSPEED_MIN at TKOFF_THR_MAX (v_reach={f.v_reach:.1f})"
                return False, why, ceiling, math.nan
            if f.rotor_saturated:
                return False, "lift rotors saturated", ceiling, math.nan
            return True, "", ceiling, math.nan
        else:  # back_transition: the D simulation completed or the segment raised
            return True, "", math.nan, math.nan
    except ValueError as exc:  # no steady or glide trim at this speed
        return False, str(exc), math.nan, math.nan
    return es.feasible, "" if es.feasible else limit_reason(vp, es), es.delta_p_cmd, es.delta_r_cmd


def _roc_at_ceiling(
    vp: VehicleParams, mp: energy.MissionProfile, soc: float, ceiling: float
) -> float:
    """Best rate of climb [m/s] with the pusher command at the ceiling, at one SoC."""
    v = energy.climb_speed(mp)
    rho = float(atmosphere.density(0.5 * (mp.cruise_amsl_m + mp.clearance_amsl), mp.isa_offset_k))

    def f(g: float) -> float:
        return trim.steady_trim(vp, rho, v, g, soc).elec.delta_p_cmd - ceiling

    try:
        if f(0.0) > 0.0:
            return math.nan  # level flight already exceeds the ceiling
        g_hi = math.asin(ROC_SEARCH_MS / v)
        if f(g_hi) <= 0.0:
            return ROC_SEARCH_MS
        return v * math.sin(float(brentq(f, 0.0, g_hi, xtol=1e-10)))
    except ValueError:
        return math.nan


def check_mission(
    mb: energy.MissionBudget, rows: list[TraceRow], ceiling: float
) -> list[SegmentCheck]:
    """Check every flown segment of the trace at both SoC ends and keep the worse."""
    out: list[SegmentCheck] = []
    for r in rows:
        if r.segment == "drop":
            continue
        vp, mp = mb.vehicles[r.leg], mb.profiles[r.leg]
        pts = [_check_at(vp, mp, r.segment, soc, ceiling) for soc in (r.soc_start, r.soc_end)]
        bad = next((p for p in pts if not p[0]), None)
        p_cmd = max((p[2] for p in pts if not math.isnan(p[2])), default=math.nan)
        r_cmd = max((p[3] for p in pts if not math.isnan(p[3])), default=math.nan)
        roc = _roc_at_ceiling(vp, mp, r.soc_end, ceiling) if r.segment == "climb" else math.nan
        out.append(SegmentCheck(
            r.leg, r.segment, r.soc_start, r.soc_end, r.energy_J / energy.J_PER_WH,
            bad is None, "" if bad is None else bad[1], p_cmd, r_cmd,
            bool(p_cmd > ceiling) if not math.isnan(p_cmd) else False, roc,
        ))  # fmt: skip
    return out


def summarise(checks: list[SegmentCheck]) -> dict[str, object]:
    """Mission level: flyable, number failing, first failure, number over the ceiling."""
    bad = [c for c in checks if not c.feasible]
    first = f"{bad[0].leg}:{bad[0].segment}: {bad[0].limit}" if bad else ""
    return {
        "flyable": not bad, "n_infeasible": len(bad), "first_infeasible": first,
        "n_over_ceiling": sum(1 for c in checks if c.over_ceiling),
    }  # fmt: skip
