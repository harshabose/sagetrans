"""Shared helpers for the ensemble analyses (B2, C): vehicle cache, entry state, run metrics."""

from __future__ import annotations

import math
from functools import lru_cache

import numpy as np

from sagetrans.analysis.common import Scenario
from sagetrans.analysis.ensemble import RunMetrics
from sagetrans.dynamics import events as ev
from sagetrans.dynamics import trim
from sagetrans.dynamics.simulate import Result
from sagetrans.physics import aero
from sagetrans.physics.params import VehicleParams, WingParams
from sagetrans.physics.vehicle import CasadiVehicle

STALL_TIME_LIMIT = 0.3  # s of alpha_w > alpha_s that counts as a stall failure


@lru_cache(maxsize=64)
def vehicle_for(vp: VehicleParams) -> CasadiVehicle:
    return CasadiVehicle(vp)


@lru_cache(maxsize=16)
def _cl_max(w: WingParams) -> float:
    return aero.cl_max(w)


def stall_speed(vp: VehicleParams, rho: float) -> float:
    return math.sqrt(2.0 * vp.m * 9.80665 / (rho * vp.wing.S * _cl_max(vp.wing)))


def cruise_entry_state(vp: VehicleParams, scen: Scenario, v_air: float) -> np.ndarray:
    """Level steady cruise at airspeed v_air, rotors stopped, pusher windmilling at trim speed."""
    tr = trim.steady_trim(vp, scen.rho, v_air, 0.0, scen.soc)
    v_ground = v_air - scen.headwind_ms
    return np.array([0.0, scen.altitude_amsl_m, v_ground, 0.0, tr.theta, 0.0, 0.0, tr.omega_p, 0.0])


def end_events(vp: VehicleParams, scen: Scenario, v_f: float, agl0: float) -> list[ev.Event]:
    return [
        ev.ground_speed_below(v_f),
        ev.battery_cutoff(vp.battery.V_min),
        ev.touchdown(scen.altitude_amsl_m - agl0),
    ]


def failure_flags(sim: Result, vp: VehicleParams, mask: np.ndarray | None = None) -> list[str]:
    """Failure events of one run: stall, battery cut-off, crash, did not stop."""
    flags: list[str] = []
    if sim.terminated_by == "battery_cutoff":
        flags.append("battery_cutoff")
    elif sim.terminated_by == "touchdown":
        flags.append("touchdown")
    elif sim.terminated_by != "ground_speed_reached":
        flags.append("not_stopped")
    aw = sim["alpha"] + vp.wing.i_w
    # a stalled wing matters only while it still has to carry load: V at or above the 1g stall
    # speed (below it the rotors carry the weight by design)
    v_s = np.sqrt(2.0 * vp.m * 9.80665 / (sim["rho"] * vp.wing.S * _cl_max(vp.wing)))
    over = (np.abs(aw) > vp.wing.alpha_s) & (sim["V"] >= v_s)
    if mask is not None:
        over = over & mask
    dt = float(sim.t[1] - sim.t[0]) if len(sim.t) > 1 else 0.0
    if float(over.sum()) * dt > STALL_TIME_LIMIT:
        flags.append("stall")
    return flags


def segment_stats(sim: Result, mask: np.ndarray, h_ref: float) -> tuple[float, float, float]:
    """(altitude excursion, bus energy, ground distance) over the masked samples."""
    idx = np.where(mask)[0]
    if idx.size < 2:
        return 0.0, 0.0, 0.0
    sl = slice(int(idx[0]), int(idx[-1]) + 1)
    h = sim["h"][sl]
    exc = max(0.0, h_ref - float(h.min()), float(h.max()) - h_ref)
    p = sim["V_bus"][sl] * sim["I_bus"][sl]
    e = float(np.sum(0.5 * (p[1:] + p[:-1]) * np.diff(sim.t[sl])))
    return exc, e, float(sim["x"][sl][-1] - sim["x"][sl][0])


def run_metrics(sim: Result, vp: VehicleParams, h_ref: float) -> RunMetrics:
    """Whole-run metrics for a single back-transition."""
    mask = np.ones(len(sim.t), dtype=bool)
    exc, e, d = segment_stats(sim, mask, h_ref)
    return RunMetrics(exc, e, d, tuple(failure_flags(sim, vp)))
