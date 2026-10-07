"""Analysis F: landing descent profile from vortex-ring-state, fragility and sensor limits.

(10.7): v_h = sqrt(T / (2 rho A_tot)), V_z,max = kappa v_h. The fraction kappa must come from a
sourced vortex-ring-state boundary (Johnson, NASA report); it has no default on purpose, so an
unsourced working figure cannot end up in a result.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.dynamics import trim
from sagetrans.physics.params import VehicleParams

G = 9.80665


@dataclass(frozen=True)
class DescentLimits:
    kappa: float | None  # V_z,max / v_h from a sourced VRS boundary; None is rejected
    kappa_source: str = "UNSOURCED"
    v_fragility: float = 3.0  # payload-fragility descent-rate limit, m/s (assumed)
    v_sensor: float | None = None  # rangefinder / flow-sensor limit, m/s
    h_slow: float = 5.0  # below this height the descent slows to v_final
    v_final: float = 0.5  # final descent rate (rangefinder, ground effect), m/s


@dataclass
class DescentProfile:
    h: np.ndarray  # height AGL, descending order
    v: np.ndarray  # descent rate [m/s] at each height
    t_to_go: np.ndarray  # time to touchdown from each height [s]
    e_to_go: np.ndarray  # hover energy to touchdown from each height [J]
    v_h: float
    v_cap: float
    binding: str  # which limit sets the cruise-descent rate
    hover_power: float
    kappa_source: str
    record: RunRecord

    @property
    def time(self) -> float:
        return float(self.t_to_go[0])

    @property
    def energy_J(self) -> float:
        return float(self.e_to_go[0])


def induced_velocity(thrust: float, rho: float, a_tot: float) -> float:
    return math.sqrt(thrust / (2.0 * rho * a_tot))


def descent_profile(
    vp: VehicleParams,
    scen: Scenario,
    h_start: float,
    limits: DescentLimits,
    n: int = 600,
) -> DescentProfile:
    """Descent rate and time against height, and the hover energy for the landing."""
    if limits.kappa is None:
        raise ValueError(
            "kappa (V_z,max / v_h) must come from a sourced vortex-ring-state boundary; "
            "no default is provided"
        )
    rho = scen.rho
    v_h = induced_velocity(vp.m * G, rho, vp.rotor.A_tot)
    caps = {"vortex_ring_state": limits.kappa * v_h, "payload_fragility": limits.v_fragility}
    if limits.v_sensor is not None:
        caps["sensor"] = limits.v_sensor
    binding = min(caps, key=lambda k: caps[k])
    v_cap = caps[binding]
    h = np.linspace(h_start, 0.0, n)
    frac = np.clip(h / limits.h_slow, 0.0, 1.0)
    v = np.minimum(v_cap, limits.v_final + (v_cap - limits.v_final) * frac)
    v = np.where(v_cap < limits.v_final, v_cap, v)
    inv = 1.0 / v
    dt = 0.5 * (inv[1:] + inv[:-1]) * (h[:-1] - h[1:])  # time to descend each interval
    t_to_go = np.concatenate([np.cumsum(dt[::-1])[::-1], [0.0]])
    p_hover = trim.hover_power(vp, rho, scen.soc)
    return DescentProfile(
        h, v, t_to_go, p_hover * t_to_go, v_h, v_cap, binding, p_hover,
        limits.kappa_source, make_record(vp, scen, h_start, limits),
    )  # fmt: skip


def vrs_ratio_at(profile: DescentProfile) -> np.ndarray:
    """V_c / v_h along the profile (negative in descent); thrust taken equal to weight."""
    return -profile.v / profile.v_h


__all__ = ["DescentLimits", "DescentProfile", "descent_profile", "induced_velocity", "vrs_ratio_at"]
