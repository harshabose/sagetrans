"""International Standard Atmosphere with a temperature offset, valid to 11,000 m (eq. 6.1).

Written with plain arithmetic only, so every function accepts floats, NumPy arrays and CasADi
SX/MX expressions alike (NFR-08: one definition used by simulation and collocation). Pressure
follows the ISA profile; the offset shifts temperature, and so density, as in eq. (6.1).
"""

from __future__ import annotations

from typing import Any

import numpy as np

# `Any` here means "float, ndarray or CasADi expression"; arithmetic is the only thing used.
Num = Any

G0 = 9.80665  # m/s^2
R_AIR = 287.053  # J/(kg K)
GAMMA_AIR = 1.4
T0 = 288.15  # K
P0 = 101325.0  # Pa
LAPSE = 0.0065  # K/m
P_EXPONENT = 5.25588
H_MAX = 11_000.0  # m, top of the troposphere


def _check_range(h: Num) -> None:
    # symbolic arguments cannot be checked; only numeric ones are
    if isinstance(h, int | float | np.ndarray | np.generic):
        if np.any(np.asarray(h) < -500.0) or np.any(np.asarray(h) > H_MAX):
            raise ValueError(f"altitude outside the validity range [-500, {H_MAX}] m")


def temperature(h: Num, d_isa: Num = 0.0) -> Num:
    """Static temperature [K] at altitude h [m] with ISA offset d_isa [K]."""
    _check_range(h)
    return T0 - LAPSE * h + d_isa


def pressure(h: Num) -> Num:
    """Static pressure [Pa] at altitude h [m] (ISA profile; the offset does not enter)."""
    _check_range(h)
    return P0 * (1.0 - LAPSE * h / T0) ** P_EXPONENT


def density(h: Num, d_isa: Num = 0.0) -> Num:
    """Air density [kg/m^3]."""
    return pressure(h) / (R_AIR * temperature(h, d_isa))


def speed_of_sound(h: Num, d_isa: Num = 0.0) -> Num:
    """Speed of sound [m/s]."""
    return (GAMMA_AIR * R_AIR * temperature(h, d_isa)) ** 0.5
