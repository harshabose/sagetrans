"""Wing lift and drag with the smooth stall blend (eqs. 6.2-6.5)."""

from __future__ import annotations

from typing import Any

import casadi as ca
import numpy as np

from sagetrans.physics.params import WingParams

Num = Any


def sigma(alpha_w: Num, p: WingParams) -> Num:
    """Blend weight: 0 in the linear region, 1 far past stall (6.2)."""
    a = ca.exp(-p.M * (alpha_w - p.alpha_s))
    b = ca.exp(p.M * (alpha_w + p.alpha_s))
    return (1.0 + a + b) / ((1.0 + a) * (1.0 + b))


def coefficients(alpha_w: Num, p: WingParams) -> tuple[Num, Num]:
    """(C_L, C_D) at wing angle of attack alpha_w [rad] (6.3)."""
    s = sigma(alpha_w, p)
    cl_lin = p.CL0 + p.CL_alpha * alpha_w
    cl = (1.0 - s) * cl_lin + s * p.CN * ca.sin(alpha_w) * ca.cos(alpha_w)
    cd = (1.0 - s) * (p.CD0 + p.k * cl_lin**2) + s * (p.CD0 + p.CN * ca.sin(alpha_w) ** 2)
    return cl, cd


def forces(rho: Num, V: Num, alpha_w: Num, p: WingParams) -> tuple[Num, Num]:
    """Lift and drag [N] (6.4)."""
    cl, cd = coefficients(alpha_w, p)
    q = 0.5 * rho * V * V * p.S
    return q * cl, q * cd


def cl_max(p: WingParams) -> float:
    """Peak of C_L in (6.3), found on a fine grid."""
    grid = np.linspace(0.0, np.pi / 2, 20001)
    cl = [float(coefficients(float(a), p)[0]) for a in grid[::20]]
    return float(max(cl))


def alpha_at_cl_max(p: WingParams) -> float:
    """Wing angle [rad] of the C_L peak of (6.3): the top of the range where lift still rises.

    The stall blend lowers the peak below `alpha_s` (C_L,max 1.444 at 13.4 deg against alpha_s
    15.7 deg in the register), so a search for a lift-supported trim must stop here, not at
    `alpha_s`, or it reports "no trim" where one exists just below the peak.
    """
    from scipy.optimize import minimize_scalar

    res = minimize_scalar(
        lambda a: -float(coefficients(float(a), p)[0]), bounds=(0.0, p.alpha_s * 1.5),
        method="bounded", options={"xatol": 1e-9},
    )
    return float(res.x)


def stall_speed(m: float, rho: float, p: WingParams, g: float = 9.80665) -> float:
    """1g stall speed (6.5)."""
    return float(np.sqrt(2.0 * m * g / (rho * p.S * cl_max(p))))
