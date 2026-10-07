"""Lift-rotor and pusher thrust, torque and in-plane force (eqs. 6.6-6.7).

Coefficient maps are the PLACEHOLDER polynomials of section 6.3, with inputs clipped to the map
range. Replace with gridded B-spline tables once bench/BEMT data exist.
"""

from __future__ import annotations

import math
from typing import Any

import casadi as ca

from sagetrans.physics.params import LiftRotorParams, PusherParams

Num = Any
TWO_PI = 2.0 * math.pi


def _clip(x: Num, lo: float, hi: float, eps: float = 0.0) -> Num:
    """clip(x, lo, hi); with eps > 0 a smooth (hyperbolic) version for collocation."""
    if eps == 0.0:
        return ca.fmin(ca.fmax(x, lo), hi)
    d = x - lo
    y = lo + 0.5 * (d + ca.sqrt(d * d + eps * eps))  # smooth max(x, lo)
    e = hi - y
    return hi - 0.5 * (e + ca.sqrt(e * e + eps * eps))  # smooth min(y, hi)


def lift_rotor(
    omega: Num, Vc: Num, Ve: Num, rho: Num, p: LiftRotorParams, smooth: float = 0.0
) -> tuple[Num, Num, Num]:
    """Total lift-rotor (T_r, Q_per_rotor, H_r) from speed and inflow components (6.6, 6.7)."""
    w_s = ca.sqrt(omega * omega + p.omega_floor**2)
    n_s = w_s / TWO_PI
    J = _clip(Vc / (n_s * p.D), p.J_min, p.J_max, smooth)
    mu = _clip(Ve / (w_s * p.R), 0.0, p.mu_max, smooth)
    n = omega / TWO_PI
    f_mu = 1.0 + p.k_mu * mu * mu
    ct = (p.CT0 + p.CT1 * J + p.CT2 * J * J) * f_mu
    cq = (p.CQ0 + p.CQ1 * J + p.CQ2 * J * J) * f_mu
    ch = p.CH1 * mu
    n2 = n * n
    T = ct * rho * n2 * p.D**4
    Q = cq * rho * n2 * p.D**5
    H = ch * rho * n2 * p.D**4
    return p.n_r * T, Q, p.n_r * H


def pusher(
    omega: Num, V_axial: Num, rho: Num, p: PusherParams, smooth: float = 0.0
) -> tuple[Num, Num]:
    """Pusher (T_p, Q_p) with one-dimensional coefficients in J_p = V cos(alpha)/(n D)."""
    w_s = ca.sqrt(omega * omega + p.omega_floor**2)
    Jp = _clip(V_axial / (w_s / TWO_PI * p.D), p.J_min, p.J_max, smooth)
    n = omega / TWO_PI
    ct = p.CT0 + p.CT1 * Jp + p.CT2 * Jp * Jp
    cq = p.CQ0 + p.CQ1 * Jp
    return ct * rho * n * n * p.D**4, cq * rho * n * n * p.D**5


def vrs_ratio(Vc: Num, T_r: Num, rho: Num, p: LiftRotorParams) -> Num:
    """V_c / v_h with v_h = sqrt(T_r / (2 rho A_tot)); the avoidance bound is sourced elsewhere."""
    v_h = ca.sqrt(ca.fmax(T_r, 1e-6) / (2.0 * rho * p.A_tot))
    return Vc / v_h


def ideal_hover_power(T: float, rho: float, A: float) -> float:
    """Momentum-theory hover power T^(3/2) / sqrt(2 rho A)."""
    return float(T**1.5 / math.sqrt(2.0 * rho * A))
