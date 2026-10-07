"""Motor and ESC electrical/mechanical model (eqs. 6.8-6.9).

`smooth` is the smoothing parameter of section 6.5 in amps: 0 gives the exact clip used by
simulation, a positive value the smooth version used by collocation.
"""

from __future__ import annotations

from typing import Any

import casadi as ca

from sagetrans.physics.params import MotorParams

Num = Any


def effective_throttle(delta: Num, p: MotorParams, smooth: float = 0.0) -> Num:
    """ESC dead zone: zero below the dead-zone fraction, rescaled to reach 1.

    `smooth` > 0 rounds the two corners (dimensionless width, for collocation)."""
    x = (delta - p.dead_zone) / (1.0 - p.dead_zone)
    if smooth == 0.0:
        return ca.fmin(ca.fmax(x, 0.0), 1.0)
    y = 0.5 * (x + ca.sqrt(x * x + smooth * smooth))
    e = 1.0 - y
    return 1.0 - 0.5 * (e + ca.sqrt(e * e + smooth * smooth))


def _smax0(x: Num, eps: float) -> tuple[Num, Num]:
    """max(x, 0) and its slope."""
    if eps == 0.0:
        return ca.fmax(x, 0.0), 0.5 * (1.0 + ca.sign(x))
    r = ca.sqrt(x * x + eps * eps)
    return 0.5 * (x + r), 0.5 * (1.0 + x / r)


def clipped_current(raw: Num, I_lim: float, smooth: float = 0.0) -> tuple[Num, Num]:
    """clip(raw, 0, I_lim) and d(clip)/d(raw)."""
    y, dy = _smax0(raw, smooth)
    z, dz = _smax0(I_lim - y, smooth)  # I_lim - min(y, I_lim)
    return I_lim - z, dz * dy


def current(delta_eff: Num, V_bus: Num, omega: Num, p: MotorParams, smooth: float = 0.0) -> Num:
    """Motor current I_m = clip((delta V_bus - K_e omega)/R_m, 0, I_lim) (6.8)."""
    raw = (delta_eff * V_bus - p.Ke * omega) / p.Rm
    return clipped_current(raw, p.I_lim, smooth)[0]


def torque(I_m: Num, p: MotorParams, smooth: float = 0.0) -> Num:
    """Q_m = K_t (I_m - I_0) for I_m > 0, else 0."""
    if smooth == 0.0:
        step = ca.sign(I_m)
    else:
        step = ca.tanh(I_m / smooth)
    return p.Kt * (I_m - p.I0 * step)


def no_load_speed(delta_eff: float, V_bus: float, p: MotorParams) -> float:
    """(V_m - I_0 R_m)/K_e."""
    return (delta_eff * V_bus - p.I0 * p.Rm) / p.Ke
