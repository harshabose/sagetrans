"""Second-order pitch-tracking surrogate with rate and angle limits (eq. 6.12)."""

from __future__ import annotations

from typing import Any

import casadi as ca

from sagetrans.physics.params import AttitudeParams

Num = Any


def derivatives(
    theta: Num, q: Num, theta_cmd: Num, theta_lim: Num, p: AttitudeParams
) -> tuple[Num, Num]:
    """(theta_dot, q_dot). theta_lim is the pitch-command magnitude limit theta_max(t)."""
    cmd = ca.fmin(ca.fmax(theta_cmd, -theta_lim), theta_lim)
    qd = p.omega_n**2 * (cmd - theta) - 2.0 * p.zeta * p.omega_n * q
    # rate limit: no further acceleration into the limit
    qd = ca.if_else(ca.logic_and(q >= p.q_max, qd > 0.0), 0.0, qd)
    qd = ca.if_else(ca.logic_and(q <= -p.q_max, qd < 0.0), 0.0, qd)
    return q, qd
