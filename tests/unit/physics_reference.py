"""Plain NumPy/math reference implementation used only by the analytical-limit tests (T-15)."""

from __future__ import annotations

import math

import numpy as np
from scipy.optimize import brentq

from sagetrans.physics.params import (
    BatteryParams,
    LiftRotorParams,
    MotorParams,
    WingParams,
)


def wing(alpha_w: float, p: WingParams) -> tuple[float, float]:
    a = math.exp(-p.M * (alpha_w - p.alpha_s))
    b = math.exp(p.M * (alpha_w + p.alpha_s))
    s = (1 + a + b) / ((1 + a) * (1 + b))
    cl_lin = p.CL0 + p.CL_alpha * alpha_w
    cl = (1 - s) * cl_lin + s * p.CN * math.sin(alpha_w) * math.cos(alpha_w)
    cd = (1 - s) * (p.CD0 + p.k * cl_lin**2) + s * (p.CD0 + p.CN * math.sin(alpha_w) ** 2)
    return cl, cd


def lift_rotor(
    omega: float, Vc: float, Ve: float, rho: float, p: LiftRotorParams
) -> tuple[float, float, float]:
    w_s = math.hypot(omega, p.omega_floor)
    J = float(np.clip(Vc / (w_s / (2 * math.pi) * p.D), p.J_min, p.J_max))
    mu = float(np.clip(Ve / (w_s * p.R), 0.0, p.mu_max))
    n = omega / (2 * math.pi)
    f = 1 + p.k_mu * mu**2
    ct = (p.CT0 + p.CT1 * J + p.CT2 * J**2) * f
    cq = (p.CQ0 + p.CQ1 * J + p.CQ2 * J**2) * f
    return (
        p.n_r * ct * rho * n * n * p.D**4,
        cq * rho * n * n * p.D**5,
        p.n_r * p.CH1 * mu * rho * n * n * p.D**4,
    )


def motor_current(delta_eff: float, V: float, omega: float, p: MotorParams) -> float:
    return float(np.clip((delta_eff * V - p.Ke * omega) / p.Rm, 0.0, p.I_lim))


def motor_torque(I_m: float, p: MotorParams) -> float:
    return p.Kt * (I_m - p.I0) if I_m > 0 else 0.0


def bus_voltage_closed_form(
    voc: float, v1: float, bp: BatteryParams, n_r: int, mr: MotorParams, mp: MotorParams,
    dr: float, dp: float, wr: float, wp: float,
) -> float:  # fmt: skip
    num = voc - v1 + bp.R0 * (n_r * dr * mr.Ke * wr / mr.Rm + dp * mp.Ke * wp / mp.Rm - bp.I_av)
    den = 1 + bp.R0 * (n_r * dr**2 / mr.Rm + dp**2 / mp.Rm)
    return num / den


def bus_voltage_iterative(
    voc: float, v1: float, bp: BatteryParams, n_r: int, mr: MotorParams, mp: MotorParams,
    dr: float, dp: float, wr: float, wp: float,
) -> float:  # fmt: skip
    def g(V: float) -> float:
        i = (
            n_r * dr * motor_current(dr, V, wr, mr)
            + dp * motor_current(dp, V, wp, mp)
            + bp.I_av
        )
        return V - (voc - v1 - bp.R0 * i)

    return float(brentq(g, 0.0, voc - v1, xtol=1e-13, rtol=1e-14))
