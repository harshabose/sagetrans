"""First-order equivalent-circuit pack model and bus-voltage solution (eqs. 6.10-6.11)."""

from __future__ import annotations

from typing import Any

import casadi as ca

from sagetrans.physics import motor
from sagetrans.physics.params import BatteryParams, MotorParams

Num = Any

NEWTON_ITERATIONS = 12


def closed_form_bus_voltage(
    voc: Num,
    v1: Num,
    bp: BatteryParams,
    n_r: int,
    mr: MotorParams,
    mp: MotorParams,
    delta_r: Num,
    delta_p: Num,
    omega_r: Num,
    omega_p: Num,
) -> Num:
    """Eq. (6.11), corrected for the ESC duty cycle; valid while no current is clipped or zero.

    delta_r, delta_p are effective (post dead-zone) throttles. The ESC chops the bus, so the
    motor sees delta V_bus and the bus carries delta I_m (power balance); the proposal's
    printed form takes the bus current equal to I_m, which overstates it by about 1/delta.
    """
    num = voc - v1 + bp.R0 * (
        n_r * delta_r * mr.Ke * omega_r / mr.Rm + delta_p * mp.Ke * omega_p / mp.Rm - bp.I_av
    )
    den = 1.0 + bp.R0 * (n_r * delta_r**2 / mr.Rm + delta_p**2 / mp.Rm)
    return num / den


def solve_bus_voltage(
    voc: Num,
    v1: Num,
    bp: BatteryParams,
    n_r: int,
    mr: MotorParams,
    mp: MotorParams,
    delta_r: Num,
    delta_p: Num,
    omega_r: Num,
    omega_p: Num,
    smooth: float = 0.0,
) -> Num:
    """Bus voltage with current clipping: Newton on a monotone scalar equation.

    g(V) = V - (voc - v1 - R0 * I_bus(V)), g' >= 1, so Newton from the unloaded voltage is
    well behaved; iterates are clamped to [0, voc - v1].
    """
    v_hi = voc - v1
    V = v_hi
    for _ in range(NEWTON_ITERATIONS):
        raw_r = (delta_r * V - mr.Ke * omega_r) / mr.Rm
        raw_p = (delta_p * V - mp.Ke * omega_p) / mp.Rm
        i_r, di_r = motor.clipped_current(raw_r, mr.I_lim, smooth)
        i_p, di_p = motor.clipped_current(raw_p, mp.I_lim, smooth)
        i_bus = n_r * delta_r * i_r + delta_p * i_p + bp.I_av
        slope = n_r * di_r * delta_r**2 / mr.Rm + di_p * delta_p**2 / mp.Rm
        g = V - (v_hi - bp.R0 * i_bus)
        V = ca.fmin(ca.fmax(V - g / (1.0 + bp.R0 * slope), 0.0), v_hi)
    return V


def branch_derivative(v1: Num, i_bus: Num, bp: BatteryParams) -> Num:
    """dv1/dt = -v1/(R1 C1) + I_bus/C1 (6.10)."""
    return -v1 / (bp.R1 * bp.C1) + i_bus / bp.C1
