"""Forward acceleration the pusher can deliver from rest to a target airspeed (analysis Q2).

The controller matches the rotors to whatever lift the wing does not provide, so the question is
the forward acceleration at `TKOFF_THR_MAX`. At each airspeed the wing carries E's share of the
weight (`energy.rotor_share`), the rotors carry the rest and load the same battery bus as the
pusher, and the pusher speed solves motor torque = propeller torque at the loaded bus voltage.
Quasi-steady: level flight, no spool dynamics, no wind.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np
from scipy.optimize import brentq

from sagetrans.analysis import energy
from sagetrans.dynamics import trim
from sagetrans.physics import battery, motor, rotor
from sagetrans.physics.params import VehicleParams

G = 9.80665


@dataclass(frozen=True)
class FwdAccel:
    V: np.ndarray  # true airspeed grid, 0 .. v_end [m/s]
    a: np.ndarray  # net forward acceleration [m/s^2]
    a_min: float  # minimum over the grid (in practice at v_end)
    v_at_min: float
    a_equiv: float  # constant a covering the same distance, v_end^2 / (2 x); NaN unless reachable
    t_s: float  # time from rest to v_end along the profile; NaN unless reachable
    x_m: float  # distance, same
    v_reach: float  # highest grid speed reached with a > 0 continuously from rest
    reachable: bool  # a > 0 over the whole grid
    rotor_saturated: bool  # lift rotors hit trim.OMEGA_MAX somewhere
    current_limited: bool  # pusher or rotor current limit active somewhere


def _point(
    vp: VehicleParams, rho: float, V: float, soc: float, cmd: float
) -> tuple[float, bool, bool]:
    """Net forward acceleration at airspeed V: pusher at `cmd`, rotors carrying W - L."""
    b, mr, mp = vp.battery, vp.motor_r, vp.motor_p
    w_n = vp.m * G
    share = energy.rotor_share(vp, rho, V)  # fraction of the weight the rotors carry
    t_r = share * w_n
    om_r = trim.rotor_speed_for_thrust(t_r, 0.0, V, rho, vp) if t_r > 0.0 else 0.0
    sat = t_r > 0.0 and om_r >= trim.OMEGA_MAX - 1e-9
    if om_r > 0.0:
        _, q_r, h_r = (float(x) for x in rotor.lift_rotor(om_r, 0.0, V, rho, vp.rotor))
        i_r = q_r / mr.Kt + mr.I0  # per-rotor current
    else:
        h_r = i_r = 0.0
    drag = energy.wing_polar_drag(vp, rho, V, (1.0 - share) * w_n) if V > 0.5 else 0.0
    d_p = float(motor.effective_throttle(cmd, mp))  # ESC dead zone: 0.8 -> 0.789
    voc = float(b.voc(soc))

    def bus(w_p: float) -> float:
        vb = voc
        for _ in range(8):  # the rotor duty depends on the bus voltage
            d_r = (mr.Ke * om_r + mr.Rm * i_r) / max(vb, 1.0) if om_r > 0.0 else 0.0
            vb = float(
                battery.solve_bus_voltage(voc, 0.0, b, vp.rotor.n_r, mr, mp, d_r, d_p, om_r, w_p)
            )
        return vb

    def resid(w_p: float) -> float:
        i_p = float(motor.current(d_p, bus(w_p), w_p, mp))
        return float(motor.torque(i_p, mp)) - float(rotor.pusher(w_p, V, rho, vp.pusher)[1])

    w_p = float(brentq(resid, 1.0, 4000.0, xtol=1e-9))
    t_p = float(rotor.pusher(w_p, V, rho, vp.pusher)[0])
    i_p = float(motor.current(d_p, bus(w_p), w_p, mp))
    limited = i_p >= mp.I_lim - 1e-6 or i_r >= mr.I_lim - 1e-6
    return (t_p - drag - h_r) / vp.m, sat, limited


def forward_accel(
    vp: VehicleParams, rho: float, v_end: float, soc: float, cmd: float, n: int = 41
) -> FwdAccel:
    """Acceleration profile over `np.linspace(0, v_end, n)` (`v_end` is a TRUE airspeed)."""
    speeds = np.linspace(0.0, v_end, n)
    a = np.empty(n)
    sat = limited = False
    for i, v in enumerate(speeds):
        a[i], s, lim = _point(vp, rho, float(v), soc, cmd)
        sat |= s
        limited |= lim
    i_min = int(np.argmin(a))
    ok = bool((a > 0.0).all())
    if ok:
        t = float(np.trapezoid(1.0 / a, speeds))
        x = float(np.trapezoid(speeds / a, speeds))
        a_eq = v_end**2 / (2.0 * x)
    else:
        t = x = a_eq = math.nan
    bad = np.flatnonzero(a <= 0.0)
    v_reach = v_end if ok else (float(speeds[bad[0] - 1]) if bad[0] > 0 else 0.0)
    return FwdAccel(
        speeds, a, float(a[i_min]), float(speeds[i_min]), a_eq, t, x, v_reach, ok, sat, limited
    )
