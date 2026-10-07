"""Numeric steady-state and inversion helpers (rotor map inversion, throttle for a thrust).

Used by analysis A (zero-loss reference) and by the controllers for a hover-throttle estimate.
These call the same CasADi physics functions as the vehicle model, so there is no second copy.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from scipy.optimize import brentq

from sagetrans.physics import aero, battery, rotor
from sagetrans.physics.params import MotorParams, VehicleParams
from sagetrans.physics.rotor import pusher

OMEGA_MAX = 3000.0  # rad/s, bracket top for the rotor-speed inversion


def rotor_speed_for_thrust(
    thrust_total: float, Vc: float, Ve: float, rho: float, vp: VehicleParams
) -> float:
    """Lift-rotor speed [rad/s] giving total thrust `thrust_total` at the given inflow."""
    if thrust_total <= 0.0:
        return 0.0

    def resid(w: float) -> float:
        return float(rotor.lift_rotor(w, Vc, Ve, rho, vp.rotor)[0]) - thrust_total

    if resid(OMEGA_MAX) < 0.0:
        return OMEGA_MAX
    return float(brentq(resid, 0.0, OMEGA_MAX, xtol=1e-10, rtol=1e-13))


@dataclass(frozen=True)
class RotorDemand:
    delta_cmd: float  # throttle command before the ESC dead zone
    current: float  # motor current [A]
    torque: float  # required motor torque [N m]
    v_bus: float
    omega_dot: float


def required_rotor_command(
    omega: float,
    omega_dot: float,
    Vc: float,
    Ve: float,
    rho: float,
    vp: VehicleParams,
    soc: float,
    v1: float = 0.0,
    delta_p_eff: float = 0.0,
    omega_p: float = 0.0,
) -> RotorDemand:
    """Throttle that produces rotor acceleration `omega_dot` at speed `omega` (eq. 6.9 inverted).

    The bus voltage depends on the throttle through the pack resistance, so the pair is solved
    by fixed-point iteration. A required torque <= 0 returns throttle 0 (the ESC cannot brake).
    """
    m, b = vp.motor_r, vp.battery
    q_i = float(rotor.lift_rotor(omega, Vc, Ve, rho, vp.rotor)[1])
    q_req = q_i + vp.rotor.I_rot * omega_dot
    voc = float(b.voc(soc))
    v_bus = voc - v1
    if q_req <= 0.0:
        v_bus = float(
            battery.solve_bus_voltage(
                voc, v1, b, vp.rotor.n_r, m, vp.motor_p, 0.0, delta_p_eff, omega, omega_p
            )
        )
        return RotorDemand(0.0, 0.0, q_req, v_bus, omega_dot)
    i_m = q_req / m.Kt + m.I0
    delta_eff = 0.0
    for _ in range(60):
        delta_eff = (m.Rm * i_m + m.Ke * omega) / v_bus
        v_new = float(
            battery.solve_bus_voltage(
                voc, v1, b, vp.rotor.n_r, m, vp.motor_p,
                min(delta_eff, 1.0), delta_p_eff, omega, omega_p,
            )
        )  # fmt: skip
        if abs(v_new - v_bus) < 1e-10:
            v_bus = v_new
            break
        v_bus = v_new
    delta_eff = (m.Rm * i_m + m.Ke * omega) / v_bus
    cmd = m.dead_zone + (1.0 - m.dead_zone) * delta_eff
    return RotorDemand(cmd, i_m, q_req, v_bus, omega_dot)


def hover_throttle(vp: VehicleParams, rho: float, soc: float, v1: float = 0.0) -> float:
    """Throttle command (pre dead zone) that hovers the aircraft in still air."""
    thrust = vp.m * 9.80665
    omega = rotor_speed_for_thrust(thrust, 0.0, 0.0, rho, vp)
    return required_rotor_command(omega, 0.0, 0.0, 0.0, rho, vp, soc, v1).delta_cmd


def hover_omega(vp: VehicleParams, rho: float) -> float:
    return rotor_speed_for_thrust(vp.m * 9.80665, 0.0, 0.0, rho, vp)


# ---- steady flight and electrical power (analyses E, F) ----------------------------------------


def pusher_speed_for_thrust(thrust: float, v_axial: float, rho: float, vp: VehicleParams) -> float:
    """Pusher speed [rad/s] giving `thrust` at axial inflow `v_axial`; 0 for thrust <= 0."""
    if thrust <= 0.0:
        return 0.0

    def resid(w: float) -> float:
        return float(pusher(w, v_axial, rho, vp.pusher)[0]) - thrust

    if resid(OMEGA_MAX) < 0.0:
        return OMEGA_MAX
    return float(brentq(resid, 0.0, OMEGA_MAX, xtol=1e-10, rtol=1e-13))


@dataclass(frozen=True)
class ElectricalState:
    """Electrical operating point from demanded motor currents (no clipping assumed)."""

    v_bus: float
    i_bus: float
    power: float
    delta_r_cmd: float
    delta_p_cmd: float
    i_rotor: float
    i_pusher: float
    feasible: bool  # currents within limits and throttles <= 1 at this bus voltage


def _cmd(vp: VehicleParams, i_m: float, omega: float, m: MotorParams, v_bus: float) -> float:
    if i_m <= 0.0:
        return 0.0
    delta_eff = (m.Rm * i_m + m.Ke * omega) / v_bus
    return m.dead_zone + (1.0 - m.dead_zone) * delta_eff


def electrical_state(
    vp: VehicleParams,
    soc: float,
    v1: float,
    omega_r: float,
    q_rotor: float,
    omega_p: float,
    q_pusher: float,
) -> ElectricalState:
    """Bus voltage and power for given motor speeds and load torques.

    Motor current is set by torque alone, I = Q/K_t + I_0 (6.8). The ESC passes the motor power
    to the bus, I_bus = P_m / V_bus + I_av, so V_bus = V_oc - v1 - R_0 I_bus is a quadratic with
    an explicit root. The throttle follows from the motor voltage, delta = (R_m I + K_e w)/V_bus.
    """
    mr, mp, b = vp.motor_r, vp.motor_p, vp.battery
    i_r = q_rotor / mr.Kt + mr.I0 if q_rotor > 0.0 else 0.0
    i_p = q_pusher / mp.Kt + mp.I0 if q_pusher > 0.0 else 0.0
    p_m = vp.rotor.n_r * (mr.Rm * i_r + mr.Ke * omega_r) * i_r if i_r > 0.0 else 0.0
    if i_p > 0.0:
        p_m += (mp.Rm * i_p + mp.Ke * omega_p) * i_p
    big_b = float(b.voc(soc)) - v1 - b.R0 * b.I_av
    disc = big_b * big_b - 4.0 * b.R0 * p_m
    transferable = disc >= 0.0  # else the pack cannot deliver this motor power at all
    v_bus = 0.5 * (big_b + math.sqrt(max(disc, 0.0)))
    i_bus = p_m / v_bus + b.I_av
    d_r = _cmd(vp, i_r, omega_r, mr, v_bus)
    d_p = _cmd(vp, i_p, omega_p, mp, v_bus)
    ok = (
        transferable and i_r <= mr.I_lim and i_p <= mp.I_lim
        and d_r <= 1.0 and d_p <= 1.0 and v_bus >= b.V_min
    )  # fmt: skip
    return ElectricalState(v_bus, i_bus, v_bus * i_bus, d_r, d_p, i_r, i_p, ok)


def hover_state(vp: VehicleParams, rho: float, soc: float, v1: float = 0.0) -> ElectricalState:
    """Electrical operating point of a steady hover."""
    omega = hover_omega(vp, rho)
    q = float(rotor.lift_rotor(omega, 0.0, 0.0, rho, vp.rotor)[1])
    return electrical_state(vp, soc, v1, omega, q, 0.0, 0.0)


def hover_power(vp: VehicleParams, rho: float, soc: float, v1: float = 0.0) -> float:
    """Electrical bus power [W] in hover."""
    return hover_state(vp, rho, soc, v1).power


@dataclass(frozen=True)
class SteadyTrim:
    """Steady wings-and-pusher flight (rotors off): the exact force balance (7.2)-(7.3)."""

    V: float
    gamma: float  # flight-path angle, positive climbing
    alpha: float  # body pitch relative to the velocity vector (= theta - gamma)
    T_p: float
    L: float
    D: float
    omega_p: float
    elec: ElectricalState

    @property
    def theta(self) -> float:
        return self.alpha + self.gamma


def _pusher_electrical(
    vp: VehicleParams, rho: float, v_axial: float, t_p: float, soc: float, v1: float
) -> tuple[float, ElectricalState]:
    w = pusher_speed_for_thrust(t_p, v_axial, rho, vp)
    q = float(pusher(w, v_axial, rho, vp.pusher)[1])
    return w, electrical_state(vp, soc, v1, 0.0, 0.0, w, q)


def steady_trim(
    vp: VehicleParams, rho: float, V: float, gamma: float, soc: float, v1: float = 0.0
) -> SteadyTrim:
    """Level (gamma = 0) or steady-climb trim: solve alpha so lift and thrust balance weight.

    With T_p = (D + W sin(gamma)) / cos(alpha) the vertical balance reduces to
    L + (D + W sin gamma) tan(alpha) = W cos gamma, solved for alpha by bracketing.
    """
    w_n = vp.m * 9.80665
    iw = vp.wing.i_w

    def forces(alpha: float) -> tuple[float, float]:
        L, D = aero.forces(rho, V, alpha + iw, vp.wing)
        return float(L), float(D)

    def resid(alpha: float) -> float:
        L, D = forces(alpha)
        return L + (D + w_n * math.sin(gamma)) * math.tan(alpha) - w_n * math.cos(gamma)

    lo, hi = -0.1, vp.wing.alpha_s - iw
    if resid(lo) * resid(hi) > 0.0:
        raise ValueError(f"no steady trim at V = {V:.2f} m/s (wing cannot carry the weight)")
    alpha = float(brentq(resid, lo, hi, xtol=1e-13, rtol=1e-14))
    L, D = forces(alpha)
    t_p = (D + w_n * math.sin(gamma)) / math.cos(alpha)
    w, es = _pusher_electrical(vp, rho, V * math.cos(alpha), t_p, soc, v1)
    return SteadyTrim(V, gamma, alpha, t_p, L, D, w, es)


def glide_trim(vp: VehicleParams, rho: float, V: float, soc: float, v1: float = 0.0) -> SteadyTrim:
    """Pusher-off glide at airspeed V: D = -W sin(gamma), L = W cos(gamma)."""
    w_n = vp.m * 9.80665
    iw = vp.wing.i_w

    def gam(alpha: float) -> float:
        d = float(aero.forces(rho, V, alpha + iw, vp.wing)[1])
        return -math.asin(min(1.0, d / w_n))

    def resid(alpha: float) -> float:
        L = float(aero.forces(rho, V, alpha + iw, vp.wing)[0])
        return L - w_n * math.cos(gam(alpha))

    lo, hi = -0.1, vp.wing.alpha_s - iw
    if resid(lo) * resid(hi) > 0.0:
        raise ValueError(f"no glide trim at V = {V:.2f} m/s")
    alpha = float(brentq(resid, lo, hi, xtol=1e-13, rtol=1e-14))
    L, D = (float(v) for v in aero.forces(rho, V, alpha + iw, vp.wing))
    es = electrical_state(vp, soc, v1, 0.0, 0.0, 0.0, 0.0)
    return SteadyTrim(V, gam(alpha), alpha, 0.0, L, D, 0.0, es)
