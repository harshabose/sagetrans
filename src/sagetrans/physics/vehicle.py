"""Assembles the sub-models into one compiled CasADi function (NFR-08, eqs. 7.1-7.4).

State x = [x, h, vx, vh, theta, q, omega_r, omega_p, v1]
Input u = [delta_r, delta_p, theta_cmd]
Parameters p = [d_isa, headwind, theta_lim, soc]
"""

from __future__ import annotations

import casadi as ca
import numpy as np

from sagetrans import atmosphere
from sagetrans.physics import aero, attitude, battery, motor, rotor
from sagetrans.physics.params import VehicleParams

STATE_NAMES = ("x", "h", "vx", "vh", "theta", "q", "omega_r", "omega_p", "v1")
INPUT_NAMES = ("delta_r", "delta_p", "theta_cmd")
PARAM_NAMES = ("d_isa", "headwind", "theta_lim", "soc")
AUX_NAMES = (
    "L", "D", "T_r", "T_p", "H_r", "I_bus", "V_bus", "alpha", "V", "gamma",
    "vrs_ratio", "I_rotor", "I_pusher", "rho", "C_L",
)  # fmt: skip
G = atmosphere.G0
SMOOTH_ND = 0.02  # corner rounding of the dimensionless clips when smoothing is on


class CasadiVehicle:
    """The single vehicle definition; `f(x, u, p) -> (xdot, aux)` is a compiled CasADi Function."""

    def __init__(
        self,
        params: VehicleParams,
        smooth: float = 0.0,
        frozen_states: tuple[str, ...] = (),
    ) -> None:
        """`frozen_states` zeroes the named state derivatives: a harness for the analytical-limit
        tests (e.g. vertical motion held at zero), never used in analyses."""
        self.params = params
        self.smooth = smooth
        self.frozen_states = frozen_states
        self._mask = np.array([0.0 if n in frozen_states else 1.0 for n in STATE_NAMES])
        self._rk4_cache: dict[tuple[float, int], ca.Function] = {}
        self.f = self._build()

    def _build(self) -> ca.Function:
        P = self.params
        x = ca.MX.sym("x", 9)
        u = ca.MX.sym("u", 3)
        p = ca.MX.sym("p", 4)
        _, h, vx, vh, theta, q, w_r, w_p, v1 = (x[i] for i in range(9))
        d_r, d_p, th_cmd = u[0], u[1], u[2]
        d_isa, w_h, th_lim, soc = p[0], p[1], p[2], p[3]

        rho = atmosphere.density(h, d_isa)
        va = vx + w_h
        V = ca.sqrt(va * va + vh * vh + 1e-9)
        gamma = ca.atan2(vh, va)
        alpha = theta - gamma
        L, D = aero.forces(rho, V, alpha + P.wing.i_w, P.wing)
        cl, _ = aero.coefficients(alpha + P.wing.i_w, P.wing)

        Vc = vh * ca.cos(theta) - va * ca.sin(theta)
        Ve = va * ca.cos(theta) + vh * ca.sin(theta)
        sm = SMOOTH_ND if self.smooth > 0.0 else 0.0  # dimensionless corner rounding
        T_r, Q_i, H_r = rotor.lift_rotor(w_r, Vc, Ve, rho, P.rotor, sm)
        T_p, Q_p = rotor.pusher(w_p, V * ca.cos(alpha), rho, P.pusher, sm)

        dr_eff = motor.effective_throttle(d_r, P.motor_r, sm)
        dp_eff = motor.effective_throttle(d_p, P.motor_p, sm)
        voc = P.battery.voc(soc)
        V_bus = battery.solve_bus_voltage(
            voc, v1, P.battery, P.rotor.n_r, P.motor_r, P.motor_p,
            dr_eff, dp_eff, w_r, w_p, self.smooth,
        )  # fmt: skip
        I_r = motor.current(dr_eff, V_bus, w_r, P.motor_r, self.smooth)
        I_p = motor.current(dp_eff, V_bus, w_p, P.motor_p, self.smooth)
        # ESC duty cycle: the bus carries delta * I_m (power balance), not I_m
        I_bus = P.rotor.n_r * dr_eff * I_r + dp_eff * I_p + P.battery.I_av

        Qm_r = motor.torque(I_r, P.motor_r, self.smooth)
        Qm_p = motor.torque(I_p, P.motor_p, self.smooth)
        w_r_dot = (Qm_r - Q_i) / P.rotor.I_rot
        w_p_dot = (Qm_p - Q_p) / P.pusher.I_p
        v1_dot = battery.branch_derivative(v1, I_bus, P.battery)

        th_dot, q_dot = attitude.derivatives(theta, q, th_cmd, th_lim, P.attitude)

        sg, cg = ca.sin(gamma), ca.cos(gamma)
        st, ct = ca.sin(theta), ca.cos(theta)
        vx_dot = (T_p * ct - T_r * st - H_r * ct - L * sg - D * cg) / P.m
        vh_dot = (T_p * st + T_r * ct - H_r * st + L * cg - D * sg) / P.m - G

        xdot = ca.vertcat(vx, vh, vx_dot, vh_dot, th_dot, q_dot, w_r_dot, w_p_dot, v1_dot)
        xdot = xdot * ca.DM(self._mask)
        aux = ca.vertcat(
            L, D, T_r, T_p, H_r, I_bus, V_bus, alpha, V, gamma,
            rotor.vrs_ratio(Vc, T_r, rho, P.rotor), I_r, I_p, rho, cl,
        )  # fmt: skip
        return ca.Function("vehicle", [x, u, p], [xdot, aux], ["x", "u", "p"], ["xdot", "aux"])

    def derivatives(
        self, x: np.ndarray, u: np.ndarray, p: np.ndarray
    ) -> tuple[np.ndarray, dict[str, float]]:
        """NumPy wrapper: (xdot, aux dict)."""
        xdot, aux = self.f(x, u, p)
        values = np.array(aux).ravel().tolist()
        return np.array(xdot).ravel(), dict(zip(AUX_NAMES, values, strict=True))

    def rk4_step(self, dt: float, substeps: int = 4) -> ca.Function:
        """One controller step (inputs held) as a single compiled RK4 function (cached)."""
        key = (dt, substeps)
        if key not in self._rk4_cache:
            self._rk4_cache[key] = self._build_rk4(dt, substeps)
        return self._rk4_cache[key]

    def _build_rk4(self, dt: float, substeps: int) -> ca.Function:
        x = ca.MX.sym("x", 9)
        u = ca.MX.sym("u", 3)
        p = ca.MX.sym("p", 4)
        h = dt / substeps
        xn = x
        for _ in range(substeps):
            k1 = self.f(xn, u, p)[0]
            k2 = self.f(xn + 0.5 * h * k1, u, p)[0]
            k3 = self.f(xn + 0.5 * h * k2, u, p)[0]
            k4 = self.f(xn + h * k3, u, p)[0]
            xn = xn + h / 6.0 * (k1 + 2 * k2 + 2 * k3 + k4)
        return ca.Function("rk4", [x, u, p], [xn])
