import dataclasses
import math

import numpy as np
import pytest

from sagetrans import atmosphere
from sagetrans.dynamics import events as ev
from sagetrans.dynamics.simulate import Environment, simulate
from sagetrans.physics.params import (
    LiftRotorParams,
    VehicleParams,
    WingParams,
    placeholder_vehicle,
)
from sagetrans.physics.vehicle import AUX_NAMES, CasadiVehicle

G = atmosphere.G0
ENV = Environment(altitude_amsl_m=0.0)


class Constant:
    def __init__(self, u):
        self.u = np.asarray(u, dtype=float)

    def command(self, t, x, aux, mode):
        return self.u, mode


def _x(h=0.0, vx=0.0, vh=0.0, theta=0.0, wr=0.0, wp=0.0):
    return np.array([0.0, h, vx, vh, theta, 0.0, wr, wp, 0.0])


def test_t04_constant_deceleration_limit():
    # no aero, ideal altitude hold, fixed pitch: stopping distance V^2 / (2 g tan(theta))
    r = LiftRotorParams(CT1=0.0, CT2=0.0, k_mu=0.0, CH1=0.0)
    vp = VehicleParams(wing=WingParams(S=0.0), rotor=r)
    theta, v0, h = 0.2, 25.0, 0.0
    rho = float(atmosphere.density(h))
    thrust = vp.m * G / math.cos(theta)
    omega = 2 * math.pi * math.sqrt(thrust / (r.n_r * r.CT0 * rho * r.D**4))
    veh = CasadiVehicle(
        vp, frozen_states=("h", "vh", "theta", "q", "omega_r", "omega_p", "v1")
    )
    res = simulate(
        veh, Constant([0.0, 0.0, theta]), ENV, _x(h, v0, 0.0, theta, omega), 60.0,
        [ev.ground_speed_below(0.0)],
    )  # fmt: skip
    assert res.terminated_by == "ground_speed_reached"
    expected = v0**2 / (2 * G * math.tan(theta))
    assert res.metrics["distance"] == pytest.approx(expected, rel=5e-3)


def test_t05_coast_limit():
    # rotors off, vertical motion held: distance L_d ln(V0/V), L_d = 2m / (rho S C_D)
    w = WingParams(CL0=0.0, CL_alpha=0.0, k=0.0, i_w=0.0, CD0=0.05)
    vp = VehicleParams(wing=w)
    veh = CasadiVehicle(
        vp, frozen_states=("h", "vh", "theta", "q", "omega_r", "omega_p", "v1")
    )
    v0, v_end = 25.0, 5.0
    res = simulate(
        veh, Constant([0.0, 0.0, 0.0]), ENV, _x(0.0, v0), 120.0,
        [ev.Event("slow", lambda t, x, a: a["V"] - v_end, -1, True)],
    )  # fmt: skip
    assert res.terminated_by == "slow"
    rho = float(atmosphere.density(0.0))
    l_d = 2 * vp.m / (rho * w.S * w.CD0)
    assert res.metrics["distance"] == pytest.approx(l_d * math.log(v0 / v_end), rel=5e-3)


def test_t06_energy_conservation_with_thrust_and_drag_off():
    vp = VehicleParams(wing=WingParams(S=0.0))
    veh = CasadiVehicle(vp, frozen_states=("theta", "q", "omega_r", "omega_p", "v1"))
    res = simulate(veh, Constant([0.0, 0.0, 0.0]), ENV, _x(1000.0, 20.0, 5.0), 60.0)
    e = 0.5 * vp.m * (res["vx"] ** 2 + res["vh"] ** 2) + vp.m * G * res["h"]
    assert np.max(np.abs(e - e[0])) / e[0] < 1e-6


def _scenario_step():
    veh = CasadiVehicle(placeholder_vehicle())
    x0 = _x(100.0, 15.0, 0.0, 0.05, 380.0, 300.0)
    return veh, x0, Constant([0.45, 0.5, 0.05])


def test_t09_step_size_convergence_against_adaptive_reference():
    veh, x0, ctrl = _scenario_step()
    env = Environment(altitude_amsl_m=0.0)
    default = simulate(veh, ctrl, env, x0, 6.0)
    ref = simulate(veh, ctrl, env, x0, 6.0, integrator="reference")
    a, b = default.metrics["altitude_excursion"], ref.metrics["altitude_excursion"]
    assert abs(a - b) < max(0.01 * abs(b), 0.05)
    assert a > 0.0


def test_t10_cartesian_matches_flight_path_angle_form():
    veh = CasadiVehicle(placeholder_vehicle())
    m = veh.params.m
    rng = np.random.default_rng(3)
    for _ in range(100):
        x = _x(
            rng.uniform(0, 3000), rng.uniform(8, 30), rng.uniform(-4, 4),
            rng.uniform(-0.2, 0.4), rng.uniform(100, 450), rng.uniform(100, 600),
        )  # fmt: skip
        u = np.array([rng.uniform(0.3, 0.8), rng.uniform(0.3, 0.8), rng.uniform(-0.1, 0.3)])
        p = np.array([rng.uniform(-10, 20), rng.uniform(-3, 5), 1.2, 0.8])
        xd, a = veh.derivatives(x, u, p)
        va, vh = x[2] + p[1], x[3]
        V, gam, alpha = a["V"], a["gamma"], a["alpha"]
        # force balance in wind axes (flight-path-angle form)
        tang = (
            a["T_p"] * math.cos(alpha) - a["T_r"] * math.sin(alpha)
            - a["H_r"] * math.cos(alpha) - a["D"] - m * G * math.sin(gam)
        )  # fmt: skip
        norm = (
            a["T_p"] * math.sin(alpha) + a["T_r"] * math.cos(alpha)
            - a["H_r"] * math.sin(alpha) + a["L"] - m * G * math.cos(gam)
        )  # fmt: skip
        v_dot, g_dot = tang / m, norm / (m * V)
        assert (va * xd[2] + vh * xd[3]) / V == pytest.approx(v_dot, rel=1e-6, abs=1e-8)
        assert (va * xd[3] - vh * xd[2]) / V**2 == pytest.approx(g_dot, rel=1e-6, abs=1e-8)


def test_events_and_metrics_and_determinism():
    veh, x0, ctrl = _scenario_step()
    events = [ev.airspeed_reached(16.0), ev.touchdown(0.0)]
    a = simulate(veh, ctrl, ENV, x0, 5.0, events)
    b = simulate(veh, ctrl, ENV, x0, 5.0, events)
    assert np.array_equal(a["h"], b["h"])  # NFR-01
    assert set(AUX_NAMES) <= set(a.columns)
    assert a.metrics["energy_J"] > 0 and a.metrics["min_bus_voltage"] > 0
    hit = a.event("airspeed_reached")
    assert hit is None or 0.0 < hit.t < 5.0


def test_terminal_event_truncates_run():
    veh = CasadiVehicle(dataclasses.replace(placeholder_vehicle()))
    res = simulate(
        veh, Constant([0.0, 0.0, 0.0]), ENV, _x(20.0, 0.0, 0.0), 30.0, [ev.touchdown(0.0)]
    )
    assert res.terminated_by == "touchdown"
    assert res["h"][-1] == pytest.approx(0.0, abs=1e-6)
    assert res.t[-1] < 30.0
