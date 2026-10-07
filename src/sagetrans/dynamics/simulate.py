"""Time-domain driver joining a vehicle model to a controller (FR-02, section 7).

Fixed controller step (default 0.02 s) with commands held over each step. Default integrator
is the compiled CasADi RK4; `integrator="reference"` uses scipy's adaptive LSODA (T-09).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass, field
from typing import Any, Protocol

import numpy as np
from scipy.integrate import solve_ivp

from sagetrans.dynamics.events import Event
from sagetrans.physics.vehicle import AUX_NAMES, INPUT_NAMES, STATE_NAMES, CasadiVehicle

DT_DEFAULT = 0.02


@dataclass(frozen=True)
class Environment:
    """Per-scenario, constant over a segment."""

    altitude_amsl_m: float  # terrain elevation under the track; h_AGL = h - this
    isa_offset_k: float = 0.0
    headwind_ms: float = 0.0  # positive against the aircraft
    gust: Callable[[float], float] | None = None  # added to the headwind, evaluated per step
    soc: float = 0.8  # pack state of charge, constant within a segment
    theta_max_rad: float = 1.2  # pitch-command limit passed to the attitude surrogate

    def param_vector(self, t: float) -> np.ndarray:
        wind = self.headwind_ms + (self.gust(t) if self.gust is not None else 0.0)
        return np.array([self.isa_offset_k, wind, self.theta_max_rad, self.soc])


class Controller(Protocol):
    def command(
        self, t: float, x: np.ndarray, aux: dict[str, float], mode: Any
    ) -> tuple[np.ndarray, Any]:
        """Return u = [delta_r, delta_p, theta_cmd] and the updated mode state."""


@dataclass(frozen=True)
class EventRecord:
    name: str
    t: float
    x: np.ndarray
    terminal: bool


@dataclass
class Result:
    """Time histories as columns, the event list, and the metrics dictionary."""

    t: np.ndarray
    columns: dict[str, np.ndarray]
    events: list[EventRecord] = field(default_factory=list)
    metrics: dict[str, float] = field(default_factory=dict)
    final_mode: Any = None
    terminated_by: str | None = None
    modes: list[str] = field(default_factory=list)  # controller mode name at each sample

    def __getitem__(self, name: str) -> np.ndarray:
        return self.columns[name]

    def event(self, name: str) -> EventRecord | None:
        return next((e for e in self.events if e.name == name), None)


def compute_metrics(t: np.ndarray, cols: dict[str, np.ndarray], h_ref: float) -> dict[str, float]:
    """Metrics of section 7 that follow from one segment's histories."""
    h = cols["h"]
    loss = max(0.0, h_ref - float(h.min()))
    gain = max(0.0, float(h.max()) - h_ref)
    power = cols["V_bus"] * cols["I_bus"]
    return {
        "altitude_loss": loss,
        "altitude_gain": gain,
        "altitude_excursion": max(loss, gain),
        "distance": float(cols["x"][-1] - cols["x"][0]),
        "duration": float(t[-1] - t[0]),
        "energy_J": float(np.sum(0.5 * (power[1:] + power[:-1]) * np.diff(t))),
        "peak_bus_current": float(cols["I_bus"].max()),
        "min_bus_voltage": float(cols["V_bus"].min()),
    }


StepFn = Callable[[np.ndarray, np.ndarray, np.ndarray, float], np.ndarray]


def _reference_step(vehicle: CasadiVehicle) -> StepFn:
    def rhs(_t: float, y: np.ndarray, u: np.ndarray, p: np.ndarray) -> np.ndarray:
        return np.asarray(vehicle.f(y, u, p)[0]).ravel()

    def step(x: np.ndarray, u: np.ndarray, p: np.ndarray, dt: float) -> np.ndarray:
        sol = solve_ivp(rhs, (0.0, dt), x, method="LSODA", rtol=1e-6, atol=1e-9, args=(u, p))
        return np.asarray(sol.y[:, -1])

    return step


def simulate(
    vehicle: CasadiVehicle,
    controller: Controller,
    env: Environment,
    x0: np.ndarray,
    t_end: float,
    events: list[Event] | None = None,
    *,
    mode0: Any = None,
    dt: float = DT_DEFAULT,
    substeps: int = 4,
    integrator: str = "rk4",
) -> Result:
    """Simulate one segment from t = 0 to t_end or the first terminal event."""
    events = events or []
    if integrator == "rk4":
        rk4 = vehicle.rk4_step(dt, substeps)

        def advance(x: np.ndarray, u: np.ndarray, p: np.ndarray) -> np.ndarray:
            return np.asarray(rk4(x, u, p)).ravel()

    elif integrator == "reference":
        ref = _reference_step(vehicle)

        def advance(x: np.ndarray, u: np.ndarray, p: np.ndarray) -> np.ndarray:
            return ref(x, u, p, dt)

    else:
        raise ValueError(f"unknown integrator {integrator!r}")

    n_steps = int(round(t_end / dt))
    x = np.asarray(x0, dtype=float).copy()
    mode = mode0
    u_prev = np.zeros(3)
    _, aux_vec = vehicle.f(x, u_prev, env.param_vector(0.0))
    aux_prev = dict(zip(AUX_NAMES, np.asarray(aux_vec).ravel().tolist(), strict=True))

    ts: list[float] = []
    xs: list[np.ndarray] = []
    us: list[np.ndarray] = []
    auxs: list[np.ndarray] = []
    modes: list[str] = []
    records: list[EventRecord] = []
    g_prev: list[float | None] = [None] * len(events)
    terminated_by: str | None = None

    for k in range(n_steps + 1):
        t = k * dt
        p = env.param_vector(t)
        u, mode = controller.command(t, x, aux_prev, mode)
        u = np.asarray(u, dtype=float)
        _, aux_vec = vehicle.f(x, u, p)
        aux_arr = np.asarray(aux_vec).ravel()
        aux = dict(zip(AUX_NAMES, aux_arr.tolist(), strict=True))

        stop = False
        for i, ev in enumerate(events):
            g = ev.fn(t, x, aux)
            gp = g_prev[i]
            g_prev[i] = g
            if gp is None or not xs:
                continue
            rising = gp < 0.0 <= g
            falling = gp > 0.0 >= g
            if (ev.direction >= 0 and rising) or (ev.direction <= 0 and falling):
                s = gp / (gp - g)
                te = ts[-1] + s * dt
                xe = xs[-1] + s * (x - xs[-1])
                records.append(EventRecord(ev.name, te, xe, ev.terminal))
                if ev.terminal and not stop:
                    stop = True
                    terminated_by = ev.name
                    ts.append(te)
                    xs.append(xe)
                    us.append(us[-1])
                    auxs.append(auxs[-1] + s * (aux_arr - auxs[-1]))
                    modes.append(modes[-1])
        if stop:
            break

        ts.append(t)
        xs.append(x.copy())
        us.append(u)
        auxs.append(aux_arr)
        modes.append(str(getattr(mode, "name", "")))
        aux_prev = aux
        if k < n_steps:
            x = advance(x, u, p)

    t_arr = np.array(ts)
    x_arr, u_arr, a_arr = np.array(xs), np.array(us), np.array(auxs)
    cols: dict[str, np.ndarray] = {}
    for j, name in enumerate(STATE_NAMES):
        cols[name] = x_arr[:, j]
    for j, name in enumerate(INPUT_NAMES):
        cols[name] = u_arr[:, j]
    for j, name in enumerate(AUX_NAMES):
        cols[name] = a_arr[:, j]
    return Result(
        t=t_arr,
        columns=cols,
        events=records,
        metrics=compute_metrics(t_arr, cols, float(x0[1])),
        final_mode=mode,
        terminated_by=terminated_by,
        modes=modes,
    )
