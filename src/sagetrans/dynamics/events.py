"""Event definitions for `simulate()` (section 7, Events).

An event function returns a scalar g(t, x, aux); the event fires when g crosses zero in the
given direction between two controller steps, and its time is refined by linear interpolation.
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import numpy as np

EventFn = Callable[[float, np.ndarray, dict[str, float]], float]


@dataclass(frozen=True)
class Event:
    name: str
    fn: EventFn
    direction: int = 0  # +1 rising, -1 falling, 0 either
    terminal: bool = False


def airspeed_reached(v_min: float, terminal: bool = False) -> Event:
    """Transition speed reached: V >= AIRSPEED_MIN."""
    return Event("airspeed_reached", lambda t, x, a: a["V"] - v_min, +1, terminal)


def battery_cutoff(v_min: float) -> Event:
    """Bus voltage below V_min; terminal, flagged as failure."""
    return Event("battery_cutoff", lambda t, x, a: a["V_bus"] - v_min, -1, True)


def touchdown(ground_amsl_m: float = 0.0) -> Event:
    """h_AGL <= 0; terminal."""
    return Event("touchdown", lambda t, x, a: float(x[1]) - ground_amsl_m, -1, True)


def ground_speed_below(v_f: float) -> Event:
    """Ground speed (`vx`, a ground-frame speed) falls to V_f: end of the back-transition."""
    return Event("ground_speed_reached", lambda t, x, a: float(x[2]) - v_f, -1, True)


def vrs_entry(limit: float) -> Event:
    """Vortex-ring-state entry: V_c / v_h below the (sourced) boundary `limit`."""
    return Event("vrs_entry", lambda t, x, a: a["vrs_ratio"] - limit, -1, False)
