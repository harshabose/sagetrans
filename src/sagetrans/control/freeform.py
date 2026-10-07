"""Free-form speed-scheduled back-transition law (section 8.1, eq. 8.1).

Rotor throttle and pitch command are piecewise-linear in normalised airspeed V / V_s. Rotors
are off above V_on and follow the schedule at or below it; the pusher stays at zero. There is
no altitude feedback: the law reacts only to the speed actually flown.
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from sagetrans.control.base import ModeState

S_KNOTS = (0.1, 0.4, 0.8, 1.2, 1.6, 2.4)  # V / V_s grid shared by both channels


@dataclass(frozen=True)
class FreeFormParams:
    delta_r: tuple[float, ...]  # rotor throttle command at each knot
    theta: tuple[float, ...]  # pitch command at each knot [rad]
    s_on: float  # rotors on when V / V_s <= s_on
    s_knots: tuple[float, ...] = S_KNOTS


class FreeFormController:
    def __init__(self, p: FreeFormParams, v_stall: float) -> None:
        self.p = p
        self.v_stall = v_stall

    def command(
        self, t: float, x: np.ndarray, aux: dict[str, float], mode: ModeState | None
    ) -> tuple[np.ndarray, ModeState]:
        p = self.p
        s = aux["V"] / self.v_stall
        theta = float(np.interp(s, p.s_knots, p.theta))
        delta = float(np.interp(s, p.s_knots, p.delta_r)) if s <= p.s_on else 0.0
        mode = mode or ModeState("freeform", t, t)
        return np.array([delta, 0.0, theta]), ModeState(mode.name, mode.t_enter, t)
