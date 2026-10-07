"""Controller interface and the mode-state container (NFR-05: knows nothing of analyses)."""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Protocol

import numpy as np


@dataclass(frozen=True)
class ModeState:
    """Immutable controller memory carried between steps by `simulate()`."""

    name: str
    t_enter: float = 0.0  # time the current mode was entered
    t_last: float = 0.0  # time of the previous command (for integrators)
    memory: dict[str, float] = field(default_factory=dict)


class Controller(Protocol):
    def command(
        self, t: float, x: np.ndarray, aux: dict[str, float], mode: ModeState | None
    ) -> tuple[np.ndarray, ModeState]:
        """Return u = [delta_r, delta_p, theta_cmd] and the updated mode state."""
