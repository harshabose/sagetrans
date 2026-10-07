"""Frozen parameter containers for the physics models (SI units).

The values built by `placeholder_vehicle()` are illustrative ASSUMED numbers so that M1 can be
exercised before the parameter register has measured data. They are not Sage's real values.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import Any


def ideal_cq0(ct0: float) -> float:
    """Static torque coefficient giving figure of merit 1 for thrust coefficient ct0 (T-03)."""
    return float(ct0**1.5 / (math.sqrt(2.0 * math.pi) * math.pi))


@dataclass(frozen=True)
class WingParams:
    S: float = 0.6
    CL0: float = 0.25
    CL_alpha: float = 5.0
    alpha_s: float = 0.35  # rad
    CD0: float = 0.03
    k: float = 0.05
    CN: float = 1.5  # unsourced working estimate (section 6.2)
    i_w: float = 0.03  # rad
    M: float = 100.0  # stall-blend sharpness, 1/rad


@dataclass(frozen=True)
class LiftRotorParams:
    n_r: int = 4
    D: float = 0.457
    I_rot: float = 1.5e-3
    CT0: float = 0.10
    CT1: float = -0.10
    CT2: float = 0.0
    CQ0: float = ideal_cq0(0.10) / 0.6  # figure of merit 0.6
    CQ1: float = -0.5 * ideal_cq0(0.10) / 0.6
    CQ2: float = 0.0
    CH1: float = 0.01
    k_mu: float = 1.0
    J_min: float = -0.5
    J_max: float = 1.0
    mu_max: float = 1.0
    omega_floor: float = 1.0  # rad/s, keeps J and mu finite at standstill

    @property
    def R(self) -> float:
        return 0.5 * self.D

    @property
    def A_tot(self) -> float:
        return self.n_r * math.pi * self.R**2


@dataclass(frozen=True)
class PusherParams:
    D: float = 0.40
    I_p: float = 2.0e-4
    CT0: float = 0.05
    CT1: float = -0.03
    CT2: float = 0.0
    CQ0: float = 0.0055
    CQ1: float = -0.0018
    J_min: float = -0.5
    J_max: float = 2.0
    omega_floor: float = 1.0


@dataclass(frozen=True)
class MotorParams:
    Ke: float = 0.03  # V s/rad (equals Kt in SI)
    Kt: float = 0.03
    Rm: float = 0.08
    I0: float = 1.5
    I_lim: float = 60.0
    dead_zone: float = 0.05  # ESC dead-zone fraction of throttle


@dataclass(frozen=True)
class BatteryParams:
    n_parallel: int = 2
    cells_series: int = 12
    R0_pack: float = 0.030
    R1_pack: float = 0.020
    C1_pack: float = 2000.0
    I_av: float = 2.0  # avionics current, A
    V_min: float = 33.0  # bus cut-off, V
    capacity_Ah_pack: float = 25.0  # datasheet (MPower 12S)
    v_nominal: float = 43.2  # V, datasheet

    def voc(self, soc: Any) -> Any:
        """Placeholder linear open-circuit voltage of the 12S pack (assumed)."""
        return self.cells_series * (3.3 + 0.9 * soc)

    @property
    def e_nominal_wh(self) -> float:
        """Nominal (not usable) pack energy: 2 x 25 Ah x 43.2 V = 2160 Wh."""
        return self.n_parallel * self.capacity_Ah_pack * self.v_nominal

    @property
    def R0(self) -> float:
        return self.R0_pack / self.n_parallel

    @property
    def R1(self) -> float:
        return self.R1_pack / self.n_parallel

    @property
    def C1(self) -> float:
        return self.C1_pack * self.n_parallel


@dataclass(frozen=True)
class AttitudeParams:
    omega_n: float = 8.0  # rad/s, assumed until SITL calibration
    zeta: float = 0.8
    q_max: float = 1.5  # rad/s


@dataclass(frozen=True)
class VehicleParams:
    m: float = 8.0
    wing: WingParams = field(default_factory=WingParams)
    rotor: LiftRotorParams = field(default_factory=LiftRotorParams)
    pusher: PusherParams = field(default_factory=PusherParams)
    motor_r: MotorParams = field(default_factory=MotorParams)
    motor_p: MotorParams = field(
        default_factory=lambda: MotorParams(Ke=0.04, Kt=0.04, Rm=0.05, I0=1.0, I_lim=80.0)
    )
    battery: BatteryParams = field(default_factory=BatteryParams)
    attitude: AttitudeParams = field(default_factory=AttitudeParams)


def placeholder_vehicle() -> VehicleParams:
    return VehicleParams()
