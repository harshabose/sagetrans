"""Shared analysis plumbing: scenarios and the run record (FR-09)."""

from __future__ import annotations

import dataclasses
import datetime as dt
import hashlib
import json
from dataclasses import dataclass
from typing import Any

import sagetrans
from sagetrans import atmosphere
from sagetrans.dynamics.simulate import Environment
from sagetrans.physics.params import VehicleParams


@dataclass(frozen=True)
class Scenario:
    """One scenario of section 9: where, how dense, how charged. Mass lives in VehicleParams."""

    altitude_amsl_m: float  # flight altitude, sets the density
    isa_offset_k: float = 0.0
    soc: float = 0.8
    headwind_ms: float = 0.0
    ground_amsl_m: float = 0.0
    mass_kg: float | None = None  # overrides the vehicle mass when set

    @property
    def rho(self) -> float:
        return float(atmosphere.density(self.altitude_amsl_m, self.isa_offset_k))

    def vehicle_for(self, vp: VehicleParams) -> VehicleParams:
        return vp if self.mass_kg is None else dataclasses.replace(vp, m=self.mass_kg)

    def environment(self) -> Environment:
        return Environment(
            altitude_amsl_m=self.ground_amsl_m,
            isa_offset_k=self.isa_offset_k,
            headwind_ms=self.headwind_ms,
            soc=self.soc,
        )


@dataclass(frozen=True)
class RunRecord:
    """Traceability for an output. The hash covers the vehicle parameters, scenario and options.

    NOTE: until `VehicleParams` is built from the YAML register this is not the register's
    config hash, and every record is DRAFT (the physics inputs are placeholders).
    """

    input_hash: str
    code_version: str
    date: str
    label: str = "DRAFT"


def make_record(*parts: Any) -> RunRecord:
    def conv(o: Any) -> Any:
        is_dc = dataclasses.is_dataclass(o) and not isinstance(o, type)
        return dataclasses.asdict(o) if is_dc else o

    blob = json.dumps([conv(p) for p in parts], sort_keys=True, default=str)
    return RunRecord(
        input_hash=hashlib.sha256(blob.encode()).hexdigest(),
        code_version=sagetrans.__version__,
        date=dt.date.today().isoformat(),
    )
