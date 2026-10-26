"""Typed settings for the plain config files: `study.yaml` and `scenarios.yaml`.

Unlike the register files (`vehicle.yaml`, `mission.yaml`, `controller.yaml`) these carry no
evidence grades: they are choices (which analyses, how much effort, which scenarios), not data.
Unknown keys are errors, so a typo cannot silently fall back to a default. The defaults here are
the values the shipped files contain.
"""

from __future__ import annotations

from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

AnalysisName = Literal["A", "D", "F", "E", "Q2", "C", "B2", "B1", "sensitivity"]
ALL_ANALYSES: tuple[str, ...] = ("A", "D", "F", "E", "Q2", "C", "B2", "B1", "sensitivity")


class _Strict(BaseModel):
    model_config = ConfigDict(extra="forbid", frozen=True)


# ---- scenarios.yaml ----------------------------------------------------------------------------
class ScenarioCfg(_Strict):
    name: str
    altitude_amsl_m: float
    isa_offset_k: float = 0.0
    soc: float = Field(0.8, ge=0.0, le=1.0)
    headwind_ms: float = 0.0
    mass_factor: float = Field(1.0, gt=0.0)
    ground_amsl_m: float = 0.0
    note: str = ""


class AxesCfg(_Strict):
    altitude_m: tuple[float, float] = (0.0, 5000.0)
    isa_offset_k: tuple[float, float] = (-10.0, 20.0)
    mass_factor: tuple[float, float] = (0.85, 1.15)
    headwind_ms: tuple[float, float] = (-5.0, 10.0)
    soc: tuple[float, float] = (0.2, 1.0)

    @model_validator(mode="after")
    def _ordered(self) -> AxesCfg:
        for name, (lo, hi) in self.model_dump().items():
            if lo > hi:
                raise ValueError(f"axis {name}: low {lo} is above high {hi}")
        return self


class EnsembleCfg(_Strict):
    axes: AxesCfg = AxesCfg()


class ScenariosFile(_Strict):
    core: list[ScenarioCfg]
    ensemble: EnsembleCfg = EnsembleCfg()

    @model_validator(mode="after")
    def _unique_names(self) -> ScenariosFile:
        names = [s.name for s in self.core]
        dup = sorted({n for n in names if names.count(n) > 1})
        if dup:
            raise ValueError(f"duplicate scenario names: {', '.join(dup)}")
        return self


# ---- study.yaml --------------------------------------------------------------------------------
class ParallelCfg(_Strict):
    enabled: bool = True
    max_workers: int = Field(0, ge=0)  # 0 = one per CPU core


class ACfg(_Strict):
    scenarios: list[str] = ["S-01", "S-02"]
    v0_eas_ms: list[float] = [10.0, 12.0, 14.0, 16.0, 18.0, 20.0]
    decel_ms2: list[float] = [0.5, 1.0, 1.5, 2.0, 3.0]
    max_decel_v0_eas_ms: list[float] = [10.0, 12.0, 14.0, 16.0]
    samples: int = Field(60, ge=10)
    # wing angle-of-attack band of the feasibility scan: from alpha_min_deg (None = no lower
    # limit) to the angle where C_L = cl_frac x C_L,max (None = the old symmetric stall check)
    alpha_min_deg: float | None = 0.0
    cl_frac: float | None = Field(0.8, gt=0.0, le=1.0)


class DRecCfg(_Strict):
    ensemble_n: int = Field(8, ge=1)
    ensemble_seed: int = 3
    v0_ground_ms: list[float] = [18.0, 22.0]
    percentile: float | None = 5.0


class DTradeCfg(_Strict):
    a_plan_ms2: list[float] = [1.5, 2.0, 2.5, 3.0, 3.5]
    headwinds_ms: list[float] = [0.0, 5.0]


class DCfg(_Strict):
    scenarios: list[str] = ["S-01", "S-02"]
    mass_factors: list[float] = [0.85, 1.0, 1.15]
    v0_eas_ms: list[float] = [16.0, 22.0]
    recommendation: DRecCfg = DRecCfg()
    trade: DTradeCfg = DTradeCfg()


class FCfg(_Strict):
    scenarios: list[str] = ["S-01", "S-02"]


class ECfg(_Strict):
    cruise_amsl_m: list[float] = [1000.0, 5000.0]
    ranges_km: list[float] = [50.0, 100.0, 113.0, 150.0]


class Q2Cfg(_Strict):
    altitudes_m: list[float] = [0.0, 1000.0, 2500.0, 5000.0]
    mass_factors: list[float] = [0.85, 1.0, 1.15]


class CCfg(_Strict):
    ensemble_n: int = Field(4, ge=1)
    ensemble_seed: int = 1
    seeds: list[int] = [1, 2, 3]
    popsize: int = Field(8, ge=4)
    maxiter: int = Field(15, ge=1)
    a_plan_source: Literal["simulated", "ideal"] = "simulated"


class B2StartCfg(_Strict):
    scenario_index: int = Field(0, ge=0)
    eps_h_m: float = 10.0
    eps_x_m: float = 300.0


class B2Cfg(_Strict):
    ensemble_n: int = Field(4, ge=1)
    ensemble_seed: int = 1
    seeds: list[int] = [1, 2, 3]
    popsize: int = Field(12, ge=4)
    maxiter: int = Field(15, ge=1)
    sigma0: float = Field(0.1, gt=0.0)
    start: B2StartCfg = B2StartCfg()


class B1Cfg(_Strict):
    scenarios: list[str] = ["S-01", "S-02"]
    eps_h_m: list[float] = [20.0, 10.0, 6.0, 3.0]
    eps_x_m: float = 300.0
    intervals: int = Field(40, ge=10)
    refine_from: list[int] = [20]
    max_iter: int = Field(800, ge=10)


class SensCfg(_Strict):
    scenario: str = "S-01"
    r_small: int = Field(6, ge=2)
    r_large: int = Field(12, ge=3)
    include_budget: bool = True

    @model_validator(mode="after")
    def _two_sizes(self) -> SensCfg:
        if self.r_large <= self.r_small:
            raise ValueError("r_large must exceed r_small (the ranking is compared across them)")
        return self


class StudyCfg(_Strict):
    run_name: str = "study"
    output_dir: str = "results"
    mode: Literal["draft", "release"] = "draft"
    parallel: ParallelCfg = ParallelCfg()
    analyses: list[AnalysisName] = list(ALL_ANALYSES)  # type: ignore[arg-type]
    switch_test: Literal["ground", "air"] = "ground"
    A: ACfg = ACfg()
    D: DCfg = DCfg()
    F: FCfg = FCfg()
    E: ECfg = ECfg()
    Q2: Q2Cfg = Q2Cfg()
    C: CCfg = CCfg()
    B2: B2Cfg = B2Cfg()
    B1: B1Cfg = B1Cfg()
    sensitivity: SensCfg = SensCfg()

    def result_relevant(self) -> dict[str, object]:
        """The settings that change numbers (everything except naming, mode and parallelism)."""
        d = self.model_dump()
        for k in ("run_name", "output_dir", "mode", "parallel", "analyses"):
            d.pop(k)
        return d
