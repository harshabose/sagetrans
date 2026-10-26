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
    # False: report the rotor V_c / v_h ratio only. True: a path or point is also infeasible when
    # the ratio is below -descent.kappa (mission.yaml)
    vrs_enforce: bool = False


class DRecCfg(_Strict):
    ensemble_n: int = Field(8, ge=1)  # a power of two: the Sobol sample keeps its balance
    ensemble_seed: int = 3
    percentile: float = Field(5.0, gt=0.0, lt=100.0)
    corners: bool = True  # also run the 8 vertices of altitude x mass x headwind
    n_boot: int = Field(1000, ge=10)  # bootstrap resamples for the percentile's interval

    @model_validator(mode="after")
    def _power_of_two(self) -> DRecCfg:
        if self.ensemble_n & (self.ensemble_n - 1):
            raise ValueError(f"ensemble_n must be a power of two, not {self.ensemble_n}")
        return self


class DTradeCfg(_Strict):
    a_plan_ms2: list[float] = [1.5, 2.0, 2.5, 3.0, 3.5]
    headwinds_ms: list[float] = [0.0, 5.0]


class DSensCfg(_Strict):
    """Cases of `D_sensitivity`, on the first named scenario at nominal mass."""

    theta_fw0_deg_sweep: list[float] = [0.0, 4.0, 8.0, 12.0]
    lag_s_sweep: list[float] = [0.0, 0.25, 0.5]
    lift_tau_s_sweep: list[float] = [0.0, 0.5]
    lift_scale_sweep: list[float] = [0.8, 1.2]


class DCfg(_Strict):
    scenarios: list[str] = ["S-01", "S-02"]
    mass_factors: list[float] = [0.85, 1.0, 1.15]
    # altitude hold: "repo" = constant hover throttle, "lift_aware" = throttle for W - L (an ORACLE
    # at tau 0 and scale 1: it reads the simulated lift)
    ff_mode: Literal["repo", "lift_aware"] = "lift_aware"
    ff_lift_tau_s: float = Field(0.0, ge=0.0)
    ff_lift_scale: float = Field(1.0, gt=0.0)
    theta_fw0_deg: float = 0.0  # last fixed-wing pitch at the AIRBRAKE exit (placeholder)
    pit_lim_mode: Literal["constant", "airspeed"] = "constant"  # Q_BCK_PIT_LIM form (O-05)
    q_assist_speed_ms: float = Field(0.0, ge=0.0)  # entry speed: max(AIRSPEED_MIN - 2, this)
    entry_convention: Literal["tas", "eas"] = "tas"  # AIRSPEED_MIN read as true or equivalent
    enforce_alpha: bool = False  # hold the wing angle of attack to the 0.8 C_L,max band top
    alpha_gate_ms: float = Field(5.0, ge=0.0)  # below this airspeed alpha is not judged
    sink_tol_ms: float = Field(0.5, ge=0.0)  # end sink rate that counts as sinking (placeholder)
    rotor_lag_s: float = Field(0.0, ge=0.0)  # sensitivity only
    # diagnostic speed grid (D_a_eq_speed), to set beside A's windows; not the headline
    v0_eas_ms: list[float] = [16.0, 22.0]
    sens: DSensCfg = DSensCfg()
    recommendation: DRecCfg = DRecCfg()
    trade: DTradeCfg = DTradeCfg()


class FCfg(_Strict):
    scenarios: list[str] = ["S-01", "S-02"]


class ECfg(_Strict):
    # airfield elevations; the cruise altitude is the site plus mission.profile.climb_height_m
    sites_amsl_m: list[float] = [0.0, 1500.0, 2500.0, 5000.0]
    missions: list[Literal["round_trip", "one_way"]] = ["round_trip", "one_way"]
    # report points: the one-way distance, or the radius of a round trip
    ranges_km: list[float] = [50.0, 100.0, 113.0, 150.0]
    a_plan_fallback_ms2: float = Field(2.5, gt=0.0)  # used when D produced no recommendation
    # forward-transition acceleration: from Q2 at E's own site ("case"), the worst over the Q2
    # grid ("worst"), or the value in mission.yaml ("mission")
    a_fwd_mode: Literal["case", "worst", "mission"] = "case"
    a_fwd_metric: Literal["min", "equiv"] = "min"  # worst over speed, or distance-equivalent


class Q2Cfg(_Strict):
    altitudes_m: list[float] = [0.0, 1000.0, 2500.0, 5000.0]
    mass_factors: list[float] = [0.85, 1.0, 1.15]
    soc: list[float] = [0.8, 0.6, 0.3]
    conventions: list[Literal["tas", "eas"]] = ["tas", "eas"]  # AIRSPEED_MIN read as TAS or EAS
    feed_convention: Literal["tas", "eas"] = "tas"  # the one E reads
    isa_offset_k: float = 0.0
    n_speed: int = Field(41, ge=11)
    write_profile: bool = False

    @model_validator(mode="after")
    def _feed_is_computed(self) -> Q2Cfg:
        if self.feed_convention not in self.conventions:
            raise ValueError(f"feed_convention '{self.feed_convention}' is not in conventions")
        return self


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
