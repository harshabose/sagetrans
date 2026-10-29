"""Typed settings for the plain config files: `study.yaml` and `scenarios.yaml`.

Unlike the register files (`vehicle.yaml`, `mission.yaml`, `controller.yaml`) these carry no
evidence grades: they are choices (which analyses, how much effort, which scenarios), not data.
Unknown keys are errors, so a typo cannot silently fall back to a default. The defaults here are
the values the shipped files contain.
"""

from __future__ import annotations

import math
from typing import Literal

from pydantic import BaseModel, ConfigDict, Field, model_validator

AnalysisName = Literal["A", "D", "F", "E", "Q2", "FT", "C", "B1", "sensitivity"]
ALL_ANALYSES: tuple[str, ...] = ("A", "D", "F", "E", "Q2", "FT", "C", "B1", "sensitivity")


# the ground speed at which D's runs end (braking.measure_a_eq `v_f`); a test keeps them equal
END_SPEED_MS = 1.0


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
    # the ground speed (m/s) at which the owner reads the curve for a single number; null = none.
    # It must lie inside D.curve.ground_speeds_ms (checked in DCfg)
    design_ground_speed_ms: float | None = Field(None, gt=0.0)

    @model_validator(mode="after")
    def _power_of_two(self) -> DRecCfg:
        if self.ensemble_n & (self.ensemble_n - 1):
            raise ValueError(f"ensemble_n must be a power of two, not {self.ensemble_n}")
        return self


class DCurveCfg(_Strict):
    """`D_a_rec_curve`: the recommendation as a function of the ground speed at the exit."""

    ground_speeds_ms: list[float] = [5.0, 6.0, 8.0, 10.0, 12.0, 14.0, 15.0, 16.0, 18.0, 20.0]

    @model_validator(mode="after")
    def _above_end_speed(self) -> DCurveCfg:
        if not self.ground_speeds_ms:
            raise ValueError("D.curve.ground_speeds_ms is empty")
        if min(self.ground_speeds_ms) <= END_SPEED_MS:
            raise ValueError(
                f"D.curve.ground_speeds_ms must all exceed the end speed {END_SPEED_MS:g} m/s"
            )
        if len(set(self.ground_speeds_ms)) != len(self.ground_speeds_ms):
            raise ValueError("D.curve.ground_speeds_ms has duplicates")
        return self


class DTradeCfg(_Strict):
    a_plan_ms2: list[float] = [1.5, 2.0, 2.5, 3.0, 3.5]
    headwinds_ms: list[float] = [0.0, 5.0]


class DSensCfg(_Strict):
    """Cases of `D_sensitivity`, on the first named scenario at nominal mass."""

    theta_fw0_deg_sweep: list[float] = [0.0, 4.0, 8.0, 12.0]
    lag_s_sweep: list[float] = [0.0, 0.1, 0.25, 0.5]
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
    entry_convention: Literal["tas", "eas"] = "tas"  # AIRSPEED_MIN read as true or equivalent
    enforce_alpha: bool = False  # hold the wing angle of attack to the 0.8 C_L,max band top
    alpha_gate_ms: float = Field(5.0, ge=0.0)  # below this airspeed alpha is not judged
    sink_tol_ms: float = Field(0.5, ge=0.0)  # end sink rate that counts as sinking (placeholder)
    rotor_lag_s: float = Field(0.0, ge=0.0)  # the lumped actuator lag; a sensitivity until measured
    # diagnostic speed grid (D_a_eq_speed), to set beside A's windows; not the headline
    v0_eas_ms: list[float] = [16.0, 22.0]
    sens: DSensCfg = DSensCfg()
    recommendation: DRecCfg = DRecCfg()
    trade: DTradeCfg = DTradeCfg()
    curve: DCurveCfg = DCurveCfg()

    @model_validator(mode="after")
    def _design_speed_on_the_curve(self) -> DCfg:
        v = self.recommendation.design_ground_speed_ms
        lo, hi = min(self.curve.ground_speeds_ms), max(self.curve.ground_speeds_ms)
        if v is not None and not lo <= v <= hi:
            raise ValueError(
                f"D.recommendation.design_ground_speed_ms {v:g} is outside the curve "
                f"({lo:g} to {hi:g} m/s)"
            )
        return self


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
    # share of the weight the wing carries at the hand-over; the rotors carry the rest through the
    # fade of Q_TRANSITION_MS. ASSUMED: the mean of FT's simulated hand-overs (FT_runs,
    # wing_share_handover 0.39 to 0.44 for the register and the recommended settings)
    fade_wing_share: float = Field(0.4, gt=0.0, le=1.0)


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


class CBoundsCfg(_Strict):
    """Search bounds of analysis C: (low, high) of each of the four variables."""

    T_bt: tuple[float, float] = (1.0, 6.0)  # Q_BACKTRANS_MS in seconds
    # Q_A_ANGLE_MAX; 30 is DEC-7, the effective top is min(this, PTCH_LIM_MAX_DEG)
    theta_A_deg: tuple[float, float] = (10.0, 30.0)
    spin_min: tuple[float, float] = (0.06, 0.25)  # Q_M_SPIN_MIN
    q: tuple[float, float] = (1.0, 3.0)  # Q_TRANS_DECEL, m/s^2

    @model_validator(mode="after")
    def _ordered(self) -> CBoundsCfg:
        for name, (lo, hi) in self.model_dump().items():
            if not lo < hi:
                raise ValueError(f"C.bounds.{name}: low {lo} is not below high {hi}")
        if self.q[0] <= 0.0 or self.spin_min[0] < 0.0 or self.T_bt[0] <= 0.0:
            raise ValueError("C.bounds: T_bt and q must be positive and spin_min non-negative")
        return self


class CWeightsCfg(_Strict):
    """Normalisers of the ensemble cost (10.1); its weights (1, 0.3, 0.5) and penalty stay."""

    h_ref: float = Field(2.0, gt=0.0)  # m
    E_ref: float = Field(15000.0, gt=0.0)  # J
    x_ref: float = Field(100.0, gt=0.0)  # m


class CZCfg(_Strict):
    """ArduPilot's vertical loop (open question O-12): master 4.8.0-dev Plane defaults (O-08)."""

    pos_p: float = Field(1.0, gt=0.0)  # Q_P_POSZ_P, 1/s
    vel_p: float = Field(5.0, gt=0.0)  # Q_P_VELZ_P
    acc_p: float = Field(0.03, ge=0.0)  # Q_P_ACCZ_P
    acc_i: float = Field(0.1, ge=0.0)  # Q_P_ACCZ_I
    acc_imax: float = Field(0.8, gt=0.0)  # Q_P_ACCZ_IMAX
    vmax: float = Field(2.5, gt=0.0)  # Q_PILOT_SPD_UP, applied up and down
    vel_flt_hz: float = Field(5.0, gt=0.0)
    acc_flt_hz: float = Field(10.0, gt=0.0)


class CSensCfg(_Strict):
    """`C_sens`: one-at-a-time changes at the optimum."""

    rotor_lag_s: list[float] = [0.0, 0.25, 0.5]
    rotor_start: list[Literal["idle", "carry"]] = ["idle", "carry"]
    z_vel_p: list[float] = [2.5, 5.0, 10.0]
    z_acc_p: list[float] = [0.015, 0.03, 0.06]
    theta_fw0_deg: list[float] = [0.0, 8.0]


class CCfg(_Strict):
    """Analysis C from the AIRBRAKE exit; its entry comes from the D block (docs/C_fix_spec.md)."""

    ensemble_n: int = Field(32, ge=1)
    ensemble_seed: int = 8
    seeds: list[int] = [1, 2, 3, 4, 5, 6, 7, 8]
    popsize: int = Field(8, ge=4)
    maxiter: int = Field(20, ge=1)
    rotor_lag_s: float = Field(0.25, ge=0.0)  # D's lumped actuator lag; C's own value
    rotor_start: Literal["idle", "carry"] = "idle"  # rotors at the AIRBRAKE exit (O-13, unverified)
    os_tol_m: float = Field(2.0, ge=0.0)  # overshoot past x_tgt that counts as a failure (PROPOSED)
    sink_tol_ms: float = Field(1.76, ge=0.0)  # end sink rate that counts as sinking (PROPOSED)
    weights: CWeightsCfg = CWeightsCfg()
    bounds: CBoundsCfg = CBoundsCfg()
    z: CZCfg = CZCfg()
    q_curve: list[float] = [1.0, 1.5, 2.0, 2.25, 2.5, 2.75, 3.0]  # C_q_curve
    # reporting only (ASSUMED): how close to a tolerance counts as "at the edge", and the fine grid
    # of q around the optimum (added to the optimum's q, the other three variables fixed)
    margin_flag_m: float = Field(0.5, ge=0.0)  # overshoot margin below this: at_edge
    margin_flag_ms: float = Field(0.3, ge=0.0)  # end-sink margin below this: at_edge
    # the margin row: worst overshoot AND worst end sink within this fraction of their tolerances
    margin_frac: float = Field(0.75, gt=0.0, le=1.0)
    q_fine_offsets_ms2: list[float] = [-0.1, -0.05, 0.0, 0.05, 0.1, 0.15, 0.2]
    sens: CSensCfg = CSensCfg()

    @model_validator(mode="after")
    def _fine_grid(self) -> CCfg:
        if any(not math.isfinite(o) or abs(o) > 5.0 for o in self.q_fine_offsets_ms2):
            raise ValueError("C.q_fine_offsets_ms2: finite offsets within 5 m/s^2 of the optimum")
        return self

    @model_validator(mode="before")
    @classmethod
    def _old_keys(cls, data: object) -> object:
        if isinstance(data, dict) and "a_plan_source" in data:
            raise ValueError(
                "C.a_plan_source was removed: Q_TRANS_DECEL is now a searched variable of C "
                "(bounds.q), see docs/C_fix_spec.md section 8"
            )
        return data


class FTWeightsCfg(_Strict):
    """Cost of analysis FT: (10.1) with these weights and normalisers; the penalty (10) stays."""

    h_ref: float = Field(5.0, gt=0.0)  # m
    E_ref: float | None = Field(None, gt=0.0)  # J; null = mean energy at the register start
    x_ref: float = Field(100.0, gt=0.0)  # m
    w_h: float = Field(1.0, ge=0.0)
    w_E: float = Field(0.3, ge=0.0)
    w_x: float = Field(0.0, ge=0.0)  # distance is not a cost here


class FTBoundsCfg(_Strict):
    """Search bounds of analysis FT: (low, high) of each of the four variables."""

    airspeed_min_eas_ms: tuple[float, float] = (14.0, 20.0)  # AIRSPEED_MIN as an EAS
    q_transition_s: tuple[float, float] = (2.0, 20.0)  # Q_TRANSITION_MS in seconds
    tkoff_thr_max: tuple[float, float] = (0.5, 1.0)  # TECS forces a 1 % range (O-22)
    q_tran_pit_max_deg: tuple[float, float] = (1.0, 10.0)  # the fade cap is 4 to 22 deg

    @model_validator(mode="after")
    def _ordered(self) -> FTBoundsCfg:
        for name, (lo, hi) in self.model_dump().items():
            if not lo < hi:
                raise ValueError(f"FT.bounds.{name}: low {lo} is not below high {hi}")
        if self.tkoff_thr_max[0] <= 0.0:
            raise ValueError("FT.bounds.tkoff_thr_max: the low end must be above 0")
        if self.airspeed_min_eas_ms[0] <= 0.0 or self.q_transition_s[0] <= 0.0:
            raise ValueError("FT.bounds: AIRSPEED_MIN and Q_TRANSITION_MS must be positive")
        return self


class FTScanCfg(_Strict):
    """`FT_scan`: AIRSPEED_MIN fixed on a grid, the other three variables re-searched."""

    airspeed_min_eas_ms: list[float] = [15.0, 16.0, 17.0, 18.0, 19.0, 20.0]
    seeds: list[int] = [1, 2, 3]
    maxiter: int = Field(12, ge=1)


class FTGridCfg(_Strict):
    """`FT_grid` (FT_spec Part 4): Q_TRANSITION_MS fixed on a grid, the other three searched."""

    q_transition_s: list[float] = [2.0, 3.0, 4.0, 5.0, 6.0, 8.0, 10.0, 12.0, 14.0, 16.0, 20.0]
    # the fine pass: the best coarse point plus these offsets (those outside the bounds are skipped)
    fine_offsets_s: list[float] = [-1.0, -0.5, 0.5, 1.0]
    seeds: list[int] = [1, 2, 3]  # the T-12 spread of a point is taken over these
    maxiter: int = Field(12, ge=1)
    descent_max_rounds: int = Field(50, ge=1)  # moves of the lattice descent

    @model_validator(mode="after")
    def _grid(self) -> FTGridCfg:
        if not self.q_transition_s or any(q < 0.5 or q > 30.0 for q in self.q_transition_s):
            raise ValueError("FT.grid.q_transition_s: use 0.5 to 30 s (the ArduPilot range)")
        if len(set(self.q_transition_s)) != len(self.q_transition_s):
            raise ValueError("FT.grid.q_transition_s has a repeated value")
        return self


class FTSensCfg(_Strict):
    """`FT_sens`: one at a time at the optimum."""

    TECS_TIME_CONST: list[float] = [3.0, 5.0, 8.0]
    TECS_INTEG_GAIN: list[float] = [0.15, 0.3, 0.6]
    rotor_lag_s: list[float] = [0.0, 0.25, 0.5]
    z_vel_p: list[float] = [2.5, 5.0, 10.0]
    z_acc_p: list[float] = [0.015, 0.03, 0.06]
    TRIM_THROTTLE: list[float] = [0.35, 0.45, 0.55]
    tecs_start: list[Literal["warm", "cold"]] = ["warm", "cold"]
    rec_cl_frac: list[float] = [0.7, 0.8, 0.9]  # changes only the projection
    # the model's C_L,max times this, through the stall angle alpha_s (provisional in vehicle.yaml);
    # the simulated lift, the stall speed, the stall flag and the recovery all follow
    CL_max_scale: list[float] = [0.9, 1.1]


class FTCfg(_Strict):
    """Analysis FT, the forward transition from hover (docs/FT_spec.md).

    The start is `mission.profile.clearance_agl_m` above the site; the accepted altitude loss and
    gain are `mission.profile.fwd_alt_loss_max_m` and `fwd_alt_gain_max_m`.
    """

    # the search ensemble: the first points of C's Sobol sample (halved, spec 6.5)
    ensemble_n: int = Field(16, ge=1)
    # the final evaluation of the optimum, the register start and the ArduPilot default: the first
    # `final_ensemble_n` points of the same sample (a prefix, so the search members are among them)
    final_ensemble_n: int = Field(32, ge=1)
    ensemble_seed: int = 8
    headwind_ms: list[float] = [0.0, 5.0]  # the ensemble's headwind range (low, high)
    popsize: int = Field(8, ge=4)  # of every FT search; seeds and iterations: scan.*, grid.*
    rotor_lag_s: float = Field(0.25, ge=0.0)  # as C
    t_end_s: float = Field(90.0, gt=0.0)  # no hand-over by then: a "no_handover" failure
    t_settle_s: float = Field(10.0, ge=0.0)  # simulated after DONE, to see the recovery
    tecs_start: Literal["warm", "cold"] = "warm"  # open item: does TECS run in the VTOL hover?
    stall_margin: float = Field(1.2, gt=0.0)  # EAS / stall speed from the hand-over on
    rec_cl_frac: float | None = Field(None, gt=0.0, le=1.0)  # null = A.cl_frac
    proj_is_failure: bool = True  # the projected minimum altitude below the loss limit fails
    weights: FTWeightsCfg = FTWeightsCfg()
    bounds: FTBoundsCfg = FTBoundsCfg()
    scan: FTScanCfg = FTScanCfg()
    grid: FTGridCfg = FTGridCfg()
    history_scenarios: list[str] = ["S-07", "S-03"]  # histories: optimum and register start
    sens: FTSensCfg = FTSensCfg()

    @model_validator(mode="after")
    def _headwind_range(self) -> FTCfg:
        if len(self.headwind_ms) != 2 or self.headwind_ms[0] > self.headwind_ms[1]:
            raise ValueError("FT.headwind_ms must be [low, high] with low <= high")
        if self.final_ensemble_n < self.ensemble_n:
            raise ValueError("FT.final_ensemble_n must be at least FT.ensemble_n")
        lo, hi = self.bounds.q_transition_s
        if any(not lo <= q <= hi for q in self.grid.q_transition_s):
            raise ValueError("FT.grid.q_transition_s must lie inside FT.bounds.q_transition_s")
        return self


class B1VariantCfg(_Strict):
    """One stated wing limit of B1: a margin on alpha_s or an angle, and the pitch limit."""

    name: str
    alpha_margin: float | None = Field(None, gt=0.0)  # |alpha_w| <= margin x alpha_s
    alpha_max_deg: float | None = Field(None, gt=0.0)  # |alpha_w| <= this angle
    # pitch-command limit; None = the register's PTCH_LIM_MAX_DEG (what C's ramp can reach)
    theta_cmd_max_deg: float | None = Field(34.4, gt=0.0)

    @model_validator(mode="after")
    def _one_limit(self) -> B1VariantCfg:
        if (self.alpha_margin is None) == (self.alpha_max_deg is None):
            raise ValueError(
                f"B1 variant '{self.name}': give exactly one of alpha_margin and alpha_max_deg"
            )
        return self


def _default_variants() -> list[B1VariantCfg]:
    return [
        B1VariantCfg(name="attached", alpha_margin=0.85, theta_cmd_max_deg=34.4),
        B1VariantCfg(name="c_envelope", alpha_max_deg=30.0, theta_cmd_max_deg=None),
    ]


class B1BandSweepCfg(_Strict):
    """Altitude bands run at the largest distance cap only, for one variant (a sensitivity)."""

    eps_h_m: list[float] = [3.0, 10.0]
    variant: str = "c_envelope"


class B1SensCfg(_Strict):
    """Sensitivity rows, first scenario only: each is skipped when false or null."""

    rotor_start_flip: bool = True  # the other rotor start, first variant
    end_vh_band_ms: float | None = 0.5  # the old end band on |v_h|, first variant
    c_envelope_theta_cmd_deg: float | None = 34.4  # pitch limit of the first absolute-angle variant


class B1GapCCfg(_Strict):
    """The C parameters the same-problem gap evaluates (paste the optimum from `C_params`)."""

    T_bt: float = Field(gt=0.0)  # s
    theta_A_deg: float = Field(gt=0.0)
    spin_min: float = Field(ge=0.0)
    q: float = Field(gt=0.0)  # Q_TRANS_DECEL, m/s^2


class B1Cfg(_Strict):
    """B1 from the AIRBRAKE exit (docs/B1_fix_spec.md); the entry comes from the C and D blocks."""

    scenarios: list[str] = ["S-01", "S-02", "S-03", "S-04", "S-05", "S-06"]
    intervals: int = Field(40, ge=10)
    refine_from: list[int] = [20]
    max_iter: int = Field(800, ge=10)
    eps_h_m: list[float] = [1.0]  # headline altitude band (C holds about 0.5 to 1 m)
    eps_x_m: list[float] = [40.0, 50.0, 60.0, 75.0, 100.0, 300.0]  # distance caps: the front
    end_vh_band_ms: float | None = Field(None, gt=0.0)  # null = C.sink_tol_ms
    variants: list[B1VariantCfg] = Field(default_factory=_default_variants, min_length=1)
    band_sweep: B1BandSweepCfg = B1BandSweepCfg()
    sens: B1SensCfg = B1SensCfg()
    gap_c: B1GapCCfg | None = None  # null = the register values at q = 2.0, said in the table

    @model_validator(mode="before")
    @classmethod
    def _old_scalars(cls, data: object) -> object:
        if isinstance(data, dict) and isinstance(data.get("eps_x_m"), int | float):
            raise ValueError(
                "B1.eps_x_m is now a list (the distance caps of the front) and B1.eps_h_m a "
                "list holding the headline band, see docs/B1_fix_spec.md section 7"
            )
        return data

    @model_validator(mode="after")
    def _consistent(self) -> B1Cfg:
        names = [v.name for v in self.variants]
        if len(set(names)) != len(names):
            raise ValueError("B1.variants has duplicate names")
        if self.band_sweep.variant not in names:
            raise ValueError(f"B1.band_sweep.variant '{self.band_sweep.variant}' is not a variant")
        if not self.eps_h_m or not self.eps_x_m:
            raise ValueError("B1.eps_h_m and B1.eps_x_m must not be empty")
        return self


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
    FT: FTCfg = FTCfg()
    B1: B1Cfg = B1Cfg()
    sensitivity: SensCfg = SensCfg()

    def result_relevant(self) -> dict[str, object]:
        """The settings that change numbers (everything except naming, mode and parallelism)."""
        d = self.model_dump()
        for k in ("run_name", "output_dir", "mode", "parallel", "analyses"):
            d.pop(k)
        return d
