"""Load the config folder into ready-to-use inputs (the single place data enters the toolkit).

The folder holds five files:

* ``vehicle.yaml``, ``mission.yaml``, ``controller.yaml`` -- REGISTER files: every entry carries a
  value, unit, source and evidence status (see `config.schema`). They are merged into one
  `Config` under the prefixes ``vehicle.``, ``mission.`` and ``controller.``, so one hash and one
  set of evidence counts describe all of them.
* ``scenarios.yaml`` and ``study.yaml`` -- plain settings (see `io.settings`).

Two modes:

* **draft** (default): an entry that is ``TBD`` is used as written; an entry that is missing or has
  no value falls back to the built-in placeholder. Everything is recorded and the run is DRAFT.
* **release**: any TBD entry, any fallback, or any referenced file that does not exist is a
  blocking problem and the run refuses to start.

Typos are errors in both modes: an unknown parameter name is reported (with a suggestion), never
silently ignored.
"""

from __future__ import annotations

import dataclasses
import difflib
import hashlib
import json
import math
from collections.abc import Mapping
from dataclasses import dataclass, field, replace
from pathlib import Path
from typing import Any

import yaml
from pydantic import ValidationError

from sagetrans import atmosphere
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.descent import DescentLimits
from sagetrans.analysis.energy import BatteryWindow, MissionProfile
from sagetrans.analysis.ensemble import EnsembleAxes
from sagetrans.config.schema import Config, ConfigError, Status
from sagetrans.control.ardupilot_like import BackTransitionParams, BaselineParams
from sagetrans.control.forward import ForwardParams
from sagetrans.io.settings import ALL_ANALYSES, ScenariosFile, StudyCfg
from sagetrans.physics.params import (
    AttitudeParams,
    BatteryParams,
    LiftRotorParams,
    MotorParams,
    PusherParams,
    VehicleParams,
    WingParams,
)

REGISTER_FILES = ("vehicle", "mission", "controller", "ardupilot_defaults")
# register files that may be absent: the study then runs without the ArduPilot-defaults comparison
OPTIONAL_REGISTER_FILES = ("ardupilot_defaults",)
DEFAULTS_ONLY = ("Q_ASSIST_SPEED",)  # listed in ardupilot_defaults.yaml, not a study parameter
PLAIN_FILES = ("scenarios", "study")

# VehicleParams attribute -> group name in vehicle.yaml
VEHICLE_GROUPS: dict[str, tuple[str, type]] = {
    "wing": ("wing", WingParams),
    "rotor": ("rotor", LiftRotorParams),
    "pusher": ("pusher", PusherParams),
    "motor_r": ("motor_lift", MotorParams),
    "motor_p": ("motor_pusher", MotorParams),
    "battery": ("battery", BatteryParams),
    "attitude": ("attitude", AttitudeParams),
}
# fields whose dataclass unit is not SI: SI value / divisor = native value
NATIVE_DIVISOR = {"vehicle.battery.capacity_Ah_pack": 3600.0}  # coulombs -> Ah

MISSION_OPTIONAL = {"v_cruise", "v_climb", "v_glide"}  # absent: best-range / the cruise speed
MISSION_FROM_CONTROLLER = {"v_trans"}  # taken from ardupilot.AIRSPEED_MIN, not mission.yaml
# set per site by analysis E (the fade fields: the register's Q_TRANSITION_MS, E.fade_wing_share)
MISSION_FROM_STUDY = {"cruise_amsl_m", "ground_amsl_m", "q_transition_s", "wing_share_handover"}

# controller.yaml: ardupilot name -> (BackTransitionParams / decision-dict key, kind)
ARDUPILOT_PARAMS = (
    "AIRSPEED_MIN", "Q_TRANSITION_MS", "Q_BACKTRANS_MS", "Q_A_ANGLE_MAX", "Q_BCK_PIT_LIM",
    "TKOFF_THR_MAX", "Q_TRANS_FAIL", "Q_M_SPIN_MIN", "Q_M_SPIN_MAX", "PTCH_LIM_MAX_DEG",
    "Q_TRANS_DECEL", "Q_TRAN_PIT_MAX", "TKOFF_THR_MIN", "TRIM_THROTTLE", "THR_MIN", "THR_MAX",
    "THR_SLEWRATE", "PTCH_LIM_MIN_DEG",
)  # fmt: skip
TECS_PARAMS = (
    "TECS_TIME_CONST", "TECS_THR_DAMP", "TECS_INTEG_GAIN", "TECS_VERT_ACC", "TECS_PTCH_DAMP",
    "TECS_CLMB_MAX", "TECS_SINK_MAX", "TECS_SINK_MIN", "TECS_PITCH_MAX",
)  # fmt: skip
CONTROLLER_GROUPS = {
    "altitude_hold": ("K_h", "K_vh", "K_i", "v_dem_max", "i_max"),
    "position": ("K_p", "K_v", "a_lim"),
    "cruise_stub": (
        "v_cruise", "r_tecs", "tau_h", "K_s", "K_si", "K_hp", "K_hd", "theta_cruise_max"
    ),
}  # fmt: skip

# sensitivity input name -> register path (its evidence grade comes from the register)
INPUT_REGISTER_PATH = {
    "mass": "vehicle.mass.m", "wing_area": "vehicle.wing.S", "CL_alpha": "vehicle.wing.CL_alpha",
    "CD0": "vehicle.wing.CD0", "alpha_stall": "vehicle.wing.alpha_s",
    "CN_flat_plate": "vehicle.wing.CN", "rotor_CT0": "vehicle.rotor.CT0",
    "rotor_CQ0": "vehicle.rotor.CQ0", "rotor_kmu": "vehicle.rotor.k_mu",
    "rotor_inertia": "vehicle.rotor.I_rot", "motor_Ke": "vehicle.motor_lift.Ke",
    "motor_Rm": "vehicle.motor_lift.Rm", "esc_dead_zone": "vehicle.motor_lift.dead_zone",
    "battery_R0": "vehicle.battery.R0_pack", "att_omega_n": "vehicle.attitude.omega_n",
    "att_zeta": "vehicle.attitude.zeta", "alt_hold_Kvh": "controller.altitude_hold.K_vh",
    "alt_hold_Ki": "controller.altitude_hold.K_i",
}  # fmt: skip


class InputError(ValueError):
    """The config folder is unusable. `.problems` lists every issue found, not just the first."""

    def __init__(self, problems: list[str]) -> None:
        self.problems = problems
        super().__init__("invalid inputs:\n  - " + "\n  - ".join(problems))


@dataclass(frozen=True)
class StudyInputs:
    """Everything the studies need, built once from the config folder."""

    config_dir: Path
    mode: str  # "draft" | "release"
    config: Config  # merged register: vehicle.*, mission.*, controller.*
    vehicle: VehicleParams
    mission: MissionProfile
    window: BatteryWindow
    descent: DescentLimits
    controller: BackTransitionParams  # template with the register's values
    controller_overrides: dict[str, float]  # gains for BackTransitionParams.for_vehicle(**...)
    baseline_overrides: dict[str, Any]  # extra BaselineParams fields (stub gains, switch test)
    start_params: dict[str, float]  # the register's values as analysis C's starting point
    theta_p: float  # PTCH_LIM_MAX_DEG in rad
    requirements_km: tuple[float, float]
    airspeed_min: float  # AIRSPEED_MIN, m/s (also `mission.v_trans`)
    tkoff_thr_max: float  # TKOFF_THR_MAX as flown: the pusher command in AUTO (0 -> THR_MAX)
    scenarios: dict[str, Scenario]
    ensemble_axes: EnsembleAxes
    study: StudyCfg
    fallbacks: tuple[str, ...]  # register paths that fell back to the built-in placeholder
    draft_reasons: tuple[str, ...]  # why this is not a release-grade input set
    settings_hash: str
    forward: ForwardParams
    defaults: dict[str, float] = field(default_factory=dict)  # ArduPilot defaults, SI, by name
    notes: tuple[str, ...] = field(default=())
    tkoff_thr_max_raw: float | None = None  # TKOFF_THR_MAX as the register states it (display)
    scenario_notes: dict[str, str] = field(default_factory=dict)  # scenarios.yaml `note` by name

    @property
    def is_draft(self) -> bool:
        return bool(self.draft_reasons)

    @property
    def inputs_hash(self) -> str:
        """Identifies every result-relevant input (register + settings); the cache key."""
        blob = f"{self.config.config_hash()}|{self.settings_hash}"
        return hashlib.sha256(blob.encode()).hexdigest()

    def status(self, path: str) -> str:
        return self.config.record(path).status.value if path in self.config else "assumed"

    def input_statuses(self) -> dict[str, str]:
        """Evidence grade of each sensitivity input, from the register."""
        return {n: self.status(p) for n, p in INPUT_REGISTER_PATH.items() if p in self.config}

    def input_fractions(self) -> dict[str, float]:
        """Relative half-range for an input whose register entry states an `uncertainty`."""
        out = {}
        for n, p in INPUT_REGISTER_PATH.items():
            if p in self.config and self.config.record(p).uncertainty is not None:
                out[n] = float(self.config.record(p).uncertainty or 0.0)
        return out

    def controller_nominals(self) -> dict[str, float]:
        return {"alt_hold_Kvh": self.controller.K_vh, "alt_hold_Ki": self.controller.K_i}

    def resolve_default(self, name: str) -> float | None:
        """The ArduPilot default of a parameter in SI units, or None (see `resolve_default`)."""
        return resolve_default(self.defaults, name)


# ---- reading files -----------------------------------------------------------------------------
def _read_yaml(path: Path, problems: list[str], optional: bool = False) -> Any:
    if not path.is_file():
        if not optional:
            problems.append(f"missing file: {path.name} (expected in {path.parent})")
        return None
    try:
        with path.open(encoding="utf-8") as fh:
            return yaml.safe_load(fh)
    except yaml.YAMLError as exc:
        problems.append(f"{path.name}: not valid YAML: {exc}")
        return None


def _suggest(name: str, known: set[str]) -> str:
    close = difflib.get_close_matches(name, sorted(known), n=1, cutoff=0.6)
    return f" (did you mean '{close[0]}'?)" if close else ""


def _known_paths() -> set[str]:
    known = {"vehicle.mass.m"}
    for _, (group, cls) in VEHICLE_GROUPS.items():
        known |= {f"vehicle.{group}.{f.name}" for f in dataclasses.fields(cls)}
    known |= {
        f"mission.profile.{f.name}"
        for f in dataclasses.fields(MissionProfile)
        if f.name not in MISSION_FROM_CONTROLLER | MISSION_FROM_STUDY
    }
    known |= {f"mission.battery_window.{f.name}" for f in dataclasses.fields(BatteryWindow)}
    known |= {
        "mission.descent.kappa", "mission.descent.v_fragility", "mission.descent.v_sensor",
        "mission.descent.h_slow", "mission.descent.v_final",
        "mission.requirements.range_primary_km", "mission.requirements.range_p95_km",
    }  # fmt: skip
    known |= {f"controller.ardupilot.{n}" for n in ARDUPILOT_PARAMS}
    known |= {f"controller.tecs.{n}" for n in TECS_PARAMS}
    # the defaults file uses the same names (and so the same unit conversion) as the register
    known |= {f"ardupilot_defaults.ardupilot.{n}" for n in (*ARDUPILOT_PARAMS, *DEFAULTS_ONLY)}
    known |= {f"ardupilot_defaults.tecs.{n}" for n in TECS_PARAMS}
    for g, names in CONTROLLER_GROUPS.items():
        known |= {f"controller.{g}.{n}" for n in names}
    return known


class _Reader:
    """Reads SI values from the merged register, recording fallbacks."""

    def __init__(self, cfg: Config) -> None:
        self.cfg = cfg
        self.fallbacks: list[str] = []

    def has(self, path: str) -> bool:
        return path in self.cfg and self.cfg.record(path).si_value is not None

    def get(self, path: str, default: float, *, optional: bool = False) -> float:
        if self.has(path):
            return float(self.cfg.record(path).si_value or 0.0)
        if not optional:
            self.fallbacks.append(path)
        return default

    def opt(self, path: str) -> float | None:
        return float(self.cfg.record(path).si_value or 0.0) if self.has(path) else None

    def build(
        self, cls: Any, prefix: str, skip: frozenset[str] | set[str] = frozenset(),
        optional: frozenset[str] | set[str] = frozenset(),
    ) -> Any:
        default = cls()
        kw: dict[str, Any] = {}
        for f in dataclasses.fields(default):
            if f.name in skip or f.name == "released":
                continue
            path = f"{prefix}.{f.name}"
            dv = getattr(default, f.name)
            if dataclasses.is_dataclass(dv):
                continue
            if self.has(path):
                si = float(self.cfg.record(path).si_value or 0.0)
                v = si / NATIVE_DIVISOR.get(path, 1.0)
                is_int = isinstance(dv, int) and not isinstance(dv, bool)
                kw[f.name] = int(round(v)) if is_int else v
            elif f.name not in optional:
                self.fallbacks.append(path)
        return cls(**kw)


def _forward_params(
    rd: _Reader, study: StudyCfg, controller: BackTransitionParams, v_cruise_tas: float,
    mission: MissionProfile, airspeed_min: float, q_transition_s: float, tkoff_thr_max: float,
) -> ForwardParams:
    """The forward-transition controller's parameters from the register (analysis FT, FT spec 3.2).

    Angles are converted from the register's radians to degrees, `THR_SLEWRATE` is a fraction per
    second. The speed demand is the cruise speed at its reference altitude, as an equivalent
    airspeed.
    """
    ap, tc = "controller.ardupilot.", "controller.tecs."
    d = ForwardParams(back=replace(controller, z_mode="ardupilot"))
    z, ft = study.C.z, study.FT
    back = replace(
        controller, z_mode="ardupilot", rotor_lag_s=ft.rotor_lag_s, z_pos_p=z.pos_p,
        z_vel_p=z.vel_p, z_acc_p=z.acc_p, z_acc_i=z.acc_i, z_acc_imax=z.acc_imax, z_vmax=z.vmax,
        z_vel_flt_hz=z.vel_flt_hz, z_acc_flt_hz=z.acc_flt_hz,
    )  # fmt: skip
    rho_ref = float(atmosphere.density(mission.v_cruise_ref_amsl_m, mission.isa_offset_k))
    rho0 = float(atmosphere.density(0.0))
    return ForwardParams(
        back=back, airspeed_min_eas=airspeed_min, q_transition_s=q_transition_s,
        tkoff_thr_max=tkoff_thr_max,
        tran_pit_max_deg=math.degrees(
            rd.get(ap + "Q_TRAN_PIT_MAX", math.radians(d.tran_pit_max_deg))
        ),
        tkoff_thr_min=rd.get(ap + "TKOFF_THR_MIN", d.tkoff_thr_min),
        trim_throttle=rd.get(ap + "TRIM_THROTTLE", d.trim_throttle),
        thr_min=rd.get(ap + "THR_MIN", d.thr_min), thr_max=rd.get(ap + "THR_MAX", d.thr_max),
        thr_slewrate=rd.get(ap + "THR_SLEWRATE", d.thr_slewrate),
        ptch_lim_min_deg=math.degrees(
            rd.get(ap + "PTCH_LIM_MIN_DEG", math.radians(d.ptch_lim_min_deg))
        ),
        tecs_time_const=rd.get(tc + "TECS_TIME_CONST", d.tecs_time_const),
        tecs_thr_damp=rd.get(tc + "TECS_THR_DAMP", d.tecs_thr_damp),
        tecs_integ_gain=rd.get(tc + "TECS_INTEG_GAIN", d.tecs_integ_gain),
        tecs_vert_acc=rd.get(tc + "TECS_VERT_ACC", d.tecs_vert_acc),
        tecs_ptch_damp=rd.get(tc + "TECS_PTCH_DAMP", d.tecs_ptch_damp),
        tecs_clmb_max=rd.get(tc + "TECS_CLMB_MAX", d.tecs_clmb_max),
        tecs_sink_max=rd.get(tc + "TECS_SINK_MAX", d.tecs_sink_max),
        tecs_sink_min=rd.get(tc + "TECS_SINK_MIN", d.tecs_sink_min),
        tecs_pitch_max_deg=math.degrees(
            rd.get(tc + "TECS_PITCH_MAX", math.radians(d.tecs_pitch_max_deg))
        ),
        airspeed_cruise_eas=v_cruise_tas * math.sqrt(rho_ref / rho0), tecs_start=ft.tecs_start,
    )


# ---- the loader --------------------------------------------------------------------------------
def load_inputs(
    config_dir: str | Path, mode: str | None = None, only: set[str] | frozenset[str] | None = None
) -> StudyInputs:
    """Read, validate and assemble the config folder. Raises `InputError` listing every problem.

    `only` replaces the `analyses` list of study.yaml (the command line's --only)."""
    root = Path(config_dir)
    problems: list[str] = []
    raw = {
        n: _read_yaml(root / f"{n}.yaml", problems, optional=n in OPTIONAL_REGISTER_FILES)
        for n in (*REGISTER_FILES, *PLAIN_FILES)
    }
    for n in REGISTER_FILES:
        if raw[n] is not None and not isinstance(raw[n], Mapping):
            problems.append(f"{n}.yaml: the top level must be a mapping of groups")
            raw[n] = None
    if problems:
        raise InputError(problems)

    # plain files -> typed settings (validation errors collected, not just the first)
    study: StudyCfg | None = None
    scen_file: ScenariosFile | None = None
    for name, model, target in (("study", StudyCfg, "study"), ("scenarios", ScenariosFile, "scen")):
        try:
            obj = model.model_validate(raw[name] or ({} if name == "study" else None))
            if target == "study":
                study = obj  # type: ignore[assignment]
            else:
                scen_file = obj  # type: ignore[assignment]
        except ValidationError as exc:
            for err in exc.errors():
                loc = ".".join(str(x) for x in err["loc"])
                problems.append(f"{name}.yaml: {loc}: {err['msg']}")
    if problems or study is None or scen_file is None:
        raise InputError(problems)

    # register files -> one merged Config
    try:
        cfg = Config.from_dict({n: raw[n] for n in REGISTER_FILES if raw[n] is not None}, root=root)
    except ConfigError as exc:
        raise InputError(str(exc).split("\n  ")[1:] or [str(exc)]) from exc
    known = _known_paths()
    for p in cfg:
        if p not in known:
            problems.append(f"unknown parameter '{p}'{_suggest(p, known)}")
    if problems:
        raise InputError(problems)

    rd = _Reader(cfg)
    # vehicle
    mass = rd.get("vehicle.mass.m", VehicleParams().m)
    parts = {a: rd.build(c, f"vehicle.{g}") for a, (g, c) in VEHICLE_GROUPS.items()}
    # mission
    airspeed_min = rd.get("controller.ardupilot.AIRSPEED_MIN", 14.0)
    mission = rd.build(
        MissionProfile, "mission.profile", skip=set(MISSION_FROM_CONTROLLER | MISSION_FROM_STUDY),
        optional=set(MISSION_OPTIONAL),
    )  # fmt: skip
    mission = replace(mission, v_trans=airspeed_min)
    window = rd.build(BatteryWindow, "mission.battery_window")
    kappa = rd.opt("mission.descent.kappa")
    kappa_rec = cfg.record("mission.descent.kappa") if "mission.descent.kappa" in cfg else None
    d_default = DescentLimits(kappa=None)
    descent = DescentLimits(
        kappa=kappa,
        kappa_source=(
            f"{kappa_rec.source} [{kappa_rec.status.value}]" if kappa_rec else "UNSOURCED"
        ),
        v_fragility=rd.get("mission.descent.v_fragility", d_default.v_fragility),
        v_sensor=rd.opt("mission.descent.v_sensor"),
        h_slow=rd.get("mission.descent.h_slow", d_default.h_slow),
        v_final=rd.get("mission.descent.v_final", d_default.v_final),
    )
    r1 = rd.get("mission.requirements.range_primary_km", 100e3) / 1e3
    r2 = rd.get("mission.requirements.range_p95_km", 113e3) / 1e3
    # controller
    bt0 = BackTransitionParams()
    ap = "controller.ardupilot."
    theta_a = rd.get(ap + "Q_A_ANGLE_MAX", bt0.theta_A)
    theta_p = rd.get(ap + "PTCH_LIM_MAX_DEG", math.radians(20.0))
    t_bt = rd.get(ap + "Q_BACKTRANS_MS", bt0.T_bt)  # the register converts ms to s
    spin_min = rd.get(ap + "Q_M_SPIN_MIN", bt0.delta_min)
    spin_max = rd.get(ap + "Q_M_SPIN_MAX", bt0.delta_max)
    gains = {
        "K_h": rd.get("controller.altitude_hold.K_h", bt0.K_h),
        "K_vh": rd.get("controller.altitude_hold.K_vh", bt0.K_vh),
        "K_i": rd.get("controller.altitude_hold.K_i", bt0.K_i),
        "v_dem_max": rd.get("controller.altitude_hold.v_dem_max", bt0.v_dem_max),
        "i_max": rd.get("controller.altitude_hold.i_max", bt0.i_max),
        "K_p": rd.get("controller.position.K_p", bt0.K_p),
        "K_v": rd.get("controller.position.K_v", bt0.K_v),
        "a_lim": rd.get("controller.position.a_lim", bt0.a_lim),
        "delta_max": spin_max,
    }
    theta_b = rd.opt(ap + "Q_BCK_PIT_LIM")
    if theta_b is not None:
        gains["theta_B"] = theta_b
    ctl_kw: dict[str, Any] = {
        "theta_A": theta_a, "theta_P": theta_p, "theta_B": theta_b, "T_bt": t_bt,
        "delta_min": spin_min, **{k: v for k, v in gains.items() if k != "theta_B"},
    }  # fmt: skip
    controller = replace(bt0, **ctl_kw)
    bl0 = BaselineParams(back=bt0)
    stub = "controller.cruise_stub."
    q_transition_s = rd.get(ap + "Q_TRANSITION_MS", bl0.q_transition_s)
    q_fail = rd.get(ap + "Q_TRANS_FAIL", 0.0)
    tkoff_raw = rd.get(ap + "TKOFF_THR_MAX", bl0.tkoff_thr_max)
    tkoff = tkoff_raw
    if tkoff_raw == 0.0:  # ArduPilot: 0 means THR_MAX (O-17). Resolved here, once, for everyone.
        if not cfg_has(cfg, ap + "THR_MAX"):
            raise InputError([
                "controller.ardupilot.TKOFF_THR_MAX is 0 (= THR_MAX) but THR_MAX is not in the "
                "register; give THR_MAX or a TKOFF_THR_MAX above 0"])
        tkoff = rd.get(ap + "THR_MAX", 1.0)
    baseline_overrides: dict[str, Any] = {
        "v_cruise": rd.get(stub + "v_cruise", bl0.v_cruise),
        "r_tecs": rd.get(stub + "r_tecs", bl0.r_tecs),
        "tau_h": rd.get(stub + "tau_h", bl0.tau_h),
        "K_s": rd.get(stub + "K_s", bl0.K_s),
        "K_si": rd.get(stub + "K_si", bl0.K_si),
        "K_hp": rd.get(stub + "K_hp", bl0.K_hp),
        "K_hd": rd.get(stub + "K_hd", bl0.K_hd),
        "theta_cruise_max": rd.get(stub + "theta_cruise_max", bl0.theta_cruise_max),
        "q_trans_fail_s": q_fail,
        "speed_test": study.switch_test,
    }
    forward = _forward_params(rd, study, controller, baseline_overrides["v_cruise"], mission,
                              airspeed_min, q_transition_s, tkoff)
    start = {
        "T_bt": t_bt, "theta_A": theta_a, "spin_min": spin_min, "airspeed_min": airspeed_min,
        "q_transition": q_transition_s, "tkoff_thr_max": tkoff,
    }  # fmt: skip

    # vehicle assembled last so `released` can reflect the whole register
    draft_reasons: list[str] = []
    tbd = sorted(p for p in cfg if cfg.record(p).status is Status.TBD)
    if tbd:
        shown = ", ".join(tbd[:6]) + (f", ... ({len(tbd)} in all)" if len(tbd) > 6 else "")
        draft_reasons.append(f"{len(tbd)} register entries are TBD: {shown}")
    if rd.fallbacks:
        shown = ", ".join(rd.fallbacks[:6]) + (f", ... ({len(rd.fallbacks)} in all)"
                                               if len(rd.fallbacks) > 6 else "")
        draft_reasons.append(f"{len(rd.fallbacks)} entries missing, placeholder used: {shown}")
    if cfg.missing_files():
        draft_reasons.append("register references files that do not exist: "
                             + ", ".join(cfg.missing_files()))
    if kappa is None:
        draft_reasons.append("mission.descent.kappa is not set (analysis F cannot run)")
    vehicle = VehicleParams(m=mass, released=not draft_reasons, **parts)

    # scenarios
    scenarios = {
        s.name: Scenario(
            s.altitude_amsl_m, s.isa_offset_k, s.soc, s.headwind_ms,
            ground_amsl_m=s.ground_amsl_m, mass_kg=mass * s.mass_factor,
        )
        for s in scen_file.core
    }  # fmt: skip
    ax = scen_file.ensemble.axes
    axes = EnsembleAxes(ax.altitude_m, ax.isa_offset_k, ax.mass_factor, ax.headwind_ms, ax.soc)
    stub_v = baseline_overrides["v_cruise"]
    if mission.v_cruise is not None and abs(mission.v_cruise - stub_v) > 1e-9:
        problems.append(
            f"mission.profile.v_cruise ({mission.v_cruise:g} m/s) and "
            f"controller.cruise_stub.v_cruise ({stub_v:g} m/s) must be the same design speed"
        )
    # cross-references between study.yaml and scenarios.yaml
    wanted = {
        "A": study.A.scenarios, "D": study.D.scenarios, "C": [study.C.history_scenario],
    }
    if "FT" in (only or study.analyses):  # its history scenarios are looked up only when it runs
        wanted["FT"] = study.FT.history_scenarios
    for an, names in wanted.items():
        for n in names:
            if n not in scenarios:
                problems.append(f"study.yaml: {an}.scenarios names unknown scenario '{n}'"
                                f"{_suggest(n, set(scenarios))}")
    if problems:
        raise InputError(problems)

    settings_blob = json.dumps(
        {"study": study.result_relevant(), "scenarios": scen_file.model_dump(),
         "ctl": baseline_overrides}, sort_keys=True, default=str,
    )  # fmt: skip
    if only:
        study = study.model_copy(update={"analyses": [a for a in ALL_ANALYSES if a in only]})
    chosen = mode or study.mode
    if chosen not in ("draft", "release"):
        raise InputError([f"mode must be 'draft' or 'release', not '{chosen}'"])
    return StudyInputs(
        config_dir=root, mode=chosen, config=cfg, vehicle=vehicle, mission=mission,
        window=window, descent=descent, controller=controller,
        controller_overrides=gains, baseline_overrides=baseline_overrides, start_params=start,
        theta_p=theta_p, requirements_km=(r1, r2), airspeed_min=airspeed_min,
        tkoff_thr_max=tkoff,
        scenarios=scenarios,
        ensemble_axes=axes, study=study, fallbacks=tuple(rd.fallbacks),
        draft_reasons=tuple(draft_reasons),
        settings_hash=hashlib.sha256(settings_blob.encode()).hexdigest(),
        forward=forward, defaults=_defaults(cfg), tkoff_thr_max_raw=tkoff_raw,
        scenario_notes={s.name: s.note for s in scen_file.core},
    )  # fmt: skip


def _defaults(cfg: Config) -> dict[str, float]:
    """The ArduPilot defaults by name in SI units (empty without the defaults file)."""
    out: dict[str, float] = {}
    for p in cfg:
        if p.startswith("ardupilot_defaults.") and cfg.record(p).si_value is not None:
            out[p.rsplit(".", 1)[1]] = float(cfg.record(p).si_value or 0.0)
    return out


def resolve_default(defaults: Mapping[str, float], name: str) -> float | None:
    """The ArduPilot default of `name` in SI units, with "use another parameter" resolved.

    `TKOFF_THR_MAX` 0 means `THR_MAX` is used (O-17); `Q_A_ANGLE_MAX` 0 means the attitude
    controller's 30 deg. A missing entry (or a missing `THR_MAX` behind a 0) gives None: the caller
    skips that comparison and says so, it never falls back to a made-up value.
    """
    v = defaults.get(name)
    if v is None:
        return None
    if name == "TKOFF_THR_MAX" and v == 0.0:
        return defaults.get("THR_MAX")
    if name == "Q_A_ANGLE_MAX" and v == 0.0:
        return math.radians(30.0)
    return float(v)


def cfg_has(cfg: Config, path: str) -> bool:
    return path in cfg and cfg.record(path).si_value is not None


def release_blockers(inputs: StudyInputs) -> list[str]:
    """What stops a release-mode run (empty when the inputs are release grade)."""
    return list(inputs.draft_reasons)
