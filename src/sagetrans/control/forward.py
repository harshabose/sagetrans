"""The forward transition of a QuadPlane (analysis FT, docs/FT_spec.md): hover, hand-over, fade.

`ForwardController` flies the three phases of ArduPilot's forward transition: WAIT (`AIRSPEED_WAIT`:
the rotor hover Z loop holds the start height while the pusher accelerates the aircraft), FADE
(`TIMER`: the rotor throttle falls linearly from its last value, no Z loop) and DONE (rotors off,
caps lifted). The pusher and the pitch come from `TecsLite`, a port of ArduPilot's TECS.

Every airspeed comparison is an EQUIVALENT airspeed (O-14). The pitch caps (O-16) and the pusher
limits (O-17) follow the mode; the TECS flight stage is VTOL in WAIT and FADE and NORMAL after DONE
(O-21). `Q_ASSIST_SPEED` is assumed 0, so nothing restarts the fade (O-20).

`TecsLite` is a line-by-line port of `AP_TECS.cpp` from ArduPilot tag `Plane-4.7.2`:
`_update_throttle_limits`, `_update_pitch_limits`, `_update_STE_rate_lim`, `_update_speed_demand`,
`_detect_underspeed`, `_update_energies`, `_update_pitch`, `_update_throttle_with_airspeed`,
`_detect_bad_descent`, with `linear_interpolate` from `AP_Math`. The source wins over O-22 where
they differ. Simplified, each a model assumption (listed in the report notes):

* speed and height come from the true states, not the complementary filters; `vel_dot` is a finite
  difference of the true airspeed with a 0.1 s moving average (the source averages the accelerometer
  over 5 samples);
* the height demand is constant, so its 2-point average, rate limit and `TECS_HDEM_TCONST` lag are
  the constant and the demanded climb rate is 0;
* `_max_climb_scaler` and `_max_sink_scaler` are 1;
* roll compensation, landing, flare, takeoff stages, glider options, synthetic airspeed, stall
  prevention, battery-voltage compensation and the `TECS_THR_ERATE` decay of the lower throttle
  limit are not modelled;
* the speed demand is `AIRSPEED_CRUISE` (EAS converted at the local density), limited to
  `[AIRSPEED_MIN, AIRSPEED_CRUISE]`.

VTOL stage, as the source has it: pitch ignores the speed (`SKE_weighting = 0`) and the throttle's
energy error ignores the height (`SPE_err_max = SPE_err_min = 0`). The height error still enters
the throttle through the feed-forward rate term (`SPEdot_dem` is part of `STEdot_dem`), which O-21
does not mention; the port keeps it because the source wins.

Calibration status: the gains are ArduPilot's defaults, the structure is the source's, nothing is
fitted to a log (TECS-lite, uncalibrated). State is carried in `ModeState.memory` (floats only).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

import numpy as np

from sagetrans import atmosphere
from sagetrans.control.ardupilot_like import BackTransitionParams, G, LiftFF, Position1Controller
from sagetrans.control.base import ModeState

RHO0 = float(atmosphere.density(0.0))
MIN_AIRSPEED = 3.0  # `min_airspeed` in `_update_speed`, m/s
V_GROUND_MIN = 3.0  # below this ground speed the WAIT pitch cap is 0 (O-16), m/s
Z_RAMP_S = 2.0  # the climb-rate demand is ramped in over the first seconds (O-18)
Z_SPREAD_S = 10.0  # the height error is spread over this time (O-18)
VEL_DOT_SAMPLES = 5  # 0.1 s moving average of d(TAS)/dt at the 0.02 s controller step


@dataclass(frozen=True)
class ForwardParams:
    """Everything the forward-transition controller reads. Angles in degrees, times in seconds."""

    back: BackTransitionParams  # Z-loop gains, delta_min, delta_max, rotor_lag_s (ardupilot Z loop)
    airspeed_min_eas: float = 17.0  # AIRSPEED_MIN as an equivalent airspeed, m/s
    q_transition_s: float = 5.0  # Q_TRANSITION_MS in seconds
    tkoff_thr_max: float = 0.8  # TKOFF_THR_MAX; 0 means THR_MAX (O-17)
    tran_pit_max_deg: float = 3.0  # Q_TRAN_PIT_MAX; the fade cap is (this + 1) x 2 (O-16)
    tkoff_thr_min: float = 0.0  # TKOFF_THR_MIN; 0 means TRIM_THROTTLE is the minimum (O-17)
    trim_throttle: float = 0.45  # TRIM_THROTTLE (THROTTLE_CRUISE)
    thr_min: float = 0.0
    thr_max: float = 1.0
    thr_slewrate: float = 1.0  # THR_SLEWRATE as a fraction per second (100 %/s)
    ptch_lim_min_deg: float = -25.0  # PTCH_LIM_MIN_DEG
    tecs_time_const: float = 5.0
    tecs_thr_damp: float = 0.5
    tecs_integ_gain: float = 0.3
    tecs_vert_acc: float = 7.0
    tecs_ptch_damp: float = 0.3
    tecs_clmb_max: float = 5.0
    tecs_sink_max: float = 5.0
    tecs_sink_min: float = 2.0
    tecs_pitch_max_deg: float = 15.0
    tecs_spd_weight: float = 1.0  # TECS_SPDWEIGHT (not a register entry: the ArduPilot default)
    airspeed_cruise_eas: float = 20.9  # the speed demand, m/s EAS
    h_start: float = 0.0  # the altitude to hold, m (AMSL, as the state)
    tecs_start: str = "warm"  # "warm" | "cold": see TecsLite
    level_transition: bool = False  # Q_OPTIONS LEVEL_TRANSITION: caps the climb demand at 0

    def __post_init__(self) -> None:
        if self.tecs_start not in ("warm", "cold"):
            raise ValueError(f"unknown tecs_start {self.tecs_start!r}")
        if self.back.z_mode != "ardupilot":
            raise ValueError("the forward transition needs back.z_mode == 'ardupilot'")


def pitch_cap_deg(fp: ForwardParams, mode: str, ground_speed: float) -> float:
    """The transition pitch cap (+- degrees) of a mode: O-16. DONE has none (90 deg = no cap)."""
    if mode == "fwd_wait":
        return 0.0 if ground_speed < V_GROUND_MIN else fp.tran_pit_max_deg
    if mode == "fwd_fade":
        return (fp.tran_pit_max_deg + 1.0) * 2.0
    return 90.0


def throttle_limits(fp: ForwardParams, mode: str) -> tuple[float, float]:
    """The pusher's (min, max) throttle of a mode (O-17), before TECS widens a range under 1 %."""
    if mode == "fwd_done":
        return fp.thr_min, fp.thr_max
    lo = fp.tkoff_thr_min if fp.tkoff_thr_min != 0.0 else fp.trim_throttle
    hi = fp.tkoff_thr_max if fp.tkoff_thr_max != 0.0 else fp.thr_max
    return max(fp.thr_min, lo), hi


def linear_interpolate(
    output_low: float, output_high: float, input_value: float, input_low: float, input_high: float
) -> float:
    """`linear_interpolate` of AP_Math (either polarity, clamped)."""
    if input_low > input_high:
        input_low, input_high = input_high, input_low
        output_low, output_high = output_high, output_low
    if input_value <= input_low:
        return output_low
    if input_value >= input_high:
        return output_high
    p = (input_value - input_low) / (input_high - input_low)
    return output_low + p * (output_high - output_low)


def _clip(x: float, lo: float, hi: float) -> float:
    return min(max(x, lo), hi)


class TecsLite:
    """Port of ArduPilot's TECS (`AP_TECS.cpp`, Plane-4.7.2): the pitch and throttle demands.

    `step` advances the state held in `mem` (floats only) by one controller step and returns the
    pitch demand [rad], the throttle demand and the new state. `stage` is `"vtol"` or `"normal"`.
    Start state (`tecs_start`, open item in O-22): `warm` puts `TAS_dem_adj` at the speed demand
    already (TECS has run through the VTOL hover), `cold` resets it to the current true airspeed at
    the first step; integrators start at 0, the last throttle demand at `TRIM_THROTTLE`.
    """

    def __init__(self, fp: ForwardParams) -> None:
        self.fp = fp

    def _tc(self) -> float:  # timeConstant()
        return max(self.fp.tecs_time_const, 0.1)

    def step(  # noqa: PLR0912, PLR0915 - a port keeps the source's order
        self,
        mem: dict[str, float],
        t: float,
        dt: float,
        v_tas: float,
        rho: float,
        h: float,
        vh: float,
        theta: float,
        thr_limits: tuple[float, float],
        pitch_ext_deg: tuple[float, float],
        stage: str,
    ) -> tuple[float, float, dict[str, float]]:
        fp, m = self.fp, dict(mem)
        reset = "init" not in m
        DT = 0.02 if reset else max(dt, 0.001)  # `_DT`: the first step uses the 50 Hz period
        eas2tas = math.sqrt(RHO0 / rho)
        vtol = stage == "vtol"
        tc = self._tc()

        # -- speed state and acceleration (`_update_speed`, simplified: true states)
        tas_state = max(v_tas, MIN_AIRSPEED)
        if reset:
            for k in range(VEL_DOT_SAMPLES):
                m[f"vd{k}"] = 0.0
            m["v_prev"] = tas_state
            vel_dot = 0.0
            m["vel_dot_lpf"] = vel_dot
        else:
            raw = (tas_state - m["v_prev"]) / DT
            m["v_prev"] = tas_state
            for k in range(VEL_DOT_SAMPLES - 1, 0, -1):
                m[f"vd{k}"] = m[f"vd{k - 1}"]
            m["vd0"] = raw
            vel_dot = sum(m[f"vd{k}"] for k in range(VEL_DOT_SAMPLES)) / VEL_DOT_SAMPLES
            alpha = DT / (DT + tc)
            m["vel_dot_lpf"] = m["vel_dot_lpf"] * (1.0 - alpha) + vel_dot * alpha

        # -- throttle limits (`_update_throttle_limits`)
        thr_max_f = min(1.0, thr_limits[1])
        thr_min_f = max(-1.0, thr_limits[0])
        if abs(thr_min_f - thr_max_f) < 0.01:  # a minimum 1 % range
            if thr_max_f < 1.0:
                thr_max_f = max(thr_max_f, thr_min_f + 0.01)
            else:
                thr_min_f = min(thr_min_f, thr_max_f - 0.01)

        # -- pitch limits (`_update_pitch_limits`): TECS_PITCH_MAX, PTCH_LIM_MIN, then the caps
        pitch_max_f = min(fp.tecs_pitch_max_deg, pitch_ext_deg[1])
        pitch_min_f = max(fp.ptch_lim_min_deg, pitch_ext_deg[0])
        pitch_max_f, pitch_min_f = math.radians(pitch_max_f), math.radians(pitch_min_f)
        pitch_max_f = max(pitch_max_f, pitch_min_f)

        # -- initialise states (`_initialise_states`) and the speed demand's start
        tas_cruise = fp.airspeed_cruise_eas * eas2tas
        tas_min = fp.airspeed_min_eas * eas2tas
        tas_max = max(tas_cruise, tas_min)
        if reset:
            m.update({
                "init": 1.0, "integ_thr": 0.0, "integ_seb": 0.0, "integ_ke": 0.0,
                "last_thr_dem": fp.trim_throttle, "last_pitch_dem": theta, "thr_dem": 0.0,
                "ste_dot_err_last": 0.0, "underspeed": 0.0, "under_t": -1e9, "bad_descent": 0.0,
                "thr_clip": 0.0, "pitch_dem_unc": 0.0, "tas_rate_dem": 0.0, "tas_rate_dem_lpf": 0.0,
                "tas_dem_adj": _clip(tas_cruise, tas_min, tas_max) if fp.tecs_start == "warm"
                else tas_state,
            })

        # -- STE rate limits (`_update_STE_rate_lim`; both scalers are 1)
        ste_dot_max = fp.tecs_clmb_max * G
        ste_dot_min = -fp.tecs_sink_min * G
        ste_dot_neg_max = -fp.tecs_sink_max * G

        # -- speed demand (`_update_speed_demand`)
        tas_dem = tas_cruise
        if m["bad_descent"] > 0.5 or m["underspeed"] > 0.5:
            tas_dem = tas_min
        tas_dem = _clip(tas_dem, tas_min, tas_max)
        vel_rate_max = 0.5 * ste_dot_max / tas_state
        vel_rate_neg_max = 0.9 * ste_dot_neg_max / tas_max
        vel_rate_neg_cruise = 0.9 * ste_dot_min / tas_cruise
        vel_rate_min = linear_interpolate(
            vel_rate_neg_max, vel_rate_neg_cruise, tas_state, tas_max, tas_cruise
        )
        prev = m["tas_dem_adj"]
        if (tas_dem - prev) > vel_rate_max * DT:
            tas_dem_adj, tas_rate_dem = prev + vel_rate_max * DT, vel_rate_max
        elif (tas_dem - prev) < vel_rate_min * DT:
            tas_dem_adj, tas_rate_dem = prev + vel_rate_min * DT, vel_rate_min
        else:
            tas_rate_dem, tas_dem_adj = (tas_dem - prev) / DT, tas_dem
        if reset:
            if fp.tecs_start == "cold":
                tas_dem_adj = tas_state
            m["tas_rate_dem_lpf"] = tas_rate_dem
        else:
            a = DT / (DT + tc)
            m["tas_rate_dem_lpf"] = m["tas_rate_dem_lpf"] * (1.0 - a) + tas_rate_dem * a
        tas_dem_adj = _clip(tas_dem_adj, tas_min, tas_max)
        m["tas_dem_adj"], m["tas_rate_dem"] = tas_dem_adj, tas_rate_dem

        # -- height demand (`_update_height_demand`): constant, so the chain is the constant
        hgt_dem, hgt_rate_dem = fp.h_start, 0.0

        # -- underspeed (`_detect_underspeed`; the source clears it after 3 s above 1.15 x min)
        if m["underspeed"] > 0.5 and tas_state >= tas_min * 1.15 and (t - m["under_t"]) > 3.0:
            m["underspeed"] = 0.0
        if vtol:
            m["underspeed"] = 0.0
        elif (tas_state < tas_min * 0.9 and m["thr_dem"] >= thr_max_f * 0.95) or (
            h < hgt_dem and m["underspeed"] > 0.5
        ):
            m["underspeed"] = 1.0
            if tas_state < tas_min * 0.9:
                m["under_t"] = t
        else:
            m["underspeed"] = 0.0

        # -- energies (`_update_energies`)
        spe_dem = hgt_dem * G
        ske_dem = 0.5 * tas_dem_adj * tas_dem_adj
        ske_dot_dem = tas_state * (tas_rate_dem - m["tas_rate_dem_lpf"])
        spe_est = h * G
        ske_est = 0.5 * tas_state * tas_state
        spe_dot = vh * G
        ske_dot = tas_state * (vel_dot - m["vel_dot_lpf"])

        # -- pitch (`_update_pitch`)
        ske_w = _clip(fp.tecs_spd_weight, 0.0, 2.0)
        if vtol:
            ske_w = 0.0  # pitch follows height only: the usual speed-height link is broken
        elif m["underspeed"] > 0.5:
            ske_w = 2.0
        spe_w = min(2.0 - ske_w, 1.0)
        ske_w = min(ske_w, 1.0)
        seb_dem = spe_dem * spe_w - ske_dem * ske_w
        seb_est = spe_est * spe_w - ske_est * ske_w
        seb_error = seb_dem - seb_est
        sebdot_dem = hgt_rate_dem * G * spe_w + seb_error / tc
        sebdot_dem = _clip(sebdot_dem, -fp.tecs_sink_max * G, fp.tecs_clmb_max * G)
        sebdot_est = spe_dot * spe_w - ske_dot * ske_w
        sebdot_error = sebdot_dem - sebdot_est
        sebdot_dem_total = sebdot_dem + sebdot_error * fp.tecs_ptch_damp
        gain_inv = tas_state * G
        integ_min = gain_inv * (pitch_min_f - math.radians(5.0)) - sebdot_dem_total
        integ_max = gain_inv * (pitch_max_f + math.radians(5.0)) - sebdot_dem_total
        integ_range = integ_max - integ_min
        integ_delta = _clip(
            sebdot_error * fp.tecs_integ_gain * DT, -integ_range * 0.1, integ_range * 0.1
        )
        unc = (sebdot_dem_total + m["integ_seb"] + integ_delta + m["integ_ke"]) / gain_inv
        inhibit = (unc > pitch_max_f and integ_delta > 0.0) or (
            unc < pitch_min_f and integ_delta < 0.0
        )
        if not inhibit:
            m["integ_seb"] += integ_delta
            m["integ_ke"] += (ske_est - ske_dem) * ske_w * DT / tc
        else:  # fade out the integrators while saturating
            coef = 1.0 - DT / (DT + tc)
            m["integ_seb"] *= coef
            m["integ_ke"] *= coef
        m["integ_seb"] = _clip(m["integ_seb"], integ_min, integ_max)
        ke_limit = 0.25 * (pitch_max_f - pitch_min_f) * gain_inv
        m["integ_ke"] = _clip(m["integ_ke"], -ke_limit, ke_limit)
        pitch_unc = (sebdot_dem_total + m["integ_seb"] + m["integ_ke"]) / gain_inv
        m["pitch_dem_unc"] = pitch_unc
        pitch_dem = _clip(pitch_unc, pitch_min_f, pitch_max_f)
        incr = DT * fp.tecs_vert_acc / tas_state  # the vertical-acceleration rate limit
        if (pitch_dem - m["last_pitch_dem"]) > incr:
            pitch_dem = m["last_pitch_dem"] + incr
        elif (pitch_dem - m["last_pitch_dem"]) < -incr:
            pitch_dem = m["last_pitch_dem"] - incr
        m["last_pitch_dem"] = pitch_dem

        # -- throttle (`_update_throttle_with_airspeed`)
        spe_err_max = max(ske_est - 0.5 * tas_min * tas_min, 0.0)
        spe_err_min = min(ske_est - 0.5 * tas_max * tas_max, 0.0)
        if vtol:  # potential energy errors are ignored: the vertical motors interfere
            spe_err_max = spe_err_min = 0.0
        spe_dot_dem = (spe_dem - spe_est) / tc
        ste_error = _clip(spe_dem - spe_est, spe_err_min, spe_err_max) + ske_dem - ske_est
        ste_dot_dem = _clip(spe_dot_dem + ske_dot_dem, ste_dot_min, ste_dot_max)
        ste_dot_error = ste_dot_dem - spe_dot - ske_dot
        filt = 2.0 * DT  # 0.5 s first-order filter
        ste_dot_error = filt * ste_dot_error + (1.0 - filt) * m["ste_dot_err_last"]
        m["ste_dot_err_last"] = ste_dot_error
        if m["underspeed"] > 0.5:
            thr_dem = 1.0
        else:
            k_thr2ste = (ste_dot_max - ste_dot_min) / (thr_max_f - thr_min_f)
            k_ste2thr = 1.0 / (tc * k_thr2ste)
            ff_throttle = fp.trim_throttle + ste_dot_dem / k_thr2ste
            thr_dem = (ste_error + ste_dot_error * fp.tecs_thr_damp) * k_ste2thr + ff_throttle
            thr_min_clipped = _clip(thr_min_f, 0.0, thr_max_f)
            max_amp = 0.5 * (thr_max_f - thr_min_clipped)
            i_max = _clip(thr_max_f - thr_dem + 0.1, -max_amp, max_amp)
            i_min = _clip(thr_min_f - thr_dem - 0.1, -max_amp, max_amp)
            m["integ_thr"] += (ste_error * fp.tecs_integ_gain) * DT * k_ste2thr
            m["integ_thr"] = _clip(m["integ_thr"], i_min, i_max)
            if fp.thr_slewrate != 0.0:  # throttle rate limit before the integrator is added
                incr_thr = DT * (thr_max_f - thr_min_clipped) * fp.thr_slewrate
                thr_dem = _clip(thr_dem, m["last_thr_dem"] - incr_thr, m["last_thr_dem"] + incr_thr)
                m["last_thr_dem"] = thr_dem
            thr_dem += m["integ_thr"]
        if thr_dem > thr_max_f:  # constrain_throttle
            m["thr_clip"], thr_dem = 1.0, thr_max_f
        elif thr_dem < thr_min_f:
            m["thr_clip"], thr_dem = -1.0, thr_min_f
        else:
            m["thr_clip"] = 0.0
        m["thr_dem"] = thr_dem

        # -- bad descent (`_detect_bad_descent`)
        if vtol or m["underspeed"] > 0.5:
            m["bad_descent"] = 0.0
        else:
            ste_dot = spe_dot + ske_dot
            if (ste_error > 200.0 and ste_dot < 0.0 and thr_dem >= thr_max_f * 0.9) or (
                m["bad_descent"] > 0.5 and ste_error > 0.0
            ):
                m["bad_descent"] = 1.0
            else:
                m["bad_descent"] = 0.0
        return pitch_dem, thr_dem, m


class ForwardController:
    """Hover to hand-over to fade to cruise: modes `fwd_wait`, `fwd_fade`, `fwd_done`.

    `t_handover` and `t_done` are set when the modes change (the `settled` event of analysis FT
    reads `t_done`); a controller object belongs to one run. The throttle mix of the fade (0.5 x
    scale) is not modelled: a point mass has no differential thrust.
    """

    def __init__(self, fp: ForwardParams, ff: LiftFF, z_i0: float) -> None:
        self.fp = fp
        self.tecs = TecsLite(fp)
        self._z = Position1Controller(fp.back, ff, z_i0=z_i0)
        self.t_handover: float | None = None
        self.t_done: float | None = None

    def command(
        self, t: float, x: np.ndarray, aux: dict[str, float], mode: ModeState | None
    ) -> tuple[np.ndarray, ModeState]:
        fp = self.fp
        if mode is None:
            mode = ModeState("fwd_wait", t_enter=t, t_last=t, memory={"t0": t})
        mem = dict(mode.memory)
        name, t_enter = mode.name, mode.t_enter
        dt = t - mode.t_last
        h, vx, vh, theta = float(x[1]), float(x[2]), float(x[3]), float(x[4])
        eas = aux["V"] * math.sqrt(aux["rho"] / RHO0)

        # 1. mode logic: the hand-over is the first step with EAS above AIRSPEED_MIN (O-14)
        if name == "fwd_wait" and eas > fp.airspeed_min_eas:
            name, t_enter = "fwd_fade", t
            mem["last_throttle"] = mem.get("rotor_prev", 0.0)
            mem["f_d"] = mem["last_throttle"]
            mem["t_handover"] = t
            if self.t_handover is None:
                self.t_handover = t
        if name == "fwd_fade" and t - t_enter >= fp.q_transition_s:
            name, t_enter = "fwd_done", t
            mem["t_done"] = t
            if self.t_done is None:
                self.t_done = t

        # 2. rotors
        if name == "fwd_wait":
            ramp = min(1.0, max(t - mem["t0"], 0.0) / Z_RAMP_S)
            v_dem = ramp * (fp.h_start - h) / Z_SPREAD_S
            v_dem = _clip(v_dem, -fp.back.z_vmax, fp.back.z_vmax)
            if fp.level_transition:
                v_dem = min(v_dem, 0.0)
            delta_r, zmem = self._z.z_loop(dt, v_dem, vh, theta, mem)
            mem.update(zmem)
        elif name == "fwd_fade":
            span = fp.q_transition_s
            cmd = mem["last_throttle"] * max(0.01, (span - (t - t_enter)) / span)
            if fp.back.rotor_lag_s > 0.0 and dt > 0.0:
                mem["f_d"] += (1.0 - math.exp(-dt / fp.back.rotor_lag_s)) * (cmd - mem["f_d"])
                cmd = mem["f_d"]
            delta_r = cmd
        else:
            delta_r = 0.0
        mem["rotor_prev"] = delta_r

        # 3. TECS-lite with the limits of the mode (O-16, O-17) and its flight stage (O-21)
        cap = pitch_cap_deg(fp, name, vx)
        lim = throttle_limits(fp, name)
        stage = "normal" if name == "fwd_done" else "vtol"
        pitch_dem, thr_dem, tmem = self.tecs.step(
            {k[2:]: v for k, v in mem.items() if k.startswith("k_")}, t, dt, aux["V"], aux["rho"],
            h, vh, theta, lim, (-cap, cap), stage,
        )  # fmt: skip
        mem.update({f"k_{k}": v for k, v in tmem.items()})
        return np.array([delta_r, thr_dem, pitch_dem]), ModeState(name, t_enter, t, mem)
