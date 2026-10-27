"""Analysis FT, the model (docs/FT_spec.md section 8, tests 1 to 8 and 11); no search."""

import math
from pathlib import Path

import numpy as np
import pytest

from sagetrans import atmosphere
from sagetrans.analysis import forward
from sagetrans.control.ardupilot_like import BackTransitionParams, LiftFF
from sagetrans.control.forward import (
    ForwardController,
    ForwardParams,
    TecsLite,
    linear_interpolate,
    pitch_cap_deg,
    throttle_limits,
)
from sagetrans.io.inputs import load_inputs
from sagetrans.physics import aero
from sagetrans.physics.params import placeholder_vehicle

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
VP = placeholder_vehicle()
RHO0 = float(atmosphere.density(0.0))
G = 9.80665


def _fp(lag: float = 0.0, **kw: float) -> ForwardParams:
    """Forward parameters over the ArduPilot Z loop (no rotor lag unless asked)."""
    back = BackTransitionParams(z_mode="ardupilot", h_tgt=0.0, rotor_lag_s=lag)
    kw.setdefault("airspeed_cruise_eas", 20.0)
    return ForwardParams(back=back, h_start=100.0, **kw)


def _controller(fp: ForwardParams, rho: float = RHO0) -> ForwardController:
    ff = LiftFF(VP, rho, 0.8, t_max_factor=3.0, n=30)
    return ForwardController(fp, ff, z_i0=ff.weight / ff.thrust_for_delta(fp.back.delta_max))


def _drive(ctl: ForwardController, speeds: list[float], rho: float, dt: float = 0.02):
    """Run the controller on a scripted true airspeed (`speeds[k]` at step k)."""
    mode, out = None, []
    for k, v in enumerate(speeds):
        x = np.array([0.0, ctl.fp.h_start, 6.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0])
        u, mode = ctl.command(k * dt, x, {"V": v, "rho": rho}, mode)
        out.append((k * dt, mode.name, u))
    return out


def _tecs(fp: ForwardParams, steps: int, v: float, h: float, lim, ext, stage: str):
    """Step TecsLite `steps` times at a fixed state; returns (pitch_dem, throttle_dem)."""
    tecs, mem, out = TecsLite(fp), {}, (0.0, 0.0)
    for k in range(steps):
        p, t, mem = tecs.step(mem, k * 0.02, 0.02, v, RHO0, h, 0.0, 0.0, lim, ext, stage)
        out = (p, t)
    return out


# ---- the mode machine (spec 8.1, 8.2) ------------------------------------------------------------
def test_mode_machine_waits_for_the_airspeed_fades_for_the_time_then_is_done():
    ctl = _controller(_fp(airspeed_min_eas=17.0, q_transition_s=2.0))
    speeds = [10.0] * 50 + [16.99] * 10 + [17.01] * 300  # the 61st sample crosses 17.0
    trace = _drive(ctl, speeds, RHO0)
    names = [n for _, n, _ in trace]
    assert names[0] == "fwd_wait" and names[59] == "fwd_wait" and names[60] == "fwd_fade"
    assert ctl.t_handover == pytest.approx(60 * 0.02)
    assert names.count("fwd_fade") == round(2.0 / 0.02)  # the fade lasts Q_TRANSITION_MS
    assert names[-1] == "fwd_done" and ctl.t_done == pytest.approx(ctl.t_handover + 2.0)
    assert trace[-1][2][0] == 0.0  # DONE: rotors off


def test_the_handover_compares_equivalent_airspeed_so_the_true_speed_rises_with_altitude():
    rho = float(atmosphere.density(5000.0))
    ctl = _controller(_fp(airspeed_min_eas=17.0, q_transition_s=2.0), rho)
    v_true = 17.0 / math.sqrt(rho / RHO0)  # the true airspeed at which EAS = AIRSPEED_MIN
    ramp = list(np.linspace(8.0, v_true + 1.0, 400))
    _drive(ctl, ramp, rho)
    k = next(i for i, v in enumerate(ramp) if v * math.sqrt(rho / RHO0) > 17.0)
    assert ctl.t_handover == pytest.approx(k * 0.02)
    assert ramp[k] == pytest.approx(v_true, abs=0.05) and v_true > 21.0  # 17 EAS is about 22 TAS


# ---- the limits (spec 8.3, 8.4) ------------------------------------------------------------------
def test_pitch_caps_follow_the_mode_and_the_ground_speed():
    fp = _fp()
    assert pitch_cap_deg(fp, "fwd_wait", 5.0) == 3.0  # Q_TRAN_PIT_MAX in WAIT
    assert pitch_cap_deg(fp, "fwd_wait", 2.0) == 0.0  # none below 3 m/s ground speed
    assert pitch_cap_deg(fp, "fwd_fade", 0.0) == 8.0  # (3 + 1) x 2 in the fade
    assert pitch_cap_deg(_fp(tran_pit_max_deg=5.0), "fwd_fade", 9.0) == 12.0
    assert pitch_cap_deg(fp, "fwd_done", 9.0) == 90.0  # lifted
    # TECS honours it: a large height error saturates the pitch demand at the cap, not above
    pitch, _ = _tecs(fp, 400, 15.0, 40.0, (0.0, 1.0), (-3.0, 3.0), "vtol")
    assert pitch == pytest.approx(math.radians(3.0))


def test_pusher_clamp_low_trim_high_tkoff_and_a_range_under_one_percent_is_widened():
    fp = _fp(trim_throttle=0.45, tkoff_thr_max=0.8)
    assert throttle_limits(fp, "fwd_wait") == (0.45, 0.8)  # minimum is TRIM (TKOFF_THR_MIN is 0)
    assert throttle_limits(_fp(tkoff_thr_min=0.3), "fwd_fade")[0] == 0.3
    assert throttle_limits(_fp(tkoff_thr_max=0.0), "fwd_wait")[1] == 1.0  # 0 means THR_MAX
    assert throttle_limits(fp, "fwd_done") == (0.0, 1.0)
    ext = (-3.0, 3.0)
    assert _tecs(fp, 600, 3.0, 100.0, (0.45, 0.8), ext, "vtol")[1] == pytest.approx(0.8)
    assert _tecs(fp, 600, 30.0, 100.0, (0.45, 0.8), ext, "vtol")[1] == pytest.approx(0.45)
    assert _tecs(fp, 600, 3.0, 100.0, (0.45, 0.452), ext, "vtol")[1] == pytest.approx(0.46)


# ---- the fade (spec 8.5) -------------------------------------------------------------------------
def test_the_fade_throttle_falls_linearly_from_the_last_wait_command():
    ctl = _controller(_fp(airspeed_min_eas=17.0, q_transition_s=4.0))
    trace = _drive(ctl, [10.0] * 100 + [17.5] * 400, RHO0)
    wait_cmd = trace[99][2][0]  # the last WAIT command
    fade = [(t, u[0]) for t, n, u in trace if n == "fwd_fade"]
    assert fade[0][1] == pytest.approx(wait_cmd)  # starts where WAIT left off
    t0 = ctl.t_handover
    for t, d in fade[::25]:
        assert d == pytest.approx(wait_cmd * max(0.01, (4.0 - (t - t0)) / 4.0))
    assert fade[-1][1] < 0.02 * wait_cmd  # ends near the 0.01 floor


# ---- TECS-lite (spec 8.6) ------------------------------------------------------------------------
def test_tecs_holds_trim_throttle_and_level_pitch_when_the_state_meets_the_demand():
    pitch, thr = _tecs(_fp(trim_throttle=0.45), 500, 20.0, 100.0, (0.0, 1.0), (-90.0, 90.0),
                       "normal")  # fmt: skip
    assert abs(pitch) < 1e-6 and thr == pytest.approx(0.45, rel=0.01)


def test_tecs_pitch_demand_is_rate_limited_by_the_vertical_acceleration():
    fp = _fp()
    tecs, mem, prev, steps = TecsLite(fp), {}, 0.0, []
    for k in range(60):  # 50 m below the demanded height: the demand saturates quickly
        pitch, _, mem = tecs.step(mem, k * 0.02, 0.02, 16.0, RHO0, 50.0, 0.0, 0.0,
                                  (0.0, 1.0), (-90.0, 90.0), "vtol")  # fmt: skip
        steps.append(pitch - prev)
        prev = pitch
    limit = 0.02 * fp.tecs_vert_acc / 16.0
    assert max(steps) == pytest.approx(limit, rel=1e-6) and max(steps) <= limit + 1e-12


def test_the_vtol_stage_ignores_the_speed_in_the_pitch_and_the_height_in_the_energy_error():
    def run(stage: str, eas_dem: float, h: float) -> tuple[float, float, float]:
        fp = _fp(airspeed_cruise_eas=eas_dem)
        tecs, mem, out = TecsLite(fp), {}, (0.0, 0.0)
        for k in range(150):
            p, t, mem = tecs.step(mem, k * 0.02, 0.02, 19.0, RHO0, h, 0.0, 0.0,
                                  (0.0, 1.0), (-90.0, 90.0), stage)  # fmt: skip
            out = (p, t)
        return out[0], out[1], mem["integ_thr"]

    # pitch: a different speed demand changes it in NORMAL, not in VTOL
    assert run("vtol", 20.0, 100.0)[0] == pytest.approx(run("vtol", 24.0, 100.0)[0])
    assert run("normal", 20.0, 100.0)[0] != pytest.approx(run("normal", 24.0, 100.0)[0])
    # throttle: with the speed at its demand, the height error does not enter the energy error
    # (`SPE_err_max = SPE_err_min = 0`), so the throttle integrator stays at 0 in VTOL and winds
    # up in NORMAL. The source keeps the height error in the feed-forward RATE term
    # (SPEdot_dem enters STEdot_dem) in both stages, so the throttle itself still moves a little:
    # this is where the pinned source is not "ignores the height error" (FT spec 8.6, O-21).
    assert run("vtol", 19.0, 96.0)[2] == pytest.approx(0.0, abs=1e-9)
    assert abs(run("normal", 19.0, 96.0)[2]) > 1e-4


def test_linear_interpolate_matches_ap_math_for_either_polarity():
    assert linear_interpolate(0.0, 10.0, 5.0, 0.0, 10.0) == 5.0
    assert linear_interpolate(0.0, 10.0, 5.0, 10.0, 0.0) == 5.0  # reversed input range
    assert linear_interpolate(1.0, 2.0, -3.0, 0.0, 1.0) == 1.0  # clamped low
    assert linear_interpolate(1.0, 2.0, 3.0, 0.0, 1.0) == 2.0  # clamped high
    assert linear_interpolate(7.0, 9.0, 4.0, 4.0, 4.0) == 7.0  # equal limits: the low output


# ---- recovery acceleration and the wing share (spec 8.7, 8.8) ---------------------------------
def test_recovery_acceleration_follows_the_wing_limit_at_the_cl_band_and_is_capped_by_tecs():
    vs = aero.stall_speed(VP.m, RHO0, VP.wing)
    for v in (1.2 * vs, 1.5 * vs):
        a_wing, a_rec, binding = forward.recovery_accel(VP, RHO0, v, 0.8, 7.0)
        assert a_wing == pytest.approx(G * (0.8 * (v / vs) ** 2 - 1.0), rel=1e-9)
        assert a_rec == min(a_wing, 7.0) and binding == ("wing" if a_wing <= 7.0 else "tecs")
    assert forward.recovery_accel(VP, RHO0, 3.0 * vs, 0.8, 7.0)[2] == "tecs"
    assert forward.projected_loss(-3.0, 1.5) == pytest.approx(9.0 / (2.0 * 1.5))
    assert forward.projected_loss(0.4, 1.5) == 0.0 and math.isnan(forward.projected_loss(-1.0, 0.0))


def test_wing_share_of_the_weight_at_the_pitch_caps_matches_the_spec_estimate():
    live = load_inputs(ROOT / "configs").vehicle
    if abs(live.m - 26.664) > 0.01 or abs(live.wing.S - 1.617) > 1e-3:
        pytest.skip("the live register no longer holds the values behind the spec estimate")
    for deg, share in ((3.0, 0.62), (8.0, 1.02)):  # spec section 1.4, 15.9 m/s EAS, sea level
        lift = float(aero.forces(RHO0, 15.9, math.radians(deg) + live.wing.i_w, live.wing)[0])
        assert lift / (live.m * G) == pytest.approx(share, rel=0.05)


# ---- one run (spec 8.11) -------------------------------------------------------------------------
def test_run_ft_is_deterministic_and_reports_a_consistent_row():
    inp = load_inputs(CONFIGS)
    m = inp.mission
    cfg = forward.FTEval(
        template=inp.forward, loss_max_m=m.fwd_alt_loss_max_m, gain_max_m=m.fwd_alt_gain_max_m
    )
    scen = forward.ft_scenario(inp.scenarios["S-01"], m.clearance_agl_m)
    assert scen.ground_amsl_m == 0.0 and scen.altitude_amsl_m == m.clearance_agl_m
    prm = {"airspeed_min": 17.0, "q_transition": 5.0, "tkoff_thr_max": 0.8, "tran_pit_max": 3.0}
    a = forward.run_ft(inp.vehicle, scen, prm, cfg)
    b = forward.run_ft(inp.vehicle, scen, prm, cfg)
    assert np.array_equal(a.sim.t, b.sim.t)
    for col in ("h", "V", "theta", "delta_p", "delta_r"):
        assert np.array_equal(a.sim[col], b.sim[col])
    assert a.t_handover is not None and a.t_done == pytest.approx(a.t_handover + 5.0, abs=0.03)
    row = a.row
    assert row["h_proj_rel_m"] == pytest.approx(row["h_end_rel_m"] - row["proj_loss_m"])
    assert row["V_handover_eas_ms"] > 17.0 and row["energy_kJ"] == pytest.approx(
        row["E_wait_kJ"] + row["E_fade_kJ"], rel=0.02
    )
