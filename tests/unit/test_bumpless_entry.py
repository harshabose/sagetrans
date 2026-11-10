"""A bumpless AIRBRAKE exit: theta_fw0_deg "trim" and the release of the pitch command."""

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from sagetrans.analysis import braking, restricted
from sagetrans.control.ardupilot_like import BackTransitionParams, Position1Controller
from sagetrans.dynamics import trim
from sagetrans.io import stages
from sagetrans.io.assemble import e_scenarios
from sagetrans.io.inputs import load_inputs
from sagetrans.io.settings import DCfg

ROOT = Path(__file__).resolve().parents[2]


def test_the_entry_pitch_setting_is_a_number_or_the_word_trim():
    assert DCfg().theta_fw0_deg == 0.0 and DCfg().pitch_release_dps == 0.0  # off unless asked
    assert DCfg(theta_fw0_deg="trim").theta_fw0_deg == "trim"
    assert DCfg(theta_fw0_deg=4.5, pitch_release_dps=2.0).theta_fw0_deg == 4.5
    with pytest.raises(ValidationError):
        DCfg(theta_fw0_deg="level")
    with pytest.raises(ValidationError):
        DCfg(pitch_release_dps=-1.0)
    shipped = load_inputs(ROOT / "configs", only={"D"}).study.D
    assert shipped.theta_fw0_deg == "trim" and shipped.pitch_release_dps == 2.0


def test_exit_params_carry_the_flag_for_trim_and_the_number_otherwise():
    inp = load_inputs(ROOT / "configs", only={"D"})
    bp = stages.d_exit_params(inp)
    assert bp.theta_fw0_trim is True and bp.theta_fw0 == 0.0
    assert bp.pitch_release_rate == pytest.approx(math.radians(2.0))
    numeric = dataclasses.replace(inp, study=inp.study.model_copy(update={
        "D": inp.study.D.model_copy(update={"theta_fw0_deg": 6.0, "pitch_release_dps": 0.0})}))
    bp = stages.d_exit_params(numeric)
    assert bp.theta_fw0_trim is False and bp.theta_fw0 == pytest.approx(math.radians(6.0))
    assert bp.pitch_release_rate == 0.0


def test_trim_resolves_per_scenario_to_the_level_flight_pitch_of_its_entry_state():
    inp = load_inputs(ROOT / "configs", only={"D"})
    bp = stages.d_exit_params(inp)
    vp = inp.vehicle
    out = {}
    for name in ("S-01", "S-03", "S-06"):
        sc = inp.scenarios[name]
        v_air = braking.entry_airspeed(inp.airspeed_min, sc.rho, inp.study.D.entry_convention)
        r = braking.resolve_entry_pitch(vp, sc, bp, v_air)
        want = trim.steady_trim(sc.vehicle_for(vp), sc.rho, v_air, 0.0, sc.soc).theta
        assert r.theta_fw0 == pytest.approx(want) and r.theta_fw0_trim is False
        out[name] = r.theta_fw0
    assert math.degrees(out["S-01"]) == pytest.approx(9.1, abs=0.1)  # 15 m/s EAS at sea level
    assert out["S-03"] > out["S-01"] > out["S-06"]  # heavier needs more pitch, lighter less
    # a number is returned untouched, and resolving twice changes nothing
    numeric = dataclasses.replace(bp, theta_fw0_trim=False, theta_fw0=0.1)
    sc = inp.scenarios["S-01"]
    assert braking.resolve_entry_pitch(vp, sc, numeric, 15.0) is numeric
    once = braking.resolve_entry_pitch(vp, sc, bp, 15.0)
    assert braking.resolve_entry_pitch(vp, sc, once, 15.0) is once


def test_the_entry_state_starts_at_the_trim_pitch_with_the_weight_carried():
    inp = load_inputs(ROOT / "configs", only={"D"})
    bp = stages.d_exit_params(inp)
    sc = inp.scenarios["S-01"]
    x0, _ = braking.airbrake_entry_state(inp.vehicle, sc, bp, 15.0)
    assert math.degrees(x0[4]) == pytest.approx(9.1, abs=0.1)
    from sagetrans.physics import aero

    lift = float(aero.forces(sc.rho, 15.0, x0[4] + inp.vehicle.wing.i_w, inp.vehicle.wing)[0])
    assert lift == pytest.approx(inp.vehicle.m * 9.80665, rel=0.02)


def test_trim_with_no_solution_says_what_to_do():
    inp = load_inputs(ROOT / "configs", only={"D"})
    bp = stages.d_exit_params(inp)
    with pytest.raises(ValueError, match="D.theta_fw0_deg is 'trim'.*give a number"):
        braking.resolve_entry_pitch(inp.vehicle, inp.scenarios["S-01"], bp, 5.0)


def test_an_unresolved_parameter_set_cannot_run():
    bp = BackTransitionParams(theta_fw0_trim=True)
    with pytest.raises(ValueError, match="resolve the entry pitch"):
        Position1Controller(bp)


def _first_seconds(inp, name, trim_flag, rate_dps, q=1.5):
    cev, base, _ = stages.c_settings(inp)
    base = dataclasses.replace(base, theta_fw0_trim=trim_flag, theta_fw0=0.0,
                               pitch_release_rate=math.radians(rate_dps))
    prm = {**{k: inp.start_params[k] for k in ("T_bt", "theta_A", "spin_min")}, "q": q}
    sim = restricted.run_c(inp.vehicle, inp.scenarios[name], prm, cev, base).sim
    t = np.asarray(sim.t)
    az = np.gradient(np.asarray(sim["vh"]), t)
    return sim, t, np.abs(az[t <= 1.5]).max()


def test_the_start_is_bumpless_with_trim_and_the_release_and_not_without():
    inp = load_inputs(ROOT / "configs", only={"C", "D"})
    _, _, untrimmed = _first_seconds(inp, "S-01", False, 0.0)
    _, _, trimmed = _first_seconds(inp, "S-01", True, 0.0)
    sim, t, bumpless = _first_seconds(inp, "S-01", True, 2.0)
    assert untrimmed > 5.0  # the lift deficit of a level entry at 15 m/s
    assert 2.0 < trimmed < untrimmed  # trim alone leaves the pitch command at 0
    assert bumpless < 0.6  # trim and the release: below a tenth of a g
    # the command starts at the entry pitch and is never below its release line
    cmd = np.degrees(np.asarray(sim["theta_cmd"]))
    entry = math.degrees(float(np.asarray(sim["theta"])[0]))
    assert cmd[0] == pytest.approx(min(entry, 10.99), abs=0.05)  # at most the enforced cap
    floor = np.maximum(0.0, entry - 2.0 * t)
    assert np.all(cmd >= np.minimum(floor, 10.99) - 1e-6)


def test_the_release_never_slows_the_braking_it_only_holds_the_pitch_up():
    inp = load_inputs(ROOT / "configs", only={"C", "D"})
    cev, base, _ = stages.c_settings(inp)
    prm = {**{k: inp.start_params[k] for k in ("T_bt", "theta_A", "spin_min")}, "q": 1.5}
    sc = inp.scenarios["S-01"]
    out = {}
    for rate in (0.0, 2.0):
        b = dataclasses.replace(base, theta_fw0_trim=True, pitch_release_rate=math.radians(rate))
        out[rate] = restricted.evaluate_c(inp.vehicle, sc, prm, cev, b)
    assert out[2.0].distance <= out[0.0].distance + 0.5  # not longer: the pitch is not lower
    assert out[2.0].excursion < out[0.0].excursion  # the dip is smaller


def test_the_scenario_table_gives_the_entry_pitch_at_both_settings():
    inp = load_inputs(ROOT / "configs", only={"E", "D", "C"})
    rows = {r["scenario"]: r for r in e_scenarios(inp, {"airspeed_min": 18.0})}
    s1, s3 = rows["S-01"], rows["S-03"]
    assert s1["entry_pitch_register_deg"] == pytest.approx(9.1, abs=0.1)
    # a higher AIRSPEED_MIN gives a faster entry, which needs less pitch to carry the weight
    assert 6.5 < s1["entry_pitch_recommended_deg"] < s1["entry_pitch_register_deg"]
    assert s3["entry_pitch_register_deg"] > s1["entry_pitch_register_deg"]


def test_e_runs_with_the_trim_entry():
    inp = load_inputs(ROOT / "configs", only={"E", "D"})
    cs = stages.solve_e_case(inp, 0.0, "one_way", 1.65, math.nan)
    assert cs.r_max > 50e3
