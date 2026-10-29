"""C's two configurations: stall exposure, the enforced headline, the unconstrained reference."""

import dataclasses
import math
from pathlib import Path

import numpy as np
import pytest
from pydantic import ValidationError

from sagetrans.analysis import braking, restricted
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.ensemble import CostWeights
from sagetrans.dynamics.simulate import Result
from sagetrans.io import recommended, stages
from sagetrans.io.inputs import load_inputs
from sagetrans.io.settings import CCfg
from sagetrans.io.study import _cache_path
from sagetrans.io.view import chapters as chp

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
G = 9.80665


def _with_c(inp, **kw):
    c = inp.study.C.model_copy(update=kw)
    return dataclasses.replace(inp, study=inp.study.model_copy(update={"C": c}))


def _synthetic(vp, alpha_deg, v, lift_over_w):
    """A run of 1 s at 50 Hz with the given wing angle, airspeed and lift share."""
    n = len(alpha_deg)
    t = np.arange(n) * 0.02
    cols = {"alpha": np.radians(alpha_deg) - vp.wing.i_w, "V": np.asarray(v, float),
            "L": np.asarray(lift_over_w, float) * vp.m * G}
    return Result(t=t, columns=cols)


def test_exposure_columns_of_a_stalling_and_a_non_stalling_run():
    vp = load_inputs(CONFIGS).vehicle
    scen = Scenario(0.0)
    peak, limit = math.radians(18.0), math.radians(10.0)
    n = 50
    stalling = _synthetic(vp, np.r_[np.full(10, 5.0), np.full(20, 22.0), np.full(20, 8.0)],
                          np.full(n, 12.0), np.r_[np.full(10, 0.8), np.full(20, 0.3),
                                                  np.full(20, 0.6)])
    calm = _synthetic(vp, np.full(n, 6.0), np.full(n, 12.0), np.full(n, 0.7))
    a = braking.stall_exposure(stalling, vp, scen, alpha_peak=peak, alpha_limit=limit)
    b = braking.stall_exposure(calm, vp, scen, alpha_peak=peak, alpha_limit=limit)
    assert a["stall_alpha_peak_deg"] == pytest.approx(22.0)
    assert a["stall_t_above_peak_s"] == pytest.approx(20 * 0.02)
    assert a["stall_t_above_limit_s"] == pytest.approx(20 * 0.02)
    assert a["stall_wing_share_min"] == pytest.approx(0.3)
    assert b["stall_alpha_peak_deg"] == pytest.approx(6.0)
    assert b["stall_t_above_peak_s"] == 0.0 and b["stall_t_above_limit_s"] == 0.0
    assert b["stall_wing_share_min"] == pytest.approx(0.7)


def test_exposure_skips_slow_samples_and_the_first_sample():
    vp = load_inputs(CONFIGS).vehicle
    # the first sample (t = 0) is at 30 deg and 0 lift; the last ten are below the gate speed
    alpha = np.r_[30.0, np.full(39, 6.0), np.full(10, 40.0)]
    v = np.r_[12.0, np.full(39, 12.0), np.full(10, 2.0)]
    lift = np.r_[0.0, np.full(39, 0.7), np.full(10, 0.0)]
    r = braking.stall_exposure(_synthetic(vp, alpha, v, lift), vp, Scenario(0.0),
                               alpha_peak=math.radians(18.0), alpha_limit=math.radians(10.0))
    assert r["stall_alpha_peak_deg"] == pytest.approx(6.0)
    assert r["stall_wing_share_min"] == pytest.approx(0.7)
    assert r["stall_t_above_peak_s"] == 0.0


def test_a_run_with_no_fast_sample_has_no_exposure_numbers():
    vp = load_inputs(CONFIGS).vehicle
    r = braking.stall_exposure(_synthetic(vp, np.full(5, 9.0), np.full(5, 1.0), np.full(5, 0.0)),
                               vp, Scenario(0.0), alpha_peak=0.3, alpha_limit=0.2)
    assert math.isnan(r["stall_alpha_peak_deg"]) and math.isnan(r["stall_wing_share_min"])


def test_the_summary_over_members():
    rows = [
        {"stall_alpha_peak_deg": 20.0, "stall_t_above_peak_s": 1.0, "stall_t_above_limit_s": 2.0,
         "stall_wing_share_min": 0.2},
        {"stall_alpha_peak_deg": 9.0, "stall_t_above_peak_s": 0.0, "stall_t_above_limit_s": 0.0,
         "stall_wing_share_min": 0.6},
    ]
    s = stages.exposure_summary(rows)
    assert s["stall_alpha_peak_deg_max"] == 20.0 and s["stall_t_above_peak_s_max"] == 1.0
    assert s["stall_members_above_peak"] == 1.0 and s["stall_members"] == 2.0
    assert s["stall_wing_share_min"] == 0.2
    assert stages.exposure_summary([]) == {}


def test_the_enforced_configuration_keeps_the_wing_below_the_lift_peak_and_the_free_one_does_not():
    """Shipped register, S-01, q = 2. The cap limits the pitch COMMAND to the limit plus the
    flight-path angle, so the angle can sit a degree above the limit (the attitude lags); what it
    must not do is reach the C_L peak."""
    inp = _with_c(load_inputs(ROOT / "configs", only={"C"}), enforce_alpha=True)
    wing = inp.vehicle.wing
    cev, held, _ = stages.c_settings(inp)
    _, free, _ = stages.c_settings(inp, enforce=False)
    assert free.alpha_cap is None
    assert held.alpha_cap == pytest.approx(0.7 * wing.alpha_s - wing.i_w)
    _, curve, _ = stages.c_settings(inp, frac=0.85)
    assert curve.alpha_cap == pytest.approx(0.85 * wing.alpha_s - wing.i_w)
    scen = inp.scenarios["S-01"]
    prm = {**{k: inp.start_params[k] for k in ("T_bt", "theta_A", "spin_min")}, "q": 2.0}
    peak = stages.wing_lift_peak(wing)[0]

    def exposure(bp):
        run = restricted.run_c(inp.vehicle, scen, prm, cev, bp)
        return braking.stall_exposure(run.sim, inp.vehicle, scen, alpha_peak=peak,
                                      alpha_limit=0.7 * wing.alpha_s)

    free_ex, held_ex = exposure(free), exposure(held)
    assert free_ex["stall_t_above_peak_s"] > 1.0 and free_ex["stall_alpha_peak_deg"] > 20.0
    assert held_ex["stall_t_above_peak_s"] == 0.0
    assert math.degrees(peak) > held_ex["stall_alpha_peak_deg"] > 0.0
    assert held_ex["stall_wing_share_min"] > free_ex["stall_wing_share_min"]


def test_alpha_options_are_validated():
    with pytest.raises(ValidationError, match="C.enforce_alpha"):
        CCfg(alpha_curve_fracs=[1.0])
    with pytest.raises(ValidationError, match="C.enforce_alpha"):
        CCfg(unconstrained_reference=True)
    for bad in ([0.0], [1.2], [0.85, 0.85]):
        with pytest.raises(ValidationError):
            CCfg(enforce_alpha=True, alpha_curve_fracs=bad)
    assert CCfg().enforce_alpha is False  # off unless asked: the old behaviour


def test_the_configurations_are_distinct_tasks_with_their_own_cache_entries(tmp_path):
    inp = _with_c(load_inputs(CONFIGS, only={"C"}), enforce_alpha=True,
                  unconstrained_reference=True, alpha_curve_fracs=[1.0, 0.85, 0.7])
    tasks = stages.plan_tasks(inp)
    assert tasks[:1] == ["C"] or "C" in tasks
    assert {"C", "C:unconstrained", "C:alpha:1", "C:alpha:0.85"} <= set(tasks)
    assert "C:alpha:0.7" not in tasks  # the headline fraction is not repeated
    paths = {_cache_path(tmp_path, inp, t) for t in tasks}
    assert len(paths) == len(tasks)
    off = _with_c(load_inputs(CONFIGS, only={"C"}), enforce_alpha=False)
    assert [t for t in stages.plan_tasks(off) if t.startswith("C")] == ["C"]


def _tables(inp):
    base = {"case": "optimum", "T_bt_s": 3.0, "theta_A_deg": 15.0, "spin_min": 0.1,
            "at_bound": "q:lo", "max_overshoot_m": 0.5, "max_end_sink_ms": 0.1,
            "p95_distance_m": 240.0, "p95_stop_time_s": 12.0, "alpha_enforced": True,
            "alpha_limit_deg": 11.0}
    return {
        "C_params": [{**base, "Q_TRANS_DECEL_ms2": 1.05}],
        "C_ref_params": [{**base, "Q_TRANS_DECEL_ms2": 1.8, "alpha_enforced": False}],
        "D_recommendation": [{"a_rec_min_ms2": 1.65}],
    }


def test_the_recommended_settings_read_the_enforced_c_only():
    inp = load_inputs(CONFIGS)
    rows = recommended.settings_rows(inp, _tables(inp))
    q = next(r for r in rows if r["parameter"] == "Q_TRANS_DECEL" and r["analysis"] == "C")
    assert q["value"] == pytest.approx(1.05)  # not the unconstrained 1.8
    note = q["note"]
    assert "1.05" in note and "1.65" in note
    assert "240 m" in note and "12.0 s" in note
    assert "provisional: depends on alpha_s, which is a guess" in note
    assert "held to 11.0 deg" in note
    assert q["flags"] == "at lower bound"  # the chip says q sits on its lower bound


def test_nothing_but_the_assembly_and_the_viewer_reads_the_reference_tables():
    allowed = {"assemble.py", "stages.py", "chapters.py", "registry.py", "workbook.py"}
    for p in (ROOT / "src" / "sagetrans").rglob("*.py"):
        text = p.read_text(encoding="utf-8")
        if ("C_ref_params" in text or "C_ref_runs" in text) and p.name not in allowed:
            pytest.fail(f"{p.name} reads the unconstrained reference")
    rec = (ROOT / "src" / "sagetrans" / "io" / "recommended.py").read_text(encoding="utf-8")
    assert "C_ref" not in rec


def test_the_alpha_curve_row_has_the_columns_of_the_table():
    inp = load_inputs(CONFIGS)
    cev, base, w = stages.c_settings(inp)
    prm = {**{k: inp.start_params[k] for k in ("T_bt", "theta_A", "spin_min")}, "q": 2.0}
    runs = [restricted.run_c(inp.vehicle, inp.scenarios["S-01"], prm, cev, base)]
    row = stages.alpha_curve_row("curve", 0.85, 13.3, prm, runs, CostWeights(), inp.study.C, "")
    for k in ("alpha_frac", "limit_deg", "Q_TRANS_DECEL_ms2", "T_bt_s", "theta_A_deg", "spin_min",
              "p95_distance_m", "p95_stop_time_s", "mean_energy_kJ", "n_fail_total",
              "max_overshoot_m", "max_end_sink_ms"):
        assert k in row
    assert row["alpha_frac"] == 0.85 and row["p95_stop_time_s"] == pytest.approx(runs[0].sim.t[-1])
    free = stages.alpha_curve_row("unconstrained", None, None, prm, runs, CostWeights(),
                                  inp.study.C)
    assert free["alpha_frac"] == "" and free["limit_deg"] == ""


def test_the_caveats_are_computed_from_the_values():
    p = {"alpha_s_deg": 15.7, "blend_M": 70.0, "cl_max": 1.444, "alpha_peak_deg": 13.43,
         "rotor_constant_coeffs": True}
    html = chp.caveats_panel(p)
    assert "15.70 deg" in html and "M = 70" in html and "1.444" in html and "13.43" in html
    assert "constant thrust and torque coefficients" in html
    html2 = chp.caveats_panel({**p, "blend_M": 25.0, "alpha_s_deg": 18.0,
                               "rotor_constant_coeffs": False})
    assert "M = 25" in html2 and "18.00 deg" in html2
    assert "constant thrust and torque" not in html2  # not claimed when the register has J terms
    assert chp.caveats_panel({}) == ""


def test_the_configuration_figures_name_the_reference_as_such():
    head = {"case": "optimum", "alpha_enforced": True, "alpha_limit_deg": 10.99,
            "Q_TRANS_DECEL_ms2": 1.05, "mean_energy_kJ": 38.0, "p95_distance_m": 240.0,
            "p95_stop_time_s": 12.0, "stall_members_above_peak": 0.0, "stall_members": 4.0}
    ref = [{**head, "alpha_enforced": False, "Q_TRANS_DECEL_ms2": 1.8, "mean_energy_kJ": 19.0,
            "p95_distance_m": 125.0, "p95_stop_time_s": 6.0, "stall_members_above_peak": 4.0}]
    html = chp.configurations_figure([head], 0, ref)
    assert html is not None
    assert "the model ignores the stall; reference only; never a recommendation" in html
    assert "2.00 x the energy" in html and "1.92 x the stopping distance" in html
    assert "4 of 4" in html and "0 of 4" in html
    assert chp.configurations_figure([head], 0, []) is None
    curve = [{"kind": "unconstrained", "alpha_frac": ""}, {"kind": "headline", "alpha_frac": 0.7,
                                                           "limit_deg": 10.99}]
    assert "the model ignores the stall" in (chp.alpha_curve_figure(curve) or "")
