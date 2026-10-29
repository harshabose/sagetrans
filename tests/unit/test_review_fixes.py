"""Review fixes: the trim bracket, a 0 setting is a setting, and what the FT ensemble really is."""

import math
from pathlib import Path

import pytest

from sagetrans.analysis import runs
from sagetrans.analysis.ensemble import braking_ensemble
from sagetrans.dynamics import trim
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs
from sagetrans.physics import aero

CONFIGS = Path(__file__).resolve().parents[1] / "data" / "placeholder_configs"


def test_the_lift_peak_comes_before_alpha_s_and_the_search_finds_it():
    wing = load_inputs(CONFIGS).vehicle.wing
    peak = aero.alpha_at_cl_max(wing)
    assert peak < wing.alpha_s
    assert float(aero.coefficients(peak, wing)[0]) == pytest.approx(aero.cl_max(wing), rel=1e-4)


def test_steady_and_glide_trim_exist_between_the_stall_edge_and_the_lift_peak_speed():
    """A trim exists wherever the lift peak carries the weight, not only where alpha_s does."""
    vp = load_inputs(CONFIGS).vehicle
    rho, w = 1.225, vp.m * 9.80665
    v = math.sqrt(2.0 * w * 1.05 / (rho * vp.wing.S * runs.cl_max(vp.wing)))  # 5 % above the peak
    # the old bracket stopped at alpha_s: the lift there is below the weight at this speed
    assert float(aero.forces(rho, v, vp.wing.alpha_s, vp.wing)[0]) < w
    tr = trim.steady_trim(vp, rho, v, 0.0, 0.8)
    assert tr.alpha + vp.wing.i_w < aero.alpha_at_cl_max(vp.wing)
    assert tr.L + tr.D * math.tan(tr.alpha) == pytest.approx(w, rel=1e-6)
    gl = trim.glide_trim(vp, rho, v, 0.8)
    assert gl.alpha + vp.wing.i_w < aero.alpha_at_cl_max(vp.wing)


def test_a_trim_found_before_the_fix_is_unchanged():
    """Where the old bracket worked (a root below the peak) the root is the same."""
    vp = load_inputs(CONFIGS).vehicle
    rho = 1.225
    tr = trim.steady_trim(vp, rho, 22.0, 0.0, 0.8)
    assert tr.alpha < aero.alpha_at_cl_max(vp.wing)


@pytest.mark.parametrize(
    "value,default,expected", [(None, 3, 3), (0, 3, 0), (0.0, 1.5, 0.0), ("", "x", "")]
)
def test_first_set_treats_zero_and_empty_as_settings(value, default, expected):
    assert stages.first_set(value, default) == expected


def test_no_b1_setting_is_chosen_with_or():
    src = (Path(stages.__file__)).read_text(encoding="utf-8")
    for needle in ("band or end_band", "theta_deg or ", "rs or c.rotor_start",
                   "end_vh_band_ms or "):
        assert needle not in src


def test_ft_members_are_ds_sobol_sample_and_not_cs_latin_hypercube():
    """The comment said FT used C's Sobol sample. C draws a Latin hypercube; FT draws D's Sobol."""
    inp = load_inputs(CONFIGS)
    vp = inp.vehicle
    c_members = stages._ensemble(inp, 4, inp.study.C.ensemble_seed)
    d_members, _ = braking_ensemble(4, vp, inp.study.FT.ensemble_seed, inp.ensemble_axes)
    ft_members, _ = stages.ft_scenarios(inp, 4)
    clearance = inp.mission.clearance_agl_m
    for f, d in zip(ft_members, d_members, strict=True):
        assert f.isa_offset_k == pytest.approx(d.isa_offset_k)
        assert f.mass_kg == pytest.approx(d.mass_kg)
        assert f.altitude_amsl_m - clearance == pytest.approx(d.altitude_amsl_m)
    assert any(abs(f.isa_offset_k - c.isa_offset_k) > 1e-6
               for f, c in zip(ft_members, c_members, strict=True))


def test_the_shipped_ft_and_d_samples_use_the_same_seed():
    root = Path(__file__).resolve().parents[2] / "configs"
    inp = load_inputs(root, only={"FT"})
    assert inp.study.FT.ensemble_seed == inp.study.D.recommendation.ensemble_seed
