"""One definition of the wing-angle margin: study.yaml margin_alpha_frac."""

import math
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs
from sagetrans.io.settings import B1VariantCfg, StudyCfg

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"


@pytest.mark.parametrize("bad", [0.0, -0.1, 1.01, 2.0])
def test_margin_alpha_frac_is_refused_out_of_range(bad):
    with pytest.raises(ValidationError, match="margin_alpha_frac"):
        StudyCfg(margin_alpha_frac=bad)


@pytest.mark.parametrize("ok", [0.05, 0.7, 1.0])
def test_margin_alpha_frac_accepts_zero_exclusive_to_one_inclusive(ok):
    assert StudyCfg(margin_alpha_frac=ok).margin_alpha_frac == ok


def test_the_shipped_value_is_07_with_its_note():
    text = (ROOT / "configs" / "study.yaml").read_text(encoding="utf-8")
    assert yaml.safe_load(text)["margin_alpha_frac"] == 0.7
    line = next(ln for ln in text.splitlines() if ln.startswith("margin_alpha_frac"))
    assert "ASSUMED" in line and "PROPOSED" in line
    assert "alpha_s is a guess (no lift peak in the CFD up to 15 deg)" in line
    assert "13.43 deg" in line


def test_a_margin_variant_takes_its_limit_from_the_one_setting():
    b1 = {"variants": [
        {"name": "attached", "alpha_margin": 0.85},
        {"name": "margin", "margin": True, "theta_cmd_max_deg": None},
    ], "band_sweep": {"variant": "attached"}}
    s = StudyCfg.model_validate({"margin_alpha_frac": 0.6, "B1": b1})
    by = {v.name: v for v in s.B1.variants}
    assert by["margin"].alpha_margin == 0.6 and by["margin"].margin is True
    assert by["attached"].alpha_margin == 0.85  # `attached` keeps its own definition
    s2 = StudyCfg.model_validate({"margin_alpha_frac": 0.8, "B1": b1})
    assert {v.name: v for v in s2.B1.variants}["margin"].alpha_margin == 0.8


def test_a_variant_has_exactly_one_limit():
    for kw in ({}, {"alpha_margin": 0.8, "margin": True}, {"alpha_max_deg": 30.0, "margin": True},
               {"alpha_margin": 0.8, "alpha_max_deg": 30.0}):
        with pytest.raises(ValidationError, match="exactly one"):
            B1VariantCfg(name="x", **kw)
    assert B1VariantCfg(name="m", margin=True).alpha_margin is None


def test_the_margin_numbers_are_computed_from_the_wing():
    inp = load_inputs(ROOT / "configs", only={"C"})
    m = stages.margin_limit(inp)
    wing = inp.vehicle.wing
    assert m["limit_deg"] == pytest.approx(0.7 * math.degrees(wing.alpha_s))
    assert m["angle_margin_to_peak_deg"] == pytest.approx(
        m["alpha_w_clmax_deg"] - m["limit_deg"])
    assert 0.0 < m["cl_at_limit_over_max"] < 1.0
    assert m["alpha_w_clmax_deg"] < math.degrees(wing.alpha_s)
