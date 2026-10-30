"""One definition of the wing-angle margin: study.yaml margin_alpha_frac."""

from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from sagetrans.io.settings import StudyCfg

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
