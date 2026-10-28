"""Settings and packaging that were cleaned up: unused FT keys, the one supported Python."""

import tomllib
from pathlib import Path

import pytest
import yaml
from pydantic import ValidationError

from sagetrans.io.settings import FTCfg

ROOT = Path(__file__).resolve().parents[2]


@pytest.mark.parametrize("key", ["seeds", "maxiter"])
def test_the_unused_ft_search_settings_are_gone_and_refused(key):
    assert key not in FTCfg.model_fields  # the grid and the scan carry their own
    with pytest.raises(ValidationError, match=key):
        FTCfg(**{key: [1] if key == "seeds" else 5})
    study = yaml.safe_load((ROOT / "configs" / "study.yaml").read_text(encoding="utf-8"))
    assert key not in study["FT"] and key in study["FT"]["grid"] and key in study["FT"]["scan"]


def test_python_311_is_dropped_and_the_pins_are_the_ones_ci_runs():
    proj = tomllib.loads((ROOT / "pyproject.toml").read_text(encoding="utf-8"))
    assert proj["project"]["requires-python"] == ">=3.13"
    assert proj["tool"]["mypy"]["python_version"] == "3.13"
    assert not any("python_version" in d for d in proj["project"]["dependencies"])  # no markers
    ci = yaml.safe_load((ROOT / ".github" / "workflows" / "ci.yml").read_text(encoding="utf-8"))
    assert ci["jobs"]["test"]["strategy"]["matrix"]["python-version"] == ["3.13"]


def test_the_two_limits_of_e_are_stated_in_its_docstring_and_readme():
    from sagetrans.analysis import energy

    doc = energy.__doc__ or ""
    readme = (ROOT / "src" / "sagetrans" / "analysis" / "README.md").read_text(encoding="utf-8")
    for text in (doc, readme):
        assert "worst-case acceleration" in text and "41.7 s" in text and "FT" in text
        assert "fade_wing_share" in text and "not an independent" in text
        assert "dissipated by drag" in text and "0.08 km" in text  # the glide-to-exit one
