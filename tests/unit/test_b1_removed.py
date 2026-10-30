"""B1 (open-loop optimal control) is removed: nothing in the study mentions or runs it."""

import re
import shutil
from pathlib import Path

import pytest
from pydantic import ValidationError

from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs
from sagetrans.io.settings import ALL_ANALYSES, StudyCfg
from sagetrans.io.view.registry import CHAPTERS

ROOT = Path(__file__).resolve().parents[2]


def test_b1_is_neither_an_analysis_nor_a_block_of_the_study():
    assert "B1" not in ALL_ANALYSES
    with pytest.raises(ValidationError):
        StudyCfg(analyses=["C", "B1"])
    with pytest.raises(ValidationError, match="B1"):
        StudyCfg.model_validate({"B1": {"scenarios": ["S-01"]}})
    for folder in (ROOT / "configs", ROOT / "tests" / "data" / "placeholder_configs"):
        text = (folder / "study.yaml").read_text(encoding="utf-8")
        assert not re.search(r"^B1:", text, re.MULTILINE) and "B1" not in text.split("\n")[14]


def test_no_b1_task_is_planned_and_the_report_has_no_b1_chapter():
    inp = load_inputs(ROOT / "configs")
    assert not [t for t in stages.plan_tasks(inp) if t.startswith("B1")]
    assert "b1" not in [c.id for c in CHAPTERS]
    with pytest.raises(ValueError, match="unknown task"):
        stages.run_task(inp, "B1:S-01")


def test_c_owns_the_scenario_of_its_history_figure(tmp_path):
    inp = load_inputs(ROOT / "configs")
    assert inp.study.C.history_scenario == "S-01" and inp.study.C.history_band_m == 1.0
    cfg = tmp_path / "configs"
    shutil.copytree(ROOT / "configs", cfg)
    study = cfg / "study.yaml"
    study.write_text(study.read_text(encoding="utf-8").replace(
        "history_scenario: S-01", "history_scenario: S-99"), encoding="utf-8")
    with pytest.raises(ValueError, match="S-99"):
        load_inputs(cfg)
