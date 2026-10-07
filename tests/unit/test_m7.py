import dataclasses
import json
import subprocess
import sys
from pathlib import Path

import numpy as np
import pytest

from sagetrans.analysis import sensitivity as S  # noqa: N812
from sagetrans.analysis.common import RunRecord, Scenario
from sagetrans.config.schema import Config, DraftConfigError
from sagetrans.io import figures, gate, report, results
from sagetrans.io.record import build_record, check_outputs
from sagetrans.physics.params import placeholder_vehicle

ROOT = Path(__file__).resolve().parents[2]
VP = placeholder_vehicle()
SC = Scenario(100.0, soc=0.8)


class Toy:
    """An analytic stand-in with the HeadlineModel interface."""

    vp = None
    scen = None
    outputs = ("y",)

    def __init__(self, inputs, fn):
        self.inputs, self._fn = inputs, fn

    def __call__(self, values):
        return {"y": self._fn({i.name: values.get(i.name, i.nominal) for i in self.inputs})}


def _inp(name, status="assumed", lo=0.0, hi=1.0):
    return S.UncertainInput(name, "vehicle", name, 0.5 * (lo + hi), lo, hi, status)


LINEAR = [_inp("a"), _inp("b"), _inp("c"), _inp("d")]


def linear(v):
    return 5.0 * v["a"] + 2.0 * v["b"] + 0.5 * v["c"] + 0.0 * v["d"]


# ---- sensitivity: machinery against known answers --------------------------------------------
def test_morris_recovers_the_known_ranking_of_a_linear_model():
    res = S.global_morris(Toy(LINEAR, linear), r=8, seed=1)
    mu = dict(zip(res.names, res.index["y"], strict=True))
    assert mu["a"] == pytest.approx(5.0, rel=1e-6) and mu["b"] == pytest.approx(2.0, rel=1e-6)
    assert mu["c"] == pytest.approx(0.5, rel=1e-6) and mu["d"] == pytest.approx(0.0, abs=1e-9)
    assert S.dominating(res, "y", k=2, min_share=1.1) == ["a", "b"]


def test_sobol_total_order_matches_the_analytic_variance_shares():
    ins = [_inp("a"), _inp("b")]
    res = S.global_sobol(Toy(ins, lambda v: v["a"] + 2.0 * v["b"]), n=512, seed=3)
    st = dict(zip(res.names, res.index["y"], strict=True))
    assert st["a"] == pytest.approx(0.2, abs=0.05) and st["b"] == pytest.approx(0.8, abs=0.05)
    assert sum(res.detail["y"]["S1"]) == pytest.approx(1.0, abs=0.1)  # additive model


def test_rank_stability_is_perfect_for_the_same_ranking_and_poor_for_a_reversed_one():
    toy = Toy(LINEAR, linear)
    a, b = S.global_morris(toy, 4, 1), S.global_morris(toy, 8, 2)
    st = S.rank_stability(a, b, k=3)["y"]
    assert st["spearman"] == pytest.approx(1.0) and st["topk_overlap"] == 1.0
    rev = dataclasses.replace(b, index={"y": b.index["y"].max() - b.index["y"]})
    assert S.rank_stability(a, rev, k=3)["y"]["spearman"] < 0.0


def test_oat_rows_match_direct_evaluation_and_the_tornado_is_sorted():
    toy = Toy(LINEAR, linear)
    rows = S.oat(toy)
    r = {x.input: x for x in rows}
    assert r["a"].lo == linear({"a": 0.0, "b": 0.5, "c": 0.5, "d": 0.5})
    assert r["a"].hi == linear({"a": 1.0, "b": 0.5, "c": 0.5, "d": 0.5})
    assert r["a"].hi_half - r["a"].lo_half == pytest.approx(0.5 * (r["a"].hi - r["a"].lo))
    assert [x.input for x in S.tornado(rows, "y")] == ["a", "b", "c", "d"]
    assert S.weakest_status(["measured", "assumed", "stated"]) == "assumed"
    assert S.weakest_status(["measured", "datasheet"]) == "datasheet"


# ---- sensitivity: the real model -----------------------------------------------------------------
def test_apply_inputs_edits_vehicle_scenario_and_controller():
    ins = S.default_inputs(VP, SC)
    vp2, sc2, ctl = S.apply_inputs(
        VP, SC, ins,
        {"mass": 9.0, "CL_alpha": 4.0, "motor_Ke": 0.036, "headwind": 4.0, "alt_hold_Kvh": 0.03},
    )  # fmt: skip
    assert vp2.m == 9.0 and vp2.wing.CL_alpha == 4.0
    assert vp2.motor_r.Ke == vp2.motor_r.Kt == 0.036
    assert sc2.headwind_ms == 4.0 and ctl == {"K_vh": 0.03, "K_i": 0.01}
    assert VP.m == 8.0  # the originals are untouched


def test_headline_model_responds_in_the_physical_direction():
    ins = S.default_inputs(VP, SC)
    model = S.HeadlineModel(VP, SC, ins)
    base = model({})
    assert all(np.isfinite(v) for v in base.values())
    heavy = model({"mass": 9.6})
    assert heavy["energy_J"] > base["energy_J"]  # more weight to hold up
    assert model({"headwind": 5.0})["distance_m"] < base["distance_m"]  # lower ground speed


def test_fr08_global_ranking_is_stable_across_two_sample_sizes_on_the_real_model():
    names = ("mass", "wing_area", "CL_alpha", "CD0", "headwind", "rotor_CT0", "att_zeta")
    ins = [i for i in S.default_inputs(VP, SC) if i.name in names]
    res = S.run_sensitivity(S.HeadlineModel(VP, SC, ins), r_small=5, r_large=10, seed=0)
    for out, st in res.stability.items():
        assert st["spearman"] >= gate.MIN_SPEARMAN, (out, st)
        assert st["topk_overlap"] >= gate.MIN_TOPK_OVERLAP, (out, st)
    assert res.large.n_runs == 10 * (len(ins) + 1)
    assert res.record.label == "DRAFT"


# ---- run record and results folder ---------------------------------------------------------------
def _draft_record():
    cfg = Config.from_yaml(ROOT / "configs" / "vehicle.yaml")
    ana = {"B": RunRecord("0" * 64, "0.0.1", "2026-10-07"), "E": RunRecord("1" * 64, "0.0.1", "d")}
    return cfg, build_record(cfg, ana, seeds=(1, 2, 3))


def _complete_config():
    return Config.from_dict(
        {"mass": {"m_kg": {"value": 8.0, "unit": "kg", "source": "scale", "status": "measured"}}}
    )


def test_record_is_draft_for_tbd_config_and_placeholder_analyses():
    cfg, rec = _draft_record()
    # TBD parameters, files the register points at that do not exist, and two placeholder analyses
    assert rec.label == "DRAFT" and len(rec.reasons) == 4
    assert any("TBD parameters" in r and "mass.m_kg" in r for r in rec.reasons)
    assert any("do not exist" in r and "rotors.ct_map" in r for r in rec.reasons)
    assert rec.config_hash == cfg.config_hash() and rec.seeds == (1, 2, 3)
    assert sum(rec.evidence_counts.values()) == len(list(cfg))
    assert rec.git_commit and rec.date and rec.code_version
    assert json.loads(rec.to_json())["label"] == "DRAFT"


def test_record_is_released_only_when_config_and_analyses_are_release_grade():
    cfg = _complete_config()
    ok = {"B": RunRecord("0" * 64, "0.0.1", "d", label="RELEASED")}
    assert build_record(cfg, ok).label == "RELEASED"
    assert build_record(cfg, {"B": RunRecord("0" * 64, "0.0.1", "d")}).label == "DRAFT"


def test_t14_results_refused_when_draft_unless_overridden_and_every_file_is_hashed(tmp_path):
    cfg, rec = _draft_record()
    tables = {"budget": [{"range_km": 100, "margin_Wh": 400.0}, {"range_km": 113, "margin_Wh": 3}]}
    texts = {"note.md": "hello\n"}
    with pytest.raises(DraftConfigError, match="DRAFT"):
        results.write_results(tmp_path, "run1", rec, cfg, tables=tables, texts=texts)
    assert not (tmp_path / "run1").exists()  # refusal writes nothing
    folder = results.write_results(
        tmp_path, "run1", rec, cfg, tables=tables, texts=texts, override=True
    )
    data = json.loads((folder / "run_record.json").read_text(encoding="utf-8"))
    assert data["override"] is True and set(data["outputs"]) == {"budget.csv", "note.md"}
    assert all(check_outputs(folder).values())
    (folder / "budget.csv").write_text("tampered\n", encoding="utf-8")  # edited by hand
    assert check_outputs(folder) == {"budget.csv": False, "note.md": True}


def test_outputs_are_reproducible_byte_for_byte_and_figures_carry_the_footer(tmp_path):
    cfg, rec = _draft_record()
    rows = S.oat(Toy(LINEAR, linear))
    hashes = []
    for k in range(2):
        folder = results.write_results(
            tmp_path, f"r{k}", rec, cfg, tables={"oat": [dataclasses.asdict(r) for r in rows]},
            figures={"tornado": figures.tornado_figure(rows, "y")}, override=True,
        )  # fmt: skip
        hashes.append(json.loads((folder / "run_record.json").read_text())["outputs"])
    assert hashes[0] == hashes[1]
    assert set(hashes[0]) == {"oat.csv", "tornado.png", "tornado.svg"}
    assert rec.footer() in (tmp_path / "r0" / "tornado.svg").read_text(encoding="utf-8")


# ---- the release gate ----------------------------------------------------------------------------
def _sens(statuses, stable=True):
    ins = [_inp(n, st) for n, st in zip("abcd", statuses, strict=True)]
    res = S.run_sensitivity(Toy(ins, linear), r_small=4, r_large=8, seed=1)
    if not stable:
        res.stability["y"] = {"spearman": 0.2, "topk_overlap": 0.33}
    return res


def _released_record():
    return build_record(
        _complete_config(), {"B": RunRecord("0" * 64, "0.0.1", "d", label="RELEASED")}
    )


ALL_PASS = {t: True for t in gate.REQUIRED_TESTS}


def test_gate_passes_when_everything_dominating_is_measured_and_tests_pass():
    g = gate.release_gate(_released_record(), ALL_PASS, _sens(["measured"] * 4), [])
    assert g.passed and not g.reasons and not g.warnings


def test_gate_blocks_a_draft_record_and_names_why():
    _, rec = _draft_record()
    g = gate.release_gate(rec, ALL_PASS, _sens(["measured"] * 4), [])
    assert not g.passed and any("DRAFT" in r and "TBD" in r for r in g.reasons)


def test_gate_blocks_missing_or_failed_verification_tests():
    sens = _sens(["measured"] * 4)
    missing = {k: v for k, v in ALL_PASS.items() if k != "T-09"}
    assert any("T-09: no verification" in r for r in
               gate.release_gate(_released_record(), missing, sens, []).reasons)  # fmt: skip
    failed = {**ALL_PASS, "T-11": False}
    assert any("T-11: verification failed" in r for r in
               gate.release_gate(_released_record(), failed, sens, []).reasons)  # fmt: skip


def test_gate_requires_dominating_assumptions_to_be_flagged_and_never_accepts_tbd():
    rec = _released_record()
    sens = _sens(["assumed", "assumed", "measured", "measured"])
    unflagged = gate.release_gate(rec, ALL_PASS, sens, [])
    assert not unflagged.passed
    assert any("'a'" in r and "not flagged" in r for r in unflagged.reasons)
    flagged = gate.release_gate(rec, ALL_PASS, sens, ["a", "b"])
    assert flagged.passed and any("range 0 to 1" in w for w in flagged.warnings)
    tbd_sens = _sens(["TBD", "measured", "measured", "measured"])
    tbd = gate.release_gate(rec, ALL_PASS, tbd_sens, ["a"])
    assert not tbd.passed and any("'a' is TBD" in r for r in tbd.reasons)


def test_gate_blocks_an_unstable_sensitivity_ranking():
    g = gate.release_gate(_released_record(), ALL_PASS, _sens(["measured"] * 4, stable=False), [])
    assert not g.passed and any("not stable" in r for r in g.reasons)
    no_sens = gate.release_gate(_released_record(), ALL_PASS, None, [])
    assert no_sens.reasons[-1].startswith("no sens")


def test_verification_ids_are_read_from_junit_and_the_real_suite_names_resolve(tmp_path):
    xml = tmp_path / "j.xml"
    xml.write_text(
        '<testsuites><testsuite>'
        '<testcase name="test_t01_a"/><testcase name="test_t07_x"><failure/></testcase>'
        '<testcase name="test_t07_y"/><testcase name="test_other"/>'
        '</testsuite></testsuites>',
        encoding="utf-8",
    )  # fmt: skip
    assert gate.verification_from_junit(xml) == {"T-01": True, "T-07": False}
    real = tmp_path / "real.xml"
    subprocess.run(
        [sys.executable, "-m", "pytest", "-q", "-p", "no:cacheprovider", f"--junitxml={real}",
         str(ROOT / "tests/unit/test_atmosphere.py"), str(ROOT / "tests/unit/test_config.py"),
         str(ROOT / "tests/unit/test_units.py")],
        check=True, capture_output=True, cwd=ROOT,
    )  # fmt: skip
    v = gate.verification_from_junit(real)
    assert v.get("T-01") and v.get("T-13") and v.get("T-14")


# ---- the report ----------------------------------------------------------------------------------
def test_report_has_the_five_sections_the_grade_next_to_each_number_and_the_draft_banner():
    cfg, rec = _draft_record()
    sens = _sens(["assumed", "stated", "measured", "measured"])
    text = report.build_report(
        report.ReportInputs(
            "Back-transition study", rec, cfg, {"y": 12.34}, sens=sens, a_rec=2.5,
            results={"D": "a_eq 2.6 m/s^2"}, limitations=["placeholder physics"],
        )  # fmt: skip
    )
    for h in ("## 1. Summary", "## 2. Evidence grade", "## 3. Results by analysis",
              "## 4. Sensitivity", "## 5. Limitations"):  # fmt: skip
        assert h in text
    assert "**DRAFT.**" in text and rec.footer() in text
    assert "| y | 12.34 " in text and "| assumed |" in text  # weakest of the dominating grades
    assert "Recommended Q_TRANS_DECEL (analysis D): 2.50" in text
    assert "Flagged as assumptions with their range: a, b" in text
    assert "`mass.m_kg`" in text  # assumed or TBD parameters are listed


def test_end_to_end_bundle_on_the_real_model_writes_a_report_and_a_record(tmp_path):
    names = ("mass", "wing_area", "CD0", "headwind")
    ins = [i for i in S.default_inputs(VP, SC) if i.name in names]
    model = S.HeadlineModel(VP, SC, ins)
    sens = S.run_sensitivity(model, r_small=3, r_large=5, seed=0)
    cfg, rec = _draft_record()
    rec = dataclasses.replace(rec, analyses={**rec.analyses, "sensitivity": sens.record.input_hash})
    text = report.build_report(report.ReportInputs("Bundle", rec, cfg, model({}), sens=sens))
    folder = results.write_results(
        tmp_path, "bundle", rec, cfg,
        tables={"oat": [dataclasses.asdict(r) for r in sens.oat]},
        figures={"tornado_excursion": figures.tornado_figure(sens.oat, "excursion_m")},
        texts={"report.md": text}, override=True,
    )  # fmt: skip
    assert all(check_outputs(folder).values()) and (folder / "report.md").is_file()
    assert "excursion_m" in text


def test_failed_runs_are_filled_with_the_worst_value_for_the_outputs_direction():
    class Failing(Toy):
        def __call__(self, values):
            if values.get("a", 0.5) > 0.9:  # part of the space where the model fails
                return {"y": float("nan")}
            return super().__call__(values)

    ins = [_inp("a"), _inp("b")]
    cost = Failing(ins, lambda v: 10.0 + v["a"] + v["b"])  # lower is better (default)
    x = np.array([[0.2, 0.2], [0.95, 0.1], [0.5, 0.5]])
    ys, n_failed = S._evaluate(cost, x)
    assert n_failed == 1 and ys["y"][1] == pytest.approx(max(ys["y"][0], ys["y"][2]))
    gain = Failing(ins, lambda v: 10.0 + v["a"] + v["b"])
    gain.higher_is_better = frozenset({"y"})  # larger is better, so failure = the minimum
    ys, _ = S._evaluate(gain, x)
    assert ys["y"][1] == pytest.approx(min(ys["y"][0], ys["y"][2]))
    assert S.HeadlineModel(VP, SC, S.default_inputs(VP, SC)).higher_is_better == {"max_range_km"}


def _record_with(analyses, spreads=None):
    recs = {n: RunRecord("0" * 64, "0.0.1", "d", label="RELEASED") for n in analyses}
    return build_record(_complete_config(), recs, search_spreads=spreads)


def test_gate_checks_t12_on_the_real_problem_not_only_the_analytic_unit_test():
    sens = _sens(["measured"] * 4)
    # the unit-test T-12 passes (ALL_PASS) but analysis B2 was never shown repeatable
    no_evidence = gate.release_gate(_record_with(["B2"]), ALL_PASS, sens, [])
    assert not no_evidence.passed
    assert any("'B2'" in r and "no three-seed spread" in r for r in no_evidence.reasons)
    # the real result from this project: B2's seeds disagree by 4-9 %
    bad = gate.release_gate(_record_with(["B2"], {"B2": 0.09}), ALL_PASS, sens, [])
    assert not bad.passed and any("'B2'" in r and "9.0%" in r for r in bad.reasons)
    nan = gate.release_gate(_record_with(["B2"], {"B2": float("nan")}), ALL_PASS, sens, [])
    assert not nan.passed
    # analysis C was repeatable (spread 2e-5), so it passes the same check
    ok = gate.release_gate(_record_with(["C"], {"C": 2e-5}), ALL_PASS, sens, [])
    assert ok.passed
    # one repeatable and one not: still blocked, and only the bad one is named
    both = _record_with(["B2", "C"], {"B2": 0.044, "C": 2e-5})
    mixed = gate.release_gate(both, ALL_PASS, sens, [])
    assert not mixed.passed
    assert [r for r in mixed.reasons if "T-12" in r] == [
        r for r in mixed.reasons if "'B2'" in r
    ]
    # analyses that use no search are not asked for a spread
    assert gate.release_gate(_record_with(["E", "D"]), ALL_PASS, sens, []).passed


def test_ci_check_fails_on_a_missing_or_failing_verification_id(tmp_path):
    from sagetrans.io import ci_check

    def junit(ids, failing=()):
        cases = "".join(
            f'<testcase name="test_t{i:02d}_x">{"<failure/>" if i in failing else ""}</testcase>'
            for i in ids
        )
        p = tmp_path / f"j{len(cases)}{len(failing)}.xml"
        p.write_text(f"<testsuites><testsuite>{cases}</testsuite></testsuites>", encoding="utf-8")
        return p

    ok, lines = ci_check.check(junit(range(1, 16)))
    assert ok and lines[0] == "T-01: pass" and len(lines) == 15
    ok, lines = ci_check.check(junit([i for i in range(1, 16) if i != 12]))
    assert not ok and "T-12: MISSING" in lines
    ok, lines = ci_check.check(junit(range(1, 16), failing=(7,)))
    assert not ok and "T-07: FAILED" in lines
    assert ci_check.main([str(junit(range(1, 16)))]) == 0
    assert ci_check.main([str(junit(range(1, 16), failing=(3,)))]) == 1
    assert ci_check.main([]) == 2


def test_search_spread_travels_in_the_run_record_json():
    rec = _record_with(["C"], {"C": 2e-5})
    assert json.loads(rec.to_json())["search_spreads"] == {"C": 2e-5}
