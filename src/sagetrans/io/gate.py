"""The release gate (section 12): when may a result go to the systems engineering package."""

from __future__ import annotations

import re
import xml.etree.ElementTree as ET
from collections.abc import Mapping, Sequence
from dataclasses import dataclass
from pathlib import Path

from sagetrans.analysis.sensitivity import SensitivityResult
from sagetrans.io.record import FullRunRecord

REQUIRED_TESTS = tuple(f"T-{i:02d}" for i in range(1, 16))
# Analyses that use a stochastic search must register under these names in the run record and
# carry their three-seed cost spread; T-12 must hold on the REAL problem, not only on the analytic
# cost the unit test uses.
SEARCH_ANALYSES = ("B2", "C")
T12_SPREAD_MAX = 0.01
# a dominating input may be quoted as released only if measured, or flagged as an assumption
MEASURED = ("measured",)
MIN_SPEARMAN = 0.8
MIN_TOPK_OVERLAP = 2 / 3


@dataclass(frozen=True)
class GateResult:
    passed: bool
    reasons: tuple[str, ...]  # why it failed (empty when passed)
    warnings: tuple[str, ...]  # passed, but these inputs rest on flagged assumptions

    def summary(self) -> str:
        head = "RELEASE: PASS" if self.passed else "RELEASE: BLOCKED"
        lines = [head, *(f"  blocked: {r}" for r in self.reasons)]
        lines += [f"  note: {w}" for w in self.warnings]
        return "\n".join(lines)


def verification_from_junit(path: Path) -> dict[str, bool]:
    """Map T-xx verification tests to pass/fail from a pytest JUnit XML file.

    A test counts for T-NN when its name starts with `test_tNN_`. An ID passes only if every
    test carrying it passed; an ID with no test is absent, and the gate treats absence as failure.
    """
    out: dict[str, bool] = {}
    for case in ET.parse(path).getroot().iter("testcase"):
        m = re.match(r"test_t(\d\d)_", case.attrib.get("name", ""))
        if not m:
            continue
        ok = case.find("failure") is None and case.find("error") is None
        tid = f"T-{m.group(1)}"
        out[tid] = out.get(tid, True) and ok
    return out


def release_gate(
    record: FullRunRecord,
    verification: Mapping[str, bool],
    sens: SensitivityResult | None,
    flagged_assumptions: Sequence[str],
    required_tests: Sequence[str] = REQUIRED_TESTS,
    outputs: Sequence[str] | None = None,
    k: int = 3,
) -> GateResult:
    """Check the four conditions: DRAFT status, verification, evidence grade, sensitivity.

    * the run record must not be DRAFT;
    * every required verification test must be present and passing;
    * the evidence-grade summary must be attached (counts present);
    * the global ranking must be stable across the two sample sizes, and every input that
      dominates it must be measured, or flagged as an assumption with its range (an input that
      is still TBD cannot be flagged away).
    """
    reasons: list[str] = []
    warnings: list[str] = []
    if record.label == "DRAFT":
        reasons += [f"run record is DRAFT: {r}" for r in record.reasons] or ["run record is DRAFT"]
    for tid in required_tests:
        if tid not in verification:
            reasons.append(f"{tid}: no verification result")
        elif not verification[tid]:
            reasons.append(f"{tid}: verification failed")
    for name in SEARCH_ANALYSES:
        if name not in record.analyses:
            continue
        spread = record.search_spreads.get(name)
        if spread is None:
            reasons.append(f"T-12 on the real problem: no three-seed spread recorded for '{name}'")
        elif not spread <= T12_SPREAD_MAX:
            reasons.append(
                f"T-12 on the real problem: '{name}' seeds disagree by {spread:.1%} "
                f"(limit {T12_SPREAD_MAX:.0%}); the optimum is not repeatable"
            )
    if sum(record.evidence_counts.values()) == 0:
        reasons.append("evidence-grade summary missing")
    if sens is None:
        reasons.append("no sensitivity analysis attached")
        return GateResult(False, tuple(reasons), tuple(warnings))
    flagged = set(flagged_assumptions)
    for out in outputs or list(sens.stability):
        st = sens.stability[out]
        if not st["spearman"] >= MIN_SPEARMAN or st["topk_overlap"] < MIN_TOPK_OVERLAP:
            reasons.append(
                f"sensitivity ranking of {out} is not stable (Spearman {st['spearman']:.2f}, "
                f"top-{k} overlap {st['topk_overlap']:.2f})"
            )
        for inp in sens.dominating(out, k):
            if inp.status in MEASURED:
                continue
            if inp.status == "TBD":
                reasons.append(f"{out}: dominating input '{inp.name}' is TBD")
            elif inp.name not in flagged:
                reasons.append(f"{out}: dominating input '{inp.name}' ({inp.status}) not flagged")
            else:
                rng = f"{inp.lo:g} to {inp.hi:g}"
                warnings.append(f"{out}: '{inp.name}' is {inp.status} (range {rng})")
    return GateResult(not reasons, tuple(reasons), tuple(sorted(set(warnings))))
