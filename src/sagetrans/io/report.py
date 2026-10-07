"""The generated report (section 11): headline numbers with the evidence grade next to each."""

from __future__ import annotations

from collections.abc import Mapping, Sequence
from dataclasses import dataclass, field

from sagetrans.analysis.energy import BudgetResult
from sagetrans.analysis.sensitivity import SensitivityResult, UncertainInput, weakest_status
from sagetrans.config.schema import Config
from sagetrans.io.record import FullRunRecord

UNITS = {"excursion_m": "m", "distance_m": "m", "energy_J": "J", "max_range_km": "km"}


@dataclass
class ReportInputs:
    title: str
    record: FullRunRecord
    config: Config
    headline: Mapping[str, float]  # nominal values of the headline outputs
    sens: SensitivityResult | None = None
    budget: BudgetResult | None = None
    a_rec: float | None = None  # Q_TRANS_DECEL recommendation, m/s^2
    required_km: Sequence[float] = (100.0, 113.0)
    results: Mapping[str, str] = field(default_factory=dict)  # section text per analysis
    limitations: Sequence[str] = ()
    files: Sequence[str] = ()


def flagged_assumptions(sens: SensitivityResult, k: int = 3) -> list[str]:
    """Names of the dominating inputs the report states as assumptions with their range."""
    names: list[str] = []
    for out in sens.stability:
        for inp in sens.dominating(out, k):
            if inp.status != "measured" and inp.name not in names:
                names.append(inp.name)
    return names


def _rng(i: UncertainInput) -> str:
    return f"{i.lo:g} to {i.hi:g}"


def build_report(r: ReportInputs, k: int = 3) -> str:
    """Markdown report in the five-part structure of section 11."""
    rec, out = r.record, []
    out += [f"# {r.title}", "", f"`{rec.footer()}`", ""]
    if rec.label == "DRAFT":
        out += [
            "> **DRAFT.** Not for release. " + "; ".join(rec.reasons) + ".",
            "" if not rec.override else "> The DRAFT refusal was explicitly overridden.",
            "",
        ]

    # 1 summary
    out += ["## 1. Summary", "", "| Output | Nominal | Weakest grade among dominating inputs |",
            "| --- | --- | --- |"]  # fmt: skip
    for name, val in r.headline.items():
        grade = "n/a"
        if r.sens is not None and name in r.sens.stability:
            grade = weakest_status([i.status for i in r.sens.dominating(name, k)])
        out.append(f"| {name} | {val:.4g} {UNITS.get(name, '')} | {grade} |")
    out.append("")
    if r.a_rec is not None:
        out += [f"Recommended Q_TRANS_DECEL (analysis D): {r.a_rec:.2f} m/s^2.", ""]
    if r.budget is not None:
        out += ["Range margin (analysis E, reserves included):", ""]
        out += ["| Range | Total [Wh] | Margin [Wh] | Fits |", "| --- | --- | --- | --- |"]
        for row in r.budget.table([x * 1e3 for x in r.required_km]):
            out.append(
                f"| {row['range_km']:.0f} km | {row['total_Wh']:.0f} | {row['margin_Wh']:.0f} "
                f"| {'yes' if row['fits'] else 'NO'} |"
            )
        conflicts = r.budget.conflicts(tuple(x * 1e3 for x in r.required_km))
        bad = [f"{km / 1e3:.0f} km" for km, c in conflicts.items() if c]
        out += ["", f"Maximum range: {r.budget.max_range_m / 1e3:.0f} km."]
        if bad:
            out.append(f"**Range requirement and usable energy conflict at: {', '.join(bad)}.**")
        out.append("")

    # 2 evidence grade
    c = rec.evidence_counts
    out += ["## 2. Evidence grade", "", "Inputs in the parameter register by grade:", "",
            "| Grade | Count |", "| --- | --- |"]  # fmt: skip
    out += [f"| {g} | {n} |" for g, n in c.items()]
    assumed = [p for p in r.config if r.config.record(p).status.value in ("assumed", "TBD")]
    if assumed:
        out += ["", "Assumed or still unknown: " + ", ".join(f"`{p}`" for p in assumed) + "."]
    out.append("")

    # 3 results by analysis
    out += ["## 3. Results by analysis", ""]
    for name, text in r.results.items():
        out += [f"### {name}", "", text, ""]
    if not r.results:
        out += ["No analysis sections supplied.", ""]

    # 4 sensitivity
    out += ["## 4. Sensitivity and what would change the conclusion", ""]
    if r.sens is None:
        out += ["No sensitivity analysis attached.", ""]
    else:
        s = r.sens
        out += [
            f"Global ranking: Morris at {s.small.size} and {s.large.size} trajectories "
            f"({s.small.n_runs} and {s.large.n_runs} runs; {s.large.n_failed} failed runs in the "
            "larger set were filled with the worst finite value).", "",
            "| Output | Spearman | Top-3 overlap | Dominating inputs (grade, range) |",
            "| --- | --- | --- | --- |",
        ]  # fmt: skip
        for o, st in s.stability.items():
            dom = "; ".join(f"{i.name} ({i.status}, {_rng(i)})" for i in s.dominating(o, k))
            out.append(f"| {o} | {st['spearman']:.2f} | {st['topk_overlap']:.2f} | {dom} |")
        out += ["", "Flagged as assumptions with their range: "
                + (", ".join(flagged_assumptions(s, k)) or "none") + ".", ""]  # fmt: skip

    # 5 limitations
    out += ["## 5. Limitations", "",
            "Linked to the uncertainty register (proposal section 13).", ""]  # fmt: skip
    out += [f"- {x}" for x in r.limitations] or ["- None recorded."]
    if r.files:
        out += ["", "## Files", ""] + [f"- `{f}`" for f in r.files]
    return "\n".join(out) + "\n"
