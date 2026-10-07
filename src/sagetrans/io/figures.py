"""Figures with the run record in the footer (section 11). Matplotlib, non-interactive backend."""

from __future__ import annotations

from collections.abc import Sequence
from pathlib import Path

import matplotlib

matplotlib.use("Agg")
matplotlib.rcParams["svg.hashsalt"] = "sagetrans"  # fixed element ids: SVG output is reproducible
import matplotlib.pyplot as plt  # noqa: E402
import numpy as np  # noqa: E402
from matplotlib.figure import Figure  # noqa: E402

from sagetrans.analysis.energy import BudgetResult  # noqa: E402
from sagetrans.analysis.sensitivity import OATRow, tornado  # noqa: E402

GRADE_COLOUR = {
    "measured": "#2a9d8f", "datasheet": "#2a9d8f", "derived": "#8ab17d", "documented": "#8ab17d",
    "stated": "#e9c46a", "assumed": "#e76f51", "TBD": "#9b2226",
}  # fmt: skip


def tornado_figure(rows: Sequence[OATRow], output: str, top: int = 12) -> Figure:
    """One tornado chart: bar from the low to the high end of each input's range, by grade."""
    sel = tornado(rows, output)[:top][::-1]
    fig, ax = plt.subplots(figsize=(7.0, 0.38 * len(sel) + 1.6))
    nominal = sel[0].nominal_out if sel else 0.0
    for i, r in enumerate(sel):
        colour = GRADE_COLOUR.get(r.status, "#888888")
        ax.barh(i, r.hi - r.lo, left=r.lo, color=colour, alpha=0.35, height=0.7)
        ax.barh(i, r.hi_half - r.lo_half, left=r.lo_half, color=colour, height=0.45)
    ax.axvline(nominal, color="black", lw=0.8)
    ax.set_yticks(range(len(sel)), [f"{r.input} [{r.status}]" for r in sel])
    ax.set_xlabel(output)
    ax.set_title(f"One-at-a-time sensitivity of {output} (full and half range)")
    fig.tight_layout()
    return fig


def range_margin_figure(
    budget: BudgetResult, required_km: Sequence[float] = (100.0, 113.0)
) -> Figure:
    """Energy margin against range: where the usable pack energy runs out."""
    lo = budget.fixed_distance_m / 1e3
    ranges = np.linspace(lo, max(budget.max_range_m / 1e3 * 1.1, max(required_km) * 1.1), 120)
    margin = [budget.margin_J(r * 1e3) / 3600.0 for r in ranges]
    fig, ax = plt.subplots(figsize=(7.0, 4.0))
    ax.plot(ranges, margin, color="#264653")
    ax.axhline(0.0, color="black", lw=0.8)
    for r in required_km:
        ax.axvline(r, color="#e76f51", ls="--", lw=1.0)
        ax.annotate(f"{r:.0f} km", (r, ax.get_ylim()[1] * 0.9), rotation=90, va="top", ha="right")
    ax.axvline(budget.max_range_m / 1e3, color="#2a9d8f", lw=1.2)
    ax.set_xlabel("flown distance [km]")
    ax.set_ylabel("usable energy margin [Wh]")
    ax.set_title("Range margin (reserves included)")
    fig.tight_layout()
    return fig


def pareto_figure(points: Sequence[tuple[float, float, float]]) -> Figure:
    """Energy against excursion, coloured by stopping distance: (excursion, energy_J, distance)."""
    fig, ax = plt.subplots(figsize=(6.5, 4.2))
    exc, en, dist = (np.array(c) for c in zip(*points, strict=True))
    sc = ax.scatter(exc, en / 1e3, c=dist, cmap="viridis", s=50)
    fig.colorbar(sc, ax=ax, label="distance [m]")
    ax.set_xlabel("altitude excursion [m]")
    ax.set_ylabel("bus energy [kJ]")
    ax.set_title("Back-transition energy against altitude excursion")
    fig.tight_layout()
    return fig


def feasibility_figure(
    v0_grid: Sequence[float], a_grid: Sequence[float], feasible: np.ndarray
) -> Figure:
    """Zero-loss feasibility over entry speed and deceleration."""
    fig, ax = plt.subplots(figsize=(6.0, 4.2))
    ax.pcolormesh(
        np.array(a_grid), np.array(v0_grid), feasible.astype(float), cmap="RdYlGn",
        shading="nearest", vmin=0.0, vmax=1.0,
    )  # fmt: skip
    ax.set_xlabel("deceleration [m/s^2]")
    ax.set_ylabel("entry speed V0 [m/s]")
    ax.set_title("Zero-altitude-loss feasibility (green = feasible)")
    fig.tight_layout()
    return fig


def save_figure(fig: Figure, folder: Path, stem: str, footer: str) -> list[Path]:
    """Write PNG and SVG with the footer, return the paths."""
    fig.text(0.01, 0.005, footer, fontsize=6.5, color="#555555", ha="left", va="bottom")
    paths = []
    for ext in ("png", "svg"):
        p = folder / f"{stem}.{ext}"
        # no timestamp or software tag in the file, so identical inputs give identical bytes
        fig.savefig(p, dpi=150, metadata={"Date": None} if ext == "svg" else {"Software": None})
        paths.append(p)
    plt.close(fig)
    return paths
