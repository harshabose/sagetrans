"""Scenario ensembles (section 9) and the ensemble cost of (10.1)."""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np
from scipy.stats import qmc

from sagetrans.analysis.common import Scenario
from sagetrans.physics.params import VehicleParams


@dataclass(frozen=True)
class EnsembleAxes:
    """Ranges of the ensemble axes (placeholders to be set from the mission set, O-14)."""

    altitude_m: tuple[float, float] = (0.0, 5000.0)
    isa_offset_k: tuple[float, float] = (-10.0, 20.0)
    mass_factor: tuple[float, float] = (0.85, 1.15)  # times the vehicle mass
    headwind_ms: tuple[float, float] = (-5.0, 10.0)
    soc: tuple[float, float] = (0.2, 1.0)


DEFAULT_AXES = EnsembleAxes()


def latin_hypercube(
    n: int, vp: VehicleParams, seed: int = 0, axes: EnsembleAxes = DEFAULT_AXES
) -> list[Scenario]:
    """n scenarios, one stratum per sample on every axis; deterministic for a given seed."""
    rng = np.random.default_rng(seed)
    u = [(rng.permutation(n) + rng.random(n)) / n for _ in range(5)]
    ranges = [axes.altitude_m, axes.isa_offset_k, axes.mass_factor, axes.headwind_ms, axes.soc]
    v = [lo + ui * (hi - lo) for ui, (lo, hi) in zip(u, ranges, strict=True)]
    return [
        Scenario(
            float(v[0][i]), float(v[1][i]), float(v[4][i]), float(v[3][i]),
            mass_kg=float(v[2][i] * vp.m),
        )  # fmt: skip
        for i in range(n)
    ]


def braking_ensemble(
    n: int, vp: VehicleParams, seed: int, axes: EnsembleAxes = DEFAULT_AXES, corners: bool = True
) -> tuple[list[Scenario], list[Scenario]]:
    """Analysis D's ensemble: a scrambled Sobol sample and, separately, the corner scenarios.

    The Sobol sample covers the five axes (`n` must be a power of two, which keeps its balance
    properties); the percentile is read from it alone. The 8 corners are the vertices of altitude x
    mass x headwind, with the ISA offset and the state of charge at their midpoints. Entry speed is
    not an axis: the caller derives it per scenario from the airspeed exit rule.
    """
    if n < 1 or n & (n - 1):
        raise ValueError(f"n must be a power of two, got {n}")
    ranges = [axes.altitude_m, axes.isa_offset_k, axes.mass_factor, axes.headwind_ms, axes.soc]
    u = qmc.Sobol(5, scramble=True, seed=seed).random(n)

    def scen(v: list[float]) -> Scenario:
        return Scenario(v[0], v[1], v[4], v[3], mass_kg=v[2] * vp.m)

    core = [
        scen([float(lo + ui * (hi - lo)) for ui, (lo, hi) in zip(row, ranges, strict=True)])
        for row in u
    ]
    if not corners:
        return core, []
    mid = [0.5 * (lo + hi) for lo, hi in ranges]
    corner = [
        scen([alt, mid[1], mf, wind, mid[4]])
        for alt in axes.altitude_m
        for mf in axes.mass_factor
        for wind in axes.headwind_ms
    ]
    return core, corner


def bootstrap_percentile(
    values: np.ndarray | list[float],
    q: float,
    n_boot: int = 1000,
    seed: int = 0,
    ci: tuple[float, float] = (5.0, 95.0),
) -> tuple[float, float, float]:
    """The q-th percentile of `values` and a bootstrap interval for it: (estimate, low, high)."""
    v = np.asarray(values, dtype=float)
    rng = np.random.default_rng(seed)
    boot = np.percentile(rng.choice(v, size=(n_boot, len(v)), replace=True), q, axis=1)
    lo, hi = np.percentile(boot, list(ci))
    return float(np.percentile(v, q)), float(lo), float(hi)


@dataclass(frozen=True)
class CostWeights:
    """Weights and normalisers of (10.1); the weights are a policy choice."""

    w_h: float = 1.0
    w_E: float = 0.3
    w_x: float = 0.5
    h_ref: float = 10.0  # m
    E_ref: float = 5.0e3  # J
    x_ref: float = 150.0  # m
    penalty: float = 10.0  # P_k per failure event


@dataclass(frozen=True)
class RunMetrics:
    excursion: float
    energy: float
    distance: float
    failures: tuple[str, ...] = ()


def ensemble_cost(runs: list[RunMetrics], w: CostWeights) -> float:
    """J = w_h p95(dh)/h_ref + w_E mean(E)/E_ref + w_x p95(x)/x_ref + sum_k P_k n_k."""
    exc = np.array([r.excursion for r in runs])
    en = np.array([r.energy for r in runs])
    dist = np.array([r.distance for r in runs])
    n_fail = sum(len(r.failures) for r in runs)
    return float(
        w.w_h * np.percentile(exc, 95) / w.h_ref
        + w.w_E * en.mean() / w.E_ref
        + w.w_x * np.percentile(dist, 95) / w.x_ref
        + w.penalty * n_fail
    )
