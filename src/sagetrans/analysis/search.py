"""CMA-ES search on the unit cube, three seeds, cached by parameter vector (FR-06).

The cost of (10.1) has jumps from the failure penalties, so a gradient-free search is used.
Evaluations run serially (CasADi functions are not picklable across processes).
"""

from __future__ import annotations

from collections.abc import Callable
from dataclasses import dataclass

import cma
import numpy as np


@dataclass(frozen=True)
class SeedResult:
    seed: int
    x: np.ndarray  # unit-cube coordinates of the best point found
    f: float
    n_evals: int


@dataclass(frozen=True)
class SearchResult:
    seeds: list[SeedResult]
    best_x: np.ndarray
    best_f: float
    f_start: float

    @property
    def spread(self) -> float:
        """(max - min) of the per-seed best costs, relative to the best (T-12)."""
        f = [s.f for s in self.seeds]
        return float((max(f) - min(f)) / max(abs(self.best_f), 1e-12))


@dataclass(frozen=True)
class Bounds:
    names: tuple[str, ...]
    lo: tuple[float, ...]
    hi: tuple[float, ...]

    def to_physical(self, u: np.ndarray) -> dict[str, float]:
        lo, hi = np.array(self.lo), np.array(self.hi)
        vals = lo + np.clip(u, 0.0, 1.0) * (hi - lo)
        return dict(zip(self.names, vals.tolist(), strict=True))

    def to_unit(self, values: dict[str, float]) -> np.ndarray:
        lo, hi = np.array(self.lo), np.array(self.hi)
        v = np.array([values[n] for n in self.names])
        return np.asarray(np.clip((v - lo) / (hi - lo), 0.0, 1.0))


def cma_minimise(
    fn: Callable[[np.ndarray], float],
    u0: np.ndarray,
    seeds: tuple[int, ...] = (1, 2, 3),
    sigma0: float = 0.25,
    popsize: int = 8,
    maxiter: int = 30,
    tolfun: float = 1e-6,
) -> SearchResult:
    """Minimise fn over [0, 1]^n starting at u0; results are cached per rounded vector."""
    cache: dict[tuple[float, ...], float] = {}

    def cached(u: np.ndarray) -> float:
        key = tuple(np.round(np.clip(u, 0.0, 1.0), 9).tolist())
        if key not in cache:
            cache[key] = float(fn(np.clip(u, 0.0, 1.0)))
        return cache[key]

    f_start = cached(u0)
    out: list[SeedResult] = []
    for seed in seeds:
        opts = {
            "seed": seed, "popsize": popsize, "maxiter": maxiter, "bounds": [0.0, 1.0],
            "verbose": -9, "tolfun": tolfun, "tolx": 1e-6,
        }  # fmt: skip
        es = cma.CMAEvolutionStrategy(list(u0), sigma0, opts)
        best_u, best_f, n = np.array(u0), f_start, 0
        while not es.stop():
            xs = es.ask()
            fs = [cached(np.array(x)) for x in xs]
            n += len(xs)
            es.tell(xs, fs)
            i = int(np.argmin(fs))
            if fs[i] < best_f:
                best_f, best_u = fs[i], np.clip(np.array(xs[i]), 0.0, 1.0)
        out.append(SeedResult(seed, best_u, best_f, n))
    best = min(out, key=lambda s: s.f)
    return SearchResult(out, best.x, best.f, f_start)
