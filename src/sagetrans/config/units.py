"""Unit conversion at the input boundary (NFR-02, T-13).

Everything inside the toolkit is SI. A parameter record keeps the value and unit exactly as the
author wrote them; `to_si` is called once, when the record is built, and the result is stored.
Nothing downstream converts again.
"""

from __future__ import annotations

import math
import re
from dataclasses import dataclass

# atom -> (factor to SI, SI atoms it maps to as {atom: exponent})
_ATOMS: dict[str, tuple[float, dict[str, int]]] = {
    "-": (1.0, {}),
    "%": (0.01, {}),
    # length
    "m": (1.0, {"m": 1}),
    "km": (1e3, {"m": 1}),
    "cm": (1e-2, {"m": 1}),
    "mm": (1e-3, {"m": 1}),
    "ft": (0.3048, {"m": 1}),
    "nmi": (1852.0, {"m": 1}),
    # time
    "s": (1.0, {"s": 1}),
    "ms": (1e-3, {"s": 1}),
    "min": (60.0, {"s": 1}),
    "h": (3600.0, {"s": 1}),
    "Hz": (1.0, {"s": -1}),
    # mass
    "kg": (1.0, {"kg": 1}),
    "g": (1e-3, {"kg": 1}),
    # force, pressure, energy, power
    "N": (1.0, {"N": 1}),
    "Pa": (1.0, {"Pa": 1}),
    "hPa": (100.0, {"Pa": 1}),
    "J": (1.0, {"J": 1}),
    "Wh": (3600.0, {"J": 1}),
    "kWh": (3.6e6, {"J": 1}),
    "W": (1.0, {"W": 1}),
    "kW": (1e3, {"W": 1}),
    # electrical
    "V": (1.0, {"V": 1}),
    "A": (1.0, {"A": 1}),
    "mA": (1e-3, {"A": 1}),
    "C": (1.0, {"C": 1}),
    "Ah": (3600.0, {"C": 1}),
    "mAh": (3.6, {"C": 1}),
    "ohm": (1.0, {"ohm": 1}),
    "mohm": (1e-3, {"ohm": 1}),
    "F": (1.0, {"F": 1}),
    # angle
    "rad": (1.0, {"rad": 1}),
    "deg": (math.pi / 180.0, {"rad": 1}),
    "rpm": (2.0 * math.pi / 60.0, {"rad": 1, "s": -1}),
    # temperature differences (offsets, e.g. the ISA offset, are differences)
    "K": (1.0, {"K": 1}),
}

# Absolute temperatures with an offset; only valid as a stand-alone unit.
_OFFSET_ATOMS: dict[str, tuple[float, float]] = {"degC": (1.0, 273.15)}

_TERM = re.compile(r"^(?P<atom>[A-Za-z%\-]+?)(?:\^(?P<exp>-?\d+))?$")


class UnitError(ValueError):
    """Unknown or malformed unit string."""


@dataclass(frozen=True)
class SIValue:
    value: float
    unit: str


def _parse(unit: str) -> tuple[float, dict[str, int]]:
    unit = unit.strip()
    if unit == "-":
        return 1.0, {}
    factor = 1.0
    dims: dict[str, int] = {}
    # split on * and /, remembering which side of the first "/" each term is on;
    # every term after a "/" is in the denominator (a/b*c means a/(b*c) is NOT assumed:
    # we follow left-to-right: a/b*c = (a/b)*c, so "*" after "/" returns to the numerator)
    tokens = re.split(r"([*/])", unit.replace(" ", ""))
    sign = 1
    for i, tok in enumerate(tokens):
        if i % 2 == 1:
            sign = 1 if tok == "*" else -1
            continue
        m = _TERM.match(tok)
        if not m or m["atom"] not in _ATOMS:
            raise UnitError(f"unknown unit {tok!r} in {unit!r}")
        exp = int(m["exp"]) if m["exp"] else 1
        f, atoms = _ATOMS[m["atom"]]
        factor *= f ** (sign * exp)
        for a, e in atoms.items():
            dims[a] = dims.get(a, 0) + sign * exp * e
    return factor, {a: e for a, e in dims.items() if e != 0}


def _render(dims: dict[str, int]) -> str:
    if not dims:
        return "-"

    def term(a: str, e: int) -> str:
        return a if abs(e) == 1 else f"{a}^{abs(e)}"

    num = [term(a, e) for a, e in sorted(dims.items()) if e > 0]
    den = [term(a, e) for a, e in sorted(dims.items()) if e < 0]
    top = "*".join(num) if num else "1"
    if not den:
        return top
    return f"{top}/{den[0]}" if len(den) == 1 else f"{top}/({'*'.join(den)})"


def to_si(value: float, unit: str) -> SIValue:
    """Convert `value` given in `unit` to SI. Call once, at the input boundary."""
    unit = unit.strip()
    if unit in _OFFSET_ATOMS:
        factor, offset = _OFFSET_ATOMS[unit]
        return SIValue(value * factor + offset, "K")
    factor, dims = _parse(unit)
    return SIValue(value * factor, _render(dims))
