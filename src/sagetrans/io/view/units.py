"""Column name to (unit, decimals): the viewer's and the workbook's one lookup.

Units come from the column-name suffix the study already uses (`_kJ`, `_m`, `_ms` = metres per
second, `_ms2` = metres per second squared, `_s`, `_deg`, ...). A column with no known suffix has no
unit and is shown to 4 significant figures. Format rules (OUTPUTS spec 3.2): kJ, m and deg to 1
decimal, m/s to 2, everything else to 4 significant figures.
"""

from __future__ import annotations

import math
from typing import Any

# (suffix, unit, decimals): the first match wins, so longer suffixes come first
_SUFFIX: tuple[tuple[str, str, int | None], ...] = (
    ("_ms2", "m/s^2", 2), ("_ms", "m/s", 2), ("_kJ", "kJ", 1), ("_Wh", "Wh", 1), ("_J", "J", 0),
    ("_km", "km", 1), ("_kg", "kg", 2), ("_deg", "deg", 1), ("_rad", "rad", 3), ("_k", "K", 1),
    ("_A", "A", 1), ("_V", "V", 2), ("_s", "s", 2), ("_m", "m", 1), ("_pct", "%", 1),
)  # fmt: skip
# columns whose name has no suffix but whose unit is certain
_EXACT: dict[str, tuple[str, int | None]] = {
    "t": ("s", 2), "x": ("m", 1), "h": ("m", 2), "h_rel": ("m", 2), "V": ("m/s", 2),
    "V_eas": ("m/s", 2), "v_h": ("m/s", 2), "theta": ("deg", 1), "theta_cap": ("deg", 1),
    "theta_cmd": ("deg", 1), "alpha_w_deg": ("deg", 1), "q": ("m/s^2", 2), "soc": ("", 2),
}  # fmt: skip


def unit_of(column: str) -> tuple[str, int | None]:
    """The unit and the number of decimals of a column ("", None when unknown)."""
    if column in _EXACT:
        return _EXACT[column]
    for suf, unit, dec in _SUFFIX:
        if column.endswith(suf):
            return unit, dec
    return "", None


def header_label(column: str) -> str:
    """The column header with its unit: `energy [kJ]`. The CSV header itself is never changed."""
    unit, _ = unit_of(column)
    return f"{column} [{unit}]" if unit else column


def fmt_value(value: Any, column: str = "") -> str:
    """A cell as text: numbers by the column's rule, bools yes/no, None empty."""
    if value is None:
        return ""
    if isinstance(value, bool):
        return "yes" if value else "no"
    if isinstance(value, int | float):
        v = float(value)
        if math.isnan(v):
            return "n/a"
        _, dec = unit_of(column)
        if dec is not None:
            return f"{v:.{dec}f}"
        if v == 0.0:
            return "0"
        return f"{v:.4g}" if abs(v) < 1e5 else f"{v:.0f}"
    return str(value)


def excel_format(column: str) -> str:
    """The workbook number format of a column (kJ, m, deg 0.0; others General)."""
    unit, _ = unit_of(column)
    return "0.0" if unit in ("kJ", "m", "deg") else "General"
