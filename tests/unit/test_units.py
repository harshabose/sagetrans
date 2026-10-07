import math

import pytest
from hypothesis import given
from hypothesis import strategies as st

from sagetrans.config import Config, UnitError, to_si


@pytest.mark.parametrize(
    ("value", "unit", "si_value", "si_unit"),
    [
        (5000, "ms", 5.0, "s"),
        (100, "km", 100_000.0, "m"),
        (25, "Ah", 90_000.0, "C"),
        (2.16, "kWh", 2.16 * 3.6e6, "J"),
        (90, "km/h", 25.0, "m/s"),
        (10, "ft/s", 3.048, "m/s"),
        (180, "deg", math.pi, "rad"),
        (3000, "rpm", 100 * math.pi, "rad/s"),
        (1.225, "kg/m^3", 1.225, "kg/m^3"),
        (5, "cm^2", 5e-4, "m^2"),
        (25, "degC", 298.15, "K"),
        (80, "%", 0.8, "-"),
        (3, "N*m", 3.0, "N*m"),
        (7, "-", 7.0, "-"),
    ],
)
def test_conversions(value, unit, si_value, si_unit):
    # T-13: a value in non-SI units is converted exactly
    conv = to_si(value, unit)
    assert conv.value == pytest.approx(si_value, rel=1e-12)
    assert conv.unit == si_unit


def test_unknown_unit_rejected():
    with pytest.raises(UnitError):
        to_si(1.0, "furlong")


@given(st.floats(-1e6, 1e6, allow_nan=False))
def test_si_input_is_unchanged(x):
    assert to_si(x, "m/s").value == x


def test_t13_converted_exactly_once_through_config():
    # T-13: the record stores the authored value and a single derived SI value; reading it
    # back any number of times never converts again.
    cfg = Config.from_dict(
        {"a": {"t": {"value": 5000, "unit": "ms", "source": "x", "status": "documented"}}}
    )
    rec = cfg.record("a.t")
    assert rec.value == 5000 and rec.unit == "ms"
    assert cfg.get("a.t") == 5.0 == cfg.get("a.t")
