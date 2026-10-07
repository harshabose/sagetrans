import casadi as ca
import numpy as np
import pytest
from hypothesis import given
from hypothesis import strategies as st

from sagetrans import atmosphere as atm


def test_t01_density_sea_level_and_5000m():
    # T-01: 1.225 and about 0.736 kg/m^3 within 0.1 %
    assert atm.density(0.0) == pytest.approx(1.225, rel=1e-3)
    assert atm.density(5000.0) == pytest.approx(0.736, rel=1e-3)


def test_isa_table_values():
    # ISA table: T(5000 m) = 255.65 K, p = 54019 Pa
    assert atm.temperature(5000.0) == pytest.approx(255.65, abs=1e-9)
    assert atm.pressure(5000.0) == pytest.approx(54019.0, rel=1e-3)
    assert atm.speed_of_sound(0.0) == pytest.approx(340.3, rel=1e-3)


def test_offset_shifts_temperature_and_lowers_density():
    assert atm.temperature(3000.0, 10.0) == pytest.approx(atm.temperature(3000.0) + 10.0)
    # hotter air is thinner
    assert atm.density(3000.0, 10.0) < atm.density(3000.0, 0.0)


def test_out_of_range_rejected():
    with pytest.raises(ValueError):
        atm.density(12_000.0)


def test_works_on_casadi_expressions_and_matches_numeric():
    h = ca.MX.sym("h")
    d = ca.MX.sym("d")
    f = ca.Function("rho", [h, d], [atm.density(h, d)])
    for hv, dv in [(0.0, 0.0), (5000.0, 0.0), (2500.0, 15.0), (5000.0, -10.0)]:
        assert float(f(hv, dv)) == pytest.approx(atm.density(hv, dv), rel=1e-12)


def test_works_on_arrays():
    h = np.linspace(0, 5000, 11)
    assert atm.density(h).shape == h.shape


@given(h=st.floats(0, 11_000), d=st.floats(-20, 30))
def test_density_positive_and_decreasing_with_altitude(h, d):
    assert atm.density(h, d) > 0
    if h < 10_000:
        assert atm.density(h + 100.0, d) < atm.density(h, d)
