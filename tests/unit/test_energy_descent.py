import math

import numpy as np
import pytest

from sagetrans import atmosphere
from sagetrans.analysis import descent, energy
from sagetrans.analysis.common import Scenario
from sagetrans.analysis.descent import DescentLimits, descent_profile
from sagetrans.analysis.energy import (
    BatteryWindow,
    BudgetResult,
    MissionProfile,
    SegmentResult,
)
from sagetrans.dynamics import trim
from sagetrans.physics import rotor
from sagetrans.physics.params import placeholder_vehicle
from sagetrans.physics.vehicle import CasadiVehicle

G = atmosphere.G0
VP = placeholder_vehicle()
SC = Scenario(0.0)
WH = 3600.0


# ---- hand-calculated reference case (gate) ------------------------------------------------------
def _hand_budget() -> BudgetResult:
    seg = {
        "takeoff": SegmentResult(20 * WH, 25, 0, "hand"),
        "fwd_transition": SegmentResult(15 * WH, 10, 1000, "hand"),
        "climb": SegmentResult(40 * WH, 400, 3000, "hand"),
        "descent": SegmentResult(5 * WH, 700, 3000, "hand"),
        "back_transition": SegmentResult(15 * WH, 12, 1000, "hand"),
        "landing": SegmentResult(25 * WH, 20, 0, "hand"),
    }
    usable = 0.8 * 2160.0 * WH  # 1728 Wh
    return BudgetResult.from_segments(seg, 12.0 * WH / 1000.0, 80 * WH, usable)


def test_usable_energy_matches_the_nominal_pack():
    assert VP.battery.e_nominal_wh == pytest.approx(2160.0)
    assert BatteryWindow().usable_J(VP) / WH == pytest.approx(1728.0)


def test_hand_calculated_reference_case():
    b = _hand_budget()
    # fixed segments: 20+15+40+5+15+25 = 120 Wh over 8 km; reserve 80 Wh; cruise 12 Wh/km
    assert b.fixed_J / WH == pytest.approx(120.0)
    assert b.fixed_distance_m == pytest.approx(8000.0)
    assert b.max_range_m / 1e3 == pytest.approx(8.0 + (1728.0 - 120.0 - 80.0) / 12.0)  # 135.33 km
    assert b.total_J(100e3) / WH == pytest.approx(120 + 80 + 12 * 92)  # 1304 Wh
    assert b.margin_J(100e3) / WH == pytest.approx(424.0)
    assert b.margin_J(113e3) / WH == pytest.approx(1728 - (200 + 12 * 105))  # 268 Wh
    rows = b.table([100e3, 113e3])
    assert [r["fits"] for r in rows] == [True, True]
    assert rows[0]["margin_pct"] == pytest.approx(100 * 424 / 1728)
    assert b.conflicts((100e3, 113e3, 140e3)) == {100e3: False, 113e3: False, 140e3: True}
    with pytest.raises(ValueError):
        b.total_J(5000.0)


# ---- electrical model and trims agree with the simulated vehicle ------------------------------
def test_hover_state_matches_the_vehicle_model():
    rho = SC.rho
    es = trim.hover_state(VP, rho, 0.8)
    veh = CasadiVehicle(VP)
    x = np.array([0, 0, 0, 0, 0, 0, trim.hover_omega(VP, rho), 0, 0.0])
    u = np.array([es.delta_r_cmd, 0.0, 0.0])
    xd, aux = veh.derivatives(x, u, np.array([0.0, 0.0, 1.2, 0.8]))
    assert aux["V_bus"] == pytest.approx(es.v_bus, rel=1e-9)
    assert aux["I_bus"] == pytest.approx(es.i_bus, rel=1e-9)
    assert xd[6] == pytest.approx(0.0, abs=1e-6)  # rotor speed steady
    assert xd[3] == pytest.approx(0.0, abs=1e-6)  # weight carried


def test_hover_power_exceeds_ideal_and_grows_in_thin_air():
    rho = SC.rho
    p = trim.hover_power(VP, rho, 0.8)
    ideal = VP.rotor.n_r * rotor.ideal_hover_power(
        VP.m * G / VP.rotor.n_r, rho, VP.rotor.A_tot / VP.rotor.n_r
    )
    assert p > ideal
    assert trim.hover_power(VP, float(atmosphere.density(5000.0)), 0.8) > p


def test_esc_bus_power_is_at_least_motor_power():
    es = trim.hover_state(VP, SC.rho, 0.8)
    mr = VP.motor_r
    w = trim.hover_omega(VP, SC.rho)
    p_motor = VP.rotor.n_r * (mr.Rm * es.i_rotor + mr.Ke * w) * es.i_rotor
    assert es.power >= p_motor
    assert es.power - VP.battery.I_av * es.v_bus >= p_motor - 1e-6  # only pack loss in between


def test_cruise_trim_balances_forces_in_the_vehicle_model():
    h, v = 1000.0, 25.0
    rho = float(atmosphere.density(h))
    tr = trim.steady_trim(VP, rho, v, 0.0, 0.8)
    veh = CasadiVehicle(VP)
    x = np.array([0, h, v, 0, tr.theta, 0, 0, tr.omega_p, 0.0])
    u = np.array([0.0, tr.elec.delta_p_cmd, tr.theta])
    xd, aux = veh.derivatives(x, u, np.array([0.0, 0.0, 1.2, 0.8]))
    w = VP.m * G
    assert VP.m * xd[2] == pytest.approx(0.0, abs=1e-6 * w)
    assert VP.m * xd[3] == pytest.approx(0.0, abs=1e-6 * w)
    assert aux["I_bus"] == pytest.approx(tr.elec.i_bus, rel=1e-6)
    assert aux["V_bus"] == pytest.approx(tr.elec.v_bus, rel=1e-6)


def test_climb_trim_and_energy_exceed_potential_energy():
    mp = MissionProfile(cruise_amsl_m=1500.0)
    seg = energy.climb_segment(VP, mp, 0.6)
    dh = mp.cruise_amsl_m - mp.clearance_amsl
    pe = VP.m * G * dh
    assert seg.energy_J > pe
    assert 0.2 < pe / seg.energy_J < 1.0  # overall climb efficiency is sane
    assert seg.time_s == pytest.approx(dh / mp.roc)


def test_glide_trim_obeys_the_pusher_off_balance():
    rho = SC.rho
    g = trim.glide_trim(VP, rho, 16.0, 0.6)
    w = VP.m * G
    assert g.D == pytest.approx(-w * math.sin(g.gamma), rel=1e-9)
    assert g.L == pytest.approx(w * math.cos(g.gamma), rel=1e-9)
    assert g.gamma < 0.0 and g.T_p == 0.0


def test_best_range_speed_minimises_energy_per_metre():
    rho = SC.rho
    v = energy.best_range_speed(VP, rho, 0.6)
    f = energy.cruise_sweep(VP, rho, 0.6, np.array([v - 3.0, v, v + 3.0]))["J_per_m"]
    assert f[1] <= f[0] and f[1] <= f[2]


def test_reserve_scales_with_go_around_time_and_includes_diversion():
    base = MissionProfile()
    r1 = energy.reserve_energy(VP, base, 0.6, 2.0, 10.0, 5.0)
    longer = MissionProfile(t_go_around=2 * base.t_go_around)
    r2 = energy.reserve_energy(VP, longer, 0.6, 2.0, 10.0, 5.0)
    assert r2["go_around"] == pytest.approx(2 * r1["go_around"])
    assert r1["diversion_cruise"] == pytest.approx(2.0 * base.d_diversion_m)
    twice = MissionProfile(n_diversion_transitions=2)
    assert energy.reserve_energy(VP, twice, 0.6, 2.0, 10.0, 5.0)[
        "diversion_transitions"
    ] == pytest.approx(30.0)
    assert r1["total"] == pytest.approx(sum(v for k, v in r1.items() if k != "total"))


def test_build_budget_end_to_end_is_consistent_and_draft():
    mp = MissionProfile()
    b = energy.build_budget(
        VP, mp, BatteryWindow(), DescentLimits(kappa=0.25, kappa_source="test value")
    )
    assert set(b.segments) == {
        "takeoff", "fwd_transition", "climb", "descent", "back_transition", "landing",
    }  # fmt: skip
    assert all(s.energy_J > 0 and s.time_s > 0 for s in b.segments.values())
    assert b.record.label == "DRAFT"
    r = 60e3
    row = b.table([r])[0]
    skip = ("total_Wh", "usable_Wh", "margin_Wh")
    parts = sum(v for k, v in row.items() if k.endswith("_Wh") and k not in skip)
    assert parts == pytest.approx(row["total_Wh"])
    assert b.table([b.max_range_m])[0]["margin_Wh"] == pytest.approx(0.0, abs=1e-6)
    chk = energy.hover_check(VP, mp, BatteryWindow())
    assert chk["ok"] and chk["v_bus_eod_V"] < trim.hover_state(VP, SC.rho, 0.8).v_bus


# ---- analysis F --------------------------------------------------------------------------------
def test_f_requires_a_sourced_kappa():
    with pytest.raises(ValueError, match="sourced"):
        descent_profile(VP, SC, 30.0, DescentLimits(kappa=None))


def test_f_induced_velocity_and_binding_limit():
    sc = Scenario(0.0)
    v_h = math.sqrt(VP.m * G / (2 * sc.rho * VP.rotor.A_tot))
    p = descent_profile(VP, sc, 30.0, DescentLimits(kappa=0.2, v_fragility=9.0, kappa_source="t"))
    assert p.v_h == pytest.approx(v_h) and p.binding == "vortex_ring_state"
    assert p.v_cap == pytest.approx(0.2 * v_h)
    p = descent_profile(VP, sc, 30.0, DescentLimits(kappa=5.0, v_fragility=1.5, kappa_source="t"))
    assert p.binding == "payload_fragility" and p.v_cap == 1.5
    p = descent_profile(
        VP, sc, 30.0, DescentLimits(kappa=5.0, v_fragility=3.0, v_sensor=1.0, kappa_source="t")
    )
    assert p.binding == "sensor"
    assert float(descent.vrs_ratio_at(p)[0]) == pytest.approx(-1.0 / p.v_h)


def test_f_time_and_energy_against_hand_calculation():
    lim = DescentLimits(kappa=5.0, v_fragility=2.0, h_slow=5.0, v_final=0.5, kappa_source="t")
    p = descent_profile(VP, SC, 30.0, lim, n=3000)
    t_hand = (30.0 - 5.0) / 2.0 + 5.0 * math.log(2.0 / 0.5) / (2.0 - 0.5)
    assert p.time == pytest.approx(t_hand, rel=1e-4)
    assert p.energy_J == pytest.approx(trim.hover_power(VP, SC.rho, SC.soc) * t_hand, rel=1e-4)
    assert p.t_to_go[-1] == 0.0 and p.v[-1] == pytest.approx(0.5)
    assert np.all(np.diff(p.t_to_go) <= 0.0)


def test_f_thin_air_allows_a_faster_vrs_limit_but_costs_hover_energy():
    lim = DescentLimits(kappa=0.2, v_fragility=9.0, kappa_source="t")
    lo = descent_profile(VP, Scenario(0.0), 30.0, lim)
    hi = descent_profile(VP, Scenario(5000.0), 30.0, lim)
    assert hi.v_cap > lo.v_cap  # v_h grows as density falls
    assert hi.hover_power > lo.hover_power
