"""F: one chart for the descent rate and one for the energy to go, a line per scenario."""

from sagetrans.io.view.chapters import F_CLASSES, descent_figures


def tables(names):
    desc = [{"scenario": n, "altitude_m": 1000.0 * i} for i, n in enumerate(names)]
    prof = [{"scenario": n, "h_m": 30.0 - 3 * k, "descent_rate_ms": 1.5 + 0.1 * i,
             "time_to_go_s": 20.0 - 2 * k, "energy_to_go_J": 4000.0 * (20.0 - 2 * k)}
            for i, n in enumerate(names) for k in range(10)]
    return desc, prof


def test_two_charts_one_line_per_scenario_with_a_legend():
    desc, prof = tables(["S-01", "S-02", "S-03"])
    figs = descent_figures(desc, prof)
    assert len(figs) == 2
    assert "Descent rate against time to go" in figs[0]
    assert "Energy to go against time to go" in figs[1]
    for f in figs:
        assert 'class="mult' not in f  # no small multiples any more
        assert f.count('<path class="ln') == 3
        for lab in ("S-01, 0 m", "S-02, 1000 m", "S-03, 2000 m"):
            assert lab in f  # the legend names the scenario with its altitude
    assert figs[1].count("<i class=\"sw") == 3  # one legend entry per scenario


def test_each_point_has_a_tooltip_naming_the_scenario_with_values_and_units():
    desc, prof = tables(["S-01", "S-02"])
    rate, energy = descent_figures(desc, prof)
    assert "S-02, 1000 m; h_m = 30" in rate and "descent_rate_ms = 1.6" in rate
    assert "energy_to_go_J = " in energy and "time_to_go_s = " in energy
    # a marker on every third sample and the last one: 0, 3, 6, 9 of ten
    assert rate.count('data-tip="S-01, 0 m') == 4


def test_the_energy_is_drawn_in_kilojoules():
    desc, prof = tables(["S-01"])
    _, energy = descent_figures(desc, prof)
    assert "energy to go [kJ]" in energy and "80" in energy  # 4000 J/s x 20 s = 80 kJ


def test_a_sixth_scenario_repeats_the_colours_dashed():
    names = [f"S-0{i}" for i in range(1, 7)]
    desc, prof = tables(names)
    rate, _ = descent_figures(desc, prof)
    assert len(F_CLASSES) == 5
    assert 'class="ln s1 dash"' in rate and rate.count('<i class="sw') == 6
