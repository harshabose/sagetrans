"""Chart polish: labels that do not overlap, y axes that follow the data, whole-number ticks."""

import math
import re

import pytest

from sagetrans.io.view import charts as ch


def _texts(svg: str, cls: str) -> list[tuple[float, float, str, str]]:
    pat = rf'<text class="{cls}" x="([-0-9.]+)" y="([-0-9.]+)" text-anchor="(\w+)">([^<]*)</text>'
    return [(float(x), float(y), a, t) for x, y, a, t in re.findall(pat, svg)]


def _boxes(svg: str) -> list[tuple[float, float, float, float, str]]:
    out = []
    for x, y, anchor, lab in _texts(svg, "dl"):
        w = 6.4 * len(lab)
        x0 = x - w if anchor == "end" else x
        out.append((x0, y - 10.0, x0 + w, y + 3.0, lab))
    return out


def _overlap(a: tuple[float, ...], b: tuple[float, ...]) -> bool:
    return min(a[2], b[2]) > max(a[0], b[0]) and min(a[3], b[3]) > max(a[1], b[1])


def test_the_labels_of_a_crowded_curve_do_not_overlap_each_other_or_the_optimum():
    """The C energy chart: a ramp of q labelled at every point, three points bunched together."""
    qs = [1.0, 1.5, 2.0, 2.25, 2.5, 2.75, 3.0]
    dist = [250.0, 160.0, 120.0, 101.0, 100.0, 99.0, 98.0]
    en = [39.0, 23.0, 19.0, 17.4, 17.2, 17.1, 17.0]
    ramp = ch.Series(
        "ramp", "s1", list(zip(dist, en, strict=True)), line=True,
        labels={i: f"q = {q:g}" for i, q in enumerate(qs)},
        label_pos={i: (-9.0, 17.0) for i in range(len(qs))},
    )
    opt = ch.Series("optimum", "s1", [(dist[2], en[2])], line=False, marker="dia",
                    labels={0: "C optimum"}, label_pos={0: (12.0, -10.0)})
    dflt = ch.Series("default", "ref", [(dist[3], en[3] + 0.3)], line=False, marker="sq",
                     labels={0: "ArduPilot default"}, label_pos={0: (12.0, 20.0)})
    svg = ch.xy_chart([ramp, dflt, opt], "distance", "energy", "t")
    boxes = _boxes(svg)
    assert len(boxes) == len(qs) + 2  # every point is labelled
    for i in range(len(boxes)):
        for j in range(i + 1, len(boxes)):
            assert not _overlap(boxes[i], boxes[j]), (boxes[i][4], boxes[j][4])


def test_a_label_that_had_to_move_far_gets_a_leader_line():
    pts = [(10.0, 5.0)] * 12  # twelve labels on one spot: the near places are used up
    s = ch.Series("p", "s1", pts, line=False, labels={i: f"label {i}" for i in range(12)})
    svg = ch.xy_chart([s], "x", "y", "t")
    assert svg.count('class="rule"') >= 1


def test_the_lower_bound_is_the_smaller_of_zero_and_the_data_with_padding():
    assert ch.limits([0.4, 24.0], floor=0.0)[0] == 0.0
    lo, hi = ch.limits([-2.0, 5.0], floor=0.0)
    assert lo < -2.0 and hi > 5.0  # below zero: padded, like any other axis
    assert ch.limits([0.4, 24.0])[0] < 0.4  # no floor: unchanged behaviour


def test_an_axis_from_zero_does_not_start_at_minus_ten_and_keeps_the_tolerance_visible():
    s = ch.Series("worst overshoot", "s1", [(1.0, 0.0), (2.0, 0.6), (3.0, 24.0)])
    svg = ch.xy_chart([s], "q", "overshoot", "t", y_from_zero=True, hlines=[(2.0, "tolerance 2")])
    ys = [float(t) for t in (t for _, _, a, t in _texts(svg, "tick") if a == "end")]
    assert min(ys) == 0.0
    assert "tolerance 2" in svg


def test_a_count_axis_has_whole_numbers_only():
    assert ch.int_ticks(0.0, 1.0) == [0.0, 1.0]
    assert ch.int_ticks(0.0, 0.75) == [0.0, 1.0]
    assert ch.int_ticks(0.0, 4.0, 2) == [0.0, 2.0, 4.0]
    items = [("a", 0.0, "", "ref"), ("b", 1.0, "", "s1")]
    svg = ch.dot_chart(items, "failing members", "c", integer=True)
    shown = [t for _, _, _, t in _texts(svg, "tick")]
    assert shown and all(float(t) == math.floor(float(t)) for t in shown)
    assert not any(t in shown for t in ("0.25", "0.5", "0.75"))
    assert "-1" not in shown  # a count does not go below zero
    zero_only = ch.dot_chart([("a", 0.0, "", "ref")], "failing members", "c", integer=True)
    assert [t for _, _, _, t in _texts(zero_only, "tick")][0] == "0"
    plain = ch.dot_chart(items, "x", "c")
    assert any("." in t for _, _, _, t in _texts(plain, "tick"))  # the default is unchanged


@pytest.mark.parametrize("lo,hi", [(0.0, 3.0), (0.0, 12.0), (2.0, 9.0)])
def test_int_ticks_cover_the_range(lo, hi):
    t = ch.int_ticks(lo, hi)
    assert t[0] <= lo and t[-1] >= hi and all(v == int(v) for v in t)


def test_the_flat_tail_of_the_q_curve_is_explained_from_the_rows():
    from sagetrans.io.view import chapters as chp

    def row(q, dist, sink):
        return {"grid": "coarse", "q": q, "p95_distance_m": dist, "max_end_sink_ms": sink}

    curve = [row(1.0, 248.0, 0.0), row(2.0, 125.0, 1.06), row(2.5, 108.1, 1.13),
             row(3.0, 108.1, 1.13)]
    runs = [{"case": "optimum", "a_cap_ms2": 1.86}, {"case": "optimum", "a_cap_ms2": 2.31},
            {"case": "ardupilot_default", "a_cap_ms2": 9.0}]
    text = chp.flat_tail_statement(curve, runs)
    assert "From q = 2.5" in text and "1.13 m/s" in text
    assert "1.86" in text and "2.31" in text and "9.00" not in text
    assert chp.flat_tail_statement(curve[:3], runs) == ""  # no repeat: nothing to explain
