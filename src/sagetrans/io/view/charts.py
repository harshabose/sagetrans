"""Pure functions rows -> SVG string. Own SVG, no library (docs/OUTPUTS_spec.md 1).

Series classes: `s1`, `s2`, `s3` (categorical slots in a fixed order; the colour follows the
entity, and a chart never has more than three), `ink` (an annotation that is not an entity, such as
E's assumed value) and `ref` (grey ink, dashed: a reference series such as the ArduPilot default,
never an entity). More than three entities go into small multiples, not more hues. Lines are 2 px,
markers 5 px with a 2 px surface ring; every mark carries `data-tip` (hover and keyboard focus show
it) and the page carries a legend and a data table, so identity never rests on colour alone.
"""

from __future__ import annotations

import bisect
import html
import json
import math
from collections.abc import Callable, Sequence
from dataclasses import dataclass, field, replace

Point = tuple[float, float]


def esc(s: object) -> str:
    return html.escape(str(s), quote=True)


def f1(v: float) -> str:
    """A coordinate to one decimal."""
    return f"{v:.1f}"


def el(tag: str, cls: str = "", body: str = "", /, **attrs: object) -> str:
    """One SVG element; `data_tip` becomes `data-tip`, `viewbox` becomes `viewBox`."""
    parts = [f' class="{cls}"'] if cls else []
    for k, v in attrs.items():
        if v is not None:
            name = "viewBox" if k == "viewbox" else k.replace("_", "-")
            parts.append(f' {name}="{esc(v)}"')
    head = f"<{tag}{''.join(parts)}"
    return f"{head}>{body}</{tag}>" if (body or tag == "text") else f"{head}/>"


def text(x: float, y: float, s: object, cls: str, anchor: str = "start", **kw: object) -> str:
    return el("text", cls, esc(s), x=f1(x), y=f1(y), text_anchor=anchor, **kw)


def nice_ticks(lo: float, hi: float, n: int = 5) -> list[float]:
    """Round tick values covering [lo, hi] with about `n` intervals."""
    if not (math.isfinite(lo) and math.isfinite(hi)):
        return [0.0, 1.0]
    if hi - lo < 1e-12:
        hi = lo + 1.0
    raw = (hi - lo) / max(n, 1)
    mag = 10 ** math.floor(math.log10(raw))
    step = next(m * mag for m in (1, 2, 2.5, 5, 10) if m * mag >= raw)
    ticks, v = [], math.floor(lo / step) * step
    while v <= hi + step * 1e-9:
        ticks.append(round(v, 12))
        v += step
    return ticks


def int_ticks(lo: float, hi: float, n: int = 5) -> list[float]:
    """Whole-number ticks covering [lo, hi] (a count axis: 0.5 failing members does not exist)."""
    if not (math.isfinite(lo) and math.isfinite(hi)):
        return [0.0, 1.0]
    a, b = math.floor(lo + 1e-9), max(math.ceil(hi - 1e-9), math.floor(lo + 1e-9) + 1)
    step = max(1, math.ceil((b - a) / max(n, 1)))
    ticks = list(range(a, b + 1, step))
    if ticks[-1] < b:
        ticks.append(ticks[-1] + step)
    return [float(v) for v in ticks]


def tick_text(v: float) -> str:
    if abs(v) >= 1e6 or (v != 0 and abs(v) < 0.01):
        return f"{v:.3g}"
    if abs(v) >= 1000:
        return f"{v:.0f}"
    s = f"{v:.4f}".rstrip("0").rstrip(".")
    return "0" if s == "-0" else s


class Frame:
    """A plot rectangle with linear scales."""

    def __init__(
        self, w: float, h: float, xlim: Point, ylim: Point,
        pl: float = 58, pr: float = 14, pt: float = 14, pb: float = 46,
    ) -> None:  # fmt: skip
        self.w, self.h, self.pl, self.pr, self.pt, self.pb = w, h, pl, pr, pt, pb
        self.xlim, self.ylim = xlim, ylim

    def sx(self, v: float) -> float:
        lo, hi = self.xlim
        return self.pl + (v - lo) / ((hi - lo) or 1.0) * (self.w - self.pl - self.pr)

    def sy(self, v: float) -> float:
        lo, hi = self.ylim
        return self.h - self.pb - (v - lo) / ((hi - lo) or 1.0) * (self.h - self.pt - self.pb)

    @property
    def right(self) -> float:
        return self.w - self.pr

    @property
    def bottom(self) -> float:
        return self.h - self.pb


def limits(vals: Sequence[float], pad: float = 0.06, floor: float | None = None) -> Point:
    v = [x for x in vals if math.isfinite(x)]
    if not v:
        return 0.0, 1.0
    lo, hi = min(v), max(v)
    if hi - lo < 1e-12:
        hi = lo + (abs(lo) * 0.1 or 1.0)
    span = hi - lo
    low = lo - span * pad if floor is None or lo < floor else floor
    return low, hi + span * pad


def _fit(fr: Frame, xt: list[float], yt: list[float]) -> None:
    fr.xlim = (min(fr.xlim[0], xt[0]), max(fr.xlim[1], xt[-1]))
    fr.ylim = (min(fr.ylim[0], yt[0]), max(fr.ylim[1], yt[-1]))


def marker(kind: str, cx: float, cy: float, cls: str, tip: str = "", hollow: bool = False) -> str:
    c = f"mk {cls}{' hollow' if hollow else ''}"
    kw: dict[str, object] = {"tabindex": 0, "data_tip": tip} if tip else {}
    if kind == "sq":
        return el("rect", c, x=f1(cx - 4.5), y=f1(cy - 4.5), width=9, height=9, **kw)
    if kind == "dia":
        d = f"M{f1(cx)} {f1(cy - 6.5)}L{f1(cx + 6.5)} {f1(cy)}L{f1(cx)} {f1(cy + 6.5)}"
        return el("path", c, d=d + f"L{f1(cx - 6.5)} {f1(cy)}Z", **kw)
    return el("circle", c, cx=f1(cx), cy=f1(cy), r=5, **kw)


def _path(pts: Sequence[Point], fr: Frame, steps: bool = False) -> str:
    out: list[str] = []
    for i, (x, y) in enumerate(pts):
        sx, sy = fr.sx(x), fr.sy(y)
        if i == 0:
            out.append(f"M{f1(sx)} {f1(sy)}")
        else:
            out.append(f"L{f1(sx)} {f1(fr.sy(pts[i - 1][1]))}" if steps else "")
            out.append(f"L{f1(sx)} {f1(sy)}")
    return "".join(out)


def axes(fr: Frame, xlab: str, ylab: str, xt: list[float], yt: list[float],
         yfmt: Callable[[float], str] | None = None) -> list[str]:  # fmt: skip
    out: list[str] = []
    for t in yt:
        y = fr.sy(t)
        out.append(el("line", "grid", x1=fr.pl, x2=f1(fr.right), y1=f1(y), y2=f1(y)))
        out.append(text(fr.pl - 7, y + 4, (yfmt or tick_text)(t), "tick", "end"))
    for t in xt:
        out.append(text(fr.sx(t), fr.bottom + 17, tick_text(t), "tick", "middle"))
    out.append(el("line", "axis", x1=fr.pl, x2=f1(fr.right), y1=f1(fr.bottom), y2=f1(fr.bottom)))
    out.append(text((fr.pl + fr.right) / 2, fr.h - 6, xlab, "axlab", "middle"))
    if ylab:
        mid = (fr.pt + fr.bottom) / 2
        out.append(el("text", "axlab", esc(ylab), transform=f"translate(13 {f1(mid)}) rotate(-90)",
                      text_anchor="middle"))  # fmt: skip
    return out


@dataclass
class Series:
    """One series of an x-y chart."""

    name: str
    cls: str = "s1"  # s1 | s2 | s3 | ink | ref
    pts: list[Point] = field(default_factory=list)
    tips: list[str] = field(default_factory=list)  # per point
    hollow: list[bool] = field(default_factory=list)  # per point: flagged
    kinds: list[str] = field(default_factory=list)  # per point marker shape (default `marker`)
    line: bool = True
    dash: bool = False
    marker: str = "dot"  # dot | sq | dia | none
    labels: dict[int, str] = field(default_factory=dict)  # point index -> direct label
    # point index -> (dx, dy) pixels of that label from its point (default 8, -8); a label placed
    # to the left (dx < 0) ends at the offset, so crowded points can be nudged apart
    label_pos: dict[int, tuple[float, float]] = field(default_factory=dict)
    thick: bool = False


def xy_chart(
    series: Sequence[Series], xlab: str, ylab: str, aria: str, *, w: float = 720, h: float = 360,
    xlim: Point | None = None, ylim: Point | None = None, y_from_zero: bool = False,
    hlines: Sequence[tuple[float, str]] = (), vlines: Sequence[tuple[float, str]] = (),
    ylog: bool = False,
) -> str:  # fmt: skip
    """A scatter / line chart. `hlines` and `vlines` are labelled reference rules.

    `ylog` puts the y axis on a log scale (positive values only; zero is not on it, the axis starts
    at the decade below the smallest value) with ticks at 1, 2 and 5 times a power of ten."""
    yfmt: Callable[[float], str] | None = None
    if ylog:
        pos = [p[1] for s in series for p in s.pts if p[1] > 0] + [v for v, _ in hlines if v > 0]
        k_lo = math.floor(math.log10(min(pos))) if pos else 0
        k_hi = math.ceil(math.log10(max(pos))) if pos else 1
        k_hi = max(k_hi, k_lo + 1)
        series = [replace(s, pts=[(x, math.log10(y) if y > 0 else math.nan) for x, y in s.pts])
                  for s in series]  # fmt: skip
        hlines = [(math.log10(v), lab) for v, lab in hlines if v > 0]
        ylim = (float(k_lo), float(k_hi))
        mult = (1, 2, 5) if k_hi - k_lo <= 3 else (1,)
        ytl = [k + math.log10(m) for k in range(k_lo, k_hi) for m in mult] + [float(k_hi)]
        yfmt = lambda t: tick_text(10 ** t)  # noqa: E731
    xs = [p[0] for s in series for p in s.pts] + [v for v, _ in vlines]
    ys = [p[1] for s in series for p in s.pts] + [v for v, _ in hlines]
    fr = Frame(w, h, xlim or limits(xs), ylim or limits(ys, floor=0.0 if y_from_zero else None))
    xt, yt = nice_ticks(*fr.xlim), (ytl if ylog else nice_ticks(*fr.ylim))
    _fit(fr, xt, yt)
    out = axes(fr, xlab, ylab, xt, yt, yfmt)
    for v, lab in hlines:
        y = fr.sy(v)
        out.append(el("line", "rule", x1=fr.pl, x2=f1(fr.right), y1=f1(y), y2=f1(y)))
        out.append(text(fr.right - 4, y - 5, lab, "dl", "end"))
    for v, lab in vlines:
        x = fr.sx(v)
        out.append(el("line", "rule", x1=f1(x), x2=f1(x), y1=fr.pt, y2=f1(fr.bottom)))
        out.append(text(x + 4, fr.pt + 12, lab, "dl"))
    for s in series:
        if s.line and len(s.pts) > 1:
            cls = f"ln {s.cls}{' dash' if s.dash else ''}{' thick' if s.thick else ''}"
            out.append(el("path", cls, d=_path(s.pts, fr)))
    for s in series:
        for i, (x, y) in enumerate(s.pts):
            kind = s.kinds[i] if i < len(s.kinds) else s.marker
            if kind != "none" and math.isfinite(y):
                out.append(marker(kind, fr.sx(x), fr.sy(y), s.cls,
                                  s.tips[i] if i < len(s.tips) else "",
                                  bool(s.hollow[i]) if i < len(s.hollow) else False))  # fmt: skip
    # Every marker is an obstacle for a label. The labels are placed one at a time, the series
    # drawn last (the one on top, such as the optimum) first, so it keeps its preferred spot; a
    # label that had to move far from its point gets a leader line.
    placed: list[tuple[float, float, float, float]] = [
        (fr.sx(x) - 7.0, fr.sy(y) - 7.0, fr.sx(x) + 7.0, fr.sy(y) + 7.0)
        for s in series for k, (x, y) in enumerate(s.pts)
        if math.isfinite(y) and (s.kinds[k] if k < len(s.kinds) else s.marker) != "none"
    ]  # fmt: skip
    for s in reversed(series):
        for i, lab in s.labels.items():
            px, py = fr.sx(s.pts[i][0]), fr.sy(s.pts[i][1])
            dx, dy = place_label(px, py, lab, s.label_pos.get(i, (8.0, -8.0)), placed)
            if math.hypot(dx, dy) > LEADER_AFTER_PX:
                out.append(el("line", "rule", x1=f1(px), y1=f1(py), x2=f1(px + dx), y2=f1(py + dy - 4)))
            out.append(text(px + dx, py + dy, lab, "dl", "end" if dx < 0 else "start"))
    return el("svg", "viz", "".join(out), viewbox=f"0 0 {w:g} {h:g}", role="img", aria_label=aria)


LEADER_AFTER_PX = 26.0  # a label further than this from its point is joined to it by a line
LABEL_ALTERNATES = (
    (-9.0, 17.0), (9.0, 17.0), (9.0, -8.0), (-9.0, -8.0), (-9.0, 31.0), (9.0, 31.0), (-9.0, -22.0),
    (9.0, -22.0), (-9.0, 45.0), (9.0, 45.0), (-9.0, -36.0), (9.0, -36.0), (-9.0, 59.0),
    (9.0, 59.0), (-9.0, -50.0), (9.0, -50.0), (-30.0, 73.0), (30.0, 73.0), (-30.0, -64.0),
    (30.0, -64.0), (-50.0, 87.0), (50.0, 87.0), (-50.0, -78.0), (50.0, -78.0),
)


def place_label(px: float, py: float, lab: str, pref: tuple[float, float],
                placed: list[tuple[float, float, float, float]]) -> tuple[float, float]:  # fmt: skip
    """The offset of a direct label from its point: the preferred one, or the first alternative
    whose box (about 6.4 px a character, 13 px high) overlaps no label already placed; the least
    overlapping one when none is free. The box is recorded in `placed`."""
    w = 6.4 * len(lab)

    def box(d: tuple[float, float]) -> tuple[float, float, float, float]:
        x0 = px + d[0] - (w if d[0] < 0 else 0.0)
        return x0, py + d[1] - 10.0, x0 + w, py + d[1] + 3.0

    def clash(b: tuple[float, float, float, float]) -> float:
        return sum(max(0.0, min(b[2], o[2]) - max(b[0], o[0]))
                   * max(0.0, min(b[3], o[3]) - max(b[1], o[1])) for o in placed)  # fmt: skip

    cands = [pref, *[a for a in LABEL_ALTERNATES if a != pref]]
    pick = next((d for d in cands if clash(box(d)) == 0.0), min(cands, key=lambda d: clash(box(d))))
    placed.append(box(pick))
    return pick


def legend(entries: Sequence[tuple[str, str, str]]) -> str:
    """HTML legend of (class, kind, text); kind is line | dash | dot | sq | dia | hollow."""
    spans = [f'<span><i class="sw {c} {k}"></i>{esc(t)}</span>' for c, k, t in entries]
    if not spans:
        return ""
    return f'<div class="legend" role="list" aria-label="Legend">{"".join(spans)}</div>'


def multiples(charts: Sequence[tuple[str, str]], cols: int = 2) -> str:
    """Small multiples: (title, svg) in a grid; the titles carry the identity, not colour."""
    cells = [f'<div class="sm"><h4>{esc(t)}</h4>{svg}</div>' for t, svg in charts]
    return f'<div class="mult c{cols}">{"".join(cells)}</div>'


@dataclass
class Panel:
    key: str
    label: str
    unit: str = ""
    dec: int = 2
    sign: bool = False
    hlines: list[tuple[float, str]] = field(default_factory=list)
    band: tuple[float, float] | None = None  # a shaded band, e.g. the altitude limits
    vticks: list[tuple[float, str]] = field(default_factory=list)  # instants, e.g. mode changes
    ymin: float | None = None  # the axis starts here (airspeed: 0), not at the data minimum
    marks: bool = False  # draw each series' `marks` on its curve in this panel
    wide: bool = False  # the panel spans both columns (long rule labels need the room)


@dataclass
class TSeries:
    """One run on a time-history figure: `vars[key]` is sampled at the times `t`."""

    name: str
    cls: str
    short: str
    t: list[float]
    vars: dict[str, list[float]]
    dash: bool = False
    steps: bool = False
    thick: bool = False
    direct: str = ""  # a direct label at the end of the line (the reference series)
    end_mark: bool = False  # a marker at the last sample: where the run stopped
    # (time, shape, tip) markers on the curve of a panel with `marks`, e.g. where the fade starts
    marks: list[tuple[float, str, str]] = field(default_factory=list)


def stack_labels(ys: Sequence[float], gap: float, lo: float, hi: float) -> list[float]:
    """Pixel rows for labels wanted at `ys`, at least `gap` apart and inside [lo, hi], in the
    order given (a label is moved up or down from its rule, never reordered)."""
    order = sorted(range(len(ys)), key=lambda i: ys[i])
    out = [0.0] * len(ys)
    prev = lo - gap
    for i in order:  # push down from the top
        out[i] = max(ys[i], prev + gap)
        prev = out[i]
    over = out[order[-1]] - hi if order else 0.0
    if over > 0:  # push back up from the bottom
        nxt = hi + gap
        for i in reversed(order):
            out[i] = min(out[i], nxt - gap)
            nxt = out[i]
    return out


def panel_figure(fig_id: str, panels: Sequence[Panel], series: Sequence[TSeries],
                 tlabel: str = "time [s]") -> str:  # fmt: skip
    """Panels with a shared time axis and one crosshair across them (spec 3.3).

    The crosshair script finds the nearest sample of each series, so runs of different length and
    sample spacing line up; a series that has ended says "run ended".
    """
    t1 = max((max(s.t) for s in series if s.t), default=0.0) or 1.0
    h, pl, pr = 200.0, 50.0, 10.0
    data = {"t1": t1, "panels": [{"key": p.key, "dec": p.dec, "sign": p.sign, "unit": p.unit}
                                 for p in panels],
            "series": [{"name": s.name, "cls": s.cls, "short": s.short, "t": s.t, "v": s.vars}
                       for s in series]}  # fmt: skip
    cells: list[str] = []
    first = next((i for i, p in enumerate(panels) if any(s.vars.get(p.key) for s in series)), 0)
    for pi, p in enumerate(panels):
        vals = [v for s in series for v in s.vars.get(p.key, []) if math.isfinite(v)]
        vals += [v for v, _ in p.hlines] + (list(p.band) if p.band else [])
        if not vals:
            continue
        if p.ymin is not None:
            vals.append(p.ymin)
        w = 720.0 if p.wide else 350.0
        fr = Frame(w, h, (0.0, t1), limits(vals), pl=pl, pr=pr, pt=12, pb=34)
        if p.ymin is not None:
            fr.ylim = (p.ymin, fr.ylim[1])
        xt, yt = nice_ticks(0.0, t1, 4), nice_ticks(*fr.ylim, 4)
        _fit(fr, xt, yt)
        o = [text(pl, 9, p.label, "ptitle")]
        if p.band:
            y0, y1 = fr.sy(p.band[1]), fr.sy(p.band[0])
            o.append(el("rect", "band", x=pl, y=f1(y0), width=f1(w - pl - pr), height=f1(y1 - y0)))
        o += axes(fr, tlabel, "", xt, yt)
        # the rule labels sit at the right edge, stacked with a minimum gap so none overprints
        rows = stack_labels([fr.sy(v) - 4 for v, _ in p.hlines], 11.0, 22.0, fr.bottom - 4)
        for (v, lab), ly in zip(p.hlines, rows, strict=True):
            y = fr.sy(v)
            o.append(el("line", "rule", x1=pl, x2=f1(w - pr), y1=f1(y), y2=f1(y)))
            o.append(text(w - pr - 3, ly, lab, "dl", "end"))
        for tv, lab in p.vticks:
            x = f1(fr.sx(tv))
            o.append(el("line", "rule", x1=x, x2=x, y1=12, y2=f1(fr.bottom)))
            o.append(text(fr.sx(tv) + 3, fr.bottom - 5, lab, "dl"))
        for s in series:
            ys = s.vars.get(p.key)
            if ys and len(ys) == len(s.t):
                cls = f"ln {s.cls}{' dash' if s.dash else ''}{' thick' if s.thick else ''}"
                o.append(el("path", cls, d=_path(list(zip(s.t, ys, strict=True)), fr, s.steps)))
                if s.direct and pi == first:
                    o.append(text(fr.sx(s.t[-1]) - 4, fr.sy(ys[-1]) + 16, s.direct, "dl", "end"))
                if p.marks:
                    for mt, shape, mtip in s.marks:
                        k = nearest(s.t, mt)
                        if k is not None and math.isfinite(ys[k]):
                            o.append(marker(shape, fr.sx(s.t[k]), fr.sy(ys[k]), s.cls, mtip))
                if s.end_mark and math.isfinite(ys[-1]):
                    o.append(marker("dot", fr.sx(s.t[-1]), fr.sy(ys[-1]), s.cls,
                                    f"{s.short}: stops at {s.t[-1]:.2f} s ({p.label} {ys[-1]:.2f})"))
        o.append(el("line", "xh", x1=0, x2=0, y1=12, y2=f1(fr.bottom), visibility="hidden"))
        o.append(el("rect", "hit", x=pl, y=12, width=f1(w - pl - pr), height=f1(fr.bottom - 12),
                    fill="transparent"))  # fmt: skip
        svg = el("svg", "viz panel", "".join(o), viewbox=f"0 0 {w:g} {h:g}", role="img",
                 aria_label=f"{p.label} against time", data_fig=fig_id, data_pi=pi, data_pl=pl,
                 data_pr=pr, data_w=w)  # fmt: skip
        cells.append(f'<div class="pan{" wide" if p.wide else ""}">{svg}</div>')
    if not cells:
        return ""
    blob = json.dumps(data, separators=(",", ":")).replace("</", "<\\/")
    return (
        f'<script type="application/json" data-hist="{fig_id}">{blob}</script>'
        f'<div class="pans">{"".join(cells)}</div>'
    )


def nearest(ts: Sequence[float], t: float) -> int | None:
    """Index of the sample nearest to `t`, None if `t` is past the end of the series."""
    if not ts or t > ts[-1] + (ts[-1] - ts[0]) / max(len(ts) - 1, 1):
        return None
    i = bisect.bisect_left(ts, t)
    if i > 0 and (i == len(ts) or t - ts[i - 1] <= ts[i] - t):
        i -= 1
    return min(i, len(ts) - 1)


Strip = tuple[str, Sequence[tuple[float, str, bool, str]], float | None]
# optional second layer of a strip: hollow grey dots (value, tip) just below the main line, e.g. the
# members of the ArduPilot default, and a labelled vertical rule (value, label), e.g. a tolerance
StripExtra = tuple[Sequence[tuple[float, str]], tuple[float, str] | None]


def strip_chart(rows: Sequence[Strip], aria: str, w: float = 720,
                extras: Sequence[StripExtra] | None = None) -> str:  # fmt: skip
    """One strip per metric on its own scale: a dot per member, a grey tick for a reference.

    Each row is (label, [(value, tip, hollow, cls)], reference value or None). `extras[i]` adds, to
    strip i, a hollow grey dot per member of a second case and a vertical rule; the axis includes
    both.
    """
    pl = max(150.0, 6.6 * max((len(r[0]) for r in rows), default=0) + 24.0)
    pr = 20.0
    out: list[str] = []
    ytop = 0.0
    for i, (label, pts, ref) in enumerate(rows):
        x_dots, x_rule = extras[i] if extras and i < len(extras) else ((), None)
        vals = ([p[0] for p in pts if math.isfinite(p[0])] + ([ref] if ref is not None else [])
                + [v for v, _ in x_dots if math.isfinite(v)]
                + ([x_rule[0]] if x_rule is not None else []))  # fmt: skip
        if not vals:
            continue
        lo, hi = limits(vals, 0.04)
        fr = Frame(w, 64.0, (lo, hi), (0, 1), pl=pl, pr=pr, pt=0, pb=0)
        marks = ([(v, tip, hol, cls) for v, tip, hol, cls in pts if math.isfinite(v)]
                 + [(v, tip, True, "ref") for v, tip in x_dots if math.isfinite(v)])  # fmt: skip
        offs = swarm([fr.sx(m[0]) for m in marks], reach=6)
        reach_px = max((abs(d) for d in offs), default=0.0)
        rh = max(46.0, 2 * reach_px + 36.0)  # the row is as tall as its swarm
        y0 = ytop + rh / 2
        ytop += rh
        out.append(text(pl - 12, y0 + 4, label, "axlab", "end"))
        out.append(el("line", "grid", x1=pl, x2=f1(w - pr), y1=f1(y0), y2=f1(y0)))
        out.append(text(pl, y0 + reach_px + 20, f"{lo:.3g}", "tick"))
        out.append(text(w - pr, y0 + reach_px + 20, f"{hi:.3g}", "tick", "end"))
        if ref is not None and math.isfinite(ref):
            x = f1(fr.sx(ref))
            out.append(el("line", "refmk", x1=x, x2=x, y1=f1(y0 - 10), y2=f1(y0 + 10)))
        if x_rule is not None and math.isfinite(x_rule[0]):
            x = f1(fr.sx(x_rule[0]))
            out.append(el("line", "rule", x1=x, x2=x, y1=f1(y0 - 15), y2=f1(y0 + 15)))
            right = fr.sx(x_rule[0]) > pl + 0.65 * (w - pl - pr)  # a label that would leave the page
            out.append(text(fr.sx(x_rule[0]) + (-3 if right else 3), y0 - 8, x_rule[1], "dl",
                            "end" if right else "start"))
        for (v, tip, hol, cls), dy in zip(marks, offs, strict=True):
            out.append(marker("dot", fr.sx(v), y0 + dy, cls, tip, hol))
    return el("svg", "viz", "".join(out), viewbox=f"0 0 {w:g} {ytop + 28:g}", role="img",
              aria_label=aria)  # fmt: skip


def swarm(xs: Sequence[float], gap: float = 10.0, step: float = 10.0, reach: int = 3) -> list[float]:
    """Vertical offsets [px] that keep marks at the pixel columns `xs` from overlapping: each mark,
    in the order given, takes the first of 0, +step, -step, +2 step, ... that is at least `gap`
    from every mark already placed; when none is free, the offset farthest from its neighbours."""
    placed: list[tuple[float, float]] = []
    out: list[float] = []
    for x in xs:
        cands = [0.0] + [s * k * step for k in range(1, reach + 1) for s in (1.0, -1.0)]

        def room(d: float, x: float = x) -> float:
            return min((math.hypot(x - px, d - py) for px, py in placed), default=math.inf)

        pick = next((d for d in cands if room(d) >= gap), max(cands, key=room))
        placed.append((x, pick))
        out.append(pick)
    return out


def symlog(v: float, lin: float) -> float:
    """A symmetric log: linear within `lin` of zero, logarithmic beyond (so one large value does not
    flatten the small ones)."""
    return math.copysign(math.log10(1.0 + abs(v) / lin), v)


def dot_chart(items: Sequence[tuple[str, float, str, str]], xlab: str, aria: str, w: float = 720,
              zero: bool = True, refs: Sequence[tuple[float, str] | None] | None = None,
              sym: float | None = None, integer: bool = False) -> str:  # fmt: skip
    """A dot per row against one axis, with a zero rule: (label, value, tip, cls).

    `refs[i]` is an optional reference for row i, (value, tip), drawn as a hollow grey diamond on
    the row's line (e.g. the nominal value the row's setting departs from); the axis includes it.
    `sym` puts the axis on a symmetric log scale, linear within `sym` of zero, with ticks at 0 and
    at +-`sym` times a power of ten, labelled in the data's units. `integer` puts whole-number
    ticks only (a count).
    """
    rh, pr, pt, pb = 26.0, 24.0, 12.0, 44.0
    pl = max(210.0, 6.4 * max((len(i[0]) for i in items), default=0) + 24.0)
    h = pt + pb + rh * len(items)

    def tf(v: float) -> float:
        return v if sym is None else symlog(v, sym)

    vals = [tf(v) for _, v, _, _ in items if math.isfinite(v)] + ([0.0] if zero else [])
    vals += [tf(r[0]) for r in (refs or []) if r is not None and math.isfinite(r[0])]
    xlim = limits(vals, 0.08)
    if integer:  # a count: from zero (or the least value) to at least one, no padding below zero
        finite = [v for v in vals if math.isfinite(v)] or [0.0]
        xlim = (min(0.0, min(finite)), max(1.0, max(finite)))
    fr = Frame(w, h, xlim, (0, 1), pl=pl, pr=pr, pt=pt, pb=pb)
    lab_of: Callable[[float], str] = tick_text
    if sym is None:
        xt = int_ticks(*fr.xlim) if integer else nice_ticks(*fr.xlim)
        _fit(fr, xt, [0, 1])
    else:
        top = max(abs(fr.xlim[0]), abs(fr.xlim[1]))
        real = [0.0] + [s * sym * 10 ** k for k in range(0, 12) for s in (1.0, -1.0)]
        xt = sorted({tf(v) for v in real if abs(tf(v)) <= top + 1e-9})
        back = {round(t, 9): v for t, v in ((tf(v), v) for v in real)}
        lab_of = lambda t: f"{back.get(round(t, 9), 0.0):.2g}"  # noqa: E731
    out: list[str] = []
    shown: list[float] = []  # the labels kept: zero first, then outwards, none within 36 px of another
    for t in sorted(xt, key=abs):
        if all(abs(fr.sx(t) - fr.sx(s)) >= 36.0 for s in shown):
            shown.append(t)
    for t in xt:
        x = f1(fr.sx(t))
        out.append(el("line", "grid", x1=x, x2=x, y1=pt, y2=f1(fr.bottom)))
        if t in shown:
            out.append(text(fr.sx(t), fr.bottom + 16, lab_of(t), "tick", "middle"))
    if zero:
        x0 = f1(fr.sx(0.0))
        out.append(el("line", "zero", x1=x0, x2=x0, y1=pt, y2=f1(fr.bottom)))
    out.append(text((pl + w - pr) / 2, h - 6, xlab, "axlab", "middle"))
    for i, (label, v, tip, cls) in enumerate(items):
        y = pt + rh * i + rh / 2
        out.append(text(pl - 10, y + 4, label, "dl", "end"))
        ref = refs[i] if refs and i < len(refs) else None
        if ref is not None and math.isfinite(ref[0]):
            out.append(marker("dia", fr.sx(tf(ref[0])), y, "ref", ref[1], True))
        if math.isfinite(v):
            out.append(marker("dot", fr.sx(tf(v)), y, cls, tip))
    return el("svg", "viz", "".join(out), viewbox=f"0 0 {w:g} {h:g}", role="img", aria_label=aria)


def bar_chart(items: Sequence[tuple[str, float, str, str]], xlab: str, aria: str, w: float = 720,
              waterfall: bool = False, hatched: Sequence[int] = ()) -> str:  # fmt: skip
    """Horizontal bars (label, value, tip, cls); a waterfall chains each bar to the last.

    The bars at the indices in `hatched` are drawn hatched, a reference that is not a result."""
    rh, pl, pr, pt, pb = 26.0, 190.0, 70.0, 8.0, 40.0
    h = pt + pb + rh * len(items)
    spans, run = [], 0.0
    for _, v, _, _ in items:
        spans.append((run, run + v) if waterfall else (0.0, v))
        run += v if waterfall else 0.0
    lo = min([s for s, _ in spans] + [0.0])
    hi = max([e for _, e in spans] + [lo + 1e-9])
    xt = nice_ticks(lo, hi)
    fr = Frame(w, h, (lo, max(hi, xt[-1])), (0, 1), pl=pl, pr=pr, pt=pt, pb=pb)
    out: list[str] = []
    for t in xt:
        x = f1(fr.sx(t))
        out.append(el("line", "grid", x1=x, x2=x, y1=pt, y2=f1(fr.bottom)))
        out.append(text(fr.sx(t), fr.bottom + 16, tick_text(t), "tick", "middle"))
    out.append(text((pl + w - pr) / 2, h - 6, xlab, "axlab", "middle"))
    for i, ((label, v, tip, cls), (a, b)) in enumerate(zip(items, spans, strict=True)):
        y = pt + rh * i + 4
        x0, x1 = fr.sx(min(a, b)), fr.sx(max(a, b))
        out.append(text(pl - 10, y + 13, label, "dl", "end"))
        out.append(el("rect", f"bar {cls}{' hatched' if i in hatched else ''}", x=f1(x0),
                      y=f1(y), width=f1(max(x1 - x0, 1.5)), height=16, rx=3, tabindex=0,
                      data_tip=tip))  # fmt: skip
        out.append(text(x1 + 6, y + 13, f"{v:.1f}", "dl"))
    if hatched:
        out.insert(0, '<defs><pattern id="hatch" width="6" height="6" patternUnits="userSpaceOnUse" '
                      'patternTransform="rotate(45)"><rect width="6" height="6" class="hatchbg"/>'
                      '<line x1="0" y1="0" x2="0" y2="6" class="hatchln"/></pattern></defs>')
    return el("svg", "viz", "".join(out), viewbox=f"0 0 {w:g} {h:g}", role="img", aria_label=aria)


def heat_chart(xlabels: Sequence[str], ylabels: Sequence[str],
               values: Sequence[Sequence[float | None]], aria: str, xlab: str = "",
               unit: str = "", dec: int = 2, flags: Sequence[Sequence[bool]] | None = None,
               w: float = 720) -> str:  # fmt: skip
    """Cells on one sequential hue with the value printed in each; a flagged cell also shows "x"."""
    pl, pt, pb, ch = 110.0, 30.0, 40.0, 34.0
    cw = (w - pl - 14) / max(len(xlabels), 1)
    h = pt + ch * len(ylabels) + pb
    flat = [v for row in values for v in row if v is not None and math.isfinite(v)]
    lo, hi = (min(flat), max(flat)) if flat else (0.0, 1.0)
    out = [text(pl + cw * (j + 0.5), pt - 8, xl, "tick", "middle") for j, xl in enumerate(xlabels)]
    for i, yl in enumerate(ylabels):
        y = pt + ch * i
        out.append(text(pl - 8, y + ch / 2 + 4, yl, "dl", "end"))
        for j in range(len(xlabels)):
            v = values[i][j] if j < len(values[i]) else None
            box = {"x": f1(pl + cw * j + 1), "y": f1(y + 1), "width": f1(cw - 2), "height": ch - 2}
            if v is None or not math.isfinite(v):
                out.append(el("rect", "cell na", **box))
                continue
            bad = bool(flags and flags[i][j])
            frac = 0.0 if hi - lo < 1e-12 else (v - lo) / (hi - lo)
            tip = f"{yl}, {xlabels[j]}: {v:.{dec}f} {unit}".strip() + (" (flagged)" if bad else "")
            out.append(el("rect", f"cell{' flag' if bad else ''}", tabindex=0, data_tip=tip,
                          style=f"fill:color-mix(in srgb,var(--seq-hi) {10 + frac * 70:.0f}%,"
                                "var(--seq-lo))", **box))  # fmt: skip
            out.append(
                text(
                    pl + cw * (j + 0.5),
                    y + ch / 2 + 4,
                    f"{v:.{dec}f}{'  x' if bad else ''}",
                    "cellv",
                    "middle",
                )
            )
    out.append(text((pl + w - 14) / 2, h - 8, xlab, "axlab", "middle"))
    return el("svg", "viz", "".join(out), viewbox=f"0 0 {w:g} {h:g}", role="img", aria_label=aria)


Cell = tuple[str, str, str]  # (class, glyph, tip)


def cell_map(
    xs: Sequence[float],
    ys: Sequence[float],
    cell: dict[tuple[float, float], Cell],
    xlab: str,
    ylab: str,
    aria: str,
    w: float = 340,
    h: float = 280,
) -> str:
    """A grid of cells over (x, y): a class (`ok` / `no`), a glyph (a feasible cell is blank,
    an infeasible one carries x) and a tooltip naming the limit; status is never colour alone."""
    pl, pr, pt, pb = 46.0, 8.0, 8.0, 40.0
    cw = (w - pl - pr) / max(len(xs), 1)
    ch = (h - pt - pb) / max(len(ys), 1)
    out: list[str] = []
    for j, x in enumerate(xs):
        out.append(text(pl + cw * (j + 0.5), h - pb + 14, tick_text(x), "tick", "middle"))
    for i, y in enumerate(ys):
        yy = h - pb - ch * (i + 1)
        out.append(text(pl - 5, yy + ch / 2 + 4, tick_text(y), "tick", "end"))
        for j, x in enumerate(xs):
            c = cell.get((x, y))
            if c is None:
                continue
            cls, glyph, tip = c
            box = {"x": f1(pl + cw * j + 0.5), "y": f1(yy + 0.5), "width": f1(cw - 1),
                   "height": f1(ch - 1)}  # fmt: skip
            out.append(el("rect", f"cell {cls}", tabindex=0, data_tip=tip, **box))
            if glyph:
                out.append(text(pl + cw * (j + 0.5), yy + ch / 2 + 4, glyph, "cellv", "middle"))
    out.append(text((pl + w - pr) / 2, h - 6, xlab, "axlab", "middle"))
    out.append(el("text", "axlab", esc(ylab), transform=f"translate(12 {f1((pt + h - pb) / 2)}) "
                  "rotate(-90)", text_anchor="middle"))  # fmt: skip
    return el("svg", "viz", "".join(out), viewbox=f"0 0 {w:g} {h:g}", role="img", aria_label=aria)
