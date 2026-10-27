"""Analysis A: quasi-steady zero-altitude-loss feasibility scan (section 10A, eqs. 10.2-10.3).

An idealised question, not a flight simulation: can the hardware hold altitude through a constant
deceleration `a` from a free entry speed V0 down to `v_end`, with gamma = 0, no wind and the pusher
off (`t_p = 0`)? The rotors are taken as already spinning at the first schedule value (pre-spun);
there is no entry step and no jerk limit.

At each speed the force balance leaves pitch theta as the one unknown. Eliminating T_r from (7.2)
and (7.3) gives

    R(theta) = (T_p - H) + (m a - D) cos(theta) - (W - L) sin(theta) = 0

which reduces to eq. (10.3) when H = 0 and T_p = 0. The rotor map is then inverted for the speed
that gives T_r, and the motor equation for the throttle including the rotor acceleration term.

The wing angle of attack is confined to a band [alpha_min, alpha_hi]: alpha_min is the lower end of
the range the lift fit was made on, alpha_hi the angle where C_L reaches `cl_frac` x C_L,max on the
rising side (proposal rule 8.6). R(theta) has several roots; the solver prefers roots inside the
band with non-negative rotor thrust, so a root past the lift peak is never taken while an in-band
one exists. For each speed the feasible deceleration then forms a window [a_lo(V), a_hi(V)]; the
lower edge is the wing's drag floor at alpha_min, the upper edge is where the low-theta root pair
disappears (a fold) and only the stalled root is left.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from functools import lru_cache

import casadi as ca
import numpy as np
from scipy.optimize import brentq

from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.dynamics import trim
from sagetrans.physics import aero, rotor
from sagetrans.physics.params import VehicleParams, WingParams

G = 9.80665
THETA_LO, THETA_HI = -0.4, 1.3  # the coarse scan range for the force-balance roots
DENSE_N = 121  # in-band scan points: spacing about 0.08 deg for a 9 deg band
TOL = 1e-9  # a margin below -TOL is a violation


@dataclass(frozen=True)
class ActuatorLimits:
    delta_r_max: float = 0.95  # Q_M_SPIN_MAX-like ceiling on the rotor throttle command
    theta_max: float = 0.6  # rad, pitch limit (never binds: theta = alpha <= alpha_hi at gamma = 0)
    # V_c / v_h onset bound (negative, e.g. -kappa); None = report only (the ratio is always
    # reported, and enforced as the `vrs` margin only when this is set)
    vrs_limit: float | None = None
    # rad; None = no lower band (legacy)
    alpha_min: float | None = 0.0
    # alpha_hi where C_L = cl_frac * C_L,max on the rising side; 1.0 = the lift peak;
    # None = legacy symmetric |alpha| <= alpha_s check
    cl_frac: float | None = 0.8


@dataclass(frozen=True)
class DecelProfile:
    """Constant deceleration `a` from V0 down to `v_end`."""

    a: float
    v_end: float = 1.0

    def sample(self, v0: float, n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        t = np.linspace(0.0, (v0 - self.v_end) / self.a, n)
        return t, v0 - self.a * t, np.full(n, self.a)


DEFAULT_LIMITS = ActuatorLimits()


@lru_cache(maxsize=64)
def _alpha_hi(wing: WingParams, cl_frac: float) -> float:
    """Wing angle of attack where C_L = cl_frac x C_L,max on the rising side (1.0 = the peak)."""
    grid = np.linspace(0.0, math.pi / 2, 1001)
    cl = np.array([float(aero.coefficients(float(a), wing)[0]) for a in grid])
    peak = int(np.argmax(cl))
    if cl_frac >= 1.0:
        return float(grid[peak])
    target = cl_frac * float(cl[peak])

    def f(al: float) -> float:
        return float(aero.coefficients(al, wing)[0]) - target

    return float(brentq(f, 0.0, float(grid[peak]), xtol=1e-12))


def _alpha_band(vp: VehicleParams, limits: ActuatorLimits) -> tuple[float, float]:
    """The allowed band of body pitch theta (= wing angle of attack minus the incidence)."""
    i_w = vp.wing.i_w
    lo = limits.alpha_min if limits.alpha_min is not None else THETA_LO + i_w
    hi = THETA_HI + i_w if limits.cl_frac is None else _alpha_hi(vp.wing, limits.cl_frac)
    return lo - i_w, hi - i_w


def _is_legacy(band: tuple[float, float] | None) -> bool:
    return band is None or band == (THETA_LO, THETA_HI)


def _vrs_ratio(vp: VehicleParams, rho: float, V: float, theta: float, t_r: float) -> float:
    """V_c / v_h at the rotors: V_c = -V sin(theta) (gamma = 0, so nose-up gives a negative
    ratio, the flow coming up through the discs), v_h from the rotor thrust. `rotor.vrs_ratio`
    floors T_r, so the ratio is finite (very negative) as T_r -> 0."""
    return float(rotor.vrs_ratio(-V * math.sin(theta), t_r, rho, vp.rotor))


class _Balance:
    """Compiled R(theta) and the wing forces; built once per vehicle."""

    def __init__(self, vp: VehicleParams) -> None:
        rho, V, th, a, tph = (ca.MX.sym(s) for s in ("rho", "V", "th", "a", "tph"))
        L, D = aero.forces(rho, V, th + vp.wing.i_w, vp.wing)
        w = vp.m * G
        res = tph + (vp.m * a - D) * ca.cos(th) - (w - L) * ca.sin(th)
        self.resid = ca.Function("R", [rho, V, th, a, tph], [res])
        self.forces = ca.Function("LD", [rho, V, th], [L, D])
        self._maps: dict[int, ca.Function] = {}
        self.scan = self.map(61)  # the coarse grid over the whole range
        self.w = w

    def map(self, n: int) -> ca.Function:
        if n not in self._maps:
            self._maps[n] = self.resid.map(n)
        return self._maps[n]


def _roots(
    bal: _Balance, rho: float, V: float, a: float, tph: float,
    lo: float = THETA_LO, hi: float = THETA_HI, n: int = 61,
) -> list[float]:  # fmt: skip
    """Roots of R(theta) on [lo, hi]: a sign scan on `n` points, then brentq in each bracket."""
    grid = np.linspace(lo, hi, n)
    vals = np.asarray(bal.map(n)(rho, V, grid, a, tph)).ravel()
    out = []
    for i in range(len(vals) - 1):
        if vals[i] == 0.0:
            out.append(float(grid[i]))
        elif vals[i] * vals[i + 1] < 0.0:
            out.append(
                float(
                    brentq(
                        lambda th: float(bal.resid(rho, V, th, a, tph)),
                        grid[i], grid[i + 1], xtol=1e-13, rtol=1e-14,
                    )  # fmt: skip
                )
            )
    return out


@dataclass(frozen=True)
class Point:
    theta: float
    T_r: float
    H_r: float
    omega: float
    L: float
    D: float
    ok: bool


def _choose(
    cands: list[tuple[float, float, float, float]],
    band: tuple[float, float] | None,
    ref: float,
) -> tuple[float, float, float, float]:
    """Pick one root. Legacy (no band): nearest `ref` among roots with T_r >= 0, else any.

    With a band: in-band roots with T_r >= 0 first, then in-band roots with T_r < 0 (these fail
    `thrust_sign`), then the out-of-band root nearest the band (it fails an alpha margin).
    """
    if band is None:
        pos = [c for c in cands if c[1] >= 0.0] or cands
        return min(pos, key=lambda c: abs(c[0] - ref))
    lo, hi = band
    inside = [c for c in cands if lo - 1e-12 <= c[0] <= hi + 1e-12]
    if inside:
        pos = [c for c in inside if c[1] >= 0.0] or inside
        return min(pos, key=lambda c: abs(c[0] - ref))
    return min(cands, key=lambda c: (max(lo - c[0], c[0] - hi), abs(c[0] - ref)))


def solve_point(
    vp: VehicleParams,
    rho: float,
    V: float,
    a: float,
    t_p: float = 0.0,
    prev_theta: float | None = None,
    bal: _Balance | None = None,
    band: tuple[float, float] | None = None,
) -> Point:
    """Pitch, rotor thrust and rotor speed that hold altitude at airspeed V, deceleration a.

    `band` is the allowed range of theta. With `band=None` (or the full scan range) the root
    nearest the previous theta among those with T_r >= 0 is taken, as before; with a band, roots
    inside it are preferred (see `_choose`), found on a dense grid so that a close pair of roots
    near a fold is not missed.
    """
    bal = bal or _Balance(vp)
    legacy = _is_legacy(band)
    h_r = 0.0
    best = Point(math.nan, math.nan, 0.0, math.nan, math.nan, math.nan, False)
    for _ in range(30):
        tph = t_p - h_r
        roots = _roots(bal, rho, V, a, tph)
        if not legacy:
            assert band is not None
            for r in _roots(bal, rho, V, a, tph, band[0], band[1], DENSE_N):
                if all(abs(r - q) > 1e-9 for q in roots):
                    roots.append(r)
        if not roots:
            return best
        cands = []
        for th in roots:
            L, D = (float(v) for v in bal.forces(rho, V, th))
            t_r = (bal.w - L - t_p * math.sin(th) + h_r * math.sin(th)) / math.cos(th)
            cands.append((th, t_r, L, D))
        ref = 0.0 if prev_theta is None else prev_theta
        th, t_r, L, D = _choose(cands, None if legacy else band, ref)
        omega = trim.rotor_speed_for_thrust(t_r, -V * math.sin(th), V * math.cos(th), rho, vp)
        vc, ve = -V * math.sin(th), V * math.cos(th)
        h_new = float(rotor.lift_rotor(omega, vc, ve, rho, vp.rotor)[2])
        best = Point(th, t_r, h_new, omega, L, D, t_r >= 0.0)
        if abs(h_new - h_r) < 1e-9:
            break
        h_r = h_new
    return best


def _margins(
    vp: VehicleParams,
    scen: Scenario,
    limits: ActuatorLimits,
    band_alpha: tuple[float, float],
    t: np.ndarray,
    V: np.ndarray,
    th: np.ndarray,
    t_r: np.ndarray,
    om: np.ndarray,
    solved: np.ndarray,
    om_dot: np.ndarray,
    th_dot: np.ndarray,
    v1_0: float,
) -> tuple[dict[str, np.ndarray], tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]]:
    """Normalised margins (>= 0 is feasible) along a profile, and the throttle, motor current, bus
    voltage and V_c / v_h ratio (NaN where unsolved) they were computed from. Shared by the path
    check and the window scan. The `vrs` margin exists only when `limits.vrs_limit` is set."""
    n = len(V)
    rho = scen.rho
    delta, i_m, vb = np.zeros(n), np.zeros(n), np.zeros(n)
    v1 = v1_0
    tau = vp.battery.R1 * vp.battery.C1
    for k in range(n):
        if not solved[k]:
            delta[k], i_m[k], vb[k] = math.nan, math.nan, math.nan
            continue
        vc, ve = -V[k] * math.sin(th[k]), V[k] * math.cos(th[k])
        d = trim.required_rotor_command(
            om[k], float(om_dot[k]), vc, ve, rho, vp, scen.soc, v1, 0.0, 0.0
        )
        delta[k], i_m[k], vb[k] = d.delta_cmd, d.current, d.v_bus
        if k < n - 1:
            dt_k = float(t[k + 1] - t[k])
            i_bus = vp.rotor.n_r * d.current + vp.battery.I_av
            v1 = v1 * math.exp(-dt_k / tau) + vp.battery.R1 * i_bus * (1 - math.exp(-dt_k / tau))
    # torque margin: required Q_m minus what natural spin-down gives (floor at zero throttle)
    q_i = np.array(
        [
            float(rotor.lift_rotor(om[k], -V[k] * math.sin(th[k]), V[k] * math.cos(th[k]),
                                   rho, vp.rotor)[1]) if solved[k] else math.nan
            for k in range(n)
        ]
    )  # fmt: skip
    torque_req = q_i + vp.rotor.I_rot * om_dot
    alpha = th + vp.wing.i_w
    ratio = np.array(
        [_vrs_ratio(vp, rho, float(V[k]), float(th[k]), float(t_r[k])) if solved[k] else math.nan
         for k in range(n)]
    )  # fmt: skip
    m = {
        "pitch_solution": np.where(solved, 1.0, -1.0),
        "thrust_sign": np.where(solved, t_r / (vp.m * G), -1.0),
        "throttle": (limits.delta_r_max - delta) / limits.delta_r_max,
        "current": (vp.motor_r.I_lim - np.where(torque_req > 0, i_m, 0.0)) / vp.motor_r.I_lim,
        "voltage": (vb - vp.battery.V_min) / vp.battery.V_min,
        "torque_floor": torque_req / (vp.motor_r.Kt * vp.motor_r.I_lim) + 1e-9,
        "pitch": (limits.theta_max - np.abs(th)) / limits.theta_max,
        "pitch_rate": (vp.attitude.q_max - np.abs(th_dot)) / vp.attitude.q_max,
    }
    a_lo, a_hi = band_alpha
    if limits.alpha_min is not None:
        m["alpha_low"] = (alpha - a_lo) / (a_hi - a_lo)
    if limits.cl_frac is not None:
        m["alpha_high"] = (a_hi - alpha) / (a_hi - a_lo)
    else:  # legacy symmetric stall check
        m["stall"] = (vp.wing.alpha_s - np.abs(alpha)) / vp.wing.alpha_s
    if limits.vrs_limit is not None:
        m["vrs"] = (ratio - limits.vrs_limit) / abs(limits.vrs_limit)
    out = {name: np.where(np.isnan(v), -1.0, v) for name, v in m.items()}
    return out, (delta, i_m, vb, ratio)


def _violations(margins: dict[str, np.ndarray], t: np.ndarray) -> dict[str, float]:
    """Each margin that goes below zero, with the first time it does, earliest first."""
    found: dict[str, float] = {}
    for name, v in margins.items():
        bad = np.where(v < -TOL)[0]
        if bad.size:
            found[name] = float(t[int(bad[0])])
    return dict(sorted(found.items(), key=lambda kv: kv[1]))  # stable: ties keep margin order


@dataclass
class ZeroLossResult:
    t: np.ndarray
    V: np.ndarray
    theta: np.ndarray
    T_r: np.ndarray
    omega_r: np.ndarray
    margins: dict[str, np.ndarray]
    delta_cmd: np.ndarray
    I_m: np.ndarray
    V_bus: np.ndarray
    feasible: bool
    violations: dict[str, float]  # violated margin -> first time [s], earliest first
    vrs_min: float  # most negative V_c / v_h on the path (reported always; nan if nothing solved)
    vrs_min_t: float  # time [s] at which it occurs
    record: RunRecord = field(default_factory=lambda: make_record())


def first_binding(result: ZeroLossResult) -> tuple[str, float] | None:
    """The earliest violation, or None when feasible (the old single-name shape)."""
    for name, time in result.violations.items():
        return name, time
    return None


def analyse(
    vp: VehicleParams,
    scen: Scenario,
    v0: float,
    profile: DecelProfile,
    limits: ActuatorLimits = DEFAULT_LIMITS,
    t_p: float = 0.0,
    n: int = 120,
    v1_0: float = 0.0,
) -> ZeroLossResult:
    """Check one constant-deceleration profile from `v0` down to `profile.v_end`.

    Rotors are pre-spun; `v1_0` is the battery RC state at the first sample.
    """
    rho = scen.rho
    bal = _Balance(vp)
    legacy = limits.alpha_min is None and limits.cl_frac is None
    band = None if legacy else _alpha_band(vp, limits)
    band_theta = band if band is not None else (THETA_LO, THETA_HI)
    band_alpha = (band_theta[0] + vp.wing.i_w, band_theta[1] + vp.wing.i_w)
    t, V, a_t = profile.sample(v0, n)
    pts: list[Point] = []
    prev: float | None = None
    for k in range(n):
        p = solve_point(vp, rho, float(V[k]), float(a_t[k]), t_p, prev, bal, band)
        pts.append(p)
        if p.ok or not math.isnan(p.theta):
            prev = p.theta
    th = np.array([p.theta for p in pts])
    t_r = np.array([p.T_r for p in pts])
    om = np.array([p.omega for p in pts])
    solved = np.array([not math.isnan(p.theta) for p in pts])
    om_dot, th_dot = np.gradient(om, t), np.gradient(th, t)
    margins, (delta, i_m, vb, ratio) = _margins(
        vp, scen, limits, band_alpha, t, V, th, t_r, om, solved, om_dot, th_dot, v1_0
    )  # fmt: skip
    viol = _violations(margins, t)
    finite = np.isfinite(ratio)
    k_min = int(np.nanargmin(ratio)) if finite.any() else -1
    vrs_min, vrs_t = (float(ratio[k_min]), float(t[k_min])) if k_min >= 0 else (math.nan, math.nan)
    return ZeroLossResult(
        t, V, th, t_r, om, margins, delta, i_m, vb, not viol, viol, vrs_min, vrs_t,
        make_record(vp, scen, profile, limits, v1_0, t_p, v0),
    )  # fmt: skip


@dataclass(frozen=True)
class Window:
    """A contiguous range of feasible decelerations at one airspeed."""

    a_lo: float
    a_hi: float
    theta_lo: float  # pitch at the two edges [rad]
    theta_hi: float
    T_r_lo: float  # rotor thrust at the two edges [N]
    T_r_hi: float
    bind_lo: list[str]  # every margin violated just below / above the window
    bind_hi: list[str]
    vrs_lo: float = math.nan  # V_c / v_h at the feasible point on each edge (reported always)
    vrs_hi: float = math.nan


def _probe(
    vp: VehicleParams, scen: Scenario, V: float, a: float, limits: ActuatorLimits,
    t_p: float, bal: _Balance,
) -> tuple[list[str], float, Point]:  # fmt: skip
    """Steady point check: the violated margin names, the worst margin, and the point."""
    band = _alpha_band(vp, limits)
    band_alpha = (band[0] + vp.wing.i_w, band[1] + vp.wing.i_w)
    p = solve_point(vp, scen.rho, V, a, t_p, None, bal, band)
    z = np.zeros(1)
    marg, _ = _margins(
        vp, scen, limits, band_alpha, z, np.array([V]), np.array([p.theta]), np.array([p.T_r]),
        np.array([p.omega]), np.array([not math.isnan(p.theta)]), z, z, 0.0,
    )  # fmt: skip
    names = [k for k, v in marg.items() if v[0] < -TOL]
    return names, min(float(v[0]) for v in marg.values()), p


def _edge(
    vp: VehicleParams, scen: Scenario, V: float, good: float, bad: float,
    bad_names: list[str], limits: ActuatorLimits, t_p: float, bal: _Balance, bisect: int,
) -> tuple[float, list[str], Point]:  # fmt: skip
    """Bisect between a feasible `good` and an infeasible `bad` deceleration."""
    names = bad_names
    for _ in range(bisect):
        mid = 0.5 * (good + bad)
        nm = _probe(vp, scen, V, mid, limits, t_p, bal)[0]
        if nm:
            bad, names = mid, nm
        else:
            good = mid
    return good, names, _probe(vp, scen, V, good, limits, t_p, bal)[2]


def feasible_window(
    vp: VehicleParams,
    scen: Scenario,
    V: float,
    limits: ActuatorLimits = DEFAULT_LIMITS,
    t_p: float = 0.0,
    a_max: float = 3.0,
    da: float = 0.02,
    bisect: int = 9,
) -> list[Window]:
    """Feasible constant decelerations at airspeed V, as windows [a_lo, a_hi].

    A pointwise scan of the force balance with a **steady rotor** (rotor acceleration 0, battery
    RC state 0): it shows the geometry of the feasible set at one speed. The verdict for a whole
    constant-a path from V0 to rest is `analyse` / `feasible_decel_range`, which also include the
    rotor acceleration, battery state and pitch rate. Empty list = no feasible deceleration.
    """
    bal = _Balance(vp)
    grid = np.arange(0.0, a_max, da)
    info = [_probe(vp, scen, V, float(a), limits, t_p, bal) for a in grid]
    ok = [not i[0] for i in info]
    windows: list[Window] = []
    k = 0
    while k < len(grid):
        if not ok[k]:
            k += 1
            continue
        i = k
        while k + 1 < len(grid) and ok[k + 1]:
            k += 1
        j = k
        k += 1
        lo_a, lo_pt = float(grid[i]), info[i][2]
        bind_lo: list[str] = []
        if i > 0:
            lo_a, bind_lo, lo_pt = _edge(
                vp, scen, V, float(grid[i]), float(grid[i - 1]), info[i - 1][0], limits, t_p, bal,
                bisect,
            )  # fmt: skip
        hi_a, hi_pt = float(grid[j]), info[j][2]
        bind_hi: list[str] = []
        if j < len(grid) - 1:
            hi_a, bind_hi, hi_pt = _edge(
                vp, scen, V, float(grid[j]), float(grid[j + 1]), info[j + 1][0], limits, t_p, bal,
                bisect,
            )  # fmt: skip
        windows.append(
            Window(
                lo_a, hi_a, lo_pt.theta, hi_pt.theta, lo_pt.T_r, hi_pt.T_r, bind_lo, bind_hi,
                _vrs_ratio(vp, scen.rho, V, lo_pt.theta, lo_pt.T_r),
                _vrs_ratio(vp, scen.rho, V, hi_pt.theta, hi_pt.T_r),
            )
        )  # fmt: skip
    return windows


def closest_to_feasible(
    vp: VehicleParams,
    scen: Scenario,
    V: float,
    limits: ActuatorLimits = DEFAULT_LIMITS,
    t_p: float = 0.0,
    a_max: float = 3.0,
    da: float = 0.02,
) -> tuple[float, list[str]]:
    """When `feasible_window` is empty: the deceleration whose worst margin is least negative,
    and the margins it violates (what to change to get a window)."""
    bal = _Balance(vp)
    best: tuple[float, float, list[str]] = (-math.inf, 0.0, [])
    for a in np.arange(0.0, a_max, da):
        names, worst, _ = _probe(vp, scen, V, float(a), limits, t_p, bal)
        if worst > best[0]:
            best = (worst, float(a), names)
    return best[1], best[2]


def feasible_decel_range(
    vp: VehicleParams,
    scen: Scenario,
    v0: float,
    limits: ActuatorLimits = DEFAULT_LIMITS,
    n: int = 60,
    a_lo: float = 0.05,
    a_hi: float = 4.0,
    scan_points: int = 48,
    iterations: int = 9,
) -> tuple[float, float, bool] | None:
    """Smallest and largest constant deceleration for which the whole path from `v0` to rest is
    feasible (`analyse`, the authoritative check), and whether the feasible set has gaps.

    A scan on `scan_points` values finds the first and last feasible ones; each edge is then
    bisected against its infeasible neighbour. `islands` is True if the feasible grid points are
    not contiguous: the full span is still returned, and the flag says it is not all feasible.
    A feasible range narrower than the scan spacing (0.084 m/s^2 for the defaults) can be missed.
    None = nothing feasible.
    """

    def ok(a: float) -> bool:
        return analyse(vp, scen, v0, DecelProfile(a), limits, n=n).feasible

    grid = np.linspace(a_lo, a_hi, scan_points)
    flags = [ok(float(a)) for a in grid]
    idx = [i for i, f in enumerate(flags) if f]
    if not idx:
        return None
    first, last = idx[0], idx[-1]
    islands = (last - first + 1) != len(idx)
    a_min = float(grid[first])
    if first > 0:
        bad, good = float(grid[first - 1]), float(grid[first])
        for _ in range(iterations):
            mid = 0.5 * (bad + good)
            bad, good = (bad, mid) if ok(mid) else (mid, good)
        a_min = good
    a_max = float(grid[last])
    if last < len(grid) - 1:
        good, bad = float(grid[last]), float(grid[last + 1])
        for _ in range(iterations):
            mid = 0.5 * (good + bad)
            good, bad = (mid, bad) if ok(mid) else (good, mid)
        a_max = good
    return a_min, a_max, islands


def max_feasible_decel(
    vp: VehicleParams,
    scen: Scenario,
    v0: float,
    limits: ActuatorLimits = DEFAULT_LIMITS,
    a_lo: float = 0.05,
    a_hi: float = 4.0,
    n: int = 60,
    iterations: int = 9,
    scan_points: int = 48,
) -> float:
    """Largest feasible constant deceleration from `v0` to rest (0 if none); see
    `feasible_decel_range`, which also gives the smallest and flags gaps."""
    r = feasible_decel_range(vp, scen, v0, limits, n, a_lo, a_hi, scan_points, iterations)
    return 0.0 if r is None else r[1]


def feasibility_map(
    vp: VehicleParams,
    scen: Scenario,
    v0_grid: list[float],
    a_grid: list[float],
    limits: ActuatorLimits = DEFAULT_LIMITS,
    n: int = 60,
) -> dict[str, np.ndarray | list[list[dict[str, float]]]]:
    """Feasibility over (V0, a), and for each cell every violated margin with its first time,
    plus the most negative V_c / v_h on the path and when it occurs (`vrs_min`, `vrs_min_t`)."""
    feasible = np.zeros((len(v0_grid), len(a_grid)), dtype=bool)
    vrs = np.full(feasible.shape, math.nan)
    vrs_t = np.full(feasible.shape, math.nan)
    viol: list[list[dict[str, float]]] = [[{} for _ in a_grid] for _ in v0_grid]
    for i, v0 in enumerate(v0_grid):
        for j, a in enumerate(a_grid):
            r = analyse(vp, scen, v0, DecelProfile(a), limits, n=n)
            feasible[i, j] = r.feasible
            viol[i][j] = r.violations
            vrs[i, j], vrs_t[i, j] = r.vrs_min, r.vrs_min_t
    return {"feasible": feasible, "violations": viol, "vrs_min": vrs, "vrs_min_t": vrs_t}


__all__ = [
    "DEFAULT_LIMITS",
    "ActuatorLimits",
    "DecelProfile",
    "Window",
    "ZeroLossResult",
    "analyse",
    "closest_to_feasible",
    "feasibility_map",
    "feasible_decel_range",
    "feasible_window",
    "first_binding",
    "max_feasible_decel",
    "solve_point",
]
