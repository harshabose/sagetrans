"""Analysis A: zero-altitude-loss reference and feasibility (section 10A, eqs. 10.2-10.3).

For a prescribed airspeed history (v_h = 0, gamma = 0, no wind, pusher thrust fixed) the force
balance leaves pitch theta as the one unknown. Eliminating T_r from (7.2) and (7.3) gives

    R(theta) = (T_p - H) + (m a - D) cos(theta) - (W - L) sin(theta) = 0

which reduces to eq. (10.3) when H = 0 and T_p = 0. The rotor map is then inverted for the speed
that gives T_r, and the motor equation for the throttle including the rotor acceleration term.

Entry discontinuity: at entry the wing carries the weight and the rotors are at rest, so the
required rotor speed jumps from its entry value to the schedule. Each result therefore carries
two verdicts: the whole segment including the entry step, and the schedule with the rotors
assumed already spinning at the schedule's first value (the V_on idea of section 8.1).
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field

import casadi as ca
import numpy as np
from scipy.optimize import brentq

from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.dynamics import trim
from sagetrans.physics import aero, rotor
from sagetrans.physics.params import VehicleParams

G = 9.80665
THETA_LO, THETA_HI = -0.4, 1.3


@dataclass(frozen=True)
class ActuatorLimits:
    delta_r_max: float = 0.95  # Q_M_SPIN_MAX-like ceiling on the rotor throttle command
    theta_max: float = 0.6  # rad, pitch limit
    vrs_limit: float | None = None  # V_c / v_h lower bound; None = report only (unsourced)


@dataclass(frozen=True)
class DecelProfile:
    """Constant deceleration `a`, optionally reached through a jerk limit, from V0 to v_end."""

    a: float
    jerk: float | None = None
    v_end: float = 1.0

    def sample(self, v0: float, n: int) -> tuple[np.ndarray, np.ndarray, np.ndarray]:
        a, j = self.a, self.jerk
        dv = v0 - self.v_end
        if j is None:
            t_end = dv / a
        elif a * a / (2.0 * j) >= dv:  # stops while still ramping
            t_end = math.sqrt(2.0 * dv / j)
        else:
            t1 = a / j
            t_end = t1 + (dv - a * a / (2.0 * j)) / a
        t = np.linspace(0.0, t_end, n)
        if j is None:
            return t, v0 - a * t, np.full(n, a)
        t1 = a / j
        a_t = np.minimum(a, j * t)
        v = np.where(t <= t1, v0 - 0.5 * j * t**2, v0 - 0.5 * a * t1 - a * (t - t1))
        return t, v, a_t


@dataclass(frozen=True)
class EntryState:
    """State just before the first sample. theta=None means level trim at V0 (a = 0)."""

    omega_r: float = 0.0
    theta: float | None = None
    v1: float = 0.0


DEFAULT_LIMITS = ActuatorLimits()
DEFAULT_ENTRY = EntryState()


class _Balance:
    """Compiled R(theta) and the wing forces; built once per vehicle."""

    def __init__(self, vp: VehicleParams) -> None:
        rho, V, th, a, tph = (ca.MX.sym(s) for s in ("rho", "V", "th", "a", "tph"))
        L, D = aero.forces(rho, V, th + vp.wing.i_w, vp.wing)
        w = vp.m * G
        res = tph + (vp.m * a - D) * ca.cos(th) - (w - L) * ca.sin(th)
        self.resid = ca.Function("R", [rho, V, th, a, tph], [res])
        self.forces = ca.Function("LD", [rho, V, th], [L, D])
        self.scan = self.resid.map(61)
        self.grid = np.linspace(THETA_LO, THETA_HI, 61)
        self.w = w


def _roots(bal: _Balance, rho: float, V: float, a: float, tph: float) -> list[float]:
    vals = np.asarray(bal.scan(rho, V, bal.grid, a, tph)).ravel()
    out = []
    for i in range(len(vals) - 1):
        if vals[i] == 0.0:
            out.append(float(bal.grid[i]))
        elif vals[i] * vals[i + 1] < 0.0:
            out.append(
                float(
                    brentq(
                        lambda th: float(bal.resid(rho, V, th, a, tph)),
                        bal.grid[i], bal.grid[i + 1], xtol=1e-13, rtol=1e-14,
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


def solve_point(
    vp: VehicleParams,
    rho: float,
    V: float,
    a: float,
    t_p: float = 0.0,
    prev_theta: float | None = None,
    bal: _Balance | None = None,
) -> Point:
    """Pitch, rotor thrust and rotor speed that hold altitude at airspeed V, deceleration a."""
    bal = bal or _Balance(vp)
    h_r = 0.0
    best = Point(math.nan, math.nan, 0.0, math.nan, math.nan, math.nan, False)
    for _ in range(30):
        roots = _roots(bal, rho, V, a, t_p - h_r)
        if not roots:
            return best
        cands = []
        for th in roots:
            L, D = (float(v) for v in bal.forces(rho, V, th))
            t_r = (bal.w - L - t_p * math.sin(th) + h_r * math.sin(th)) / math.cos(th)
            cands.append((th, t_r, L, D))
        pos = [c for c in cands if c[1] >= 0.0] or cands
        ref = 0.0 if prev_theta is None else prev_theta
        th, t_r, L, D = min(pos, key=lambda c: abs(c[0] - ref))
        omega = trim.rotor_speed_for_thrust(t_r, -V * math.sin(th), V * math.cos(th), rho, vp)
        vc, ve = -V * math.sin(th), V * math.cos(th)
        h_new = float(rotor.lift_rotor(omega, vc, ve, rho, vp.rotor)[2])
        best = Point(th, t_r, h_new, omega, L, D, t_r >= 0.0)
        if abs(h_new - h_r) < 1e-9:
            break
        h_r = h_new
    return best


@dataclass
class ZeroLossResult:
    t: np.ndarray
    V: np.ndarray
    theta: np.ndarray
    T_r: np.ndarray
    omega_r: np.ndarray
    margins_whole: dict[str, np.ndarray]
    margins_pre_spun: dict[str, np.ndarray]
    delta_cmd: np.ndarray
    I_m: np.ndarray
    V_bus: np.ndarray
    feasible_whole: bool
    feasible_pre_spun: bool
    first_binding_whole: tuple[str, float] | None
    first_binding_pre_spun: tuple[str, float] | None
    record: RunRecord = field(default_factory=lambda: make_record())


def _first_binding(
    margins: dict[str, np.ndarray], t: np.ndarray
) -> tuple[bool, tuple[str, float] | None]:
    stack = np.vstack([margins[k] for k in margins])
    bad = np.where(np.any(stack < -1e-9, axis=0))[0]
    if bad.size == 0:
        return True, None
    i = int(bad[0])
    name = list(margins)[int(np.argmin(stack[:, i]))]
    return False, (name, float(t[i]))


def analyse(
    vp: VehicleParams,
    scen: Scenario,
    v0: float,
    profile: DecelProfile,
    limits: ActuatorLimits = DEFAULT_LIMITS,
    entry: EntryState = DEFAULT_ENTRY,
    t_p: float = 0.0,
    n: int = 120,
) -> ZeroLossResult:
    rho = scen.rho
    bal = _Balance(vp)
    t, V, a_t = profile.sample(v0, n)
    pts: list[Point] = []
    prev: float | None = None
    for k in range(n):
        p = solve_point(vp, rho, float(V[k]), float(a_t[k]), t_p, prev, bal)
        pts.append(p)
        if p.ok or not math.isnan(p.theta):
            prev = p.theta
    th = np.array([p.theta for p in pts])
    t_r = np.array([p.T_r for p in pts])
    om = np.array([p.omega for p in pts])
    solved = np.array([not math.isnan(p.theta) for p in pts])

    th_entry = entry.theta
    if th_entry is None:
        p0 = solve_point(vp, rho, v0, 0.0, t_p, None, bal)
        th_entry = p0.theta if not math.isnan(p0.theta) else 0.0

    def build(om_prev: float, th_prev: float, t_prev: float) -> dict[str, np.ndarray]:
        tt = np.concatenate([[t_prev], t])
        om_dot = np.gradient(np.concatenate([[om_prev], om]), tt)[1:]
        th_dot = np.gradient(np.concatenate([[th_prev], th]), tt)[1:]
        delta = np.zeros(n)
        i_m = np.zeros(n)
        vb = np.zeros(n)
        v1 = entry.v1
        for k in range(n):
            if not solved[k]:
                delta[k], i_m[k], vb[k] = math.nan, math.nan, math.nan
                continue
            vc, ve = -V[k] * math.sin(th[k]), V[k] * math.cos(th[k])
            d = trim.required_rotor_command(
                om[k], float(om_dot[k]), vc, ve, rho, vp, scen.soc, v1, 0.0, 0.0
            )
            delta[k], i_m[k], vb[k] = d.delta_cmd, d.current, d.v_bus
            dt_k = tt[k + 1] - tt[k]
            i_bus = vp.rotor.n_r * d.current + vp.battery.I_av
            tau = vp.battery.R1 * vp.battery.C1
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
        aw = th + vp.wing.i_w
        alpha_s = vp.wing.alpha_s
        m = {
            "pitch_solution": np.where(solved, 1.0, -1.0),
            "thrust_sign": np.where(solved, t_r / (vp.m * G), -1.0),
            "throttle": (limits.delta_r_max - delta) / limits.delta_r_max,
            "current": (vp.motor_r.I_lim - np.where(torque_req > 0, i_m, 0.0)) / vp.motor_r.I_lim,
            "voltage": (vb - vp.battery.V_min) / vp.battery.V_min,
            "torque_floor": torque_req / (vp.motor_r.Kt * vp.motor_r.I_lim) + 1e-9,
            "pitch": (limits.theta_max - np.abs(th)) / limits.theta_max,
            "pitch_rate": (vp.attitude.q_max - np.abs(th_dot)) / vp.attitude.q_max,
            "stall": (alpha_s - np.abs(aw)) / alpha_s,
        }
        out = {name: np.where(np.isnan(v), -1.0, v) for name, v in m.items()}
        out["_delta"], out["_i"], out["_vb"] = delta, i_m, vb
        return out

    dt0 = float(t[1] - t[0])
    whole = build(entry.omega_r, th_entry, -dt0)
    spun = build(float(om[0]) if solved[0] else 0.0, float(th[0]), -dt0)
    aux = {key: spun.pop(key) for key in ("_delta", "_i", "_vb")}
    for key in ("_delta", "_i", "_vb"):
        whole.pop(key)
    ok_w, fb_w = _first_binding(whole, t)
    ok_s, fb_s = _first_binding(spun, t)
    return ZeroLossResult(
        t, V, th, t_r, om, whole, spun, aux["_delta"], aux["_i"], aux["_vb"],
        ok_w, ok_s, fb_w, fb_s, make_record(vp, scen, profile, limits, entry, t_p, v0),
    )  # fmt: skip


def max_feasible_decel(
    vp: VehicleParams,
    scen: Scenario,
    v0: float,
    limits: ActuatorLimits = DEFAULT_LIMITS,
    pre_spun: bool = True,
    a_lo: float = 0.2,
    a_hi: float = 8.0,
    jerk: float | None = None,
    n: int = 60,
    iterations: int = 9,
) -> float:
    """Largest constant deceleration with a feasible zero-loss schedule (0 if none)."""

    def ok(a: float) -> bool:
        r = analyse(vp, scen, v0, DecelProfile(a, jerk), limits, n=n)
        return r.feasible_pre_spun if pre_spun else r.feasible_whole

    if not ok(a_lo):
        return 0.0
    if ok(a_hi):
        return a_hi
    lo, hi = a_lo, a_hi
    for _ in range(iterations):
        mid = 0.5 * (lo + hi)
        lo, hi = (mid, hi) if ok(mid) else (lo, mid)
    return lo


def feasibility_map(
    vp: VehicleParams,
    scen: Scenario,
    v0_grid: list[float],
    a_grid: list[float],
    limits: ActuatorLimits = DEFAULT_LIMITS,
    n: int = 60,
) -> dict[str, np.ndarray | list[list[tuple[str, float] | None]]]:
    """Feasibility over (V0, a): both verdicts and the first binding constraint of each."""
    shape = (len(v0_grid), len(a_grid))
    whole = np.zeros(shape, dtype=bool)
    spun = np.zeros(shape, dtype=bool)
    bind: list[list[tuple[str, float] | None]] = [[None] * len(a_grid) for _ in v0_grid]
    for i, v0 in enumerate(v0_grid):
        for j, a in enumerate(a_grid):
            r = analyse(vp, scen, v0, DecelProfile(a), limits, n=n)
            whole[i, j], spun[i, j] = r.feasible_whole, r.feasible_pre_spun
            bind[i][j] = r.first_binding_pre_spun
    return {"whole": whole, "pre_spun": spun, "first_binding_pre_spun": bind}


__all__ = [
    "ActuatorLimits",
    "DecelProfile",
    "EntryState",
    "ZeroLossResult",
    "analyse",
    "feasibility_map",
    "max_feasible_decel",
    "solve_point",
]
