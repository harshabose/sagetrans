"""Analysis B1: open-loop optimal back-transition by direct collocation (section 10B, eq. 10.4).

Hermite-Simpson collocation of the single CasADi vehicle model (smoothing parameter > 0),
free final time, IPOPT. Throttle and pitch-command histories are the unknowns; the pusher stays
at zero. Two objectives: minimum bus energy under an altitude band and a distance cap, or the
smallest altitude excursion (the epsilon_h* of the Pareto front) under a distance cap.

B1 keeps operational limits that B2 and C do not respect (|v_h| <= vh_max at the nodes, a stall
margin, throttle and pitch-command slew limits). It is therefore the best trajectory UNDER THOSE
LIMITS, not a strict lower bound on the baseline or the free-form law: a controller that breaks
them can be cheaper at the same excursion. Relaxing the limits makes the NLP much harder.

Every result must be replayed in `simulate()` (T-11); an optimiser that finds control dither or
a stall-cliff the mesh cannot resolve returns a solution whose replay disagrees, and such a
solution is not `verified`. Controls are EFFECTIVE rotor throttles (ESC dead zone removed from
the NLP model and re-applied in the replay).

Not modelled here: a sourced vortex-ring-state exclusion (no boundary available), and a free
rotor start condition is an option (`prespin_max`) whose spin-up energy before entry is NOT
counted, so comparisons with B2 and C use the default of rotors at rest.
"""

from __future__ import annotations

import math
from collections.abc import Sequence
from dataclasses import dataclass, field, replace

import casadi as ca
import numpy as np

from sagetrans.analysis import runs
from sagetrans.analysis.common import RunRecord, Scenario, make_record
from sagetrans.control.ardupilot_like import BackTransitionParams, Position1Controller
from sagetrans.dynamics.simulate import Result, simulate
from sagetrans.physics.params import VehicleParams
from sagetrans.physics.vehicle import AUX_NAMES, CasadiVehicle

N_X = 9
X_SCALE = np.array([100.0, 10.0, 10.0, 2.0, 0.3, 1.0, 400.0, 600.0, 1.0])  # h is h - h_ref
G = 9.80665
I_V_BUS, I_I_BUS, I_V, I_ALPHA = (AUX_NAMES.index(n) for n in ("V_bus", "I_bus", "V", "alpha"))


@dataclass(frozen=True)
class CollocationOptions:
    n: int = 60  # intervals (the proposal suggests 100 to 200 for production runs)
    smooth: float = 0.02  # current-clip smoothing, A (replay uses the near-exact model)
    delta_max: float = 0.95  # effective rotor throttle ceiling
    delta_rate: float = 5.0  # 1/s
    theta_cmd_max: float = 0.6  # rad
    theta_rate: float = 1.0  # rad/s
    spin_floor: float = 0.0  # rotor throttle floor (Q_M_SPIN_MIN); 0 = none
    v_f: float = 1.0  # end ground speed
    vh_band: float = 0.5  # |v_h| at the end, m/s
    prespin_max: float = 0.0  # rad/s, free rotor start speed (0 = rotors at rest)
    t_f_max: float = 60.0
    max_iter: int = 600
    mu_strategy: str = "monotone"
    stall_constraint: bool = True
    stall_margin: float = 0.85  # |alpha_w| <= margin * alpha_s while the wing carries load
    vh_max: float | None = 3.0  # operational |v_h| limit at the nodes, m/s (None = free)
    tol: float = 1e-5
    hessian: str = "exact"  # limited-memory stalls on this model
    expand: bool = True  # SX expansion: much faster derivatives
    grading: float = 0.5  # mesh grading in [0, 1): 0 uniform, larger = finer steps at both ends
    reg: float = 1e-2  # weight of the control-rate regulariser, relative to the energy in kJ
    verbose: int = 0


@dataclass
class B1Solution:
    ok: bool  # IPOPT reported success
    objective: str
    eps_h: float
    eps_x: float
    t: np.ndarray
    x: np.ndarray  # states (n+1, 9), absolute altitude
    u: np.ndarray  # controls (n+1, 2): delta_r, theta_cmd
    t_f: float
    energy: float  # collocated bus energy, J
    excursion: float  # max |h - h_ref|, m
    distance: float
    iterations: int
    status: str
    record: RunRecord | None = None


def to_guess(sol: B1Solution, n: int = 40) -> tuple[np.ndarray, np.ndarray, float]:
    """Resample a solution onto a uniform mesh, the form `solve` accepts as an initial guess."""
    tt = np.linspace(0.0, sol.t_f, n + 1)
    xg = np.column_stack([np.interp(tt, sol.t, sol.x[:, j]) for j in range(sol.x.shape[1])])
    ug = np.column_stack([np.interp(tt, sol.t, sol.u[:, j]) for j in range(sol.u.shape[1])])
    return xg, ug, sol.t_f


@dataclass
class ReplayCheck:
    """T-11: the collocation result replayed in `simulate()` with the near-exact vehicle."""

    sim: Result
    metrics: dict[str, float]
    collocated: dict[str, float]
    rel_error: dict[str, float]

    def passes(self, tol: float = 0.02, exc_abs: float = 0.3) -> bool:
        """Energy, distance and duration within `tol`; excursion within `tol` or `exc_abs` m.

        The excursion is compared in absolute terms as well because it is a small difference of
        large altitude-rate integrals: at the mesh sizes used here it agrees to about 0.25 m.
        """
        rel_ok = all(self.rel_error[k] <= tol for k in ("energy", "distance", "duration"))
        d_exc = abs(self.metrics["excursion"] - self.collocated["excursion"])
        return rel_ok and d_exc <= max(tol * self.collocated["excursion"], exc_abs)


@dataclass
class Problem:
    vp: VehicleParams  # mass already applied
    scen: Scenario
    x0: np.ndarray
    opt: CollocationOptions
    vehicle: CasadiVehicle = field(init=False)

    def __post_init__(self) -> None:
        # The NLP controls are EFFECTIVE rotor throttles (after the ESC dead zone), so the
        # dead-zone corner, which Hermite-Simpson integrates badly and the optimiser exploits
        # by dithering, is not in the NLP model. Replay maps back with dz + (1 - dz) * eff.
        nlp_params = replace(self.vp, motor_r=replace(self.vp.motor_r, dead_zone=0.0))
        self.vehicle = CasadiVehicle(nlp_params, smooth=self.opt.smooth)

    @property
    def dead_zone(self) -> float:
        return self.vp.motor_r.dead_zone

    @property
    def params(self) -> np.ndarray:
        return np.array([self.scen.isa_offset_k, self.scen.headwind_ms, 1.2, self.scen.soc])


def make_problem(
    vp: VehicleParams, scen: Scenario, v0_air: float = 22.0, opt: CollocationOptions | None = None
) -> Problem:
    vps = scen.vehicle_for(vp)
    x0 = runs.cruise_entry_state(vps, scen, v0_air)
    return Problem(vps, scen, x0, opt or CollocationOptions())


# ---- initial guesses ---------------------------------------------------------------------------


def baseline_run(
    prob: Problem, theta_a: float = math.radians(20.0), t_bt: float = 3.0
) -> Result:
    """The baseline Position1 controller flown from the problem's entry state."""
    from sagetrans.analysis import restricted

    vps, scen, opt = prob.vp, prob.scen, prob.opt
    v_g = float(prob.x0[2])
    a_plan = restricted._plan_decel(v_g, theta_a, t_bt, opt.v_f)
    bp = BackTransitionParams.for_vehicle(
        vps, scen.rho, scen.soc, scen.altitude_amsl_m, T_bt=t_bt, theta_A=theta_a,
        theta_P=restricted.THETA_P, delta_min=0.1, x_tgt=v_g**2 / (2.0 * a_plan),
    )  # fmt: skip
    return simulate(
        runs.vehicle_for(vps), Position1Controller(bp), scen.environment(), prob.x0, 60.0,
        runs.end_events(vps, scen, opt.v_f, 100.0),
    )  # fmt: skip


def baseline_guess(
    prob: Problem, theta_a: float = math.radians(20.0), t_bt: float = 3.0
) -> tuple[np.ndarray, np.ndarray, float]:
    """Dynamic guess from the baseline run: (x nodes, u nodes in effective throttle, T_f)."""
    sim = baseline_run(prob, theta_a, t_bt)
    t_f = float(sim.t[-1])
    tau = np.linspace(0.0, t_f, prob.opt.n + 1)
    cols = ("x", "h", "vx", "vh", "theta", "q", "omega_r", "omega_p", "v1")
    xg = np.column_stack([np.interp(tau, sim.t, sim[c]) for c in cols])
    ug = np.column_stack([np.interp(tau, sim.t, sim[c]) for c in ("delta_r", "theta_cmd")])
    dz = prob.dead_zone
    ug[:, 0] = np.maximum(ug[:, 0] - dz, 0.0) / (1.0 - dz)  # command -> effective throttle
    return xg, ug, t_f


def mesh_fractions(n: int, grading: float) -> np.ndarray:
    """Normalised node times in [0, 1]; steps are (1 - grading) times uniform at both ends."""
    s = np.linspace(0.0, 1.0, n + 1)
    return s - grading * np.sin(2.0 * np.pi * s) / (2.0 * np.pi)


def _resample(
    guess: tuple[np.ndarray, np.ndarray, float], dst_fractions: np.ndarray
) -> tuple[np.ndarray, np.ndarray, float]:
    """Interpolate a guess given on a UNIFORM mesh onto the nodes at `dst_fractions`."""
    xg, ug, tg = guess
    src, dst = np.linspace(0.0, 1.0, len(xg)), dst_fractions
    xs_ = np.column_stack([np.interp(dst, src, xg[:, j]) for j in range(xg.shape[1])])
    us_ = np.column_stack([np.interp(dst, src, ug[:, j]) for j in range(ug.shape[1])])
    return xs_, us_, tg


# ---- the NLP -----------------------------------------------------------------------------------


def _stall_expr(aux: ca.MX, prob: Problem, v_s: float, opt: CollocationOptions) -> ca.MX:
    """<= 0 when the wing is within its stall margin; relaxed below the 1g stall speed."""
    aw = aux[I_ALPHA] + prob.vp.wing.i_w
    load = 1.0 / (1.0 + ca.exp(-(aux[I_V] - v_s)))
    lim = opt.stall_margin * prob.vp.wing.alpha_s
    expr: ca.MX = load * (aw * aw - lim * lim)
    return expr


def solve(
    prob: Problem,
    eps_h: float,
    eps_x: float,
    objective: str = "energy",
    guess: tuple[np.ndarray, np.ndarray, float] | None = None,
) -> B1Solution:
    """Solve one collocation problem; non-convergence is recorded in `ok`, not hidden."""
    opt, f = prob.opt, prob.vehicle.f
    n = opt.n
    h_ref = float(prob.x0[1])
    p = ca.DM(prob.params)
    xs = np.copy(X_SCALE)

    tau = mesh_fractions(n, opt.grading)
    xg, ug, tg = _resample(guess or baseline_guess(prob), tau)
    dtau = np.diff(tau)
    o = ca.Opti()
    z = o.variable(n + 1, N_X)  # scaled states, h relative to h_ref
    u = o.variable(n + 1, 2)
    t_f = o.variable()
    t_scale = 10.0
    s_exc = o.variable() if objective == "excursion" else None
    shift = np.array([0.0, h_ref, 0, 0, 0, 0, 0, 0, 0])

    def xk(k: int) -> ca.MX:
        row: ca.MX = ca.horzcat(*[z[k, j] * xs[j] + shift[j] for j in range(N_X)]).T
        return row

    def uk(k: int) -> ca.MX:
        col: ca.MX = ca.vertcat(u[k, 0], 0.0, u[k, 1])
        return col

    v_s = runs.stall_speed(prob.vp, prob.scen.rho)
    energy = 0.0
    for k in range(n):
        h_step = t_f * t_scale * dtau[k]
        xa, xb = xk(k), xk(k + 1)
        ua, ub = uk(k), uk(k + 1)
        fa, auxa = f(xa, ua, p)
        fb, auxb = f(xb, ub, p)
        xc = 0.5 * (xa + xb) + h_step / 8.0 * (fa - fb)
        uc = 0.5 * (ua + ub)
        fc, auxc = f(xc, uc, p)
        o.subject_to((xb - xa - h_step / 6.0 * (fa + 4.0 * fc + fb)) / xs == 0)
        pw = [auxv[I_V_BUS] * auxv[I_I_BUS] for auxv in (auxa, auxc, auxb)]
        energy += h_step / 6.0 * (pw[0] + 4.0 * pw[1] + pw[2])
        if objective == "excursion":  # altitude band also at the collocation point
            o.subject_to(xc[1] - h_ref <= s_exc)
            o.subject_to(h_ref - xc[1] <= s_exc)
        else:
            o.subject_to(o.bounded(-eps_h, xc[1] - h_ref, eps_h))
        if opt.stall_constraint:  # also at the collocation point, so the cliff is not skipped
            o.subject_to(_stall_expr(auxc, prob, v_s, opt) <= 0)

    # path constraints at the nodes
    for k in range(n + 1):
        _, aux = f(xk(k), uk(k), p)
        o.subject_to(o.bounded(0.0, u[k, 0], opt.delta_max))
        if opt.spin_floor > 0.0:
            o.subject_to(u[k, 0] >= opt.spin_floor)
        o.subject_to(o.bounded(-opt.theta_cmd_max, u[k, 1], opt.theta_cmd_max))
        o.subject_to(aux[I_V_BUS] >= prob.vp.battery.V_min)
        # stall: only while the wing still carries load (V at or above the 1g stall speed)
        if opt.stall_constraint:
            o.subject_to(_stall_expr(aux, prob, v_s, opt) <= 0)
        o.subject_to(z[k, 6] >= 0)
        if opt.vh_max is not None:
            o.subject_to(o.bounded(-opt.vh_max, z[k, 3] * xs[3], opt.vh_max))
        if objective == "excursion":
            o.subject_to(z[k, 1] * xs[1] <= s_exc)
            o.subject_to(-z[k, 1] * xs[1] <= s_exc)
        else:
            o.subject_to(o.bounded(-eps_h, z[k, 1] * xs[1], eps_h))
    for k in range(n):  # slew limits
        hk = t_f * t_scale * dtau[k]
        o.subject_to(o.bounded(-opt.delta_rate * hk, u[k + 1, 0] - u[k, 0], opt.delta_rate * hk))
        o.subject_to(o.bounded(-opt.theta_rate * hk, u[k + 1, 1] - u[k, 1], opt.theta_rate * hk))

    # boundary conditions
    x0 = prob.x0
    for j in range(N_X):
        if j == 6 and opt.prespin_max > 0.0:
            o.subject_to(o.bounded(0.0, z[0, 6] * xs[6], opt.prespin_max))
        else:
            o.subject_to(z[0, j] * xs[j] + shift[j] == x0[j])
    o.subject_to(xk(n)[2] == opt.v_f)  # end: ground speed V_f
    o.subject_to(o.bounded(-opt.vh_band, xk(n)[3], opt.vh_band))
    o.subject_to(xk(n)[0] - x0[0] <= eps_x)
    o.subject_to(o.bounded(1.0, t_f * t_scale, opt.t_f_max))

    # control-rate regulariser: conditions the flat energy optimum (reported, not hidden)
    rate_sq = 0.0
    for k in range(n):
        rate_sq += (
            (u[k + 1, 0] - u[k, 0]) ** 2 + (u[k + 1, 1] - u[k, 1]) ** 2
        ) / (t_f * t_scale * dtau[k])
    reg = opt.reg * rate_sq
    primary = s_exc if s_exc is not None else energy / 1e3
    o.minimize(primary + reg)

    # initial guess
    o.set_initial(z, (xg - shift) / xs)
    o.set_initial(u, ug)
    o.set_initial(t_f, tg / t_scale)
    if s_exc is not None:
        o.set_initial(s_exc, float(np.max(np.abs(xg[:, 1] - h_ref))) + 0.1)
    o.solver(
        "ipopt",
        {"print_time": False, "expand": opt.expand},
        {
            "max_iter": opt.max_iter, "tol": opt.tol, "print_level": opt.verbose, "sb": "yes",
            "hessian_approximation": opt.hessian, "acceptable_tol": 1e-4,
            "acceptable_iter": 8, "mu_strategy": opt.mu_strategy,
            "acceptable_constr_viol_tol": 1e-5,
        },
    )
    ok, status, iters = True, "Solve_Succeeded", 0
    try:
        sol = o.solve()
        get = sol.value
        status = str(sol.stats().get("return_status", ""))
        iters = int(sol.stats().get("iter_count", 0))
    except RuntimeError as exc:  # non-convergence is recorded, not hidden
        st = o.debug.stats()
        ok = False
        status = str(st.get("return_status", str(exc).splitlines()[0][:80]))
        iters = int(st.get("iter_count", 0))
        get = o.debug.value
        if opt.verbose >= 5:
            o.debug.show_infeasibilities(1e-3)
    zv, uv, tfv = np.asarray(get(z)), np.asarray(get(u)), float(get(t_f)) * t_scale
    xv = zv * xs + shift
    t = tau * tfv
    h_dev = np.abs(xv[:, 1] - h_ref)
    e_val = _energy_from(prob, xv, uv, t)
    ok = ok and status in ("Solve_Succeeded", "Solved_To_Acceptable_Level")
    return B1Solution(
        ok, objective, eps_h, eps_x, t, xv, uv, tfv, e_val, float(h_dev.max()),
        float(xv[-1, 0] - x0[0]), iters, status,
        make_record(prob.vp, prob.scen, prob.opt, objective, eps_h, eps_x),
    )  # fmt: skip


def _energy_from(prob: Problem, xv: np.ndarray, uv: np.ndarray, t: np.ndarray) -> float:
    """Hermite-Simpson energy of a trajectory, recomputed with the solved values."""
    f, p = prob.vehicle.f, ca.DM(prob.params)
    n = prob.opt.n
    total = 0.0
    for k in range(n):
        h = float(t[k + 1] - t[k])
        ua = np.array([uv[k, 0], 0.0, uv[k, 1]])
        ub = np.array([uv[k + 1, 0], 0.0, uv[k + 1, 1]])
        fa, aa = f(xv[k], ua, p)
        fb, ab = f(xv[k + 1], ub, p)
        xc = 0.5 * (xv[k] + xv[k + 1]) + h / 8.0 * (np.array(fa).ravel() - np.array(fb).ravel())
        fc, ac = f(xc, 0.5 * (ua + ub), p)
        pw = [float(a[I_V_BUS] * a[I_I_BUS]) for a in (aa, ac, ab)]
        total += h / 6.0 * (pw[0] + 4.0 * pw[1] + pw[2])
    return total


def solve_refined(
    prob: Problem,
    eps_h: float,
    eps_x: float,
    objective: str = "energy",
    ns: Sequence[int] = (20, 40),
    guess: tuple[np.ndarray, np.ndarray, float] | None = None,
) -> B1Solution:
    """Mesh refinement: solve on coarse meshes and warm-start each finer one from the last.

    The final mesh is `prob.opt.n`; `ns` lists the coarser levels solved first. A failure at
    any level stops the chain and returns that (non-converged) solution.
    """
    g = guess or baseline_guess(prob)
    for n in [*ns, prob.opt.n]:
        sub = Problem(prob.vp, prob.scen, prob.x0, replace(prob.opt, n=n))
        sol = solve(sub, eps_h, eps_x, objective, g)
        if not sol.ok:
            return sol
        g = to_guess(sol)
    return sol


# ---- replay (T-11) -----------------------------------------------------------------------------


class OpenLoopController:
    """Plays back a collocated control history, linearly interpolated in time.

    `u[:, 0]` is the effective rotor throttle; `dead_zone` maps it to the ESC command.
    """

    def __init__(self, t: np.ndarray, u: np.ndarray, dead_zone: float = 0.0) -> None:
        self.t, self.u, self.dz = t, u, dead_zone

    def command(
        self, t: float, x: np.ndarray, aux: dict[str, float], mode: object
    ) -> tuple[np.ndarray, object]:
        eff = float(np.interp(t, self.t, self.u[:, 0]))
        d = self.dz + (1.0 - self.dz) * eff
        th = float(np.interp(t, self.t, self.u[:, 1]))
        return np.array([d, 0.0, th]), mode


def replay(prob: Problem, sol: B1Solution) -> ReplayCheck:
    """Replay in `simulate()` (near-exact vehicle, RK4) and compare the headline metrics."""
    veh = runs.vehicle_for(prob.vp)
    sim = simulate(
        veh, OpenLoopController(sol.t, sol.u, prob.dead_zone), prob.scen.environment(), prob.x0,
        sol.t_f + 0.5, runs.end_events(prob.vp, prob.scen, prob.opt.v_f, 100.0)[:1],
    )  # fmt: skip
    h_ref = float(prob.x0[1])
    p_bus = sim["V_bus"] * sim["I_bus"]
    metrics = {
        "energy": float(np.sum(0.5 * (p_bus[1:] + p_bus[:-1]) * np.diff(sim.t))),
        "excursion": float(np.max(np.abs(sim["h"] - h_ref))),
        "distance": float(sim["x"][-1] - sim["x"][0]),
        "duration": float(sim.t[-1]),
    }
    coll = {
        "energy": sol.energy, "excursion": sol.excursion, "distance": sol.distance,
        "duration": sol.t_f,
    }  # fmt: skip
    rel = {
        k: abs(metrics[k] - coll[k]) / max(abs(coll[k]), 1e-9) for k in metrics
    }
    return ReplayCheck(sim, metrics, coll, rel)


# ---- Pareto sweeps, robustness, comparison -----------------------------------------------------


@dataclass
class Verified:
    """A collocation result together with its replay check (T-11) and every attempt made."""

    sol: B1Solution
    check: ReplayCheck | None
    attempts: list[B1Solution]

    @property
    def verified(self) -> bool:
        return self.sol.ok and self.check is not None and self.check.passes()


def solve_verified(
    prob: Problem,
    eps_h: float,
    eps_x: float,
    objective: str = "energy",
    guesses: Sequence[tuple[np.ndarray, np.ndarray, float]] | None = None,
    ns: Sequence[int] = (20,),
) -> Verified:
    """Try several initial guesses and keep the best result that converged AND replays.

    Preference: converged and replay-verified, then converged only, then the first attempt.
    Non-convergence is recorded in the attempts, never hidden.
    """
    starts = ((20.0, 3.0), (12.0, 5.0), (25.0, 2.0))
    gs = list(guesses) if guesses else [
        baseline_guess(prob, math.radians(a), tb) for a, tb in starts
    ]

    def key(s: B1Solution) -> float:
        return s.excursion if objective == "excursion" else s.energy
    attempts: list[B1Solution] = []
    checked: list[tuple[B1Solution, ReplayCheck]] = []
    for g in gs:
        sol = solve_refined(prob, eps_h, eps_x, objective, ns, g)
        attempts.append(sol)
        if sol.ok:
            checked.append((sol, replay(prob, sol)))
    good = [(s, c) for s, c in checked if c.passes()]
    if good:
        best, chk = min(good, key=lambda pair: key(pair[0]))
        return Verified(best, chk, attempts)
    if checked:
        best, chk = min(checked, key=lambda pair: key(pair[0]))
        return Verified(best, chk, attempts)
    return Verified(attempts[0], None, attempts)


def pareto_sweep(
    prob: Problem,
    eps_h_values: Sequence[float],
    eps_x_values: Sequence[float],
    guesses: Sequence[tuple[np.ndarray, np.ndarray, float]] | None = None,
) -> list[tuple[float, float, Verified]]:
    """Energy against (eps_h, eps_x) by the epsilon-constraint method; failures are kept."""
    return [
        (eh, ex, solve_verified(prob, eh, ex, "energy", guesses))
        for eh in eps_h_values
        for ex in eps_x_values
    ]


def min_excursion(prob: Problem, eps_x: float) -> Verified:
    """The smallest altitude excursion achievable under a distance cap (epsilon_h*).

    The least reliable mode: an unverified result (replay disagrees) must not be quoted.
    """
    return solve_verified(prob, 0.0, eps_x, "excursion")


def with_options(prob: Problem, **kw: float | None) -> Problem:
    """The same problem with some options changed."""
    return make_problem(
        prob.vp, prob.scen, float(prob.x0[2] + prob.scen.headwind_ms), replace(prob.opt, **kw)  # type: ignore[arg-type]
    )


# ---- comparison with the baseline, C and B2 ----------------------------------------------------


@dataclass
class BoundCheck:
    """B1 against the baseline controller on the constraints the baseline itself satisfies."""

    baseline: dict[str, float]
    b1: B1Solution
    energy_margin: float  # (baseline - B1) / baseline; >= 0 means B1 is the bound


def bound_check(
    prob: Problem, theta_a: float = math.radians(20.0), t_bt: float = 3.0
) -> BoundCheck:
    """B1 at the baseline's own excursion and distance, under B1's operational limits.

    B1 keeps |v_h|, slew and stall-margin limits that the baseline does not respect. If it is
    still cheaper than the baseline there, the bound (which has fewer limits) is at least that
    good, so a positive `energy_margin` is a one-sided statement about the baseline's headroom.
    """
    sim = baseline_run(prob, theta_a, t_bt)
    h_ref = float(prob.x0[1])
    base = {
        "energy": sim.metrics["energy_J"],
        "excursion": float(np.max(np.abs(sim["h"] - h_ref))),
        "distance": sim.metrics["distance"],
        "vh_max": float(np.max(np.abs(sim["vh"]))),
    }
    sol: B1Solution | None = None
    for ta, tb in ((theta_a, t_bt), (math.radians(15.0), 4.0), (math.radians(25.0), 2.0)):
        sol = solve_refined(
            prob, base["excursion"] * 1.02, base["distance"] * 1.02, "energy", ns=(20,),
            guess=baseline_guess(prob, ta, tb),
        )  # fmt: skip
        if sol.ok:
            break
    assert sol is not None
    return BoundCheck(base, sol, (base["energy"] - sol.energy) / base["energy"])


def compare(
    prob: Problem,
    b1: B1Solution,
    c_params: dict[str, float] | None = None,
    b2_params: object = None,
) -> dict[str, dict[str, float]]:
    """Excursion, energy and distance of B1, the baseline (C) and the free-form law (B2)."""
    from sagetrans.analysis import freeform_opt, restricted

    v0_air = float(prob.x0[2] + prob.scen.headwind_ms)
    out = {"B1": {"excursion": b1.excursion, "energy": b1.energy, "distance": b1.distance}}
    c = restricted.evaluate_back(prob.vp, prob.scen, c_params or restricted.DEFAULT_PARAMS, v0_air)
    out["C"] = {"excursion": c.excursion, "energy": c.energy, "distance": c.distance}
    if b2_params is not None:
        b = freeform_opt.evaluate(prob.vp, prob.scen, b2_params, v0_air)  # type: ignore[arg-type]
        out["B2"] = {"excursion": b.excursion, "energy": b.energy, "distance": b.distance}
    return out
