"""Analysis FT (docs/FT_spec.md): configuration, the z_loop split and, in later steps, the model."""

import shutil
from pathlib import Path

import pytest
from pydantic import ValidationError

from sagetrans import atmosphere
from sagetrans.analysis import forward
from sagetrans.analysis.common import Scenario
from sagetrans.control.ardupilot_like import BackTransitionParams, LiftFF, Position1Controller
from sagetrans.io import stages
from sagetrans.io.inputs import load_inputs
from sagetrans.io.settings import FTBoundsCfg, FTCfg, StudyCfg
from sagetrans.physics.params import placeholder_vehicle

ROOT = Path(__file__).resolve().parents[2]
CONFIGS = ROOT / "tests" / "data" / "placeholder_configs"
VP = placeholder_vehicle()


# ---- settings (spec 8.10) -----------------------------------------------------------------------
def test_ft_bounds_must_be_ordered_and_the_throttle_ceiling_positive():
    with pytest.raises(ValidationError, match="airspeed_min_eas_ms"):
        FTBoundsCfg(airspeed_min_eas_ms=(20.0, 14.0))
    with pytest.raises(ValidationError, match="q_transition_s"):
        FTBoundsCfg(q_transition_s=(5.0, 5.0))
    with pytest.raises(ValidationError, match="tkoff_thr_max"):
        FTBoundsCfg(tkoff_thr_max=(0.0, 1.0))
    FTBoundsCfg()  # the shipped defaults are valid


def test_ft_settings_reject_unknown_keys_and_a_reversed_headwind_range():
    with pytest.raises(ValidationError):
        FTCfg(not_a_setting=1)
    with pytest.raises(ValidationError, match="headwind_ms"):
        FTCfg(headwind_ms=[5.0, 0.0])
    with pytest.raises(ValidationError, match="headwind_ms"):
        FTCfg(headwind_ms=[0.0])
    assert FTCfg().stall_margin == 1.2 and FTCfg().weights.w_x == 0.0


def test_ft_is_a_known_analysis_with_the_spec_defaults():
    cfg = StudyCfg()
    assert "FT" in cfg.analyses and cfg.analyses.index("FT") == cfg.analyses.index("Q2") + 1
    assert cfg.FT.bounds.q_tran_pit_max_deg == (1.0, 10.0)
    assert cfg.FT.scan.airspeed_min_eas_ms == [15.0, 16.0, 17.0, 18.0, 19.0, 20.0]
    assert cfg.FT.sens.tecs_start == ["warm", "cold"]


def test_new_mission_fields_load_and_the_unknown_key_check_passes():
    inp = load_inputs(CONFIGS)
    assert inp.mission.fwd_alt_loss_max_m == 10.0 and inp.mission.fwd_alt_gain_max_m == 10.0
    assert inp.mission.clearance_agl_m == 30.0
    for name in ("fwd_alt_loss_max_m", "fwd_alt_gain_max_m"):
        assert inp.status(f"mission.profile.{name}") == "assumed"


def test_new_controller_and_tecs_entries_are_known_register_paths():
    live = load_inputs(ROOT / "configs")
    for path in (
        "controller.ardupilot.Q_TRAN_PIT_MAX", "controller.ardupilot.TRIM_THROTTLE",
        "controller.ardupilot.TKOFF_THR_MIN", "controller.tecs.TECS_VERT_ACC",
        "controller.tecs.TECS_PITCH_MAX",
    ):
        assert path in live.config, path
    assert live.config.record("controller.tecs.TECS_TIME_CONST").si_value == 5.0


# ---- z_loop refactor (spec 4.1, 8.9) ------------------------------------------------------------
def test_z_loop_split_gives_the_same_command_as_the_altitude_error_form():
    sc = Scenario(0.0)
    rho = float(atmosphere.density(0.0))
    ff = LiftFF(VP, rho, sc.soc, t_max_factor=3.0, n=30)
    bp = BackTransitionParams(
        z_mode="ardupilot", h_tgt=10.0, delta_max=0.95, rotor_lag_s=0.1, ff_mode="repo"
    )
    t_max = ff.thrust_for_delta(bp.delta_max)
    ctl = Position1Controller(bp, ff, z_i0=ff.weight / t_max)
    h, vh, theta = 7.5, -0.4, 0.1
    v_t = min(max(bp.z_pos_p * (bp.h_tgt - h), -bp.z_vmax), bp.z_vmax)
    prev: dict[str, float] = {}
    prev2: dict[str, float] = {}
    for k in range(6):  # first call initialises, the rest run the loop; both forms stay equal
        a, prev = ctl._z_loop(0.02 if k else 0.0, h, vh + 0.01 * k, theta, prev)
        b, prev2 = ctl.z_loop(0.02 if k else 0.0, v_t, vh + 0.01 * k, theta, prev2)
        assert a == b and prev == prev2


# ---- the task (spec 6, acceptance 1) -------------------------------------------------------------
FT_TINY = """
FT:
  ensemble_n: 2
  final_ensemble_n: 4
  headwind_ms: [0.0, 5.0]
  popsize: 4
  history_scenarios: [S-01]
  scan: {airspeed_min_eas_ms: [16.0, 18.0], seeds: [1], maxiter: 1}
  grid: {q_transition_s: [4.0, 6.0], fine_offsets_s: [-0.5, 0.5, 2.0], seeds: [1], maxiter: 1,
         descent_max_rounds: 20}
  sens:
    TECS_TIME_CONST: [3.0]
    TECS_INTEG_GAIN: [0.3]
    rotor_lag_s: [0.25]
    z_vel_p: [5.0]
    z_acc_p: [0.03]
    TRIM_THROTTLE: [0.45]
    tecs_start: [warm, cold]
    rec_cl_frac: [0.8]
    CL_max_scale: [0.9, 1.1]
"""


def test_ft_task_writes_every_table_of_the_contract(tmp_path):
    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    with (cfg / "study.yaml").open("a", encoding="utf-8") as fh:
        fh.write(FT_TINY)
    inp = load_inputs(cfg, only={"FT"})
    assert stages.plan_tasks(inp) == ["FT_grid:4000", "FT_grid:6000", "FT_scan:16", "FT_scan:18"]
    out = stages.run_task(inp, "FT:18:5000:100:3")  # the report of a (snapped) winner
    assert out["optimum"] == {"airspeed_min": 18.0, "q_transition": 5.0, "tkoff_thr_max": 1.0,
                              "tran_pit_max": 3.0}
    tables = out["tables"]
    assert set(tables) == {"FT_params", "FT_runs", "FT_sens", "FT_history"}
    params = tables["FT_params"]
    assert [r["case"] for r in params] == ["optimum", "register"]
    for col in ("J", "J_exc", "J_E", "J_pen", "J_start"):
        assert col in params[0]
    assert abs(params[0]["J"] - (params[0]["J_exc"] + params[0]["J_E"] + params[0]["J_x"]
                                 + params[0]["J_pen"])) < 1e-6
    runs = tables["FT_runs"]
    assert {(r["case"], r["set"]) for r in runs} == {
        (c, k) for c in ("optimum", "register") for k in ("core", "corner")
    }
    # the search ensemble is 2; the three cases are reported on the first 4 members (a prefix)
    assert all(r["ensemble_n"] == 4 for r in params)
    members = {r["member"] for r in runs if r["set"] == "core" and r["case"] == "optimum"}
    assert members == {0, 1, 2, 3}
    assert any("first 4 members" in n and "first 2 members" in n for n in out["notes"])
    # FT_sens: a base row on the search ensemble, and the C_L,max cases next to it
    sens = {(r["parameter"], r["value"]): r for r in tables["FT_sens"]}
    base, lo, hi = sens[("base", "")], sens[("CL_max_scale", 0.9)], sens[("CL_max_scale", 1.1)]
    assert base["ensemble_n"] == 2
    assert lo["cl_max"] == pytest.approx(0.9 * base["cl_max"], rel=2e-3)
    assert hi["cl_max"] == pytest.approx(1.1 * base["cl_max"], rel=2e-3)
    # a lower C_L,max is a higher stall speed, so a smaller margin (the flag reads the changed wing)
    assert lo["worst_stall_margin"] < base["worst_stall_margin"] < hi["worst_stall_margin"]
    scan = stages.run_task(inp, "FT_scan:16")["row"]
    assert scan["airspeed_min_eas_ms"] == 16.0 and scan["entry_eas_back_ms"] == 14.0


def test_the_final_ensemble_is_a_prefix_of_a_larger_sample():
    inp = load_inputs(CONFIGS)
    small, _ = stages.ft_scenarios(inp, 4)
    big, _ = stages.ft_scenarios(inp, 8)
    assert [s.mass_kg for s in big[:4]] == [s.mass_kg for s in small]
    with pytest.raises(ValueError, match="final_ensemble_n"):
        FTCfg(ensemble_n=16, final_ensemble_n=8)


def test_cl_max_sensitivity_moves_only_alpha_s():
    import dataclasses

    from sagetrans.analysis import runs

    inp = load_inputs(CONFIGS)
    vp = inp.vehicle
    for scale in (0.9, 1.1):
        v = stages.vehicle_with_cl_max(vp, scale)
        assert runs.cl_max(v.wing) == pytest.approx(scale * runs.cl_max(vp.wing), rel=2e-3)
        assert dataclasses.replace(v.wing, alpha_s=vp.wing.alpha_s) == vp.wing  # only alpha_s
        assert v.m == vp.m and v.rotor == vp.rotor


# ---- the grid, the snap and the lattice (FT_spec Part 4) -----------------------------------------
BOUNDS = {"airspeed_min": (14.0, 20.0), "tkoff_thr_max": (0.5, 1.0), "tran_pit_max": (1.0, 10.0)}


def test_snap_puts_the_values_on_ardupilots_integers_inside_the_bounds():
    from sagetrans.analysis import forward

    s = forward.snap_ft({"airspeed_min": 17.67, "q_transition": 11.99982, "tkoff_thr_max": 0.99982,
                         "tran_pit_max": 2.78}, BOUNDS)
    assert s == {"airspeed_min": 18.0, "q_transition": 12.0, "tkoff_thr_max": 1.0,
                 "tran_pit_max": 3.0}
    assert forward.on_lattice(s)
    # a value just outside a bound rounds back in
    s2 = forward.snap_ft(
        {"airspeed_min": 20.4, "tkoff_thr_max": 0.496, "tran_pit_max": 10.4}, BOUNDS)
    assert (s2["airspeed_min"], s2["tkoff_thr_max"], s2["tran_pit_max"]) == (20.0, 0.5, 10.0)
    assert not forward.on_lattice(
        {"airspeed_min": 17.5, "tkoff_thr_max": 0.8, "tran_pit_max": 3.0})


def test_lattice_neighbours_are_the_in_bound_3x3x3_without_the_centre():
    from sagetrans.analysis import forward

    mid = {"airspeed_min": 17.0, "tkoff_thr_max": 0.8, "tran_pit_max": 5.0}
    assert len(forward.lattice_neighbours(mid, BOUNDS)) == 26
    corner = {"airspeed_min": 20.0, "tkoff_thr_max": 1.0, "tran_pit_max": 10.0}
    nb = forward.lattice_neighbours(corner, BOUNDS)
    assert len(nb) == 7 and all(forward.on_lattice(n) for n in nb)  # 2 x 2 x 2 - 1


def test_lattice_descent_finds_a_point_no_neighbour_beats():
    from sagetrans.analysis import forward

    target = {"airspeed_min": 16.0, "tkoff_thr_max": 0.9, "tran_pit_max": 6.0}

    def bowl(p):  # a convex stand-in with a minimum on the lattice
        return (sum(((p[k] - target[k]) / forward.LATTICE_STEP[k]) ** 2 for k in target)
                + 0.1 * (p["airspeed_min"] - target["airspeed_min"]))  # tilts it, no tie

    res = forward.lattice_descent(
        bowl, {"airspeed_min": 18.4, "tkoff_thr_max": 0.97, "tran_pit_max": 3.2}, BOUNDS)
    assert forward.on_lattice(res.params)
    assert all(bowl(n) >= res.cost for n in forward.lattice_neighbours(res.params, BOUNDS))
    assert res.cost <= res.cost_start and res.evaluations > 1


def test_grid_task_snaps_and_its_optimum_is_not_worse_than_its_lattice_neighbours(tmp_path):
    from sagetrans.analysis import forward

    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    with (cfg / "study.yaml").open("a", encoding="utf-8") as fh:
        fh.write(FT_TINY)
    inp = load_inputs(cfg, only={"FT"})
    out = stages.run_task(inp, "FT_grid:5000")
    row, snapped = out["row"], out["snapped"]
    assert forward.on_lattice(snapped) and snapped["q_transition"] == 5.0
    assert row["J_snapped"] == pytest.approx(
        row["J_continuous"] + row["J_snapped_minus_continuous"])
    assert row["J_snapped"] <= row["J_snap"] + 1e-12  # the descent never makes it worse
    assert {"airspeed_min_eas_ms_continuous", "seed_spread", "t12_ok", "at_bound"} <= set(row)
    # re-evaluate the in-bound subset of the 26 neighbours on the same ensemble and weights
    fev, start = stages.ft_settings(inp)
    core, _ = stages.ft_scenarios(inp)
    w = stages.ft_weights(inp, core, fev, start)
    vp = inp.vehicle

    def cost(p):
        return forward.ft_cost(vp, core, fev, {**p, "q_transition": 5.0}, w)

    assert cost(snapped) == pytest.approx(row["J_snapped"])
    nbs = forward.lattice_neighbours(snapped, stages.ft_lattice_bounds(inp))
    assert nbs and all(cost(n) >= row["J_snapped"] - 1e-9 for n in nbs)


def test_the_fine_pass_and_the_winner_follow_the_grid_results(tmp_path):
    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    with (cfg / "study.yaml").open("a", encoding="utf-8") as fh:
        fh.write(FT_TINY)
    inp = load_inputs(cfg, only={"FT"})

    def fake(q, j_cont, j_snap):
        return {"q_transition_s": q, "row": {"J_continuous": j_cont, "J_snapped": j_snap},
                "snapped": {"airspeed_min": 18.0, "q_transition": q, "tkoff_thr_max": 1.0,
                            "tran_pit_max": 3.0}}

    res = {"FT_grid:4000": fake(4.0, 1.0, 1.1), "FT_grid:6000": fake(6.0, 0.9, 1.0)}
    assert stages.follow_up_tasks(inp, {"FT_grid:4000": res["FT_grid:4000"]}, {}) == []  # not yet
    # the best coarse point is 6 s: 5.5 and 6.5 are new, 8.0 is outside no bound but 2 s is not
    # offered (the offsets are -0.5, 0.5, 2.0 -> 5.5, 6.5, 8.0, all inside 2 to 12)
    assert stages.follow_up_tasks(inp, res, {}) == [
        "FT_grid:5500", "FT_grid:6500", "FT_grid:8000"]
    res["FT_grid:5500"] = fake(5.5, 0.95, 0.8)  # the lowest snapped cost
    res["FT_grid:6500"] = fake(6.5, 1.2, 1.3)
    res["FT_grid:8000"] = fake(8.0, 1.3, 1.4)
    assert stages.grid_winner(res) == "FT_grid:5500"
    assert stages.follow_up_tasks(inp, res, {}) == ["FT:18:5500:100:3"]  # the report of the winner
    res["FT:18:5500:100:3"] = {}
    assert stages.follow_up_tasks(inp, res, {}) == []
    # a failed grid point is not retried and does not hold the later phases back
    part = {k: v for k, v in res.items() if k != "FT_grid:8000" and not k.startswith("FT:")}
    assert stages.follow_up_tasks(inp, part, {"FT_grid:8000": "boom"}) == ["FT:18:5500:100:3"]
    assert stages.ft_task_id(stages.parse_ft_id("18:5500:100:3")) == "FT:18:5500:100:3"


def test_a_fine_offset_outside_the_bounds_is_skipped(tmp_path):
    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    with (cfg / "study.yaml").open("a", encoding="utf-8") as fh:
        fh.write(FT_TINY.replace("q_transition_s: [4.0, 6.0]", "q_transition_s: [19.5, 20.0]")
                 .replace("[-0.5, 0.5, 2.0]", "[-1.0, 0.5, 1.0]"))
    inp = load_inputs(cfg, only={"FT"})
    res = {"FT_grid:19500": {"q_transition_s": 19.5, "row": {"J_continuous": 2.0}},
           "FT_grid:20000": {"q_transition_s": 20.0, "row": {"J_continuous": 1.0}}}
    # the best coarse point is on the 20 s bound: 20.5 and 21 s are not searched (no widening)
    assert stages.grid_fine_ids(inp, res) == ["FT_grid:19000"]


def test_the_shipped_bound_and_grid_reach_20_s_inside_the_ardupilot_range():
    inp = load_inputs(ROOT / "configs", only={"FT"})
    f = inp.study.FT
    assert f.bounds.q_transition_s == (2.0, 20.0)  # ArduPilot allows up to 30 s
    assert {14.0, 16.0, 20.0} <= set(f.grid.q_transition_s) and max(f.grid.q_transition_s) == 20.0
    assert all(f.bounds.q_transition_s[0] <= q <= f.bounds.q_transition_s[1]
               for q in f.grid.q_transition_s)
    assert FTCfg().bounds.q_transition_s == (2.0, 20.0)  # the defaults agree with the file
    assert "FT_grid:20000" in stages.grid_coarse_ids(inp)


def test_the_range_within_two_percent_of_the_best_cost():
    from sagetrans.io.assemble import grid_near_best

    rows = [{"q_transition_s": q, "J_snapped": j}
            for q, j in ((8.0, 0.81), (10.0, 0.79), (12.0, 0.774), (14.0, 0.770), (16.0, 0.772),
                         (20.0, 0.80))]
    assert grid_near_best(rows) == [12.0, 14.0, 16.0]  # <= 0.770 * 1.02 = 0.7854
    assert grid_near_best(rows, 0.0) == [14.0]


# ---- F1: the pitch cap written to FT_history is the case's own -------------------------------
def test_each_case_has_its_own_pitch_cap_by_mode(tmp_path):
    from sagetrans.control.forward import pitch_cap_deg

    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    with (cfg / "study.yaml").open("a", encoding="utf-8") as fh:
        fh.write(FT_TINY)
    inp = load_inputs(cfg, only={"FT"})
    fev, start = stages.ft_settings(inp)
    sc = inp.scenarios["S-01"]
    opt = {**start, "tran_pit_max": 6.0}
    fp_opt, fp_reg = forward.case_params(fev, sc, opt), forward.case_params(fev, sc, start)
    assert fp_opt.tran_pit_max_deg == 6.0 and fp_reg.tran_pit_max_deg == start["tran_pit_max"]
    assert fev.template.tran_pit_max_deg == start["tran_pit_max"]  # the template is the register's
    # WAIT cap = Q_TRAN_PIT_MAX, FADE cap = (Q_TRAN_PIT_MAX + 1) x 2, DONE has none
    assert pitch_cap_deg(fp_opt, "fwd_wait", 5.0) == 6.0
    assert pitch_cap_deg(fp_opt, "fwd_fade", 5.0) == 14.0
    assert pitch_cap_deg(fp_reg, "fwd_wait", 5.0) == start["tran_pit_max"]
    assert pitch_cap_deg(fp_reg, "fwd_fade", 5.0) == (start["tran_pit_max"] + 1.0) * 2.0


def test_the_ft_report_writes_each_cases_own_cap_and_the_run_flew_with_it(tmp_path):
    cfg = tmp_path / "configs"
    shutil.copytree(CONFIGS, cfg)
    with (cfg / "study.yaml").open("a", encoding="utf-8") as fh:
        fh.write(FT_TINY)
    inp = load_inputs(cfg, only={"FT"})
    out = stages.run_task(inp, "FT:18:5000:100:6")  # an optimum with Q_TRAN_PIT_MAX 6, register 3
    reg_pit = round(float(stages.ft_settings(inp)[1]["tran_pit_max"]), 6)
    hist = out["tables"]["FT_history"]
    caps = {}
    for r in hist:
        caps.setdefault((r["case"], r["mode"]), set()).add(round(r["theta_cap"], 6))
    wait_o, wait_r = caps[("optimum", "fwd_wait")], caps[("register", "fwd_wait")]
    assert 6.0 in wait_o and 6.0 not in wait_r  # a wait-mode cap of 6 is the optimum's alone
    assert reg_pit in wait_r and reg_pit not in wait_o - {0.0}
    fade_o, fade_r = caps[("optimum", "fwd_fade")], caps[("register", "fwd_fade")]
    assert fade_o == {14.0} and fade_r == {round((reg_pit + 1.0) * 2.0, 6)}  # 14 against 8
    assert fade_o != fade_r
    # t_pitch_cap_s in FT_runs is computed in _evaluate from the run's own `fp`, so it already used
    # each run's parameters; the defect was the history rows only
    assert all("t_pitch_cap_s" in r for r in out["tables"]["FT_runs"])
