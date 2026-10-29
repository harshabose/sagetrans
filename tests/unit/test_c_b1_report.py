"""The C and B1 chapters of the results viewer: scenarios, margins, spreads, sensitivity."""

import csv
import json
import math
import re
from pathlib import Path

import numpy as np

from sagetrans.io import stages
from sagetrans.io.view import charts as ch
from sagetrans.io.view.chapters import (
    history_panels,
    scenario_caption,
    select_history,
)
from sagetrans.io.view.load import load
from sagetrans.io.view.registry import build_html


def write_table(folder: Path, name: str, rows: list[dict]) -> None:
    cols: list[str] = []
    for r in rows:
        cols += [c for c in r if c not in cols]
    with (folder / f"{name}.csv").open("w", newline="", encoding="utf-8") as fh:
        w = csv.DictWriter(fh, fieldnames=cols, lineterminator="\n")
        w.writeheader()
        w.writerows(rows)


def write_record(folder: Path, **extra) -> None:
    rec = {"label": "DRAFT", "reasons": ["72 register entries are TBD"], "config_hash": "ab" * 32,
           "git_commit": "deadbee", "git_dirty": False, "date": "2026-10-29", "seeds": [1, 2],
           "outputs": {}, "analyses": {}, "search_spreads": {}, "schema_version": 2, **extra}
    (folder / "run_record.json").write_text(json.dumps(rec), encoding="utf-8")


def section(html: str, chapter: str) -> str:
    return html.split(f'id="{chapter}"')[1].split("</section>")[0]


# ---- a synthetic run: C on two scenarios and two members, B1 on two scenarios -----------------
def hist_rows(case, view, scenario, member, v0=15.0, n=6, **extra):
    meta = {"view": view, "scenario": scenario, "altitude_m": 0.0, "isa_offset_k": 0.0, "soc": 0.8,
            "headwind_ms": 0.0, "mass_kg": 26.7, "V0_tas_ms": v0, "V0_eas_ms": v0 * 0.99,
            "band_eps_h_m": 1.0, "run_overshoot_m": 1.5 if "default" not in case else 3.1}
    return [{"case": case, "member": member, **meta, **extra, "t": 0.5 * i, "x": 10.0 * i,
             "h": 0.1 * i, "V": v0 - 2.0 * i, "v_h": 0.0, "theta": 4.0 * i, "alpha_w_deg": 3.0 * i,
             "delta_r": 0.05 * i, "theta_cmd": 4.0 * i, "omega_r": 100.0}
            for i in range(n)]


def b1_hist(variant, scenario, v0=15.0, eps_x=60.0):
    return [{"case": f"B1 {variant}", "scenario": scenario, "eps_x_m": eps_x, "t": 0.5 * i,
             "x": 9.0 * i, "h": -0.05 * i, "V": v0 - 2.2 * i, "v_h": 0.0, "theta": 5.0 * i,
             "alpha_w_deg": 3.0 * i, "delta_r": 0.04 * i, "theta_cmd": 5.0 * i, "omega_r": 90.0}
            for i in range(6)]


def synthetic_run(folder: Path, late_ok: bool = False) -> None:
    write_table(folder, "C_params", [
        {"case": "optimum", "T_bt_s": 1.6, "theta_A_deg": 19.6, "spin_min": 0.11,
         "Q_TRANS_DECEL_ms2": 2.03, "J": 1.48, "p95_distance_m": 119.0, "mean_energy_kJ": 20.0},
        {"case": "ardupilot_default", "T_bt_s": 3.0, "theta_A_deg": 30.0, "spin_min": 0.15,
         "Q_TRANS_DECEL_ms2": 2.0, "J": 61.5, "p95_distance_m": 121.0, "mean_energy_kJ": 20.3}])
    c_hist = (hist_rows("C optimum", "core", "S-01", -1)
              + hist_rows("C ardupilot_default", "core", "S-01", -1, v0=15.0)
              + hist_rows("C optimum", "core", "S-03", -1, v0=19.0)  # another scenario
              + hist_rows("C optimum", "default_fails", "ensemble member", 14, v0=19.3,
                          default_fails=True)
              + hist_rows("C ardupilot_default", "default_fails", "ensemble member", 14, v0=19.3,
                          default_fails=True)
              + hist_rows("C optimum", "default_fails", "ensemble member", 2, v0=17.0,
                          default_fails=True))  # another member
    write_table(folder, "C_history", c_hist)
    write_table(folder, "B1_history", [*b1_hist("attached", "S-01"), *b1_hist("c_envelope", "S-01"),
                                       *b1_hist("attached", "S-03", v0=19.0)])
    write_table(folder, "B1_front", [
        {"scenario": "S-01", "variant": "attached", "eps_x_m": 60.0, "eps_h_m": 1.0,
         "converged": True, "replay_verified": True, "replay_late_command_ok": True,
         "energy_kJ": 11.3, "distance_m": 60.0, "V0_air_ms": 15.0},
        {"scenario": "S-01", "variant": "c_envelope", "eps_x_m": 60.0, "eps_h_m": 1.0,
         "converged": True, "replay_verified": True, "replay_late_command_ok": late_ok,
         "energy_kJ": 10.5, "distance_m": 60.0, "V0_air_ms": 15.0}])
    write_record(folder)


def numeric(rows):
    return [{k: (float(v) if re.fullmatch(r"-?[0-9.]+", v or "x") else v) for k, v in r.items()}
            for r in rows]


# ---- C1 ---------------------------------------------------------------------------------------
def test_a_history_figure_never_mixes_scenarios_or_members(tmp_path):
    synthetic_run(tmp_path)
    c_rows = numeric(csv.DictReader((tmp_path / "C_history.csv").open(encoding="utf-8")))
    b1_rows = list(csv.DictReader((tmp_path / "B1_history.csv").open(encoding="utf-8")))
    core, ident = select_history(c_rows, b1_rows, "core", "S-01")
    assert ident == ("scenario", "S-01")
    assert set(core) == {"C optimum", "C ardupilot_default", "B1 attached", "B1 c_envelope"}
    assert {r["scenario"] for rows in core.values() for r in rows} == {"S-01"}  # not S-03
    worst, ident = select_history(c_rows, b1_rows, "default_fails")
    assert ident == ("member", 14.0) and set(worst) == {"C optimum", "C ardupilot_default"}
    assert {r["member"] for rows in worst.values() for r in rows} == {14.0}  # not member 2
    assert not any(k.startswith("B1") for k in worst)  # no B1 curves in the second figure


def test_the_c_chapter_has_two_history_figures_with_a_stated_scenario(tmp_path):
    synthetic_run(tmp_path, late_ok=False)
    c = section(build_html(load(tmp_path)), "c")
    assert "Same scenario as B1 (S-01)" in c
    assert "Where the default fails (ensemble member 14)" in c
    assert "Member 0 of the ensemble" not in c  # the old single figure is gone
    # the caption states altitude, ISA, mass, SoC, headwind and the entry speed, true and equivalent
    for text in ("altitude 0 m", "ISA +0.0 K", "mass 26.7 kg", "SoC 0.80", "headwind +0.00 m/s",
                 "entry airspeed 15.0 m/s true, 14.8 m/s equivalent"):
        assert text in c, text
    assert "C optimum (15.0 m/s)" in c and "B1 attached (15.0 m/s)" in c  # entry speed in legend
    assert "open-loop bounds that use the altitude band, not controllers" in c
    assert "c_envelope is verified for exact timing only" in c  # from replay_late_command_ok
    assert "stops at" in c  # an end marker on every curve
    assert "+1 m" in c and "-1 m" in c  # the B1 altitude band as two lines
    assert "overshoots this member by 3.10 m" in c  # the default's overshoot on that member


def test_the_b1_note_follows_the_flag_not_a_fixed_text(tmp_path):
    synthetic_run(tmp_path, late_ok=True)
    c = section(build_html(load(tmp_path)), "c")
    assert "c_envelope also replays with commands 10 ms late" in c
    assert "verified for exact timing only" not in c


def test_the_airspeed_axis_starts_at_zero_and_a_run_ends_with_a_marker():
    t = list(range(6))
    s = ch.TSeries("a", "s1", "a", [float(x) for x in t],
                   {"V": [15.0 - 2.0 * x for x in t], "h": [0.0] * 6, "theta": [0.0] * 6,
                    "delta_r": [0.0] * 6}, end_mark=True)
    svg = ch.panel_figure("f", history_panels(1.0), [s])
    v_panel = svg.split('data-pi="0"')[1].split("</svg>")[0]
    ticks = re.findall(r'class="tick"[^>]*>([^<]*)<', v_panel)
    assert "0" in ticks and all(not tk.startswith("-") for tk in ticks)  # not -10
    assert svg.count("stops at") == 4  # one marker per panel


def test_scenario_caption_skips_what_is_missing():
    assert scenario_caption({"altitude_m": 500.0}) == "altitude 500 m"
    assert "entry airspeed" not in scenario_caption({"V0_tas_ms": 15.0})


def test_the_history_rows_end_at_the_stop():
    assert stages._sample_indices(12, 5) == [0, 5, 10, 11]
    assert stages._sample_indices(11, 5) == [0, 5, 10]
    assert stages._sample_indices(1, 5) == [0]

    class Sim:
        t = np.array([0.0, 0.1, 0.2, 0.3, 0.4, 0.5, 0.6])

        def __getitem__(self, k):
            return np.zeros(len(self.t))

    rows = stages._c_trace(Sim(), 0.0, "C optimum", -1, meta={"scenario": "S-01"})
    assert [r["t"] for r in rows] == [0.0, 0.5, 0.6] and rows[-1]["scenario"] == "S-01"
    assert math.isfinite(rows[0]["V"])
