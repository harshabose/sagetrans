# Sage VTOL transition study — study (DRAFT)

`run record 332d5dcc | DRAFT | commit 0e6e606 | 2026-10-29`

> **DRAFT.** Not for release. 72 register entries are TBD: controller.altitude_hold.K_h, controller.altitude_hold.K_i, controller.altitude_hold.K_vh, controller.altitude_hold.i_max, controller.altitude_hold.v_dem_max, controller.ardupilot.AIRSPEED_MIN, ... (72 in all); analysis 'E:0m_round_trip' ran on placeholder inputs (record is DRAFT); analysis 'E:0m_one_way' ran on placeholder inputs (record is DRAFT); analysis 'E:1500m_round_trip' ran on placeholder inputs (record is DRAFT); analysis 'E:1500m_one_way' ran on placeholder inputs (record is DRAFT); analysis 'E:2500m_round_trip' ran on placeholder inputs (record is DRAFT); analysis 'E:2500m_one_way' ran on placeholder inputs (record is DRAFT); analysis 'E:5000m_round_trip' ran on placeholder inputs (record is DRAFT); analysis 'E:5000m_one_way' ran on placeholder inputs (record is DRAFT); analysis 'FT' ran on placeholder inputs (record is DRAFT); analysis 'C' ran on placeholder inputs (record is DRAFT); analysis 'sensitivity' ran on placeholder inputs (record is DRAFT).
> The DRAFT refusal was explicitly overridden.

## 1. Summary

| Output | Nominal | Weakest grade among dominating inputs |
| --- | --- | --- |
| excursion_m | 0.4285 m | assumed |
| distance_m | 82.58 m | assumed |
| energy_J | 1.799e+04 J | assumed |
| max_range_km | 91.48 km | assumed |

Recommended Q_TRANS_DECEL (analysis D): 1.65 m/s^2.

Range margin (analysis E, reserves included):

| Range | Total [Wh] | Margin [Wh] | Fits |
| --- | --- | --- | --- |
| 100 km | 3553 | -1777 | NO |
| 120 km | 4208 | -2432 | NO |

Maximum range: 46 km.
**Range requirement and usable energy conflict at: 100 km, 120 km.**

## 2. Evidence grade

Inputs in the parameter register by grade:

| Grade | Count |
| --- | --- |
| measured | 0 |
| datasheet | 9 |
| derived | 19 |
| documented | 16 |
| stated | 6 |
| assumed | 21 |
| TBD | 72 |

Assumed or still unknown: `controller.altitude_hold.K_h`, `controller.altitude_hold.K_i`, `controller.altitude_hold.K_vh`, `controller.altitude_hold.i_max`, `controller.altitude_hold.v_dem_max`, `controller.ardupilot.AIRSPEED_MIN`, `controller.ardupilot.PTCH_LIM_MAX_DEG`, `controller.ardupilot.PTCH_LIM_MIN_DEG`, `controller.ardupilot.Q_A_ANGLE_MAX`, `controller.ardupilot.Q_BACKTRANS_MS`, `controller.ardupilot.Q_M_SPIN_MAX`, `controller.ardupilot.Q_M_SPIN_MIN`, `controller.ardupilot.Q_TRAN_PIT_MAX`, `controller.ardupilot.THR_MAX`, `controller.ardupilot.THR_MIN`, `controller.ardupilot.THR_SLEWRATE`, `controller.ardupilot.TKOFF_THR_MAX`, `controller.ardupilot.TKOFF_THR_MIN`, `controller.ardupilot.TRIM_THROTTLE`, `controller.cruise_stub.K_hd`, `controller.cruise_stub.K_hp`, `controller.cruise_stub.K_s`, `controller.cruise_stub.K_si`, `controller.cruise_stub.r_tecs`, `controller.cruise_stub.tau_h`, `controller.cruise_stub.theta_cruise_max`, `controller.cruise_stub.v_cruise`, `controller.position.K_p`, `controller.position.K_v`, `controller.position.a_lim`, `controller.tecs.TECS_CLMB_MAX`, `controller.tecs.TECS_INTEG_GAIN`, `controller.tecs.TECS_PITCH_MAX`, `controller.tecs.TECS_PTCH_DAMP`, `controller.tecs.TECS_SINK_MAX`, `controller.tecs.TECS_SINK_MIN`, `controller.tecs.TECS_THR_DAMP`, `controller.tecs.TECS_TIME_CONST`, `controller.tecs.TECS_VERT_ACC`, `mission.battery_window.eta_usable`, `mission.battery_window.soc_eod`, `mission.battery_window.soc_start`, `mission.battery_window.soh`, `mission.battery_window.temp_factor`, `mission.descent.h_slow`, `mission.descent.v_final`, `mission.descent.v_fragility`, `mission.profile.a_fwd`, `mission.profile.clearance_agl_m`, `mission.profile.climb_height_m`, `mission.profile.d_diversion_m`, `mission.profile.fwd_alt_gain_max_m`, `mission.profile.fwd_alt_loss_max_m`, `mission.profile.h_bt_agl_m`, `mission.profile.isa_offset_k`, `mission.profile.n_diversion_transitions`, `mission.profile.payload_kg`, `mission.profile.roc`, `mission.profile.soc_mean`, `mission.profile.t_drop_s`, `mission.profile.t_go_around`, `mission.profile.t_hold`, `mission.profile.t_hover_pre`, `mission.profile.v_cruise`, `mission.profile.v_cruise_ref_amsl_m`, `mission.profile.vz_takeoff`, `vehicle.attitude.omega_n`, `vehicle.attitude.q_max`, `vehicle.attitude.zeta`, `vehicle.battery.C1_pack`, `vehicle.battery.R0_pack`, `vehicle.battery.R1_pack`, `vehicle.battery.V_min`, `vehicle.motor_lift.dead_zone`, `vehicle.motor_pusher.dead_zone`, `vehicle.pusher.D`, `vehicle.pusher.I_p`, `vehicle.pusher.omega_floor`, `vehicle.rotor.CH1`, `vehicle.rotor.CQ1`, `vehicle.rotor.CQ2`, `vehicle.rotor.CT1`, `vehicle.rotor.CT2`, `vehicle.rotor.I_rot`, `vehicle.rotor.J_max`, `vehicle.rotor.J_min`, `vehicle.rotor.k_mu`, `vehicle.rotor.mu_max`, `vehicle.rotor.omega_floor`, `vehicle.wing.CD0`, `vehicle.wing.CN`, `vehicle.wing.M`, `vehicle.wing.alpha_s`.

## 3. Results by analysis

### A — zero-altitude-loss feasibility

An idealised quasi-steady scan, not a flight simulation: a constant deceleration from the entry speed (equivalent airspeed) to rest, level flight, no wind, pusher off, rotors already spinning. The wing's angle of attack must stay inside a band (see `study.yaml`, `A:`), so a profile that needs negative angle of attack or one past the lift limit is infeasible. The band's upper edge rests on a provisional C_L,max. `vrs_min` / `vrs_lo` / `vrs_hi` give the rotor V_c / v_h ratio (negative = flow up through the discs); it is reported only, unless `vrs_enforce` is set in `study.yaml`.

**S-01**

| V0_eas | a=0.2 | a=0.4 | a=0.6 | a=0.8 | a=1 | a=1.2 | a=1.4 | a=1.6 | a=1.8 | a=2 | a=2.2 | a=2.4 | a=2.6 | a=2.8 | a=3 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | no | feasible | feasible | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no |
| 12 | no | feasible | feasible | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no |
| 14 | no | no | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 16 | no | no | no | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 18 | no | no | no | no | feasible | no | no | no | no | no | no | no | no | no | no |
| 20.9 | no | no | no | no | no | no | no | no | no | no | no | no | no | no | no |

**S-03**

| V0_eas | a=0.2 | a=0.4 | a=0.6 | a=0.8 | a=1 | a=1.2 | a=1.4 | a=1.6 | a=1.8 | a=2 | a=2.2 | a=2.4 | a=2.6 | a=2.8 | a=3 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | no | feasible | feasible | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no |
| 12 | no | feasible | feasible | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no |
| 14 | no | no | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 16 | no | no | no | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 18 | no | no | no | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 20.9 | no | no | no | no | no | no | no | no | no | no | no | no | no | no | no |

**S-06**

| V0_eas | a=0.2 | a=0.4 | a=0.6 | a=0.8 | a=1 | a=1.2 | a=1.4 | a=1.6 | a=1.8 | a=2 | a=2.2 | a=2.4 | a=2.6 | a=2.8 | a=3 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | no | feasible | feasible | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no |
| 12 | no | no | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 14 | no | no | no | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 16 | no | no | no | no | feasible | no | no | no | no | no | no | no | no | no | no |
| 18 | no | no | no | no | no | no | no | no | no | no | no | no | no | no | no |
| 20.9 | no | no | no | no | no | no | no | no | no | no | no | no | no | no | no |

**S-07**

| V0_eas | a=0.2 | a=0.4 | a=0.6 | a=0.8 | a=1 | a=1.2 | a=1.4 | a=1.6 | a=1.8 | a=2 | a=2.2 | a=2.4 | a=2.6 | a=2.8 | a=3 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 10 | no | feasible | feasible | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no |
| 12 | no | feasible | feasible | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no |
| 14 | no | no | feasible | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 16 | no | no | no | feasible | feasible | no | no | no | no | no | no | no | no | no | no |
| 18 | no | no | no | no | feasible | no | no | no | no | no | no | no | no | no | no |
| 20.9 | no | no | no | no | no | no | no | no | no | no | no | no | no | no | no |

Feasible constant deceleration for a whole path from the entry speed to rest (`islands` = the feasible set has gaps; the span is still reported):

| scenario | altitude_m | V0_eas_ms | min_feasible_decel_ms2 | max_feasible_decel_ms2 | islands |
| --- | --- | --- | --- | --- | --- |
| S-01 | 0 | 10 | 0.27 | 1.34 | no |
| S-01 | 0 | 12 | 0.389 | 1.23 | no |
| S-01 | 0 | 14 | 0.529 | 1.12 | no |
| S-01 | 0 | 16 | 0.692 | 1.08 | no |
| S-01 | 0 | 18 | 0.875 | 1.08 | no |
| S-01 | 0 | 20.9 |  | 0 |  |
| S-03 | 5000 | 10 | 0.235 | 1.37 | no |
| S-03 | 5000 | 12 | 0.338 | 1.28 | no |
| S-03 | 5000 | 14 | 0.461 | 1.17 | no |
| S-03 | 5000 | 16 | 0.601 | 1.09 | no |
| S-03 | 5000 | 18 | 0.761 | 1.08 | no |
| S-03 | 5000 | 20.9 | 1.03 | 1.08 | no |
| S-06 | 5000 | 10 | 0.332 | 1.28 | no |
| S-06 | 5000 | 12 | 0.479 | 1.16 | no |
| S-06 | 5000 | 14 | 0.652 | 1.08 | no |
| S-06 | 5000 | 16 | 0.851 | 1.08 | no |
| S-06 | 5000 | 18 |  | 0 |  |
| S-06 | 5000 | 20.9 |  | 0 |  |
| S-07 | 1500 | 10 | 0.27 | 1.34 | no |
| S-07 | 1500 | 12 | 0.389 | 1.23 | no |
| S-07 | 1500 | 14 | 0.529 | 1.12 | no |
| S-07 | 1500 | 16 | 0.692 | 1.08 | no |
| S-07 | 1500 | 18 | 0.875 | 1.08 | no |
| S-07 | 1500 | 20.9 |  | 0 |  |

Feasible window at each speed (a steady-rotor check, pointwise in speed; `bind_lo` and `bind_hi` list every limit violated just outside each edge; a row without `a_lo` means no deceleration is feasible at that speed and `bind_lo` names what fails closest to feasible):

| scenario | altitude_m | V0_eas_ms | V0_tas_ms | a_lo | a_hi | bind_lo | bind_hi | theta_lo_deg | theta_hi_deg | vrs_lo | vrs_hi |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S-01 | 0 | 10 | 10 | 0.27 | 1.34 | alpha_low | alpha_high | 3.09e-05 | 9.12 | -7.65e-07 | -0.274 |
| S-01 | 0 | 12 | 12 | 0.389 | 1.23 | alpha_low | alpha_high | 0.000152 | 9.12 | -4.69e-06 | -0.406 |
| S-01 | 0 | 14 | 14 | 0.529 | 1.12 | alpha_low | thrust_sign;alpha_high | 0.000234 | 8.32 | -8.9e-06 | -0.602 |
| S-01 | 0 | 16 | 16 | 0.692 | 1.08 | alpha_low | thrust_sign;alpha_high | 0.000272 | 5.91 | -1.27e-05 | -0.581 |
| S-01 | 0 | 18 | 18 | 0.875 | 1.14 | thrust_sign | thrust_sign;alpha_high | 0.000259 | 4.29 | -1.49e-05 | -0.652 |
| S-01 | 0 | 20.9 | 20.9 | 1.18 | 1.32 | thrust_sign | thrust_sign | 4.77e-05 | 2.46 | -3.91e-06 | -6.11 |
| S-03 | 5000 | 10 | 13.2 | 0.235 | 1.37 | alpha_low | alpha_high | 1.57e-05 | 9.12 | -3.59e-07 | -0.244 |
| S-03 | 5000 | 12 | 15.8 | 0.338 | 1.28 | alpha_low | alpha_high | 9.57e-05 | 9.11 | -2.71e-06 | -0.343 |
| S-03 | 5000 | 14 | 18.4 | 0.46 | 1.17 | alpha_low | alpha_high | 0.000174 | 9.11 | -6.02e-06 | -0.533 |
| S-03 | 5000 | 16 | 21 | 0.601 | 1.09 | alpha_low | thrust_sign;alpha_high | 0.000253 | 7.07 | -1.06e-05 | -0.582 |
| S-03 | 5000 | 18 | 23.7 | 0.761 | 1.1 | alpha_low | thrust_sign;alpha_high | 2.63e-05 | 5.2 | -1.33e-06 | -0.591 |
| S-03 | 5000 | 20.9 | 27.5 | 1.03 | 1.22 | thrust_sign | thrust_sign;alpha_high | 0.000311 | 3.38 | -2.12e-05 | -1.03 |
| S-06 | 5000 | 10 | 12.9 | 0.332 | 1.28 | alpha_low | alpha_high | 9.82e-05 | 9.11 | -2.76e-06 | -0.336 |
| S-06 | 5000 | 12 | 15.5 | 0.479 | 1.16 | alpha_low | alpha_high | 0.000193 | 9.11 | -6.87e-06 | -0.578 |
| S-06 | 5000 | 14 | 18.1 | 0.652 | 1.08 | alpha_low | thrust_sign;alpha_high | 5.53e-05 | 6.41 | -2.46e-06 | -0.585 |
| S-06 | 5000 | 16 | 20.6 | 0.851 | 1.13 | thrust_sign | thrust_sign;alpha_high | 0.000262 | 4.45 | -1.47e-05 | -0.627 |
| S-06 | 5000 | 18 | 23.2 | 1.08 | 1.25 | thrust_sign | thrust_sign;alpha_high | 0.000214 | 3.15 | -1.55e-05 | -3.28 |
| S-06 | 5000 | 20.9 | 27 | 1.45 | 1.52 | thrust_sign | thrust_sign | 0.000344 | 1.1 | -4.23e-05 | -4.33 |
| S-07 | 1500 | 10 | 10.8 | 0.27 | 1.34 | alpha_low | alpha_high | 3.09e-05 | 9.12 | -7.65e-07 | -0.274 |
| S-07 | 1500 | 12 | 12.9 | 0.389 | 1.23 | alpha_low | alpha_high | 0.000152 | 9.12 | -4.69e-06 | -0.406 |
| S-07 | 1500 | 14 | 15.1 | 0.529 | 1.12 | alpha_low | thrust_sign;alpha_high | 0.000234 | 8.32 | -8.9e-06 | -0.602 |
| S-07 | 1500 | 16 | 17.2 | 0.692 | 1.08 | alpha_low | thrust_sign;alpha_high | 0.000272 | 5.91 | -1.27e-05 | -0.581 |
| S-07 | 1500 | 18 | 19.4 | 0.875 | 1.14 | thrust_sign | thrust_sign;alpha_high | 0.000259 | 4.29 | -1.49e-05 | -0.652 |
| S-07 | 1500 | 20.9 | 22.5 | 1.18 | 1.32 | thrust_sign | thrust_sign | 4.77e-05 | 2.46 | -3.91e-06 | -6.11 |


### D — braking and Q_TRANS_DECEL

Entry is the AIRBRAKE exit: the rotors already carry what the wing does not, the pusher is off, and the pitch limit ramps from the last fixed-wing pitch to the cap over `Q_BACKTRANS_MS`. The entry air speed comes from the exit rule `AIRSPEED_MIN - 2`; the ground speed is that minus the headwind.

| scenario | mass_kg | V0_air_ms | V0_ground_ms | a_eq_ms2 | x_actual_m | alt_gain_m | alt_loss_m | alpha_max_gated_deg | t_above_alpha_s_s | vh_end_ms | sinking |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S-01 | 21.3 | 15 | 15 | 1.97 | 56.8 | 0.505 | 0.391 | 24.6 | 1.9 | -0.219 | no |
| S-01 | 26.7 | 15 | 15 | 2 | 56.1 | 0.225 | 0.367 | 22 | 2.02 | 0.0041 | no |
| S-01 | 30.7 | 15 | 15 | 1.99 | 56.2 | 0.185 | 0.384 | 21.9 | 2.04 | -0.0144 | no |
| S-02 | 21.3 | 19.4 | 19.4 | 2.13 | 87.8 | 1.39 | 1.21 | 31.1 | 2.98 | -0.506 | yes |
| S-02 | 26.7 | 19.4 | 19.4 | 2.2 | 84.8 | 0.302 | 0.497 | 21.6 | 3.2 | 0.0852 | no |
| S-02 | 30.7 | 19.4 | 19.4 | 2.2 | 84.8 | 0.243 | 0.525 | 21.5 | 3.2 | 0.0621 | no |
| S-03 | 21.3 | 19.7 | 19.7 | 2.14 | 90.8 | 1.51 | 1.33 | 32 | 3.08 | -0.607 | yes |
| S-03 | 26.7 | 19.7 | 19.7 | 2.22 | 87.5 | 0.314 | 0.567 | 21.7 | 3.28 | 0.0638 | no |
| S-03 | 30.7 | 19.7 | 19.7 | 2.22 | 87.6 | 0.252 | 0.612 | 21.7 | 3.3 | 0.0362 | no |
| S-05 | 21.3 | 15.4 | 10.4 | 1.67 | 31.9 | 0.556 | 6.21e-05 | 24.2 | 1.7 | -0.442 | no |
| S-05 | 26.7 | 15.4 | 10.4 | 1.69 | 31.5 | 0.232 | 0.241 | 21.9 | 1.84 | -0.203 | no |
| S-05 | 30.7 | 15.4 | 10.4 | 1.68 | 31.6 | 0.19 | 0.251 | 21.9 | 1.86 | -0.196 | no |
| S-06 | 21.3 | 19.4 | 19.4 | 2.13 | 87.8 | 1.42 | 1.22 | 31.6 | 2.98 | -0.648 | yes |
| S-06 | 26.7 | 19.4 | 19.4 | 2.2 | 84.9 | 0.312 | 0.6 | 21.9 | 3.18 | 0.0314 | no |
| S-06 | 30.7 | 19.4 | 19.4 | 2.2 | 84.9 | 0.25 | 0.659 | 21.9 | 3.2 | 0.0013 | no |
| S-07 | 21.3 | 16.1 | 16.1 | 2.02 | 64.4 | 0.682 | 0.518 | 26 | 2.18 | -0.31 | no |
| S-07 | 26.7 | 16.1 | 16.1 | 2.06 | 63.1 | 0.246 | 0.399 | 21.9 | 2.34 | 0.024 | no |
| S-07 | 30.7 | 16.1 | 16.1 | 2.05 | 63.2 | 0.201 | 0.42 | 21.8 | 2.34 | 0.00393 | no |

**Read the angle-of-attack columns first.** With the altitude held the wing angle of attack equals the pitch, so the pitch ramp is a wing angle of attack well above the stall angle for most of the braking, and the rotors carry the weight. `a_eq` here is the most the idealised controller can do in that state, not validated hardware performance: the lift and drag beyond the stall angle are a flat-plate placeholder. Setting `D.enforce_alpha` holds the angle to the top of A's band and shows the other bound (see `D_sensitivity`). `D.ff_mode: lift_aware` is an oracle at its defaults (it reads the simulated lift). Altitude excursions are lower bounds: ideal actuators, assumed gains, no gusts. `D.theta_fw0_deg` is a placeholder until a SITL run or flight log gives the pitch at the AIRBRAKE exit.

**Q_TRANS_DECEL recommendation** (32 Sobol runs, seed 8, 8 corners, entry `eas`): **1.78 m/s²** at the 5th percentile (bootstrap 90% interval 1.74 to 1.92); the minimum over sample and corners is **1.65 m/s²** (set by alt 0 m, mass 21.3 kg, headwind +5.0 m/s, ISA +5.0 K, SoC 0.60). 0 of 32 sample entries are below their own 1g stall speed (flagged, not dropped).

These single numbers are valid only inside the ground speeds they cover: ground speeds 10.1 to 24.5 m/s (air speed 15.1 to 19.6, headwind -5.0 to +5.0). The curve below gives the recommendation at other ground speeds.

| n | seed | n_corners | convention | percentile | a_rec_min_ms2 | a_rec_pct_ms2 | a_rec_pct_ci_lo_ms2 | a_rec_pct_ci_hi_ms2 | a_rec_pct_half_ms2 | n_below_vs | n_nan | min_set_by | worst_corner | spearman_altitude | spearman_isa | spearman_mass | spearman_headwind | spearman_soc | ground_speed_min_ms | ground_speed_max_ms | headwind_min_ms | headwind_max_ms | covers | a_rec_design_ms2 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 32 | 8 | 8 | eas | 5 | 1.65 | 1.78 | 1.74 | 1.92 | 1.88 | 0 | 0 | alt 0 m, mass 21.3 kg, headwind +5.0 m/s, ISA +5.0 K, SoC 0.60 | alt 0 m, mass 21.3 kg, headwind +5.0 m/s, ISA +5.0 K, SoC 0.60 | 0.309 | 0.0733 | 0.0235 | -0.927 | 0.0363 | 10.1 | 24.5 | -5 | 5 | ground speeds 10.1 to 24.5 m/s (air speed 15.1 to 19.6, headwind -5.0 to +5.0) |  |

**`a_rec` as a function of the ground speed at the AIRBRAKE exit** (the minimum over the named scenarios x mass factors; the air speed is the entry rule's, so the headwind is the air speed minus the ground speed; read `Q_TRANS_DECEL` at the slowest ground speed that must be covered):

| ground_speed_ms | n_runs | n_nan | headwind_ms | V0_air_ms | a_rec_min_ms2 | set_by | a_eq_median_ms2 | a_eq_max_ms2 | x_at_min_m | t_stop_at_min_s | excursion_max_m | n_sinking | note |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 5 | 18 | 0 | 14.7 | 19.7 | 1.14 | S-03 x1.15 | 1.16 | 1.19 | 10.5 | 2.98 | 1.24 | 0 |  |
| 6 | 18 | 0 | 13.7 | 19.7 | 1.26 | S-03 x1.15 | 1.27 | 1.28 | 13.9 | 3.26 | 1.46 | 0 |  |
| 8 | 18 | 0 | 11.7 | 19.7 | 1.45 | S-03 x0.8 | 1.47 | 1.48 | 21.7 | 3.99 | 1.51 | 0 |  |
| 10 | 18 | 0 | 9.73 | 19.7 | 1.61 | S-03 x0.8 | 1.65 | 1.66 | 30.8 | 4.55 | 1.51 | 4 |  |
| 12 | 18 | 0 | 7.73 | 19.7 | 1.74 | S-03 x0.8 | 1.8 | 1.81 | 41 | 5.12 | 1.51 | 4 |  |
| 14 | 18 | 0 | 5.73 | 19.7 | 1.86 | S-03 x0.8 | 1.93 | 1.94 | 52.3 | 5.67 | 1.51 | 3 |  |
| 15 | 18 | 0 | 4.73 | 19.7 | 1.92 | S-03 x0.8 | 1.99 | 2 | 58.4 | 5.95 | 1.51 | 3 |  |
| 16 | 18 | 0 | 3.73 | 19.7 | 1.97 | S-03 x0.8 | 2.04 | 2.05 | 64.8 | 6.22 | 1.51 | 3 |  |
| 18 | 18 | 0 | 1.73 | 19.7 | 2.06 | S-03 x0.8 | 2.14 | 2.15 | 78.3 | 6.75 | 1.51 | 3 |  |
| 20 | 18 | 0 | -0.275 | 19.7 | 2.15 | S-03 x0.8 | 2.23 | 2.23 | 92.9 | 7.29 | 1.51 | 2 |  |

One change at a time, first scenario at nominal mass (excursion columns: lower bound, ideal actuators):

| case | a_eq_ms2 | x_actual_m | alt_gain_m | alt_loss_m | vh_end_ms | alpha_max_gated_deg | t_above_alpha_08_s | t_above_alpha_s_s | a_rec_pct_ensemble_ms2 | rotor_lag_s | tau_rotor_model_s | tau_total_s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| base | 2 | 56.1 | 0.225 | 0.367 | 0.0041 | 22 | 2.9 | 2.02 | 1.78 | 0 | 0.0227 | 0.0227 |
| theta_fw0_0deg | 2 | 56.1 | 0.225 | 0.367 | 0.0041 | 22 | 2.9 | 2.02 |  | 0 | 0.0227 | 0.0227 |
| theta_fw0_4deg | 2.1 | 53.4 | 0.117 | 0.326 | 0.0769 | 21.5 | 3.16 | 2.06 |  | 0 | 0.0227 | 0.0227 |
| theta_fw0_8deg | 2.19 | 51.1 | 0.0168 | 0.387 | 0.198 | 20.8 | 3.72 | 2.26 |  | 0 | 0.0227 | 0.0227 |
| theta_fw0_12deg | 2.36 | 47.4 | 0.689 | 0.617 | -0.23 | 25.5 | 4 | 2.22 |  | 0 | 0.0227 | 0.0227 |
| rotor_lag_0s | 2 | 56.1 | 0.225 | 0.367 | 0.0041 | 22 | 2.9 | 2.02 |  | 0 | 0.0227 | 0.0227 |
| rotor_lag_0.1s | 2.01 | 55.8 | 0.548 | 0.875 | 0.0491 | 25.2 | 2.78 | 1.96 |  | 0.1 | 0.0227 | 0.123 |
| rotor_lag_0.25s | 2 | 56 | 0.972 | 1.81 | -0.0705 | 31.9 | 2.7 | 1.92 |  | 0.25 | 0.0227 | 0.273 |
| rotor_lag_0.5s | 1.98 | 56.5 | 1.49 | 3.29 | -0.817 | 44.5 | 2.72 | 1.96 |  | 0.5 | 0.0227 | 0.523 |
| ff_mode_repo | 2.05 | 54.5 | 5.31 | 0.32 | -3.86 | 63.9 | 2.2 | 1.5 | 1.98 | 0 | 0.0227 | 0.0227 |
| lift_filter_0s | 2 | 56.1 | 0.225 | 0.367 | 0.0041 | 22 | 2.9 | 2.02 |  | 0 | 0.0227 | 0.0227 |
| lift_filter_0.5s | 2 | 56.1 | 1.32 | 2.29 | -0.477 | 37.4 | 2.66 | 1.88 |  | 0 | 0.0227 | 0.0227 |
| lift_scale_0.8 | 2.01 | 55.8 | 2.14 | 0.436 | -1.49 | 35.3 | 2.48 | 1.62 |  | 0 | 0.0227 | 0.0227 |
| lift_scale_1.2 | 1.96 | 57.1 | 0 | 1.69 | 1.13 | 21.1 | 3.44 | 1.8 |  | 0 | 0.0227 | 0.0227 |
| enforce_alpha | 1.26 | 88.9 | 0.179 | 0.307 | 0.1 | 9.31 | 2.88 | 0 | 1.14 | 0 | 0.0227 | 0.0227 |

Trade curve (switch earlier = smaller a_plan = more VTOL distance and energy):

| a_plan_ms2 | a_lim_ms2 | speed_test | headwind_ms | x_plan_m | x_actual_m | overshoot_m | energy_J |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1.5 | 1.5 | ground | -5 | 133 | 134 | 0.616 | 27583 |
| 1.5 | 3 | ground | -5 | 133 | 142 | 8.69 | 29391 |
| 1.5 | 1.5 | ground | 0 | 75 | 75.6 | 0.577 | 18070 |
| 1.5 | 3 | ground | 0 | 75 | 78.9 | 3.89 | 20744 |
| 1.5 | 1.5 | ground | 5 | 33.3 | 34.1 | 0.805 | 10621 |
| 1.5 | 3 | ground | 5 | 33.3 | 34.3 | 0.957 | 11983 |
| 1.5 | 1.5 | air | 5 | 75 | 76 | 0.98 | 27373 |
| 1.5 | 3 | air | 5 | 75 | 76 | 0.979 | 27397 |
| 1.55 | 1.55 | ground | -5 | 129 | 130 | 0.661 | 26706 |
| 1.55 | 3 | ground | -5 | 129 | 138 | 8.7 | 28698 |
| 1.55 | 1.55 | ground | 0 | 72.6 | 73.2 | 0.591 | 17783 |
| 1.55 | 3 | ground | 0 | 72.6 | 76.6 | 4.01 | 20258 |
| 1.55 | 1.55 | ground | 5 | 32.3 | 33.1 | 0.812 | 10440 |
| 1.55 | 3 | ground | 5 | 32.3 | 33.4 | 1.14 | 11401 |
| 1.55 | 1.55 | air | 5 | 72.6 | 73.5 | 0.964 | 26233 |
| 1.55 | 3 | air | 5 | 72.6 | 73.5 | 0.962 | 26256 |
| 1.6 | 1.6 | ground | -5 | 125 | 126 | 0.704 | 25974 |
| 1.6 | 3 | ground | -5 | 125 | 134 | 8.7 | 28058 |
| 1.6 | 1.6 | ground | 0 | 70.3 | 70.9 | 0.606 | 17525 |
| 1.6 | 3 | ground | 0 | 70.3 | 74.4 | 4.13 | 19807 |
| 1.6 | 1.6 | ground | 5 | 31.2 | 32.1 | 0.827 | 10262 |
| 1.6 | 3 | ground | 5 | 31.2 | 32.7 | 1.48 | 10790 |
| 1.6 | 1.6 | air | 5 | 70.3 | 71.3 | 0.947 | 25208 |
| 1.6 | 3 | air | 5 | 70.3 | 71.3 | 0.946 | 25225 |
| 1.65 | 1.65 | ground | -5 | 121 | 122 | 0.744 | 25357 |
| 1.65 | 3 | ground | -5 | 121 | 130 | 8.7 | 27466 |
| 1.65 | 1.65 | ground | 0 | 68.2 | 68.8 | 0.622 | 17288 |
| 1.65 | 3 | ground | 0 | 68.2 | 72.4 | 4.25 | 19388 |
| 1.65 | 1.65 | ground | 5 | 30.3 | 31.2 | 0.91 | 9918 |
| 1.65 | 3 | ground | 5 | 30.3 | 32.3 | 1.95 | 10322 |
| 1.65 | 1.65 | air | 5 | 68.2 | 69.1 | 0.935 | 24272 |
| 1.65 | 3 | air | 5 | 68.2 | 69.1 | 0.935 | 24283 |
| 1.7 | 1.7 | ground | -5 | 118 | 118 | 0.777 | 24862 |
| 1.7 | 3 | ground | -5 | 118 | 126 | 8.7 | 26915 |
| 1.7 | 1.7 | ground | 0 | 66.2 | 66.8 | 0.638 | 17063 |
| 1.7 | 3 | ground | 0 | 66.2 | 70.5 | 4.35 | 18998 |
| 1.7 | 1.7 | ground | 5 | 29.4 | 30.6 | 1.17 | 9414 |
| 1.7 | 3 | ground | 5 | 29.4 | 31.9 | 2.47 | 9965 |
| 1.7 | 1.7 | air | 5 | 66.2 | 67.1 | 0.927 | 23414 |
| 1.7 | 3 | air | 5 | 66.2 | 67.1 | 0.927 | 23419 |
| 1.75 | 1.75 | ground | -5 | 114 | 115 | 0.805 | 24459 |
| 1.75 | 3 | ground | -5 | 114 | 123 | 8.7 | 26402 |
| 1.75 | 1.75 | ground | 0 | 64.3 | 64.9 | 0.657 | 16838 |
| 1.75 | 3 | ground | 0 | 64.3 | 68.7 | 4.46 | 18634 |
| 1.75 | 1.75 | ground | 5 | 28.6 | 30.2 | 1.6 | 8976 |
| 1.75 | 3 | ground | 5 | 28.6 | 31.6 | 3 | 9669 |
| 1.75 | 1.75 | air | 5 | 64.3 | 65.2 | 0.922 | 22623 |
| 1.75 | 3 | air | 5 | 64.3 | 65.2 | 0.922 | 22625 |
| 1.8 | 1.8 | ground | -5 | 111 | 112 | 0.836 | 24118 |
| 1.8 | 3 | ground | -5 | 111 | 120 | 8.69 | 25924 |
| 1.8 | 1.8 | ground | 0 | 62.5 | 63.2 | 0.677 | 16606 |
| 1.8 | 3 | ground | 0 | 62.5 | 67.1 | 4.56 | 18292 |
| 1.8 | 1.8 | ground | 5 | 27.8 | 30 | 2.18 | 8730 |
| 1.8 | 3 | ground | 5 | 27.8 | 31.3 | 3.52 | 9440 |
| 1.8 | 1.8 | air | 5 | 62.5 | 63.4 | 0.918 | 21892 |
| 1.8 | 3 | air | 5 | 62.5 | 63.4 | 0.918 | 21892 |
| 1.85 | 1.85 | ground | -5 | 108 | 109 | 0.887 | 23811 |
| 1.85 | 3 | ground | -5 | 108 | 117 | 8.69 | 25477 |
| 1.85 | 1.85 | ground | 0 | 60.8 | 61.5 | 0.712 | 16359 |
| 1.85 | 3 | ground | 0 | 60.8 | 65.5 | 4.66 | 17970 |
| 1.85 | 1.85 | ground | 5 | 27 | 29.9 | 2.83 | 8568 |
| 1.85 | 3 | ground | 5 | 27 | 31.1 | 4.04 | 9260 |
| 1.85 | 1.85 | air | 5 | 60.8 | 61.7 | 0.915 | 21215 |
| 1.85 | 3 | air | 5 | 60.8 | 61.7 | 0.915 | 21215 |
| 1.9 | 1.9 | ground | -5 | 105 | 106 | 0.952 | 23531 |
| 1.9 | 3 | ground | -5 | 105 | 114 | 8.69 | 25058 |
| 1.9 | 1.9 | ground | 0 | 59.2 | 60 | 0.771 | 16094 |
| 1.9 | 3 | ground | 0 | 59.2 | 64.1 | 4.86 | 17662 |
| 1.9 | 1.9 | ground | 5 | 26.3 | 29.8 | 3.52 | 8508 |
| 1.9 | 3 | ground | 5 | 26.3 | 30.9 | 4.54 | 9113 |
| 1.9 | 1.9 | air | 5 | 59.2 | 60.1 | 0.912 | 20588 |
| 1.9 | 3 | air | 5 | 59.2 | 60.1 | 0.912 | 20588 |
| 1.95 | 1.95 | ground | -5 | 103 | 104 | 1.03 | 23274 |
| 1.95 | 3 | ground | -5 | 103 | 111 | 8.68 | 24664 |
| 1.95 | 1.95 | ground | 0 | 57.7 | 58.5 | 0.857 | 15832 |
| 1.95 | 3 | ground | 0 | 57.7 | 62.9 | 5.19 | 17373 |
| 1.95 | 1.95 | ground | 5 | 25.6 | 29.8 | 4.18 | 8491 |
| 1.95 | 3 | ground | 5 | 25.6 | 30.7 | 5.03 | 8991 |
| 1.95 | 1.95 | air | 5 | 57.7 | 58.6 | 0.91 | 20005 |
| 1.95 | 3 | air | 5 | 57.7 | 58.6 | 0.91 | 20005 |
| 2 | 2 | ground | -5 | 100 | 101 | 1.13 | 23031 |
| 2 | 3 | ground | -5 | 100 | 109 | 8.68 | 24294 |
| 2 | 2 | ground | 0 | 56.2 | 57.3 | 1 | 15516 |
| 2 | 3 | ground | 0 | 56.2 | 61.9 | 5.63 | 17102 |
| 2 | 2 | ground | 5 | 25 | 29.8 | 4.82 | 8489 |
| 2 | 3 | ground | 5 | 25 | 30.5 | 5.5 | 8886 |
| 2 | 2 | air | 5 | 56.2 | 57.2 | 0.908 | 19463 |
| 2 | 3 | air | 5 | 56.2 | 57.2 | 0.908 | 19463 |
| 2.1 | 2.1 | ground | -5 | 95.2 | 96.6 | 1.4 | 22514 |
| 2.1 | 3 | ground | -5 | 95.2 | 104 | 8.68 | 23616 |
| 2.1 | 2.1 | ground | 0 | 53.6 | 56.1 | 2.57 | 14420 |
| 2.1 | 3 | ground | 0 | 53.6 | 60.4 | 6.85 | 16581 |
| 2.1 | 2.1 | ground | 5 | 23.8 | 29.8 | 6.01 | 8489 |
| 2.1 | 3 | ground | 5 | 23.8 | 30.2 | 6.43 | 8722 |
| 2.1 | 2.1 | air | 5 | 53.6 | 54.5 | 0.905 | 18485 |
| 2.1 | 3 | air | 5 | 53.6 | 54.5 | 0.905 | 18485 |
| 2.25 | 2.25 | ground | -5 | 88.9 | 91.1 | 2.17 | 21120 |
| 2.25 | 3 | ground | -5 | 88.9 | 97.6 | 8.68 | 22729 |
| 2.25 | 2.25 | ground | 0 | 50 | 56.1 | 6.1 | 14294 |
| 2.25 | 3 | ground | 0 | 50 | 59.1 | 9.12 | 15835 |
| 2.25 | 2.25 | ground | 5 | 22.2 | 29.8 | 7.6 | 8489 |
| 2.25 | 3 | ground | 5 | 22.2 | 30 | 7.75 | 8570 |
| 2.25 | 2.25 | air | 5 | 50 | 50.9 | 0.9 | 17236 |
| 2.25 | 3 | air | 5 | 50 | 50.9 | 0.9 | 17236 |
| 2.5 | 2.5 | ground | -5 | 80 | 89.3 | 9.32 | 20397 |
| 2.5 | 3 | ground | -5 | 80 | 92.2 | 12.2 | 21450 |
| 2.5 | 2.5 | ground | 0 | 45 | 56.1 | 11.1 | 14294 |
| 2.5 | 3 | ground | 0 | 45 | 57.5 | 12.5 | 14909 |
| 2.5 | 2.5 | ground | 5 | 20 | 29.8 | 9.82 | 8489 |
| 2.5 | 3 | ground | 5 | 20 | 29.8 | 9.82 | 8489 |
| 2.5 | 2.5 | air | 5 | 45 | 45.9 | 0.895 | 15584 |
| 2.5 | 3 | air | 5 | 45 | 45.9 | 0.895 | 15584 |
| 3 | 3 | ground | -5 | 66.7 | 89.3 | 22.6 | 20397 |
| 3 | 3 | ground | 0 | 37.5 | 56.1 | 18.6 | 14294 |
| 3 | 3 | ground | 5 | 16.7 | 29.8 | 13.2 | 8489 |
| 3 | 3 | air | 5 | 37.5 | 38.4 | 0.893 | 13292 |
| 3.5 | 3 | ground | -5 | 57.1 | 89.3 | 32.2 | 20397 |
| 3.5 | 3.5 | ground | -5 | 57.1 | 89.3 | 32.2 | 20397 |
| 3.5 | 3 | ground | 0 | 32.1 | 56.1 | 24 | 14294 |
| 3.5 | 3.5 | ground | 0 | 32.1 | 56.1 | 24 | 14294 |
| 3.5 | 3 | ground | 5 | 14.3 | 29.8 | 15.5 | 8489 |
| 3.5 | 3.5 | ground | 5 | 14.3 | 29.8 | 15.5 | 8489 |
| 3.5 | 3 | air | 5 | 32.1 | 33.3 | 1.17 | 11331 |
| 3.5 | 3.5 | air | 5 | 32.1 | 33.3 | 1.17 | 11331 |

**Reading the overshoot.** The recommendation is the worst-case a_eq (the deceleration at the pitch limit), but the position cascade does not brake at the limit: it follows a speed profile built on its own deceleration limit `a_lim` (open question O-11). When `a_lim` exceeds what the aircraft can do it starts braking late and overshoots whatever `a_plan` is; with `a_lim = a_plan` the overshoot grows as `a_plan` approaches the aircraft's capability, because the cascade then has no margin left. The Q_TRANS_DECEL number alone does not remove overshoot.

### F — landing descent

κ source: Johnson, NASA/TP-2005-213477, Yeates test data: observable vortex ring state from Vz/vh = -0.23 near zero forward speed (the most conservative onset reported; Drees roughness -0.62 to -1.53, inflow instability -0.5 to -1.5). Read from a summary of the report: confirm against the PDF before quoting [documented]

| scenario | altitude_m | v_h_ms | v_cap_ms | binding_limit | descent_time_s | hover_power_W | hover_energy_J | kappa_source |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S-01 | 0 | 7.65 | 1.76 | vortex_ring_state | 19.6 | 3932 | 77206 | Johnson, NASA/TP-2005-213477, Yeates test data: observable vortex ring state from Vz/vh = -0.23 near zero forward speed (the most conservative onset reported; Drees roughness -0.62 to -1.53, inflow instability -0.5 to -1.5). Read from a summary of the report: confirm against the PDF before quoting [documented] |
| S-02 | 5000 | 9.87 | 2.27 | vortex_ring_state | 15.7 | 4811 | 75559 | Johnson, NASA/TP-2005-213477, Yeates test data: observable vortex ring state from Vz/vh = -0.23 near zero forward speed (the most conservative onset reported; Drees roughness -0.62 to -1.53, inflow instability -0.5 to -1.5). Read from a summary of the report: confirm against the PDF before quoting [documented] |
| S-03 | 5000 | 10.8 | 2.48 | vortex_ring_state | 14.5 | 5977 | 86821 | Johnson, NASA/TP-2005-213477, Yeates test data: observable vortex ring state from Vz/vh = -0.23 near zero forward speed (the most conservative onset reported; Drees roughness -0.62 to -1.53, inflow instability -0.5 to -1.5). Read from a summary of the report: confirm against the PDF before quoting [documented] |
| S-05 | 500 | 7.84 | 1.8 | vortex_ring_state | 19.2 | 4006 | 77010 | Johnson, NASA/TP-2005-213477, Yeates test data: observable vortex ring state from Vz/vh = -0.23 near zero forward speed (the most conservative onset reported; Drees roughness -0.62 to -1.53, inflow instability -0.5 to -1.5). Read from a summary of the report: confirm against the PDF before quoting [documented] |
| S-06 | 5000 | 8.89 | 2.05 | vortex_ring_state | 17.2 | 3565 | 61313 | Johnson, NASA/TP-2005-213477, Yeates test data: observable vortex ring state from Vz/vh = -0.23 near zero forward speed (the most conservative onset reported; Drees roughness -0.62 to -1.53, inflow instability -0.5 to -1.5). Read from a summary of the report: confirm against the PDF before quoting [documented] |

### E — energy budget and reserve

| site_amsl_m | mission | cruise_amsl_m | v_cruise_tas_ms | v_cruise_eas_ms | max_range_km | flown_total_km | fixed_Wh | fixed_out_Wh | fixed_ret_Wh | cruise_Wh | reserve_Wh | reserve_binding_leg | drop_Wh | usable_Wh | soc_start | soc_end_before_reserve | soc_end_after_reserve | window_ok | flyable | n_infeasible | first_infeasible | n_over_ceiling | a_fwd_mode | a_fwd_out_ms2 | a_fwd_ret_ms2 | a_fwd_worst_ms2 | a_plan_ms2 | a_plan_source | margin_Wh_at_100km | conflict_100km | margin_Wh_at_120km | conflict_120km |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | round_trip | 200 | 21.1 | 20.9 | 45.8 | 91.5 | 220 | 122 | 98 | 1407 | 150 | out | 0 | 1776 | 1 | 0.268 | 0.2 | yes | yes | 0 |  | 2 | case | 1.71 | 1.69 | -0.0806 | 1.65 | D | -1777 | yes | -2432 | yes |
| 0 | one_way | 200 | 21.1 | 20.9 | 92 | 92 | 122 |  |  | 1505 | 150 | leg | 0 | 1776 | 1 | 0.268 | 0.2 | yes | yes | 0 |  | 1 | case | 1.71 |  | -0.0806 | 1.65 | D | -136 | yes | -474 | yes |
| 1500 | round_trip | 1700 | 22.7 | 20.9 | 46.1 | 92.3 | 231 | 128 | 103 | 1393 | 153 | out | 0 | 1776 | 1 | 0.269 | 0.2 | yes | yes | 0 |  | 2 | case | 1.36 | 1.31 | -0.0806 | 1.65 | D | -1739 | yes | -2384 | yes |
| 1500 | one_way | 1700 | 22.7 | 20.9 | 92.9 | 92.9 | 128 |  |  | 1496 | 153 | leg | 0 | 1776 | 1 | 0.269 | 0.2 | yes | yes | 0 |  | 1 | case | 1.36 |  | -0.0806 | 1.65 | D | -118 | yes | -451 | yes |
| 2500 | round_trip | 2700 | 23.9 | 20.9 | 46.3 | 92.6 | 241 | 133 | 107 | 1381 | 155 | out | 0 | 1776 | 1 | 0.27 | 0.2 | yes | yes | 0 |  | 3 | case | 1.13 | 1.06 | -0.0806 | 1.65 | D | -1716 | yes | -2356 | yes |
| 2500 | one_way | 2700 | 23.9 | 20.9 | 93.4 | 93.4 | 133 |  |  | 1488 | 155 | leg | 0 | 1776 | 1 | 0.27 | 0.2 | yes | yes | 0 |  | 2 | case | 1.13 |  | -0.0806 | 1.65 | D | -108 | yes | -438 | yes |
| 5000 | round_trip | 5200 | 27.3 | 20.9 | 46.2 | 92.3 | 288 | 158 | 130 | 1327 | 161 | out | 0 | 1776 | 1 | 0.272 | 0.2 | yes | yes | 0 |  | 4 | case | 0.598 | 0.487 | -0.0806 | 1.65 | D | -1682 | yes | -2307 | yes |
| 5000 | one_way | 5200 | 27.3 | 20.9 | 94.1 | 94.1 | 158 |  |  | 1457 | 161 | leg | 0 | 1776 | 1 | 0.272 | 0.2 | yes | yes | 0 |  | 2 | case | 0.598 |  | -0.0806 | 1.65 | D | -94.9 | yes | -417 | yes |

The range is an OUTPUT: what is left after the fixed segments and the reserve goes to cruise (the radius for a round trip, which flies twice that). The requirements are comparison columns only. A pusher command above `TKOFF_THR_MAX` is reported (`over_ceiling`); a segment fails only above 1 or at another limit (`feasible`, `limit`).

Budget by range (a range shorter than the largest leg's fixed distance is not tabulated):

| site_amsl_m | mission | range_km | cruise_Wh | reserve_Wh | total_Wh | usable_Wh | margin_Wh | fits |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | round_trip | 50 | 1545 | 150 | 1915 | 1776 | -139 | no |
| 0 | round_trip | 100 | 3183 | 150 | 3553 | 1776 | -1777 | no |
| 0 | round_trip | 120 | 3839 | 150 | 4208 | 1776 | -2432 | no |
| 0 | round_trip | 150 | 4821 | 150 | 5191 | 1776 | -3415 | no |
| 0 | one_way | 50 | 796 | 150 | 1067 | 1776 | 709 | yes |
| 0 | one_way | 100 | 1640 | 150 | 1912 | 1776 | -136 | no |
| 0 | one_way | 120 | 1978 | 150 | 2250 | 1776 | -474 | no |
| 0 | one_way | 150 | 2485 | 150 | 2757 | 1776 | -981 | no |
| 1500 | round_trip | 50 | 1517 | 153 | 1900 | 1776 | -124 | no |
| 1500 | round_trip | 100 | 3132 | 153 | 3515 | 1776 | -1739 | no |
| 1500 | round_trip | 120 | 3777 | 153 | 4160 | 1776 | -2384 | no |
| 1500 | round_trip | 150 | 4746 | 153 | 5129 | 1776 | -3353 | no |
| 1500 | one_way | 50 | 781 | 153 | 1062 | 1776 | 714 | yes |
| 1500 | one_way | 100 | 1614 | 153 | 1894 | 1776 | -118 | no |
| 1500 | one_way | 120 | 1947 | 153 | 2227 | 1776 | -451 | no |
| 1500 | one_way | 150 | 2447 | 153 | 2727 | 1776 | -951 | no |
| 2500 | round_trip | 50 | 1498 | 155 | 1894 | 1776 | -118 | no |
| 2500 | round_trip | 100 | 3097 | 155 | 3492 | 1776 | -1716 | no |
| 2500 | round_trip | 120 | 3737 | 155 | 4132 | 1776 | -2356 | no |
| 2500 | round_trip | 150 | 4696 | 155 | 5091 | 1776 | -3315 | no |
| 2500 | one_way | 50 | 772 | 155 | 1059 | 1776 | 717 | yes |
| 2500 | one_way | 100 | 1596 | 155 | 1884 | 1776 | -108 | no |
| 2500 | one_way | 120 | 1926 | 155 | 2214 | 1776 | -438 | no |
| 2500 | one_way | 150 | 2421 | 155 | 2709 | 1776 | -933 | no |
| 5000 | round_trip | 50 | 1447 | 161 | 1896 | 1776 | -120 | no |
| 5000 | round_trip | 100 | 3009 | 161 | 3458 | 1776 | -1682 | no |
| 5000 | round_trip | 120 | 3634 | 161 | 4083 | 1776 | -2307 | no |
| 5000 | round_trip | 150 | 4571 | 161 | 5020 | 1776 | -3244 | no |
| 5000 | one_way | 50 | 746 | 161 | 1065 | 1776 | 711 | yes |
| 5000 | one_way | 100 | 1552 | 161 | 1871 | 1776 | -94.9 | no |
| 5000 | one_way | 120 | 1874 | 161 | 2193 | 1776 | -417 | no |
| 5000 | one_way | 150 | 2358 | 161 | 2677 | 1776 | -901 | no |

Segments:

| site_amsl_m | mission | leg | segment | energy_Wh | soc_start | soc_end | pusher_cmd_max | over_ceiling | feasible | limit |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | round_trip | out | takeoff | 31.5 | 1 | 0.986 | 0 | no | yes |  |
| 0 | round_trip | out | fwd_transition | 12.2 | 0.986 | 0.98 | 0.8 | no | yes |  |
| 0 | round_trip | out | climb | 48.8 | 0.98 | 0.958 | 0.804 | yes | yes |  |
| 0 | round_trip | out | cruise | 724 | 0.958 | 0.632 | 0.686 | no | yes |  |
| 0 | round_trip | out | descent | 2.29 | 0.632 | 0.631 | 0 | no | yes |  |
| 0 | round_trip | out | back_transition | 5.38 | 0.631 | 0.629 |  | no | yes |  |
| 0 | round_trip | out | landing | 21.4 | 0.629 | 0.619 | 0 | no | yes |  |
| 0 | round_trip | ret | takeoff | 23.8 | 0.619 | 0.608 | 0 | no | yes |  |
| 0 | round_trip | ret | fwd_transition | 9.16 | 0.608 | 0.604 | 0.8 | no | yes |  |
| 0 | round_trip | ret | climb | 41.8 | 0.604 | 0.585 | 0.828 | yes | yes |  |
| 0 | round_trip | ret | cruise | 683 | 0.585 | 0.278 | 0.728 | no | yes |  |
| 0 | round_trip | ret | descent | 1.98 | 0.278 | 0.277 | 0 | no | yes |  |
| 0 | round_trip | ret | back_transition | 3.75 | 0.277 | 0.275 |  | no | yes |  |
| 0 | round_trip | ret | landing | 17.4 | 0.275 | 0.268 | 0 | no | yes |  |
| 0 | one_way | leg | takeoff | 31.5 | 1 | 0.986 | 0 | no | yes |  |
| 0 | one_way | leg | fwd_transition | 12.2 | 0.986 | 0.98 | 0.8 | no | yes |  |
| 0 | one_way | leg | climb | 48.8 | 0.98 | 0.958 | 0.804 | yes | yes |  |
| 0 | one_way | leg | cruise | 1505 | 0.958 | 0.281 | 0.743 | no | yes |  |
| 0 | one_way | leg | descent | 2.29 | 0.281 | 0.28 | 0 | no | yes |  |
| 0 | one_way | leg | back_transition | 5.38 | 0.28 | 0.277 |  | no | yes |  |
| 0 | one_way | leg | landing | 21.4 | 0.277 | 0.268 | 0 | no | yes |  |
| 1500 | round_trip | out | takeoff | 33.1 | 1 | 0.985 | 0 | no | yes |  |
| 1500 | round_trip | out | fwd_transition | 15.4 | 0.985 | 0.978 | 0.8 | no | yes |  |
| 1500 | round_trip | out | climb | 49.7 | 0.978 | 0.956 | 0.843 | yes | yes |  |
| 1500 | round_trip | out | cruise | 717 | 0.956 | 0.633 | 0.73 | no | yes |  |
| 1500 | round_trip | out | descent | 2.13 | 0.633 | 0.632 | 0 | no | yes |  |
| 1500 | round_trip | out | back_transition | 6.14 | 0.632 | 0.629 |  | no | yes |  |
| 1500 | round_trip | out | landing | 21.3 | 0.629 | 0.619 | 0 | no | yes |  |
| 1500 | round_trip | ret | takeoff | 24.9 | 0.619 | 0.608 | 0 | no | yes |  |
| 1500 | round_trip | ret | fwd_transition | 11.7 | 0.608 | 0.603 | 0.8 | no | yes |  |
| 1500 | round_trip | ret | climb | 42.9 | 0.603 | 0.584 | 0.87 | yes | yes |  |
| 1500 | round_trip | ret | cruise | 676 | 0.584 | 0.279 | 0.775 | no | yes |  |
| 1500 | round_trip | ret | descent | 1.84 | 0.279 | 0.278 | 0 | no | yes |  |
| 1500 | round_trip | ret | back_transition | 4.3 | 0.278 | 0.277 |  | no | yes |  |
| 1500 | round_trip | ret | landing | 17.3 | 0.277 | 0.269 | 0 | no | yes |  |
| 1500 | one_way | leg | takeoff | 33.1 | 1 | 0.985 | 0 | no | yes |  |
| 1500 | one_way | leg | fwd_transition | 15.4 | 0.985 | 0.978 | 0.8 | no | yes |  |
| 1500 | one_way | leg | climb | 49.7 | 0.978 | 0.956 | 0.843 | yes | yes |  |
| 1500 | one_way | leg | cruise | 1496 | 0.956 | 0.282 | 0.79 | no | yes |  |
| 1500 | one_way | leg | descent | 2.13 | 0.282 | 0.281 | 0 | no | yes |  |
| 1500 | one_way | leg | back_transition | 6.14 | 0.281 | 0.278 |  | no | yes |  |
| 1500 | one_way | leg | landing | 21.3 | 0.278 | 0.269 | 0 | no | yes |  |
| 2500 | round_trip | out | takeoff | 34.2 | 1 | 0.985 | 0 | no | yes |  |
| 2500 | round_trip | out | fwd_transition | 18.7 | 0.985 | 0.976 | 0.8 | no | yes |  |
| 2500 | round_trip | out | climb | 50.5 | 0.976 | 0.953 | 0.872 | yes | yes |  |
| 2500 | round_trip | out | cruise | 711 | 0.953 | 0.633 | 0.761 | no | yes |  |
| 2500 | round_trip | out | descent | 2.02 | 0.633 | 0.632 | 0 | no | yes |  |
| 2500 | round_trip | out | back_transition | 6.71 | 0.632 | 0.629 |  | no | yes |  |
| 2500 | round_trip | out | landing | 21.2 | 0.629 | 0.62 | 0 | no | yes |  |
| 2500 | round_trip | ret | takeoff | 25.8 | 0.62 | 0.608 | 0 | no | yes |  |
| 2500 | round_trip | ret | fwd_transition | 14.3 | 0.608 | 0.602 | 0.8 | no | yes |  |
| 2500 | round_trip | ret | climb | 43.6 | 0.602 | 0.582 | 0.9 | yes | yes |  |
| 2500 | round_trip | ret | cruise | 670 | 0.582 | 0.28 | 0.809 | yes | yes |  |
| 2500 | round_trip | ret | descent | 1.75 | 0.28 | 0.279 | 0 | no | yes |  |
| 2500 | round_trip | ret | back_transition | 4.71 | 0.279 | 0.277 |  | no | yes |  |
| 2500 | round_trip | ret | landing | 17.2 | 0.277 | 0.27 | 0 | no | yes |  |
| 2500 | one_way | leg | takeoff | 34.2 | 1 | 0.985 | 0 | no | yes |  |
| 2500 | one_way | leg | fwd_transition | 18.7 | 0.985 | 0.976 | 0.8 | no | yes |  |
| 2500 | one_way | leg | climb | 50.5 | 0.976 | 0.953 | 0.872 | yes | yes |  |
| 2500 | one_way | leg | cruise | 1488 | 0.953 | 0.283 | 0.825 | yes | yes |  |
| 2500 | one_way | leg | descent | 2.02 | 0.283 | 0.282 | 0 | no | yes |  |
| 2500 | one_way | leg | back_transition | 6.71 | 0.282 | 0.279 |  | no | yes |  |
| 2500 | one_way | leg | landing | 21.2 | 0.279 | 0.27 | 0 | no | yes |  |
| 5000 | round_trip | out | takeoff | 37.4 | 1 | 0.983 | 0 | no | yes |  |
| 5000 | round_trip | out | fwd_transition | 36.8 | 0.983 | 0.967 | 0.8 | no | yes |  |
| 5000 | round_trip | out | climb | 52.8 | 0.967 | 0.943 | 0.955 | yes | yes |  |
| 5000 | round_trip | out | cruise | 684 | 0.943 | 0.635 | 0.852 | yes | yes |  |
| 5000 | round_trip | out | descent | 1.77 | 0.635 | 0.634 | 0 | no | yes |  |
| 5000 | round_trip | out | back_transition | 8.49 | 0.634 | 0.63 |  | no | yes |  |
| 5000 | round_trip | out | landing | 21 | 0.63 | 0.621 | 0 | no | yes |  |
| 5000 | round_trip | ret | takeoff | 28.2 | 0.621 | 0.608 | 0 | no | yes |  |
| 5000 | round_trip | ret | fwd_transition | 31.5 | 0.608 | 0.594 | 0.8 | no | yes |  |
| 5000 | round_trip | ret | climb | 46 | 0.594 | 0.573 | 0.99 | yes | yes |  |
| 5000 | round_trip | ret | cruise | 643 | 0.573 | 0.283 | 0.905 | yes | yes |  |
| 5000 | round_trip | ret | descent | 1.53 | 0.283 | 0.283 | 0 | no | yes |  |
| 5000 | round_trip | ret | back_transition | 5.94 | 0.283 | 0.28 |  | no | yes |  |
| 5000 | round_trip | ret | landing | 17.1 | 0.28 | 0.272 | 0 | no | yes |  |
| 5000 | one_way | leg | takeoff | 37.4 | 1 | 0.983 | 0 | no | yes |  |
| 5000 | one_way | leg | fwd_transition | 36.8 | 0.983 | 0.967 | 0.8 | no | yes |  |
| 5000 | one_way | leg | climb | 52.8 | 0.967 | 0.943 | 0.955 | yes | yes |  |
| 5000 | one_way | leg | cruise | 1457 | 0.943 | 0.286 | 0.923 | yes | yes |  |
| 5000 | one_way | leg | descent | 1.77 | 0.286 | 0.286 | 0 | no | yes |  |
| 5000 | one_way | leg | back_transition | 8.49 | 0.286 | 0.282 |  | no | yes |  |
| 5000 | one_way | leg | landing | 21 | 0.282 | 0.272 | 0 | no | yes |  |

Hover power check (stress point, landing start, after the reserve):

| site_amsl_m | mission | point | soc | p_hover_req_W | v_bus_eod_V | p_available_W | margin_pct | throttle_cmd | ok |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | round_trip | stress (soc_eod) | 0.1 | 2902 | 40.2 | 11591 | 299 | 0.59 | yes |
| 0 | round_trip | landing start | 0.275 | 2907 | 42.2 | 12142 | 318 | 0.565 | yes |
| 0 | round_trip | after reserve | 0.2 | 2905 | 41.3 | 11905 | 310 | 0.576 | yes |
| 0 | one_way | stress (soc_eod) | 0.1 | 3909 | 40.1 | 11547 | 195 | 0.661 | yes |
| 0 | one_way | landing start | 0.277 | 3915 | 42 | 12106 | 209 | 0.633 | yes |
| 0 | one_way | after reserve | 0.2 | 3912 | 41.2 | 11863 | 203 | 0.645 | yes |
| 1500 | round_trip | stress (soc_eod) | 0.1 | 3074 | 40.2 | 11584 | 277 | 0.624 | yes |
| 1500 | round_trip | landing start | 0.277 | 3079 | 42.1 | 12139 | 294 | 0.597 | yes |
| 1500 | round_trip | after reserve | 0.2 | 3077 | 41.3 | 11898 | 287 | 0.609 | yes |
| 1500 | one_way | stress (soc_eod) | 0.1 | 4139 | 40.1 | 11537 | 179 | 0.699 | yes |
| 1500 | one_way | landing start | 0.278 | 4145 | 42 | 12100 | 192 | 0.669 | yes |
| 1500 | one_way | after reserve | 0.2 | 4143 | 41.2 | 11853 | 186 | 0.682 | yes |
| 2500 | round_trip | stress (soc_eod) | 0.1 | 3199 | 40.2 | 11578 | 262 | 0.648 | yes |
| 2500 | round_trip | landing start | 0.277 | 3205 | 42.1 | 12136 | 279 | 0.621 | yes |
| 2500 | round_trip | after reserve | 0.2 | 3202 | 41.3 | 11893 | 271 | 0.633 | yes |
| 2500 | one_way | stress (soc_eod) | 0.1 | 4308 | 40 | 11530 | 168 | 0.727 | yes |
| 2500 | one_way | landing start | 0.279 | 4313 | 42 | 12096 | 180 | 0.695 | yes |
| 2500 | one_way | after reserve | 0.2 | 4311 | 41.1 | 11846 | 175 | 0.709 | yes |
| 5000 | round_trip | stress (soc_eod) | 0.1 | 3558 | 40.1 | 11563 | 225 | 0.719 | yes |
| 5000 | round_trip | landing start | 0.28 | 3564 | 42.1 | 12130 | 240 | 0.688 | yes |
| 5000 | round_trip | after reserve | 0.2 | 3561 | 41.2 | 11878 | 234 | 0.701 | yes |
| 5000 | one_way | stress (soc_eod) | 0.1 | 4788 | 40 | 11509 | 140 | 0.806 | yes |
| 5000 | one_way | landing start | 0.282 | 4794 | 42 | 12084 | 152 | 0.77 | yes |
| 5000 | one_way | after reserve | 0.2 | 4792 | 41.1 | 11825 | 147 | 0.785 | yes |

Reserve (go-around, hold, diversion):

| component | energy_Wh |
| --- | --- |
| go_around | 65.4 |
| hold | 0 |
| diversion_cruise | 84.5 |
| diversion_transitions | 0 |
| total | 150 |

### Q2 — pusher forward acceleration to AIRSPEED_MIN

Forward acceleration the pusher can deliver at `TKOFF_THR_MAX` (0.8, through the ESC dead zone) from rest to `AIRSPEED_MIN` (17 m/s). The wing carries E's share (V/V_lw)^2 of the weight, the rotors carry the rest and load the same battery bus. `a_min_ms2` is the worst over speed, `a_equiv_ms2` the constant acceleration covering the same distance. A cell with `reachable = no` cannot reach the speed (`a_min_ms2` <= 0, `v_reach_ms` says how far it gets). `AIRSPEED_MIN` is registered as a true airspeed; both readings are tabulated (DEC-1 is open) and E reads `eas`. The battery is a placeholder (linear open-circuit voltage, assumed resistances, no pack current limit).

Worst cell in each convention:

| convention | altitude_m | mass_factor | soc | v_end_tas_ms | a_min_ms2 | a_equiv_ms2 | t_s | x_m | v_reach_ms | reachable |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| tas | 5000 | 1.15 | 0.3 | 17 | 0.476 | 0.886 | 16.1 | 163 | 17 | yes |
| eas | 5000 | 1.15 | 0.3 | 21.9 | -0.0806 |  |  |  | 20.3 | no |

All cells:

| convention | altitude_m | mass_kg | soc | v_end_tas_ms | a_min_ms2 | a_equiv_ms2 | reachable | current_limited |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| tas | 0 | 21.3 | 0.8 | 17 | 2.02 | 2.76 | yes | no |
| tas | 0 | 21.3 | 0.6 | 17 | 1.72 | 2.43 | yes | no |
| tas | 0 | 21.3 | 0.3 | 17 | 1.28 | 1.96 | yes | no |
| tas | 0 | 26.7 | 0.8 | 17 | 1.48 | 2.12 | yes | no |
| tas | 0 | 26.7 | 0.6 | 17 | 1.24 | 1.86 | yes | no |
| tas | 0 | 26.7 | 0.3 | 17 | 0.889 | 1.47 | yes | no |
| tas | 0 | 30.7 | 0.8 | 17 | 1.19 | 1.81 | yes | no |
| tas | 0 | 30.7 | 0.6 | 17 | 0.973 | 1.58 | yes | no |
| tas | 0 | 30.7 | 0.3 | 17 | 0.668 | 1.23 | yes | no |
| tas | 1500 | 21.3 | 0.8 | 17 | 1.79 | 2.47 | yes | no |
| tas | 1500 | 21.3 | 0.6 | 17 | 1.51 | 2.17 | yes | no |
| tas | 1500 | 21.3 | 0.3 | 17 | 1.11 | 1.74 | yes | no |
| tas | 1500 | 26.7 | 0.8 | 17 | 1.27 | 1.91 | yes | no |
| tas | 1500 | 26.7 | 0.6 | 17 | 1.05 | 1.67 | yes | no |
| tas | 1500 | 26.7 | 0.3 | 17 | 0.73 | 1.31 | yes | no |
| tas | 1500 | 30.7 | 0.8 | 17 | 1.03 | 1.65 | yes | no |
| tas | 1500 | 30.7 | 0.6 | 17 | 0.838 | 1.44 | yes | no |
| tas | 1500 | 30.7 | 0.3 | 17 | 0.56 | 1.12 | yes | no |
| tas | 2500 | 21.3 | 0.8 | 17 | 1.62 | 2.28 | yes | no |
| tas | 2500 | 21.3 | 0.6 | 17 | 1.36 | 2 | yes | no |
| tas | 2500 | 21.3 | 0.3 | 17 | 0.985 | 1.59 | yes | no |
| tas | 2500 | 26.7 | 0.8 | 17 | 1.13 | 1.79 | yes | no |
| tas | 2500 | 26.7 | 0.6 | 17 | 0.924 | 1.56 | yes | no |
| tas | 2500 | 26.7 | 0.3 | 17 | 0.625 | 1.22 | yes | no |
| tas | 2500 | 30.7 | 0.8 | 17 | 0.981 | 1.55 | yes | no |
| tas | 2500 | 30.7 | 0.6 | 17 | 0.8 | 1.35 | yes | no |
| tas | 2500 | 30.7 | 0.3 | 17 | 0.54 | 1.06 | yes | no |
| tas | 5000 | 21.3 | 0.8 | 17 | 1.23 | 1.88 | yes | no |
| tas | 5000 | 21.3 | 0.6 | 17 | 1.01 | 1.64 | yes | no |
| tas | 5000 | 21.3 | 0.3 | 17 | 0.698 | 1.3 | yes | no |
| tas | 5000 | 26.7 | 0.8 | 17 | 0.975 | 1.49 | yes | no |
| tas | 5000 | 26.7 | 0.6 | 17 | 0.801 | 1.3 | yes | no |
| tas | 5000 | 26.7 | 0.3 | 17 | 0.553 | 1.03 | yes | no |
| tas | 5000 | 30.7 | 0.8 | 17 | 0.843 | 1.29 | yes | no |
| tas | 5000 | 30.7 | 0.6 | 17 | 0.692 | 1.13 | yes | no |
| tas | 5000 | 30.7 | 0.3 | 17 | 0.476 | 0.886 | yes | no |
| eas | 0 | 21.3 | 0.8 | 17 | 2.02 | 2.76 | yes | no |
| eas | 0 | 21.3 | 0.6 | 17 | 1.72 | 2.43 | yes | no |
| eas | 0 | 21.3 | 0.3 | 17 | 1.28 | 1.96 | yes | no |
| eas | 0 | 26.7 | 0.8 | 17 | 1.48 | 2.12 | yes | no |
| eas | 0 | 26.7 | 0.6 | 17 | 1.24 | 1.86 | yes | no |
| eas | 0 | 26.7 | 0.3 | 17 | 0.889 | 1.47 | yes | no |
| eas | 0 | 30.7 | 0.8 | 17 | 1.19 | 1.81 | yes | no |
| eas | 0 | 30.7 | 0.6 | 17 | 0.973 | 1.58 | yes | no |
| eas | 0 | 30.7 | 0.3 | 17 | 0.668 | 1.23 | yes | no |
| eas | 1500 | 21.3 | 0.8 | 18.3 | 1.61 | 2.32 | yes | no |
| eas | 1500 | 21.3 | 0.6 | 18.3 | 1.33 | 2.02 | yes | no |
| eas | 1500 | 21.3 | 0.3 | 18.3 | 0.937 | 1.58 | yes | no |
| eas | 1500 | 26.7 | 0.8 | 18.3 | 1.15 | 1.77 | yes | no |
| eas | 1500 | 26.7 | 0.6 | 18.3 | 0.932 | 1.53 | yes | no |
| eas | 1500 | 26.7 | 0.3 | 18.3 | 0.614 | 1.16 | yes | no |
| eas | 1500 | 30.7 | 0.8 | 18.3 | 0.898 | 1.5 | yes | no |
| eas | 1500 | 30.7 | 0.6 | 18.3 | 0.705 | 1.28 | yes | no |
| eas | 1500 | 30.7 | 0.3 | 18.3 | 0.428 | 0.948 | yes | no |
| eas | 2500 | 21.3 | 0.8 | 19.2 | 1.35 | 2.04 | yes | no |
| eas | 2500 | 21.3 | 0.6 | 19.2 | 1.09 | 1.76 | yes | no |
| eas | 2500 | 21.3 | 0.3 | 19.2 | 0.717 | 1.33 | yes | no |
| eas | 2500 | 26.7 | 0.8 | 19.2 | 0.941 | 1.54 | yes | no |
| eas | 2500 | 26.7 | 0.6 | 19.2 | 0.735 | 1.3 | yes | no |
| eas | 2500 | 26.7 | 0.3 | 19.2 | 0.438 | 0.948 | yes | no |
| eas | 2500 | 30.7 | 0.8 | 19.2 | 0.713 | 1.29 | yes | no |
| eas | 2500 | 30.7 | 0.6 | 19.2 | 0.533 | 1.08 | yes | no |
| eas | 2500 | 30.7 | 0.3 | 19.2 | 0.275 | 0.747 | yes | no |
| eas | 5000 | 21.3 | 0.8 | 21.9 | 0.722 | 1.35 | yes | no |
| eas | 5000 | 21.3 | 0.6 | 21.9 | 0.509 | 1.09 | yes | no |
| eas | 5000 | 21.3 | 0.3 | 21.9 | 0.205 | 0.681 | yes | no |
| eas | 5000 | 26.7 | 0.8 | 21.9 | 0.442 | 0.958 | yes | no |
| eas | 5000 | 26.7 | 0.6 | 21.9 | 0.272 | 0.73 | yes | no |
| eas | 5000 | 26.7 | 0.3 | 21.9 | 0.0286 | 0.274 | yes | no |
| eas | 5000 | 30.7 | 0.8 | 21.9 | 0.279 | 0.756 | yes | no |
| eas | 5000 | 30.7 | 0.6 | 21.9 | 0.131 | 0.523 | yes | no |
| eas | 5000 | 30.7 | 0.3 | 21.9 | -0.0806 |  | no | no |

### FT — forward transition (TECS-lite, uncalibrated)

Cost 31.296 (register values) → **0.759**; 3-seed spread **0.39%** (T-12 limit 1%). The aircraft starts in hover 30 m above the site and flies the forward transition (WAIT, hand-over when the equivalent airspeed exceeds `AIRSPEED_MIN`, FADE, DONE); `AIRSPEED_MIN`, `Q_TRANSITION_MS`, `TKOFF_THR_MAX` and `Q_TRAN_PIT_MAX` are searched over the first members of D's Sobol sample (C uses a Latin hypercube, so the members differ from C's). Every speed is an equivalent airspeed. A run fails on an EAS below 1.2 x the stall speed from the hand-over on, an altitude loss beyond 10 m or gain beyond 10 m (assumed limits), a projected minimum altitude below the loss limit, a stall, or no hand-over.

**Model status.** The pitch and pusher come from TECS-lite, a port of ArduPilot's TECS (`Plane-4.7.2`) with its default gains; nothing is calibrated against a log or SITL. TECS starts `warm` (does TECS run through the VTOL hover? open). `Q_ASSIST_SPEED` is assumed 0 (O-20): nothing restarts the fade, and the minimum EAS of the fade is reported (`V_eas_min_fade_ms`) so a flown value can be checked. The height demand is held constant and the speed and height filters of the source are replaced by the true states. The projected minimum altitude is a lower bound on the loss (the pull-up is assumed to start at once). The weights of `J` are a policy choice; its terms are printed.

| case | airspeed_min_eas_ms | q_transition_s | tkoff_thr_max | tran_pit_max_deg | J | J_exc | J_E | J_pen | J_start | J_snapped_minus_continuous | seed_spread | at_bound |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| optimum | 18 | 17 | 1 | 6 | 0.759 | 0.391 | 0.368 | 0 | 31.3 | 0.00358 | 0.00392 | tkoff_thr_max:hi |
| register | 17 | 5 | 0.8 | 3 | 51.3 | 1.01 | 0.298 | 50 |  |  |  |  |
| ardupilot_default | 9 | 5 | 1 | 3 | 332 | 1.74 | 0.184 | 330 |  |  |  |  |
| optimum_robust | 19 | 17 | 1 | 6 | 0.771 |  |  |  |  |  | 0.00392 |  |

| case | p95_excursion_m | mean_energy_kJ | worst_sink_ms | worst_stall_margin | n_flags | n_no_handover | n_stall_margin | n_stall | n_alt_loss | n_alt_gain | n_proj_loss | n_underspeed | n_battery_cutoff | n_touchdown |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| optimum | 1.96 | 76.5 | -0.0312 | 1.25 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| register | 5.06 | 62.1 | 0.481 | 1.18 | 5 | 0 | 5 | 0 | 0 | 0 | 0 | 0 | 0 | 0 |
| ardupilot_default | 8.69 | 38.3 | 0.251 | 0.628 | 33 | 0 | 32 | 0 | 1 | 0 | 0 | 0 | 0 | 0 |
| optimum_robust |  |  |  | 1.25 | 0 |  |  |  |  |  |  |  |  |  |

_FT_params, FT_runs and FT_history evaluate the optimum, the register start and the ArduPilot default on the first 32 members of the Sobol sample and the 8 corners. The grid search, FT_grid, FT_sens and J_start use the first 16 members. The optimum is the best snapped point of FT_grid: whole m/s, whole degrees, whole percent and whole milliseconds, the values ArduPilot can take._
_FT_robust and the optimum_robust row of FT_params are evaluated on the first 16 members, the ensemble of the sensitivities, and are reported only: no search, never the recommendation._
_T-12 on the grid: 15 of 15 points have a three-seed spread within 1%. The winner (17 s) passes. Q_TRANSITION_MS within 2% of the best cost: 14, 15, 15.5, 16, 16.5, 17, 20 s._

Search of the winning grid point:

| seed | best_cost | evaluations | q_transition_s | airspeed_min_eas_ms | tkoff_thr_max | tran_pit_max_deg |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 0.756 | 96 | 17 | 17.6 | 1 | 4.04 |
| 2 | 0.755 | 96 | 17 | 17.5 | 1 | 7 |
| 3 | 0.758 | 96 | 17 | 17.6 | 0.998 | 2.21 |

The grid over `Q_TRANSITION_MS` (each point searched with `Q_TRANSITION_MS` fixed, then snapped to whole m/s, degrees, percent and milliseconds and descended on the lattice):

| point | winner | q_transition_s | J_continuous | seed_spread | t12_ok | J_snapped | J_snapped_minus_continuous | airspeed_min_eas_ms | tkoff_thr_max | tran_pit_max_deg | n_flags |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| coarse | no | 2 | 0.819 | 0.00741 | yes | 0.878 | 0.0591 | 19 | 1 | 2 | 0 |
| coarse | no | 3 | 0.827 | 0.0028 | yes | 0.861 | 0.0339 | 19 | 1 | 6 | 0 |
| coarse | no | 4 | 0.833 | 0.00177 | yes | 0.849 | 0.0156 | 19 | 1 | 2 | 0 |
| coarse | no | 5 | 0.833 | 0.00507 | yes | 0.839 | 0.00674 | 19 | 1 | 2 | 0 |
| coarse | no | 6 | 0.829 | 0.000231 | yes | 0.831 | 0.00193 | 19 | 1 | 1 | 0 |
| coarse | no | 8 | 0.813 | 0.000554 | yes | 0.814 | 0.000687 | 19 | 1 | 2 | 0 |
| coarse | no | 10 | 0.792 | 0.000352 | yes | 0.792 | 7.03e-05 | 18 | 1 | 2 | 0 |
| coarse | no | 12 | 0.773 | 0.00121 | yes | 0.774 | 0.0015 | 18 | 1 | 6 | 0 |
| coarse | no | 14 | 0.761 | 0.000766 | yes | 0.763 | 0.00284 | 18 | 1 | 6 | 0 |
| fine | no | 15 | 0.757 | 0.00388 | yes | 0.761 | 0.00353 | 18 | 1 | 6 | 0 |
| fine | no | 15.5 | 0.756 | 0.00392 | yes | 0.76 | 0.00356 | 18 | 1 | 6 | 0 |
| coarse | no | 16 | 0.756 | 0.00392 | yes | 0.759 | 0.00358 | 18 | 1 | 6 | 0 |
| fine | no | 16.5 | 0.755 | 0.00391 | yes | 0.759 | 0.00358 | 18 | 1 | 6 | 0 |
| fine | yes | 17 | 0.755 | 0.00392 | yes | 0.759 | 0.00358 | 18 | 1 | 6 | 0 |
| coarse | no | 20 | 0.759 | 0.00256 | yes | 0.763 | 0.00385 | 18 | 1 | 8 | 0 |

`AIRSPEED_MIN` held fixed, the other three re-searched:

| airspeed_min_eas_ms | q_transition_s | tkoff_thr_max | tran_pit_max_deg | J | J_exc | J_E | J_pen | n_flags | entry_eas_back_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 15 | 16.8 | 0.999 | 1.51 | 131 | 0.403 | 0.345 | 130 | 13 | 13 |
| 16 | 16.8 | 1 | 3.92 | 80.7 | 0.398 | 0.35 | 80 | 8 | 14 |
| 17 | 17 | 1 | 1.76 | 30.8 | 0.392 | 0.359 | 30 | 3 | 15 |
| 18 | 17 | 1 | 5.33 | 0.759 | 0.39 | 0.369 | 0 | 0 | 16 |
| 19 | 18 | 1 | 8.74 | 0.771 | 0.38 | 0.391 | 0 | 0 | 17 |
| 20 | 2.01 | 1 | 2.8 | 20.9 | 0.509 | 0.424 | 20 | 2 | 18 |

One at a time at the optimum:

| parameter | value | J | J_pen | p95_excursion_m | mean_energy_kJ | worst_sink_ms | worst_stall_margin | n_flags |
| --- | --- | --- | --- | --- | --- | --- | --- | --- |
| base |  | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| TECS_TIME_CONST | 3 | 0.641 | 0 | 1.35 | 77.1 | -0.0615 | 1.25 | 0 |
| TECS_TIME_CONST | 5 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| TECS_TIME_CONST | 8 | 0.945 | 0 | 2.79 | 80.5 | 0.0108 | 1.24 | 0 |
| TECS_INTEG_GAIN | 0.15 | 0.925 | 0 | 2.77 | 77.3 | 0.00487 | 1.24 | 0 |
| TECS_INTEG_GAIN | 0.3 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| TECS_INTEG_GAIN | 0.6 | 0.61 | 0 | 1.2 | 76.9 | -0.0528 | 1.25 | 0 |
| rotor_lag_s | 0 | 0.754 | 0 | 1.93 | 76.3 | -0.0419 | 1.24 | 0 |
| rotor_lag_s | 0.25 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| rotor_lag_s | 0.5 | 0.764 | 0 | 1.97 | 77 | -0.0409 | 1.24 | 0 |
| z_vel_p | 2.5 | 0.779 | 0 | 2.02 | 78 | -0.047 | 1.25 | 0 |
| z_vel_p | 5 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| z_vel_p | 10 | 0.746 | 0 | 1.91 | 75.9 | -0.0381 | 1.24 | 0 |
| z_acc_p | 0.015 | 0.759 | 0 | 1.95 | 76.7 | -0.042 | 1.25 | 0 |
| z_acc_p | 0.03 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| z_acc_p | 0.06 | 0.758 | 0 | 1.95 | 76.7 | -0.0412 | 1.24 | 0 |
| TRIM_THROTTLE | 0.35 | 0.851 | 0 | 2.42 | 76.2 | -0.0538 | 1.25 | 0 |
| TRIM_THROTTLE | 0.45 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| TRIM_THROTTLE | 0.55 | 0.716 | 0 | 1.72 | 77.3 | -0.0234 | 1.25 | 0 |
| tecs_start | warm | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| tecs_start | cold | 0.814 | 0 | 2 | 86.2 | -0.0465 | 1.24 | 0 |
| rec_cl_frac | 0.7 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| rec_cl_frac | 0.8 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| rec_cl_frac | 0.9 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.25 | 0 |
| CL_max_scale | 0.9 | 20.8 | 20 | 1.95 | 76.7 | -0.0418 | 1.18 | 2 |
| CL_max_scale | 1.1 | 0.759 | 0 | 1.95 | 76.7 | -0.0418 | 1.31 | 0 |

`AIRSPEED_MIN` against the C_L,max scale (reporting only, no search):

| airspeed_min_eas_ms | cl_max_scale | J | worst_stall_margin | n_flags | flags |
| --- | --- | --- | --- | --- | --- |
| 17 | 0.9 | 70.8 | 1.12 | 7 | stall_margin |
| 17 | 1 | 30.8 | 1.18 | 3 | stall_margin |
| 17 | 1.1 | 0.752 | 1.23 | 0 |  |
| 18 | 0.9 | 20.8 | 1.18 | 2 | stall_margin |
| 18 | 1 | 0.759 | 1.25 | 0 |  |
| 18 | 1.1 | 0.759 | 1.31 | 0 |  |
| 19 | 0.9 | 0.771 | 1.25 | 0 |  |
| 19 | 1 | 0.771 | 1.31 | 0 |  |
| 19 | 1.1 | 0.771 | 1.38 | 0 |  |
| 20 | 0.9 | 21 | 1.3 | 2 | no_handover |
| 20 | 1 | 21 | 1.37 | 2 | no_handover |
| 20 | 1.1 | 21 | 1.44 | 2 | no_handover |

### E and Q2 at the recommended settings

AIRSPEED_MIN and TKOFF_THR_MAX of the FT optimum instead of the register's; the register case above stays the headline. D is re-run at the recommended entry speed. E's forward transition is the quasi-steady wait to AIRSPEED_MIN plus the rotor fade of Q_TRANSITION_MS. `ft_site_energy_kJ` is the mean energy of FT's simulated transition (hover to the end of the fade) over the FT members nearest to the site, in the same case (`ft_site_wait_kJ` and `ft_site_fade_kJ` its parts, `ft_site_members` how many); `e_fwd_over_ft` is E divided by it. The members of a group still differ in mass, ISA, headwind, SoC and by up to about 1 km of altitude: information, not a check.

| case | site_amsl_m | mission | airspeed_min_ms | tkoff_thr_max | max_range_km | e_fwd_kJ | e_fwd_time_s | e_fwd_distance_m | ft_site_energy_kJ | ft_site_wait_kJ | ft_site_fade_kJ | ft_site_members | e_fwd_over_ft |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| register | 0 | round_trip | 17 | 0.8 | 45.8 | 43.9 | 15 | 170 | 46.4 | 37.3 | 9.06 | 9 | 0.947 |
| register | 0 | one_way | 17 | 0.8 | 92 | 43.9 | 15 | 170 | 46.4 | 37.3 | 9.06 | 9 | 0.947 |
| register | 1500 | round_trip | 17 | 0.8 | 46.1 | 55.6 | 18.5 | 215 | 52.8 | 44 | 8.83 | 7 | 1.05 |
| register | 1500 | one_way | 17 | 0.8 | 92.9 | 55.6 | 18.5 | 215 | 52.8 | 44 | 8.83 | 7 | 1.05 |
| register | 2500 | round_trip | 17 | 0.8 | 46.3 | 67.2 | 22 | 260 | 63.2 | 54.3 | 8.92 | 12 | 1.06 |
| register | 2500 | one_way | 17 | 0.8 | 93.4 | 67.2 | 22 | 260 | 63.2 | 54.3 | 8.92 | 12 | 1.06 |
| register | 5000 | round_trip | 17 | 0.8 | 46.2 | 132 | 41.7 | 513 | 82 | 73.1 | 8.9 | 12 | 1.62 |
| register | 5000 | one_way | 17 | 0.8 | 94.1 | 132 | 41.7 | 513 | 82 | 73.1 | 8.9 | 12 | 1.62 |
| recommended | 0 | round_trip | 18 | 1 | 45.8 | 55 | 22.5 | 356 | 64.3 | 33.6 | 30.7 | 9 | 0.855 |
| recommended | 0 | one_way | 18 | 1 | 92 | 55 | 22.5 | 356 | 64.3 | 33.6 | 30.7 | 9 | 0.855 |
| recommended | 1500 | round_trip | 18 | 1 | 46.2 | 62.1 | 24 | 398 | 69.8 | 38.5 | 31.2 | 7 | 0.89 |
| recommended | 1500 | one_way | 18 | 1 | 93 | 62.1 | 24 | 398 | 69.8 | 38.5 | 31.2 | 7 | 0.89 |
| recommended | 2500 | round_trip | 18 | 1 | 46.5 | 68.2 | 25.4 | 432 | 77.8 | 45.3 | 32.5 | 12 | 0.876 |
| recommended | 2500 | one_way | 18 | 1 | 93.6 | 68.2 | 25.4 | 432 | 77.8 | 45.3 | 32.5 | 12 | 0.876 |
| recommended | 5000 | round_trip | 18 | 1 | 46.9 | 91.7 | 30.9 | 557 | 90.8 | 57.3 | 33.4 | 12 | 1.01 |
| recommended | 5000 | one_way | 18 | 1 | 94.9 | 91.7 | 30.9 | 557 | 90.8 | 57.3 | 33.4 | 12 | 1.01 |

### C — ArduPilot parameters from the AIRBRAKE exit

**Configuration: enforced.** The wing angle is held to |alpha_w| <= 10.99 deg (0.7 x alpha_s, `margin_alpha_frac`); this is the headline and the one written to `settings_recommended`.

Cost 321.631 (register values, q = 2) → **2.168**; 8-seed spread **1.26%** (T-12 limit 1%). Entry is the AIRBRAKE exit like D; four variables are searched, `Q_TRANS_DECEL` (q) among them, which sets the stopping point and is the cascade's acceleration limit. The vertical loop is ArduPilot's structure with its default values; rotor lag 0.25 s, rotors `idle` at the exit.

**The overshoot tolerance decides the answer.** A run that stops more than 2 m past its stopping point, or ends sinking faster than 1.76 m/s, is a failure; both tolerances are proposals. The pitch ramp can deliver 1.19 to 1.48 m/s^2 across the ensemble (`a_cap_ms2`, median 1.36); the optimum asks for `q` = 1.07, so it is below every member's ceiling. Variables within 10% of a bound: `spin_min` (hi), `q` (lo).

`theta_A` against its bound is a stalled-wing brake as in D (check D's alpha and VRS columns in `C_runs`); it is not a recommendation to fly at that angle. `rotor_start` idle is read from the ArduPilot source and not checked in SITL or a log (O-13); the Z-loop defaults are master 4.8.0-dev, not a pinned tag (O-08).

| case | T_bt_s | theta_A_deg | spin_min | Q_TRANS_DECEL_ms2 | J | J_start | seed_spread | at_bound | p95_excursion_m | mean_energy_kJ | p95_distance_m | max_overshoot_m | n_overshoot | n_sinking | n_other_flags | theta_A_hi_deg | theta_A_cfg_hi_deg | ptch_lim_deg | rotor_lag_s | p95_stop_time_s | stall_alpha_peak_deg_max | stall_t_above_peak_s_max | stall_t_above_peak_s_mean | stall_t_above_limit_s_max | stall_wing_share_min | stall_members_above_peak | stall_members | alpha_enforced | alpha_limit_deg | alpha_frac | alpha_peak_deg | alpha_s_deg | blend_M | cl_max | rotor_constant_coeffs | os_tol_m | sink_tol_ms | max_end_sink_ms | os_margin_m | sink_margin_ms | at_edge | n_failing_members | n_not_stopped | n_fail_total | J_smooth |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| optimum | 2.57 | 18.7 | 0.25 | 1.07 | 2.17 | 322 | 0.0126 | spin_min:hi;q:lo | 0.592 | 37.4 | 225 | 1.79 | 0 | 0 | 0 | 20 | 30 | 20 | 0.25 | 17.6 | 11.5 | 0 | 0 | 1.1 | 0.0522 | 0 | 32 | yes | 11 | 0.7 | 13.4 | 15.7 | 70 | 1.44 | yes | 2 | 1.76 | 0.0127 | 0.206 | 1.75 | yes | 0 | 0 | 0 | 2.17 |
| register at q = 2 | 3 | 30 | 0.1 | 2 | 322 |  |  |  | 0.68 | 22.5 | 168 | 61.7 | 32 | 0 | 0 |  |  |  |  | 12.1 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  | 2 | 1.76 | 0.163 | -59.7 | 1.6 | yes | 32 | 0 | 32 | 1.63 |
| ardupilot_default | 3 | 30 | 0.15 | 2 | 322 |  |  |  | 0.655 | 22.5 | 168 | 61.6 | 32 | 0 | 0 |  |  |  |  | 12.1 | 12.5 | 0 | 0 | 4.58 | 0.0687 | 0 | 32 |  |  |  |  |  |  |  |  | 2 | 1.76 | 0.166 | -59.6 | 1.59 | yes | 32 | 0 | 32 | 1.62 |
| optimum_margin | 2.57 | 18.7 | 0.25 | 1.02 | 2.28 |  |  |  | 0.597 | 40.2 | 236 | 1.14 | 0 | 0 | 0 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  | 2 | 1.76 | 0.012 | 0.856 | 1.75 | no | 0 | 0 | 0 | 2.28 |

_Q_A_ANGLE_MAX is searched up to 20 deg = min(30, PTCH_LIM_MAX_DEG 20): the pitch cap is min(Q_A_ANGLE_MAX, PTCH_LIM_MAX_DEG), so the 30 deg limit (DEC-7) is now the pitch limit._

Search:

| seed | best_cost | evaluations | T_bt_s | theta_A_deg | spin_min | Q_TRANS_DECEL_ms2 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 2.19 | 160 | 3.74 | 18.7 | 0.135 | 1.08 |
| 2 | 2.19 | 160 | 2.64 | 19.9 | 0.191 | 1.08 |
| 3 | 2.18 | 160 | 3.45 | 13.3 | 0.196 | 1.08 |
| 4 | 2.19 | 160 | 4.59 | 19.9 | 0.207 | 1.07 |
| 5 | 2.18 | 160 | 2.58 | 18.8 | 0.208 | 1.08 |
| 6 | 2.18 | 160 | 3.07 | 15.5 | 0.169 | 1.08 |
| 7 | 2.17 | 160 | 2.57 | 18.7 | 0.25 | 1.07 |
| 8 | 2.19 | 160 | 1.83 | 20 | 0.139 | 1.08 |

Energy against distance with the optimum's ramp and q varied:

| case | J | p95_excursion_m | mean_energy_kJ | p95_distance_m | max_overshoot_m | n_overshoot | n_sinking | n_other_flags | n_failing_members | n_not_stopped | n_fail_total | J_smooth | max_end_sink_ms | q | grid |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| q = 1 | 2.33 | 0.599 | 41.5 | 241 | 0.921 | 0 | 0 | 0 | 0 | 0 | 0 | 2.33 | 0.0116 | 1 | coarse |
| q = 1.5 | 322 | 0.549 | 23.6 | 171 | 18.2 | 32 | 0 | 0 | 32 | 0 | 32 | 1.6 | 0.12 | 1.5 | coarse |
| q = 2 | 322 | 1.32 | 22.9 | 168 | 62.2 | 32 | 0 | 0 | 32 | 0 | 32 | 1.96 | 0.187 | 2 | coarse |
| q = 2.25 | 322 | 1.98 | 23.1 | 168 | 78.7 | 32 | 0 | 0 | 32 | 0 | 32 | 2.29 | 0.203 | 2.25 | coarse |
| q = 2.5 | 322 | 2.07 | 23.1 | 168 | 92.1 | 32 | 0 | 0 | 32 | 0 | 32 | 2.34 | 0.203 | 2.5 | coarse |
| q = 2.75 | 322 | 2.07 | 23.1 | 168 | 103 | 32 | 0 | 0 | 32 | 0 | 32 | 2.34 | 0.203 | 2.75 | coarse |
| q = 3 | 322 | 2.07 | 23.1 | 168 | 112 | 32 | 0 | 0 | 32 | 0 | 32 | 2.34 | 0.203 | 3 | coarse |
| q = 0.971 | 2.41 | 0.603 | 43.4 | 248 | 0.653 | 0 | 0 | 0 | 0 | 0 | 0 | 2.41 | 0.0108 | 0.971 | fine |
| q = 1.021 | 2.28 | 0.597 | 40.2 | 236 | 1.14 | 0 | 0 | 0 | 0 | 0 | 0 | 2.28 | 0.012 | 1.02 | fine |
| q = 1.071 | 2.17 | 0.592 | 37.4 | 225 | 1.79 | 0 | 0 | 0 | 0 | 0 | 0 | 2.17 | 0.0127 | 1.07 | fine |
| q = 1.121 | 42.1 | 0.586 | 34.8 | 215 | 2.6 | 4 | 0 | 0 | 4 | 0 | 4 | 2.06 | 0.0138 | 1.12 | fine |
| q = 1.171 | 92 | 0.581 | 32.4 | 206 | 3.54 | 9 | 0 | 0 | 9 | 0 | 9 | 1.97 | 0.088 | 1.17 | fine |
| q = 1.221 | 102 | 0.576 | 30.3 | 197 | 4.6 | 10 | 0 | 0 | 10 | 0 | 10 | 1.88 | 0.12 | 1.22 | fine |
| q = 1.271 | 132 | 0.571 | 28.4 | 190 | 5.74 | 13 | 0 | 0 | 13 | 0 | 13 | 1.8 | 0.116 | 1.27 | fine |

One at a time at the optimum:

| parameter | value | J | p95_excursion_m | mean_energy_kJ | p95_distance_m | max_overshoot_m | n_overshoot | n_sinking | n_other_flags | n_failing_members | n_not_stopped | n_fail_total | J_smooth | max_end_sink_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| rotor_lag_s | 0 | 2.09 | 0.429 | 37.4 | 225 | 1.76 | 0 | 0 | 0 | 0 | 0 | 0 | 2.09 | -3.99e-06 |
| rotor_lag_s | 0.25 | 2.17 | 0.592 | 37.4 | 225 | 1.79 | 0 | 0 | 0 | 0 | 0 | 0 | 2.17 | 0.0127 |
| rotor_lag_s | 0.5 | 2.27 | 0.8 | 37.3 | 225 | 1.9 | 0 | 0 | 0 | 0 | 0 | 0 | 2.27 | 0.147 |
| rotor_start | idle | 2.17 | 0.592 | 37.4 | 225 | 1.79 | 0 | 0 | 0 | 0 | 0 | 0 | 2.17 | 0.0127 |
| rotor_start | carry | 1.93 | 0.13 | 37 | 225 | 1.84 | 0 | 0 | 0 | 0 | 0 | 0 | 1.93 | 0.0128 |
| z_vel_p | 2.5 | 12.3 | 0.838 | 37.3 | 225 | 2.04 | 1 | 0 | 0 | 1 | 0 | 1 | 2.29 | 0.0416 |
| z_vel_p | 5 | 2.17 | 0.592 | 37.4 | 225 | 1.79 | 0 | 0 | 0 | 0 | 0 | 0 | 2.17 | 0.0127 |
| z_vel_p | 10 | 2.08 | 0.408 | 37.4 | 225 | 1.69 | 0 | 0 | 0 | 0 | 0 | 0 | 2.08 | 0.0503 |
| z_acc_p | 0.015 | 2.23 | 0.711 | 37.4 | 225 | 1.84 | 0 | 0 | 0 | 0 | 0 | 0 | 2.23 | 0.0349 |
| z_acc_p | 0.03 | 2.17 | 0.592 | 37.4 | 225 | 1.79 | 0 | 0 | 0 | 0 | 0 | 0 | 2.17 | 0.0127 |
| z_acc_p | 0.06 | 2.09 | 0.441 | 37.4 | 225 | 1.75 | 0 | 0 | 0 | 0 | 0 | 0 | 2.09 | 0.00124 |
| theta_fw0_deg | 0 | 2.17 | 0.592 | 37.4 | 225 | 1.79 | 0 | 0 | 0 | 0 | 0 | 0 | 2.17 | 0.0127 |
| theta_fw0_deg | 8 | 2.16 | 0.579 | 37.3 | 225 | 1.77 | 0 | 0 | 0 | 0 | 0 | 0 | 2.16 | 0.0127 |

Ensemble members at the optimum:

| member | altitude_m | mass_kg | soc | headwind_ms | V0_ground_ms | x_tgt_m | x_end_m | overshoot_m | energy_kJ | excursion_m | end_sink_ms | flags | a_cap_ms2 | q_over_a_cap | alpha_max_gated_deg | vrs_min | stall_alpha_peak_deg | stall_t_above_peak_s | stall_t_above_limit_s | stall_wing_share_min |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 4789 | 24.9 | 0.777 | -0.665 | 20 | 186 | 186 | 0.19 | 44.9 | 0.442 | -0.0341 |  | 1.36 | 0.788 | 7.76 | -0.223 | 7.76 | 0 | 0 | 0.0644 |
| 1 | 1705 | 30.4 | 0.607 | -3.27 | 19.9 | 185 | 185 | 0.168 | 59.8 | 0.538 | 0.0127 |  | 1.47 | 0.726 | 7.17 | -0.198 | 7.17 | 0 | 0 | 0.0678 |
| 2 | 2055 | 22.5 | 0.844 | -1.28 | 18.1 | 153 | 154 | 0.19 | 31 | 0.263 | -0.027 |  | 1.37 | 0.781 | 7.65 | -0.219 | 7.65 | 0 | 0 | 0.092 |
| 3 | 3027 | 27.6 | 0.522 | 0.902 | 16.6 | 128 | 128 | 0.312 | 38 | 0.504 | -0.0412 |  | 1.34 | 0.801 | 11.1 | -0.213 | 11.1 | 0 | 0.1 | 0.0895 |
| 4 | 2701 | 25.4 | 0.771 | 1.17 | 16.1 | 122 | 122 | 0.386 | 31.4 | 0.379 | -0.0371 |  | 1.32 | 0.813 | 11.2 | -0.22 | 11.2 | 0 | 0.3 | 0.0995 |
| 5 | 1096 | 24.4 | 0.436 | -4.04 | 19.7 | 181 | 181 | 0.177 | 39.7 | 0.336 | 0.0119 |  | 1.47 | 0.73 | 7.11 | -0.206 | 7.11 | 0 | 0 | 0.0945 |
| 6 | 3466 | 24.8 | 0.47 | -2.96 | 20.5 | 197 | 197 | 0.168 | 45.1 | 0.42 | 0.00796 |  | 1.41 | 0.757 | 7.23 | -0.214 | 7.23 | 0 | 0 | 0.0747 |
| 7 | 3343 | 28.6 | 0.64 | 3.52 | 14.5 | 98.2 | 99.2 | 0.985 | 33.2 | 0.543 | -0.0578 |  | 1.28 | 0.833 | 11.4 | -0.218 | 11.4 | 0 | 1.08 | 0.0786 |
| 8 | 1311 | 29.4 | 0.228 | -0.26 | 16.8 | 132 | 132 | 0.209 | 43 | 0.587 | -0.0502 |  | 1.39 | 0.773 | 8.63 | -0.208 | 8.63 | 0 | 0 | 0.0788 |
| 9 | 4471 | 23.2 | 0.871 | 2.64 | 16.3 | 124 | 125 | 0.835 | 27.8 | 0.352 | -0.0422 |  | 1.28 | 0.836 | 11.3 | -0.235 | 11.3 | 0 | 1 | 0.0887 |
| 10 | 2258 | 29.1 | 0.685 | 0.564 | 16.3 | 124 | 125 | 0.265 | 40.7 | 0.495 | -0.0435 |  | 1.36 | 0.789 | 10.9 | -0.205 | 10.9 | 0 | 0 | 0.0899 |
| 11 | 4314 | 27.4 | 0.25 | -3.45 | 22.7 | 240 | 240 | 0.166 | 64.6 | 0.653 | 0.0116 |  | 1.44 | 0.743 | 7.11 | -0.217 | 7.11 | 0 | 0 | 0.0558 |
| 12 | 2379 | 25.7 | 0.705 | 2.97 | 13.6 | 86.3 | 87.4 | 1.08 | 23.9 | 0.369 | -0.0504 |  | 1.27 | 0.844 | 11.4 | -0.224 | 11.4 | 0 | 1 | 0.104 |
| 13 | 1740 | 27.8 | 0.406 | 3.91 | 12.2 | 69.2 | 70.6 | 1.35 | 22.9 | 0.466 | -0.0734 |  | 1.25 | 0.855 | 11.4 | -0.227 | 11.4 | 0 | 1.06 | 0.099 |
| 14 | 2551 | 29.6 | 0.552 | 4.65 | 12.4 | 72.2 | 73.6 | 1.35 | 27.2 | 0.547 | -0.0769 |  | 1.26 | 0.851 | 11.4 | -0.224 | 11.4 | 0 | 1.1 | 0.104 |
| 15 | 15 | 23.6 | 0.924 | 2.37 | 12.5 | 72.6 | 73.6 | 1.05 | 17.2 | 0.217 | -0.0168 |  | 1.28 | 0.838 | 11.4 | -0.223 | 11.4 | 0 | 0.88 | 0.142 |
| 16 | 3651 | 22 | 0.386 | 1.82 | 16.8 | 131 | 132 | 0.577 | 26.2 | 0.377 | -0.0394 |  | 1.28 | 0.834 | 11.3 | -0.235 | 11.3 | 0 | 0.66 | 0.0999 |
| 17 | 541 | 26.2 | 0.339 | 3.28 | 12 | 67.3 | 68.7 | 1.32 | 19.8 | 0.392 | -0.0662 |  | 1.25 | 0.856 | 11.4 | -0.228 | 11.4 | 0 | 1.02 | 0.118 |
| 18 | 1519 | 26.7 | 0.815 | -1.88 | 18.5 | 159 | 159 | 0.174 | 42.5 | 0.38 | -0.0178 |  | 1.41 | 0.759 | 7.46 | -0.207 | 7.46 | 0 | 0 | 0.0789 |
| 19 | 760 | 23.9 | 0.324 | 4.91 | 11.1 | 57.6 | 59.4 | 1.79 | 14.8 | 0.35 | -0.0771 |  | 1.21 | 0.882 | 11.5 | -0.307 | 11.5 | 0 | 1.04 | 0.168 |
| 20 | 3758 | 29.9 | 0.989 | 0.142 | 18.5 | 159 | 159 | 0.211 | 54 | 0.544 | -0.0363 |  | 1.38 | 0.774 | 10.1 | -0.206 | 10.1 | 0 | 0 | 0.0683 |
| 21 | 4681 | 21.6 | 0.484 | -4.95 | 24.6 | 283 | 284 | 0.181 | 50.1 | 0.383 | 0.009 |  | 1.42 | 0.755 | 6.98 | -0.225 | 6.98 | 0 | 0 | 0.0667 |
| 22 | 1988 | 27 | 0.286 | 1.54 | 15 | 105 | 105 | 0.482 | 30.3 | 0.486 | -0.0396 |  | 1.31 | 0.814 | 11.4 | -0.214 | 11.4 | 0 | 0.46 | 0.105 |
| 23 | 311 | 23.1 | 0.358 | 4.08 | 11.1 | 57.2 | 58.9 | 1.67 | 13.7 | 0.287 | -0.0559 |  | 1.22 | 0.874 | 11.5 | -0.297 | 11.5 | 0 | 0.98 | 0.144 |
| 24 | 2831 | 26.5 | 0.594 | -2.79 | 20.5 | 196 | 196 | 0.167 | 50.7 | 0.464 | 0.00426 |  | 1.43 | 0.751 | 7.25 | -0.21 | 7.25 | 0 | 0 | 0.0687 |
| 25 | 996 | 25.4 | 0.747 | -4.37 | 20 | 186 | 187 | 0.18 | 43.5 | 0.314 | 0.00677 |  | 1.48 | 0.721 | 7.07 | -0.203 | 7.07 | 0 | 0 | 0.091 |
| 26 | 4922 | 28.7 | 0.879 | -1.61 | 21.3 | 212 | 213 | 0.169 | 64.7 | 0.582 | -0.0213 |  | 1.42 | 0.754 | 7.46 | -0.21 | 7.46 | 0 | 0 | 0.0522 |
| 27 | 898 | 24 | 0.926 | -1.03 | 16.9 | 133 | 133 | 0.2 | 30.2 | 0.261 | -0.0303 |  | 1.38 | 0.777 | 7.75 | -0.213 | 7.75 | 0 | 0 | 0.0991 |
| 28 | 4111 | 30.3 | 0.972 | 1.89 | 16.5 | 128 | 128 | 0.484 | 46 | 0.552 | -0.0353 |  | 1.34 | 0.799 | 11.3 | -0.208 | 11.3 | 0 | 0.64 | 0.0744 |
| 29 | 3977 | 21.8 | 0.666 | -2.48 | 21 | 207 | 207 | 0.169 | 38.8 | 0.324 | -0.000881 |  | 1.38 | 0.776 | 7.33 | -0.224 | 7.33 | 0 | 0 | 0.0765 |
| 30 | 356 | 22.3 | 0.548 | -4.64 | 20.2 | 190 | 190 | 0.186 | 35.6 | 0.253 | 0.00294 |  | 1.47 | 0.728 | 7.06 | -0.208 | 7.06 | 0 | 0 | 0.104 |
| 31 | 3195 | 28.1 | 0.208 | -0.409 | 17.9 | 150 | 150 | 0.2 | 44.8 | 0.597 | -0.0471 |  | 1.37 | 0.781 | 8.08 | -0.212 | 8.08 | 0 | 0 | 0.0706 |
| 0 | 4789 | 24.9 | 0.777 | -0.665 | 20 | 99.7 | 146 | 46.8 | 26.3 | 0.491 | -0.211 | overshoot | 1.36 | 1.48 | 11.7 | -1.95 | 11.7 | 0 | 4.56 | 0.0811 |
| 1 | 1705 | 30.4 | 0.607 | -3.27 | 19.9 | 99 | 133 | 34.4 | 33.6 | 0.605 | -0.00896 | overshoot | 1.48 | 1.35 | 12.2 | -0.901 | 12.2 | 0 | 3.34 | 0.0847 |
| 2 | 2055 | 22.5 | 0.844 | -1.28 | 18.1 | 82.1 | 121 | 38.9 | 17.5 | 0.344 | -0.156 | overshoot | 1.35 | 1.48 | 11.9 | -1.62 | 11.9 | 0 | 3.52 | 0.116 |
| 3 | 3027 | 27.6 | 0.522 | 0.902 | 16.6 | 68.5 | 102 | 33.5 | 24.4 | 0.559 | 0.0636 | overshoot | 1.33 | 1.5 | 12.2 | -1.62 | 12.2 | 0 | 3.38 | 0.0865 |
| 4 | 2701 | 25.4 | 0.771 | 1.17 | 16.1 | 65.2 | 98.9 | 33.8 | 20 | 0.444 | 0.0286 | overshoot | 1.31 | 1.53 | 11.8 | -1.74 | 11.8 | 0 | 3.44 | 0.0987 |
| 5 | 1096 | 24.4 | 0.436 | -4.04 | 19.7 | 96.9 | 132 | 34.9 | 20.7 | 0.399 | 0.0858 | overshoot | 1.46 | 1.37 | 11.8 | -1.65 | 11.8 | 0 | 3.18 | 0.123 |
| 6 | 3466 | 24.8 | 0.47 | -2.96 | 20.5 | 105 | 148 | 42.4 | 24.5 | 0.469 | -0.0831 | overshoot | 1.42 | 1.41 | 11.8 | -1.9 | 11.8 | 0 | 3.46 | 0.0978 |
| 7 | 3343 | 28.6 | 0.64 | 3.52 | 14.5 | 52.6 | 81.7 | 29.2 | 23.8 | 0.6 | -0.0762 | overshoot | 1.28 | 1.56 | 12.3 | -1.66 | 12.3 | 0 | 3.44 | 0.081 |
| 8 | 1311 | 29.4 | 0.228 | -0.26 | 16.8 | 70.5 | 102 | 31 | 26.8 | 0.638 | -0.305 | overshoot | 1.38 | 1.45 | 12.4 | -1.06 | 12.4 | 0 | 3.5 | 0.0935 |
| 9 | 4471 | 23.2 | 0.871 | 2.64 | 16.3 | 66.3 | 105 | 38.7 | 18.2 | 0.471 | 0.131 | overshoot | 1.26 | 1.59 | 11.8 | -1.82 | 11.8 | 0 | 4.5 | 0.0921 |
| 10 | 2258 | 29.1 | 0.685 | 0.564 | 16.3 | 66.6 | 97.7 | 31.1 | 26.1 | 0.562 | -0.0127 | overshoot | 1.36 | 1.47 | 12.2 | -1.12 | 12.2 | 0 | 3.32 | 0.0863 |
| 11 | 4314 | 27.4 | 0.25 | -3.45 | 22.7 | 128 | 176 | 47.9 | 35.6 | 0.677 | -0.321 | overshoot | 1.45 | 1.38 | 12.4 | -2.06 | 12.4 | 0 | 3.78 | 0.0755 |
| 12 | 2379 | 25.7 | 0.705 | 2.97 | 13.6 | 46.2 | 73 | 26.8 | 16.6 | 0.437 | 0.0445 | overshoot | 1.26 | 1.59 | 12 | -1.69 | 12 | 0 | 3.3 | 0.103 |
| 13 | 1740 | 27.8 | 0.406 | 3.91 | 12.2 | 37.1 | 59.2 | 22.1 | 16.7 | 0.527 | 0.0323 | overshoot | 1.24 | 1.61 | 12.4 | -1.49 | 12.4 | 0 | 3.34 | 0.101 |
| 14 | 2551 | 29.6 | 0.552 | 4.65 | 12.4 | 38.7 | 61.4 | 22.7 | 20.2 | 0.607 | 0.0535 | overshoot | 1.25 | 1.6 | 12.5 | -1.38 | 12.5 | 0 | 3.44 | 0.105 |
| 15 | 15 | 23.6 | 0.924 | 2.37 | 12.5 | 38.9 | 61.9 | 23.1 | 11.5 | 0.298 | 0.114 | overshoot | 1.25 | 1.6 | 11.9 | -1.44 | 11.9 | 0 | 3.08 | 0.144 |
| 16 | 3651 | 22 | 0.386 | 1.82 | 16.8 | 70.3 | 111 | 40.6 | 16.7 | 0.615 | 0.143 | overshoot | 1.26 | 1.58 | 12 | -1.98 | 12 | 0 | 4.1 | 0.0974 |
| 17 | 541 | 26.2 | 0.339 | 3.28 | 12 | 36.1 | 57.6 | 21.6 | 14 | 0.456 | 0.0871 | overshoot | 1.24 | 1.61 | 12.3 | -1.57 | 12.3 | 0 | 3.16 | 0.123 |
| 18 | 1519 | 26.7 | 0.815 | -1.88 | 18.5 | 85.1 | 120 | 34.8 | 24.2 | 0.453 | -0.145 | overshoot | 1.41 | 1.42 | 11.9 | -1.38 | 11.9 | 0 | 3.08 | 0.101 |
| 19 | 760 | 23.9 | 0.324 | 4.91 | 11.1 | 30.8 | 51.3 | 20.5 | 10.1 | 0.405 | 0.0675 | overshoot | 1.19 | 1.68 | 12.1 | -1.81 | 12.1 | 0 | 3.27 | 0.174 |
| 20 | 3758 | 29.9 | 0.989 | 0.142 | 18.5 | 85.1 | 123 | 37.6 | 33.7 | 0.611 | -0.133 | overshoot | 1.38 | 1.45 | 12.1 | -1.21 | 12.1 | 0 | 3.74 | 0.0753 |
| 21 | 4681 | 21.6 | 0.484 | -4.95 | 24.6 | 152 | 213 | 61.6 | 25.5 | 0.759 | 0.166 | overshoot | 1.42 | 1.41 | 12.1 | -2.02 | 12.1 | 0 | 4.58 | 0.0917 |
| 22 | 1988 | 27 | 0.286 | 1.54 | 15 | 56 | 85.2 | 29.2 | 20.2 | 0.538 | -0.011 | overshoot | 1.31 | 1.53 | 12.3 | -1.62 | 12.3 | 0 | 3.42 | 0.0972 |
| 23 | 311 | 23.1 | 0.358 | 4.08 | 11.1 | 30.6 | 50.8 | 20.2 | 9.37 | 0.35 | 0.0413 | overshoot | 1.2 | 1.67 | 12.1 | -1.68 | 12.1 | 0 | 3.29 | 0.147 |
| 24 | 2831 | 26.5 | 0.594 | -2.79 | 20.5 | 105 | 146 | 40.8 | 28.1 | 0.519 | -0.115 | overshoot | 1.43 | 1.4 | 11.9 | -1.72 | 11.9 | 0 | 3.36 | 0.0906 |
| 25 | 996 | 25.4 | 0.747 | -4.37 | 20 | 99.8 | 134 | 34 | 22.7 | 0.39 | 0.0806 | overshoot | 1.48 | 1.35 | 11.8 | -1.39 | 11.8 | 0 | 3 | 0.119 |
| 26 | 4922 | 28.7 | 0.879 | -1.61 | 21.3 | 114 | 160 | 46.3 | 37.8 | 0.634 | -0.204 | overshoot | 1.41 | 1.41 | 12.1 | -1.69 | 12.1 | 0 | 4.38 | 0.0687 |
| 27 | 898 | 24 | 0.926 | -1.03 | 16.9 | 71.2 | 104 | 32.7 | 17.4 | 0.338 | -0.164 | overshoot | 1.36 | 1.47 | 11.8 | -1.53 | 11.8 | 0 | 3.14 | 0.122 |
| 28 | 4111 | 30.3 | 0.972 | 1.89 | 16.5 | 68.4 | 102 | 33.5 | 30.9 | 0.62 | 0.162 | overshoot | 1.34 | 1.5 | 12.2 | -1.21 | 12.2 | 0 | 3.66 | 0.0747 |
| 29 | 3977 | 21.8 | 0.666 | -2.48 | 21 | 111 | 161 | 50.5 | 21 | 0.593 | -0.0741 | overshoot | 1.37 | 1.46 | 12 | -1.83 | 12 | 0 | 4.06 | 0.1 |
| 30 | 356 | 22.3 | 0.548 | -4.64 | 20.2 | 102 | 139 | 37 | 18 | 0.319 | 0.065 | overshoot | 1.46 | 1.37 | 11.8 | -1.61 | 11.8 | 0 | 3.32 | 0.137 |
| 31 | 3195 | 28.1 | 0.208 | -0.409 | 17.9 | 80.4 | 117 | 36.3 | 27.7 | 0.637 | 0.0155 | overshoot | 1.37 | 1.46 | 12.3 | -1.52 | 12.3 | 0 | 3.54 | 0.0794 |

The price of the margin (one C search per wing limit; `unconstrained` ignores the stall and is a reference only, never a recommendation):

| kind | alpha_frac | limit_deg | Q_TRANS_DECEL_ms2 | T_bt_s | theta_A_deg | spin_min | p95_distance_m | p95_stop_time_s | mean_energy_kJ | n_fail_total | n_failing_members | max_overshoot_m | max_end_sink_ms | J | at_bound |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| unconstrained |  |  | 2.03 | 1.6 | 19.6 | 0.109 | 120 | 8.87 | 20 | 0 | 0 | 1.69 | 1.15 | 1.48 | theta_A:hi |
| curve | 1 | 15.7 | 1.3 | 2.37 | 18.2 | 0.216 | 185 | 14.1 | 26.7 | 0 | 0 | 1.97 | 0.764 | 1.76 |  |
| curve | 0.85 | 13.3 | 1.15 | 2.55 | 19.2 | 0.219 | 209 | 16.3 | 33.1 | 0 | 0 | 1.97 | 0.0148 | 2.02 | theta_A:hi;q:lo |
| headline | 0.7 | 11 | 1.07 | 2.57 | 18.7 | 0.25 | 225 | 17.6 | 37.4 | 0 | 0 | 1.79 | 0.0127 | 2.17 | spin_min:hi;q:lo |

### B1 — least energy from the AIRBRAKE exit (direct collocation)

Least bus energy from the AIRBRAKE exit to the stop, as a function of the distance cap, with the altitude held to 1 m, under a **stated wing limit**: `attached` (|alpha_w| <= 0.85 alpha_s, pitch command <= 34.4 deg); `c_envelope` (|alpha_w| <= 30 deg, pitch command <= 20 deg); `margin` (|alpha_w| <= 0.7 alpha_s, pitch command <= 20 deg). A run with no wing limit is not part of the study. **The wing limit decides the number.** B1 is clairvoyant (it knows the scenario) and open loop, and it is not a strict lower bound (B-2). The end band on the vertical speed is 1.76 m/s (C's sink tolerance; B1 bounds climb as well as sink). The entry, the sink tolerance and the rotor start are C's and D's.

**Front** (`J_point` is C's cost of the point; `j_best` marks the least per scenario and variant):

| scenario | variant | eps_x_m | converged | replay_verified | replay_late_command_ok | converged_attempts | energy_kJ | excursion_m | distance_m | t_f_s | below_vs | t_rotor_on_s | V_over_Vs_rotor_on | alpha_w_peak_above_vs_deg | J_point | j_best |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S-01 | attached | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-01 | attached | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-01 | attached | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-01 | attached | 75 | yes | yes | yes | 1 | 9.1 | 1 | 72.5 | 6.66 | no | 2.26 | 0.976 | 13.4 | 1.04 | no |
| S-01 | attached | 100 | yes | yes | yes | 3 | 9.11 | 1 | 72.5 | 6.66 | no | 2.28 | 0.972 | 13.3 | 1.04 | no |
| S-01 | attached | 300 | yes | yes | yes | 3 | 9.1 | 1 | 72.2 | 6.64 | no | 2.26 | 0.973 | 13.4 | 1.04 | yes |
| S-01 | c_envelope | 40 | yes | no |  | 3 |  |  |  |  | no |  |  |  |  | no |
| S-01 | c_envelope | 50 | yes | yes | no | 3 | 11 | 1 | 50 | 5.29 | no | 0.32 | 1.09 | 17.9 | 0.97 | yes |
| S-01 | c_envelope | 60 | yes | yes | no | 3 | 10.6 | 1 | 60 | 6.1 | no | 1.36 | 1.03 | 17.3 | 1.01 | no |
| S-01 | c_envelope | 75 | yes | yes | no | 3 | 10.5 | 1 | 64.5 | 6.45 | no | 1.98 | 0.986 | 15 | 1.03 | no |
| S-01 | c_envelope | 100 | yes | yes | no | 3 | 10.5 | 1 | 65.4 | 6.54 | no | 2.02 | 0.983 | 14.9 | 1.04 | no |
| S-01 | c_envelope | 300 | yes | yes | no | 3 | 10.5 | 1 | 66.1 | 6.6 | no | 2.06 | 0.98 | 14.9 | 1.04 | no |
| S-01 | margin | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-01 | margin | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-01 | margin | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-01 | margin | 75 | yes | yes | yes | 3 | 12.5 | 1 | 75 | 7.42 | no | 0.32 | 1.11 | 11.1 | 1.13 | yes |
| S-01 | margin | 100 | yes | yes | yes | 3 | 12.2 | 1 | 77.2 | 7.57 | no | 1.18 | 1.06 | 11 | 1.13 | no |
| S-01 | margin | 300 | yes | yes | yes | 3 | 12.2 | 1 | 77.2 | 7.57 | no | 1.18 | 1.06 | 11 | 1.13 | no |
| S-02 | attached | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | attached | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | attached | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | attached | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | attached | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | attached | 300 | yes | yes | yes | 3 | 16.3 | 1 | 123 | 9.13 | no | 2.68 | 0.969 | 13.4 | 1.44 | yes |
| S-02 | c_envelope | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | c_envelope | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | c_envelope | 60 | yes | no |  | 2 |  |  |  |  | no |  |  |  |  | no |
| S-02 | c_envelope | 75 | yes | no |  | 3 |  |  |  |  | no |  |  |  |  | no |
| S-02 | c_envelope | 100 | yes | no |  | 3 |  |  |  |  | no |  |  |  |  | no |
| S-02 | c_envelope | 300 | yes | yes | no | 2 | 17.4 | 1 | 104 | 8.22 | no | 2 | 0.999 | 17.5 | 1.37 | yes |
| S-02 | margin | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | margin | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | margin | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | margin | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | margin | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-02 | margin | 300 | yes | yes | yes | 3 | 21.2 | 1 | 131 | 10.3 | no | 0.84 | 1.08 | 11.1 | 1.58 | yes |
| S-03 | attached | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | attached | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | attached | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | attached | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | attached | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | attached | 300 | yes | yes | yes | 3 | 22.1 | 1 | 118 | 8.74 | no | 1.06 | 0.991 | 13.4 | 1.53 | yes |
| S-03 | c_envelope | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | c_envelope | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | c_envelope | 60 | yes | yes | no | 3 | 26.1 | 1 | 60 | 5.65 | no | 0.04 | 1.03 | 14.6 | 1.32 | yes |
| S-03 | c_envelope | 75 | yes | yes | no | 3 | 24.8 | 1 | 75 | 6.46 | no | 0.12 | 1.03 | 14.1 | 1.37 | no |
| S-03 | c_envelope | 100 | yes | yes | no | 3 | 23.6 | 1 | 100 | 7.85 | no | 1.02 | 0.991 | 14.5 | 1.47 | no |
| S-03 | c_envelope | 300 | yes | yes | no | 3 | 23.6 | 1 | 106 | 8.25 | no | 1.06 | 0.991 | 13.6 | 1.5 | no |
| S-03 | margin | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | margin | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | margin | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | margin | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | margin | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-03 | margin | 300 | yes | yes | yes | 3 | 28.7 | 1 | 128 | 9.99 | no | 0.6 | 1.01 | 11 | 1.71 | yes |
| S-04 | attached | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | attached | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | attached | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | attached | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | attached | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | attached | 300 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | c_envelope | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | c_envelope | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | c_envelope | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | c_envelope | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | c_envelope | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | c_envelope | 300 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | margin | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | margin | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | margin | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | margin | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | margin | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-04 | margin | 300 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-05 | attached | 40 | yes | no |  | 3 |  |  |  |  | no |  |  |  |  | no |
| S-05 | attached | 50 | yes | no |  | 2 |  |  |  |  | no |  |  |  |  | no |
| S-05 | attached | 60 | yes | no |  | 2 |  |  |  |  | no |  |  |  |  | no |
| S-05 | attached | 75 | yes | no |  | 1 |  |  |  |  | no |  |  |  |  | no |
| S-05 | attached | 100 | yes | no |  | 2 |  |  |  |  | no |  |  |  |  | no |
| S-05 | attached | 300 | yes | no |  | 2 |  |  |  |  | no |  |  |  |  | no |
| S-05 | c_envelope | 40 | yes | yes | no | 3 | 5.06 | 1 | 35.5 | 5.16 | no | 1.88 | 0.996 | 15 | 0.779 | yes |
| S-05 | c_envelope | 50 | yes | yes | no | 3 | 5.06 | 1 | 35.6 | 5.17 | no | 1.92 | 0.993 | 15 | 0.779 | no |
| S-05 | c_envelope | 60 | yes | yes | no | 3 | 5.06 | 1 | 35.6 | 5.17 | no | 1.92 | 0.993 | 15 | 0.779 | no |
| S-05 | c_envelope | 75 | yes | yes | no | 3 | 5.06 | 1 | 35.5 | 5.16 | no | 1.88 | 0.996 | 15 | 0.779 | no |
| S-05 | c_envelope | 100 | yes | yes | no | 3 | 5.06 | 1 | 35.5 | 5.17 | no | 1.88 | 0.996 | 15 | 0.779 | no |
| S-05 | c_envelope | 300 | yes | yes | no | 3 | 5.25 | 1 | 36.6 | 5.24 | no | 2.04 | 0.983 | 15.1 | 0.788 | no |
| S-05 | margin | 40 | yes | yes | yes | 3 | 6.69 | 1 | 40 | 5.6 | no | 0.32 | 1.11 | 11 | 0.834 | yes |
| S-05 | margin | 50 | yes | yes | yes | 3 | 6.46 | 1 | 41.1 | 5.72 | no | 0.92 | 1.07 | 11 | 0.835 | no |
| S-05 | margin | 60 | yes | yes | yes | 3 | 6.46 | 1 | 41.1 | 5.73 | no | 1.2 | 1.06 | 11 | 0.835 | no |
| S-05 | margin | 75 | yes | yes | yes | 3 | 6.47 | 1 | 41.1 | 5.73 | no | 1.22 | 1.06 | 11 | 0.835 | no |
| S-05 | margin | 100 | yes | yes | yes | 2 | 6.46 | 1 | 41.1 | 5.73 | no | 1.2 | 1.06 | 11 | 0.835 | no |
| S-05 | margin | 300 | yes | yes | yes | 3 | 6.46 | 1 | 41.1 | 5.73 | no | 1.22 | 1.06 | 11 | 0.835 | no |
| S-06 | attached | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | attached | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | attached | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | attached | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | attached | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | attached | 300 | yes | yes | yes | 2 | 10.9 | 1 | 129 | 9.66 | no | 4.2 | 0.96 | 13.4 | 1.36 | yes |
| S-06 | c_envelope | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | c_envelope | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | c_envelope | 60 | yes | yes | no | 2 | 13.4 | 1 | 60 | 5.66 | no | 0.32 | 1.21 | 24.5 | 1.07 | yes |
| S-06 | c_envelope | 75 | yes | no |  | 3 |  |  |  |  | no |  |  |  |  | no |
| S-06 | c_envelope | 100 | yes | no |  | 3 |  |  |  |  | no |  |  |  |  | no |
| S-06 | c_envelope | 300 | yes | no |  | 2 |  |  |  |  | no |  |  |  |  | no |
| S-06 | margin | 40 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | margin | 50 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | margin | 60 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | margin | 75 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | margin | 100 | no | no |  | 0 |  |  |  |  | no |  |  |  |  | no |
| S-06 | margin | 300 | yes | yes | yes | 1 | 14.2 | 1 | 139 | 10.9 | no | 3.58 | 1.02 | 11 | 1.48 | yes |

**No verified solution (energy blank, do not quote): S-01 attached eps_x=40; S-01 attached eps_x=50; S-01 attached eps_x=60; S-01 c_envelope eps_x=40; S-01 margin eps_x=40; S-01 margin eps_x=50; S-01 margin eps_x=60; S-02 attached eps_x=40; S-02 attached eps_x=50; S-02 attached eps_x=60; S-02 attached eps_x=75; S-02 attached eps_x=100; S-02 c_envelope eps_x=40; S-02 c_envelope eps_x=50; S-02 c_envelope eps_x=60; S-02 c_envelope eps_x=75; S-02 c_envelope eps_x=100; S-02 margin eps_x=40; S-02 margin eps_x=50; S-02 margin eps_x=60; S-02 margin eps_x=75; S-02 margin eps_x=100; S-03 attached eps_x=40; S-03 attached eps_x=50; S-03 attached eps_x=60; S-03 attached eps_x=75; S-03 attached eps_x=100; S-03 c_envelope eps_x=40; S-03 c_envelope eps_x=50; S-03 margin eps_x=40; S-03 margin eps_x=50; S-03 margin eps_x=60; S-03 margin eps_x=75; S-03 margin eps_x=100; S-04 attached eps_x=40; S-04 attached eps_x=50; S-04 attached eps_x=60; S-04 attached eps_x=75; S-04 attached eps_x=100; S-04 attached eps_x=300; S-04 c_envelope eps_x=40; S-04 c_envelope eps_x=50; S-04 c_envelope eps_x=60; S-04 c_envelope eps_x=75; S-04 c_envelope eps_x=100; S-04 c_envelope eps_x=300; S-04 margin eps_x=40; S-04 margin eps_x=50; S-04 margin eps_x=60; S-04 margin eps_x=75; S-04 margin eps_x=100; S-04 margin eps_x=300; S-05 attached eps_x=40; S-05 attached eps_x=50; S-05 attached eps_x=60; S-05 attached eps_x=75; S-05 attached eps_x=100; S-05 attached eps_x=300; S-06 attached eps_x=40; S-06 attached eps_x=50; S-06 attached eps_x=60; S-06 attached eps_x=75; S-06 attached eps_x=100; S-06 c_envelope eps_x=40; S-06 c_envelope eps_x=50; S-06 c_envelope eps_x=75; S-06 c_envelope eps_x=100; S-06 c_envelope eps_x=300; S-06 margin eps_x=40; S-06 margin eps_x=50; S-06 margin eps_x=60; S-06 margin eps_x=75; S-06 margin eps_x=100.** A point that did not converge from three starts, or whose replay in the simulator disagrees, is a solver result, not a proof that it is infeasible.

**Verified, but not robust to a 10 ms late command: S-01 c_envelope eps_x=50; S-01 c_envelope eps_x=60; S-01 c_envelope eps_x=75; S-01 c_envelope eps_x=100; S-01 c_envelope eps_x=300; S-02 c_envelope eps_x=300; S-03 c_envelope eps_x=60; S-03 c_envelope eps_x=75; S-03 c_envelope eps_x=100; S-03 c_envelope eps_x=300; S-05 c_envelope eps_x=40; S-05 c_envelope eps_x=50; S-05 c_envelope eps_x=60; S-05 c_envelope eps_x=75; S-05 c_envelope eps_x=100; S-05 c_envelope eps_x=300; S-06 c_envelope eps_x=60.** Their replay agrees when the schedule is applied as the optimiser means it (the step's mean command) and fails when each 20 ms step reads it at its start (`replay_late_command_ok` = no; the excursion is in `replay_late_command_excursion_m`): the optimum sits on the lift peak, a fraction of a degree below the stall cliff. Quote them as the best the model allows with exact timing, not as a flyable schedule.

**Same-problem gap to C** (B1 at C's own excursion and distance; B1 has no rotor lag where C has `rotor_lag_s`):

| scenario | variant | c_source | c_energy_kJ | c_excursion_m | c_distance_m | b1_converged | b1_replay_verified | b1_late_command_ok | b1_energy_kJ | energy_margin |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S-01 | attached | register parameters at q = 2 | 18 | 0.428 | 82.6 | yes | yes | yes | 11.1 | 0.384 |
| S-01 | c_envelope | register parameters at q = 2 | 18 | 0.428 | 82.6 | yes | no | None |  |  |
| S-01 | margin | register parameters at q = 2 | 18 | 0.428 | 82.6 | yes | yes | yes | 14.2 | 0.212 |
| S-02 | attached | register parameters at q = 2 | 29.4 | 0.586 | 138 | yes | yes | yes | 18.1 | 0.384 |
| S-02 | c_envelope | register parameters at q = 2 | 29.4 | 0.586 | 138 | yes | yes | no | 18 | 0.388 |
| S-02 | margin | register parameters at q = 2 | 29.4 | 0.586 | 138 | yes | yes | yes | 22.9 | 0.221 |
| S-03 | attached | register parameters at q = 2 | 39.9 | 0.822 | 140 | yes | yes | yes | 22.9 | 0.427 |
| S-03 | c_envelope | register parameters at q = 2 | 39.9 | 0.822 | 140 | yes | no | None |  |  |
| S-03 | margin | register parameters at q = 2 | 39.9 | 0.822 | 140 | yes | yes | yes | 29.4 | 0.263 |
| S-04 | attached | register parameters at q = 2 | 32.9 | 0.562 | 136 | no | no | None |  |  |
| S-04 | c_envelope | register parameters at q = 2 | 32.9 | 0.562 | 136 | no | no | None |  |  |
| S-04 | margin | register parameters at q = 2 | 32.9 | 0.562 | 136 | no | no | None |  |  |
| S-05 | attached | register parameters at q = 2 | 11.7 | 0.441 | 44.3 | yes | yes | yes | 6.08 | 0.482 |
| S-05 | c_envelope | register parameters at q = 2 | 11.7 | 0.441 | 44.3 | yes | no | None |  |  |
| S-05 | margin | register parameters at q = 2 | 11.7 | 0.441 | 44.3 | yes | yes | yes | 7.63 | 0.349 |
| S-06 | attached | register parameters at q = 2 | 19.4 | 0.705 | 143 | yes | yes | yes | 11.7 | 0.396 |
| S-06 | c_envelope | register parameters at q = 2 | 19.4 | 0.705 | 143 | yes | no | None |  |  |
| S-06 | margin | register parameters at q = 2 | 19.4 | 0.705 | 143 | yes | yes | yes | 15 | 0.227 |

**Sensitivities** (altitude band on every scenario, the rest on the first):

| scenario | sensitivity | value | variant | eps_x_m | converged | replay_verified | energy_kJ | distance_m | t_f_s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| S-01 | eps_h_m | 3 | c_envelope | 300 | yes | yes | 9.97 | 69.6 | 6.75 |
| S-01 | eps_h_m | 10 | c_envelope | 300 | yes | yes | 9.94 | 81.5 | 7.49 |
| S-01 | rotor_start | carry | attached | 300 | yes | yes | 9.1 | 72.7 | 6.68 |
| S-01 | end_vh_band_ms | 0.5 | attached | 300 | yes | yes | 9.1 | 72.2 | 6.64 |
| S-01 | theta_cmd_max_deg | 34.4 | c_envelope | 300 | yes | yes | 7.85 | 54.2 | 4.92 |
| S-02 | eps_h_m | 3 | c_envelope | 300 | yes | yes | 16.1 | 106 | 8.35 |
| S-02 | eps_h_m | 10 | c_envelope | 300 | yes | yes | 16.1 | 120 | 8.99 |
| S-03 | eps_h_m | 3 | c_envelope | 300 | yes | yes | 22.2 | 106 | 8.25 |
| S-03 | eps_h_m | 10 | c_envelope | 300 | yes | yes | 22 | 161 | 10.9 |
| S-04 | eps_h_m | 3 | c_envelope | 300 | no | no |  |  |  |
| S-04 | eps_h_m | 10 | c_envelope | 300 | no | no |  |  |  |
| S-05 | eps_h_m | 3 | c_envelope | 300 | yes | no |  |  |  |
| S-05 | eps_h_m | 10 | c_envelope | 300 | yes | no |  |  |  |
| S-06 | eps_h_m | 3 | c_envelope | 300 | yes | yes | 10.8 | 99.9 | 8.12 |
| S-06 | eps_h_m | 10 | c_envelope | 300 | yes | yes | 10.5 | 72.5 | 6.61 |


### Recommended settings

One row per ArduPilot parameter the study recommends or reads. A parameter no analysis has decided keeps its register value ("not yet analysed"). The ArduPilot default is read from `ardupilot_defaults.yaml` (Plane-4.7.2) and is a reference only: it is not tuned for this airframe and is never an input.

| parameter | unit | default_text | register | value | analysis | grade | flags | change_vs_default | note |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Q_BACKTRANS_MS | ms | 3000 | 3000 | 2574 | C | TBD |  | -426 | searched (back transition) |
| Q_A_ANGLE_MAX | deg | 30 | 30 | 18.7 | C | TBD |  | -11.3 | searched (back transition) |
| Q_M_SPIN_MIN | - | 0.15 | 0.1 | 0.25 | C | TBD | at upper bound | 0.0997 | searched (back transition) |
| Q_TRANS_DECEL | m/s^2 | 2 |  | 1.07 | C | TBD | at lower bound | -0.929 | searched (back transition): C's q is 1.07 m/s^2; the wing angle is held to 11.0 deg (0.7 x alpha_s); the stop takes 225 m and 17.6 s (95th percentile over the ensemble); D's a_rec minimum is 1.65 m/s^2, so C is -35 % above it; the worst overshoot is 1.79 m against the tolerance 2 m (PROPOSED); the worst end sink is 0.01 m/s against 1.76 m/s (PROPOSED); the margin row (worst overshoot and end sink within 75% of the tolerances) has q = 1.02 m/s^2; provisional: depends on alpha_s, which is a guess (no lift peak in the CFD up to 15 deg) |
| Q_TRANS_DECEL | m/s^2 | 2 |  | 1.65 | D | TBD |  | -0.347 | the minimum over D's ensemble (a_rec) |
| AIRSPEED_MIN | m/s | 9 | 17 | 18 | FT | TBD |  | 9 | searched (forward transition, equivalent airspeed); the register values fail the stall margin in 5 member(s) |
| Q_TRANSITION_MS | ms | 5000 | 5000 | 17000 | FT | TBD |  | 12000 | searched (forward transition, equivalent airspeed); the register values fail the stall margin in 5 member(s) |
| TKOFF_THR_MAX | - | 0 (= THR_MAX) | 0.8 | 1 | FT | TBD | at upper bound | 0 | searched (forward transition, equivalent airspeed); the register values fail the stall margin in 5 member(s) |
| Q_TRAN_PIT_MAX | deg | 3 | 3 | 6 | FT | TBD |  | 3 | searched (forward transition, equivalent airspeed); the register values fail the stall margin in 5 member(s) |

## 4. Sensitivity and what would change the conclusion

Global ranking: Morris at 6 and 12 trajectories (132 and 264 runs; 0 failed runs in the larger set were filled with the worst finite value).

| Output | Spearman | Top-3 overlap | Dominating inputs (grade, range) |
| --- | --- | --- | --- |
| excursion_m | 0.95 | 0.33 | mass (derived, 21.3312 to 31.9968); z_acc_p (assumed, 0.015 to 0.045); rotor_CT0 (derived, 0.07424 to 0.11136) |
| distance_m | 0.91 | 0.67 | headwind (assumed, -5 to 5); isa_offset (assumed, -10 to 10); z_acc_p (assumed, 0.015 to 0.045) |
| energy_J | 0.97 | 1.00 | mass (derived, 21.3312 to 31.9968); headwind (assumed, -5 to 5); rotor_CT0 (derived, 0.07424 to 0.11136) |
| max_range_km | 1.00 | 1.00 | CD0 (assumed, 0.0528 to 0.0792); wing_area (stated, 1.2936 to 1.9404); mass (derived, 21.3312 to 31.9968) |

Flagged as assumptions with their range: mass, z_acc_p, rotor_CT0, headwind, isa_offset, CD0, wing_area.

## 5. Limitations

Linked to the uncertainty register (proposal section 13).

- Back-transition: B1 and C. Forward transition: FT, when it is among the analyses; B1 does not cover it.
- The forward transition in the energy budget is a quasi-steady estimate.
- AIRSPEED_MIN: D, C, B1 and FT read it as an equivalent airspeed (D.entry_convention); Q2, which feeds E, reads it as an equivalent airspeed (Q2.feed_convention). Which one ArduPilot compares is open (O-02).
- Input set is not release grade: 72 register entries are TBD: controller.altitude_hold.K_h, controller.altitude_hold.K_i, controller.altitude_hold.K_vh, controller.altitude_hold.i_max, controller.altitude_hold.v_dem_max, controller.ardupilot.AIRSPEED_MIN, ... (72 in all)
