# Sage VTOL transition study — study (DRAFT)

`run record 332d5dcc | DRAFT | commit 9a46bd4 | 2026-11-10`

> **DRAFT.** Not for release. 72 register entries are TBD: controller.altitude_hold.K_h, controller.altitude_hold.K_i, controller.altitude_hold.K_vh, controller.altitude_hold.i_max, controller.altitude_hold.v_dem_max, controller.ardupilot.AIRSPEED_MIN, ... (72 in all); analysis 'E:0m_round_trip' ran on placeholder inputs (record is DRAFT); analysis 'E:0m_one_way' ran on placeholder inputs (record is DRAFT); analysis 'E:1500m_round_trip' ran on placeholder inputs (record is DRAFT); analysis 'E:1500m_one_way' ran on placeholder inputs (record is DRAFT); analysis 'E:2500m_round_trip' ran on placeholder inputs (record is DRAFT); analysis 'E:2500m_one_way' ran on placeholder inputs (record is DRAFT); analysis 'E:5000m_round_trip' ran on placeholder inputs (record is DRAFT); analysis 'E:5000m_one_way' ran on placeholder inputs (record is DRAFT); analysis 'FT' ran on placeholder inputs (record is DRAFT); analysis 'C' ran on placeholder inputs (record is DRAFT); analysis 'sensitivity' ran on placeholder inputs (record is DRAFT).
> The DRAFT refusal was explicitly overridden.

## 1. Summary

| Output | Nominal | Weakest grade among dominating inputs |
| --- | --- | --- |
| excursion_m | 0.1583 m | assumed |
| distance_m | 79.29 m | assumed |
| energy_J | 1.618e+04 J | assumed |
| max_range_km | 91.5 km | assumed |

Recommended Q_TRANS_DECEL (analysis D): 1.78 m/s^2.

Range margin (analysis E, reserves included):

| Range | Total [Wh] | Margin [Wh] | Fits |
| --- | --- | --- | --- |
| 100 km | 1913 | -137 | NO |
| 120 km | 2241 | -465 | NO |

Maximum range: 92 km flown.
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
| S-01 | 21.3 | 15 | 15 | 2.08 | 53.7 | 0.889 | 0.552 | 27.7 | 1.88 | -0.437 | no |
| S-01 | 26.7 | 15 | 15 | 2.22 | 50.4 | 0.103 | 0.349 | 21.2 | 2.26 | 0.128 | no |
| S-01 | 30.7 | 15 | 15 | 2.36 | 47.5 | 0.0372 | 0.377 | 20.6 | 2.42 | 0.154 | no |
| S-02 | 21.3 | 19.4 | 19.4 | 2.23 | 83.7 | 2.01 | 1.73 | 35.3 | 2.94 | -0.68 | yes |
| S-02 | 26.7 | 19.4 | 19.4 | 2.38 | 78.4 | 0.374 | 0.537 | 22 | 3.32 | 0.041 | no |
| S-02 | 30.7 | 19.4 | 19.4 | 2.54 | 73.6 | 0.0673 | 0.458 | 20.8 | 3.54 | 0.184 | no |
| S-03 | 21.3 | 19.7 | 19.7 | 2.24 | 86.6 | 2.13 | 1.88 | 36.3 | 3.02 | -0.79 | yes |
| S-03 | 26.7 | 19.7 | 19.7 | 2.39 | 81.1 | 0.403 | 0.631 | 22.3 | 3.4 | 0.00431 | no |
| S-03 | 30.7 | 19.7 | 19.7 | 2.55 | 76.2 | 0.071 | 0.513 | 20.9 | 3.64 | 0.175 | no |
| S-05 | 21.3 | 15.4 | 10.4 | 1.79 | 29.7 | 0.965 | 0 | 26.8 | 1.66 | -0.72 | yes |
| S-05 | 26.7 | 15.4 | 10.4 | 1.92 | 27.8 | 0.12 | 0.331 | 21.2 | 2.07 | -0.0799 | no |
| S-05 | 30.7 | 15.4 | 10.4 | 2.06 | 25.8 | 0.0391 | 0.383 | 20.6 | 2.22 | -0.0127 | no |
| S-06 | 21.3 | 19.4 | 19.4 | 2.23 | 83.7 | 2.01 | 1.72 | 35.8 | 2.94 | -0.847 | yes |
| S-06 | 26.7 | 19.4 | 19.4 | 2.38 | 78.4 | 0.376 | 0.641 | 22.4 | 3.32 | -0.0154 | no |
| S-06 | 30.7 | 19.4 | 19.4 | 2.53 | 73.7 | 0.0681 | 0.543 | 20.9 | 3.54 | 0.16 | no |
| S-07 | 21.3 | 16.1 | 16.1 | 2.13 | 61 | 1.14 | 0.768 | 29.6 | 2.14 | -0.527 | yes |
| S-07 | 26.7 | 16.1 | 16.1 | 2.27 | 57.2 | 0.159 | 0.361 | 21.3 | 2.54 | 0.103 | no |
| S-07 | 30.7 | 16.1 | 16.1 | 2.41 | 53.8 | 0.0439 | 0.398 | 20.7 | 2.72 | 0.164 | no |

**Read the angle-of-attack columns first.** With the altitude held the wing angle of attack equals the pitch, so the pitch ramp is a wing angle of attack well above the stall angle for most of the braking, and the rotors carry the weight. `a_eq` here is the most the idealised controller can do in that state, not validated hardware performance: the lift and drag beyond the stall angle are a flat-plate placeholder. Setting `D.enforce_alpha` holds the angle to the top of A's band and shows the other bound (see `D_sensitivity`). `D.ff_mode: lift_aware` is an oracle at its defaults (it reads the simulated lift). Altitude excursions are lower bounds: ideal actuators, assumed gains, no gusts. `D.theta_fw0_deg` is the pitch at the AIRBRAKE exit: "trim" (the pitch of level steady flight at each scenario's entry state) or a number, a placeholder until a SITL run or flight log gives it. `D.pitch_release_dps` (assumed) holds the pitch command at the entry pitch and releases it, so the start is bumpless.

**Q_TRANS_DECEL recommendation** (32 Sobol runs, seed 8, 8 corners, entry `eas`): **1.93 m/s²** at the 5th percentile (bootstrap 90% interval 1.88 to 2.11); the minimum over sample and corners is **1.78 m/s²** (set by alt 0 m, mass 21.3 kg, headwind +5.0 m/s, ISA +5.0 K, SoC 0.60). 0 of 32 sample entries are below their own 1g stall speed (flagged, not dropped).

These single numbers are valid only inside the ground speeds they cover: ground speeds 10.1 to 24.5 m/s (air speed 15.1 to 19.6, headwind -5.0 to +5.0). The curve below gives the recommendation at other ground speeds.

| n | seed | n_corners | convention | percentile | a_rec_min_ms2 | a_rec_pct_ms2 | a_rec_pct_ci_lo_ms2 | a_rec_pct_ci_hi_ms2 | a_rec_pct_half_ms2 | n_below_vs | n_nan | min_set_by | worst_corner | spearman_altitude | spearman_isa | spearman_mass | spearman_headwind | spearman_soc | ground_speed_min_ms | ground_speed_max_ms | headwind_min_ms | headwind_max_ms | covers | a_rec_design_ms2 |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 32 | 8 | 8 | eas | 5 | 1.78 | 1.93 | 1.88 | 2.11 | 2.04 | 0 | 0 | alt 0 m, mass 21.3 kg, headwind +5.0 m/s, ISA +5.0 K, SoC 0.60 | alt 0 m, mass 21.3 kg, headwind +5.0 m/s, ISA +5.0 K, SoC 0.60 | 0.193 | 0.0381 | 0.552 | -0.775 | 0.111 | 10.1 | 24.5 | -5 | 5 | ground speeds 10.1 to 24.5 m/s (air speed 15.1 to 19.6, headwind -5.0 to +5.0) |  |

**`a_rec` as a function of the ground speed at the AIRBRAKE exit** (the minimum over the named scenarios x mass factors; the air speed is the entry rule's, so the headwind is the air speed minus the ground speed; read `Q_TRANS_DECEL` at the slowest ground speed that must be covered):

| ground_speed_ms | n_runs | n_nan | headwind_ms | V0_air_ms | a_rec_min_ms2 | set_by | a_eq_median_ms2 | a_eq_max_ms2 | x_at_min_m | t_stop_at_min_s | excursion_max_m | n_sinking | note |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 5 | 18 | 0 | 10 | 15 | 1.33 | S-01 x0.8 | 1.38 | 1.51 | 9 | 2.81 | 1.68 | 0 |  |
| 6 | 18 | 0 | 9.37 | 15.4 | 1.43 | S-05 x0.8 | 1.48 | 1.64 | 12.3 | 3.11 | 2.09 | 0 |  |
| 8 | 18 | 0 | 11.4 | 19.4 | 1.6 | S-02 x0.8 | 1.69 | 1.86 | 19.7 | 3.78 | 2.13 | 4 |  |
| 10 | 18 | 0 | 9.73 | 19.7 | 1.74 | S-03 x0.8 | 1.86 | 2.04 | 28.4 | 4.37 | 2.13 | 6 |  |
| 12 | 18 | 0 | 7.73 | 19.7 | 1.87 | S-03 x0.8 | 2.01 | 2.18 | 38.3 | 4.94 | 2.13 | 6 |  |
| 14 | 18 | 0 | 5.73 | 19.7 | 1.98 | S-03 x0.8 | 2.14 | 2.31 | 49.3 | 5.49 | 2.13 | 6 |  |
| 15 | 18 | 0 | 4.73 | 19.7 | 2.03 | S-03 x0.8 | 2.19 | 2.36 | 55.2 | 5.76 | 2.13 | 5 |  |
| 16 | 18 | 0 | 3.73 | 19.7 | 2.08 | S-03 x0.8 | 2.25 | 2.41 | 61.3 | 6.03 | 2.13 | 4 |  |
| 18 | 18 | 0 | 1.73 | 19.7 | 2.17 | S-03 x0.8 | 2.34 | 2.5 | 74.4 | 6.56 | 2.13 | 3 |  |
| 20 | 18 | 0 | -0.275 | 19.7 | 2.25 | S-03 x0.8 | 2.42 | 2.57 | 88.6 | 7.08 | 2.13 | 3 |  |

One change at a time, first scenario at nominal mass (excursion columns: lower bound, ideal actuators):

| case | a_eq_ms2 | x_actual_m | alt_gain_m | alt_loss_m | vh_end_ms | alpha_max_gated_deg | t_above_alpha_08_s | t_above_alpha_s_s | a_rec_pct_ensemble_ms2 | rotor_lag_s | tau_rotor_model_s | tau_total_s |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| base | 2.22 | 50.4 | 0.103 | 0.349 | 0.128 | 21.2 | 3.9 | 2.26 | 1.93 | 0 | 0.0227 | 0.0227 |
| theta_fw0_0deg | 2 | 56.1 | 0.225 | 0.367 | 0.0041 | 22 | 2.9 | 2.02 |  | 0 | 0.0227 | 0.0227 |
| theta_fw0_4deg | 2.1 | 53.4 | 0.117 | 0.326 | 0.0769 | 21.5 | 3.16 | 2.06 |  | 0 | 0.0227 | 0.0227 |
| theta_fw0_8deg | 2.19 | 51.1 | 0.0168 | 0.387 | 0.198 | 20.8 | 3.72 | 2.26 |  | 0 | 0.0227 | 0.0227 |
| theta_fw0_12deg | 2.36 | 47.4 | 0.689 | 0.617 | -0.23 | 25.5 | 4 | 2.22 |  | 0 | 0.0227 | 0.0227 |
| rotor_lag_0s | 2.22 | 50.4 | 0.103 | 0.349 | 0.128 | 21.2 | 3.9 | 2.26 |  | 0 | 0.0227 | 0.0227 |
| rotor_lag_0.1s | 2.26 | 49.6 | 0.0991 | 0.982 | 0.555 | 22.6 | 3.82 | 2.26 |  | 0.1 | 0.0227 | 0.123 |
| rotor_lag_0.25s | 2.26 | 49.5 | 0.0975 | 2.22 | 1.29 | 27 | 3.8 | 2.24 |  | 0.25 | 0.0227 | 0.273 |
| rotor_lag_0.5s | 2.24 | 49.9 | 0.097 | 4.38 | 1.79 | 34.6 | 3.84 | 2.28 |  | 0.5 | 0.0227 | 0.523 |
| ff_mode_repo | 2.55 | 43.9 | 5.21 | 0.244 | -3.76 | 61.2 | 1.98 | 1.44 | 2.44 | 0 | 0.0227 | 0.0227 |
| lift_filter_0s | 2.22 | 50.4 | 0.103 | 0.349 | 0.128 | 21.2 | 3.9 | 2.26 |  | 0 | 0.0227 | 0.0227 |
| lift_filter_0.5s | 2.26 | 49.7 | 0.0895 | 2.97 | 1.27 | 29.5 | 3.82 | 2.28 |  | 0 | 0.0227 | 0.0227 |
| lift_scale_0.8 | 2.31 | 48.4 | 2.38 | 0.434 | -1.63 | 36.9 | 2.58 | 1.48 |  | 0 | 0.0227 | 0.0227 |
| lift_scale_1.2 | 2.26 | 49.6 | 0.0827 | 1.02 | 0.52 | 23.1 | 3.84 | 2.3 |  | 0 | 0.0227 | 0.0227 |
| enforce_alpha | 1.34 | 83.5 | 0.0269 | 0.188 | 0.00859 | 9.2 | 2.5 | 0 | 1.2 | 0 | 0.0227 | 0.0227 |

Trade curve (switch earlier = smaller a_plan = more VTOL distance and energy):

| a_plan_ms2 | a_lim_ms2 | speed_test | headwind_ms | x_plan_m | x_actual_m | overshoot_m | energy_J |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 1.5 | 1.5 | ground | -5 | 133 | 134 | 0.605 | 27230 |
| 1.5 | 3 | ground | -5 | 133 | 141 | 7.6 | 27449 |
| 1.5 | 1.5 | ground | 0 | 75 | 75.7 | 0.663 | 16724 |
| 1.5 | 3 | ground | 0 | 75 | 76.6 | 1.59 | 17942 |
| 1.5 | 1.5 | ground | 5 | 33.3 | 34.2 | 0.853 | 9154 |
| 1.5 | 3 | ground | 5 | 33.3 | 34.2 | 0.86 | 9279 |
| 1.5 | 1.5 | air | 5 | 75 | 75 | 0.0478 | 34531 |
| 1.5 | 3 | air | 5 | 75 | 75 | 0.0478 | 34531 |
| 1.55 | 1.55 | ground | -5 | 129 | 130 | 0.658 | 25707 |
| 1.55 | 3 | ground | -5 | 129 | 137 | 7.71 | 26560 |
| 1.55 | 1.55 | ground | 0 | 72.6 | 73.3 | 0.685 | 16114 |
| 1.55 | 3 | ground | 0 | 72.6 | 74.2 | 1.67 | 17316 |
| 1.55 | 1.55 | ground | 5 | 32.3 | 33.1 | 0.846 | 9011 |
| 1.55 | 3 | ground | 5 | 32.3 | 33.1 | 0.852 | 9105 |
| 1.55 | 1.55 | air | 5 | 72.6 | 72.7 | 0.13 | 32524 |
| 1.55 | 3 | air | 5 | 72.6 | 72.7 | 0.13 | 32524 |
| 1.6 | 1.6 | ground | -5 | 125 | 126 | 0.71 | 24436 |
| 1.6 | 3 | ground | -5 | 125 | 133 | 7.8 | 25735 |
| 1.6 | 1.6 | ground | 0 | 70.3 | 71 | 0.699 | 15715 |
| 1.6 | 3 | ground | 0 | 70.3 | 72.1 | 1.76 | 16760 |
| 1.6 | 1.6 | ground | 5 | 31.2 | 32.1 | 0.841 | 8893 |
| 1.6 | 3 | ground | 5 | 31.2 | 32.1 | 0.846 | 8961 |
| 1.6 | 1.6 | air | 5 | 70.3 | 70.5 | 0.208 | 30712 |
| 1.6 | 3 | air | 5 | 70.3 | 70.5 | 0.208 | 30712 |
| 1.65 | 1.65 | ground | -5 | 121 | 122 | 0.757 | 23535 |
| 1.65 | 3 | ground | -5 | 121 | 129 | 7.88 | 24971 |
| 1.65 | 1.65 | ground | 0 | 68.2 | 68.9 | 0.707 | 15486 |
| 1.65 | 3 | ground | 0 | 68.2 | 70 | 1.86 | 16268 |
| 1.65 | 1.65 | ground | 5 | 30.3 | 31.1 | 0.838 | 8794 |
| 1.65 | 3 | ground | 5 | 30.3 | 31.1 | 0.841 | 8841 |
| 1.65 | 1.65 | air | 5 | 68.2 | 68.5 | 0.279 | 29069 |
| 1.65 | 3 | air | 5 | 68.2 | 68.5 | 0.279 | 29069 |
| 1.7 | 1.7 | ground | -5 | 118 | 118 | 0.8 | 22920 |
| 1.7 | 3 | ground | -5 | 118 | 126 | 7.97 | 24274 |
| 1.7 | 1.7 | ground | 0 | 66.2 | 66.9 | 0.716 | 15337 |
| 1.7 | 3 | ground | 0 | 66.2 | 68.2 | 1.98 | 15834 |
| 1.7 | 1.7 | ground | 5 | 29.4 | 30.2 | 0.838 | 8715 |
| 1.7 | 3 | ground | 5 | 29.4 | 30.3 | 0.84 | 8744 |
| 1.7 | 1.7 | air | 5 | 66.2 | 66.5 | 0.342 | 27569 |
| 1.7 | 3 | air | 5 | 66.2 | 66.5 | 0.342 | 27569 |
| 1.75 | 1.75 | ground | -5 | 114 | 115 | 0.842 | 22439 |
| 1.75 | 3 | ground | -5 | 114 | 122 | 8.07 | 23642 |
| 1.75 | 1.75 | ground | 0 | 64.3 | 65 | 0.728 | 15205 |
| 1.75 | 3 | ground | 0 | 64.3 | 66.4 | 2.1 | 15453 |
| 1.75 | 1.75 | ground | 5 | 28.6 | 29.4 | 0.849 | 8645 |
| 1.75 | 3 | ground | 5 | 28.6 | 29.4 | 0.85 | 8660 |
| 1.75 | 1.75 | air | 5 | 64.3 | 64.7 | 0.397 | 26193 |
| 1.75 | 3 | air | 5 | 64.3 | 64.7 | 0.397 | 26193 |
| 1.8 | 1.8 | ground | -5 | 111 | 112 | 0.912 | 22020 |
| 1.8 | 3 | ground | -5 | 111 | 119 | 8.17 | 23071 |
| 1.8 | 1.8 | ground | 0 | 62.5 | 63.2 | 0.749 | 15071 |
| 1.8 | 3 | ground | 0 | 62.5 | 64.7 | 2.23 | 15119 |
| 1.8 | 1.8 | ground | 5 | 27.8 | 28.7 | 0.88 | 8554 |
| 1.8 | 3 | ground | 5 | 27.8 | 28.7 | 0.88 | 8559 |
| 1.8 | 1.8 | air | 5 | 62.5 | 62.9 | 0.445 | 24924 |
| 1.8 | 3 | air | 5 | 62.5 | 62.9 | 0.445 | 24924 |
| 1.85 | 1.85 | ground | -5 | 108 | 109 | 1 | 21659 |
| 1.85 | 3 | ground | -5 | 108 | 116 | 8.25 | 22558 |
| 1.85 | 1.85 | ground | 0 | 60.8 | 61.6 | 0.78 | 14927 |
| 1.85 | 3 | ground | 0 | 60.8 | 63.2 | 2.36 | 14826 |
| 1.85 | 1.85 | ground | 5 | 27 | 28 | 0.948 | 8420 |
| 1.85 | 3 | ground | 5 | 27 | 28 | 0.948 | 8421 |
| 1.85 | 1.85 | air | 5 | 60.8 | 61.3 | 0.486 | 23747 |
| 1.85 | 3 | air | 5 | 60.8 | 61.3 | 0.486 | 23747 |
| 1.9 | 1.9 | ground | -5 | 105 | 106 | 1.12 | 21352 |
| 1.9 | 3 | ground | -5 | 105 | 114 | 8.32 | 22100 |
| 1.9 | 1.9 | ground | 0 | 59.2 | 60 | 0.824 | 14768 |
| 1.9 | 3 | ground | 0 | 59.2 | 61.7 | 2.5 | 14572 |
| 1.9 | 1.9 | ground | 5 | 26.3 | 27.4 | 1.07 | 8233 |
| 1.9 | 3 | ground | 5 | 26.3 | 27.4 | 1.07 | 8233 |
| 1.9 | 1.9 | air | 5 | 59.2 | 59.7 | 0.522 | 22652 |
| 1.9 | 3 | air | 5 | 59.2 | 59.7 | 0.522 | 22652 |
| 1.95 | 1.95 | ground | -5 | 103 | 104 | 1.25 | 21093 |
| 1.95 | 3 | ground | -5 | 103 | 111 | 8.36 | 21694 |
| 1.95 | 1.95 | ground | 0 | 57.7 | 58.6 | 0.885 | 14589 |
| 1.95 | 3 | ground | 0 | 57.7 | 60.3 | 2.65 | 14350 |
| 1.95 | 1.95 | ground | 5 | 25.6 | 26.9 | 1.28 | 8001 |
| 1.95 | 3 | ground | 5 | 25.6 | 26.9 | 1.28 | 8001 |
| 1.95 | 1.95 | air | 5 | 57.7 | 58.2 | 0.553 | 21627 |
| 1.95 | 3 | air | 5 | 57.7 | 58.2 | 0.553 | 21627 |
| 2 | 2 | ground | -5 | 100 | 101 | 1.39 | 20871 |
| 2 | 3 | ground | -5 | 100 | 108 | 8.37 | 21335 |
| 2 | 2 | ground | 0 | 56.2 | 57.2 | 0.969 | 14391 |
| 2 | 3 | ground | 0 | 56.2 | 59.1 | 2.8 | 14158 |
| 2 | 2 | ground | 5 | 25 | 26.6 | 1.58 | 7756 |
| 2 | 3 | ground | 5 | 25 | 26.6 | 1.58 | 7756 |
| 2 | 2 | air | 5 | 56.2 | 56.8 | 0.582 | 20666 |
| 2 | 3 | air | 5 | 56.2 | 56.8 | 0.582 | 20666 |
| 2.1 | 2.1 | ground | -5 | 95.2 | 97 | 1.73 | 20518 |
| 2.1 | 3 | ground | -5 | 95.2 | 104 | 8.36 | 20741 |
| 2.1 | 2.1 | ground | 0 | 53.6 | 54.8 | 1.22 | 13967 |
| 2.1 | 3 | ground | 0 | 53.6 | 56.8 | 3.24 | 13859 |
| 2.1 | 2.1 | ground | 5 | 23.8 | 26.3 | 2.51 | 7377 |
| 2.1 | 3 | ground | 5 | 23.8 | 26.3 | 2.51 | 7377 |
| 2.1 | 2.1 | air | 5 | 53.6 | 54.2 | 0.636 | 18907 |
| 2.1 | 3 | air | 5 | 53.6 | 54.2 | 0.636 | 18907 |
| 2.25 | 2.25 | ground | -5 | 88.9 | 91.3 | 2.37 | 20143 |
| 2.25 | 3 | ground | -5 | 88.9 | 97.2 | 8.3 | 20096 |
| 2.25 | 2.25 | ground | 0 | 50 | 52.2 | 2.17 | 13378 |
| 2.25 | 3 | ground | 0 | 50 | 54.2 | 4.21 | 13608 |
| 2.25 | 2.25 | ground | 5 | 22.2 | 26.3 | 4.05 | 7111 |
| 2.25 | 3 | ground | 5 | 22.2 | 26.3 | 4.05 | 7111 |
| 2.25 | 2.25 | air | 5 | 50 | 50.7 | 0.718 | 16607 |
| 2.25 | 3 | air | 5 | 50 | 50.7 | 0.718 | 16607 |
| 2.5 | 2.5 | ground | -5 | 80 | 83.8 | 3.76 | 19602 |
| 2.5 | 3 | ground | -5 | 80 | 88.2 | 8.21 | 19482 |
| 2.5 | 2.5 | ground | 0 | 45 | 50.6 | 5.64 | 13179 |
| 2.5 | 3 | ground | 0 | 45 | 51.5 | 6.5 | 13424 |
| 2.5 | 2.5 | ground | 5 | 20 | 26.2 | 6.18 | 7045 |
| 2.5 | 3 | ground | 5 | 20 | 26.2 | 6.18 | 7045 |
| 2.5 | 2.5 | air | 5 | 45 | 45.9 | 0.895 | 13458 |
| 2.5 | 3 | air | 5 | 45 | 45.9 | 0.895 | 13458 |
| 3 | 3 | ground | -5 | 66.7 | 81.8 | 15.1 | 19068 |
| 3 | 3 | ground | 0 | 37.5 | 50.5 | 13 | 12952 |
| 3 | 3 | ground | 5 | 16.7 | 26.2 | 9.51 | 7016 |
| 3 | 3 | air | 5 | 37.5 | 38.4 | 0.924 | 10167 |
| 3.5 | 3 | ground | -5 | 57.1 | 81.6 | 24.5 | 18910 |
| 3.5 | 3.5 | ground | -5 | 57.1 | 81.8 | 24.6 | 18962 |
| 3.5 | 3 | ground | 0 | 32.1 | 50.4 | 18.2 | 12833 |
| 3.5 | 3.5 | ground | 0 | 32.1 | 50.4 | 18.3 | 12849 |
| 3.5 | 3 | ground | 5 | 14.3 | 26.2 | 11.9 | 7016 |
| 3.5 | 3.5 | ground | 5 | 14.3 | 26.2 | 11.9 | 7016 |
| 3.5 | 3 | air | 5 | 32.1 | 33 | 0.851 | 9088 |
| 3.5 | 3.5 | air | 5 | 32.1 | 33 | 0.851 | 9088 |

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

| site_amsl_m | mission | cruise_amsl_m | v_cruise_tas_ms | v_cruise_eas_ms | max_range_km | flown_total_km | fixed_Wh | fixed_out_Wh | fixed_ret_Wh | cruise_Wh | reserve_Wh | reserve_binding_leg | drop_Wh | usable_Wh | soc_start | soc_end_before_reserve | soc_end_after_reserve | window_ok | flyable | n_infeasible | first_infeasible | n_over_ceiling | a_fwd_mode | a_fwd_out_ms2 | a_fwd_ret_ms2 | a_fwd_worst_ms2 | a_plan_ms2 | a_plan_source | margin_Wh_at_100km | conflict_100km | radius_at_100km | margin_Wh_at_120km | conflict_120km | radius_at_120km |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | round_trip | 200 | 21.1 | 20.9 | 45.8 | 91.6 | 218 | 120 | 97.3 | 1408 | 150 | out | 0 | 1776 | 1 | 0.268 | 0.2 | yes | yes | 0 |  | 2 | case | 1.71 | 1.69 | -0.0806 | 1.78 | D | -137 | yes | 50 | -465 | yes | 60 |
| 0 | one_way | 200 | 21.1 | 20.9 | 92 | 92 | 120 |  |  | 1506 | 150 | leg | 0 | 1776 | 1 | 0.268 | 0.2 | yes | yes | 0 |  | 1 | case | 1.71 |  | -0.0806 | 1.78 | D | -135 | yes | 100 | -473 | yes | 120 |
| 1500 | round_trip | 1700 | 22.7 | 20.9 | 46.2 | 92.4 | 229 | 127 | 102 | 1395 | 153 | out | 0 | 1776 | 1 | 0.269 | 0.2 | yes | yes | 0 |  | 2 | case | 1.36 | 1.31 | -0.0806 | 1.78 | D | -123 | yes | 50 | -446 | yes | 60 |
| 1500 | one_way | 1700 | 22.7 | 20.9 | 93 | 93 | 127 |  |  | 1497 | 153 | leg | 0 | 1776 | 1 | 0.269 | 0.2 | yes | yes | 0 |  | 1 | case | 1.36 |  | -0.0806 | 1.78 | D | -117 | yes | 100 | -450 | yes | 120 |
| 2500 | round_trip | 2700 | 23.9 | 20.9 | 46.4 | 92.8 | 239 | 132 | 107 | 1383 | 155 | out | 0 | 1776 | 1 | 0.27 | 0.2 | yes | yes | 0 |  | 3 | case | 1.13 | 1.06 | -0.0806 | 1.78 | D | -116 | yes | 50 | -436 | yes | 60 |
| 2500 | one_way | 2700 | 23.9 | 20.9 | 93.5 | 93.5 | 132 |  |  | 1490 | 155 | leg | 0 | 1776 | 1 | 0.27 | 0.2 | yes | yes | 0 |  | 2 | case | 1.13 |  | -0.0806 | 1.78 | D | -107 | yes | 100 | -437 | yes | 120 |
| 5000 | round_trip | 5200 | 27.3 | 20.9 | 46.2 | 92.5 | 286 | 157 | 129 | 1329 | 161 | out | 0 | 1776 | 1 | 0.272 | 0.2 | yes | yes | 0 |  | 4 | case | 0.598 | 0.488 | -0.0806 | 1.78 | D | -118 | yes | 50 | -430 | yes | 60 |
| 5000 | one_way | 5200 | 27.3 | 20.9 | 94.2 | 94.2 | 157 |  |  | 1459 | 161 | leg | 0 | 1776 | 1 | 0.272 | 0.2 | yes | yes | 0 |  | 2 | case | 0.598 |  | -0.0806 | 1.78 | D | -93.5 | yes | 100 | -416 | yes | 120 |

The range is an OUTPUT: what is left after the fixed segments and the reserve goes to cruise (the radius for a round trip, which flies twice that). The requirements are distances flown (a round trip of 100 km is a 50 km radius) and comparison columns only. A pusher command above `TKOFF_THR_MAX` is reported (`over_ceiling`); a segment fails only above 1 or at another limit (`feasible`, `limit`).

Budget at the maximum range (`plausible`) and at each required distance (`range_km` is the one-way distance or the radius, `flown_km` the distance flown in all; a case shorter than the fixed segments is not tabulated):

| site_amsl_m | mission | case | flown_km | range_km | cruise_Wh | reserve_Wh | total_Wh | usable_Wh | margin_Wh | fits |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | round_trip | plausible | 91.6 | 45.8 | 1408 | 150 | 1776 | 1776 | 0 | yes |
| 0 | round_trip | required_100 | 100 | 50 | 1545 | 150 | 1913 | 1776 | -137 | no |
| 0 | round_trip | required_120 | 120 | 60 | 1873 | 150 | 2241 | 1776 | -465 | no |
| 0 | one_way | plausible | 92 | 92 | 1506 | 150 | 1776 | 1776 | -2.59e-13 | yes |
| 0 | one_way | required_100 | 100 | 100 | 1641 | 150 | 1911 | 1776 | -135 | no |
| 0 | one_way | required_120 | 120 | 120 | 1979 | 150 | 2249 | 1776 | -473 | no |
| 1500 | round_trip | plausible | 92.4 | 46.2 | 1395 | 153 | 1776 | 1776 | 0 | yes |
| 1500 | round_trip | required_100 | 100 | 50 | 1518 | 153 | 1899 | 1776 | -123 | no |
| 1500 | round_trip | required_120 | 120 | 60 | 1840 | 153 | 2222 | 1776 | -446 | no |
| 1500 | one_way | plausible | 93 | 93 | 1497 | 153 | 1776 | 1776 | 0 | yes |
| 1500 | one_way | required_100 | 100 | 100 | 1614 | 153 | 1893 | 1776 | -117 | no |
| 1500 | one_way | required_120 | 120 | 120 | 1947 | 153 | 2226 | 1776 | -450 | no |
| 2500 | round_trip | plausible | 92.8 | 46.4 | 1383 | 155 | 1776 | 1776 | 0 | yes |
| 2500 | round_trip | required_100 | 100 | 50 | 1499 | 155 | 1892 | 1776 | -116 | no |
| 2500 | round_trip | required_120 | 120 | 60 | 1818 | 155 | 2212 | 1776 | -436 | no |
| 2500 | one_way | plausible | 93.5 | 93.5 | 1490 | 155 | 1776 | 1776 | 0 | yes |
| 2500 | one_way | required_100 | 100 | 100 | 1597 | 155 | 1883 | 1776 | -107 | no |
| 2500 | one_way | required_120 | 120 | 120 | 1927 | 155 | 2213 | 1776 | -437 | no |
| 5000 | round_trip | plausible | 92.5 | 46.2 | 1329 | 161 | 1776 | 1776 | 0 | yes |
| 5000 | round_trip | required_100 | 100 | 50 | 1447 | 161 | 1894 | 1776 | -118 | no |
| 5000 | round_trip | required_120 | 120 | 60 | 1760 | 161 | 2206 | 1776 | -430 | no |
| 5000 | one_way | plausible | 94.2 | 94.2 | 1459 | 161 | 1776 | 1776 | 0 | yes |
| 5000 | one_way | required_100 | 100 | 100 | 1552 | 161 | 1870 | 1776 | -93.5 | no |
| 5000 | one_way | required_120 | 120 | 120 | 1875 | 161 | 2192 | 1776 | -416 | no |

Segments:

| site_amsl_m | mission | leg | segment | energy_Wh | soc_start | soc_end | pusher_cmd_max | over_ceiling | feasible | limit |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | round_trip | out | takeoff | 31.5 | 1 | 0.986 | 0 | no | yes |  |
| 0 | round_trip | out | fwd_transition | 12.2 | 0.986 | 0.98 | 0.8 | no | yes |  |
| 0 | round_trip | out | climb | 48.8 | 0.98 | 0.958 | 0.804 | yes | yes |  |
| 0 | round_trip | out | cruise | 725 | 0.958 | 0.632 | 0.686 | no | yes |  |
| 0 | round_trip | out | descent | 2.29 | 0.632 | 0.631 | 0 | no | yes |  |
| 0 | round_trip | out | back_transition | 4.24 | 0.631 | 0.629 |  | no | yes |  |
| 0 | round_trip | out | landing | 21.4 | 0.629 | 0.619 | 0 | no | yes |  |
| 0 | round_trip | ret | takeoff | 23.8 | 0.619 | 0.608 | 0 | no | yes |  |
| 0 | round_trip | ret | fwd_transition | 9.16 | 0.608 | 0.604 | 0.8 | no | yes |  |
| 0 | round_trip | ret | climb | 41.8 | 0.604 | 0.585 | 0.828 | yes | yes |  |
| 0 | round_trip | ret | cruise | 683 | 0.585 | 0.278 | 0.729 | no | yes |  |
| 0 | round_trip | ret | descent | 1.98 | 0.278 | 0.277 | 0 | no | yes |  |
| 0 | round_trip | ret | back_transition | 3.12 | 0.277 | 0.275 |  | no | yes |  |
| 0 | round_trip | ret | landing | 17.4 | 0.275 | 0.268 | 0 | no | yes |  |
| 0 | one_way | leg | takeoff | 31.5 | 1 | 0.986 | 0 | no | yes |  |
| 0 | one_way | leg | fwd_transition | 12.2 | 0.986 | 0.98 | 0.8 | no | yes |  |
| 0 | one_way | leg | climb | 48.8 | 0.98 | 0.958 | 0.804 | yes | yes |  |
| 0 | one_way | leg | cruise | 1506 | 0.958 | 0.28 | 0.743 | no | yes |  |
| 0 | one_way | leg | descent | 2.29 | 0.28 | 0.279 | 0 | no | yes |  |
| 0 | one_way | leg | back_transition | 4.24 | 0.279 | 0.277 |  | no | yes |  |
| 0 | one_way | leg | landing | 21.4 | 0.277 | 0.268 | 0 | no | yes |  |
| 1500 | round_trip | out | takeoff | 33.1 | 1 | 0.985 | 0 | no | yes |  |
| 1500 | round_trip | out | fwd_transition | 15.4 | 0.985 | 0.978 | 0.8 | no | yes |  |
| 1500 | round_trip | out | climb | 49.7 | 0.978 | 0.956 | 0.843 | yes | yes |  |
| 1500 | round_trip | out | cruise | 718 | 0.956 | 0.632 | 0.73 | no | yes |  |
| 1500 | round_trip | out | descent | 2.13 | 0.632 | 0.631 | 0 | no | yes |  |
| 1500 | round_trip | out | back_transition | 4.9 | 0.631 | 0.629 |  | no | yes |  |
| 1500 | round_trip | out | landing | 21.3 | 0.629 | 0.62 | 0 | no | yes |  |
| 1500 | round_trip | ret | takeoff | 24.9 | 0.62 | 0.608 | 0 | no | yes |  |
| 1500 | round_trip | ret | fwd_transition | 11.7 | 0.608 | 0.603 | 0.8 | no | yes |  |
| 1500 | round_trip | ret | climb | 42.9 | 0.603 | 0.584 | 0.87 | yes | yes |  |
| 1500 | round_trip | ret | cruise | 677 | 0.584 | 0.279 | 0.775 | no | yes |  |
| 1500 | round_trip | ret | descent | 1.84 | 0.279 | 0.278 | 0 | no | yes |  |
| 1500 | round_trip | ret | back_transition | 3.61 | 0.278 | 0.277 |  | no | yes |  |
| 1500 | round_trip | ret | landing | 17.3 | 0.277 | 0.269 | 0 | no | yes |  |
| 1500 | one_way | leg | takeoff | 33.1 | 1 | 0.985 | 0 | no | yes |  |
| 1500 | one_way | leg | fwd_transition | 15.4 | 0.985 | 0.978 | 0.8 | no | yes |  |
| 1500 | one_way | leg | climb | 49.7 | 0.978 | 0.956 | 0.843 | yes | yes |  |
| 1500 | one_way | leg | cruise | 1497 | 0.956 | 0.281 | 0.79 | no | yes |  |
| 1500 | one_way | leg | descent | 2.13 | 0.281 | 0.28 | 0 | no | yes |  |
| 1500 | one_way | leg | back_transition | 4.9 | 0.28 | 0.278 |  | no | yes |  |
| 1500 | one_way | leg | landing | 21.3 | 0.278 | 0.269 | 0 | no | yes |  |
| 2500 | round_trip | out | takeoff | 34.2 | 1 | 0.985 | 0 | no | yes |  |
| 2500 | round_trip | out | fwd_transition | 18.7 | 0.985 | 0.976 | 0.8 | no | yes |  |
| 2500 | round_trip | out | climb | 50.5 | 0.976 | 0.953 | 0.872 | yes | yes |  |
| 2500 | round_trip | out | cruise | 712 | 0.953 | 0.633 | 0.761 | no | yes |  |
| 2500 | round_trip | out | descent | 2.02 | 0.633 | 0.632 | 0 | no | yes |  |
| 2500 | round_trip | out | back_transition | 5.42 | 0.632 | 0.629 |  | no | yes |  |
| 2500 | round_trip | out | landing | 21.2 | 0.629 | 0.62 | 0 | no | yes |  |
| 2500 | round_trip | ret | takeoff | 25.8 | 0.62 | 0.608 | 0 | no | yes |  |
| 2500 | round_trip | ret | fwd_transition | 14.3 | 0.608 | 0.602 | 0.8 | no | yes |  |
| 2500 | round_trip | ret | climb | 43.6 | 0.602 | 0.582 | 0.9 | yes | yes |  |
| 2500 | round_trip | ret | cruise | 671 | 0.582 | 0.28 | 0.809 | yes | yes |  |
| 2500 | round_trip | ret | descent | 1.75 | 0.28 | 0.279 | 0 | no | yes |  |
| 2500 | round_trip | ret | back_transition | 3.99 | 0.279 | 0.277 |  | no | yes |  |
| 2500 | round_trip | ret | landing | 17.2 | 0.277 | 0.27 | 0 | no | yes |  |
| 2500 | one_way | leg | takeoff | 34.2 | 1 | 0.985 | 0 | no | yes |  |
| 2500 | one_way | leg | fwd_transition | 18.7 | 0.985 | 0.976 | 0.8 | no | yes |  |
| 2500 | one_way | leg | climb | 50.5 | 0.976 | 0.953 | 0.872 | yes | yes |  |
| 2500 | one_way | leg | cruise | 1490 | 0.953 | 0.282 | 0.825 | yes | yes |  |
| 2500 | one_way | leg | descent | 2.02 | 0.282 | 0.282 | 0 | no | yes |  |
| 2500 | one_way | leg | back_transition | 5.42 | 0.282 | 0.279 |  | no | yes |  |
| 2500 | one_way | leg | landing | 21.2 | 0.279 | 0.27 | 0 | no | yes |  |
| 5000 | round_trip | out | takeoff | 37.4 | 1 | 0.983 | 0 | no | yes |  |
| 5000 | round_trip | out | fwd_transition | 36.8 | 0.983 | 0.967 | 0.8 | no | yes |  |
| 5000 | round_trip | out | climb | 52.8 | 0.967 | 0.943 | 0.955 | yes | yes |  |
| 5000 | round_trip | out | cruise | 685 | 0.943 | 0.634 | 0.852 | yes | yes |  |
| 5000 | round_trip | out | descent | 1.77 | 0.634 | 0.633 | 0 | no | yes |  |
| 5000 | round_trip | out | back_transition | 7 | 0.633 | 0.63 |  | no | yes |  |
| 5000 | round_trip | out | landing | 21 | 0.63 | 0.621 | 0 | no | yes |  |
| 5000 | round_trip | ret | takeoff | 28.2 | 0.621 | 0.608 | 0 | no | yes |  |
| 5000 | round_trip | ret | fwd_transition | 31.5 | 0.608 | 0.594 | 0.8 | no | yes |  |
| 5000 | round_trip | ret | climb | 46 | 0.594 | 0.573 | 0.99 | yes | yes |  |
| 5000 | round_trip | ret | cruise | 644 | 0.573 | 0.283 | 0.905 | yes | yes |  |
| 5000 | round_trip | ret | descent | 1.53 | 0.283 | 0.282 | 0 | no | yes |  |
| 5000 | round_trip | ret | back_transition | 5.11 | 0.282 | 0.28 |  | no | yes |  |
| 5000 | round_trip | ret | landing | 17.1 | 0.28 | 0.272 | 0 | no | yes |  |
| 5000 | one_way | leg | takeoff | 37.4 | 1 | 0.983 | 0 | no | yes |  |
| 5000 | one_way | leg | fwd_transition | 36.8 | 0.983 | 0.967 | 0.8 | no | yes |  |
| 5000 | one_way | leg | climb | 52.8 | 0.967 | 0.943 | 0.955 | yes | yes |  |
| 5000 | one_way | leg | cruise | 1459 | 0.943 | 0.286 | 0.923 | yes | yes |  |
| 5000 | one_way | leg | descent | 1.77 | 0.286 | 0.285 | 0 | no | yes |  |
| 5000 | one_way | leg | back_transition | 7 | 0.285 | 0.282 |  | no | yes |  |
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
| register | 1500 | round_trip | 17 | 0.8 | 46.2 | 55.6 | 18.5 | 215 | 52.8 | 44 | 8.83 | 7 | 1.05 |
| register | 1500 | one_way | 17 | 0.8 | 93 | 55.6 | 18.5 | 215 | 52.8 | 44 | 8.83 | 7 | 1.05 |
| register | 2500 | round_trip | 17 | 0.8 | 46.4 | 67.2 | 22 | 260 | 63.2 | 54.3 | 8.92 | 12 | 1.06 |
| register | 2500 | one_way | 17 | 0.8 | 93.5 | 67.2 | 22 | 260 | 63.2 | 54.3 | 8.92 | 12 | 1.06 |
| register | 5000 | round_trip | 17 | 0.8 | 46.2 | 132 | 41.7 | 513 | 82 | 73.1 | 8.9 | 12 | 1.62 |
| register | 5000 | one_way | 17 | 0.8 | 94.2 | 132 | 41.7 | 513 | 82 | 73.1 | 8.9 | 12 | 1.62 |
| recommended | 0 | round_trip | 18 | 1 | 45.8 | 55 | 22.5 | 356 | 64.3 | 33.6 | 30.7 | 9 | 0.855 |
| recommended | 0 | one_way | 18 | 1 | 92 | 55 | 22.5 | 356 | 64.3 | 33.6 | 30.7 | 9 | 0.855 |
| recommended | 1500 | round_trip | 18 | 1 | 46.3 | 62.1 | 24 | 398 | 69.8 | 38.5 | 31.2 | 7 | 0.89 |
| recommended | 1500 | one_way | 18 | 1 | 93 | 62.1 | 24 | 398 | 69.8 | 38.5 | 31.2 | 7 | 0.89 |
| recommended | 2500 | round_trip | 18 | 1 | 46.5 | 68.2 | 25.4 | 432 | 77.8 | 45.3 | 32.5 | 12 | 0.876 |
| recommended | 2500 | one_way | 18 | 1 | 93.7 | 68.2 | 25.4 | 432 | 77.8 | 45.3 | 32.5 | 12 | 0.876 |
| recommended | 5000 | round_trip | 18 | 1 | 47 | 91.7 | 30.9 | 557 | 90.8 | 57.3 | 33.4 | 12 | 1.01 |
| recommended | 5000 | one_way | 18 | 1 | 94.9 | 91.7 | 30.9 | 557 | 90.8 | 57.3 | 33.4 | 12 | 1.01 |

### C — ArduPilot parameters from the AIRBRAKE exit

**Configuration: enforced.** The wing angle is held to |alpha_w| <= 10.99 deg (0.7 x alpha_s, `margin_alpha_frac`); this is the headline and the one written to `settings_recommended`.

Cost 321.414 (register values, q = 2) → **1.914**; 8-seed spread **0.74%** (T-12 limit 1%). Entry is the AIRBRAKE exit like D; four variables are searched, `Q_TRANS_DECEL` (q) among them, which sets the stopping point and is the cascade's acceleration limit. The vertical loop is ArduPilot's structure with its default values; rotor lag 0.25 s, rotors `idle` at the exit.

**The overshoot tolerance decides the answer.** A run that stops more than 2 m past its stopping point, or ends sinking faster than 1.76 m/s, is a failure; both tolerances are proposals. The pitch ramp can deliver 1.21 to 1.56 m/s^2 across the ensemble (`a_cap_ms2`, median 1.38); the optimum asks for `q` = 1.11, so it is below every member's ceiling. Variables within 10% of a bound: `q` (lo).

`theta_A` against its bound is a stalled-wing brake as in D (check D's alpha and VRS columns in `C_runs`); it is not a recommendation to fly at that angle. `rotor_start` idle is read from the ArduPilot source and not checked in SITL or a log (O-13); the Z-loop defaults are master 4.8.0-dev, not a pinned tag (O-08).

| case | T_bt_s | theta_A_deg | spin_min | Q_TRANS_DECEL_ms2 | J | J_start | seed_spread | at_bound | p95_excursion_m | mean_energy_kJ | p95_distance_m | max_overshoot_m | n_overshoot | n_sinking | n_other_flags | theta_A_hi_deg | theta_A_cfg_hi_deg | ptch_lim_deg | rotor_lag_s | p95_stop_time_s | stall_alpha_peak_deg_max | stall_t_above_peak_s_max | stall_t_above_peak_s_mean | stall_t_above_limit_s_max | stall_wing_share_min | stall_members_above_peak | stall_members | alpha_enforced | alpha_limit_deg | alpha_frac | alpha_peak_deg | alpha_s_deg | blend_M | cl_max | rotor_constant_coeffs | os_tol_m | sink_tol_ms | max_end_sink_ms | os_margin_m | sink_margin_ms | at_edge | n_failing_members | n_not_stopped | n_fail_total | J_smooth |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| optimum | 3.19 | 16.8 | 0.091 | 1.11 | 1.91 | 321 | 0.00738 | q:lo | 0.283 | 34.6 | 216 | 1.99 | 0 | 0 | 0 | 20 | 30 | 20 | 0.25 | 17 | 11.6 | 0 | 0 | 1.08 | 0.054 | 0 | 32 | yes | 11 | 0.7 | 13.4 | 15.7 | 70 | 1.44 | yes | 2 | 1.76 | 0.0135 | 0.00959 | 1.75 | yes | 0 | 0 | 0 | 1.91 |
| register at q = 2 | 3 | 30 | 0.1 | 2 | 321 |  |  |  | 0.362 | 20.4 | 165 | 59.5 | 32 | 0 | 0 |  |  |  |  | 12 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  | 2 | 1.76 | 0.162 | -57.5 | 1.6 | yes | 32 | 0 | 32 | 1.41 |
| ardupilot_default | 3 | 30 | 0.15 | 2 | 321 |  |  |  | 0.431 | 20.5 | 165 | 59.3 | 32 | 0 | 0 |  |  |  |  | 12 | 12.1 | 0 | 0 | 4.46 | 0.068 | 0 | 32 |  |  |  |  |  |  |  |  | 2 | 1.76 | 0.162 | -57.3 | 1.6 | yes | 32 | 0 | 32 | 1.45 |
| optimum_margin | 3.19 | 16.8 | 0.091 | 1.06 | 2.02 |  |  |  | 0.283 | 37.2 | 226 | 1.39 | 0 | 0 | 0 |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  |  | 2 | 1.76 | 0.0128 | 0.609 | 1.75 | no | 0 | 0 | 0 | 2.02 |

_Q_A_ANGLE_MAX is searched up to 20 deg = min(30, PTCH_LIM_MAX_DEG 20): the pitch cap is min(Q_A_ANGLE_MAX, PTCH_LIM_MAX_DEG), so the 30 deg limit (DEC-7) is now the pitch limit._

Search:

| seed | best_cost | evaluations | T_bt_s | theta_A_deg | spin_min | Q_TRANS_DECEL_ms2 |
| --- | --- | --- | --- | --- | --- | --- |
| 1 | 1.92 | 160 | 2.67 | 20 | 0.104 | 1.11 |
| 2 | 1.92 | 160 | 5.07 | 19.6 | 0.0918 | 1.11 |
| 3 | 1.92 | 160 | 3.07 | 18.6 | 0.143 | 1.11 |
| 4 | 1.92 | 160 | 4.5 | 15.1 | 0.0673 | 1.11 |
| 5 | 1.92 | 160 | 5.73 | 14.8 | 0.0623 | 1.11 |
| 6 | 1.91 | 160 | 3.19 | 16.8 | 0.091 | 1.11 |
| 7 | 1.91 | 160 | 1.09 | 12.2 | 0.0681 | 1.11 |
| 8 | 1.93 | 160 | 4.31 | 19.6 | 0.164 | 1.11 |

Energy against distance with the optimum's ramp and q varied:

| case | J | p95_excursion_m | mean_energy_kJ | p95_distance_m | max_overshoot_m | n_overshoot | n_sinking | n_other_flags | n_failing_members | n_not_stopped | n_fail_total | J_smooth | max_end_sink_ms | q | grid |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| q = 1 | 2.16 | 0.283 | 41 | 241 | 0.781 | 0 | 0 | 0 | 0 | 0 | 0 | 2.16 | 0.0116 | 1 | coarse |
| q = 1.5 | 321 | 0.282 | 22.7 | 168 | 15.8 | 32 | 0 | 0 | 32 | 0 | 32 | 1.43 | 0.101 | 1.5 | coarse |
| q = 2 | 321 | 0.374 | 21.2 | 167 | 62.1 | 32 | 0 | 0 | 32 | 0 | 32 | 1.44 | 0.121 | 2 | coarse |
| q = 2.25 | 322 | 0.69 | 21 | 167 | 79.5 | 32 | 0 | 0 | 32 | 0 | 32 | 1.6 | 0.135 | 2.25 | coarse |
| q = 2.5 | 322 | 0.737 | 20.9 | 167 | 93.2 | 32 | 0 | 0 | 32 | 0 | 32 | 1.62 | 0.137 | 2.5 | coarse |
| q = 2.75 | 322 | 0.737 | 20.9 | 167 | 105 | 32 | 0 | 0 | 32 | 0 | 32 | 1.62 | 0.136 | 2.75 | coarse |
| q = 3 | 322 | 0.733 | 20.8 | 167 | 114 | 32 | 0 | 0 | 32 | 0 | 32 | 1.62 | 0.135 | 3 | coarse |
| q = 1.014 | 2.13 | 0.283 | 40.1 | 237 | 0.902 | 0 | 0 | 0 | 0 | 0 | 0 | 2.13 | 0.012 | 1.01 | fine |
| q = 1.064 | 2.02 | 0.283 | 37.2 | 226 | 1.39 | 0 | 0 | 0 | 0 | 0 | 0 | 2.02 | 0.0128 | 1.06 | fine |
| q = 1.114 | 1.91 | 0.283 | 34.6 | 216 | 1.99 | 0 | 0 | 0 | 0 | 0 | 0 | 1.91 | 0.0135 | 1.11 | fine |
| q = 1.164 | 61.8 | 0.283 | 32.2 | 207 | 2.7 | 6 | 0 | 0 | 6 | 0 | 6 | 1.82 | 0.0154 | 1.16 | fine |
| q = 1.214 | 102 | 0.283 | 30.1 | 198 | 3.51 | 10 | 0 | 0 | 10 | 0 | 10 | 1.73 | 0.103 | 1.21 | fine |
| q = 1.264 | 132 | 0.283 | 28.2 | 191 | 4.41 | 13 | 0 | 0 | 13 | 0 | 13 | 1.66 | 0.109 | 1.26 | fine |
| q = 1.314 | 142 | 0.283 | 26.5 | 183 | 5.87 | 14 | 0 | 0 | 14 | 0 | 14 | 1.59 | 0.102 | 1.31 | fine |

One at a time at the optimum:

| parameter | value | J | p95_excursion_m | mean_energy_kJ | p95_distance_m | max_overshoot_m | n_overshoot | n_sinking | n_other_flags | n_failing_members | n_not_stopped | n_fail_total | J_smooth | max_end_sink_ms |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| rotor_lag_s | 0 | 1.91 | 0.266 | 34.6 | 216 | 1.94 | 0 | 0 | 0 | 0 | 0 | 0 | 1.91 | 8.03e-05 |
| rotor_lag_s | 0.25 | 1.91 | 0.283 | 34.6 | 216 | 1.99 | 0 | 0 | 0 | 0 | 0 | 0 | 1.91 | 0.0135 |
| rotor_lag_s | 0.5 | 12 | 0.376 | 34.6 | 216 | 2.1 | 1 | 0 | 0 | 1 | 0 | 1 | 1.96 | 0.158 |
| rotor_start | idle | 1.91 | 0.283 | 34.6 | 216 | 1.99 | 0 | 0 | 0 | 0 | 0 | 0 | 1.91 | 0.0135 |
| rotor_start | carry | 1.92 | 0.287 | 34.6 | 216 | 1.98 | 0 | 0 | 0 | 0 | 0 | 0 | 1.92 | 0.0135 |
| z_vel_p | 2.5 | 32.1 | 0.615 | 34.3 | 216 | 2.38 | 3 | 0 | 0 | 3 | 0 | 3 | 2.07 | 0.0129 |
| z_vel_p | 5 | 1.91 | 0.283 | 34.6 | 216 | 1.99 | 0 | 0 | 0 | 0 | 0 | 0 | 1.91 | 0.0135 |
| z_vel_p | 10 | 1.84 | 0.123 | 34.7 | 216 | 1.84 | 0 | 0 | 0 | 0 | 0 | 0 | 1.84 | 0.0512 |
| z_acc_p | 0.015 | 11.9 | 0.321 | 34.6 | 216 | 2.03 | 1 | 0 | 0 | 1 | 0 | 1 | 1.93 | 0.0384 |
| z_acc_p | 0.03 | 1.91 | 0.283 | 34.6 | 216 | 1.99 | 0 | 0 | 0 | 0 | 0 | 0 | 1.91 | 0.0135 |
| z_acc_p | 0.06 | 1.9 | 0.246 | 34.6 | 216 | 1.91 | 0 | 0 | 0 | 0 | 0 | 0 | 1.9 | 0.00125 |
| theta_fw0_deg | 0 | 22.1 | 0.716 | 35.2 | 216 | 2.49 | 2 | 0 | 0 | 2 | 0 | 2 | 2.14 | 0.0135 |
| theta_fw0_deg | 8 | 1.93 | 0.318 | 34.7 | 216 | 1.95 | 0 | 0 | 0 | 0 | 0 | 0 | 1.93 | 0.0135 |

Ensemble members at the optimum:

| member | altitude_m | mass_kg | soc | headwind_ms | V0_ground_ms | x_tgt_m | x_end_m | overshoot_m | energy_kJ | excursion_m | end_sink_ms | flags | a_cap_ms2 | q_over_a_cap | alpha_max_gated_deg | vrs_min | stall_alpha_peak_deg | stall_t_above_peak_s | stall_t_above_limit_s | stall_wing_share_min |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| 0 | 4789 | 24.9 | 0.777 | -0.665 | 20 | 179 | 179 | 0.239 | 41.4 | 0.227 | -0.0389 |  | 1.37 | 0.816 | 8.37 | -2.34 | 8.37 | 0 | 0 | 0.0675 |
| 1 | 1705 | 30.4 | 0.607 | -3.27 | 19.9 | 178 | 178 | 0.209 | 55.6 | 0.24 | 0.013 |  | 1.54 | 0.726 | 11 | -2.92 | 11 | 0 | 0.36 | 0.0696 |
| 2 | 2055 | 22.5 | 0.844 | -1.28 | 18.1 | 147 | 148 | 0.24 | 28.6 | 0.177 | -0.03 |  | 1.35 | 0.824 | 8.13 | -1.99 | 8.13 | 0 | 0 | 0.0963 |
| 3 | 3027 | 27.6 | 0.522 | 0.902 | 16.6 | 123 | 123 | 0.449 | 34.9 | 0.243 | -0.047 |  | 1.38 | 0.808 | 11.2 | -2.64 | 11.2 | 0 | 0.26 | 0.0905 |
| 4 | 2701 | 25.4 | 0.771 | 1.17 | 16.1 | 117 | 118 | 0.575 | 28.8 | 0.203 | -0.0406 |  | 1.34 | 0.835 | 11.3 | -2.34 | 11.3 | 0 | 0.46 | 0.0991 |
| 5 | 1096 | 24.4 | 0.436 | -4.04 | 19.7 | 174 | 174 | 0.218 | 36.9 | 0.206 | 0.0135 |  | 1.46 | 0.761 | 7.96 | -2.29 | 7.96 | 0 | 0 | 0.0974 |
| 6 | 3466 | 24.8 | 0.47 | -2.96 | 20.5 | 189 | 189 | 0.21 | 41.8 | 0.232 | 0.00742 |  | 1.43 | 0.782 | 8.12 | -2.38 | 8.11 | 0 | 0 | 0.0772 |
| 7 | 3343 | 28.6 | 0.64 | 3.52 | 14.5 | 94.3 | 95.7 | 1.37 | 30.9 | 0.246 | -0.0618 |  | 1.34 | 0.834 | 11.3 | -2.73 | 11.3 | 0 | 1.08 | 0.078 |
| 8 | 1311 | 29.4 | 0.228 | -0.26 | 16.8 | 127 | 127 | 0.262 | 39.7 | 0.278 | -0.0595 |  | 1.44 | 0.775 | 10.5 | -2.94 | 10.5 | 0 | 0 | 0.0863 |
| 9 | 4471 | 23.2 | 0.871 | 2.64 | 16.3 | 119 | 120 | 1.23 | 25.5 | 0.203 | -0.04 |  | 1.27 | 0.88 | 11.3 | -2.11 | 11.3 | 0 | 0.98 | 0.0884 |
| 10 | 2258 | 29.1 | 0.685 | 0.564 | 16.3 | 119 | 120 | 0.373 | 37.6 | 0.228 | -0.0472 |  | 1.41 | 0.788 | 11.1 | -2.76 | 11.1 | 0 | 0.1 | 0.0899 |
| 11 | 4314 | 27.4 | 0.25 | -3.45 | 22.7 | 230 | 231 | 0.207 | 59.9 | 0.31 | 0.0122 |  | 1.47 | 0.757 | 9.47 | -2.78 | 9.47 | 0 | 0 | 0.0575 |
| 12 | 2379 | 25.7 | 0.705 | 2.97 | 13.6 | 82.9 | 84.4 | 1.45 | 22.1 | 0.201 | -0.0468 |  | 1.29 | 0.862 | 11.3 | -2.41 | 11.3 | 0 | 0.98 | 0.104 |
| 13 | 1740 | 27.8 | 0.406 | 3.91 | 12.2 | 66.5 | 68.1 | 1.55 | 21.3 | 0.237 | -0.0807 |  | 1.3 | 0.857 | 11.5 | -2.67 | 11.5 | 0 | 1.04 | 0.102 |
| 14 | 2551 | 29.6 | 0.552 | 4.65 | 12.4 | 69.4 | 70.9 | 1.48 | 25.5 | 0.248 | -0.0893 |  | 1.32 | 0.842 | 11.4 | -2.86 | 11.4 | 0 | 1.08 | 0.105 |
| 15 | 15 | 23.6 | 0.924 | 2.37 | 12.5 | 69.8 | 71.1 | 1.37 | 16 | 0.154 | -0.0135 |  | 1.27 | 0.879 | 11.4 | -2.08 | 11.4 | 0 | 0.88 | 0.14 |
| 16 | 3651 | 22 | 0.386 | 1.82 | 16.8 | 126 | 127 | 0.882 | 23.9 | 0.24 | -0.0431 |  | 1.27 | 0.88 | 11.3 | -2.07 | 11.3 | 0 | 0.84 | 0.099 |
| 17 | 541 | 26.2 | 0.339 | 3.28 | 12 | 64.7 | 66.3 | 1.57 | 18.4 | 0.222 | -0.0673 |  | 1.28 | 0.868 | 11.5 | -2.48 | 11.5 | 0 | 1 | 0.118 |
| 18 | 1519 | 26.7 | 0.815 | -1.88 | 18.5 | 153 | 153 | 0.219 | 39.4 | 0.197 | -0.0204 |  | 1.44 | 0.774 | 9.11 | -2.47 | 9.1 | 0 | 0 | 0.0819 |
| 19 | 760 | 23.9 | 0.324 | 4.91 | 11.1 | 55.3 | 57.3 | 1.99 | 13.6 | 0.219 | -0.0737 |  | 1.21 | 0.922 | 11.6 | -2.22 | 11.6 | 0 | 1.07 | 0.17 |
| 20 | 3758 | 29.9 | 0.989 | 0.142 | 18.5 | 153 | 153 | 0.283 | 49.9 | 0.227 | -0.0401 |  | 1.44 | 0.775 | 10.8 | -2.84 | 10.8 | 0 | 0 | 0.0714 |
| 21 | 4681 | 21.6 | 0.484 | -4.95 | 24.6 | 272 | 273 | 0.221 | 46.6 | 0.244 | 0.0111 |  | 1.4 | 0.794 | 7.38 | -1.99 | 7.38 | 0 | 0 | 0.0686 |
| 22 | 1988 | 27 | 0.286 | 1.54 | 15 | 101 | 101 | 0.718 | 27.8 | 0.252 | -0.0478 |  | 1.35 | 0.826 | 11.3 | -2.63 | 11.3 | 0 | 0.64 | 0.103 |
| 23 | 311 | 23.1 | 0.358 | 4.08 | 11.1 | 55 | 56.9 | 1.91 | 12.6 | 0.199 | -0.0467 |  | 1.21 | 0.921 | 11.6 | -2.12 | 11.6 | 0 | 1.08 | 0.146 |
| 24 | 2831 | 26.5 | 0.594 | -2.79 | 20.5 | 188 | 188 | 0.209 | 47 | 0.232 | 0.00333 |  | 1.45 | 0.767 | 9.03 | -2.53 | 9.02 | 0 | 0 | 0.071 |
| 25 | 996 | 25.4 | 0.747 | -4.37 | 20 | 179 | 179 | 0.22 | 40.5 | 0.184 | 0.00788 |  | 1.49 | 0.746 | 8.45 | -2.33 | 8.45 | 0 | 0 | 0.0944 |
| 26 | 4922 | 28.7 | 0.879 | -1.61 | 21.3 | 204 | 204 | 0.213 | 59.9 | 0.247 | -0.0242 |  | 1.45 | 0.766 | 10.2 | -2.78 | 10.1 | 0 | 0 | 0.054 |
| 27 | 898 | 24 | 0.926 | -1.03 | 16.9 | 128 | 128 | 0.252 | 28 | 0.166 | -0.0334 |  | 1.38 | 0.81 | 8.24 | -2.13 | 8.24 | 0 | 0 | 0.102 |
| 28 | 4111 | 30.3 | 0.972 | 1.89 | 16.5 | 123 | 124 | 0.725 | 42.6 | 0.228 | -0.0412 |  | 1.4 | 0.794 | 11.3 | -2.87 | 11.3 | 0 | 0.82 | 0.0735 |
| 29 | 3977 | 21.8 | 0.666 | -2.48 | 21 | 199 | 199 | 0.212 | 35.9 | 0.209 | -0.0022 |  | 1.36 | 0.818 | 7.75 | -1.98 | 7.75 | 0 | 0 | 0.0788 |
| 30 | 356 | 22.3 | 0.548 | -4.64 | 20.2 | 183 | 183 | 0.226 | 33.1 | 0.183 | 0.00344 |  | 1.45 | 0.77 | 7.48 | -2.02 | 7.48 | 0 | 0 | 0.108 |
| 31 | 3195 | 28.1 | 0.208 | -0.409 | 17.9 | 144 | 144 | 0.25 | 41.3 | 0.288 | -0.0551 |  | 1.42 | 0.785 | 9.83 | -2.82 | 9.83 | 0 | 0 | 0.0774 |
| 0 | 4789 | 24.9 | 0.777 | -0.665 | 20 | 99.7 | 143 | 43.5 | 24.5 | 0.242 | -0.222 | overshoot | 1.38 | 1.45 | 11.8 | -1.85 | 11.8 | 0 | 4.28 | 0.0824 |
| 1 | 1705 | 30.4 | 0.607 | -3.27 | 19.9 | 99 | 127 | 27.7 | 30.1 | 0.222 | -0.00693 | overshoot | 1.56 | 1.28 | 11.7 | -1.98 | 11.7 | 0 | 3.08 | 0.0916 |
| 2 | 2055 | 22.5 | 0.844 | -1.28 | 18.1 | 82.1 | 119 | 36.5 | 16.4 | 0.191 | -0.154 | overshoot | 1.37 | 1.46 | 11.8 | -1.62 | 11.8 | 0 | 3.4 | 0.116 |
| 3 | 3027 | 27.6 | 0.522 | 0.902 | 16.6 | 68.5 | 98.2 | 29.7 | 22 | 0.236 | -0.0385 | overshoot | 1.38 | 1.45 | 11.8 | -1.84 | 11.8 | 0 | 3.52 | 0.0868 |
| 4 | 2701 | 25.4 | 0.771 | 1.17 | 16.1 | 65.2 | 95.9 | 30.8 | 18.3 | 0.185 | 0.0218 | overshoot | 1.34 | 1.49 | 11.8 | -1.52 | 11.8 | 0 | 3.3 | 0.0973 |
| 5 | 1096 | 24.4 | 0.436 | -4.04 | 19.7 | 96.9 | 128 | 31.5 | 19.4 | 0.183 | 0.104 | overshoot | 1.49 | 1.34 | 11.8 | -1.42 | 11.8 | 0 | 3.14 | 0.126 |
| 6 | 3466 | 24.8 | 0.47 | -2.96 | 20.5 | 105 | 145 | 39.2 | 22.9 | 0.214 | -0.0736 | overshoot | 1.44 | 1.39 | 11.8 | -1.59 | 11.8 | 0 | 3.42 | 0.0956 |
| 7 | 3343 | 28.6 | 0.64 | 3.52 | 14.5 | 52.6 | 78 | 25.4 | 21.1 | 0.236 | -0.0775 | overshoot | 1.34 | 1.5 | 11.8 | -1.94 | 11.8 | 0 | 3.6 | 0.0779 |
| 8 | 1311 | 29.4 | 0.228 | -0.26 | 16.8 | 70.5 | 96.7 | 26.2 | 23.5 | 0.26 | -0.189 | overshoot | 1.45 | 1.38 | 11.8 | -2.07 | 11.8 | 0 | 2.92 | 0.0966 |
| 9 | 4471 | 23.2 | 0.871 | 2.64 | 16.3 | 66.3 | 103 | 36.6 | 16.9 | 0.266 | 0.128 | overshoot | 1.28 | 1.57 | 11.8 | -1.82 | 11.8 | 0 | 4.3 | 0.0908 |
| 10 | 2258 | 29.1 | 0.685 | 0.564 | 16.3 | 66.6 | 93.1 | 26.5 | 23.2 | 0.21 | -0.11 | overshoot | 1.42 | 1.41 | 11.7 | -1.86 | 11.7 | 0 | 3.4 | 0.0908 |
| 11 | 4314 | 27.4 | 0.25 | -3.45 | 22.7 | 128 | 172 | 43.7 | 33.2 | 0.322 | -0.157 | overshoot | 1.49 | 1.35 | 12 | -2.09 | 12 | 0 | 3.86 | 0.0692 |
| 12 | 2379 | 25.7 | 0.705 | 2.97 | 13.6 | 46.2 | 70.3 | 24.1 | 14.8 | 0.186 | 0.0514 | overshoot | 1.29 | 1.55 | 11.8 | -1.51 | 11.8 | 0 | 3.24 | 0.105 |
| 13 | 1740 | 27.8 | 0.406 | 3.91 | 12.2 | 37.1 | 56.3 | 19.2 | 14.2 | 0.225 | 0.0392 | overshoot | 1.3 | 1.54 | 11.8 | -1.77 | 11.8 | 0 | 3.08 | 0.107 |
| 14 | 2551 | 29.6 | 0.552 | 4.65 | 12.4 | 38.7 | 57.9 | 19.2 | 16.8 | 0.226 | 0.027 | overshoot | 1.32 | 1.51 | 11.7 | -1.98 | 11.7 | 0 | 3.06 | 0.113 |
| 15 | 15 | 23.6 | 0.924 | 2.37 | 12.5 | 38.9 | 60 | 21.1 | 10.4 | 0.142 | 0.118 | overshoot | 1.28 | 1.56 | 11.8 | -1.36 | 11.8 | 0 | 2.92 | 0.144 |
| 16 | 3651 | 22 | 0.386 | 1.82 | 16.8 | 70.3 | 109 | 39 | 15.5 | 0.434 | 0.133 | overshoot | 1.28 | 1.57 | 12 | -1.98 | 12 | 0 | 4.04 | 0.0969 |
| 17 | 541 | 26.2 | 0.339 | 3.28 | 12 | 36.1 | 55.2 | 19.2 | 12.1 | 0.212 | 0.111 | overshoot | 1.28 | 1.56 | 11.9 | -1.58 | 11.9 | 0 | 2.78 | 0.126 |
| 18 | 1519 | 26.7 | 0.815 | -1.88 | 18.5 | 85.1 | 116 | 30.8 | 22.3 | 0.178 | -0.122 | overshoot | 1.45 | 1.38 | 11.8 | -1.56 | 11.8 | 0 | 3.26 | 0.101 |
| 19 | 760 | 23.9 | 0.324 | 4.91 | 11.1 | 30.8 | 49.9 | 19 | 8.8 | 0.249 | 0.0445 | overshoot | 1.22 | 1.64 | 12.1 | -1.77 | 12.1 | 0 | 3.1 | 0.175 |
| 20 | 3758 | 29.9 | 0.989 | 0.142 | 18.5 | 85.1 | 118 | 32.4 | 30.5 | 0.209 | -0.063 | overshoot | 1.44 | 1.38 | 11.7 | -1.96 | 11.7 | 0 | 3.52 | 0.0714 |
| 21 | 4681 | 21.6 | 0.484 | -4.95 | 24.6 | 152 | 211 | 59.3 | 24.4 | 0.605 | 0.162 | overshoot | 1.43 | 1.4 | 12 | -2.01 | 12 | 0 | 4.46 | 0.0921 |
| 22 | 1988 | 27 | 0.286 | 1.54 | 15 | 56 | 81.7 | 25.7 | 17.8 | 0.246 | 0.0439 | overshoot | 1.35 | 1.48 | 11.9 | -1.8 | 11.9 | 0 | 3.32 | 0.104 |
| 23 | 311 | 23.1 | 0.358 | 4.08 | 11.1 | 30.6 | 49.5 | 18.9 | 8.27 | 0.217 | 0.0336 | overshoot | 1.22 | 1.63 | 12.1 | -1.67 | 12.1 | 0 | 3.16 | 0.148 |
| 24 | 2831 | 26.5 | 0.594 | -2.79 | 20.5 | 105 | 142 | 36.8 | 26.1 | 0.22 | -0.0879 | overshoot | 1.47 | 1.36 | 11.8 | -1.73 | 11.8 | 0 | 3.44 | 0.0879 |
| 25 | 996 | 25.4 | 0.747 | -4.37 | 20 | 99.8 | 130 | 30.2 | 21.2 | 0.159 | 0.0829 | overshoot | 1.52 | 1.32 | 11.8 | -1.41 | 11.8 | 0 | 3.08 | 0.12 |
| 26 | 4922 | 28.7 | 0.879 | -1.61 | 21.3 | 114 | 154 | 40.7 | 34.7 | 0.24 | -0.234 | overshoot | 1.46 | 1.37 | 11.7 | -2 | 11.7 | 0 | 3.9 | 0.068 |
| 27 | 898 | 24 | 0.926 | -1.03 | 16.9 | 71.2 | 101 | 29.8 | 16.1 | 0.141 | -0.162 | overshoot | 1.39 | 1.44 | 11.7 | -1.38 | 11.7 | 0 | 3 | 0.124 |
| 28 | 4111 | 30.3 | 0.972 | 1.89 | 16.5 | 68.4 | 97.1 | 28.6 | 27.6 | 0.211 | 0.0657 | overshoot | 1.41 | 1.42 | 11.6 | -1.99 | 11.6 | 0 | 3.52 | 0.0711 |
| 29 | 3977 | 21.8 | 0.666 | -2.48 | 21 | 111 | 159 | 48.3 | 19.9 | 0.429 | -0.0758 | overshoot | 1.38 | 1.45 | 12 | -1.83 | 12 | 0 | 3.98 | 0.0997 |
| 30 | 356 | 22.3 | 0.548 | -4.64 | 20.2 | 102 | 136 | 34.4 | 17 | 0.182 | 0.0653 | overshoot | 1.48 | 1.35 | 11.8 | -1.59 | 11.8 | 0 | 3.2 | 0.137 |
| 31 | 3195 | 28.1 | 0.208 | -0.409 | 17.9 | 80.4 | 112 | 31.4 | 24.7 | 0.287 | -0.273 | overshoot | 1.43 | 1.4 | 11.9 | -2.04 | 11.9 | 0 | 3.7 | 0.087 |

The price of the margin (one C search per wing limit; `unconstrained` ignores the stall and is a reference only, never a recommendation):

| kind | alpha_frac | limit_deg | Q_TRANS_DECEL_ms2 | T_bt_s | theta_A_deg | spin_min | p95_distance_m | p95_stop_time_s | mean_energy_kJ | n_fail_total | n_failing_members | max_overshoot_m | max_end_sink_ms | J | at_bound |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| unconstrained |  |  | 2.07 | 1.9 | 20 | 0.119 | 117 | 8.77 | 17.7 | 0 | 0 | 1.82 | 1.16 | 1.41 | theta_A:hi |
| curve | 1 | 15.7 | 1.37 | 2.88 | 19.8 | 0.125 | 176 | 13.5 | 24 | 0 | 0 | 2 | 0.689 | 1.55 | theta_A:hi |
| curve | 0.85 | 13.3 | 1.19 | 2.7 | 16.8 | 0.0834 | 202 | 15.7 | 30.6 | 0 | 0 | 1.99 | 0.0164 | 1.76 | q:lo |
| headline | 0.7 | 11 | 1.11 | 3.19 | 16.8 | 0.091 | 216 | 17 | 34.6 | 0 | 0 | 1.99 | 0.0135 | 1.91 | q:lo |

### Recommended settings

One row per ArduPilot parameter the study recommends or reads. A parameter no analysis has decided keeps its register value ("not yet analysed"). The ArduPilot default is read from `ardupilot_defaults.yaml` (Plane-4.7.2) and is a reference only: it is not tuned for this airframe and is never an input.

| parameter | unit | default_text | register | value | analysis | grade | flags | change_vs_default | note |
| --- | --- | --- | --- | --- | --- | --- | --- | --- | --- |
| Q_BACKTRANS_MS | ms | 3000 | 3000 | 3189 | C | TBD |  | 189 | searched (back transition) |
| Q_A_ANGLE_MAX | deg | 30 | 30 | 16.8 | C | TBD |  | -13.2 | searched (back transition) |
| Q_M_SPIN_MIN | - | 0.15 | 0.1 | 0.091 | C | TBD |  | -0.059 | searched (back transition) |
| Q_TRANS_DECEL | m/s^2 | 2 |  | 1.11 | C | TBD | at lower bound | -0.886 | searched (back transition): C's q is 1.11 m/s^2; the wing angle is held to 11.0 deg (0.7 x alpha_s); the stop takes 216 m and 17.0 s (95th percentile over the ensemble); D's a_rec minimum is 1.78 m/s^2, so C is -37 % above it; the worst overshoot is 1.99 m against the tolerance 2 m (PROPOSED); the worst end sink is 0.01 m/s against 1.76 m/s (PROPOSED); the margin row (worst overshoot and end sink within 75% of the tolerances) has q = 1.06 m/s^2; provisional: depends on alpha_s, which is a guess (no lift peak in the CFD up to 15 deg) |
| Q_TRANS_DECEL | m/s^2 | 2 |  | 1.78 | D | TBD |  | -0.222 | the minimum over D's ensemble (a_rec) |
| AIRSPEED_MIN | m/s | 9 | 17 | 18 | FT | TBD |  | 9 | searched (forward transition, equivalent airspeed); the register values fail the stall margin in 5 member(s) |
| Q_TRANSITION_MS | ms | 5000 | 5000 | 17000 | FT | TBD |  | 12000 | searched (forward transition, equivalent airspeed); the register values fail the stall margin in 5 member(s) |
| TKOFF_THR_MAX | - | 0 (= THR_MAX) | 0.8 | 1 | FT | TBD | at upper bound | 0 | searched (forward transition, equivalent airspeed); the register values fail the stall margin in 5 member(s) |
| Q_TRAN_PIT_MAX | deg | 3 | 3 | 6 | FT | TBD |  | 3 | searched (forward transition, equivalent airspeed); the register values fail the stall margin in 5 member(s) |

## 4. Sensitivity and what would change the conclusion

Global ranking: Morris at 6 and 12 trajectories (132 and 264 runs; 62 failed runs in the larger set were filled with the worst finite value).

| Output | Spearman | Top-3 overlap | Dominating inputs (grade, range) |
| --- | --- | --- | --- |
| excursion_m | 0.93 | 0.67 | mass (derived, 21.3312 to 31.9968); wing_area (stated, 1.2936 to 1.9404); alpha_stall (assumed, 0.2192 to 0.3288) |
| distance_m | 0.96 | 1.00 | headwind (assumed, -5 to 5); wing_area (stated, 1.2936 to 1.9404); mass (derived, 21.3312 to 31.9968); alpha_stall (assumed, 0.2192 to 0.3288) |
| energy_J | 0.97 | 1.00 | mass (derived, 21.3312 to 31.9968); wing_area (stated, 1.2936 to 1.9404); headwind (assumed, -5 to 5); rotor_CT0 (derived, 0.07424 to 0.11136) |
| max_range_km | 0.99 | 1.00 | mass (derived, 21.3312 to 31.9968); wing_area (stated, 1.2936 to 1.9404); CD0 (assumed, 0.0528 to 0.0792) |

Flagged as assumptions with their range: mass, wing_area, alpha_stall, headwind, rotor_CT0, CD0.

## 5. Limitations

Linked to the uncertainty register (proposal section 13).

- Back-transition: C. Forward transition: FT, when it is among the analyses; C does not cover it.
- The forward transition in the energy budget is a quasi-steady estimate.
- AIRSPEED_MIN: D, C and FT read it as an equivalent airspeed (D.entry_convention); Q2, which feeds E, reads it as an equivalent airspeed (Q2.feed_convention). Which one ArduPilot compares is open (O-02).
- Input set is not release grade: 72 register entries are TBD: controller.altitude_hold.K_h, controller.altitude_hold.K_i, controller.altitude_hold.K_vh, controller.altitude_hold.i_max, controller.altitude_hold.v_dem_max, controller.ardupilot.AIRSPEED_MIN, ... (72 in all)
