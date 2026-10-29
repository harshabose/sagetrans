# Inputs used by this run

Mode: **draft**. Register hash `332d5dcccda9`, settings hash `02dcb79b189a`.

Why this is not a release-grade input set:

- 72 register entries are TBD: controller.altitude_hold.K_h, controller.altitude_hold.K_i, controller.altitude_hold.K_vh, controller.altitude_hold.i_max, controller.altitude_hold.v_dem_max, controller.ardupilot.AIRSPEED_MIN, ... (72 in all)
| parameter | value | unit | status | source |
| --- | --- | --- | --- | --- |
| ardupilot_defaults.ardupilot.AIRSPEED_MIN | 9 | m/s | documented | AIRSPEED_FBW_MIN 9 in ArduPlane/config.h (the default of AIRSPEED_MIN in Parameters.cpp), Plane-4.7.2; the same value as Plane-4.6.3 |
| ardupilot_defaults.ardupilot.Q_ASSIST_SPEED | 0 | m/s | documented | quadplane.cpp AP_GROUPINFO ASSIST_SPEED (assist.speed), Plane-4.7.2; not modelled, listed for the table (O-20) |
| ardupilot_defaults.ardupilot.Q_A_ANGLE_MAX | 30 | deg | documented | ANGLE_MAX of AC_AttitudeControl (Q_A_ANGLE_MAX): AC_ATTITUDE_CONTROL_ANGLE_MAX_DEFAULT 30 deg, libraries/AC_AttitudeControl/AC_AttitudeControl.cpp, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.Q_BACKTRANS_MS | 3000 | ms | documented | quadplane.cpp AP_GROUPINFO BACKTRANS_MS, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.Q_BCK_PIT_LIM | 10 | deg | documented | quadplane.cpp AP_GROUPINFO BCK_PIT_LIM, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.Q_M_SPIN_MIN | 0.15 | - | documented | AP_MOTORS_SPIN_MIN_DEFAULT in libraries/AP_Motors/AP_MotorsMulticopter.h, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.Q_TRANSITION_MS | 5000 | ms | documented | quadplane.cpp AP_GROUPINFO TRANSITION_MS, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.Q_TRANS_DECEL | 2 | m/s^2 | documented | quadplane.cpp AP_GROUPINFO TRANS_DECEL, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.Q_TRAN_PIT_MAX | 3 | deg | documented | quadplane.cpp AP_GROUPINFO TRAN_PIT_MAX, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.THR_MAX | 1 | - | documented | THROTTLE_MAX 100 percent, ArduPlane/config.h, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.THR_MIN | 0 | - | documented | THROTTLE_MIN 0 percent, ArduPlane/config.h, Plane-4.7.2 |
| ardupilot_defaults.ardupilot.TKOFF_THR_MAX | 0 | - | documented | Parameters.cpp TKOFF_THR_MAX, Plane-4.7.2; 0 means THR_MAX is used (O-17) |
| ardupilot_defaults.ardupilot.TRIM_THROTTLE | 0.45 | - | documented | AP_PLANE_TRIM_THROTTLE_DEFAULT 45 percent in ArduPlane/config.h (called THROTTLE_CRUISE before 4.7), Plane-4.7.2 |
| controller.altitude_hold.K_h | 0.5 | s^-1 | TBD | altitude error to demanded climb rate |
| controller.altitude_hold.K_i | 0.01 | m^-1 | TBD | integral gain: throttle per metre of accumulated altitude-rate error |
| controller.altitude_hold.K_vh | 0.02 | s/m | TBD | throttle per m/s of climb-rate error |
| controller.altitude_hold.i_max | 20 | m | TBD | integral clamp |
| controller.altitude_hold.v_dem_max | 3 | m/s | TBD | climb-rate demand limit |
| controller.ardupilot.AIRSPEED_MIN | 17 | m/s | TBD | transition speed, read as EQUIVALENT airspeed (O-14: the forward transition ends when EAS > AIRSPEED_MIN; O-23: the AIRBRAKE exit uses EAS too). 17.0 EAS = 1.26 x stall at nominal mass (stall 13.52 m/s EAS: m 26.664 kg, S 1.617, model CL_max 1.444) and 1.17 x at mass x1.15 (stall 14.50). Same margin at every altitude; CL_max is provisional (vehicle.yaml alpha_s). Value kept from the earlier TAS reading; to be revisited by analysis FT |
| controller.ardupilot.PTCH_LIM_MAX_DEG | 20 | deg | TBD | (DEFAULT ARDU VALUE) PITCH_MAX 20 in ArduPlane/config.h, used by Parameters.cpp PTCH_LIM_MAX_DEG, Plane-4.7.2; fixed in analysis C, not searched (owner decision 2026-10-28) |
| controller.ardupilot.PTCH_LIM_MIN_DEG | -25 | deg | TBD | ArduPilot default as remembered; not re-read from source. Check |
| controller.ardupilot.Q_A_ANGLE_MAX | 30 | deg | TBD | (DEFAULT ARDU VALUE) max VTOL lean angle |
| controller.ardupilot.Q_BACKTRANS_MS | 3000 | ms | TBD | (DEFAULT ARDU VALUE) pitch-limit ramp time on back-transition |
| controller.ardupilot.Q_M_SPIN_MAX | 0.95 | - | TBD | rotor throttle ceiling |
| controller.ardupilot.Q_M_SPIN_MIN | 0.1 | - | TBD | rotor throttle floor in altitude hold; set above the ESC dead zone |
| controller.ardupilot.Q_TRANSITION_MS | 5000 | ms | documented | (DEFAULT ARDU VALUE) Plane docs default 5000 |
| controller.ardupilot.Q_TRANS_FAIL | 0 | ms | documented | (DEFAULT ARDU VALUE) 0 disables the stuck-transition timeout (documented) |
| controller.ardupilot.Q_TRAN_PIT_MAX | 3 | deg | TBD | (DEFAULT ARDU VALUE) pitch cap in the forward transition; the fade cap is (Q_TRAN_PIT_MAX + 1) x 2 (O-16) |
| controller.ardupilot.THR_MAX | 1 | - | TBD | (DEFAULT ARDU VALUE) |
| controller.ardupilot.THR_MIN | 0 | - | TBD | (DEFAULT ARDU VALUE) |
| controller.ardupilot.THR_SLEWRATE | 100 | %/s | TBD | (DEFAULT ARDU VALUE) |
| controller.ardupilot.TKOFF_THR_MAX | 0.8 | - | TBD | (DEFAULT ARDU VALUE) pusher throttle in AUTO during the forward transition |
| controller.ardupilot.TKOFF_THR_MIN | 0 | - | TBD | (DEFAULT ARDU VALUE) 0 = use TRIM_THROTTLE as the pusher minimum in the forward transition (O-17) |
| controller.ardupilot.TRIM_THROTTLE | 0.45 | - | TBD | (DEFAULT ARDU VALUE) THROTTLE_CRUISE 45 percent (config.h). Set to the flown value: it is the pusher minimum during the forward transition and the TECS feed-forward |
| controller.cruise_stub.K_hd | 0.03 | rad*s/m | TBD | pitch per m/s of climb rate |
| controller.cruise_stub.K_hp | 0.02 | rad/m | TBD | pitch per metre of altitude error |
| controller.cruise_stub.K_s | 0.05 | s/m | TBD | pusher command per m/s speed error |
| controller.cruise_stub.K_si | 0.01 | m^-1 | TBD | speed integral gain |
| controller.cruise_stub.r_tecs | 0.5 | s^-1 | TBD | pusher-command rate limit; TECS_THR_ERATE meaning open (O-10) |
| controller.cruise_stub.tau_h | 1 | s | TBD | pusher hand-off time constant |
| controller.cruise_stub.theta_cruise_max | 0.15 | rad | TBD | cruise pitch limit |
| controller.cruise_stub.v_cruise | 22.5 | m/s | TBD | cruise speed target (true airspeed); 22.5 TAS at the 1500 m reference altitude = 20.9 EAS |
| controller.position.K_p | 0.5 | s^-1 | TBD | position error to demanded speed (linear zone) |
| controller.position.K_v | 1 | s^-1 | TBD | speed error to commanded deceleration |
| controller.position.a_lim | 3 | m/s^2 | TBD | deceleration limit of the speed profile; what it is in Plane is open (O-11) |
| controller.tecs.TECS_CLMB_MAX | 5 | m/s | TBD | (DEFAULT ARDU VALUE) |
| controller.tecs.TECS_INTEG_GAIN | 0.3 | - | TBD | (DEFAULT ARDU VALUE) |
| controller.tecs.TECS_PITCH_MAX | 15 | deg | TBD | (DEFAULT ARDU VALUE) 0 would use PTCH_LIM_MAX_DEG |
| controller.tecs.TECS_PTCH_DAMP | 0.3 | s | TBD | (DEFAULT ARDU VALUE) |
| controller.tecs.TECS_SINK_MAX | 5 | m/s | TBD | (DEFAULT ARDU VALUE) |
| controller.tecs.TECS_SINK_MIN | 2 | m/s | TBD | (DEFAULT ARDU VALUE) |
| controller.tecs.TECS_THR_DAMP | 0.5 | s | TBD | (DEFAULT ARDU VALUE) |
| controller.tecs.TECS_TIME_CONST | 5 | s | TBD | (DEFAULT ARDU VALUE) |
| controller.tecs.TECS_VERT_ACC | 7 | m/s^2 | TBD | (DEFAULT ARDU VALUE) |
| mission.battery_window.eta_usable | 0.8 | - | TBD | usable fraction of nominal energy; BMS and cut-off limits |
| mission.battery_window.soc_eod | 0.1 | - | TBD | state of charge at end of discharge, for the hover power check |
| mission.battery_window.soc_start | 1 | - | TBD | state of charge at take-off (a full pack); the usable energy is eta_usable of nominal, so the mission ends at 1 - eta_usable |
| mission.battery_window.soh | 1 | - | TBD | state of health (1 = new) |
| mission.battery_window.temp_factor | 1 | - | TBD | temperature derating (1 = 25 degC) |
| mission.descent.h_slow | 6 | m | assumed | below this height the descent slows to v_final; set to the ArduPilot Q_LAND_FINAL_ALT default |
| mission.descent.kappa | 0.23 | - | documented | Johnson, NASA/TP-2005-213477, Yeates test data: observable vortex ring state from Vz/vh = -0.23 near zero forward speed (the most conservative onset reported; Drees roughness -0.62 to -1.53, inflow instability -0.5 to -1.5). Read from a summary of the report: confirm against the PDF before quoting |
| mission.descent.v_final | 0.5 | m/s | assumed | final descent rate; set to the ArduPilot Q_LAND_FINAL_SPD default; rangefinder and ground effect |
| mission.descent.v_fragility | 3 | m/s | TBD | payload-fragility descent-rate limit; never binds while the vortex-ring cap (1.5 to 2.4 m/s over the scenarios) is lower; confirm with payload data |
| mission.profile.a_fwd | 1.5 | m/s^2 | TBD | forward-transition acceleration; ASSUMED until analysis C |
| mission.profile.clearance_agl_m | 30 | m | TBD | hover climb before the forward transition; also the height above ground at which analysis FT starts |
| mission.profile.climb_height_m | 200 | m | TBD | climb above the airfield; cruise altitude = site + this (the sites are swept in study.yaml, E.sites_amsl_m) |
| mission.profile.d_diversion_m | 5000 | m | TBD | diversion distance (reserve) |
| mission.profile.fwd_alt_gain_max_m | 10 | m | assumed | largest altitude gain above the forward-transition start that is accepted; a failure beyond it (analysis FT). Symmetric with the loss limit by default |
| mission.profile.fwd_alt_loss_max_m | 10 | m | assumed | largest altitude loss below the forward-transition start that is accepted; a failure beyond it (analysis FT). Set by the owner 2026-10-27: 30 m start, 10 m clearance, so the floor is 20 m AGL |
| mission.profile.h_bt_agl_m | 30 | m | TBD | height above ground where the back-transition starts |
| mission.profile.isa_offset_k | 0 | K | TBD | ISA temperature offset for the budget |
| mission.profile.n_diversion_transitions | 0 | - | TBD | extra forward+back transition pairs |
| mission.profile.payload_kg | 5 | kg | TBD | mass delivered at the destination; the return leg flies at vehicle mass minus this (O-12; 1 - 5/26.664 = 0.8125, scenario S-06) |
| mission.profile.roc | 2.5 | m/s | TBD | rate of climb |
| mission.profile.soc_mean | 0.6 | - | TBD | one mean state of charge used for all segment powers (the mean of the trace, 1.0 to 0.2) |
| mission.profile.t_drop_s | 0 | s | TBD | hover at the destination for the drop, at the loaded mass |
| mission.profile.t_go_around | 60 | s | TBD | go-around hover time (reserve) |
| mission.profile.t_hold | 0 | s | TBD | hold at the landing zone (reserve) |
| mission.profile.t_hover_pre | 10 | s | TBD | hover on the pad before lift-off |
| mission.profile.v_cruise | 22.5 | m/s | TBD | design true airspeed at v_cruise_ref_amsl_m (22.5 TAS at 1500 m = 20.9 EAS); at any other altitude the TAS follows from the density (same dynamic pressure). Same as controller.cruise_stub.v_cruise |
| mission.profile.v_cruise_ref_amsl_m | 1500 | m | TBD | altitude where the design speeds hold |
| mission.profile.vz_takeoff | 2 | m/s | TBD | vertical climb rate in take-off |
| mission.requirements.range_p95_km | 120 | km | stated | about 113 km needed for the 95th-percentile mission |
| mission.requirements.range_primary_km | 100 | km | stated | specified range requirement |
| vehicle.attitude.omega_n | 8 | rad/s | TBD | pitch-loop natural frequency; SITL or flight logs |
| vehicle.attitude.q_max | 1.5 | rad/s | TBD | pitch-rate limit |
| vehicle.attitude.zeta | 0.8 | - | TBD | pitch-loop damping ratio |
| vehicle.battery.C1_pack | 250 | F | assumed | ONE pack: k*C1_cell/n_s = 5*600F/12; tau = 1.8 s (placeholder preset; HPPC/GITT fit) |
| vehicle.battery.I_av | 3 | A | derived | 120 W rated (5 V: 15+25 W, 12 V: 80 W) / (0.9 x 44 V); excludes servos and RF air unit |
| vehicle.battery.R0_pack | 12 | mohm | assumed | ONE pack, fresh, 25 degC, SoC>=30%: n_s*R0_cell/k = 12*5mohm/5 (placeholder LiPo preset scaled to 25 Ah; replace with HPPC) |
| vehicle.battery.R1_pack | 7.2 | mohm | assumed | ONE pack: n_s*R1_cell/k = 12*3mohm/5 (placeholder preset; HPPC/GITT fit) |
| vehicle.battery.V_min | 33 | V | TBD | bus cut-off; battery-management limit (O-15) |
| vehicle.battery.capacity_Ah_pack | 25 | Ah | datasheet | MPower datasheet, per pack |
| vehicle.battery.cells_series | 12 | - | stated | MPower 12S (stated) |
| vehicle.battery.n_parallel | 2 | - | stated | Y-connector to PDB (stated) |
| vehicle.battery.v_nominal | 44.4 | V | datasheet | MPower 12S datasheet |
| vehicle.mass.m | 26.664 | kg | derived | from sage excel sheet |
| vehicle.motor_lift.I0 | 3 | A | datasheet | datasheet no-load current at 48 V (the sweep cannot resolve it: torque vs current gives 0 +/- a few A) |
| vehicle.motor_lift.I_lim | 75 | A | datasheet | datasheet maximum continuous current (180 s); 75 A was reached at 100% throttle in the VT3010 sweep (burst, not rated) |
| vehicle.motor_lift.Ke | 0.0796 | V*s/rad | datasheet | 60/(2 pi KV), KV = 120 (Vector Technics 8020 PRO datasheet); load-test regression 50-100% throttle gives 0.0795 |
| vehicle.motor_lift.Kt | 0.0796 | N*m/A | derived | equals Ke in SI; torque vs current in the datasheet sweep gives 0.075-0.080 (the datasheet's Kt = 0.108 @ 3200 rpm does not match its own data) |
| vehicle.motor_lift.Rm | 0.14 | ohm | derived | 2 x Rph (0.06) = 0.12 at 25 degC, x1.2 hot copper, plus ESC/wiring; sweep fit (Ke fixed) gives 0.13-0.15 over 50-100% throttle |
| vehicle.motor_lift.dead_zone | 0.05 | - | TBD | ESC dead-zone fraction of throttle; bench motor test |
| vehicle.motor_pusher.I0 | 2.31 | A | datasheet | sheet idle current at 10 V (about 2800 rpm); no-load current rises with rpm, so this is a lower bound at operating speed |
| vehicle.motor_pusher.I_lim | 107.9 | A | datasheet | sheet max current at 100% throttle, 48 V, 2 min (copper 98 degC); ESC rated 120 A |
| vehicle.motor_pusher.Ke | 0.0341 | V*s/rad | datasheet | 60/(2 pi KV), KV = 280 (Nipple 6220 spec sheet) |
| vehicle.motor_pusher.Kt | 0.0341 | N*m/A | derived | equals Ke in SI (no torque measurement on the sheet) |
| vehicle.motor_pusher.Rm | 0.09 | ohm | derived | EFFECTIVE series resistance fitted to the 48 V bench sweep (60-100% throttle, GF 18X8E); the sheet's internal resistance is 0.0192 ohm; includes ESC, wiring and duty compression; under-predicts current at 30-50% throttle |
| vehicle.motor_pusher.dead_zone | 0.05 | - | TBD | ESC dead-zone fraction of throttle; bench motor test |
| vehicle.pusher.CQ0 | 0.0052 | - | derived | linear fit of APC Cp/(2 pi) vs J, J 0.1-0.55, tables 6000-11000 rpm; error about 8% at J 0.2-0.45, +13% at J 0.1; the model has no CQ2 term (Cp is concave in J) |
| vehicle.pusher.CQ1 | -0.00626 | - | derived | same fit |
| vehicle.pusher.CT0 | 0.0826 | - | derived | quadratic fit of APC 18x8E Ct(J), J 0-0.58, tables 6000-11000 rpm (APC computed prediction, ISA sea level); within 2% for J<=0.5 |
| vehicle.pusher.CT1 | -0.0787 | - | derived | same fit |
| vehicle.pusher.CT2 | -0.1112 | - | derived | same fit |
| vehicle.pusher.D | 457.2 | mm | assumed | APC 18x8E (assumed pusher propeller; 18 in, 8 in pitch) |
| vehicle.pusher.I_p | 0.0002 | kg*m^2 | TBD | propeller + motor inertia |
| vehicle.pusher.J_max | 0.585 | - | derived | end of the APC 18x8E tables (Ct crosses zero near J = 0.58); beyond this the coefficients are held |
| vehicle.pusher.J_min | 0 | - | derived | APC tables start at J = 0 (no reverse-flow data) |
| vehicle.pusher.omega_floor | 1 | rad/s | assumed | NUMERICAL guard |
| vehicle.rotor.CH1 | 0 | - | assumed | in-plane (H) force removed by decision |
| vehicle.rotor.CQ0 | 0.005 | - | derived | constant CQ = Q/(rho n^2 D^5), same rows and density; spread 0.00476-0.00521 |
| vehicle.rotor.CQ1 | 0 | - | assumed | J dependence removed by decision |
| vehicle.rotor.CQ2 | 0 | - | assumed | J dependence removed by decision |
| vehicle.rotor.CT0 | 0.0928 | - | derived | constant CT = T/(rho n^2 D^4), VT3010 CFRP_FP on 8020 PRO, mean of 50-80% throttle rows (2343-3475 rpm), rho = 1.225 assumed; spread 0.0896-0.0951; 0.0855 at 95% |
| vehicle.rotor.CT1 | 0 | - | assumed | J dependence removed by decision (no data); constant-CT idealisation |
| vehicle.rotor.CT2 | 0 | - | assumed | J dependence removed by decision |
| vehicle.rotor.D | 762 | mm | datasheet | VT 3010 CFRP_FP propeller (30 in) |
| vehicle.rotor.I_rot | 0.0015 | kg*m^2 | TBD | rotor + motor inertia; sets spool-up |
| vehicle.rotor.J_max | 1 | - | assumed | INERT while CT1=CT2=CQ1=CQ2=0 (J removed) |
| vehicle.rotor.J_min | -0.5 | - | assumed | INERT while CT1=CT2=CQ1=CQ2=0 (J removed); kept so the map stays valid if J is restored |
| vehicle.rotor.k_mu | 0 | - | assumed | edgewise-flow thrust/torque factor removed by decision |
| vehicle.rotor.mu_max | 1 | - | assumed | INERT while k_mu=CH1=0 (edgewise terms removed) |
| vehicle.rotor.n_r | 4 | - | stated | 4+1 configuration (stated) |
| vehicle.rotor.omega_floor | 1 | rad/s | assumed | NUMERICAL guard, keep > 0: it stops 0/0 in J and mu at standstill even though they are multiplied by zero |
| vehicle.wing.CD0 | 0.066 | - | assumed | ASSUMED: 2 x 0.033 (wing-only CFD symmetric fit CD = CD0 + k CL^2, 0-10 deg) as a stand-in for whole-aircraft CD0; replace with whole-aircraft CFD |
| vehicle.wing.CL0 | 0.4 | - | derived | CFD (half-model, doubled): linear fit CL vs alpha, 0-10 deg; raw 0.376 at 0 deg; AoA reference to confirm |
| vehicle.wing.CL_alpha | 4.75 | rad^-1 | derived | CFD linear fit 0-10 deg (lifting-line estimate 5.0-5.2) |
| vehicle.wing.CN | 1.5 | - | TBD | flat-plate normal-force coefficient (Cory and Tedrake 1.2 to 2); sourced or measured value needed |
| vehicle.wing.M | 70 | rad^-1 | TBD | stall-blend sharpness: a NUMERICAL parameter, not physical; was 100 (needed for T-02), set to 70 on 2026-10-14; re-check T-02 and the A thrust_sign window |
| vehicle.wing.S | 1.617 | m^2 | stated | aircraft parameter sheet (note: b x MAC = 1.605 m^2, 0.7% off) |
| vehicle.wing.alpha_s | 0.274 | rad | assumed | PROVISIONAL: chosen for a CL_max of about 1.5 (parameter sheet, not supported by CFD, which has no peak up to 15 deg). The model's CL_max is 1.444, not 1.5: the stall-blend sharpness M went from 100 to 70 on 2026-10-14 and that lowered the peak; alpha_s and M are unchanged, the owner decides whether to re-tune. Needs CFD at 16-24 deg |
| vehicle.wing.i_w | 0 | rad | derived | wing incidence relative to the body x-axis |
| vehicle.wing.k | 0.042 | - | derived | CFD fit 0-10 deg; agrees with Raymer straight-wing K = 0.0405 (not the swept-wing 0.0631) |
