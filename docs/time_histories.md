# Why the C, B1 and FT time histories look the way they do

Review of 2026-10-29 on `master` at 3270491, register as shipped (all numbers are DRAFT: 72 entries
are TBD). **Everything here depends on the stall model, and the stall model is provisional:**
`alpha_s` (15.70 deg) is a guess (the CFD has no lift peak up to 15 deg), the blend sharpness
`M` (70) is a numerical parameter that also sets C_L,max, and the C_L peak is at `alpha_w`
13.43 deg (C_L,max 1.444). Numbers marked *tiny* come from a throwaway 4-member, 2-iteration
configuration and show the mechanism only; numbers marked *mid* come from a 32-member, one-seed,
15-minute search and are also not the study's result. The real numbers come from the first full run
after the register is filled.

There is no single cause. One mechanism per analysis, plus two small ones all three share.

## C: the wing is flown through the stall

The pitch ramp (`Q_A_ANGLE_MAX`, `PTCH_LIM_MAX_DEG` 20 deg) takes the wing angle past the C_L peak.
The model's C_L falls from 1.44 to about a third within 4 deg, the wing lift drops, the rotors must
catch the aircraft, the sink raises the wing angle further (alpha = theta - gamma) and the cycle
repeats. Nothing flagged it: the stall check in `runs.failure_flags` only counts time above the 1 g
stall speed, and `D.enforce_alpha` never reached C.

*mid*, unconstrained optimum (`C.enforce_alpha: false`, the old behaviour): all 32 members go above the
C_L peak, up to 2.7 s each, peak wing angle up to 33.8 deg, the least wing share of the weight
0.03. The ArduPilot default is the same (34.1 deg). That optimum has `q` 2.03 m/s^2 and `T_bt` at its
lower bound.

The fix is a configuration, not a change of the model: `C.enforce_alpha` (shipped on) caps the pitch
command at `limit + gamma` while `V` is above the 5 m/s gate, with `limit = margin_alpha_frac x
alpha_s` (0.7: 10.99 deg; ASSUMED, PROPOSED, one study-level setting shared with B1 `margin`). It is
a command-side cap, so the angle can still sit about a degree above the limit; the C_L peak is the
stall, and no member reaches it. The unconstrained search stays as a reference (`C:unconstrained`,
"the model ignores the stall; reference only; never a recommendation").

| *mid*, 32 members | q [m/s^2] | mean energy | p95 distance | p95 stop time | members above the C_L peak | peak wing angle |
| --- | --- | --- | --- | --- | --- | --- |
| unconstrained (reference) | 2.03 | 20.0 kJ | 119.5 m | 8.9 s | 32 of 32 | 33.8 deg |
| held to 1.00 alpha_s (15.70 deg) | 1.30 | 26.9 kJ | 185.4 m | 14.2 s | n/a | n/a |
| held to 0.85 alpha_s (13.34 deg) | 1.15 | 33.1 kJ | 209.1 m | 16.2 s | n/a | n/a |
| held to 0.70 alpha_s (10.99 deg), the headline | 1.08 | 36.8 kJ | 222.3 m | 17.4 s | 0 of 32 | 11.5 deg |

The rows between are one search each (`C_alpha_curve`) with a short search, so they are not smooth;
only the two ends are read. The price is real: holding the wing below the lift peak costs 1.84 x
the energy, 1.86 x the distance and 1.96 x the stop time of the unconstrained search. The stop is
long (about 220 m), and **the 250 m stop distance is accepted as provisional**: it depends on
`alpha_s`. The headline `q` sits near its lower bound (1.0, flagged `q:lo`).

### Does C now agree with D? (the reconciliation, step 4.4)

D's recommendation (`a_rec` minimum over its 32 Sobol members) is 1.653 m/s^2 as the study is
configured. That is a different problem from C's enforced one: D runs with a stalled wing
(`D.enforce_alpha: false`), no lift-peak limit, the lift-aware feed-forward and no rotor lag. C's
unconstrained `q` (2.03) is 23 % above it, because C's `q` also sets the stop target and the search
sits on the overshoot tolerance, while D asks only what the aircraft can do. Holding D to the same
definition as C closes the gap (D run once each way on the shipped register, 32 members, `a_rec`
minimum):

| D, same wing cap as C | rotor lag 0 | rotor lag 0.25 s (C's) |
| --- | --- | --- |
| wing free (D as configured) | 1.653 | 1.668 |
| held to 0.7 alpha_s | 1.126 | 1.047 |

With the same wing limit and the same rotor lag, D's minimum is 1.05 m/s^2 and C's enforced `q` is
1.08, 3 % above it (the 95th-percentile `a_rec` is 1.09). So C and D agree once they are asked the
same question; the 1.65 number is the capability of an aircraft that stalls its wing, and C
enforced is not expected to meet it. C's own capability ceiling at the enforced optimum
(`a_cap_ms2` in `C_runs`) is 1.17 to 1.35 m/s^2. `settings_recommended` still takes D's headline
`a_rec` for E; that D headline is the stalled-wing number. Whether D's headline should be run under
the margin is a decision for the author and was not changed here.

### Amplifiers and what is not a cause

The assumed 0.25 s lumped rotor lag (measured earlier on the unconstrained S-01 run: lag 0 halves the climb-rate swing, lag 0.5 doubles it) and the
untrimmed entry (below) amplify the swing. Not causes: the integration step and the battery model.
Under the enforced configuration the one-at-a-time sensitivity has no failing member in any row
(*tiny*; the unconstrained search had failures at `rotor_lag_s` 0.5 and `z_acc_p` 0.015), and the
change of J without the penalty stays within about 10 %.

### Where the entry state is set (not changed here)

`braking.airbrake_entry_state` builds all nine states at the AIRBRAKE exit: pitch is
`bp.theta_fw0` (from `D.theta_fw0_deg`, a placeholder at 0 deg, through `stages.exit_params`), the
pusher is off and the rotors are spun to `entry_rotor_thrust` (`C.rotor_start`). The wing is
therefore not trimmed at 15 m/s (the wing carries about a third of the weight at t = 0), so the
first second has a vertical acceleration of about -6 m/s^2. Trimming the entry is a separate
change and is out of scope.

## B1: the optimum sits on the edge of the lift curve and ends falling

* `attached` (0.85 alpha_s = 13.34 deg) holds C_L at 99.97 % of its peak: the limit is 0.09 deg
  below the C_L peak, so there is no margin in lift. `c_envelope` (30 deg) flies far past the peak
  and does not survive a command 10 ms late.
* The end condition only bounded the climb rate, so the optimum ended falling at about 0.6 g
  (vertical acceleration -5.8 m/s^2 at the last node: S-01, `c_envelope`, 20 deg ceiling, 20 intervals, unit test): the last
  interval buys energy by dumping throttle and pitch. `B1.end_balance` (shipped on, every variant)
  makes the last node a vertical force balance.
* The new `margin` variant flies C's wing limit (`margin_alpha_frac x alpha_s`, 10.99 deg, 2.44 deg
  below the peak, C_L 90.6 % of C_L,max) and C's pitch ceiling.

*tiny*, S-01 and S-03 (the real register, 40 intervals), energy at distance caps 100 and 300 m, all
replay-verified except where stated:

| B1 energy [kJ] | options off (= master) | `end_balance` on | change |
| --- | --- | --- | --- |
| S-01 `attached`, 100 m | 8.92 | 9.10 | +2.0 % |
| S-01 `c_envelope`, 100 m | 10.40 | 10.53 | +1.3 % |
| S-01 `margin`, 100 and 300 m | n/a | 12.23 (77 m) | |
| S-03 `attached`, 300 m | 21.81 | 22.15 | +1.5 % |
| S-03 `c_envelope`, 300 m | 23.30 | 23.56 | +1.1 % |
| S-03 `margin`, 300 m | n/a | 28.68 (128 m) | |

With the options off the B1 table of the new code is identical to master, point for point
(energy and distance). Cases that did not verify:

* S-03 `attached` at 100 m: no solution (also on master; not converged, `Maximum_Iterations_Exceed`).
* S-03 `margin` at 100 m: `Maximum_Iterations_Exceed`; 100 m is below what the margin variant needs
  there (its 300 m solution stops in 128 m). Not tuned.
* **S-01 `c_envelope`, the gap point (B1 at C's own distance, 82.6 m), with `end_balance` on:** IPOPT
  `Solve_Succeeded` in 111 iterations, but the replay does not agree on the excursion (3.26 m
  replayed against 0.44 m collocated, relative error 6.5 against the 2 % tolerance; energy and
  distance agree to 0.6 % and 0.5 %). With `end_balance` off the same point verifies (10.96 kJ, 61 m)
  and fails only the 10 ms-late command check. The gap row is therefore left unverified and nothing
  is quoted for it; the `c_envelope` front points at 100 and 300 m do verify. Not tuned: it is the
  known behaviour of an optimum that rides the post-stall model.

The gap to C (S-01, C at the register parameters with q = 2, wing held to 10.99 deg: 18.0 kJ over
83 m; *tiny*, mechanism only): `margin` 14.2 kJ, `attached` 11.1 kJ; C without the cap 16.0 kJ over
58 m. `margin` flies C's wing limit and pitch ceiling, so the pitch freedom is out of the gap and
what is left is B1's missing rotor lag and its clairvoyance; `attached` and `c_envelope` add
operation at the lift peak.

## FT: the dive after the fade is the TECS flight-stage switch, not a stall

Found in the review on the earlier branch and **not changed here** (FT is out of scope). The wing
never stalls in FT. At the end of the fade the controller changes from the VTOL stage (pitch ignores
speed) to the NORMAL stage (speed weight 1); if the airspeed is below the speed demand TECS pitches
down to gain speed, the lift falls, the aircraft sinks and the pusher is slew-limited upwards. That
review's counterfactual (keeping the VTOL stage after DONE) is not ArduPilot behaviour and is not in
this branch. The one FT change is a comment: `configs/study.yaml` said FT uses "the first 16 points
of C's Sobol sample"; C is a Latin hypercube, FT is D's Sobol with its own headwind range
(`tests/unit/test_review_fixes.py`).

## Shared, and other defects found on the way

* `trim.steady_trim` and `glide_trim` searched up to `alpha_s`, past the C_L peak, and reported "no
  steady trim" at 14 to 15.5 m/s where one exists (fixed: they search up to the peak,
  `aero.alpha_at_cl_max`).
* `stages.task_b1` used `x or default` for the end band, rotor start and pitch ceiling, so a legitimate
  0 fell back to the default (fixed: `first_set`).
* The colour for `margin` (series 4, `s4`) is not checked by the dataviz palette
  validator.
