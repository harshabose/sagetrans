# Tests

144 tests, about three minutes (`pytest`). Layout:

| File | Tests | Covers |
| --- | --- | --- |
| `unit/test_atmosphere.py` | 7 | **T-01**; ISA table; offset; range rejection; CasADi/numeric agreement; Hypothesis positivity |
| `unit/test_units.py` | 17 | unit conversions, unknown units, **T-13** (converted exactly once) |
| `unit/test_config.py` | 16 | register loading and validation, **T-14**, evidence counts, hashing |
| `unit/test_physics.py` | 10 | **T-02, T-03, T-07, T-08, T-15**, monotonicity properties, 60 s speed check |
| `unit/test_dynamics.py` | 7 | **T-04, T-05, T-06, T-09, T-10**, events, determinism |
| `unit/test_analyses.py` | 15 | analyses A and D, including the scenario-mass regression for D |
| `unit/test_energy_descent.py` | 15 | analyses E and F, steady-flight helpers, hand-calculated budget |
| `unit/test_m5.py` | 16 | **T-12** (analytic), search, ensembles, baseline and free-form controllers, analyses B2 and C (including C taking `Q_TRANS_DECEL` from analysis D) |
| `unit/test_m6.py` | 10 | analysis B1 including **T-11**, and B2 starting from a B1 solution |
| `unit/test_m7.py` | 23 | sensitivity, run records, results writer, release gate (including the real-problem T-12 rule), the CI check, report |
| `unit/test_register_drift.py` | 8 | the YAML register against the physics defaults; missing register files |
| `unit/physics_reference.py` | — | the NumPy reference implementation used only by tests (T-15, T-07) |

## Verification IDs (proposal section 12)

The release gate reads tests by name: a test whose name starts `test_tNN_` counts for `T-NN`.

| ID | Test | Where |
| --- | --- | --- |
| T-01 | atmosphere at 0 and 5,000 m | `test_atmosphere.py` |
| T-02 | wing limits and continuity | `test_physics.py` |
| T-03 | hover power with figure of merit 1 | `test_physics.py` |
| T-04 | constant-deceleration limit | `test_dynamics.py` |
| T-05 | coast limit | `test_dynamics.py` |
| T-06 | energy conservation | `test_dynamics.py` |
| T-07 | battery closed form against the iterative solve | `test_physics.py` |
| T-08 | motor steady state | `test_physics.py` |
| T-09 | step-size convergence against the adaptive reference | `test_dynamics.py` |
| T-10 | Cartesian against flight-path-angle dynamics | `test_dynamics.py` |
| T-11 | collocation replayed in the simulator | `test_m6.py` |
| T-12 | optimiser repeatability from three seeds | `test_m5.py` tests the search machinery on an analytic cost only. **The real-problem T-12 is not a unit test** (a three-seed optimisation takes minutes); it is enforced by the release gate through `search_spreads` in the run record (`test_m7.py`). See `analysis/README.md` for the measured spreads |
| T-13 | unit boundary | `test_units.py` |
| T-14 | TBD makes a run DRAFT; report writing refused | `test_config.py` (and `test_m7.py` for the writer) |
| T-15 | NumPy reference against the CasADi model | `test_physics.py` |

## Notes

* **Tests are not under mypy** (`files = ["src"]`), and `ruff` ignores the naming rules for physics
  symbols in `tests/` (`T_r`, `Vc`, …) as in `src/`.
* **Slow tests** are the collocation ones (`test_m6.py`, about 70 s in total; the solves are shared
  through module-scoped fixtures) and the sensitivity runs (`test_m7.py`, about 30 s).
* **Not in the suite:** the `benchmarks/` and `regression/` folders of the proposed layout; the
  real-problem T-12 runs; a Sobol run on the real model.
* **Property-based tests** use Hypothesis (density positivity and monotonicity, thrust and drag
  properties, bus-voltage monotonicity, unit-conversion identities).
