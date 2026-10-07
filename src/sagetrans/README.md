# `sagetrans` — package overview

`sagetrans` is a Python toolkit that turns Sage's airframe, propulsion and battery data into
numbers for the VTOL **transition** and **landing approach**: how much altitude is lost or gained,
how far the back-transition needs, what `Q_TRANS_DECEL` to configure, and whether the mission
energy fits in the pack. It runs without the autopilot. The design is in `proposal.md` at the
repository root; the history of building it is in `PROJECT_REPORT.md`.

> **Everything the toolkit currently computes rests on placeholder physics.** The vehicle numbers
> in `physics/params.py` are illustrative, not Sage's. Every output is labelled `DRAFT` and the
> release gate blocks it. Read the numbers as a demonstration that the machinery works, not as
> results about the aircraft.

## Layers and the dependency rule

```
          io                     tables, figures, report, run record, release gate
           |
        analysis                 A, B1, B2, C, D, E, F, sensitivity, search, ensembles
           |
        control                  controllers (baseline, free-form), mode state
           |
        dynamics                 simulate(), events, steady-flight and electrical helpers
           |
        physics                  wing, rotors, motor/ESC, battery, attitude, vehicle (CasADi)
           |
      atmosphere                 ISA density, pressure, temperature, speed of sound
           |
         config                  parameter register, units, hashing
```

A layer may import only from layers **below** it. `import-linter` enforces this in CI
(`lint-imports`, contract in `pyproject.toml`, requirement NFR-05). Consequences worth knowing:

* `physics` knows nothing of controllers, and `control` knows nothing of analyses.
* `dynamics` cannot import `control`, so `simulate()` accepts any object with a `command` method
  (a structural `Protocol`) and treats the controller's mode state as opaque.
* Modules inside one layer (for example the analyses) may import each other.

## Conventions used everywhere

| Item | Convention |
| --- | --- |
| Units | SI internally. Conversion happens exactly once, when a parameter is loaded (`config.units`). |
| x | ground distance along the track, positive in the direction of flight |
| h | altitude above mean sea level; height above ground is derived where needed |
| V | true **air**speed. Ground speed is `vx`, with `va = vx + headwind` |
| Headwind | positive **against** the aircraft |
| gamma | flight-path angle, positive climbing |
| theta | body pitch, positive nose-up. `alpha = theta - gamma`; the wing sees `alpha + i_w` |
| Bus current | positive on discharge; the ESC never regenerates |
| ArduPilot parameters | written in upper case as in the documentation (`AIRSPEED_MIN`, `Q_TRANS_DECEL`) |

State vector (9 states): `[x, h, vx, vh, theta, q, omega_r, omega_p, v1]` — ground distance,
altitude, ground-frame horizontal and vertical velocity, pitch, pitch rate, lift-rotor speed,
pusher speed, battery RC-branch voltage. Inputs: `[delta_r, delta_p, theta_cmd]`. Scenario
parameters passed to the vehicle: `[d_isa, headwind, theta_lim, soc]`.

## The `atmosphere` module (`atmosphere.py`)

International Standard Atmosphere with a temperature offset, eq. (6.1) of the proposal.

| Function | Returns | Notes |
| --- | --- | --- |
| `temperature(h, d_isa=0)` | `T = 288.15 - 0.0065 h + d_isa` [K] | the offset shifts temperature |
| `pressure(h)` | `p = 101325 (1 - 0.0065 h / 288.15)^5.25588` [Pa] | the offset does **not** enter pressure |
| `density(h, d_isa=0)` | `rho = p / (287.053 T)` [kg/m^3] | 1.225 at sea level, about 0.736 at 5,000 m |
| `speed_of_sound(h, d_isa=0)` | `sqrt(1.4 * 287.053 * T)` [m/s] | |

Design points:

* **Plain arithmetic only.** The functions accept floats, NumPy arrays and CasADi `SX`/`MX`
  expressions alike. That is what lets the single vehicle model be used both for simulation and
  inside the collocation NLP (NFR-08).
* **Validity range.** Numeric arguments outside `[-500, 11000]` m raise `ValueError`. Symbolic
  arguments cannot be checked, so the check is skipped for them; this matters inside collocation,
  where the altitude is a decision variable.
* **Tests** (`tests/unit/test_atmosphere.py`): T-01 (1.225 and about 0.736 kg/m^3 within 0.1 %),
  ISA table values, offset behaviour, range rejection, CasADi-versus-numeric agreement to 1e-12,
  and Hypothesis checks that density is positive and decreases with altitude.

## Where to start reading

1. `config/README.md` — how parameters are loaded and graded.
2. `physics/README.md` — the vehicle model and its equations.
3. `dynamics/README.md` — `simulate()` and the steady-flight helpers.
4. `control/README.md` — the controllers.
5. `analysis/README.md` — what each analysis computes.
6. `io/README.md` — outputs, run records, and the release gate.

## Running things

```bash
pip install -e ".[dev]"
pytest                 # 130 tests, about two minutes
ruff check . && mypy && lint-imports
```

A minimal simulation (hover, nothing commanded):

```python
import numpy as np
from sagetrans.physics.params import placeholder_vehicle
from sagetrans.physics.vehicle import CasadiVehicle
from sagetrans.dynamics.simulate import simulate, Environment

class Constant:
    def __init__(self, u): self.u = np.asarray(u, float)
    def command(self, t, x, aux, mode): return self.u, mode

veh = CasadiVehicle(placeholder_vehicle())
x0 = np.array([0, 100, 0, 0, 0, 0, 380.0, 0, 0])      # hover at 100 m
res = simulate(veh, Constant([0.32, 0, 0]), Environment(0.0), x0, t_end=5.0)
print(res.metrics)
```
