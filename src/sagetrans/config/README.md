# `sagetrans.config` — the parameter register, units, and hashing

This package is the bottom layer: it depends on nothing else in `sagetrans`. It implements the
proposal's rule that **no number is buried in code — every parameter carries a value, a unit, a
source and a status** (FR-01, design principles of section 1, section 9).

| File | Contents |
| --- | --- |
| `schema.py` | `Status`, `ParamRecord`, `Config`, and the three exceptions |
| `units.py` | `to_si`, the unit parser, `SIValue`, `UnitError` |
| `__init__.py` | re-exports the public names |
| `../../../configs/vehicle.yaml` | the current parameter register |

> **Important limitation.** The register is *not yet connected to the physics.* The physics
> models read frozen dataclasses from `physics/params.py` (illustrative placeholders), not a
> `Config`. So a `Config` currently supplies the *evidence grade, DRAFT label and hash* of a
> result, while the numbers come from `placeholder_vehicle()`. Closing that link is the first
> step toward a releasable result (see `PROJECT_REPORT.md`, "What is not done").
>
> **Until then, a drift guard** (`tests/unit/test_register_drift.py`) keeps the two from silently
> disagreeing: every register entry that carries a value must be listed with the physics value it
> corresponds to (rotor count, pack count, nominal voltage, capacity, `Q_TRANSITION_MS`) and must
> agree with it in SI. Adding a value to the register without wiring it into that table fails the
> test. The register also points at two files that do not exist (`rotors.ct_map`,
> `battery.ecm_file`); `Config.missing_files()` reports them and the run record counts them as a
> reason for DRAFT.

## The status vocabulary

`Status` is a fixed enum (the proposal's section 9):

| Status | Meaning |
| --- | --- |
| `measured` | taken from our own bench or flight data |
| `datasheet` | from a manufacturer datasheet |
| `derived` | computed from other parameters |
| `documented` | taken from documentation (for example ArduPilot's Plane docs) |
| `stated` | stated in the project brief |
| `assumed` | an engineering assumption; flagged wherever it dominates a result |
| `TBD` | not yet known |

Anything containing a `TBD` makes the whole configuration **DRAFT**.

## `ParamRecord`

One parameter. A frozen pydantic model with `extra="forbid"` (an unknown field fails loading).

| Field | Meaning |
| --- | --- |
| `value`, `unit` | the value and unit **as authored** |
| `file` | a path (relative to the YAML) for table parameters such as coefficient maps |
| `source` | free text: where the number came from |
| `status` | a `Status` |
| `uncertainty` | optional; used by sensitivity ranges |
| `si_value`, `si_unit` | derived **once** at construction, never recomputed |

Validation rules (all tested):

* a record has either `value` or `file`, **not both**;
* a record whose status is anything other than `TBD` must have a value or a file;
* a record with a value must have a unit (`"-"` for dimensionless);
* when a value is present it is converted to SI immediately and stored in `si_value`/`si_unit`.

## `Config`

A validated, immutable set of records, keyed by dotted path (`"battery.v_nominal_V"`), plus the
directory of the YAML (the root for `file` records).

| Method / property | Behaviour |
| --- | --- |
| `Config.from_yaml(path)` | load and validate; the top level must be a mapping |
| `Config.from_dict(data, root=None)` | same from a dictionary |
| `record(path)` | the `ParamRecord`; `MissingParameterError` if the path is unknown |
| `get(path)` | the value **in SI**; raises `MissingParameterError` if it is `TBD` or a file record |
| `file_path(path)` | the resolved path of a file record |
| `evidence_counts()` | `{status: count}` over every grade (zeros included) |
| `missing_files()` | dotted paths of file records whose file does not exist (the hash records these as `"missing"`); `build_record` treats any as a DRAFT reason |
| `is_draft` / `label` | `True`/`"DRAFT"` if any record is `TBD`, else `"RELEASED"` |
| `require_releasable(override=False)` | raise `DraftConfigError` listing the TBD paths, unless overridden |
| `config_hash()` | SHA-256 of what determines a result (below) |
| iteration, `in` | over the dotted paths, sorted |

How the YAML is walked: a mapping is a *record* if it contains `status` or `source`, otherwise a
*group* that is descended into. Validation errors from **all** records are collected and reported
together in one `ConfigError`, so a bad file is fixed in one pass.

### The configuration hash

`config_hash()` is a SHA-256 over a canonical JSON of, for each record: `si_value`, `si_unit`,
`status`, `uncertainty`, and — for file records — the **content digest** of the file (or the string
`"missing"` if it does not exist). Deliberately **excluded** is the free-text `source`, so
rewording a citation does not change the hash. Consequences (tested): the hash is deterministic,
changes when a value, unit, status or file content changes, and does not change when only the
source text changes. The hash is what ties an output to its inputs in the run record (NFR-07).

## Units (`units.py`)

Unit conversion happens **once, at the input boundary** (NFR-02, test T-13). `to_si(value, unit)`
returns an `SIValue(value, unit)` in SI base units.

* **Atoms** include: lengths (`m km cm mm ft nmi`), time (`s ms min h Hz`), mass (`kg g`), force,
  pressure (`N Pa hPa`), energy and power (`J Wh kWh W kW`), electrical (`V A mA C Ah mAh ohm
  mohm F`), angle (`rad deg rpm`), temperature difference (`K`), and `%` and `-` for
  dimensionless.
* **Compound units** are parsed left to right with `*`, `/` and integer exponents (`m^2`,
  `m/s^2`). `a/b*c` is read as `(a/b)*c`. The result is rendered canonically (for example
  `km/h` gives `m/s`, `Ah` gives `C`).
* **Offset temperatures:** `degC` is accepted only as a stand-alone unit and converts to kelvin
  with the `273.15` offset. Temperature **differences** (such as the ISA offset) use `K`.
* **Errors:** an unknown or malformed unit raises `UnitError`.
* **Why it matters:** battery capacity in `Ah` becomes coulombs, `rpm` becomes rad/s, `%` becomes
  a fraction — all at load time. Nothing downstream converts again, which is what T-13 checks.

## The current register (`configs/vehicle.yaml`)

| Path | Value | Status |
| --- | --- | --- |
| `mass.m_kg` | — | **TBD** (the meaning of "5 kg" is open, O-12) |
| `wing.S_m2` | — | **TBD** |
| `wing.C_N` | — (uncertainty 0.3) | **TBD** |
| `rotors.n_lift` | 4 | stated |
| `rotors.ct_map` | file `../maps/ct_lift.csv` | assumed (the file does not exist yet) |
| `battery.packs_parallel` | 2 | stated |
| `battery.v_nominal_V` | 43.2 V | datasheet |
| `battery.capacity_Ah_per_pack` | 25 Ah | datasheet |
| `battery.ecm_file` | file `../maps/ecm_fit_v1.npz` | **TBD** |
| `ardupilot.AIRSPEED_MIN` | — | **TBD** (an analysis output) |
| `ardupilot.Q_TRANSITION_MS` | 5000 ms | documented |

Evidence counts for this file: 0 measured, 2 datasheet, 0 derived, 1 documented, 2 stated,
1 assumed, 5 TBD — so it is DRAFT. This is exactly what makes the demo release blocked.

## Tests

`tests/unit/test_config.py` (16 tests) and `test_units.py` (17 tests): loading the YAML, missing
or unknown fields, bad status, value/file exclusivity, error collection, **T-14** (a `TBD` makes
the config DRAFT and report writing is refused without an override), a complete config being
releasable, reading a `TBD` raising, evidence counts, hash determinism and sensitivity, and
**T-13** (a value in non-SI units is converted exactly once through the config).
