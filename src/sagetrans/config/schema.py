"""Parameter register: every parameter carries value, unit, source and status (FR-01)."""

from __future__ import annotations

import hashlib
import json
from collections import Counter
from collections.abc import Iterator, Mapping
from enum import StrEnum
from pathlib import Path
from typing import Any

import yaml
from pydantic import BaseModel, ConfigDict, ValidationError, model_validator

from sagetrans.config.units import to_si


class Status(StrEnum):
    """Fixed evidence-grade vocabulary (section 9)."""

    MEASURED = "measured"
    DATASHEET = "datasheet"
    DERIVED = "derived"
    DOCUMENTED = "documented"
    STATED = "stated"
    ASSUMED = "assumed"
    TBD = "TBD"


class ConfigError(ValueError):
    """The configuration failed schema validation."""


class MissingParameterError(KeyError):
    """A parameter was requested but has no value (TBD or absent)."""


class DraftConfigError(RuntimeError):
    """A configuration containing TBD parameters was used for a release output (T-14)."""


class ParamRecord(BaseModel):
    """One parameter. `value`/`unit` are as authored; `si_value`/`si_unit` are derived once."""

    model_config = ConfigDict(frozen=True, extra="forbid")

    value: float | int | None = None
    unit: str | None = None
    file: str | None = None
    source: str
    status: Status
    uncertainty: float | None = None

    # derived once at construction; never recomputed
    si_value: float | None = None
    si_unit: str | None = None

    @model_validator(mode="after")
    def _check(self) -> ParamRecord:
        if self.value is not None and self.file is not None:
            raise ValueError("a record has either value or file, not both")
        if self.status is not Status.TBD and self.value is None and self.file is None:
            raise ValueError(f"status {self.status.value!r} requires a value or a file")
        if self.value is not None and self.unit is None:
            raise ValueError("a record with a value requires a unit ('-' for dimensionless)")
        if self.value is not None and self.unit is not None:
            conv = to_si(float(self.value), self.unit)
            object.__setattr__(self, "si_value", conv.value)
            object.__setattr__(self, "si_unit", conv.unit)
        return self


def _is_record(node: Mapping[str, Any]) -> bool:
    return "status" in node or "source" in node


def _walk(node: Mapping[str, Any], prefix: str = "") -> Iterator[tuple[str, Mapping[str, Any]]]:
    for key, child in node.items():
        path = f"{prefix}.{key}" if prefix else str(key)
        if not isinstance(child, Mapping):
            raise ConfigError(f"{path}: expected a parameter record or a group, got {child!r}")
        if _is_record(child):
            yield path, child
        else:
            yield from _walk(child, path)


class Config:
    """A validated, immutable set of parameter records plus its on-disk root for map files."""

    def __init__(self, records: Mapping[str, ParamRecord], root: Path | None = None) -> None:
        self._records: dict[str, ParamRecord] = dict(sorted(records.items()))
        self.root = root

    # ---- construction -------------------------------------------------------------------
    @classmethod
    def from_dict(cls, data: Mapping[str, Any], root: Path | None = None) -> Config:
        records: dict[str, ParamRecord] = {}
        errors: list[str] = []
        for path, raw in _walk(data):
            try:
                records[path] = ParamRecord.model_validate(dict(raw))
            except ValidationError as exc:
                for err in exc.errors():
                    loc = ".".join(str(p) for p in err["loc"])
                    errors.append(f"{path}{'.' + loc if loc else ''}: {err['msg']}")
        if errors:
            raise ConfigError("invalid configuration:\n  " + "\n  ".join(errors))
        return cls(records, root)

    @classmethod
    def from_yaml(cls, path: str | Path) -> Config:
        path = Path(path)
        with path.open(encoding="utf-8") as fh:
            data = yaml.safe_load(fh)
        if not isinstance(data, Mapping):
            raise ConfigError(f"{path}: top level must be a mapping")
        return cls.from_dict(data, root=path.parent)

    # ---- access -------------------------------------------------------------------------
    def __contains__(self, path: str) -> bool:
        return path in self._records

    def __iter__(self) -> Iterator[str]:
        return iter(self._records)

    def record(self, path: str) -> ParamRecord:
        try:
            return self._records[path]
        except KeyError:
            raise MissingParameterError(f"unknown parameter {path!r}") from None

    def get(self, path: str) -> float:
        """The parameter in SI units. Raises if it is still TBD or is a file record."""
        rec = self.record(path)
        if rec.si_value is None:
            raise MissingParameterError(f"parameter {path!r} has no value (status {rec.status})")
        return rec.si_value

    def file_path(self, path: str) -> Path:
        rec = self.record(path)
        if rec.file is None:
            raise MissingParameterError(f"parameter {path!r} is not a file record")
        return (self.root or Path.cwd()) / rec.file

    # ---- evidence grade, draft label, hash ----------------------------------------------
    def evidence_counts(self) -> dict[str, int]:
        counts = Counter(r.status.value for r in self._records.values())
        return {s.value: counts.get(s.value, 0) for s in Status}

    @property
    def is_draft(self) -> bool:
        return any(r.status is Status.TBD for r in self._records.values())

    @property
    def label(self) -> str:
        return "DRAFT" if self.is_draft else "RELEASED"

    def require_releasable(self, override: bool = False) -> None:
        """Refuse report writing for a DRAFT configuration unless explicitly overridden."""
        if self.is_draft and not override:
            tbd = sorted(p for p, r in self._records.items() if r.status is Status.TBD)
            raise DraftConfigError(
                "configuration is DRAFT; TBD parameters: " + ", ".join(tbd)
                + " (pass override=True to write anyway)"
            )

    def config_hash(self) -> str:
        """SHA-256 over what determines a result: SI values, status, uncertainty, file contents.

        The free-text `source` is documentation and is excluded, so rewording a citation does
        not change the hash. A referenced file contributes its content digest, or 'missing'.
        """
        payload: dict[str, Any] = {}
        for path, rec in self._records.items():
            entry: dict[str, Any] = {
                "si_value": rec.si_value,
                "si_unit": rec.si_unit,
                "status": rec.status.value,
                "uncertainty": rec.uncertainty,
            }
            if rec.file is not None:
                target = self.file_path(path)
                entry["file"] = "missing"
                if target.is_file():
                    entry["file"] = hashlib.sha256(target.read_bytes()).hexdigest()
            payload[path] = entry
        blob = json.dumps(payload, sort_keys=True, separators=(",", ":"), allow_nan=False)
        return hashlib.sha256(blob.encode()).hexdigest()
