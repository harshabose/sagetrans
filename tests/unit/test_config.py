from pathlib import Path

import pytest

from sagetrans.config import (
    Config,
    ConfigError,
    DraftConfigError,
    MissingParameterError,
    Status,
)

VEHICLE_YAML = Path(__file__).resolve().parents[2] / "configs" / "vehicle.yaml"


def rec(**kw):
    base = {"value": 1.0, "unit": "m", "source": "test", "status": "assumed"}
    base.update(kw)
    return base


def test_vehicle_yaml_loads_and_is_draft():
    cfg = Config.from_yaml(VEHICLE_YAML)
    assert cfg.get("battery.v_nominal_V") == 43.2
    assert cfg.get("battery.capacity_Ah_per_pack") == 90_000.0  # coulombs
    assert cfg.get("ardupilot.Q_TRANSITION_MS") == 5.0  # seconds
    assert cfg.label == "DRAFT"


@pytest.mark.parametrize("missing", ["source", "status", "unit"])
def test_missing_field_fails_loading(missing):
    # FR-01: a missing field fails loading
    raw = rec()
    del raw[missing]
    with pytest.raises(ConfigError, match="g.p"):
        Config.from_dict({"g": {"p": raw}})


def test_unknown_field_fails_loading():
    with pytest.raises(ConfigError):
        Config.from_dict({"g": {"p": rec(colour="red")}})


def test_bad_status_fails_loading():
    with pytest.raises(ConfigError):
        Config.from_dict({"g": {"p": rec(status="probably")}})


def test_non_tbd_requires_value():
    with pytest.raises(ConfigError):
        Config.from_dict({"g": {"p": rec(value=None)}})


def test_value_and_file_exclusive():
    with pytest.raises(ConfigError):
        Config.from_dict({"g": {"p": rec(file="x.csv")}})


def test_errors_are_collected():
    with pytest.raises(ConfigError) as ei:
        Config.from_dict({"a": rec(unit=None), "b": rec(source=None)})
    assert "a" in str(ei.value) and "b" in str(ei.value)


def test_t14_tbd_makes_draft_and_report_refused_without_override():
    cfg = Config.from_dict({"g": {"p": rec(value=None, status="TBD")}})
    assert cfg.label == "DRAFT"
    with pytest.raises(DraftConfigError, match="g.p"):
        cfg.require_releasable()
    cfg.require_releasable(override=True)  # explicit override is allowed


def test_complete_config_is_releasable():
    cfg = Config.from_dict({"g": {"p": rec()}})
    assert cfg.label == "RELEASED"
    cfg.require_releasable()


def test_tbd_parameter_cannot_be_read():
    cfg = Config.from_dict({"g": {"p": rec(value=None, status="TBD")}})
    with pytest.raises(MissingParameterError):
        cfg.get("g.p")
    with pytest.raises(MissingParameterError):
        cfg.get("g.nope")


def test_evidence_counts():
    cfg = Config.from_dict(
        {"a": rec(), "b": rec(status="measured"), "c": rec(value=None, status="TBD")}
    )
    counts = cfg.evidence_counts()
    assert counts["assumed"] == 1 and counts["measured"] == 1 and counts["TBD"] == 1
    assert set(counts) == {s.value for s in Status}


def test_hash_deterministic_and_value_sensitive():
    a = Config.from_dict({"g": {"p": rec(value=1.0)}})
    b = Config.from_dict({"g": {"p": rec(value=1.0)}})
    c = Config.from_dict({"g": {"p": rec(value=1.1)}})
    assert a.config_hash() == b.config_hash()
    assert a.config_hash() != c.config_hash()


def test_hash_ignores_source_text_but_not_units_or_status():
    base = Config.from_dict({"g": {"p": rec(value=1000.0, unit="m")}})
    reworded = Config.from_dict({"g": {"p": rec(value=1000.0, unit="m", source="new citation")}})
    same_si = Config.from_dict({"g": {"p": rec(value=1.0, unit="km")}})
    new_status = Config.from_dict({"g": {"p": rec(value=1000.0, unit="m", status="measured")}})
    assert base.config_hash() == reworded.config_hash() == same_si.config_hash()
    assert base.config_hash() != new_status.config_hash()


def test_hash_tracks_file_contents(tmp_path):
    f = tmp_path / "map.csv"
    f.write_text("1,2\n")
    data = {"m": {"file": "map.csv", "source": "bench", "status": "assumed"}}
    h1 = Config.from_dict(data, root=tmp_path).config_hash()
    f.write_text("1,3\n")
    h2 = Config.from_dict(data, root=tmp_path).config_hash()
    assert h1 != h2
    f.unlink()
    assert Config.from_dict(data, root=tmp_path).config_hash() not in (h1, h2)
