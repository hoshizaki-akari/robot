import json
import math
from datetime import datetime, timezone

import pytest

from backend.runtime_recorder import RuntimeRecorder


def _timestamp(value: str) -> float:
    return datetime.fromisoformat(value).replace(tzinfo=timezone.utc).timestamp()


def _snapshot() -> dict:
    return {
        "schema_version": "2.0",
        "sequence": 7,
        "connected": True,
        "frame_id": "base_link",
        "robot": {
            "identity_verified": True,
            "data_age_ms": 12,
            "joint_position_rad": [0.1, 0.2, 0.3],
            "ee_position_m": [0.4, 0.5, 0.6],
        },
        "traction": {
            "actual_force_n": 8.5,
            "data_age_ms": 9,
        },
        "motion_enabled": False,
        "history": {"summaries": ["must not be recorded"]},
        "secret": "must not be recorded",
    }


def test_append_persists_a_deep_copy_of_whitelisted_real_snapshot(tmp_path):
    recorder = RuntimeRecorder(tmp_path)
    snapshot = _snapshot()

    recorder.append(snapshot, timestamp=_timestamp("2026-10-19T23:59:59.250000"))
    snapshot["robot"]["joint_position_rad"][0] = 999.0
    snapshot["traction"]["actual_force_n"] = 999.0

    path = tmp_path / "runtime_20261019.jsonl"
    row = json.loads(path.read_text(encoding="utf-8"))
    assert row["timestamp_wall"] == 1792454399.25
    assert row["snapshot"]["robot"]["joint_position_rad"] == [0.1, 0.2, 0.3]
    assert row["snapshot"]["traction"]["actual_force_n"] == 8.5
    assert "history" not in row["snapshot"]
    assert "secret" not in row["snapshot"]


def test_append_uses_utc_day_names(tmp_path):
    recorder = RuntimeRecorder(tmp_path)

    recorder.append(_snapshot(), timestamp=_timestamp("2026-10-19T23:30:00"))

    assert (tmp_path / "runtime_20261019.jsonl").exists()
    assert not (tmp_path / "runtime_20261020.jsonl").exists()


def test_rows_between_filters_inclusively_and_sorts_rows(tmp_path):
    recorder = RuntimeRecorder(tmp_path)
    recorder.append({**_snapshot(), "sequence": 3}, timestamp=200.0)
    recorder.append({**_snapshot(), "sequence": 1}, timestamp=100.0)
    recorder.append({**_snapshot(), "sequence": 2}, timestamp=150.0)

    rows = recorder.rows_between(100.0, 200.0)

    assert [row["timestamp_wall"] for row in rows] == [100.0, 150.0, 200.0]
    assert [row["snapshot"]["sequence"] for row in rows] == [1, 2, 3]


def test_rows_between_reads_across_midnight_and_ignores_missing_days(tmp_path):
    recorder = RuntimeRecorder(tmp_path)
    before_midnight = _timestamp("2026-10-19T23:59:59")
    after_midnight = _timestamp("2026-10-20T00:00:01")
    recorder.append(_snapshot(), timestamp=after_midnight)
    recorder.append(_snapshot(), timestamp=before_midnight)

    rows = recorder.rows_between(before_midnight, after_midnight)

    assert [row["timestamp_wall"] for row in rows] == [
        before_midnight,
        after_midnight,
    ]


def test_rows_between_skips_malformed_and_incomplete_lines(tmp_path):
    recorder = RuntimeRecorder(tmp_path)
    path = tmp_path / "runtime_19700101.jsonl"
    valid = recorder.append(_snapshot(), timestamp=1.0)
    path.write_text(
        json.dumps(valid) + "\n{not json}\n{\"timestamp_wall\": 2",
        encoding="utf-8",
    )

    rows = recorder.rows_between(0.0, 3.0)

    assert rows == [valid]


def test_invalid_timestamps_and_spans_are_rejected(tmp_path):
    recorder = RuntimeRecorder(tmp_path)

    for value in (math.nan, math.inf, -math.inf):
        with pytest.raises(ValueError):
            recorder.append(_snapshot(), timestamp=value)
        with pytest.raises(ValueError):
            recorder.rows_between(0.0, value)

    with pytest.raises(ValueError):
        recorder.rows_between(2.0, 1.0)
    with pytest.raises(ValueError):
        recorder.rows_between(0.0, 31 * 24 * 60 * 60 + 0.1)


def test_rows_between_returns_empty_for_missing_files(tmp_path):
    recorder = RuntimeRecorder(tmp_path)

    assert recorder.rows_between(0.0, 10.0) == []
