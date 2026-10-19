"""Append-only persistence for live workstation runtime snapshots."""

from __future__ import annotations

import copy
import json
import math
import threading
import time
from datetime import date, datetime, timedelta, timezone
from numbers import Real
from pathlib import Path
from typing import Any

_SNAPSHOT_FIELDS = (
    "schema_version",
    "sequence",
    "connected",
    "frame_id",
    "robot",
    "traction",
    "motion_enabled",
)
_MAX_QUERY_SPAN_SECONDS = 31 * 24 * 60 * 60


class RuntimeRecorder:
    """Persist and query whitelisted runtime snapshots in UTC day files."""

    def __init__(self, root: Path):
        self._root = Path(root)
        self._lock = threading.Lock()

    def append(
        self, snapshot: dict[str, Any], timestamp: float | None = None
    ) -> dict[str, Any]:
        """Append one snapshot and return the independent persisted row."""
        if not isinstance(snapshot, dict):
            raise TypeError("snapshot must be a dictionary")

        timestamp_wall = _validate_timestamp(
            time.time() if timestamp is None else timestamp
        )
        persisted_snapshot = {
            field: copy.deepcopy(snapshot[field])
            for field in _SNAPSHOT_FIELDS
            if field in snapshot
        }
        row = {
            "timestamp_wall": timestamp_wall,
            "snapshot": persisted_snapshot,
        }
        path = self._root / f"runtime_{_utc_day(timestamp_wall):%Y%m%d}.jsonl"
        line = json.dumps(row, ensure_ascii=False, separators=(",", ":")) + "\n"

        with self._lock:
            self._root.mkdir(parents=True, exist_ok=True)
            with path.open("a", encoding="utf-8") as stream:
                stream.write(line)
                stream.flush()
        return row

    def rows_between(self, start: float, end: float) -> list[dict[str, Any]]:
        """Read rows in the inclusive UTC wall-time interval."""
        start_wall = _validate_timestamp(start)
        end_wall = _validate_timestamp(end)
        if end_wall < start_wall:
            raise ValueError("end must be greater than or equal to start")
        if end_wall - start_wall > _MAX_QUERY_SPAN_SECONDS:
            raise ValueError("query interval must not exceed 31 days")

        rows: list[dict[str, Any]] = []
        with self._lock:
            for day in _days_between(start_wall, end_wall):
                path = self._root / f"runtime_{day:%Y%m%d}.jsonl"
                try:
                    stream = path.open("r", encoding="utf-8")
                except FileNotFoundError:
                    continue
                with stream:
                    for line in stream:
                        row = _decode_row(line)
                        if row is None:
                            continue
                        if start_wall <= row["timestamp_wall"] <= end_wall:
                            rows.append(row)

        return sorted(rows, key=lambda row: row["timestamp_wall"])


def _validate_timestamp(value: Real) -> float:
    if isinstance(value, bool) or not isinstance(value, Real):
        raise TypeError("timestamp must be a finite real number")
    timestamp = float(value)
    if not math.isfinite(timestamp):
        raise ValueError("timestamp must be finite")
    return timestamp


def _utc_day(timestamp: float) -> datetime:
    return datetime.fromtimestamp(timestamp, tz=timezone.utc)


def _days_between(start: float, end: float) -> list[date]:
    current = _utc_day(start).date()
    last = _utc_day(end).date()
    days: list[date] = []
    while current <= last:
        days.append(current)
        current += timedelta(days=1)
    return days


def _decode_row(line: str) -> dict[str, Any] | None:
    try:
        row = json.loads(line)
    except (json.JSONDecodeError, TypeError):
        return None
    if not isinstance(row, dict):
        return None
    try:
        timestamp_wall = _validate_timestamp(row["timestamp_wall"])
        snapshot = row["snapshot"]
    except (KeyError, TypeError, ValueError):
        return None
    if not isinstance(snapshot, dict):
        return None
    return {
        "timestamp_wall": timestamp_wall,
        "snapshot": snapshot,
    }
