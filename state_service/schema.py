from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
from time import monotonic
from typing import Any


SCHEMA_VERSION = "1.0"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def device_state(valid: bool, **values: Any) -> dict[str, Any]:
    return {
        "valid": valid,
        "age_ms": 0,
        "timestamp": utc_now(),
        "_updated_monotonic": monotonic(),
        **values,
    }


def unavailable_snapshot(source: str, message: str) -> dict[str, Any]:
    common = {"valid": False, "message": message}
    return {
        "schema_version": SCHEMA_VERSION,
        "sequence": 0,
        "timestamp": utc_now(),
        "source": source,
        "system": device_state(False, message=message),
        "fr5": device_state(
            **common,
            connected=False,
            mode="unknown",
            enabled=False,
            errors={"main": None, "sub": None},
            emergency_stop=None,
            safety_stop=[None, None],
            motion_done=None,
            joint_position_deg=[],
            joint_velocity_deg_s=[],
            flange_pose_mm_deg=[],
            tcp_pose_mm_deg=[],
            joint_torque_raw=[],
            frame_id="base",
        ),
        "kwr75d": device_state(
            **common,
            connected=False,
            wrench=[0.0] * 6,
            frame_id="RCS",
            units=["N", "N", "N", "Nm", "Nm", "Nm"],
        ),
        "ag95": device_state(
            **common,
            connected=False,
            initialized=False,
            init_status=None,
            motion_status=None,
            position_raw=None,
            fault=None,
            timeout=False,
            frame_id="ag95",
        ),
        "d435": device_state(
            **common,
            connected=False,
            color_fps=0.0,
            depth_fps=0.0,
            last_color_frame=None,
            last_depth_frame=None,
            frame_id="camera_link",
        ),
    }


def public_snapshot(snapshot: dict[str, Any]) -> dict[str, Any]:
    now = monotonic()
    result = deepcopy(snapshot)
    result["timestamp"] = utc_now()
    for value in result.values():
        if not isinstance(value, dict):
            continue
        updated = value.pop("_updated_monotonic", None)
        if updated is not None:
            value["age_ms"] = max(0, int((now - updated) * 1000))
    return result

