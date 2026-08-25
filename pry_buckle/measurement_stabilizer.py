"""Motion-grade multi-frame validation for heel measurements.

The live UI may display a held result, but only a fresh, stable raw result is
allowed to become a motion plan. This module deliberately keeps the gates
separate so an out-of-range but geometrically stable result is diagnosable.
"""
from __future__ import annotations

from collections import deque
from dataclasses import dataclass
from typing import Any

import numpy as np


def _mad(values: list[float]) -> float:
    if not values:
        return float("inf")
    array = np.asarray(values, dtype=np.float64)
    median = float(np.median(array))
    return float(np.median(np.abs(array - median)))


@dataclass(frozen=True)
class StabilizerConfig:
    sample_count: int = 8
    minimum_valid_frames: int = 6
    width_mad_max_mm: float = 1.5
    center_max_deviation_mm: float = 3.0
    angle_mad_max_deg: float = 2.0
    depth_valid_ratio_min: float = 0.85


class MeasurementStabilizer:
    """Accept only fresh, non-duplicate raw frames for motion planning."""

    def __init__(self, config: StabilizerConfig | None = None) -> None:
        self.config = config or StabilizerConfig()
        self._samples: deque[dict[str, Any]] = deque(maxlen=self.config.sample_count)
        self._last_frame_seq: int | None = None
        self._last_output: dict[str, Any] | None = None

    def reset(self) -> None:
        self._samples.clear()
        self._last_frame_seq = None
        self._last_output = None

    @property
    def sample_count(self) -> int:
        return len(self._samples)

    def update(self, raw: dict[str, Any], frame_seq: int, timestamp_monotonic: float) -> dict[str, Any]:
        result = dict(raw)
        result["frame_seq"] = int(frame_seq)
        result["measurement_time_monotonic"] = float(timestamp_monotonic)
        result.setdefault("display_only", False)
        result["motion_grade"] = False
        result["stable_valid"] = False
        result["measurement_status"] = "raw_invalid"

        if self._last_frame_seq == int(frame_seq):
            return dict(self._last_output or result)
        self._last_frame_seq = int(frame_seq)

        geometry_valid = bool(raw.get("geometry_valid", raw.get("valid", False)))
        if geometry_valid:
            self._samples.append({
                "result": dict(raw),
                "frame_seq": int(frame_seq),
                "timestamp_monotonic": float(timestamp_monotonic),
            })

        result["geometry_valid"] = geometry_valid
        result["stable_sample_count"] = len(self._samples)
        if len(self._samples) < self.config.minimum_valid_frames:
            result["valid"] = False
            result["message"] = (
                f"正在采集稳定视觉结果：{len(self._samples)}/"
                f"{self.config.minimum_valid_frames} 帧有效"
            )
            self._last_output = dict(result)
            return result

        samples = list(self._samples)
        widths = [float(item["result"]["width_mm"]) for item in samples]
        centers = [
            np.asarray(item["result"].get("center_camera_mm"), dtype=np.float64)
            for item in samples
            if item["result"].get("center_camera_mm") is not None
        ]
        angles = [float(item["result"].get("principal_angle_deg", 0.0)) for item in samples]
        depth_ratios = [float(item["result"].get("depth_valid_ratio", 0.0)) for item in samples]
        width_mad = _mad(widths)
        center_median = np.median(np.vstack(centers), axis=0) if centers else np.zeros(3)
        center_deviation = max(
            (float(np.linalg.norm(center - center_median)) for center in centers),
            default=float("inf"),
        )
        angle_mad = _mad(angles)
        depth_valid = bool(min(depth_ratios, default=0.0) >= self.config.depth_valid_ratio_min)
        stable = bool(
            width_mad <= self.config.width_mad_max_mm
            and center_deviation <= self.config.center_max_deviation_mm
            and angle_mad <= self.config.angle_mad_max_deg
            and depth_valid
        )
        latest = dict(samples[-1]["result"])
        latest["frame_seq"] = int(frame_seq)
        latest["measurement_time_monotonic"] = float(timestamp_monotonic)
        latest["stable_sample_count"] = len(samples)
        latest["stable_width_mad_mm"] = round(width_mad, 4)
        latest["stable_center_max_deviation_mm"] = round(center_deviation, 4)
        latest["stable_angle_mad_deg"] = round(angle_mad, 4)
        latest["stable_depth_valid_min_ratio"] = round(min(depth_ratios, default=0.0), 4)
        latest["stable_valid"] = stable
        latest["within_expected_width_range"] = bool(latest.get("within_expected_width_range", False))
        latest["motion_grade"] = bool(stable and latest["within_expected_width_range"])
        latest["valid"] = latest["motion_grade"]
        latest["display_only"] = False
        latest["measurement_status"] = "stabilized" if stable else "unstable"
        if latest["motion_grade"]:
            latest["message"] = "多帧视觉稳定，结果可用于冻结夹持规划"
        elif stable:
            latest["message"] = "视觉几何稳定，但宽度超出预期范围，禁止运动"
        else:
            latest["message"] = "多帧视觉尚未满足稳定性要求，禁止运动"
        self._last_output = dict(latest)
        return latest
