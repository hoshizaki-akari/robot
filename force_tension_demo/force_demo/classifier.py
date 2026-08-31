"""Force baseline, motion gating and slack/tension classification."""

from __future__ import annotations

import csv
import json
import math
import statistics
import threading
import time
from collections import deque
from collections.abc import Callable, Iterable
from dataclasses import dataclass
from datetime import datetime, timezone
from pathlib import Path
from typing import ClassVar

Vector3 = tuple[float, float, float]


def finite_vector(values: Iterable[float]) -> Vector3:
    vector = tuple(float(value) for value in values)
    if len(vector) != 3 or not all(math.isfinite(value) for value in vector):
        raise ValueError("force vector must contain three finite values")
    return vector  # type: ignore[return-value]


def magnitude(vector: Vector3) -> float:
    return math.sqrt(sum(value * value for value in vector))


def median_vector(vectors: list[Vector3]) -> Vector3:
    return tuple(statistics.median(v[index] for v in vectors) for index in range(3))  # type: ignore[return-value]


def direction(vector: Vector3, minimum: float = 0.2) -> Vector3 | None:
    length = magnitude(vector)
    return None if length < minimum else tuple(v / length for v in vector)  # type: ignore[return-value]


def dominant_axis(vector: Vector3 | None) -> str:
    if vector is None:
        return "--"
    index = max(range(3), key=lambda item: abs(vector[item]))
    return f"{'XYZ'[index]}{'+' if vector[index] >= 0 else '-'}"


@dataclass(frozen=True)
class EngineConfig:
    baseline_duration_s: float = 2.0
    slack_threshold_n: float = 0.5
    tension_threshold_n: float = 1.0
    warning_force_n: float = 10.0
    motion_joint_speed_deg_s: float = 0.15
    max_orientation_change_deg: float = 1.0
    settle_after_motion_s: float = 0.4
    tension_confirm_s: float = 0.2
    slack_confirm_s: float = 0.5
    transition_confirm_s: float = 0.15
    data_stale_s: float = 0.8
    filter_cutoff_hz: float = 5.0
    stability_window_s: float = 0.25
    stability_std_n: float = 0.28
    baseline_max_component_span_n: float = 0.8
    baseline_timeout_s: float = 8.0
    increase_direction_sign: int = -1

    def __post_init__(self) -> None:
        if not (0.0 <= self.slack_threshold_n < self.tension_threshold_n < self.warning_force_n):
            raise ValueError("thresholds must satisfy 0 <= slack < tension < warning")
        positive = (
            self.baseline_duration_s,
            self.motion_joint_speed_deg_s,
            self.max_orientation_change_deg,
            self.settle_after_motion_s,
            self.tension_confirm_s,
            self.slack_confirm_s,
            self.data_stale_s,
            self.filter_cutoff_hz,
        )
        if any(value <= 0.0 for value in positive):
            raise ValueError("timing, speed and filter values must be positive")
        if self.increase_direction_sign not in (-1, 1):
            raise ValueError("increase_direction_sign must be -1 or 1")

    @classmethod
    def from_mapping(cls, values: dict[str, object]) -> EngineConfig:
        known = set(cls.__dataclass_fields__)
        return cls(**{key: value for key, value in values.items() if key in known})


@dataclass(frozen=True)
class SensorSample:
    monotonic_time: float
    wall_time: str
    force_n: Vector3
    torque_nm: Vector3 = (0.0, 0.0, 0.0)
    frame_id: str = "base_link"
    source: str = "unknown"
    source_detail: str = ""
    priority: int = 0
    motion_available: bool = False
    max_joint_speed_deg_s: float = 0.0
    tcp_position_mm: Vector3 | None = None
    tcp_rpy_deg: Vector3 | None = None

    @classmethod
    def create(
        cls,
        force_n: Iterable[float],
        *,
        torque_nm: Iterable[float] = (0.0, 0.0, 0.0),
        monotonic_time: float | None = None,
        wall_time: str | None = None,
        **kwargs: object,
    ) -> SensorSample:
        return cls(
            monotonic_time=time.monotonic() if monotonic_time is None else float(monotonic_time),
            wall_time=wall_time or datetime.now(timezone.utc).isoformat(timespec="milliseconds"),
            force_n=finite_vector(force_n),
            torque_nm=finite_vector(torque_nm),
            **kwargs,
        )


class CsvRecorder:
    HEADER: ClassVar[tuple[str, ...]] = (
        "wall_time", "source", "source_detail", "frame_id",
        "raw_fx_n", "raw_fy_n", "raw_fz_n",
        "baseline_fx_n", "baseline_fy_n", "baseline_fz_n",
        "relative_fx_n", "relative_fy_n", "relative_fz_n",
        "resultant_n", "state", "moving", "max_joint_speed_deg_s", "warning",
    )

    def __init__(self, root: Path) -> None:
        stamp = datetime.now(timezone.utc).astimezone().strftime("%Y%m%d_%H%M%S")
        self.session_dir = root / f"session_{stamp}"
        self.session_dir.mkdir(parents=True, exist_ok=True)
        self.csv_path = self.session_dir / "force_samples.csv"
        self.diagnostics_path = self.session_dir / "diagnostics.json"
        self._handle = self.csv_path.open("w", newline="", encoding="utf-8")
        self._writer = csv.DictWriter(self._handle, fieldnames=self.HEADER)
        self._writer.writeheader()
        self._last_flush = time.monotonic()
        self._lock = threading.Lock()

    def write(self, row: dict[str, object]) -> None:
        with self._lock:
            self._writer.writerow({key: row.get(key, "") for key in self.HEADER})
            if time.monotonic() - self._last_flush >= 1.0:
                self._handle.flush()
                self._last_flush = time.monotonic()

    def diagnostics(self, payload: dict[str, object]) -> None:
        with self._lock:
            self.diagnostics_path.write_text(
                json.dumps(payload, ensure_ascii=False, indent=2), encoding="utf-8"
            )

    def flush(self) -> None:
        with self._lock:
            self._handle.flush()

    def close(self) -> None:
        with self._lock:
            if not self._handle.closed:
                self._handle.flush()
                self._handle.close()


class ForceTensionEngine:
    """Thread-safe processing state shared by all real sensor transports."""

    def __init__(
        self,
        config: EngineConfig | None = None,
        recorder: CsvRecorder | None = None,
        clock: Callable[[], float] = time.monotonic,
    ) -> None:
        self.config = config or EngineConfig()
        self._clock = clock
        self._recorder = recorder
        self._lock = threading.RLock()
        self._baseline: Vector3 | None = None
        self._baseline_requested = False
        self._baseline_started_at: float | None = None
        self._baseline_request_time: float | None = None
        self._baseline_samples: list[Vector3] = []
        self._baseline_orientation_samples: list[Vector3] = []
        self._baseline_orientation_deg: Vector3 | None = None
        self._baseline_message = "请在绳带松动、机械臂静止时设置基线"
        self._source = "none"
        self._source_detail = "等待真实传感器"
        self._source_priority = -1
        self._frame_id = "base_link"
        self._last_sample_time: float | None = None
        self._last_wall_time = ""
        self._raw_force: Vector3 = (0.0, 0.0, 0.0)
        self._filtered_force: Vector3 = (0.0, 0.0, 0.0)
        self._resultant_n = 0.0
        self._moving = False
        self._motion_available = False
        self._max_joint_speed = 0.0
        self._orientation_available = False
        self._orientation_change_deg = 0.0
        self._stationary_since: float | None = None
        self._phase = "waiting_data"
        self._phase_text = "等待 KWR75D 数据"
        self._confirmed_state = "unknown"
        self._candidate_state: str | None = None
        self._candidate_since: float | None = None
        self._relative_window: deque[tuple[float, Vector3]] = deque(maxlen=500)
        self._rate_window: deque[float] = deque(maxlen=500)
        self._history: deque[dict[str, object]] = deque(maxlen=1800)
        self._direction_sign = -1 if self.config.increase_direction_sign < 0 else 1
        self._last_error = ""
        self._diagnostics: dict[str, object] = {}

    def source_is_fresh(self, minimum_priority: int = 0) -> bool:
        with self._lock:
            return (
                self._last_sample_time is not None
                and self._clock() - self._last_sample_time <= self.config.data_stale_s
                and self._source_priority >= minimum_priority
            )

    def set_diagnostic(self, key: str, value: object) -> None:
        with self._lock:
            self._diagnostics[key] = value

    def set_error(self, message: str) -> None:
        with self._lock:
            self._last_error = message

    def begin_baseline(self) -> tuple[bool, str]:
        with self._lock:
            if not self.source_is_fresh():
                return False, "没有新鲜的真实传感器数据，不能设置基线"
            if self._moving:
                return False, "机械臂仍在移动，请停稳后再设置基线"
            if self._baseline is not None and (
                self._confirmed_state == "tension"
                or self._resultant_n >= self.config.tension_threshold_n
            ):
                return False, "当前仍有明显牵引力，请先真正松绳再重新设置基线"
            self._baseline = None
            self._baseline_requested = True
            self._baseline_started_at = None
            self._baseline_request_time = self._clock()
            self._baseline_samples.clear()
            self._baseline_orientation_samples.clear()
            self._baseline_orientation_deg = None
            self._filtered_force = (0.0, 0.0, 0.0)
            self._resultant_n = 0.0
            self._relative_window.clear()
            self._history.clear()
            self._confirmed_state = "unknown"
            self._candidate_state = None
            self._candidate_since = None
            self._stationary_since = None
            self._orientation_change_deg = 0.0
            self._phase = "baseline_capturing"
            self._phase_text = "正在采集松绳基线"
            self._baseline_message = "保持绳带松动、机械臂静止、法兰朝向不变"
            return True, "已开始采集松绳基线"

    def clear_baseline(self, message: str = "基线已清除") -> None:
        with self._lock:
            self._reset_baseline(message)

    def _reset_baseline(self, message: str) -> None:
        self._baseline = None
        self._baseline_requested = False
        self._baseline_started_at = None
        self._baseline_request_time = None
        self._baseline_samples.clear()
        self._baseline_orientation_samples.clear()
        self._baseline_orientation_deg = None
        self._filtered_force = (0.0, 0.0, 0.0)
        self._resultant_n = 0.0
        self._relative_window.clear()
        self._history.clear()
        self._candidate_state = None
        self._candidate_since = None
        self._confirmed_state = "unknown"
        self._orientation_available = False
        self._orientation_change_deg = 0.0
        self._baseline_message = message

    def reverse_increase_direction(self) -> int:
        with self._lock:
            self._direction_sign *= -1
            return self._direction_sign

    def ingest(self, sample: SensorSample) -> bool:
        if sample.frame_id != "base_link":
            self.set_error(f"拒绝非 base_link 数据：{sample.frame_id or '空 frame_id'}")
            return False
        with self._lock:
            now = sample.monotonic_time
            current_fresh = self._last_sample_time is not None and now - self._last_sample_time <= self.config.data_stale_s
            if current_fresh and sample.priority < self._source_priority:
                return False
            source_changed = self._source != sample.source or self._source_detail != sample.source_detail
            if source_changed and self._last_sample_time is not None:
                self._reset_baseline("真实数据来源发生变化，请重新设置松绳基线")
            self._source = sample.source
            self._source_detail = sample.source_detail
            self._source_priority = sample.priority
            self._frame_id = sample.frame_id
            previous_time = self._last_sample_time
            self._last_sample_time = now
            self._last_wall_time = sample.wall_time
            self._raw_force = sample.force_n
            self._motion_available = sample.motion_available
            self._max_joint_speed = abs(sample.max_joint_speed_deg_s)
            self._moving = sample.motion_available and self._max_joint_speed >= self.config.motion_joint_speed_deg_s
            self._orientation_available = sample.tcp_rpy_deg is not None
            self._rate_window.append(now)
            self._process_baseline(sample)
            if self._baseline is not None:
                self._process_relative(sample, previous_time)
            elif not self._baseline_requested:
                self._phase = "need_baseline"
                self._phase_text = "数据已连接，请设置松绳基线"
            try:
                self._record(sample)
            except OSError as error:
                self._last_error = f"CSV 记录失败，实时显示继续：{error}"
            return True

    def _process_baseline(self, sample: SensorSample) -> None:
        if not self._baseline_requested:
            return
        now = sample.monotonic_time
        if self._baseline_request_time is not None and now - self._baseline_request_time > self.config.baseline_timeout_s:
            self._reset_baseline("基线采集超时：请保持静止后重试")
            return
        if self._moving:
            self._baseline_started_at = None
            self._baseline_samples.clear()
            self._phase_text = "检测到机械臂移动，基线计时已重新开始"
            return
        if self._baseline_started_at is None:
            self._baseline_started_at = now
        self._baseline_samples.append(sample.force_n)
        if sample.tcp_rpy_deg is not None:
            self._baseline_orientation_samples.append(sample.tcp_rpy_deg)
        duration = now - self._baseline_started_at
        if duration < self.config.baseline_duration_s:
            self._phase = "baseline_capturing"
            self._phase_text = f"正在采集松绳基线 {duration:.1f}/{self.config.baseline_duration_s:.1f} 秒"
            return
        spans = [max(v[i] for v in self._baseline_samples) - min(v[i] for v in self._baseline_samples) for i in range(3)]
        if max(spans) > self.config.baseline_max_component_span_n:
            self._baseline_started_at = now
            self._baseline_samples.clear()
            self._phase_text = "力仍在波动，正在重新采集稳定数据"
            return
        self._baseline = median_vector(self._baseline_samples)
        self._baseline_orientation_deg = (
            median_vector(self._baseline_orientation_samples)
            if self._baseline_orientation_samples else None
        )
        self._baseline_requested = False
        self._baseline_request_time = None
        self._baseline_message = "松绳基线已建立"
        self._stationary_since = now
        self._phase = "settling"
        self._phase_text = "基线完成，正在确认松绳状态"

    def _process_relative(self, sample: SensorSample, previous_time: float | None) -> None:
        assert self._baseline is not None
        now = sample.monotonic_time
        relative = tuple(sample.force_n[i] - self._baseline[i] for i in range(3))
        dt = 0.05 if previous_time is None else max(0.001, min(0.25, now - previous_time))
        alpha = 1.0 - math.exp(-2.0 * math.pi * self.config.filter_cutoff_hz * dt)
        self._filtered_force = tuple(self._filtered_force[i] + alpha * (relative[i] - self._filtered_force[i]) for i in range(3))  # type: ignore[assignment]
        self._resultant_n = magnitude(self._filtered_force)
        self._relative_window.append((now, self._filtered_force))
        cutoff = now - self.config.stability_window_s
        while self._relative_window and self._relative_window[0][0] < cutoff:
            self._relative_window.popleft()
        self._history.append({
            "t": sample.wall_time, "monotonic": now,
            "force_n": round(self._resultant_n, 4),
            "fx": round(self._filtered_force[0], 4),
            "fy": round(self._filtered_force[1], 4),
            "fz": round(self._filtered_force[2], 4),
        })
        if sample.tcp_rpy_deg is not None and self._baseline_orientation_deg is not None:
            differences = [
                abs((sample.tcp_rpy_deg[index] - self._baseline_orientation_deg[index] + 180.0) % 360.0 - 180.0)
                for index in range(3)
            ]
            self._orientation_change_deg = max(differences)
            if self._orientation_change_deg > self.config.max_orientation_change_deg:
                self._stationary_since = None
                self._candidate_state = None
                self._candidate_since = None
                self._phase = "orientation_changed"
                self._phase_text = "法兰朝向已变化：请恢复基线朝向或重新设置基线"
                return
        else:
            self._orientation_change_deg = 0.0
        if self._moving:
            self._stationary_since = None
            self._candidate_state = None
            self._candidate_since = None
            self._phase = "moving"
            self._phase_text = "机械臂移动中：显示实时力，暂不判定松紧"
            return
        if self._stationary_since is None:
            self._stationary_since = now
        if now - self._stationary_since < self.config.settle_after_motion_s:
            self._phase = "settling"
            self._phase_text = "机械臂已停，正在排除惯性力"
            return
        if not self._is_stable():
            self._phase = "unstable"
            self._phase_text = "受力仍在波动，等待稳定后判定"
            self._candidate_state = None
            self._candidate_since = None
            return
        # Full hysteresis: after a state is confirmed, keep it throughout the
        # dead band. This prevents small KWR75D quantisation/noise near 0.5 N
        # from making the operator display flicker.
        if self._confirmed_state == "slack" and self._resultant_n < self.config.tension_threshold_n:
            candidate, confirm_s = "slack", self.config.slack_confirm_s
        elif self._confirmed_state == "tension" and self._resultant_n > self.config.slack_threshold_n or self._resultant_n >= self.config.tension_threshold_n:
            candidate, confirm_s = "tension", self.config.tension_confirm_s
        elif self._resultant_n <= self.config.slack_threshold_n:
            candidate, confirm_s = "slack", self.config.slack_confirm_s
        else:
            candidate, confirm_s = "transition", self.config.transition_confirm_s
        if candidate != self._candidate_state:
            self._candidate_state, self._candidate_since = candidate, now
        if self._candidate_since is None or now - self._candidate_since < confirm_s:
            if self._confirmed_state in ("slack", "tension"):
                self._phase = self._confirmed_state
                base_text = "绳带松动" if self._confirmed_state == "slack" else "绳带已张紧"
                self._phase_text = f"{base_text}（正在复核变化）"
            else:
                self._phase, self._phase_text = "confirming", "稳定数据确认中"
            return
        self._confirmed_state = self._phase = candidate
        self._phase_text = {"slack": "绳带松动", "tension": "绳带已张紧，检测到牵引力", "transition": "松紧过渡区"}[candidate]

    def _is_stable(self) -> bool:
        values = [magnitude(vector) for _, vector in self._relative_window]
        return len(values) >= 3 and statistics.pstdev(values) <= self.config.stability_std_n

    def _sample_rate(self, now: float) -> float:
        while self._rate_window and self._rate_window[0] < now - 2.0:
            self._rate_window.popleft()
        if len(self._rate_window) < 2:
            return 0.0
        elapsed = self._rate_window[-1] - self._rate_window[0]
        return 0.0 if elapsed <= 0 else (len(self._rate_window) - 1) / elapsed

    def snapshot(self, include_history: bool = False) -> dict[str, object]:
        with self._lock:
            now = self._clock()
            age_s = None if self._last_sample_time is None else max(0.0, now - self._last_sample_time)
            connected = age_s is not None and age_s <= self.config.data_stale_s
            phase = self._phase if connected else "disconnected"
            phase_text = self._phase_text if connected else "KWR75D 数据已断开或超时"
            # A direction below the tension threshold is mostly sensor noise
            # and would mislead the operator while the band is still slack.
            actual = direction(self._filtered_force, self.config.tension_threshold_n)
            increase = None if actual is None else tuple(self._direction_sign * v for v in actual)
            progress = 0.0
            if self._baseline_requested and self._baseline_started_at is not None:
                progress = min(1.0, (now - self._baseline_started_at) / self.config.baseline_duration_s)
            warning = bool(
                connected
                and self._baseline is not None
                and self._resultant_n > self.config.warning_force_n
            )
            payload: dict[str, object] = {
                "connected": connected, "sensor_model": "KWR75D",
                "source": self._source, "source_detail": self._source_detail,
                "frame_id": self._frame_id, "last_sample_wall_time": self._last_wall_time,
                "data_age_ms": None if age_s is None else round(age_s * 1000),
                "sample_rate_hz": round(self._sample_rate(now), 1),
                "phase": phase, "phase_text": phase_text,
                "confirmed_state": self._confirmed_state,
                "raw_force_n": [round(v, 4) for v in self._raw_force],
                "baseline_force_n": None if self._baseline is None else [round(v, 4) for v in self._baseline],
                "relative_force_n": [round(v, 4) for v in self._filtered_force],
                "resultant_force_n": round(self._resultant_n, 4),
                "actual_force_direction": None if actual is None else [round(v, 5) for v in actual],
                "actual_force_axis": dominant_axis(actual),
                "increase_motion_direction": None if increase is None else [round(v, 5) for v in increase],
                "increase_motion_axis": dominant_axis(increase),
                "increase_direction_sign": self._direction_sign,
                "motion_available": self._motion_available, "moving": self._moving,
                "max_joint_speed_deg_s": round(self._max_joint_speed, 4),
                "orientation_available": self._orientation_available,
                "orientation_change_deg": round(self._orientation_change_deg, 4),
                "baseline_orientation_deg": None if self._baseline_orientation_deg is None else [round(v, 4) for v in self._baseline_orientation_deg],
                "baseline_ready": self._baseline is not None,
                "baseline_capturing": self._baseline_requested,
                "baseline_progress": round(progress, 3),
                "baseline_message": self._baseline_message,
                "warning": warning,
                "warning_text": f"牵引力已超过 {self.config.warning_force_n:g} N，请停止继续拉紧" if warning else "",
                "thresholds": {
                    "slack_n": self.config.slack_threshold_n,
                    "tension_n": self.config.tension_threshold_n,
                    "warning_n": self.config.warning_force_n,
                    "motion_joint_speed_deg_s": self.config.motion_joint_speed_deg_s,
                    "max_orientation_change_deg": self.config.max_orientation_change_deg,
                    "settle_after_motion_s": self.config.settle_after_motion_s,
                },
                "last_error": self._last_error, "diagnostics": dict(self._diagnostics),
            }
            if include_history:
                payload["history"] = list(self._history)
            return payload

    def _record(self, sample: SensorSample) -> None:
        if self._recorder is None:
            return
        base = self._baseline or (0.0, 0.0, 0.0)
        self._recorder.write({
            "wall_time": sample.wall_time, "source": sample.source,
            "source_detail": sample.source_detail, "frame_id": sample.frame_id,
            "raw_fx_n": sample.force_n[0], "raw_fy_n": sample.force_n[1], "raw_fz_n": sample.force_n[2],
            "baseline_fx_n": base[0], "baseline_fy_n": base[1], "baseline_fz_n": base[2],
            "relative_fx_n": self._filtered_force[0], "relative_fy_n": self._filtered_force[1], "relative_fz_n": self._filtered_force[2],
            "resultant_n": self._resultant_n, "state": self._phase,
            "moving": self._moving, "max_joint_speed_deg_s": self._max_joint_speed,
            "warning": self._resultant_n > self.config.warning_force_n,
        })

    def close(self) -> None:
        if self._recorder is not None:
            self._recorder.diagnostics(self.snapshot())
            self._recorder.close()
