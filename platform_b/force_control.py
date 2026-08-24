"""平台 B 的恒力牵引计算核心。

人手只用来确定牵引方向。方向锁定后，控制器只计算该方向上的力，
并持续给出机械臂下一小步应往哪里走。模块本身不连接机械臂。
"""

from __future__ import annotations

import json
import math
import threading
import time
from collections import deque
from dataclasses import asdict, dataclass
from pathlib import Path
from typing import Any, Sequence


CONFIG_PATH = Path(__file__).with_name("config.json")


@dataclass(frozen=True)
class ForceControlResult:
    force_vector_n: tuple[float, float, float]
    force_magnitude_n: float
    traction_force_n: float
    lateral_force_n: float
    direction: tuple[float, float, float] | None
    error_n: float
    movement_mm: tuple[float, float, float]
    speed_mm_s: float
    state: str


def load_force_config(path: Path = CONFIG_PATH) -> dict[str, Any]:
    with path.open(encoding="utf-8") as file:
        return json.load(file)["force_control"]


def _vector(values: Sequence[float]) -> tuple[float, float, float]:
    if len(values) < 3:
        raise ValueError("力数据至少需要 Fx、Fy、Fz 三个数")
    return float(values[0]), float(values[1]), float(values[2])


def _length(vector: Sequence[float]) -> float:
    return math.sqrt(sum(float(value) ** 2 for value in vector))


def _dot(left: Sequence[float], right: Sequence[float]) -> float:
    return sum(float(a) * float(b) for a, b in zip(left, right))


def _normalise(vector: Sequence[float]) -> tuple[float, float, float]:
    length = _length(vector)
    if length <= 1e-9:
        raise ValueError("牵引方向不能是零向量")
    return tuple(float(value) / length for value in vector)


def calculate_step(
    wrench: Sequence[float],
    baseline: Sequence[float],
    target_force_n: float,
    direction: Sequence[float] | None = None,
    dt_s: float = 0.25,
    config: dict[str, Any] | None = None,
    integral_error_n_s: float = 0.0,
    traction_force_override_n: float | None = None,
) -> ForceControlResult:
    """计算下一小步。

    目标力与测量力只在已锁定的牵引方向上比较。三方向合力和侧向力
    仅用于超限检查，避免侧向受力被误当作有效牵引力。
    """

    cfg = config or load_force_config()
    current = _vector(wrench)
    zero = _vector(baseline)
    force = tuple(current[index] - zero[index] for index in range(3))
    total_force = _length(force)

    if direction is None:
        threshold = float(cfg["direction_capture_threshold_n"])
        locked_direction = (
            _normalise(force) if total_force >= threshold else None
        )
    else:
        locked_direction = _normalise(_vector(direction))

    raw_traction_force = (
        max(0.0, _dot(force, locked_direction))
        if locked_direction is not None
        else total_force
    )
    traction_force = (
        max(0.0, float(traction_force_override_n))
        if traction_force_override_n is not None and locked_direction is not None
        else raw_traction_force
    )
    lateral_force = (
        math.sqrt(max(0.0, total_force**2 - raw_traction_force**2))
        if locked_direction is not None
        else 0.0
    )
    error = float(target_force_n) - traction_force
    movement = (0.0, 0.0, 0.0)
    speed = 0.0

    if total_force >= float(cfg["max_force_n"]):
        state = "超过安全上限"
    elif lateral_force >= float(cfg.get("max_lateral_force_n", 20.0)):
        state = "侧向力过大"
    elif locked_direction is None:
        state = "等待轻拉以确定方向"
    elif abs(error) <= float(cfg["deadband_n"]):
        state = "接近目标"
    else:
        kp = float(cfg.get("kp_mm_s_per_n", cfg["gain_mm_s_per_n"]))
        ki = float(cfg.get("ki_mm_s_per_n_s", 0.0))
        requested_speed = kp * error + ki * integral_error_n_s
        max_speed = float(cfg["max_speed_mm_s"])
        signed_speed = max(-max_speed, min(max_speed, requested_speed))
        distance = min(
            float(cfg["max_step_mm"]),
            abs(signed_speed) * max(0.0, dt_s),
        )
        increase_sign = float(cfg["increase_force_motion_sign"])
        move_sign = increase_sign if signed_speed > 0 else -increase_sign
        movement = tuple(
            value * move_sign * distance for value in locked_direction
        )
        speed = abs(signed_speed)
        state = "增加牵引力" if error > 0 else "减小牵引力"

    return ForceControlResult(
        force_vector_n=force,
        force_magnitude_n=total_force,
        traction_force_n=traction_force,
        lateral_force_n=lateral_force,
        direction=locked_direction,
        error_n=error,
        movement_mm=movement,
        speed_mm_s=speed,
        state=state,
    )


class ConstantForceController:
    """保存一次牵引过程的方向、零点和实时调节状态。"""

    def __init__(self, config: dict[str, Any] | None = None) -> None:
        self.config = config or load_force_config()
        sample_count = int(self.config.get("baseline_sample_count", 8))
        self._recent_wrenches: deque[tuple[float, ...]] = deque(
            maxlen=max(3, sample_count)
        )
        self._lock = threading.Lock()
        self._active = False
        self._target_force_n = float(
            self.config.get("default_target_force_n", 10.0)
        )
        self._baseline: tuple[float, ...] | None = None
        self._direction: tuple[float, float, float] | None = None
        self._filtered_force_n = 0.0
        self._integral_error_n_s = 0.0
        self._last_update = time.monotonic()
        self._result: ForceControlResult | None = None
        self._message = "设备就绪"

    @staticmethod
    def _extract_wrench(snapshot: dict[str, Any]) -> tuple[float, ...] | None:
        kwr = snapshot.get("kwr75d") or {}
        wrench = kwr.get("wrench")
        if (
            not kwr.get("valid")
            or int(kwr.get("age_ms", 999999999)) > 1500
            or not isinstance(wrench, list)
            or len(wrench) != 6
        ):
            return None
        return tuple(float(value) for value in wrench)

    def observe(self, snapshot: dict[str, Any]) -> None:
        wrench = self._extract_wrench(snapshot)
        with self._lock:
            if wrench is None:
                if self._active:
                    self._message = "力传感器数据不可用"
                    self._result = None
                return
            self._recent_wrenches.append(wrench)
            if not self._active or self._baseline is None:
                return

            now = time.monotonic()
            dt_s = min(0.5, max(0.01, now - self._last_update))
            self._last_update = now
            result = calculate_step(
                wrench,
                self._baseline,
                self._target_force_n,
                direction=self._direction,
                dt_s=dt_s,
                config=self.config,
                integral_error_n_s=self._integral_error_n_s,
            )
            if self._direction is None and result.direction is not None:
                self._direction = result.direction
                self._filtered_force_n = result.traction_force_n
                self._integral_error_n_s = 0.0

            if self._direction is not None:
                alpha = float(self.config.get("filter_alpha", 0.3))
                self._filtered_force_n += alpha * (
                    result.traction_force_n - self._filtered_force_n
                )
                filtered_error = self._target_force_n - self._filtered_force_n
                if abs(filtered_error) > float(self.config["deadband_n"]):
                    limit = float(self.config.get("integral_limit_n_s", 20.0))
                    self._integral_error_n_s = max(
                        -limit,
                        min(
                            limit,
                            self._integral_error_n_s + filtered_error * dt_s,
                        ),
                    )
                else:
                    self._integral_error_n_s *= 0.8
                result = calculate_step(
                    wrench,
                    self._baseline,
                    self._target_force_n,
                    direction=self._direction,
                    dt_s=dt_s,
                    config=self.config,
                    integral_error_n_s=self._integral_error_n_s,
                    traction_force_override_n=self._filtered_force_n,
                )

            self._result = result
            self._message = result.state

    def start(self, target_force_n: float) -> None:
        with self._lock:
            minimum = float(self.config.get("target_force_min_n", 0.0))
            maximum = min(
                float(self.config.get("target_force_max_n", 10.0)),
                float(self.config["max_force_n"]),
            )
            if not minimum <= target_force_n <= maximum:
                raise ValueError(f"目标牵引力必须在 {minimum:g}～{maximum:g} N")
            if len(self._recent_wrenches) < self._recent_wrenches.maxlen:
                raise ValueError("真实力数据样本不足，请保持夹爪不受外力后重试")
            force_samples = list(self._recent_wrenches)
            spans = [
                max(sample[axis] for sample in force_samples)
                - min(sample[axis] for sample in force_samples)
                for axis in range(3)
            ]
            if max(spans) > float(self.config.get("baseline_max_span_n", 2.0)):
                raise ValueError("当前受力不稳定，请松手后重试")
            self._baseline = tuple(
                sum(sample[axis] for sample in force_samples) / len(force_samples)
                for axis in range(6)
            )
            self._target_force_n = float(target_force_n)
            self._direction = None
            self._filtered_force_n = 0.0
            self._integral_error_n_s = 0.0
            self._result = None
            self._active = True
            self._last_update = time.monotonic()
            self._message = "请轻拉以确定牵引方向"

    def set_target(self, target_force_n: float) -> None:
        with self._lock:
            minimum = float(self.config.get("target_force_min_n", 0.0))
            maximum = min(
                float(self.config.get("target_force_max_n", 10.0)),
                float(self.config["max_force_n"]),
            )
            if not minimum <= target_force_n <= maximum:
                raise ValueError(f"目标牵引力必须在 {minimum:g}～{maximum:g} N")
            self._target_force_n = float(target_force_n)
            self._integral_error_n_s = 0.0

    def stop(self, message: str = "设备就绪") -> None:
        with self._lock:
            self._active = False
            self._direction = None
            self._baseline = None
            self._integral_error_n_s = 0.0
            self._result = None
            self._message = message

    def fail(self, message: str) -> None:
        with self._lock:
            if self._active:
                self._message = message
                self._result = None

    def status(self) -> dict[str, Any]:
        with self._lock:
            result = asdict(self._result) if self._result is not None else None
            measured = (
                self._filtered_force_n
                if self._direction is not None
                else (
                    self._result.traction_force_n
                    if self._result is not None
                    else 0.0
                )
            )
            return {
                "active": self._active,
                "target_force_n": self._target_force_n,
                "direction_locked": self._direction is not None,
                "direction": self._direction,
                "measured_force_n": measured,
                "message": self._message,
                "send_robot_motion": bool(
                    self.config.get("send_robot_motion", False)
                ),
                "result": result,
            }
