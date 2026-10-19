#!/usr/bin/env python3
"""
Fail-closed KWR75 wrench bridge for the AUBO traction workstation.

The KWR75 publishes a wrench expressed at its own sensor origin. This node
uses a verified static sensor-to-flange transform and the measured flange pose
to publish a wrench expressed in ``base_link`` and referenced about the
flange origin. It intentionally contains no robot, sensor, or controller SDK
calls.
"""

from __future__ import annotations

import math
import time
from collections.abc import Sequence
from dataclasses import dataclass
from typing import Any

try:  # Keep pure-math unit tests independent of a sourced ROS installation.
    import rclpy
    from fr_traction.msg import EndEffectorState
    from geometry_msgs.msg import WrenchStamped
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
except ImportError:  # pragma: no cover - exercised only outside ROS.
    rclpy = None
    EndEffectorState = Any
    WrenchStamped = Any
    Node = object


Vector3 = tuple[float, float, float]
Quaternion = tuple[float, float, float, float]  # x, y, z, w
Stamp = tuple[int, int]


@dataclass(frozen=True)
class TransformConfiguration:
    """
    Validated bridge configuration.

    ``p_flange_sensor_m`` is the vector from flange origin to sensor origin,
    expressed in the flange frame. Thus the sensor moment is shifted to the
    flange with ``p_flange_sensor_m x force_flange``.
    """

    q_flange_sensor: Quaternion
    p_flange_sensor_m: Vector3
    expected_sensor_frame: str = "kwr75_sensor"
    expected_pose_frame: str = "base_link"
    mounting_verified: bool = False
    pose_frame_verified: bool = False
    sensor_timeout_s: float = 0.10
    pose_timeout_s: float = 0.10
    timestamp_future_tolerance_s: float = 0.05
    quaternion_norm_tolerance: float = 0.05
    fixed_pose_tare_enabled: bool = False
    force_bias_sensor_n: Vector3 = (0.0, 0.0, 0.0)
    force_sensor_sign: Vector3 = (1.0, 1.0, 1.0)
    torque_bias_sensor_nm: Vector3 = (0.0, 0.0, 0.0)
    tare_q_base_flange: Quaternion = (0.0, 0.0, 0.0, 1.0)
    tare_pose_tolerance_rad: float = 0.03


@dataclass(frozen=True)
class TransformedWrench:
    """Validated base-frame wrench expressed about the flange origin."""

    stamp: Stamp
    force_base: Vector3
    torque_base_about_flange: Vector3


@dataclass(frozen=True)
class _SensorSample:
    stamp: Stamp
    force_sensor: Vector3
    torque_sensor: Vector3
    received_monotonic_s: float


@dataclass(frozen=True)
class _PoseSample:
    q_base_flange: Quaternion
    received_monotonic_s: float


def _finite_float(value: Any) -> float:
    result = float(value)
    if not math.isfinite(result):
        raise ValueError("value is not finite")
    return result


def _vector3(value: Sequence[Any]) -> Vector3:
    if len(value) != 3:
        raise ValueError("expected exactly three vector components")
    return (_finite_float(value[0]), _finite_float(value[1]), _finite_float(value[2]))


def _normalize_quaternion(value: Sequence[Any], norm_tolerance: float) -> Quaternion:
    if len(value) != 4:
        raise ValueError("expected exactly four quaternion components")
    if not math.isfinite(norm_tolerance) or norm_tolerance < 0.0:
        raise ValueError("quaternion norm tolerance must be finite and nonnegative")
    x, y, z, w = (_finite_float(component) for component in value)
    norm = math.sqrt(x * x + y * y + z * z + w * w)
    if norm <= 1e-12 or abs(norm - 1.0) > norm_tolerance:
        raise ValueError("quaternion is zero length or outside the configured norm tolerance")
    return (x / norm, y / norm, z / norm, w / norm)


def _cross(left: Vector3, right: Vector3) -> Vector3:
    return (
        left[1] * right[2] - left[2] * right[1],
        left[2] * right[0] - left[0] * right[2],
        left[0] * right[1] - left[1] * right[0],
    )


def _add(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] + right[0], left[1] + right[1], left[2] + right[2])


def _subtract(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] - right[0], left[1] - right[1], left[2] - right[2])


def _multiply_components(left: Vector3, right: Vector3) -> Vector3:
    return (left[0] * right[0], left[1] * right[1], left[2] * right[2])


def _force_sensor_sign(value: Any) -> Vector3:
    sign = _vector3(value)
    if any(component not in (-1.0, 1.0) for component in sign):
        raise ValueError("force sensor signs must each be -1 or 1")
    return sign


def _quaternion_angle_rad(left: Quaternion, right: Quaternion) -> float:
    """Return the shortest orientation distance, treating q and -q as equal."""
    dot = sum(first * second for first, second in zip(left, right, strict=True))
    return 2.0 * math.acos(min(1.0, max(0.0, abs(dot))))


def rotate_vector(quaternion: Quaternion, vector: Vector3) -> Vector3:
    """Rotate ``vector`` using a normalized active xyzw quaternion."""
    x, y, z, w = quaternion
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def transform_sensor_wrench(
    force_sensor: Vector3,
    torque_sensor: Vector3,
    q_flange_sensor: Quaternion,
    p_flange_sensor_m: Vector3,
    q_base_flange: Quaternion,
) -> tuple[Vector3, Vector3]:
    """Return force and flange-referenced torque, both expressed in base."""
    force_flange = rotate_vector(q_flange_sensor, force_sensor)
    torque_flange = _add(
        rotate_vector(q_flange_sensor, torque_sensor),
        _cross(p_flange_sensor_m, force_flange),
    )
    return (
        rotate_vector(q_base_flange, force_flange),
        rotate_vector(q_base_flange, torque_flange),
    )


def _stamp_seconds(stamp: Any) -> tuple[Stamp, float]:
    try:
        sec = int(stamp.sec)
        nanosec = int(stamp.nanosec)
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise ValueError("missing or invalid header stamp") from error
    if sec < 0 or nanosec < 0 or nanosec >= 1_000_000_000:
        raise ValueError("header stamp is outside the valid ROS time range")
    if sec == 0 and nanosec == 0:
        raise ValueError("zero header stamp is not accepted")
    return (sec, nanosec), sec + nanosec * 1e-9


def _message_vector(message: Any, field_name: str) -> Vector3:
    try:
        field = getattr(message, field_name)
        return _vector3((field.x, field.y, field.z))
    except (AttributeError, TypeError, ValueError, OverflowError) as error:
        raise ValueError(f"invalid {field_name} vector") from error


class WrenchTransformCore:
    """
    ROS-free validation, freshness, and transform state.

    Every callback supplies receipt monotonic time and current ROS time. Source
    stamps are rejected at receipt; output freshness uses monotonic receipt
    time, so changing wall/ROS time cannot extend a sample's lifetime.
    """

    def __init__(self, configuration: TransformConfiguration):
        """Initialize the transform cache with a fully validated configuration."""
        self._configuration = configuration
        self._validate_configuration()
        self._force_sensor_sign = _force_sensor_sign(configuration.force_sensor_sign)
        self._tare_q_base_flange = (
            _normalize_quaternion(
                configuration.tare_q_base_flange, configuration.quaternion_norm_tolerance
            )
            if configuration.fixed_pose_tare_enabled
            else None
        )
        self._sensor: _SensorSample | None = None
        self._pose: _PoseSample | None = None

    def _validate_configuration(self) -> None:
        config = self._configuration
        if not config.expected_sensor_frame or not config.expected_pose_frame:
            raise ValueError("expected input frame IDs must be nonempty")
        for timeout in (config.sensor_timeout_s, config.pose_timeout_s):
            if not math.isfinite(timeout) or timeout <= 0.0:
                raise ValueError("freshness timeouts must be finite and positive")
        if (
            not math.isfinite(config.timestamp_future_tolerance_s)
            or config.timestamp_future_tolerance_s < 0.0
        ):
            raise ValueError("timestamp future tolerance must be finite and nonnegative")
        _normalize_quaternion(config.q_flange_sensor, config.quaternion_norm_tolerance)
        _vector3(config.p_flange_sensor_m)
        _force_sensor_sign(config.force_sensor_sign)
        if config.fixed_pose_tare_enabled:
            _vector3(config.force_bias_sensor_n)
            _vector3(config.torque_bias_sensor_nm)
            _normalize_quaternion(config.tare_q_base_flange, config.quaternion_norm_tolerance)
            if (
                not math.isfinite(config.tare_pose_tolerance_rad)
                or config.tare_pose_tolerance_rad < 0.0
            ):
                raise ValueError("tare pose tolerance must be finite and nonnegative")

    def _validate_header(
        self, message: Any, expected_frame: str, source_timeout_s: float, ros_now_s: float
    ) -> Stamp:
        if not math.isfinite(ros_now_s):
            raise ValueError("current ROS time is not finite")
        try:
            header = message.header
            frame_id = header.frame_id
        except AttributeError as error:
            raise ValueError("message has no valid header") from error
        if frame_id != expected_frame:
            raise ValueError(f"unexpected frame_id {frame_id!r}; expected {expected_frame!r}")
        stamp, source_time_s = _stamp_seconds(header.stamp)
        age_s = ros_now_s - source_time_s
        if age_s > source_timeout_s:
            raise ValueError("source timestamp is stale")
        if age_s < -self._configuration.timestamp_future_tolerance_s:
            raise ValueError("source timestamp is too far in the future")
        return stamp

    def accept_sensor(
        self, message: Any, receipt_monotonic_s: float, ros_now_s: float
    ) -> str | None:
        """Store a valid sensor sample, returning a rejection reason if invalid."""
        try:
            received = _finite_float(receipt_monotonic_s)
            stamp = self._validate_header(
                message,
                self._configuration.expected_sensor_frame,
                self._configuration.sensor_timeout_s,
                ros_now_s,
            )
            force = _message_vector(message.wrench, "force")
            torque = _message_vector(message.wrench, "torque")
            self._sensor = _SensorSample(stamp, force, torque, received)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            self._sensor = None
            return str(error)
        return None

    def accept_pose(
        self, message: Any, receipt_monotonic_s: float, ros_now_s: float
    ) -> str | None:
        """Store a valid EE pose sample, returning a rejection reason if invalid."""
        try:
            received = _finite_float(receipt_monotonic_s)
            self._validate_header(
                message,
                self._configuration.expected_pose_frame,
                self._configuration.pose_timeout_s,
                ros_now_s,
            )
            pose = message.pose
            _message_vector(pose, "position")
            orientation = pose.orientation
            q_base_flange = _normalize_quaternion(
                (orientation.x, orientation.y, orientation.z, orientation.w),
                self._configuration.quaternion_norm_tolerance,
            )
            self._pose = _PoseSample(q_base_flange, received)
        except (AttributeError, TypeError, ValueError, OverflowError) as error:
            self._pose = None
            return str(error)
        return None

    def output_if_fresh(self, now_monotonic_s: float) -> TransformedWrench | None:
        """Return a result only if calibration gate and both inputs are fresh."""
        config = self._configuration
        try:
            now = _finite_float(now_monotonic_s)
        except ValueError:
            return None
        if not config.mounting_verified or not config.pose_frame_verified:
            return None
        if self._sensor is None or self._pose is None:
            return None
        sensor_age = now - self._sensor.received_monotonic_s
        pose_age = now - self._pose.received_monotonic_s
        if sensor_age < 0.0 or pose_age < 0.0:
            return None
        if sensor_age > config.sensor_timeout_s or pose_age > config.pose_timeout_s:
            return None
        if (
            config.fixed_pose_tare_enabled
            and self._tare_q_base_flange is not None
            and _quaternion_angle_rad(self._pose.q_base_flange, self._tare_q_base_flange)
            > config.tare_pose_tolerance_rad
        ):
            return None
        force_sensor = self._sensor.force_sensor
        torque_sensor = self._sensor.torque_sensor
        if config.fixed_pose_tare_enabled:
            force_sensor = _subtract(force_sensor, _vector3(config.force_bias_sensor_n))
            torque_sensor = _subtract(torque_sensor, _vector3(config.torque_bias_sensor_nm))
        force_sensor = _multiply_components(force_sensor, self._force_sensor_sign)
        force_base, torque_base = transform_sensor_wrench(
            force_sensor,
            torque_sensor,
            _normalize_quaternion(config.q_flange_sensor, config.quaternion_norm_tolerance),
            _vector3(config.p_flange_sensor_m),
            self._pose.q_base_flange,
        )
        if not all(math.isfinite(component) for component in force_base + torque_base):
            return None
        return TransformedWrench(self._sensor.stamp, force_base, torque_base)


class AuboWrenchTransformNode(Node):
    """ROS wrapper around :class:`WrenchTransformCore`."""

    def __init__(self) -> None:
        """Declare the bridge parameters and create ROS subscriptions/publisher."""
        if rclpy is None:
            raise RuntimeError("rclpy and generated fr_traction messages are required")
        super().__init__("aubo_wrench_transform")
        q_flange_sensor = tuple(
            self.declare_parameter("q_flange_sensor", [0.0, 0.0, 0.0, 1.0]).value
        )
        p_flange_sensor_m = tuple(
            self.declare_parameter("p_flange_sensor_m", [0.0, 0.0, 0.0]).value
        )
        configuration = TransformConfiguration(
            q_flange_sensor=q_flange_sensor,  # type: ignore[arg-type]
            p_flange_sensor_m=p_flange_sensor_m,  # type: ignore[arg-type]
            expected_sensor_frame=str(
                self.declare_parameter("expected_sensor_frame", "kwr75_sensor").value
            ),
            expected_pose_frame=str(
                self.declare_parameter("expected_pose_frame", "base_link").value
            ),
            mounting_verified=bool(
                self.declare_parameter("mounting_verified", False).value
            ),
            pose_frame_verified=bool(
                self.declare_parameter("pose_frame_verified", False).value
            ),
            sensor_timeout_s=float(self.declare_parameter("sensor_timeout_s", 0.10).value),
            pose_timeout_s=float(self.declare_parameter("pose_timeout_s", 0.10).value),
            timestamp_future_tolerance_s=float(
                self.declare_parameter("timestamp_future_tolerance_s", 0.05).value
            ),
            quaternion_norm_tolerance=float(
                self.declare_parameter("quaternion_norm_tolerance", 0.05).value
            ),
            fixed_pose_tare_enabled=bool(
                self.declare_parameter("fixed_pose_tare_enabled", False).value
            ),
            force_bias_sensor_n=tuple(
                self.declare_parameter("force_bias_sensor_n", [0.0, 0.0, 0.0]).value
            ),  # type: ignore[arg-type]
            force_sensor_sign=tuple(
                self.declare_parameter("force_sensor_sign", [1.0, 1.0, 1.0]).value
            ),  # type: ignore[arg-type]
            torque_bias_sensor_nm=tuple(
                self.declare_parameter("torque_bias_sensor_nm", [0.0, 0.0, 0.0]).value
            ),  # type: ignore[arg-type]
            tare_q_base_flange=tuple(
                self.declare_parameter("tare_q_base_flange", [0.0, 0.0, 0.0, 1.0]).value
            ),  # type: ignore[arg-type]
            tare_pose_tolerance_rad=float(
                self.declare_parameter("tare_pose_tolerance_rad", 0.03).value
            ),
        )
        self._core = WrenchTransformCore(configuration)
        self._last_rejection = ""
        self._last_published_stamp: Stamp | None = None
        raw_wrench_topic = str(
            self.declare_parameter("raw_wrench_topic", "/aubo/kwr75/raw_wrench").value
        )
        ee_state_topic = str(
            self.declare_parameter("ee_state_topic", "/controller_manager/ee_state").value
        )
        output_wrench_topic = str(
            self.declare_parameter("output_wrench_topic", "/controller_manager/wrench").value
        )
        self._publisher = self.create_publisher(WrenchStamped, output_wrench_topic, 10)
        self.create_subscription(
            WrenchStamped, raw_wrench_topic, self._on_sensor, qos_profile_sensor_data
        )
        self.create_subscription(EndEffectorState, ee_state_topic, self._on_pose, 10)
        self.create_timer(0.01, self._publish_if_fresh)

    def _ros_now_seconds(self) -> float:
        return self.get_clock().now().nanoseconds * 1e-9

    def _record_rejection(self, source: str, reason: str | None) -> None:
        if reason is not None and reason != self._last_rejection:
            self.get_logger().warning(f"Rejected {source} wrench bridge input: {reason}")
            self._last_rejection = reason
        elif reason is None:
            self._last_rejection = ""

    def _on_sensor(self, message: WrenchStamped) -> None:
        reason = self._core.accept_sensor(message, time.monotonic(), self._ros_now_seconds())
        self._record_rejection("sensor", reason)

    def _on_pose(self, message: EndEffectorState) -> None:
        reason = self._core.accept_pose(message, time.monotonic(), self._ros_now_seconds())
        self._record_rejection("pose", reason)

    def _publish_if_fresh(self) -> None:
        result = self._core.output_if_fresh(time.monotonic())
        if result is None or result.stamp == self._last_published_stamp:
            return
        output = WrenchStamped()
        output.header.stamp.sec = result.stamp[0]
        output.header.stamp.nanosec = result.stamp[1]
        output.header.frame_id = "base_link"
        output.wrench.force.x, output.wrench.force.y, output.wrench.force.z = result.force_base
        (
            output.wrench.torque.x,
            output.wrench.torque.y,
            output.wrench.torque.z,
        ) = result.torque_base_about_flange
        self._publisher.publish(output)
        self._last_published_stamp = result.stamp


def main(args: Sequence[str] | None = None) -> None:
    """Run the ROS wrench transform bridge until shutdown."""
    if rclpy is None:
        raise RuntimeError("rclpy and generated fr_traction messages are required")
    rclpy.init(args=args)
    node = AuboWrenchTransformNode()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
