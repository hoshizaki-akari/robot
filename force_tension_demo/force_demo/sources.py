"""Read-only real data sources for ROS 2, the shared service and Fairino SDK."""

from __future__ import annotations

import json
import math
import os
import threading
import time
from datetime import datetime, timezone
from typing import Any
from urllib.error import URLError
from urllib.request import urlopen

from .classifier import ForceTensionEngine, SensorSample, Vector3, finite_vector


def _wall_time() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def _result_values(result: object, length: int) -> list[float]:
    if not isinstance(result, (list, tuple)) or len(result) < 2:
        raise ValueError(f"SDK 返回格式异常：{result!r}")
    if int(result[0]) != 0:
        raise RuntimeError(f"SDK 错误码：{result[0]}")
    values = result[1]
    if not isinstance(values, (list, tuple)) or len(values) < length:
        raise ValueError(f"SDK 数据长度异常：{result!r}")
    return [float(value) for value in values[:length]]


def _quaternion_rotate(vector: Vector3, quaternion: tuple[float, float, float, float]) -> Vector3:
    x, y, z, w = quaternion
    q_length = math.sqrt(x * x + y * y + z * z + w * w)
    if q_length < 1e-9:
        raise ValueError("TF quaternion is zero")
    x, y, z, w = x / q_length, y / q_length, z / q_length, w / q_length
    vx, vy, vz = vector
    tx = 2.0 * (y * vz - z * vy)
    ty = 2.0 * (z * vx - x * vz)
    tz = 2.0 * (x * vy - y * vx)
    return (
        vx + w * tx + (y * tz - z * ty),
        vy + w * ty + (z * tx - x * tz),
        vz + w * tz + (x * ty - y * tx),
    )


def _quaternion_to_rpy_deg(quaternion: tuple[float, float, float, float]) -> Vector3:
    x, y, z, w = quaternion
    sinr_cosp = 2.0 * (w * x + y * z)
    cosr_cosp = 1.0 - 2.0 * (x * x + y * y)
    roll = math.atan2(sinr_cosp, cosr_cosp)
    sinp = max(-1.0, min(1.0, 2.0 * (w * y - z * x)))
    pitch = math.asin(sinp)
    siny_cosp = 2.0 * (w * z + x * y)
    cosy_cosp = 1.0 - 2.0 * (y * y + z * z)
    yaw = math.atan2(siny_cosp, cosy_cosp)
    return tuple(value * 180.0 / math.pi for value in (roll, pitch, yaw))  # type: ignore[return-value]


class Ros2Source:
    """Discover one WrenchStamped topic and convert vectors into base_link."""

    def __init__(self, engine: ForceTensionEngine, config: dict[str, object]) -> None:
        self.engine = engine
        self.config = config
        self._thread: threading.Thread | None = None
        self._executor: Any = None
        self._node: Any = None
        self._subscription: Any = None
        self._active_topic = ""
        self._joint_velocity_deg_s: list[float] = []
        self._joint_update_time = 0.0
        self._last_positions: list[float] | None = None
        self._last_position_time = 0.0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="ros2-force-source", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        try:
            import rclpy
            import tf2_ros
            from rclpy.executors import SingleThreadedExecutor
            from rclpy.qos import qos_profile_sensor_data
        except ImportError as error:
            self.engine.set_diagnostic("ros2", f"不可用：{error}；请通过 run_demo.sh 启动")
            return
        try:
            if not rclpy.ok():
                rclpy.init(args=None)
            self._node = rclpy.create_node("kwr75d_force_tension_demo")
            self._qos = qos_profile_sensor_data
            self._tf_buffer = tf2_ros.Buffer(cache_time=rclpy.duration.Duration(seconds=5.0))
            self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self._node, spin_thread=False)
            from sensor_msgs.msg import JointState

            joint_topic = str(self.config.get("joint_state_topic", "/joint_states"))
            self._node.create_subscription(JointState, joint_topic, self._on_joint_state, self._qos)
            self._node.create_timer(0.5, self._discover_wrench)
            self._executor = SingleThreadedExecutor()
            self._executor.add_node(self._node)
            self.engine.set_diagnostic("ros2", "已启动，正在发现 WrenchStamped 话题")
            self._executor.spin()
        except Exception as error:  # noqa: BLE001 - ROS boundary can raise plugin-specific errors.
            self.engine.set_diagnostic("ros2", f"启动失败：{type(error).__name__}: {error}")

    def _discover_wrench(self) -> None:
        if self._node is None:
            return
        names = {
            name: types
            for name, types in self._node.get_topic_names_and_types()
            if "geometry_msgs/msg/WrenchStamped" in types
        }
        preferred = [str(topic) for topic in self.config.get("wrench_topics", [])]
        discovered = [name for name in names if "wrench" in name.lower() or "force" in name.lower()]
        candidates = [name for name in preferred + sorted(discovered) if name in names]
        if not candidates:
            self.engine.set_diagnostic("ros2", "未发现 WrenchStamped 话题")
            return
        selected = candidates[0]
        if selected == self._active_topic:
            return
        if self._subscription is not None:
            self._node.destroy_subscription(self._subscription)
        from geometry_msgs.msg import WrenchStamped

        self._subscription = self._node.create_subscription(
            WrenchStamped, selected, self._on_wrench, self._qos
        )
        self._active_topic = selected
        self.engine.set_diagnostic("ros2", f"已订阅 {selected}")

    def _on_joint_state(self, message: Any) -> None:
        now = time.monotonic()
        velocity = [float(value) * 180.0 / math.pi for value in message.velocity]
        if not velocity and message.position:
            positions = [float(value) for value in message.position]
            if self._last_positions is not None and len(positions) == len(self._last_positions):
                dt = now - self._last_position_time
                if dt > 0.001:
                    velocity = [
                        (positions[index] - self._last_positions[index]) / dt * 180.0 / math.pi
                        for index in range(len(positions))
                    ]
            self._last_positions = positions
            self._last_position_time = now
        if velocity:
            self._joint_velocity_deg_s = velocity
            self._joint_update_time = now

    def _base_vector(self, vector: Vector3, frame_id: str) -> Vector3:
        clean_frame = frame_id.lstrip("/")
        if clean_frame == "base_link":
            return vector
        if not clean_frame:
            raise ValueError("WrenchStamped.frame_id 为空，不能确认 base_link")
        import rclpy

        transform = self._tf_buffer.lookup_transform(
            "base_link", clean_frame, rclpy.time.Time(),
            timeout=rclpy.duration.Duration(seconds=0.05),
        )
        rotation = transform.transform.rotation
        return _quaternion_rotate(vector, (rotation.x, rotation.y, rotation.z, rotation.w))

    def _on_wrench(self, message: Any) -> None:
        try:
            force = self._base_vector(
                finite_vector((message.wrench.force.x, message.wrench.force.y, message.wrench.force.z)),
                str(message.header.frame_id),
            )
            torque = self._base_vector(
                finite_vector((message.wrench.torque.x, message.wrench.torque.y, message.wrench.torque.z)),
                str(message.header.frame_id),
            )
            now = time.monotonic()
            motion_available = now - self._joint_update_time <= 0.3
            tool_orientation = self._tool_orientation()
            self.engine.ingest(SensorSample.create(
                force, torque_nm=torque, monotonic_time=now, frame_id="base_link",
                source="ros2", source_detail=self._active_topic, priority=30,
                motion_available=motion_available,
                max_joint_speed_deg_s=max((abs(value) for value in self._joint_velocity_deg_s), default=0.0),
                tcp_rpy_deg=tool_orientation,
            ))
        except Exception as error:  # noqa: BLE001 - reject any malformed ROS/TF payload.
            self.engine.set_error(f"ROS 2 Wrench 转换失败：{error}")

    def _tool_orientation(self) -> Vector3 | None:
        try:
            import rclpy

            tool_frame = str(self.config.get("tool_frame", "wrist3_link"))
            transform = self._tf_buffer.lookup_transform(
                "base_link", tool_frame, rclpy.time.Time(),
                timeout=rclpy.duration.Duration(seconds=0.02),
            )
            rotation = transform.transform.rotation
            return _quaternion_to_rpy_deg((rotation.x, rotation.y, rotation.z, rotation.w))
        except Exception:  # noqa: BLE001 - orientation is optional when TF is unavailable.
            return None

    def stop(self) -> None:
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        if self._node is not None:
            self._node.destroy_node()
        if self._thread is not None:
            self._thread.join(timeout=2.0)


class FallbackSource:
    """Use a valid shared state service, otherwise own a read-only SDK connection."""

    def __init__(self, engine: ForceTensionEngine, config: dict[str, object]) -> None:
        self.engine = engine
        self.config = config
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._robot: Any = None
        self._next_sdk_attempt = 0.0
        self._last_sdk_frame: int | None = None
        self._last_sdk_frame_change = 0.0
        self._next_state_service_poll = 0.0

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, name="fallback-force-source", daemon=True)
        self._thread.start()

    def _run(self) -> None:
        while not self._stop.is_set():
            if self.engine.source_is_fresh(minimum_priority=30):
                self._close_sdk()
                self._stop.wait(0.2)
                continue
            now = time.monotonic()
            service_checked = now >= self._next_state_service_poll
            service_valid = service_checked and self._poll_state_service()
            if service_valid:
                self._next_state_service_poll = now + 0.05
                self._close_sdk()
                self._stop.wait(0.05)
                continue
            if service_checked:
                self._next_state_service_poll = now + (1.0 if self._robot is not None else 0.2)
            if os.environ.get("FORCE_DEMO_DISABLE_DIRECT_SDK") != "1":
                self._poll_sdk()
            self._stop.wait(0.05)
        self._close_sdk()

    def _poll_state_service(self) -> bool:
        url = str(self.config.get("state_service_url", "http://127.0.0.1:8765/api/state"))
        try:
            with urlopen(url, timeout=0.3) as response:
                state = json.load(response)
            sensor, robot = state.get("kwr75d", {}), state.get("fr5", {})
            if state.get("source") != "real" or not sensor.get("valid") or not robot.get("valid"):
                message = sensor.get("message") or robot.get("message") or "不是有效真机数据"
                self.engine.set_diagnostic("state_service", f"不可用：{message}")
                return False
            values = [float(value) for value in sensor.get("wrench", [])]
            if len(values) < 6 or not all(math.isfinite(value) for value in values[:6]):
                raise ValueError("KWR75D wrench 数据格式异常")
            speeds = [float(value) for value in robot.get("joint_velocity_deg_s", [])]
            pose = [float(value) for value in robot.get("tcp_pose_mm_deg", [])]
            self.engine.ingest(SensorSample.create(
                values[:3], torque_nm=values[3:6], frame_id="base_link",
                source="state_service", source_detail="KWR75D FT_GetForceTorqueRCS", priority=20,
                motion_available=len(speeds) >= 6,
                max_joint_speed_deg_s=max((abs(value) for value in speeds), default=0.0),
                tcp_position_mm=tuple(pose[:3]) if len(pose) >= 3 else None,
                tcp_rpy_deg=tuple(pose[3:6]) if len(pose) >= 6 else None,
            ))
            self.engine.set_diagnostic("state_service", "已连接有效真机状态")
            return True
        except (OSError, ValueError, TypeError, KeyError, URLError, json.JSONDecodeError) as error:
            self.engine.set_diagnostic("state_service", f"连接失败：{type(error).__name__}: {error}")
            return False

    def _connect_sdk(self) -> bool:
        if self._robot is not None:
            return True
        now = time.monotonic()
        if now < self._next_sdk_attempt:
            return False
        self._next_sdk_attempt = now + 5.0
        try:
            from fairino import Robot

            robot_ip = str(self.config.get("robot_ip", "192.168.58.2"))
            for key in ("NO_PROXY", "no_proxy"):
                values = [value for value in os.environ.get(key, "").split(",") if value]
                if robot_ip not in values:
                    values.append(robot_ip)
                os.environ[key] = ",".join(values)
            self._robot = Robot.RPC(robot_ip)
            deadline = time.monotonic() + 3.0
            while not self._stop.is_set() and time.monotonic() < deadline:
                state = getattr(self._robot, "robot_state_pkg", None)
                if state is not None and not isinstance(state, type) and int(getattr(state, "frame_head", 0)) == 0x5A5A:
                    self._last_sdk_frame = int(state.frame_cnt) & 0xFF
                    self._last_sdk_frame_change = time.monotonic()
                    self.engine.set_diagnostic("fairino_sdk", f"已只读连接 {robot_ip}")
                    return True
                time.sleep(0.05)
            raise TimeoutError("实时状态帧未就绪；可能有另一个 SDK 程序占用状态接收端口")
        except Exception as error:  # noqa: BLE001 - proprietary SDK raises nonstandard errors.
            self.engine.set_diagnostic("fairino_sdk", f"连接失败：{type(error).__name__}: {error}")
            self._close_sdk()
            return False

    def _poll_sdk(self) -> bool:
        if not self._connect_sdk():
            return False
        try:
            state = self._robot.robot_state_pkg
            now = time.monotonic()
            frame_count = int(state.frame_cnt) & 0xFF
            if frame_count != self._last_sdk_frame:
                self._last_sdk_frame = frame_count
                self._last_sdk_frame_change = now
            elif now - self._last_sdk_frame_change > 0.6:
                raise TimeoutError("法奥实时状态帧超过 0.6 秒没有更新")
            if not int(state.ft_sensor_active):
                self.engine.set_diagnostic("fairino_sdk", "已连接，但控制器中的力传感器未启用")
                return False
            values = [float(state.ft_sensor_data[index]) for index in range(6)]
            speeds = [float(state.actual_qd[index]) for index in range(6)]
            pose = tuple(float(state.tl_cur_pos[index]) for index in range(3))
            orientation = tuple(float(state.tl_cur_pos[index]) for index in range(3, 6))
            self.engine.ingest(SensorSample.create(
                values[:3], torque_nm=values[3:6], frame_id="base_link",
                source="fairino_sdk", source_detail="KWR75D FT_GetForceTorqueRCS (read-only)", priority=10,
                motion_available=True,
                max_joint_speed_deg_s=max((abs(value) for value in speeds), default=0.0),
                tcp_position_mm=pose,
                tcp_rpy_deg=orientation,
            ))
            return True
        except Exception as error:  # noqa: BLE001 - proprietary SDK raises nonstandard errors.
            self.engine.set_diagnostic("fairino_sdk", f"读取失败：{type(error).__name__}: {error}")
            self._close_sdk()
            return False

    def _close_sdk(self) -> None:
        if self._robot is not None:
            try:
                if hasattr(self._robot, "CloseRPC"):
                    self._robot.CloseRPC()
            except Exception as error:  # noqa: BLE001 - shutdown must still finish.
                self.engine.set_diagnostic("fairino_sdk_close", f"关闭连接时提示：{error}")
        self._robot = None
        self._last_sdk_frame = None
        self._last_sdk_frame_change = 0.0

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=4.0)
        self._close_sdk()


class SourceManager:
    def __init__(self, engine: ForceTensionEngine, config: dict[str, object]) -> None:
        self.ros = Ros2Source(engine, config)
        self.fallback = FallbackSource(engine, config)

    def start(self) -> None:
        self.ros.start()
        self.fallback.start()

    def stop(self) -> None:
        self.fallback.stop()
        self.ros.stop()
