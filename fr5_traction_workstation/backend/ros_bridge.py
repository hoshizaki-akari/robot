"""Single-owner ROS2 bridge for the FR5 traction workstation.

The web process never connects to the Fairino SDK.  It only subscribes to
ROS2 feedback and calls the traction manager services.
"""

from __future__ import annotations

import copy
import threading
import time
from typing import Any


class RosBridgeError(RuntimeError):
    """Raised when ROS2 is unavailable or rejects a request."""


STATE_NAMES = {
    0: "INITIALIZING",
    1: "READY",
    2: "MANUAL_SETUP",
    3: "PRETENSION",
    4: "CALIBRATING",
    5: "DIRECTION_LOCKED",
    6: "TRACTION",
    7: "RELEASING",
    8: "COMPLETED",
    9: "FAULT",
    10: "EMERGENCY_STOP",
}


class RosBridge:
    """Thread-safe snapshots and service calls backed by one rclpy node."""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._node: Any = None
        self._executor: Any = None
        self._thread: threading.Thread | None = None
        self._clients: dict[str, Any] = {}
        self._parameter_clients: dict[str, Any] = {}
        self._heartbeat_publisher: Any = None
        self._started = False
        self._last_joint_at = 0.0
        self._last_traction_at = 0.0
        self._sequence = 0
        self._fr5: dict[str, Any] = {
            "valid": False,
            "frame_id": "base_link",
            "joint_names": [],
            "joint_position_rad": [],
            "joint_position_deg": [],
            "joint_velocity_rad_s": [],
            "joint_velocity_deg_s": [],
        }
        self._traction: dict[str, Any] = {
            "valid": False,
            "state": 0,
            "state_name": "INITIALIZING",
            "ready": False,
            "message": "等待 ROS2 牵引管理器状态",
        }
        self._history: dict[str, Any] = {"valid": False, "summaries": []}

    def start(self) -> None:
        try:
            import rclpy
            from fr_traction.msg import TractionHistory, TractionStatus
            from fr_traction.srv import SetTargetForce
            from rcl_interfaces.srv import GetParameters, SetParametersAtomically
            from sensor_msgs.msg import JointState
            from std_msgs.msg import Empty
            from std_srvs.srv import Trigger
        except ImportError:
            return

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("fr5_traction_workstation_bridge")
        qos = 10
        history_qos = rclpy.qos.QoSProfile(
            depth=1,
            reliability=rclpy.qos.ReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._node.create_subscription(
            JointState, "/joint_states", self._on_joint_state, qos
        )
        self._node.create_subscription(
            TractionStatus, "/traction/status", self._on_traction, qos
        )
        self._node.create_subscription(
            TractionHistory, "/traction/history", self._on_history, history_qos
        )
        self._heartbeat_publisher = self._node.create_publisher(
            Empty, "/traction/ui_heartbeat", qos
        )
        self._clients = {
            "prepare": self._node.create_client(Trigger, "/traction/prepare"),
            "calibrate_direction": self._node.create_client(
                Trigger, "/traction/calibrate_direction"
            ),
            "start": self._node.create_client(Trigger, "/traction/start"),
            "stop": self._node.create_client(Trigger, "/traction/stop"),
            "emergency_stop": self._node.create_client(
                Trigger, "/traction/emergency_stop"
            ),
            "reset_fault": self._node.create_client(Trigger, "/traction/reset_fault"),
            "set_zero_pose": self._node.create_client(
                Trigger, "/traction/set_zero_pose"
            ),
            "return_zero_pose": self._node.create_client(
                Trigger, "/traction/return_zero_pose"
            ),
            "set_target_force": self._node.create_client(
                SetTargetForce, "/traction/set_target_force"
            ),
        }
        self._parameter_clients = {
            "get_manager": self._node.create_client(
                GetParameters, "/traction_manager/get_parameters"
            ),
            "set_manager": self._node.create_client(
                SetParametersAtomically,
                "/traction_manager/set_parameters_atomically",
            ),
            "set_driver": self._node.create_client(
                SetParametersAtomically,
                "/fr5_direct_driver/set_parameters_atomically",
            ),
        }
        from rclpy.executors import MultiThreadedExecutor

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        self._started = True

    def stop(self) -> None:
        self._started = False
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        if self._node is not None:
            self._node.destroy_node()
        self._executor = None
        self._node = None
        self._parameter_clients = {}

    @staticmethod
    def _stamp_seconds(message: Any) -> float:
        stamp = getattr(getattr(message, "header", None), "stamp", None)
        if stamp is None:
            return 0.0
        return float(stamp.sec) + float(stamp.nanosec) * 1e-9

    def _on_joint_state(self, message: Any) -> None:
        positions_by_name = {
            _joint_index(name): float(value)
            for name, value in zip(message.name, message.position)
            if _joint_index(name) is not None
        }
        velocities_by_name = {
            _joint_index(name): float(value)
            for name, value in zip(message.name, message.velocity)
            if _joint_index(name) is not None
        }
        if set(positions_by_name) != set(range(6)) or any(
            not _finite(positions_by_name[index]) for index in range(6)
        ):
            return
        if set(velocities_by_name) != set(range(6)) or any(
            not _finite(velocities_by_name[index]) for index in range(6)
        ):
            return
        positions = [positions_by_name[index] for index in range(6)]
        velocities = [velocities_by_name[index] for index in range(6)]
        import math

        with self._lock:
            self._fr5 = {
                "valid": True,
                "frame_id": "base_link",
                "joint_names": [f"j{index + 1}" for index in range(6)],
                "joint_position_rad": positions,
                "joint_position_deg": [math.degrees(value) for value in positions],
                "joint_velocity_rad_s": velocities,
                "joint_velocity_deg_s": [math.degrees(value) for value in velocities],
            }
            self._last_joint_at = time.monotonic()
            self._sequence += 1

    def _on_traction(self, message: Any) -> None:
        direction = message.locked_direction_base
        ee = message.ee_position_base
        force_vector = [float(message.fx), float(message.fy), float(message.fz)]
        reported_force_direction = getattr(message, "measured_force_direction_base", None)
        force_direction = (
            [
                float(reported_force_direction.x),
                float(reported_force_direction.y),
                float(reported_force_direction.z),
            ]
            if bool(getattr(message, "force_direction_valid", False))
            and reported_force_direction is not None
            else _normalized(force_vector, minimum_norm=1.0)
        )
        locked_direction = [float(direction.x), float(direction.y), float(direction.z)]
        reported_increase_direction = getattr(message, "increase_direction_base", None)
        increase_direction = locked_direction if _vector_norm(locked_direction) >= 0.9 else None
        if reported_increase_direction is not None:
            candidate = [
                float(reported_increase_direction.x),
                float(reported_increase_direction.y),
                float(reported_increase_direction.z),
            ]
            if _vector_norm(candidate) >= 0.9:
                increase_direction = candidate
        if increase_direction is None and force_direction:
            increase_direction = force_direction
        reported_lateral_velocity = getattr(message, "lateral_correction_velocity_base", None)
        lateral_velocity = (
            [
                float(reported_lateral_velocity.x),
                float(reported_lateral_velocity.y),
                float(reported_lateral_velocity.z),
            ]
            if reported_lateral_velocity is not None
            else [0.0, 0.0, 0.0]
        )
        with self._lock:
            self._traction = {
                "valid": True,
                "state": int(message.state),
                "state_name": STATE_NAMES.get(int(message.state), "UNKNOWN"),
                "ready": bool(message.ready),
                "target_force_n": float(message.target_force_n),
                "actual_force_n": float(message.actual_force_n),
                "lateral_force_n": float(message.lateral_force_n),
                "force_vector_n": force_vector,
                "force_direction_base": force_direction,
                "increase_direction_base": increase_direction,
                "locked_direction_base": locked_direction,
                "ee_position_base_m": [float(ee.x), float(ee.y), float(ee.z)],
                "axis_displacement_m": float(message.axis_displacement_m),
                "velocity_cmd_mps": float(message.velocity_cmd_mps),
                "direction_track_state": int(getattr(message, "direction_track_state", 4)),
                "direction_correction_active": bool(
                    getattr(message, "direction_correction_active", False)
                ),
                "direction_error_rad": float(getattr(message, "direction_error_rad", 0.0)),
                "direction_fast_slow_error_rad": float(
                    getattr(message, "direction_fast_slow_error_rad", 0.0)
                ),
                "direction_entry_threshold_rad": float(
                    getattr(message, "direction_entry_threshold_rad", 0.0)
                ),
                "direction_correction_velocity_mps": float(
                    getattr(message, "direction_correction_velocity_mps", 0.0)
                ),
                "direction_correction_displacement_m": float(
                    getattr(message, "direction_correction_displacement_m", 0.0)
                ),
                "lateral_correction_velocity_base": lateral_velocity,
                "fault_code": str(message.fault_code),
                "stop_reason": str(message.stop_reason),
                "message": str(getattr(message, "message", "")),
            }
            self._last_traction_at = time.monotonic()
            self._sequence += 1

    @staticmethod
    def _summary_to_dict(summary: Any) -> dict[str, Any]:
        return {
            "session_id": summary.session_id,
            "start_time": {
                "sec": summary.start_time.sec,
                "nanosec": summary.start_time.nanosec,
            },
            "end_time": {
                "sec": summary.end_time.sec,
                "nanosec": summary.end_time.nanosec,
            },
            "target_force_n": float(summary.target_force_n),
            "average_force_n": float(summary.average_force_n),
            "max_force_n": float(summary.max_force_n),
            "final_state": int(summary.final_state),
            "stop_reason": summary.stop_reason,
            "record_path": summary.record_path,
        }

    def _on_history(self, message: Any) -> None:
        with self._lock:
            self._history = {
                "valid": True,
                "summaries": [self._summary_to_dict(item) for item in message.summaries],
            }

    def snapshot(self) -> dict[str, Any]:
        now = time.monotonic()
        with self._lock:
            fr5 = copy.deepcopy(self._fr5)
            traction = copy.deepcopy(self._traction)
            history = copy.deepcopy(self._history)
            joint_age_ms = (
                None if self._last_joint_at == 0 else int((now - self._last_joint_at) * 1000)
            )
            traction_age_ms = (
                None
                if self._last_traction_at == 0
                else int((now - self._last_traction_at) * 1000)
            )
            sequence = self._sequence
        fr5["data_age_ms"] = joint_age_ms
        traction["data_age_ms"] = traction_age_ms
        fr5["connected"] = bool(fr5.get("valid") and joint_age_ms is not None and joint_age_ms <= 500)
        traction["connected"] = bool(
            traction.get("valid") and traction_age_ms is not None and traction_age_ms <= 500
        )
        return {
            "schema_version": "2.0",
            "timestamp_wall": time.time(),
            "sequence": sequence,
            "connected": bool(fr5["connected"] and traction["connected"]),
            "frame_id": "base_link",
            "fr5": fr5,
            "traction": traction,
            "history": history,
        }

    def heartbeat(self) -> None:
        if self._heartbeat_publisher is not None:
            from std_msgs.msg import Empty

            self._heartbeat_publisher.publish(Empty())

    def call(self, name: str, target_force_n: float | None = None) -> dict[str, Any]:
        if not self._started or self._node is None:
            raise RosBridgeError("ROS2牵引系统不可用，请先启动FR5牵引系统")
        client = self._clients.get(name)
        if client is None:
            raise RosBridgeError(f"未知牵引操作：{name}")
        if not client.wait_for_service(timeout_sec=0.8):
            raise RosBridgeError(f"牵引服务不可用：/traction/{name}")
        request = client.srv_type.Request()
        if target_force_n is not None:
            request.target_force_n = float(target_force_n)
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=2.0):
            raise RosBridgeError(f"牵引服务超时：/traction/{name}")
        try:
            response = future.result()
        except Exception as error:
            raise RosBridgeError(f"牵引服务调用失败：{error}") from error
        if not response.success:
            raise RosBridgeError(response.message)
        return {"success": True, "message": response.message, "snapshot": self.snapshot()}

    @staticmethod
    def _wait_for_future(future: Any, timeout_s: float, operation: str) -> Any:
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=timeout_s):
            raise RosBridgeError(f"{operation}超时")
        try:
            return future.result()
        except Exception as error:
            raise RosBridgeError(f"{operation}失败：{error}") from error

    def get_motion_settings(self) -> dict[str, Any]:
        if not self._started or self._node is None:
            raise RosBridgeError("ROS2牵引系统不可用，请先启动FR5牵引系统")
        client = self._parameter_clients.get("get_manager")
        if client is None or not client.wait_for_service(timeout_sec=0.8):
            raise RosBridgeError("牵引参数服务不可用")
        request = client.srv_type.Request()
        request.names = ["axial_travel_limit_m"]
        response = self._wait_for_future(
            client.call_async(request), 2.0, "读取牵引参数"
        )
        if not response.values:
            raise RosBridgeError("未读取到最大行程参数")
        return {
            "success": True,
            "max_travel_mm": float(response.values[0].double_value) * 1000.0,
        }

    def set_max_travel_mm(self, max_travel_mm: float) -> dict[str, Any]:
        if not self._started or self._node is None:
            raise RosBridgeError("ROS2牵引系统不可用，请先启动FR5牵引系统")
        from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue

        manager_client = self._parameter_clients.get("set_manager")
        driver_client = self._parameter_clients.get("set_driver")
        if manager_client is None or not manager_client.wait_for_service(timeout_sec=0.8):
            raise RosBridgeError("牵引参数服务不可用")
        if driver_client is None or not driver_client.wait_for_service(timeout_sec=0.8):
            raise RosBridgeError("机械臂返回参数服务不可用")
        driver_request = driver_client.srv_type.Request()
        driver_request.parameters = [
            Parameter(
                name="return_max_distance_mm",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(max_travel_mm) + 20.0,
                ),
            )
        ]
        driver_response = self._wait_for_future(
            driver_client.call_async(driver_request), 2.0, "保存返回行程"
        )
        if not driver_response.result.successful:
            reason = driver_response.result.reason or "返回行程设置失败"
            raise RosBridgeError(reason)
        manager_request = manager_client.srv_type.Request()
        manager_request.parameters = [
            Parameter(
                name="axial_travel_limit_m",
                value=ParameterValue(
                    type=ParameterType.PARAMETER_DOUBLE,
                    double_value=float(max_travel_mm) / 1000.0,
                ),
            )
        ]
        response = self._wait_for_future(
            manager_client.call_async(manager_request), 2.0, "保存最大行程"
        )
        if not response.result.successful:
            reason = response.result.reason or "当前状态不能修改最大行程"
            raise RosBridgeError(reason)
        return {
            "success": True,
            "message": "最大行程已保存",
            "max_travel_mm": float(max_travel_mm),
            "snapshot": self.snapshot(),
        }


def _finite(value: float) -> bool:
    import math

    return math.isfinite(value)


def _vector_norm(vector: list[float]) -> float:
    return sum(value * value for value in vector) ** 0.5


def _normalized(vector: list[float], minimum_norm: float) -> list[float] | None:
    norm = _vector_norm(vector)
    if norm < minimum_norm or not _finite(norm):
        return None
    return [value / norm for value in vector]


def _joint_index(name: str) -> int | None:
    normalized = str(name).strip().lower().replace("-", "_")
    candidates = ("j1", "j2", "j3", "j4", "j5", "j6")
    if normalized in candidates:
        return candidates.index(normalized)
    if normalized.startswith("joint_") and normalized[6:].isdigit():
        index = int(normalized[6:]) - 1
        return index if 0 <= index < 6 else None
    if normalized.startswith("joint") and normalized[5:].isdigit():
        index = int(normalized[5:]) - 1
        return index if 0 <= index < 6 else None
    return None
