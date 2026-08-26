"""Small rclpy bridge used by the Platform B page.

The web server never computes force or sends robot motion directly.  It only
subscribes to the ROS2 traction manager's status/history and forwards explicit
service requests from the authenticated operator.
"""

from __future__ import annotations

import threading
from typing import Any


class RosTractionError(RuntimeError):
    """Raised when the ROS2 traction manager cannot be reached or rejects an action."""


class RosTractionBridge:
    """Own one rclpy node and expose thread-safe snapshots to FastAPI."""

    _STATE_NAMES = {
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

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._node: Any = None
        self._executor: Any = None
        self._thread: threading.Thread | None = None
        self._clients: dict[str, Any] = {}
        self._available = False
        self._status: dict[str, Any] | None = None
        self._history: dict[str, Any] = {"valid": False, "summaries": []}

    @property
    def available(self) -> bool:
        with self._lock:
            return self._available

    def start(self) -> None:
        try:
            import rclpy
            from fr_traction.msg import TractionHistory, TractionStatus
            from fr_traction.srv import SetTargetForce
            from std_msgs.msg import Empty
            from std_srvs.srv import Trigger
        except ImportError:
            return

        if not rclpy.ok():
            rclpy.init(args=None)
        self._node = rclpy.create_node("platform_b_traction_bridge")
        qos = 10
        from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy

        history_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._node.create_subscription(
            TractionStatus, "/traction/status", self._on_status, qos
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
            "set_target_force": self._node.create_client(
                SetTargetForce, "/traction/set_target_force"
            ),
        }
        from rclpy.executors import MultiThreadedExecutor

        self._executor = MultiThreadedExecutor(num_threads=2)
        self._executor.add_node(self._node)
        self._thread = threading.Thread(target=self._executor.spin, daemon=True)
        self._thread.start()
        with self._lock:
            self._available = True

    def stop(self) -> None:
        with self._lock:
            self._available = False
        if self._executor is not None:
            self._executor.shutdown(timeout_sec=1.0)
        if self._node is not None:
            self._node.destroy_node()
        self._executor = None
        self._node = None

    @staticmethod
    def _summary_to_dict(summary: Any) -> dict[str, Any]:
        return {
            "session_id": summary.session_id,
            "start_time": {"sec": summary.start_time.sec, "nanosec": summary.start_time.nanosec},
            "end_time": {"sec": summary.end_time.sec, "nanosec": summary.end_time.nanosec},
            "target_force_n": summary.target_force_n,
            "average_force_n": summary.average_force_n,
            "max_force_n": summary.max_force_n,
            "final_state": summary.final_state,
            "stop_reason": summary.stop_reason,
            "record_path": summary.record_path,
        }

    def _on_status(self, message: Any) -> None:
        state = int(message.state)
        traction = {
            "valid": True,
            "state": state,
            "state_name": self._STATE_NAMES.get(state, f"UNKNOWN_{state}"),
            "ready": bool(message.ready),
            "target_force_n": float(message.target_force_n),
            "actual_force_n": float(message.actual_force_n),
            "lateral_force_n": float(message.lateral_force_n),
            "force_vector_n": [float(message.fx), float(message.fy), float(message.fz)],
            "direction": [
                float(message.locked_direction_base.x),
                float(message.locked_direction_base.y),
                float(message.locked_direction_base.z),
            ],
            "ee_position_base_m": [
                float(message.ee_position_base.x),
                float(message.ee_position_base.y),
                float(message.ee_position_base.z),
            ],
            "axis_displacement_m": float(message.axis_displacement_m),
            "velocity_cmd_mps": float(message.velocity_cmd_mps),
            "fault_code": message.fault_code,
            "stop_reason": message.stop_reason,
        }
        with self._lock:
            self._status = traction

    def _on_history(self, message: Any) -> None:
        with self._lock:
            self._history = {
                "valid": True,
                "summaries": [self._summary_to_dict(item) for item in message.summaries],
            }

    def status(self) -> dict[str, Any]:
        with self._lock:
            traction = dict(self._status) if self._status is not None else {
                "valid": False,
                "state": 0,
                "state_name": "INITIALIZING",
                "ready": False,
                "message": "等待 ROS2 牵引管理器状态",
            }
        return {"source": "ros2_traction", "valid": bool(traction.get("valid")), "traction": traction}

    def history(self) -> dict[str, Any]:
        with self._lock:
            return {"valid": self._history["valid"], "summaries": list(self._history["summaries"])}

    def heartbeat(self) -> None:
        if self._node is not None:
            from std_msgs.msg import Empty

            self._heartbeat_publisher.publish(Empty())

    def call(self, name: str, target_force_n: float | None = None) -> dict[str, Any]:
        if not self.available or self._node is None:
            raise RosTractionError("ROS2 牵引管理器不可用；请先启动 traction_system.launch.py")
        client = self._clients.get(name)
        if client is None:
            raise RosTractionError(f"未知牵引操作：{name}")
        if not client.wait_for_service(timeout_sec=0.5):
            raise RosTractionError(f"牵引服务不可用：/traction/{name}")
        if target_force_n is None:
            request = client.srv_type.Request()
        else:
            request = client.srv_type.Request()
            request.target_force_n = float(target_force_n)
        future = client.call_async(request)
        event = threading.Event()
        future.add_done_callback(lambda _: event.set())
        if not event.wait(timeout=2.0):
            raise RosTractionError(f"牵引服务超时：/traction/{name}")
        try:
            response = future.result()
        except Exception as error:
            raise RosTractionError(f"牵引服务调用失败：{error}") from error
        if not response.success:
            raise RosTractionError(response.message)
        return {"success": True, "message": response.message, **self.status()}
