#!/usr/bin/env python3
"""Single-owner FR5 feedback, native drag and Cartesian-servo bridge."""

import inspect
import math
import os
import sys
import threading
import time
import types
import xmlrpc.client

import rclpy
from controller_manager_msgs.srv import SwitchController
from fairino_msgs.msg import PoseTwist
from fr_traction.msg import TractionCommand
from geometry_msgs.msg import Twist, WrenchStamped
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


def _load_robot_sdk(path):
    if path not in sys.path:
        sys.path.insert(0, path)
    # The vendor file imports one unused Cython option even though runtime is
    # pure Python/XML-RPC. Keep deployment independent of a compiler package.
    for name in ("Cython", "Cython.Compiler", "Cython.Compiler.Options"):
        sys.modules.setdefault(name, types.ModuleType(name))
    sys.modules["Cython.Compiler.Options"].error_on_unknown_names = False
    from fairino import Robot  # pylint: disable=import-outside-toplevel

    return Robot


def _quaternion_from_rpy_degrees(roll, pitch, yaw):
    roll, pitch, yaw = map(math.radians, (roll, pitch, yaw))
    cr, sr = math.cos(roll / 2.0), math.sin(roll / 2.0)
    cp, sp = math.cos(pitch / 2.0), math.sin(pitch / 2.0)
    cy, sy = math.cos(yaw / 2.0), math.sin(yaw / 2.0)
    return (
        sr * cp * cy - cr * sp * sy,
        cr * sp * cy + sr * cp * sy,
        cr * cp * sy - sr * sp * cy,
        cr * cp * cy + sr * sp * sy,
    )


def _custom_drag_wrench_to_base(wrench, pose):
    """Convert the drag-only tool frame, rotated 180 deg about Z, to base."""
    roll, pitch, yaw = map(math.radians, pose[3:6])
    cr, sr = math.cos(roll), math.sin(roll)
    cp, sp = math.cos(pitch), math.sin(pitch)
    cy, sy = math.cos(yaw), math.sin(yaw)
    rotation = (
        (cy * cp, cy * sp * sr - sy * cr, cy * sp * cr + sy * sr),
        (sy * cp, sy * sp * sr + cy * cr, sy * sp * cr - cy * sr),
        (-sp, cp * sr, cp * cr),
    )
    result = []
    for start in (0, 3):
        # R_tool_custom = diag(-1, -1, 1); no moment-arm term since
        # the drag reference has zero translation from the tool origin.
        vector = (-wrench[start], -wrench[start + 1], wrench[start + 2])
        result.extend(sum(row[i] * vector[i] for i in range(3)) for row in rotation)
    return result


class _TimeoutTransport(xmlrpc.client.Transport):
    """Apply a real socket timeout to the vendor SDK's command connection."""

    def __init__(self, timeout_s):
        super().__init__()
        self._timeout_s = timeout_s

    def make_connection(self, host):
        connection = super().make_connection(host)
        connection.timeout = self._timeout_s
        if connection.sock is not None:
            connection.sock.settimeout(self._timeout_s)
        return connection


class Fr5DirectDriver(Node):
    """Own the FR SDK connection and expose the existing ROS interface."""

    def __init__(self):
        super().__init__("fr5_direct_driver")
        robot_ip = self.declare_parameter("robot_ip", "192.168.58.2").value
        sdk_path = self.declare_parameter(
            "sdk_python_path",
            os.environ.get("FR5_SDK_PYTHON_PATH", ""),
        ).value
        if not sdk_path:
            raise RuntimeError(
                "sdk_python_path is empty; run the deployment setup or set "
                "FR5_SDK_PYTHON_PATH"
            )
        self._rate_hz = float(self.declare_parameter("update_rate_hz", 100.0).value)
        self._motion_rate_hz = float(
            self.declare_parameter("motion_rate_hz", 25.0).value
        )
        if not math.isfinite(self._motion_rate_hz) or self._motion_rate_hz <= 0.0:
            raise ValueError("motion_rate_hz must be positive")
        self._motion_period_s = 1.0 / self._motion_rate_hz
        self._command_timeout = float(
            self.declare_parameter("command_timeout_s", 0.10).value
        )
        self._command_rpc_timeout_s = float(
            self.declare_parameter("command_rpc_timeout_s", 1.5).value
        )
        self._startup_rpc_timeout_s = float(
            self.declare_parameter("startup_rpc_timeout_s", 10.0).value
        )
        self._allow_existing_force_reference = bool(
            self.declare_parameter("allow_existing_force_reference", True).value
        )
        if (
            not math.isfinite(self._command_rpc_timeout_s)
            or self._command_rpc_timeout_s <= 0.0
        ):
            raise ValueError("command_rpc_timeout_s must be positive")
        if not math.isfinite(self._startup_rpc_timeout_s) or self._startup_rpc_timeout_s <= 0.0:
            raise ValueError("startup_rpc_timeout_s must be positive")
        self._max_speed = float(
            self.declare_parameter("max_linear_speed_mps", 0.025).value
        )
        # The installed FR5 SDK reports the commanded incremental base pose
        # with the opposite sign to GetActualTCPPose on this controller. Keep
        # the compensation explicit so the force controller's physical
        # direction remains the same as the direction shown in base_link.
        self._base_servo_sign = float(
            self.declare_parameter("base_servo_sign", -1.0).value
        )
        if (
            not math.isfinite(self._base_servo_sign)
            or abs(abs(self._base_servo_sign) - 1.0) > 1e-9
        ):
            raise ValueError("base_servo_sign must be either -1.0 or 1.0")
        self._return_speed_mm_s = float(
            self.declare_parameter("return_speed_mm_s", 20.0).value
        )
        self._return_acceleration_mm_s2 = float(
            self.declare_parameter("return_acceleration_mm_s2", 100.0).value
        )
        if not math.isfinite(self._return_speed_mm_s) or self._return_speed_mm_s <= 0.0:
            raise ValueError("return_speed_mm_s must be positive")
        if (
            not math.isfinite(self._return_acceleration_mm_s2)
            or self._return_acceleration_mm_s2 <= 0.0
        ):
            raise ValueError("return_acceleration_mm_s2 must be positive")
        self._tension_search_max_mm = float(
            self.declare_parameter("tension_search_max_mm", 30.0).value
        )
        fixed_zero_pose = list(
            self.declare_parameter(
                "fixed_zero_pose_mm_deg",
                [
                    500.7035522460938,
                    -371.84417724609375,
                    276.12460327148436,
                    -93.77832794189455,
                    1.9229726791381836,
                    -133.9364776611328,
                ],
            ).value
        )
        if len(fixed_zero_pose) != 6 or not all(
            math.isfinite(float(value)) for value in fixed_zero_pose
        ):
            raise ValueError("fixed_zero_pose_mm_deg must contain six finite values")
        self._fixed_zero_pose = [float(value) for value in fixed_zero_pose]
        self._realtime_state_timeout_s = float(
            self.declare_parameter("realtime_state_timeout_s", 0.5).value
        )
        if (
            not math.isfinite(self._realtime_state_timeout_s)
            or self._realtime_state_timeout_s <= 0.0
        ):
            raise ValueError("realtime_state_timeout_s must be positive")
        self._native_drag_mass = self._six_finite_parameter(
            "native_drag_mass", [8.0, 8.0, 8.0, 0.5, 0.5, 0.1]
        )
        self._native_drag_damping = self._six_finite_parameter(
            "native_drag_damping", [120.0, 120.0, 120.0, 5.0, 5.0, 1.0]
        )
        self._native_drag_stiffness = self._six_finite_parameter(
            "native_drag_stiffness", [0.0] * 6
        )
        # The SDK's native drag thresholds refer to its own force processing,
        # not the raw ft_sensor_data stream or the manager's software tare.
        # Use the vendor's recommended lower translational threshold (5 N).
        self._native_drag_threshold = self._six_finite_parameter(
            "native_drag_threshold", [5.0, 5.0, 5.0, 5.0, 5.0, 5.0]
        )
        # The observed native drag motion has inverted tool X/Y signs while Z
        # agrees. Try a 180-degree tool-Z reference for this mode only; the
        # firmware's physical direction still requires a short on-robot check.
        self._native_drag_tool_xy_flip = bool(
            self.declare_parameter("native_drag_tool_xy_flip", True).value
        )
        if any(value <= 0.0 for value in self._native_drag_threshold):
            raise ValueError("native_drag_threshold must contain positive values")
        self._native_drag_max_force_n = float(
            self.declare_parameter("native_drag_max_force_n", 50.0).value
        )
        self._native_drag_max_joint_speed_deg_s = float(
            self.declare_parameter(
                "native_drag_max_joint_speed_deg_s", 50.0
            ).value
        )
        if (
            not math.isfinite(self._native_drag_max_force_n)
            or self._native_drag_max_force_n <= 0.0
        ):
            raise ValueError("native_drag_max_force_n must be positive")
        if (
            not math.isfinite(self._native_drag_max_joint_speed_deg_s)
            or self._native_drag_max_joint_speed_deg_s <= 0.0
        ):
            raise ValueError(
                "native_drag_max_joint_speed_deg_s must be positive"
            )

        robot_module = _load_robot_sdk(str(sdk_path))
        self._robot_ip = str(robot_ip)
        self._robot = robot_module.RPC(self._robot_ip)
        # The vendor constructor restores an XML-RPC proxy with no timeout.
        # A lost ServoMoveEnd response can then block every ROS service in this
        # node indefinitely. Replace only the command proxy; the independent
        # realtime-state socket remains owned by the vendor SDK.
        # FT_SetRCS is a startup/configuration RPC and can take longer than a
        # real-time motion command while the FR5 controller initializes. Use a
        # separate generous timeout here, then switch to the short runtime
        # timeout for ServoCart/ServoMoveEnd and other operating calls.
        self._reset_command_proxy(self._startup_rpc_timeout_s)
        try:
            rcs_code = self._robot.FT_SetRCS(1, [0.0] * 6)
        except (OSError, xmlrpc.client.Error) as error:
            if not self._allow_existing_force_reference:
                raise
            # Some FR5/KWR75D firmware responds to ordinary XML-RPC calls but
            # does not reply to a repeated FT_SetRCS command. In that case the
            # reference selected by the previous successful setup remains in
            # the controller. Continue with an explicit warning; connection
            # and motion RPC failures are still fatal or faulted normally.
            self.get_logger().warning(
                f"FT_SetRCS(base_link) did not respond; using existing force "
                f"reference because allow_existing_force_reference is enabled: {error}"
            )
            rcs_code = 0
        if rcs_code != 0:
            raise RuntimeError(f"FT_SetRCS(base_link) failed: {rcs_code}")
        self._reset_command_proxy()

        self._joint_pub = self.create_publisher(JointState, "/joint_states", 10)
        self._wrench_pub = self.create_publisher(
            WrenchStamped, "/controller_manager/wrench", 10
        )
        self._ee_pub = self.create_publisher(
            PoseTwist, "/controller_manager/ee_state", 10
        )
        health_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._health_pub = self.create_publisher(
            Bool, "/controller_manager/healthy", health_qos
        )
        self.create_subscription(
            Twist, "/controller_manager/command_cart_vel", self._on_twist, 10
        )
        self.create_subscription(
            TractionCommand, "/traction/command", self._on_traction_command, 10
        )
        self.create_service(
            SwitchController,
            "/controller_manager/switch_controller",
            self._on_switch,
        )
        self.create_service(Trigger, "/traction/set_zero_pose", self._on_set_zero)
        self.create_service(Trigger, "/traction/return_zero_pose", self._on_return_zero)
        self.create_service(
            Trigger,
            "/traction/hardware_emergency_stop",
            self._on_hardware_emergency_stop,
        )
        self.create_service(
            Trigger,
            "/traction/hardware_emergency_recover",
            self._on_hardware_emergency_recover,
        )
        self.create_service(
            Trigger,
            "/traction/return_pretraction_pose",
            self._on_return_pretraction,
        )
        self.create_service(
            Trigger,
            "/traction/native_drag_start",
            self._on_native_drag_start,
        )
        self.create_service(
            Trigger,
            "/traction/native_drag_stop",
            self._on_native_drag_stop,
        )
        calibration_qos = QoSProfile(
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(
            Bool,
            "/traction/slack_calibration",
            self._on_slack_calibration,
            calibration_qos,
        )
        self.create_service(
            Trigger,
            "/traction/auto_tension_tool_z_minus",
            self._on_auto_tension,
        )
        self.create_service(
            Trigger,
            "/traction/auto_tension_tool_y_minus",
            self._on_auto_tension_tool_y,
        )
        self.create_service(
            Trigger,
            "/traction/auto_tension_base_z_minus",
            self._on_auto_tension_base,
        )

        self._twist = Twist()
        self._traction_mode = TractionCommand.DISABLED
        self._last_command_at = 0.0
        self._last_motion_at = 0.0
        self._servo_enabled = False
        self._servo_cleanup_pending = False
        self._native_drag_active = False
        self._native_drag_auto_arm_active = False
        self._force_reference_custom_active = False
        self._return_active = False
        self._return_started_at = 0.0
        self._return_duration_s = 0.0
        self._return_start_pose = None
        self._return_target_pose = None
        self._auto_tension_active = False
        self._auto_tension_baseline = None
        self._auto_tension_start_pose = None
        self._auto_tension_since = None
        self._auto_tension_mode = 2
        self._auto_tension_label = "Tool Z-"
        self._auto_tension_increment = [0.0, 0.0, -0.02, 0.0, 0.0, 0.0]
        self._latest_pose = None
        self._latest_joints = None
        self._latest_wrench = None
        self._last_realtime_state = None
        self._last_realtime_state_at = 0.0
        self._zero_pose = list(self._fixed_zero_pose)
        self._pretraction_pose = None
        self._pretraction_joints = None
        self._healthy = True
        self._hardware_fault_latched = False
        self._hardware_emergency_latched = False
        self._last_tick = time.monotonic()
        self._feedback_stop_event = threading.Event()
        self._feedback_thread = threading.Thread(
            target=self._feedback_loop, name="fr5-feedback", daemon=True
        )
        # Feedback is collected by the independent feedback thread at
        # update_rate_hz.  Motion must use its own configured cadence; tying
        # it to the feedback rate made 50 Hz Cartesian servo commands run at
        # only 25 Hz, which was felt as discrete steps during assisted drag.
        self._motion_timer = self.create_timer(
            1.0 / self._motion_rate_hz, self._tick
        )
        # A previous process can die while the FR5 controller keeps native
        # force drag enabled. Never hand control to a fresh UI in that state.
        if self._native_drag_state():
            self._native_drag_active = True
            # The previous process may have left the vendor auto-arm flag on.
            # Force a disarm when closing that inherited drag session.
            self._native_drag_auto_arm_active = True
            if self._stop_native_drag_best_effort("driver startup") != 0:
                self._disable_after_failed_native_stop("driver startup")
                raise RuntimeError("FR5 native force drag remained active at startup")
        self._publish_health(True)
        self._feedback_thread.start()
        self.get_logger().info(
            "FR5 direct driver connected; SDK owner is unique. "
            f"Feedback {self._rate_hz:.1f} Hz, Cartesian servo "
            f"{self._motion_rate_hz:.1f} Hz."
        )

    def _six_finite_parameter(self, name, default):
        values = list(self.declare_parameter(name, default).value)
        if len(values) != 6 or not all(
            math.isfinite(float(value)) for value in values
        ):
            raise ValueError(f"{name} must contain six finite values")
        return [float(value) for value in values]

    def _publish_health(self, value):
        self._healthy = bool(value)
        message = Bool()
        message.data = self._healthy
        self._health_pub.publish(message)

    def _reset_command_proxy(self, timeout_s=None):
        timeout_s = self._command_rpc_timeout_s if timeout_s is None else timeout_s
        self._robot.robot = xmlrpc.client.ServerProxy(
            f"http://{self._robot_ip}:20003",
            transport=_TimeoutTransport(timeout_s),
        )

    def _end_servo(self, context):
        """End servo mode with one fresh-connection retry and bounded latency."""
        for attempt in range(2):
            try:
                code = self._robot.ServoMoveEnd()
            except Exception as error:  # noqa: BLE001 - vendor transport exceptions vary.
                self.get_logger().warning(
                    f"ServoMoveEnd during {context} failed on attempt {attempt + 1}: {error}"
                )
                self._reset_command_proxy()
                continue
            if code == 0:
                self._servo_cleanup_pending = False
                return 0
            self.get_logger().warning(
                f"ServoMoveEnd during {context} returned {code} on attempt {attempt + 1}."
            )
            if attempt == 0:
                self._reset_command_proxy()
        return -4

    def _on_twist(self, message):
        values = (
            message.linear.x,
            message.linear.y,
            message.linear.z,
            message.angular.x,
            message.angular.y,
            message.angular.z,
        )
        if all(math.isfinite(value) for value in values):
            self._twist = message
            self._last_command_at = time.monotonic()

    def _on_traction_command(self, message):
        if message.mode in (
            TractionCommand.DISABLED,
            TractionCommand.PRETENSION,
            TractionCommand.TRACTION,
            TractionCommand.RELEASING,
            TractionCommand.DRAG,
            TractionCommand.POSITIONING,
        ):
            self._traction_mode = message.mode

    @staticmethod
    def _switch_lists(request):
        activate = list(request.activate_controllers) + list(request.start_controllers)
        deactivate = list(request.deactivate_controllers) + list(request.stop_controllers)
        return activate, deactivate

    def _on_switch(self, request, response):
        activate, deactivate = self._switch_lists(request)
        if "cartesian_velocity_controller" in activate and (
            self._return_active
            or self._auto_tension_active
            or self._native_drag_active
            or self._servo_cleanup_pending
            or self._hardware_fault_latched
            or self._hardware_emergency_latched
        ):
            response.ok = False
            return response
        if "cartesian_velocity_controller" in deactivate and self._servo_enabled:
            code = self._end_servo("controller deactivation")
            if code != 0:
                response.ok = False
                return response
            self._servo_enabled = False
            self._return_active = False
            self._return_target_pose = None
            self._auto_tension_active = False
        if "cartesian_velocity_controller" in activate and not self._servo_enabled:
            mode_error = self._ensure_enabled_for_motion(0)
            if mode_error is not None:
                self.get_logger().error(mode_error)
                response.ok = False
                return response
            try:
                code = self._robot.ServoMoveStart()
            except Exception as error:  # noqa: BLE001 - always return a ROS response.
                self.get_logger().error(f"ServoMoveStart failed: {error}")
                self._reset_command_proxy()
                response.ok = False
                return response
            if code != 0:
                response.ok = False
                return response
            self._servo_enabled = True
            self._servo_cleanup_pending = True
            # This is the exact pose before the Cartesian force loop takes
            # control. It is intentionally separate from the slack zero.
            self._pretraction_pose = list(self._latest_pose)
            self._pretraction_joints = list(self._latest_joints)
            self._last_motion_at = time.monotonic()
            # A drag velocity published just before a mode switch must not
            # become the first ServoCart command of a traction task.
            self._twist = Twist()
            self._last_command_at = 0.0
        response.ok = True
        return response

    def _set_native_drag(self, enabled):
        """Call the controller-resident drag loop across supported SDK versions."""
        status = 1 if enabled else 0
        method = self._robot.EndForceDragControl
        # The deployed, pinned SDK has the original 9-argument API. Newer
        # FAIRINO SDK releases inserted singularity/collision flags. Inspect
        # the wrapped method so the same project remains deployable with both.
        parameter_count = len(inspect.signature(method).parameters)
        common = (
            self._native_drag_mass,
            self._native_drag_damping,
            self._native_drag_stiffness,
            self._native_drag_threshold,
            self._native_drag_max_force_n,
            self._native_drag_max_joint_speed_deg_s,
        )
        if parameter_count >= 11:
            return int(method(status, 0, 0, 0, 0, *common))
        return int(method(status, 0, 0, *common))

    def _set_drag_force_reference(self, custom):
        """Set the native drag FT frame; use bounded RPC, not SDK retry loop."""
        ref = 2 if custom else 1
        coord = [0.0, 0.0, 0.0, 0.0, 0.0, 180.0 if custom else 0.0]
        if custom:
            # A timed-out response may still mean the controller accepted the
            # command. Force the failure path to attempt a base-frame restore.
            self._force_reference_custom_active = True
        # The configuration RPC may take longer than a ServoCart command.
        self._reset_command_proxy(self._startup_rpc_timeout_s)
        try:
            proxy = getattr(self._robot, "robot", self._robot)
            code = int(proxy.FT_SetRCS(ref, coord))
        finally:
            self._reset_command_proxy()
        if code != 0:
            raise RuntimeError(f"FT_SetRCS({ref}) failed: {code}")
        self._force_reference_custom_active = custom

    def _native_drag_state(self):
        state = self._robot.GetForceAndTorqueDragState()
        if not isinstance(state, (tuple, list)) or len(state) < 3:
            raise RuntimeError(f"FR5 native drag status is unavailable: {state}")
        if int(state[0]) != 0:
            raise RuntimeError(f"FR5 native drag status failed: {state[0]}")
        return int(state[1]) == 1

    def _wait_native_drag_state(self, expected):
        for _ in range(4):
            if self._native_drag_state() == expected:
                return True
            time.sleep(0.1)
        return False

    def _disarm_native_drag_auto(self, context):
        if not self._native_drag_auto_arm_active:
            return 0
        try:
            code = int(self._robot.SetForceSensorDragAutoFlag(0))
        except Exception as error:  # noqa: BLE001 - vendor transport exceptions vary.
            self.get_logger().warning(
                f"Native drag auto-arm disable during {context} failed: {error}"
            )
            self._reset_command_proxy()
            return -7
        if code != 0:
            self.get_logger().warning(
                f"Native drag auto-arm disable during {context} returned {code}."
            )
            return -7
        self._native_drag_auto_arm_active = False
        return 0

    def _stop_native_drag_best_effort(self, context):
        if not self._native_drag_active:
            disarm_code = self._disarm_native_drag_auto(context)
            if self._force_reference_custom_active:
                try:
                    self._set_drag_force_reference(False)
                except Exception as error:  # noqa: BLE001 - vendor exceptions vary.
                    self.get_logger().warning(
                        f"Force reference restore during {context} failed: {error}"
                    )
                    self._reset_command_proxy()
                    return -6
            return disarm_code
        try:
            code = self._set_native_drag(False)
        except Exception as error:  # noqa: BLE001 - vendor exceptions vary.
            self.get_logger().warning(
                f"Native force drag stop during {context} failed: {error}"
            )
            self._reset_command_proxy()
            return -4
        if code == 0:
            try:
                if not self._wait_native_drag_state(False):
                    self.get_logger().warning(
                        f"Native force drag remained on during {context}."
                    )
                    return -5
            except Exception as error:  # noqa: BLE001 - vendor status errors vary.
                self.get_logger().warning(
                    f"Native force drag stop status during {context} failed: {error}"
                )
                self._reset_command_proxy()
                return -4
            self._native_drag_active = False
            # Automatic-mode native drag is armed explicitly at session start.
            # Disarm it after a confirmed stop so clearing a later fault cannot
            # restart assisted drag without a new operator request.
            disarm_code = self._disarm_native_drag_auto(context)
            if self._force_reference_custom_active:
                try:
                    self._set_drag_force_reference(False)
                except Exception as error:  # noqa: BLE001 - vendor exceptions vary.
                    self.get_logger().warning(
                        f"Force reference restore during {context} failed: {error}"
                    )
                    self._reset_command_proxy()
                    return -6
            return disarm_code
        else:
            self.get_logger().warning(
                f"Native force drag stop during {context} returned {code}."
            )
        return code

    def _disable_after_failed_native_stop(self, context):
        """Remove robot enable when controller drag-off cannot be confirmed."""
        self._hardware_emergency_latched = True
        self._hardware_fault_latched = True
        self._run_hardware_command("StopMotion")
        disable_code, _ = self._run_hardware_command("RobotEnable", 0)
        self._disarm_native_drag_auto(context)
        if disable_code == 0:
            self._native_drag_active = False
        self._publish_health(False)
        self.get_logger().error(
            f"Native force drag stop during {context} was unconfirmed; "
            f"FR5 RobotEnable(0) result: {disable_code}."
        )

    def _on_native_drag_start(self, _request, response):
        if self._native_drag_active:
            response.success = True
            response.message = "FR5 native force drag is already active."
            return response
        if (
            self._servo_enabled
            or self._return_active
            or self._auto_tension_active
            or self._hardware_fault_latched
            or self._hardware_emergency_latched
            or self._servo_cleanup_pending
            or self._latest_pose is None
            or self._latest_wrench is None
            or time.monotonic() - self._last_realtime_state_at
            > self._realtime_state_timeout_s
        ):
            response.success = False
            response.message = (
                "FR5 native force drag rejected because motion, feedback, or "
                "previous Cartesian servo cleanup is unavailable."
            )
            return response
        # Preserve the ee9830d native-drag path when manual mode is already
        # enabled: do not change the controller mode out from under drag.
        # Emergency recovery may leave manual mode intentionally unenabled;
        # only in that case hand off to enabled automatic mode.
        state = self._robot.robot_state_pkg
        manual_enabled = (
            int(state.robot_mode) == 1 and int(state.rbtEnableState) == 1
        )
        expected_mode = 1 if manual_enabled else 0
        if not manual_enabled:
            mode_error = self._ensure_enabled_for_motion(0)
            if mode_error is not None:
                response.success = False
                response.message = "FR5 native force drag rejected: " + mode_error
                return response
        readiness_error = self._robot_motion_readiness_error(expected_mode=expected_mode)
        if readiness_error is not None:
            response.success = False
            response.message = "FR5 native force drag rejected: " + readiness_error
            return response
        # The raw stream can include the sensor/tool payload even after the
        # software slack tare. Its absolute magnitude is not evidence of a
        # user pull and must not block SDK drag activation (session 1789878276624).
        if not all(math.isfinite(float(value)) for value in self._latest_wrench):
            response.success = False
            response.message = "Native drag rejected: invalid force sensor data."
            return response
        try:
            # This start path selects automatic mode. The vendor's assisted-
            # drag sequence arms the force sensor before EndForceDragControl;
            # leaving the flag off can report drag ON without physical motion
            # after an emergency recovery. The stop path disarms it again.
            auto_code = int(self._robot.SetForceSensorDragAutoFlag(1))
            if auto_code != 0:
                response.success = False
                response.message = (
                    "SetForceSensorDragAutoFlag failed: " + str(auto_code)
                )
                return response
            self._native_drag_auto_arm_active = True
            if self._native_drag_tool_xy_flip:
                self._set_drag_force_reference(True)
            code = self._set_native_drag(True)
        except Exception as error:  # noqa: BLE001 - vendor exceptions vary.
            self.get_logger().error(f"Native force drag start failed: {error}")
            self._reset_command_proxy()
            if self._stop_native_drag_best_effort("failed drag start") != 0:
                self._disable_after_failed_native_stop("failed drag start")
            response.success = False
            response.message = f"FR5 native force drag start failed: {error}"
            return response
        if code != 0:
            if self._stop_native_drag_best_effort("rejected drag start") != 0:
                self._disable_after_failed_native_stop("rejected drag start")
            response.success = False
            response.message = f"EndForceDragControl start failed: {code}"
            return response
        self._native_drag_active = True
        self._twist = Twist()
        self._last_command_at = 0.0
        try:
            if not self._wait_native_drag_state(True):
                raise RuntimeError("FR5 did not confirm native force drag activation")
        except Exception as error:  # noqa: BLE001 - status RPC failures vary.
            if self._stop_native_drag_best_effort("unconfirmed drag activation") != 0:
                self._disable_after_failed_native_stop("unconfirmed drag activation")
            response.success = False
            response.message = str(error)
            return response
        self._pretraction_pose = list(self._latest_pose)
        self._pretraction_joints = list(self._latest_joints or [])
        response.success = True
        response.message = "FR5 controller-resident force drag started."
        self.get_logger().info(
            "Native force drag active. Translational thresholds (N): "
            f"{self._native_drag_threshold[:3]}."
        )
        return response

    def _on_native_drag_stop(self, _request, response):
        if (not self._native_drag_active and not self._force_reference_custom_active
                and not self._native_drag_auto_arm_active):
            response.success = True
            response.message = "FR5 native force drag is already stopped."
            return response
        code = self._stop_native_drag_best_effort("drag stop service")
        self._twist = Twist()
        self._last_command_at = 0.0
        if code != 0:
            self._disable_after_failed_native_stop("drag stop service")
        response.success = code == 0
        response.message = (
            "FR5 controller-resident force drag stopped."
            if code == 0
            else f"EndForceDragControl stop failed: {code}"
        )
        return response

    def _run_hardware_command(self, command, *args):
        """Run one bounded FR5 state command and keep later commands possible."""
        try:
            return int(getattr(self._robot, command)(*args)), ""
        except Exception as error:  # noqa: BLE001 - vendor exceptions vary.
            self.get_logger().error(f"{command} failed: {error}")
            self._reset_command_proxy()
            return None, str(error)

    def _robot_motion_readiness_error(self, expected_mode=None, require_enabled=True):
        """Check controller feedback, not just zero-valued RPC acknowledgements."""
        state = self._robot.robot_state_pkg
        if isinstance(state, type) or not hasattr(state, "frame_cnt"):
            return "FR5 realtime controller status is unavailable."
        if int(state.EmergencyStop) != 0:
            return "FR5 emergency stop input is still active."
        if require_enabled and int(state.rbtEnableState) != 1:
            return "FR5 robot enable did not become active."
        if int(state.ft_sensor_active) != 1:
            return "FR5 force sensor is not active."
        if int(state.main_code) != 0:
            return f"FR5 controller fault is still active: {state.main_code}/{state.sub_code}."
        if expected_mode is not None and int(state.robot_mode) != expected_mode:
            label = "manual" if expected_mode == 1 else "automatic"
            return f"FR5 did not enter {label} mode."
        return None

    def _select_motion_mode(self, mode, force=False):
        """Switch mode before starting native drag or Cartesian servo motion."""
        state = self._robot.robot_state_pkg
        if not force and hasattr(state, "robot_mode") and int(state.robot_mode) == mode:
            return None
        previous_frame = getattr(state, "frame_cnt", None)
        code, detail = self._run_hardware_command("Mode", mode)
        if code != 0:
            return f"Mode({mode}) failed: {code if code is not None else detail}."
        for _ in range(20):
            feedback = self._robot.robot_state_pkg
            if (getattr(feedback, "frame_cnt", None) != previous_frame
                    and int(feedback.robot_mode) == mode):
                return None
            time.sleep(0.05)
        return f"FR5 did not confirm Mode({mode}) from realtime feedback."

    def _ensure_enabled_for_motion(self, mode):
        """Enable only for a requested motion, never for passive manual recovery."""
        if time.monotonic() - self._last_realtime_state_at > self._realtime_state_timeout_s:
            return "FR5 realtime feedback is stale."
        mode_error = self._select_motion_mode(mode)
        if mode_error is not None:
            return mode_error
        state = self._robot.robot_state_pkg
        if int(state.EmergencyStop) != 0 or int(state.main_code) != 0:
            return "FR5 emergency stop or controller fault is still active."
        if int(state.rbtEnableState) != 1:
            previous_frame = state.frame_cnt
            code, detail = self._run_hardware_command("RobotEnable", 1)
            if code != 0:
                return f"RobotEnable(1) failed: {code if code is not None else detail}."
            for _ in range(20):
                state = self._robot.robot_state_pkg
                if state.frame_cnt != previous_frame and int(state.rbtEnableState) == 1:
                    break
                time.sleep(0.05)
        return self._robot_motion_readiness_error(expected_mode=mode)

    def _on_hardware_emergency_stop(self, _request, response):
        # Latch locally before the first RPC so no later controller request can
        # restart motion while the hardware stop sequence is still executing.
        self._hardware_emergency_latched = True
        self._twist = Twist()
        self._traction_mode = TractionCommand.DISABLED
        self._return_active = False
        self._return_target_pose = None
        self._auto_tension_active = False

        results = []
        for command, args in (
            ("StopMotion", ()),
            ("RobotEnable", (0,)),
            ("Mode", (1,)),
        ):
            code, detail = self._run_hardware_command(command, *args)
            results.append((command, code, detail))
        # Stop/disable first; a slow native-drag RPC must never delay the
        # physical emergency response. Then clear the controller drag mode.
        self._stop_native_drag_best_effort("hardware emergency stop")
        self._servo_enabled = False

        disable_code = next(code for name, code, _ in results if name == "RobotEnable")
        if disable_code == 0:
            # A hardware-disabled robot cannot remain physically in drag mode.
            # Clear a stale local flag even if the preceding drag-stop RPC was
            # the command that timed out, otherwise recovery would be blocked.
            self._native_drag_active = False
            warnings = [
                f"{name}={code if code is not None else detail}"
                for name, code, detail in results
                if code != 0
            ]
            response.success = True
            response.message = "FR5 motion stopped and robot power enable disabled."
            if warnings:
                response.message += " Additional results: " + ", ".join(warnings)
            return response

        response.success = False
        response.message = "FR5 emergency stop could not confirm RobotEnable(0): " + ", ".join(
            f"{name}={code if code is not None else detail}"
            for name, code, detail in results
        )
        return response

    def _on_hardware_emergency_recover(self, _request, response):
        if self._native_drag_active or self._force_reference_custom_active:
            code = self._stop_native_drag_best_effort("hardware recovery")
            if code != 0:
                response.success = False
                response.message = "FR5 emergency recovery could not stop native drag."
                return response
        if (
            self._servo_enabled
            or self._return_active
            or self._auto_tension_active
        ):
            response.success = False
            response.message = "FR5 emergency recovery rejected while motion is active."
            return response

        results = []
        for command, args in (
            ("ResetAllError", ()),
            ("Mode", (0,)),
            ("RobotEnable", (1,)),
        ):
            code, detail = self._run_hardware_command(command, *args)
            results.append((command, code, detail))

        enable_code = next(code for name, code, _ in results if name == "RobotEnable")
        mode_code = next(code for name, code, _ in results if name == "Mode")
        reset_code = next(code for name, code, _ in results if name == "ResetAllError")
        if enable_code == 0 and mode_code == 0 and reset_code == 0:
            if self._servo_cleanup_pending:
                # StopMotion and RobotEnable(0) halt motion, but do not close
                # ServoMoveStart. A stale Cartesian servo session can make
                # EndForceDragControl report ON while the arm cannot move.
                servo_end_code = self._end_servo("hardware emergency recovery")
                if servo_end_code != 0:
                    self._run_hardware_command("RobotEnable", 0)
                    self._run_hardware_command("Mode", 1)
                    response.success = False
                    response.message = (
                        "FR5 emergency recovery could not close the previous "
                        "Cartesian servo session. Robot disabled."
                    )
                    return response
            state = self._robot.robot_state_pkg
            if hasattr(state, "ft_sensor_active") and int(state.ft_sensor_active) != 1:
                sensor_code, sensor_detail = self._run_hardware_command("FT_Activate", 1)
                if sensor_code != 0:
                    self._run_hardware_command("RobotEnable", 0)
                    self._run_hardware_command("Mode", 1)
                    response.success = False
                    response.message = (
                        "FR5 emergency recovery could not reactivate the force "
                        f"sensor: {sensor_code if sensor_code is not None else sensor_detail}."
                    )
                    return response
            # Green manual mode does not imply motor enable. With a teach
            # pendant, the physical three-position switch can leave the arm
            # disabled until the next deliberate motion request.
            mode_error = self._select_motion_mode(1, force=True)
            if mode_error is not None:
                self._run_hardware_command("RobotEnable", 0)
                response.success = False
                response.message = "FR5 emergency recovery incomplete: " + mode_error
                return response
            readiness_error = None
            for _ in range(20):
                readiness_error = self._robot_motion_readiness_error(
                    expected_mode=1, require_enabled=False
                )
                if readiness_error is None:
                    break
                time.sleep(0.05)
            if readiness_error is not None:
                self._run_hardware_command("RobotEnable", 0)
                response.success = False
                response.message = "FR5 emergency recovery incomplete: " + readiness_error
                return response
            self._hardware_emergency_latched = False
            self._hardware_fault_latched = False
            self._publish_health(True)
            response.success = True
            response.message = "FR5 errors cleared; manual mode ready for setup."
            return response

        self._run_hardware_command("RobotEnable", 0)
        self._run_hardware_command("Mode", 1)
        response.success = False
        response.message = "FR5 emergency recovery failed: " + ", ".join(
            f"{name}={code if code is not None else detail}"
            for name, code, detail in results
        )
        return response

    def _on_set_zero(self, _request, response):
        response.success = False
        response.message = "The return-zero pose is fixed by the project configuration."
        return response

    def _on_slack_calibration(self, message):
        if not message.data:
            return
        if (
            self._servo_enabled
            or self._native_drag_active
            or self._hardware_fault_latched
            or self._hardware_emergency_latched
        ):
            self.get_logger().warning(
                "Slack calibration ignored because FR5 motion or feedback is active."
            )
            return
        # Initial force calibration updates the sensor baseline in the manager,
        # but must never redefine the fixed return-zero pose.
        self.get_logger().info("Slack force calibration received; fixed zero pose unchanged.")

    def _on_return_zero(self, _request, response):
        return self._start_return_to_pose(
            response,
            self._zero_pose,
            "stored slack zero",
        )

    def _on_return_pretraction(self, _request, response):
        return self._start_return_to_pose(
            response,
            self._pretraction_pose,
            "pre-traction pose",
        )

    def _start_return_to_pose(self, response, target_pose, label):
        if target_pose is None:
            response.success = False
            response.message = f"Return rejected: {label} is not stored."
            return response
        if self._servo_enabled or self._servo_cleanup_pending or self._native_drag_active:
            response.success = False
            response.message = "Return rejected: servo motion is busy."
            return response
        if (
            self._latest_pose is None
            or self._hardware_fault_latched
            or self._hardware_emergency_latched
        ):
            response.success = False
            response.message = "Return rejected: FR5 feedback is unavailable."
            return response
        distance_mm = math.sqrt(
            sum((target_pose[i] - self._latest_pose[i]) ** 2 for i in range(3))
        )
        mode_error = self._ensure_enabled_for_motion(0)
        if mode_error is not None:
            response.success = False
            response.message = "Return rejected: " + mode_error
            return response
        try:
            code = self._robot.ServoMoveStart()
        except Exception as error:  # noqa: BLE001 - always return a ROS response.
            self.get_logger().error(f"Return ServoMoveStart failed: {error}")
            self._reset_command_proxy()
            response.success = False
            response.message = "Return rejected: FR5 command connection timed out."
            return response
        if code != 0:
            response.success = False
            response.message = f"ServoMoveStart failed: {code}."
            return response
        self._servo_enabled = True
        self._servo_cleanup_pending = True
        self._last_motion_at = time.monotonic()
        self._return_active = True
        self._return_started_at = time.monotonic()
        self._return_start_pose = list(self._latest_pose)
        self._return_target_pose = list(target_pose)
        # Quintic minimum-jerk position profile. 1.875 and 5.774 are the peak
        # first/second derivatives of 10s^3-15s^4+6s^5 on [0, 1].
        speed_limited_duration = 1.875 * distance_mm / self._return_speed_mm_s
        acceleration_limited_duration = math.sqrt(
            5.774 * distance_mm / self._return_acceleration_mm_s2
        )
        self._return_duration_s = max(
            0.10, speed_limited_duration, acceleration_limited_duration
        )
        response.success = True
        response.message = f"Position return to the {label} has started."
        return response

    def _on_auto_tension(self, _request, response):
        return self._start_auto_tension(
            response, 2, "Tool Z-", [0.0, 0.0, -0.02, 0.0, 0.0, 0.0]
        )

    def _on_auto_tension_tool_y(self, _request, response):
        return self._start_auto_tension(
            response, 2, "Tool Y-", [0.0, -0.02, 0.0, 0.0, 0.0, 0.0]
        )

    def _on_auto_tension_base(self, _request, response):
        return self._start_auto_tension(
            response, 1, "Base Z-", [0.0, 0.0, -0.02, 0.0, 0.0, 0.0]
        )

    def _start_auto_tension(self, response, mode, label, increment):
        # Read the limit at each operation. ROS2 parameter updates are valid
        # at runtime; using only the constructor cache made a successful
        # parameter change appear ineffective to the operator.
        current_limit = float(self.get_parameter("tension_search_max_mm").value)
        if not math.isfinite(current_limit) or current_limit <= 0.0:
            response.success = False
            response.message = "Auto tension rejected: tension_search_max_mm is invalid."
            return response
        self._tension_search_max_mm = current_limit
        if (
            self._servo_enabled
            or self._servo_cleanup_pending
            or self._native_drag_active
            or self._hardware_fault_latched
            or self._hardware_emergency_latched
            or self._latest_pose is None
            or self._latest_wrench is None
        ):
            response.success = False
            response.message = "Auto tension rejected: servo is busy or feedback is stale."
            return response
        mode_error = self._ensure_enabled_for_motion(0)
        if mode_error is not None:
            response.success = False
            response.message = "Auto tension rejected: " + mode_error
            return response
        try:
            code = self._robot.ServoMoveStart()
        except Exception as error:  # noqa: BLE001 - always return a ROS response.
            self.get_logger().error(f"Auto-tension ServoMoveStart failed: {error}")
            self._reset_command_proxy()
            response.success = False
            response.message = "Auto tension rejected: FR5 command connection timed out."
            return response
        if code != 0:
            response.success = False
            response.message = f"ServoMoveStart failed: {code}."
            return response
        self._servo_enabled = True
        self._servo_cleanup_pending = True
        self._last_motion_at = time.monotonic()
        self._auto_tension_active = True
        self._auto_tension_baseline = list(self._latest_wrench)
        self._auto_tension_start_pose = list(self._latest_pose)
        self._auto_tension_since = None
        self._auto_tension_mode = mode
        self._auto_tension_label = label
        self._auto_tension_increment = increment
        response.success = True
        response.message = (
            f"Bounded {label} tension search started: 2 mm/s, "
            f"{self._tension_search_max_mm:.0f} mm maximum."
        )
        return response

    def _publish_feedback(self, stamp, joints, speeds, wrench, pose):
        joint_message = JointState()
        joint_message.header.stamp = stamp
        joint_message.name = [f"j{i}" for i in range(1, 7)]
        joint_message.position = [math.radians(value) for value in joints]
        joint_message.velocity = [math.radians(value) for value in speeds]
        self._joint_pub.publish(joint_message)

        wrench_message = WrenchStamped()
        wrench_message.header.stamp = stamp
        wrench_message.header.frame_id = "base_link"
        (
            wrench_message.wrench.force.x,
            wrench_message.wrench.force.y,
            wrench_message.wrench.force.z,
        ) = wrench[:3]
        (
            wrench_message.wrench.torque.x,
            wrench_message.wrench.torque.y,
            wrench_message.wrench.torque.z,
        ) = wrench[3:]
        self._wrench_pub.publish(wrench_message)

        ee_message = PoseTwist()
        ee_message.header.stamp = stamp
        ee_message.header.frame_id = "base_link"
        ee_message.pose.position.x = pose[0] / 1000.0
        ee_message.pose.position.y = pose[1] / 1000.0
        ee_message.pose.position.z = pose[2] / 1000.0
        quaternion = _quaternion_from_rpy_degrees(*pose[3:])
        ee_message.pose.orientation.x, ee_message.pose.orientation.y = quaternion[:2]
        ee_message.pose.orientation.z, ee_message.pose.orientation.w = quaternion[2:]
        self._ee_pub.publish(ee_message)

    def _read_realtime_state(self, now):
        """
        Read one coherent snapshot from the SDK's realtime state stream.

        The individual GetActual* XML-RPC queries are not suitable for the
        force loop: GetActualJointPosDegree can block for about a second while
        ServoCart is active.  The SDK already receives all required values in
        its realtime port 20004 packet, so use that single packet instead.
        """
        state = self._robot.robot_state_pkg
        if isinstance(state, type) or not hasattr(state, "frame_cnt"):
            raise RuntimeError("FR5 realtime state packet is not available")
        if state is self._last_realtime_state:
            if now - self._last_realtime_state_at > self._realtime_state_timeout_s:
                raise RuntimeError("FR5 realtime state stream is stale")
            return None
        self._last_realtime_state = state
        self._last_realtime_state_at = now
        joints = [state.jt_cur_pos[index] for index in range(6)]
        pose = [state.tl_cur_pos[index] for index in range(6)]
        speeds = [state.actual_qd[index] for index in range(6)]
        wrench = [state.ft_sensor_data[index] for index in range(6)]
        if self._force_reference_custom_active:
            wrench = _custom_drag_wrench_to_base(wrench, pose)
        return joints, pose, speeds, wrench

    def _servo_cart(self, mode, desc_pos):
        # The FR5 XML-RPC parser does not accept denormal floating-point
        # values serialized in scientific notation.  A velocity command that
        # is already far below the robot's resolution has no physical effect,
        # so transmit it as an exact zero instead of allowing a long
        # deceleration tail to become e-318 and fault the controller.
        if len(desc_pos) != 6:
            raise ValueError("ServoCart desc_pos must contain six values")
        sanitized_desc_pos = []
        for value in desc_pos:
            value = float(value)
            if not math.isfinite(value):
                raise ValueError("ServoCart desc_pos must contain finite values")
            sanitized_desc_pos.append(0.0 if abs(value) < 1e-9 else value)
        started_at = time.monotonic()
        # cmdT is the execution duration understood by the FR5 controller,
        # not merely metadata. Keep it equal to the host send period so the
        # controller does not finish an 8 ms move and sit idle while waiting
        # for a later command (the former source of assisted-drag stepping).
        code = self._robot.ServoCart(
            mode, sanitized_desc_pos, cmdT=self._motion_period_s
        )
        elapsed = time.monotonic() - started_at
        if elapsed > 0.05:
            self.get_logger().warning(
                f"ServoCart mode {mode} RPC took {elapsed:.3f} s."
            )
        return code

    def _send_motion(self, now, dt):
        # Assisted drag runs continuously inside the FR5 controller. Never
        # interleave host-side ServoCart packets with that controller mode.
        if self._native_drag_active:
            return
        if not self._servo_enabled:
            return
        # _tick itself already runs at motion_rate_hz. A second strict period
        # gate here used to discard a callback whenever timer jitter made it
        # arrive a fraction early, effectively halving the requested rate.
        motion_dt = now - self._last_motion_at if self._last_motion_at > 0.0 else dt
        self._last_motion_at = now
        motion_dt = min(max(motion_dt, 0.0), 0.05)

        if self._auto_tension_active:
            displacement = [
                self._latest_pose[i] - self._auto_tension_start_pose[i]
                for i in range(3)
            ]
            travel_mm = math.sqrt(
                sum(
                    (self._latest_pose[i] - self._auto_tension_start_pose[i]) ** 2
                    for i in range(3)
                )
            )
            force_change = [
                self._latest_wrench[i] - self._auto_tension_baseline[i]
                for i in range(3)
            ]
            if travel_mm > 0.5:
                travel_direction = [value / travel_mm for value in displacement]
                force_increase = sum(
                    force_change[i] * travel_direction[i] for i in range(3)
                )
            else:
                force_increase = 0.0
            if force_increase >= 10.0:
                self._end_servo("auto-tension force limit")
                self._servo_enabled = False
                self._auto_tension_active = False
                self.get_logger().warning(
                    "Auto tension stopped at the 10 N diagnostic limit."
                )
                return
            if force_increase >= 1.5:
                if self._auto_tension_since is None:
                    self._auto_tension_since = now
                elif now - self._auto_tension_since >= 0.2:
                    self._end_servo("auto-tension completion")
                    self._servo_enabled = False
                    self._auto_tension_active = False
                    self.get_logger().info(
                        f"{self._auto_tension_label} tension found at "
                        f"+{force_increase:.2f} N after {travel_mm:.2f} mm."
                    )
                    return
            else:
                self._auto_tension_since = None
            if travel_mm >= self._tension_search_max_mm:
                self._end_servo("auto-tension travel limit")
                self._servo_enabled = False
                self._auto_tension_active = False
                self.get_logger().warning(
                    f"{self._auto_tension_label} tension search reached "
                    f"{self._tension_search_max_mm:.0f} mm without stable tension."
                )
                return
            increment = [value * motion_dt / 0.01 for value in self._auto_tension_increment]
            code = self._servo_cart(self._auto_tension_mode, increment)
            if code != 0:
                raise RuntimeError(
                    f"ServoCart {self._auto_tension_label} tension search failed: {code}"
                )
            return
        if self._return_active:
            elapsed_s = min(self._return_duration_s, now - self._return_started_at)
            phase = min(1.0, max(0.0, elapsed_s / self._return_duration_s))
            alpha = 10.0 * phase**3 - 15.0 * phase**4 + 6.0 * phase**5
            target = [
                start + alpha * (target - start)
                for start, target in zip(self._return_start_pose, self._return_target_pose)
            ]
            code = self._servo_cart(0, target)
            if alpha >= 1.0:
                self._end_servo("position return completion")
                self._servo_enabled = False
                self._return_active = False
                self._return_target_pose = None
            if code != 0:
                raise RuntimeError(f"ServoCart return-zero failed: {code}")
            return

        command_fresh = now - self._last_command_at <= self._command_timeout
        linear = [self._twist.linear.x, self._twist.linear.y, self._twist.linear.z]
        magnitude = math.sqrt(sum(value * value for value in linear))
        if not command_fresh:
            linear = [0.0, 0.0, 0.0]
        elif magnitude > self._max_speed and magnitude > 0.0:
            linear = [value * self._max_speed / magnitude for value in linear]
        # Assisted drag is generated in tool coordinates by the controller;
        # all other velocity commands remain base-coordinate increments.
        servo_mode = 2 if self._traction_mode == TractionCommand.DRAG else 1
        command_sign = self._base_servo_sign if servo_mode == 1 else 1.0
        increment = [
            command_sign * value * motion_dt * 1000.0
            for value in linear
        ]
        code = self._servo_cart(servo_mode, increment + [0.0, 0.0, 0.0])
        if code != 0:
            raise RuntimeError(f"ServoCart traction failed: {code}")

    def _feedback_loop(self):
        """Publish feedback independently of potentially slow motion RPCs."""
        period = 1.0 / self._rate_hz
        while not self._feedback_stop_event.is_set():
            now = time.monotonic()
            try:
                state_snapshot = self._read_realtime_state(now)
                if state_snapshot is not None:
                    joints, pose, speeds, wrench = state_snapshot
                    self._latest_joints, self._latest_pose = list(joints), list(pose)
                    self._latest_wrench = list(wrench)
                    self._publish_feedback(
                        self.get_clock().now().to_msg(), joints, speeds, wrench, pose
                    )
                    self._publish_health(not self._hardware_fault_latched)
                # No new packet means no new health confirmation. The manager
                # will use the last real feedback timestamp for stale detection.
            except Exception as error:  # noqa: BLE001  # Hardware faults must latch health loss.
                self.get_logger().error(str(error))
                if self._stop_native_drag_best_effort("feedback fault") != 0:
                    self._disable_after_failed_native_stop("feedback fault")
                if self._servo_enabled:
                    self._end_servo("feedback fault")
                self._servo_enabled = False
                self._return_active = False
                self._return_target_pose = None
                self._auto_tension_active = False
                self._hardware_fault_latched = True
                self._publish_health(False)
            self._feedback_stop_event.wait(period)

    def _tick(self):
        """Run motion commands without holding up the feedback publisher."""
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        try:
            self._send_motion(now, dt)
        except Exception as error:  # noqa: BLE001  # Hardware faults must latch health loss.
            self.get_logger().error(str(error))
            if self._stop_native_drag_best_effort("motion fault") != 0:
                self._disable_after_failed_native_stop("motion fault")
            if self._servo_enabled:
                self._end_servo("motion fault")
            self._servo_enabled = False
            self._return_active = False
            self._return_target_pose = None
            self._auto_tension_active = False
            self._hardware_fault_latched = True
            self._publish_health(False)

    def destroy_node(self):
        self._feedback_stop_event.set()
        if self._feedback_thread.is_alive():
            self._feedback_thread.join(timeout=1.0)
        if self._servo_enabled:
            self._end_servo("driver shutdown")
        if self._stop_native_drag_best_effort("driver shutdown") != 0:
            self._disable_after_failed_native_stop("driver shutdown")
        self._robot.CloseRPC()
        return super().destroy_node()


def main():
    rclpy.init()
    node = Fr5DirectDriver()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
