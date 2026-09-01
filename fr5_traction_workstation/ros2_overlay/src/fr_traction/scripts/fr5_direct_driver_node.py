#!/usr/bin/env python3
"""Single-owner FR5 feedback and Cartesian-servo bridge for traction."""

import math
import sys
import time
import types

import rclpy
from controller_manager_msgs.srv import SwitchController
from fairino_msgs.msg import PoseTwist
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


class Fr5DirectDriver(Node):
    """Own the FR SDK connection and expose the existing ROS interface."""

    def __init__(self):
        super().__init__("fr5_direct_driver")
        robot_ip = self.declare_parameter("robot_ip", "192.168.58.2").value
        sdk_path = self.declare_parameter(
            "sdk_python_path",
            "/home/zhj/projects/fr5_learning/vendor/fairino-python-sdk/linux",
        ).value
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
        self._max_speed = float(
            self.declare_parameter("max_linear_speed_mps", 0.005).value
        )
        # The installed FR5 SDK reports the commanded incremental base pose
        # with the opposite sign to GetActualTCPPose on this controller. Keep
        # the compensation explicit so the force controller's physical
        # direction remains the same as the direction shown in base_link.
        self._base_servo_sign = float(
            self.declare_parameter("base_servo_sign", -1.0).value
        )
        if not math.isfinite(self._base_servo_sign) or abs(abs(self._base_servo_sign) - 1.0) > 1e-9:
            raise ValueError("base_servo_sign must be either -1.0 or 1.0")
        self._return_speed_mm_s = float(
            self.declare_parameter("return_speed_mm_s", 2.0).value
        )
        self.declare_parameter("return_max_distance_mm", 35.0)
        self._tension_search_max_mm = float(
            self.declare_parameter("tension_search_max_mm", 30.0).value
        )
        self._auto_set_zero = bool(
            self.declare_parameter("auto_set_zero_on_start", True).value
        )

        robot_module = _load_robot_sdk(str(sdk_path))
        self._robot = robot_module.RPC(str(robot_ip))
        rcs_code = self._robot.FT_SetRCS(1, [0.0] * 6)
        if rcs_code != 0:
            raise RuntimeError(f"FT_SetRCS(base_link) failed: {rcs_code}")

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
        self.create_service(
            SwitchController,
            "/controller_manager/switch_controller",
            self._on_switch,
        )
        self.create_service(Trigger, "/traction/set_zero_pose", self._on_set_zero)
        self.create_service(Trigger, "/traction/return_zero_pose", self._on_return_zero)
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
        self._last_command_at = 0.0
        self._last_motion_at = 0.0
        self._servo_enabled = False
        self._return_active = False
        self._return_started_at = 0.0
        self._return_duration_s = 0.0
        self._return_start_pose = None
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
        self._zero_pose = None
        self._zero_joints = None
        self._healthy = True
        self._hardware_fault_latched = False
        self._last_tick = time.monotonic()
        self.create_timer(1.0 / self._rate_hz, self._tick)
        self._publish_health(True)
        self.get_logger().info("FR5 direct driver connected; SDK owner is unique.")

    def _publish_health(self, value):
        self._healthy = bool(value)
        message = Bool()
        message.data = self._healthy
        self._health_pub.publish(message)

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

    @staticmethod
    def _switch_lists(request):
        activate = list(request.activate_controllers) + list(request.start_controllers)
        deactivate = list(request.deactivate_controllers) + list(request.stop_controllers)
        return activate, deactivate

    def _on_switch(self, request, response):
        activate, deactivate = self._switch_lists(request)
        if "cartesian_velocity_controller" in activate and (
            self._return_active or self._auto_tension_active or self._hardware_fault_latched
        ):
            response.ok = False
            return response
        if "cartesian_velocity_controller" in deactivate and self._servo_enabled:
            code = self._robot.ServoMoveEnd()
            self._servo_enabled = False
            self._return_active = False
            self._auto_tension_active = False
            if code != 0:
                response.ok = False
                return response
        if "cartesian_velocity_controller" in activate and not self._servo_enabled:
            code = self._robot.ServoMoveStart()
            if code != 0:
                response.ok = False
                return response
            self._servo_enabled = True
            self._last_motion_at = time.monotonic()
            self._last_command_at = time.monotonic()
        response.ok = True
        return response

    def _on_set_zero(self, _request, response):
        if (
            self._servo_enabled
            or self._hardware_fault_latched
            or self._latest_pose is None
            or self._latest_joints is None
        ):
            response.success = False
            response.message = "Zero rejected: stop traction and wait for fresh FR5 feedback."
            return response
        self._zero_pose = list(self._latest_pose)
        self._zero_joints = list(self._latest_joints)
        response.success = True
        response.message = "Current slack pose stored as zero."
        return response

    def _on_return_zero(self, _request, response):
        if (
            self._zero_pose is None
            or self._latest_pose is None
            or self._servo_enabled
            or self._hardware_fault_latched
        ):
            response.success = False
            response.message = "Return rejected: zero is unset or servo motion is busy."
            return response
        distance_mm = math.sqrt(
            sum((self._zero_pose[i] - self._latest_pose[i]) ** 2 for i in range(3))
        )
        return_max_distance_mm = float(
            self.get_parameter("return_max_distance_mm").value
        )
        if not math.isfinite(return_max_distance_mm) or return_max_distance_mm <= 0.0:
            response.success = False
            response.message = "Return rejected: return_max_distance_mm is invalid."
            return response
        if distance_mm > return_max_distance_mm:
            response.success = False
            response.message = (
                "Return rejected: zero is more than "
                f"{return_max_distance_mm:.0f} mm away."
            )
            return response
        code = self._robot.ServoMoveStart()
        if code != 0:
            response.success = False
            response.message = f"ServoMoveStart failed: {code}."
            return response
        self._servo_enabled = True
        self._last_motion_at = time.monotonic()
        self._return_active = True
        self._return_started_at = time.monotonic()
        self._return_start_pose = list(self._latest_pose)
        self._return_duration_s = max(0.5, distance_mm / self._return_speed_mm_s)
        response.success = True
        response.message = "Low-speed return to the stored slack zero has started."
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
        if (
            self._servo_enabled
            or self._hardware_fault_latched
            or self._latest_pose is None
            or self._latest_wrench is None
        ):
            response.success = False
            response.message = "Auto tension rejected: servo is busy or feedback is stale."
            return response
        code = self._robot.ServoMoveStart()
        if code != 0:
            response.success = False
            response.message = f"ServoMoveStart failed: {code}."
            return response
        self._servo_enabled = True
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

    def _servo_cart(self, mode, desc_pos):
        started_at = time.monotonic()
        code = self._robot.ServoCart(mode, desc_pos, cmdT=0.008)
        elapsed = time.monotonic() - started_at
        if elapsed > 0.15:
            self.get_logger().warning(
                f"ServoCart mode {mode} RPC took {elapsed:.3f} s."
            )
        return code

    def _send_motion(self, now, dt):
        if not self._servo_enabled:
            return
        if self._last_motion_at > 0.0 and now - self._last_motion_at < self._motion_period_s:
            return
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
                self._robot.ServoMoveEnd()
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
                    self._robot.ServoMoveEnd()
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
                self._robot.ServoMoveEnd()
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
            alpha = min(1.0, (now - self._return_started_at) / self._return_duration_s)
            target = [
                start + alpha * (zero - start)
                for start, zero in zip(self._return_start_pose, self._zero_pose)
            ]
            code = self._servo_cart(0, target)
            if alpha >= 1.0:
                self._robot.ServoMoveEnd()
                self._servo_enabled = False
                self._return_active = False
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
        increment = [
            self._base_servo_sign * value * motion_dt * 1000.0
            for value in linear
        ]
        code = self._servo_cart(1, increment + [0.0, 0.0, 0.0])
        if code != 0:
            raise RuntimeError(f"ServoCart traction failed: {code}")

    def _tick(self):
        now = time.monotonic()
        dt = now - self._last_tick
        self._last_tick = now
        try:
            joint_result = self._robot.GetActualJointPosDegree()
            pose_result = self._robot.GetActualTCPPose()
            speed_code, speeds = self._robot.GetActualJointSpeedsDegree()
            wrench_code, wrench = self._robot.FT_GetForceTorqueRCS()
            if joint_result[0] or pose_result[0] or speed_code or wrench_code:
                raise RuntimeError("FR5 feedback call returned a non-zero code")
            joints, pose = joint_result[1], pose_result[1]
            self._latest_joints, self._latest_pose = list(joints), list(pose)
            self._latest_wrench = list(wrench)
            if self._auto_set_zero and self._zero_pose is None:
                self._zero_pose, self._zero_joints = list(pose), list(joints)
            self._publish_feedback(self.get_clock().now().to_msg(), joints, speeds, wrench, pose)
            self._send_motion(now, dt)
            self._publish_health(not self._hardware_fault_latched)
        except Exception as error:  # noqa: BLE001  # Hardware faults must latch health loss.
            self.get_logger().error(str(error))
            if self._servo_enabled:
                self._robot.ServoMoveEnd()
            self._servo_enabled = False
            self._return_active = False
            self._auto_tension_active = False
            self._hardware_fault_latched = True
            self._publish_health(False)

    def destroy_node(self):
        if self._servo_enabled:
            self._robot.ServoMoveEnd()
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
