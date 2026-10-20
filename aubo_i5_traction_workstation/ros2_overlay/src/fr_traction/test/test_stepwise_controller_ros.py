"""Local ROS integration tests for the stepwise traction controller wiring."""

from collections import deque
import math
import os
from pathlib import Path
import subprocess
import time

import pytest
import rclpy
from geometry_msgs.msg import Twist, WrenchStamped
from fr_traction.msg import EndEffectorState, PositionControlDiagnostics, TractionCommand
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from std_msgs.msg import Bool


DOMAIN_ID = "174"
BASE_FRAME = "base_link"
WRENCH_TOPIC = "/test/stepwise/wrench"
COMMAND_TOPIC = "/test/stepwise/command"
EE_TOPIC = "/test/stepwise/ee"
VELOCITY_TOPIC = "/test/stepwise/velocity"
DIAGNOSTICS_TOPIC = "/test/stepwise/diagnostics"
HEALTH_TOPIC = "/traction_controller/healthy"
ZERO_EPSILON = 1.0e-9


def _reliable_qos(depth=10, transient_local=False):
    """Return the QoS used by the controller input or health endpoints."""
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        ),
    )


class ControllerProcess:
    """Own one controller subprocess and never signal another process."""

    def __init__(self, verified, extra_parameters=None):
        """Record the verified-mode flag and optional test-only ROS parameters."""
        self.verified = verified
        self.extra_parameters = extra_parameters or []
        self.process = None

    def start(self):
        """Start the parent-supplied controller executable on localhost."""
        executable = os.environ.get("AUBO_TRACTION_CONTROLLER_EXECUTABLE")
        if not executable:
            pytest.skip("AUBO_TRACTION_CONTROLLER_EXECUTABLE is not set")

        arguments = [
            str(Path(executable)),
            "--ros-args",
            "-p",
            "stepwise_backend:=true",
            "-p",
            f"wrench_topic:={WRENCH_TOPIC}",
            "-p",
            f"command_topic:={COMMAND_TOPIC}",
            "-p",
            f"ee_state_topic:={EE_TOPIC}",
            "-p",
            f"cartesian_command_topic:={VELOCITY_TOPIC}",
            "-p",
            f"position_diagnostics_topic:={DIAGNOSTICS_TOPIC}",
        ]
        if self.verified:
            arguments.extend(
                [
                    "-p",
                    "stepwise_force_polarity_verified:=true",
                    "-p",
                    "stepwise_drag_axes_verified:=true",
                    "-p",
                    "stepwise_axis_sign:=1.0",
                ]
            )
        arguments.extend(self.extra_parameters)
        environment = os.environ.copy()
        environment.update(
            {
                "ROS_DOMAIN_ID": DOMAIN_ID,
                "ROS_LOCALHOST_ONLY": "1",
                "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
            }
        )
        self.process = subprocess.Popen(
            arguments,
            env=environment,
            stdout=subprocess.DEVNULL,
            stderr=subprocess.DEVNULL,
        )
        return self

    def alive(self):
        """Return whether this test-owned process is still running."""
        return self.process is not None and self.process.poll() is None

    def stop(self):
        """Terminate only this controller, with bounded cleanup waits."""
        if self.process is None or self.process.poll() is not None:
            return
        self.process.terminate()
        try:
            self.process.wait(timeout=3.0)
        except subprocess.TimeoutExpired:
            self.process.kill()
            self.process.wait(timeout=3.0)


class ControllerHarness:
    """Publish scripted ROS inputs and collect only fresh controller outputs."""

    def __init__(self, runtime, controller):
        """Create the isolated test node around one owned controller process."""
        self.controller = controller
        self.node = Node("stepwise_controller_ros_test", context=runtime)
        input_qos = _reliable_qos()
        output_qos = _reliable_qos()
        health_qos = _reliable_qos(depth=1, transient_local=True)
        self.wrench_publisher = self.node.create_publisher(
            WrenchStamped, WRENCH_TOPIC, input_qos
        )
        self.command_publisher = self.node.create_publisher(
            TractionCommand, COMMAND_TOPIC, input_qos
        )
        self.ee_publisher = self.node.create_publisher(
            EndEffectorState, EE_TOPIC, input_qos
        )
        self.outputs = deque(maxlen=1000)
        self.diagnostics = deque(maxlen=1000)
        self.health = deque(maxlen=1000)
        self.node.create_subscription(
            Twist, VELOCITY_TOPIC, lambda message: self.outputs.append(message), output_qos
        )
        self.node.create_subscription(
            PositionControlDiagnostics,
            DIAGNOSTICS_TOPIC,
            lambda message: self.diagnostics.append(message),
            output_qos,
        )
        self.node.create_subscription(
            Bool,
            HEALTH_TOPIC,
            lambda message: self.health.append(message.data),
            health_qos,
        )
        self.mode = TractionCommand.DISABLED
        self.target_force = 0.0
        self.direction = (1.0, 0.0, 0.0)
        self.force = (0.0, 0.0, 0.0)
        self.return_velocity = (0.0, 0.0, 0.0)
        self.command_frame = BASE_FRAME
        self.wrench_frame = BASE_FRAME
        self.ee_frame = BASE_FRAME
        self.command_offset_s = 0.0
        self.wrench_offset_s = 0.0
        self.ee_offset_s = 0.0
        self.ee_quaternion = (0.0, 0.0, 0.0, 1.0)

    def close(self):
        """Stop the child and release this test node."""
        self.controller.stop()
        self.node.destroy_node()

    def configure(
        self,
        mode,
        target_force,
        force,
        command_offset_s=0.0,
        wrench_offset_s=0.0,
        ee_offset_s=0.0,
        command_frame=BASE_FRAME,
        wrench_frame=BASE_FRAME,
        ee_frame=BASE_FRAME,
        ee_quaternion=(0.0, 0.0, 0.0, 1.0),
    ):
        """Set the next periodic scripted command, feedback, and pose."""
        self.mode = mode
        self.target_force = target_force
        self.force = force
        self.command_offset_s = command_offset_s
        self.wrench_offset_s = wrench_offset_s
        self.ee_offset_s = ee_offset_s
        self.command_frame = command_frame
        self.wrench_frame = wrench_frame
        self.ee_frame = ee_frame
        self.ee_quaternion = ee_quaternion

    def _stamp(self, offset_s):
        total_ns = int(
            self.node.get_clock().now().nanoseconds + offset_s * 1.0e9
        )
        stamp = self.node.get_clock().now().to_msg()
        stamp.sec = total_ns // 1_000_000_000
        stamp.nanosec = total_ns % 1_000_000_000
        return stamp

    def publish_inputs(self):
        """Publish one current set of input messages with controlled headers."""
        command = TractionCommand()
        command.header.frame_id = self.command_frame
        command.header.stamp = self._stamp(self.command_offset_s)
        command.mode = self.mode
        command.locked_direction_base.x = self.direction[0]
        command.locked_direction_base.y = self.direction[1]
        command.locked_direction_base.z = self.direction[2]
        command.target_force_n = self.target_force
        command.direction_correction_mode = TractionCommand.DIRECTION_CORRECTION_OFF
        command.lateral_velocity_base.x = self.return_velocity[0]
        command.lateral_velocity_base.y = self.return_velocity[1]
        command.lateral_velocity_base.z = self.return_velocity[2]
        self.command_publisher.publish(command)

        wrench = WrenchStamped()
        wrench.header.frame_id = self.wrench_frame
        wrench.header.stamp = self._stamp(self.wrench_offset_s)
        wrench.wrench.force.x = self.force[0]
        wrench.wrench.force.y = self.force[1]
        wrench.wrench.force.z = self.force[2]
        self.wrench_publisher.publish(wrench)

        ee_state = EndEffectorState()
        ee_state.header.frame_id = self.ee_frame
        ee_state.header.stamp = self._stamp(self.ee_offset_s)
        ee_state.pose.orientation.x = self.ee_quaternion[0]
        ee_state.pose.orientation.y = self.ee_quaternion[1]
        ee_state.pose.orientation.z = self.ee_quaternion[2]
        ee_state.pose.orientation.w = self.ee_quaternion[3]
        self.ee_publisher.publish(ee_state)

    def wait_until(self, predicate, timeout_s=3.0):
        """Pump bounded ROS work until predicate succeeds or timeout expires."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            if not self.controller.alive():
                raise AssertionError("controller subprocess exited during test")
            self.publish_inputs()
            remaining = max(0.0, deadline - time.monotonic())
            rclpy.spin_once(self.node, timeout_sec=min(0.02, remaining))
            if predicate():
                return
        raise AssertionError("timed out waiting for fresh controller state")

    def mark(self):
        """Return collection sizes used to exclude startup output."""
        return len(self.outputs), len(self.diagnostics), len(self.health)

    def wait_for_stable_output(self, mark, predicate, timeout_s=3.0, samples=3):
        """Require several newest outputs after mark to satisfy predicate."""
        def ready():
            recent = list(self.outputs)[mark[0]:]
            return len(recent) >= samples and all(predicate(item) for item in recent[-samples:])

        self.wait_until(ready, timeout_s)

    def wait_for_stable_health(self, mark, expected, timeout_s=3.0, samples=3):
        """Require several newest health samples after mark to match expected."""
        def ready():
            recent = list(self.health)[mark[2]:]
            return len(recent) >= samples and recent[-samples:] == [expected] * samples

        self.wait_until(ready, timeout_s)

    def wait_for_position_settling(self, mark, timeout_s=3.0):
        """Require fresh zero velocity and SETTLING diagnostics together."""
        def ready():
            outputs = list(self.outputs)[mark[0]:]
            diagnostics = list(self.diagnostics)[mark[1]:]

            def zero(item):
                return _norm(item) <= ZERO_EPSILON
            return (
                len(outputs) >= 3
                and len(diagnostics) >= 3
                and all(zero(item) for item in outputs[-3:])
                and all(
                    item.phase == PositionControlDiagnostics.SETTLING
                    for item in diagnostics[-3:]
                )
            )

        self.wait_until(ready, timeout_s)


def _norm(twist):
    """Return the linear speed of a Twist message."""
    return math.sqrt(
        twist.linear.x * twist.linear.x
        + twist.linear.y * twist.linear.y
        + twist.linear.z * twist.linear.z
    )


@pytest.fixture(scope="session")
def ros_runtime():
    """Initialize one isolated rclpy context for the localhost tests."""
    os.environ.update({
        "ROS_DOMAIN_ID": DOMAIN_ID,
        "ROS_LOCALHOST_ONLY": "1",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
    })
    if not rclpy.ok():
        rclpy.init(args=None)
    yield rclpy.get_default_context()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def controller(ros_runtime):
    """Run one verified stepwise controller and clean it up boundedly."""
    child = ControllerProcess(verified=True).start()
    harness = ControllerHarness(ros_runtime, child)
    try:
        harness.wait_until(lambda: bool(harness.health), timeout_s=5.0)
        yield harness
    finally:
        harness.close()


@pytest.fixture
def unverified_controller(ros_runtime):
    """Run one controller without calibration-verification launch flags."""
    child = ControllerProcess(verified=False).start()
    harness = ControllerHarness(ros_runtime, child)
    try:
        harness.wait_until(lambda: bool(harness.health), timeout_s=5.0)
        yield harness
    finally:
        harness.close()


@pytest.fixture
def xy_drag_controller(ros_runtime):
    """Run a verified controller that explicitly permits only XY drag motion."""
    child = ControllerProcess(
        verified=True,
        extra_parameters=["-p", "stepwise_drag_axis_mask:=[1.0, 1.0, 0.0]"],
    ).start()
    harness = ControllerHarness(ros_runtime, child)
    try:
        harness.wait_until(lambda: bool(harness.health), timeout_s=5.0)
        yield harness
    finally:
        harness.close()


@pytest.mark.parametrize(
    ("force", "expected_sign"),
    [((3.0, 0.0, 0.0), 1.0), ((7.0, 0.0, 0.0), -1.0)],
)
def test_traction_force_error_commands_bounded_signed_velocity(controller, force, expected_sign):
    """TRACTION target 5 N commands signed speed no larger than 2 mm/s."""
    mark = controller.mark()
    controller.configure(TractionCommand.TRACTION, 5.0, force)
    controller.wait_for_stable_output(
        mark,
        lambda message: (
            expected_sign * message.linear.x > 1.0e-5
            and abs(message.linear.x) <= 0.002000001
            and abs(message.linear.y) <= ZERO_EPSILON
            and abs(message.linear.z) <= ZERO_EPSILON
        ),
        timeout_s=3.0,
    )
    controller.wait_for_stable_health(mark, True)


def test_positioning_reaches_settling_without_virtual_momentum(controller):
    """POSITIONING at measured target force settles at a zero Cartesian command."""
    mark = controller.mark()
    controller.configure(TractionCommand.POSITIONING, 5.0, (5.0, 0.0, 0.0))
    controller.wait_for_position_settling(mark, timeout_s=3.0)
    controller.wait_for_stable_health(mark, True)


def test_low_target_positioning_does_not_settle_when_unloaded(controller):
    """A 1 N target must not accept the unloaded 0 N pose as reached."""
    mark = controller.mark()
    controller.configure(TractionCommand.POSITIONING, 1.0, (0.0, 0.0, 0.0))
    controller.wait_for_stable_output(mark, lambda message: message.linear.x > 0.001)


def test_geometric_return_uses_velocity_with_correction_off(unverified_controller):
    """RELEASING return velocity is geometric, not force-axis calibration."""
    mark = unverified_controller.mark()
    unverified_controller.configure(TractionCommand.RELEASING, 0.0, (0.0, 0.0, 0.0))
    unverified_controller.return_velocity = (0.0015, -0.0005, 0.0)
    unverified_controller.wait_for_stable_output(
        mark, lambda message: message.linear.x > 0.001 and message.linear.y < 0.0)


def test_drag_axes_follow_identity_pose_and_let_go_returns_zero(controller):
    """DRAG follows XYZ signs through identity pose and releases to zero."""
    mark = controller.mark()
    controller.configure(TractionCommand.DRAG, 0.0, (2.0, -3.0, 4.0))
    controller.wait_for_stable_output(
        mark,
        lambda message: (
            message.linear.x > 0.0
            and message.linear.y < 0.0
            and message.linear.z > 0.0
            and _norm(message) <= 0.002000001
        ),
        timeout_s=3.0,
    )

    released = controller.mark()
    controller.configure(TractionCommand.DRAG, 0.0, (0.0, 0.0, 0.0))
    controller.wait_for_stable_output(
        released, lambda message: _norm(message) <= ZERO_EPSILON, timeout_s=3.0
    )


@pytest.mark.parametrize(("gain", "minimum", "maximum", "force_ceiling"), [
    (0.003, 0.0012, 0.005, 0.002), (0.010, 0.005, 0.020, 0.002),
    (0.080, 0.005, 0.050, 0.020),
])
def test_responsive_drag_parameters_reach_all_axes_without_retuning_force_modes(
    ros_runtime, gain, minimum, maximum, force_ceiling,
):
    """Exercise real ROS parameter wiring, not only the pure decision policy."""
    child = ControllerProcess(verified=True, extra_parameters=[
        "-p", "stepwise_drag_deadband_n:=0.5",
        "-p", f"stepwise_drag_gain_mps_per_n:={gain}",
        "-p", f"stepwise_drag_minimum_speed_mps:={minimum}",
        "-p", f"stepwise_drag_maximum_speed_mps:={maximum}",
        "-p", f"stepwise_maximum_speed_mps:={force_ceiling}",
    ]).start()
    harness = ControllerHarness(ros_runtime, child)
    try:
        harness.wait_until(lambda: bool(harness.health), timeout_s=5.0)
        for force in ((0.8, 0.0, 0.0), (0.0, 2.0, 0.0), (0.0, 0.0, 3.0)):
            mark = harness.mark()
            harness.configure(TractionCommand.DRAG, 0.0, force)
            expected = min(maximum, max(minimum, gain * (math.sqrt(
                sum(value * value for value in force)) - 0.5)))
            harness.wait_for_stable_output(
                mark, lambda message: abs(_norm(message) - expected) < 0.00001)
        mark = harness.mark()
        harness.configure(TractionCommand.DRAG, 0.0, (0.0, 0.0, 0.0))
        harness.wait_for_stable_output(mark, lambda message: _norm(message) <= ZERO_EPSILON)
        for mode in (TractionCommand.TRACTION, TractionCommand.POSITIONING):
            mark = harness.mark()
            harness.configure(mode, 20.0, (0.0, 0.0, 0.0))
            harness.wait_for_stable_output(
                mark, lambda message: 0.001 < _norm(message) <= force_ceiling + 1e-9)
    finally:
        harness.close()


def _reference_rotate_force(force, quaternion):
    """Independent cross-product rotation for offline test expectations."""
    magnitude = math.sqrt(sum(value * value for value in quaternion))
    x, y, z, w = (value / magnitude for value in quaternion)
    fx, fy, fz = force
    tx, ty, tz = 2 * (y * fz - z * fy), 2 * (z * fx - x * fz), 2 * (x * fy - y * fx)
    return (fx + w * tx + y * tz - z * ty,
            fy + w * ty + z * tx - x * tz,
            fz + w * tz + x * ty - y * tx)


@pytest.mark.parametrize('quaternion', [
    (0.0, 0.0, 0.0, 1.0),
    (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5)),
    (-0.3317575632, -0.5098593730, 0.7853675603, 0.1147960553),
])
def test_full_xyz_drag_and_release_through_rotated_tool_pose(controller, quaternion):
    """Six signed axes and diagonals survive tool/base rotation without XY projection."""
    for force_tool in ((3, 0, 0), (-3, 0, 0), (0, 3, 0), (0, -3, 0),
                       (0, 0, 3), (0, 0, -3), (2, -3, 4), (-2, 3, -4)):
        force_base = _reference_rotate_force(force_tool, quaternion)
        force_norm = math.sqrt(sum(value * value for value in force_base))
        mark = controller.mark()
        controller.configure(TractionCommand.DRAG, 0.0, force_base, ee_quaternion=quaternion)

        def follows_full_force(message):
            speed = _norm(message)
            velocity = (message.linear.x, message.linear.y, message.linear.z)
            alignment = sum(v * f for v, f in zip(velocity, force_base))
            return (0.001 <= speed <= 0.002000001 and
                    alignment / (speed * force_norm) > 0.9999)

        controller.wait_for_stable_output(mark, follows_full_force)
        controller.wait_for_stable_health(mark, True)
        released = controller.mark()
        controller.configure(TractionCommand.DRAG, 0.0, (0.0, 0.0, 0.0), ee_quaternion=quaternion)
        controller.wait_for_stable_output(released, lambda message: _norm(message) <= ZERO_EPSILON)


def test_drag_axis_mask_blocks_z_and_preserves_xy_response(xy_drag_controller):
    """An explicit XY mask rejects Z force while retaining XY drag response."""
    masked_z = xy_drag_controller.mark()
    xy_drag_controller.configure(TractionCommand.DRAG, 0.0, (0.0, 0.0, 4.0))
    xy_drag_controller.wait_for_stable_output(
        masked_z, lambda message: _norm(message) <= ZERO_EPSILON, timeout_s=3.0
    )

    xy_force = xy_drag_controller.mark()
    xy_drag_controller.configure(TractionCommand.DRAG, 0.0, (2.0, -3.0, 4.0))
    xy_drag_controller.wait_for_stable_output(
        xy_force,
        lambda message: (
            message.linear.x > 0.0
            and message.linear.y < 0.0
            and abs(message.linear.z) <= ZERO_EPSILON
            and _norm(message) <= 0.002000001
        ),
        timeout_s=3.0,
    )


@pytest.mark.parametrize(
    "invalid_stream",
    [
        "stale_force",
        "future_force",
        "wrong_frame_force",
        "stale_command",
        "zero_ee_orientation",
        "stale_ee",
    ],
)
def test_invalid_or_stale_stepwise_inputs_never_produce_velocity(controller, invalid_stream):
    """Repeated invalid headers or poses force a fresh zero output."""
    active = controller.mark()
    controller.configure(TractionCommand.TRACTION, 5.0, (3.0, 0.0, 0.0))
    controller.wait_for_stable_output(
        active,
        lambda message: message.linear.x > 1.0e-5,
        timeout_s=3.0,
    )

    mark = controller.mark()
    if invalid_stream == "stale_force":
        controller.configure(
            TractionCommand.TRACTION,
            5.0,
            (3.0, 0.0, 0.0),
            wrench_offset_s=-1.0,
        )
    elif invalid_stream == "future_force":
        controller.configure(
            TractionCommand.TRACTION,
            5.0,
            (3.0, 0.0, 0.0),
            wrench_offset_s=1.0,
        )
    elif invalid_stream == "wrong_frame_force":
        controller.configure(
            TractionCommand.TRACTION,
            5.0,
            (3.0, 0.0, 0.0),
            wrench_frame="wrong_frame",
        )
    elif invalid_stream == "stale_command":
        controller.configure(
            TractionCommand.TRACTION,
            5.0,
            (3.0, 0.0, 0.0),
            command_offset_s=-1.0,
        )
    elif invalid_stream == "zero_ee_orientation":
        controller.configure(
            TractionCommand.TRACTION,
            5.0,
            (3.0, 0.0, 0.0),
            ee_quaternion=(0.0, 0.0, 0.0, 0.0),
        )
    else:
        controller.configure(
            TractionCommand.TRACTION,
            5.0,
            (3.0, 0.0, 0.0),
            ee_offset_s=-1.0,
        )

    controller.wait_for_stable_output(
        mark, lambda message: _norm(message) <= ZERO_EPSILON, timeout_s=3.0
    )
    controller.wait_for_stable_health(
        mark, invalid_stream not in {"zero_ee_orientation", "stale_ee"}
    )


def test_unverified_calibration_keeps_active_mode_zero_and_unhealthy(unverified_controller):
    """Without calibration flags an active stepwise mode remains fail-closed."""
    mark = unverified_controller.mark()
    unverified_controller.configure(
        TractionCommand.TRACTION, 5.0, (3.0, 0.0, 0.0)
    )
    unverified_controller.wait_for_stable_output(
        mark, lambda message: _norm(message) <= ZERO_EPSILON, timeout_s=3.0
    )
    unverified_controller.wait_for_stable_health(mark, False, timeout_s=3.0)
