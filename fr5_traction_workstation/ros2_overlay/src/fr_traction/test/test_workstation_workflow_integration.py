"""
Run the real manager/controller against fake feedback, never a robot driver.

Source ROS, the fairino_msgs install and the newly built fr_traction install,
then run with FR5_TEST_INSTALL_PREFIX=/tmp/fr5-workflow-install pytest -q <file>.
The explicit install prefix opts in; every participant is forced into local
ROS domain 89 before creating a context or subprocess. No SDK is imported.
"""

import math
import os
import subprocess
import threading
import time
from collections import deque
from pathlib import Path

import pytest
import rclpy
from controller_manager_msgs.srv import SwitchController
from fairino_msgs.msg import PoseTwist
from fr_traction.msg import TractionCommand, TractionStatus
from fr_traction.srv import SetOperationMode, SetTargetForce
from geometry_msgs.msg import Twist, WrenchStamped
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import SetParametersAtomically
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger

# Imports do not initialize DDS. Force isolation before any context or child.
os.environ["ROS_DOMAIN_ID"] = "89"
os.environ["ROS_LOCALHOST_ONLY"] = "1"

pytestmark = pytest.mark.skipif(
    not os.environ.get("FR5_TEST_INSTALL_PREFIX"),
    reason="Set FR5_TEST_INSTALL_PREFIX to explicitly enable isolated ROS integration.",
)


def vector(message):
    return (message.x, message.y, message.z)


def close_direction(actual, expected, tolerance=0.02):
    return math.dist(actual, expected) < tolerance


class WorkflowHarness:
    """Publish only synthetic feedback and serve fake hardware handoffs."""

    def __init__(self, directory):
        assert os.environ["ROS_DOMAIN_ID"] == "89"
        assert os.environ["ROS_LOCALHOST_ONLY"] == "1"
        prefix = Path(os.environ["FR5_TEST_INSTALL_PREFIX"]).resolve()
        candidates = (prefix / "fr_traction/lib/fr_traction", prefix / "lib/fr_traction")
        self.binary_dir = next(
            (path for path in candidates if (path / "traction_manager_node").is_file()), None
        )
        assert self.binary_dir is not None, f"No manager binary in {prefix}"
        assert hasattr(TractionCommand, "DIRECTION_SEARCH"), "Source the new message install first"
        self.directory = directory
        self.context = Context()
        rclpy.init(context=self.context)
        self.node = rclpy.create_node("fake_workstation_feedback", context=self.context)
        self.executor = MultiThreadedExecutor(num_threads=3, context=self.context)
        self.executor.add_node(self.node)
        self.processes = []
        self.logs = []
        self.status = None
        self.statuses = deque(maxlen=3000)
        self.status_times = deque(maxlen=3000)
        self.commands = deque(maxlen=3000)
        self.twists = deque(maxlen=3000)
        self.calibration_events = 0
        self.switch_calls = []
        self.native_calls = []
        self.native_stop_delay_s = 0.0
        self.native_stop_entered = threading.Event()
        self.wrench_paused = False
        self.allow_fault = False
        self.offset = (12.0, -3.0, 6.0)
        self.force = (0.0, 0.0, 0.0)
        self.position = (0.4, 0.2, 0.3)
        self.orientation = (0.0, 0.0, 0.0, 1.0)
        self.wrench_pub = self.node.create_publisher(WrenchStamped, "/test/raw_wrench", 10)
        self.teach_exit_pub = self.node.create_publisher(WrenchStamped, "/controller_manager/teach_drag_exit_wrench", 10)
        self.pose_pub = self.node.create_publisher(PoseTwist, "/test/ee_state", 10)
        self.joint_pub = self.node.create_publisher(JointState, "/test/joint_states", 10)
        health_qos = QoSProfile(
            depth=1, reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.health_pub = self.node.create_publisher(Bool, "/test/hardware_healthy", health_qos)
        self.physical_teach_pub = self.node.create_publisher(
            Bool, "/controller_manager/physical_teach_drag_active", health_qos)
        self.node.create_subscription(TractionStatus, "/traction/status", self.on_status, 10)
        self.node.create_subscription(
            TractionCommand, "/traction/command",
            lambda message: self.commands.append((time.monotonic(), message)), 10,
        )
        self.node.create_subscription(
            Twist, "/test/command_cart_vel",
            lambda message: self.twists.append((time.monotonic(), message)), 10,
        )
        self.node.create_subscription(
            Bool, "/traction/slack_calibration", self.on_calibration, health_qos,
        )
        self.node.create_service(SwitchController, "/test/switch_controller", self.on_switch)
        self.node.create_service(
            Trigger, "/test/native_drag_start",
            lambda request, response: self.on_native("start", request, response),
        )
        self.node.create_service(
            Trigger, "/test/native_drag_stop",
            lambda request, response: self.on_native("stop", request, response),
            callback_group=ReentrantCallbackGroup(),
        )
        self.node.create_service(
            Trigger, "/test/native_drag_gate_stop",
            lambda request, response: self.on_native("stop", request, response),
        )
        self.node.create_timer(0.01, self.publish_feedback)
        self.clients = {
            name: self.node.create_client(Trigger, f"/traction/{name}")
            for name in ("start", "stop", "unload", "prepare", "reset_transient_fault", "emergency_stop")
        }
        self.mode_client = self.node.create_client(
            SetOperationMode, "/traction/set_operation_mode"
        )
        self.target_client = self.node.create_client(SetTargetForce, "/traction/set_target_force")
        self.parameters_client = self.node.create_client(
            SetParametersAtomically, "/traction_manager/set_parameters_atomically"
        )
        manager_parameters = {
            "wrench_topic": "/test/raw_wrench",
            "ee_state_topic": "/test/ee_state",
            "joint_state_topic": "/test/joint_states",
            "hardware_health_topic": "/test/hardware_healthy",
            "controller_manager_switch_service": "/test/switch_controller",
            "native_drag_start_service": "/test/native_drag_start",
            "native_drag_stop_service": "/test/native_drag_stop",
            "native_drag_gate_stop_service": "/test/native_drag_gate_stop",
            "data_directory": str(directory / "sessions"),
            "require_ui_heartbeat": "false",
            # Match formal hardware-launch budgets rather than the class's
            # legacy 100 ms wrench timeout under WSL scheduler jitter.
            "wrench_timeout_s": 1.2,
            "ee_state_timeout_s": 1.2,
            "controller_health_timeout_s": 1.2,
            "motion_pause_timeout_s": 0.25,
        }
        controller_parameters = {
            "wrench_topic": "/traction/corrected_wrench",
            "ee_state_topic": "/test/ee_state",
            "cartesian_command_topic": "/test/command_cart_vel",
            "wrench_timeout_s": 0.25,
            "command_timeout_s": 0.25,
        }
        self.launch("traction_controller_node", controller_parameters)
        self.launch("traction_manager_node", manager_parameters)

    def launch(self, executable, parameters):
        command = [str(self.binary_dir / executable), "--ros-args"]
        for name, value in parameters.items():
            command.extend(["-p", f"{name}:={value}"])
        log = (self.directory / f"{executable}.log").open("w+", encoding="utf-8")
        self.logs.append(log)
        environment = dict(os.environ, ROS_DOMAIN_ID="89", ROS_LOCALHOST_ONLY="1")
        self.processes.append(subprocess.Popen(
            command, env=environment, stdout=log, stderr=subprocess.STDOUT,
        ))

    def on_status(self, message):
        self.status = message
        self.statuses.append(message)
        self.status_times.append((time.monotonic(), message))

    def on_calibration(self, message):
        self.calibration_events += int(message.data)

    def on_switch(self, request, response):
        self.switch_calls.append(request)
        response.ok = True
        return response

    def on_native(self, action, _request, response):
        self.native_calls.append(action)
        if action == "stop" and self.native_stop_delay_s > 0.0:
            self.native_stop_entered.set()
            time.sleep(self.native_stop_delay_s)
        response.success = True
        response.message = "Fake controller handoff accepted"
        return response

    def publish_feedback(self):
        stamp = self.node.get_clock().now().to_msg()
        wrench = WrenchStamped()
        wrench.header.stamp = stamp
        wrench.header.frame_id = "base_link"
        wrench.wrench.force.x, wrench.wrench.force.y, wrench.wrench.force.z = (
            offset + force for offset, force in zip(self.offset, self.force)
        )
        if not self.wrench_paused:
            self.wrench_pub.publish(wrench)
        pose = PoseTwist()
        pose.header.stamp = stamp
        pose.header.frame_id = "base_link"
        pose.pose.position.x, pose.pose.position.y, pose.pose.position.z = self.position
        (pose.pose.orientation.x, pose.pose.orientation.y,
         pose.pose.orientation.z, pose.pose.orientation.w) = self.orientation
        self.pose_pub.publish(pose)
        joints = JointState()
        joints.header.stamp = stamp
        joints.name = [f"j{index}" for index in range(1, 7)]
        joints.position = [0.0] * 6
        joints.velocity = [0.0] * 6
        self.joint_pub.publish(joints)
        self.health_pub.publish(Bool(data=True))

    def diagnostics(self):
        status = self.status
        detail = "No status" if status is None else (
            f"state={status.state} calibrated={status.calibrated} "
            f"force={status.actual_force_n} fault={status.fault_code} "
            f"reason={status.stop_reason} message={status.message}"
        )
        for log in self.logs:
            log.flush()
            detail += f"\n{Path(log.name).name}:\n{Path(log.name).read_text()[-3000:]}"
        return detail

    def pump(self, duration):
        end = time.monotonic() + duration
        while time.monotonic() < end:
            assert all(process.poll() is None for process in self.processes), self.diagnostics()
            self.executor.spin_once(timeout_sec=0.005)
            assert self.allow_fault or self.status is None or self.status.state not in (
                TractionStatus.FAULT, TractionStatus.EMERGENCY_STOP
            ), self.diagnostics()

    def wait(self, predicate, timeout=5.0):
        end = time.monotonic() + timeout
        while time.monotonic() < end:
            if predicate():
                return
            self.pump(0.01)
        raise AssertionError(self.diagnostics())

    def call(self, name):
        client = self.clients[name]
        response = self.request(client, Trigger.Request())
        assert response.success, response.message + "\n" + self.diagnostics()
        return response

    def request(self, client, request):
        self.wait(client.service_is_ready)
        future = client.call_async(request)
        self.wait(future.done)
        return future.result()

    def mode(self, mode):
        request = SetOperationMode.Request()
        request.mode = mode
        response = self.request(self.mode_client, request)
        assert response.success, response.message
        self.wait(lambda: self.status is not None and self.status.operation_mode == mode)

    def ready(self):
        self.wait(lambda: self.status is not None and self.status.calibrated)
        self.wait(lambda: self.status.state == TractionStatus.READY)
        self.pump(0.1)

    def close(self):
        for process in self.processes:
            if process.poll() is None:
                process.terminate()
        for process in self.processes:
            try:
                process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                process.kill()
                process.wait(timeout=3.0)
        self.executor.remove_node(self.node)
        self.node.destroy_node()
        self.executor.shutdown()
        self.context.shutdown()
        for log in self.logs:
            log.close()


@pytest.fixture
def workstation(tmp_path):
    harness = WorkflowHarness(tmp_path)
    try:
        harness.ready()
        yield harness
    finally:
        harness.close()


def test_automatic_calibration_direction_updates_lock_and_manual_recalibration(workstation):
    robot = workstation
    assert robot.calibration_events == 1
    assert robot.status.actual_force_n < 0.01
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (8.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.direction_available, timeout=3.0)
    assert close_direction(vector(robot.status.locked_direction_base), (1.0, 0.0, 0.0))
    robot.force = (0.0, 8.0, 0.0)
    robot.pump(0.2)
    assert close_direction(vector(robot.status.locked_direction_base), (1.0, 0.0, 0.0))
    robot.wait(
        lambda: close_direction(vector(robot.status.locked_direction_base), (0.0, 1.0, 0.0)),
        timeout=4.5,
    )
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    locked = vector(robot.status.locked_direction_base)
    robot.force = (200.0, 0.0, 0.0)
    robot.pump(2.3)
    assert robot.status.direction_locked
    assert close_direction(vector(robot.status.locked_direction_base), locked)
    assert robot.status.actual_force_n > 190.0
    robot.call("stop")
    robot.ready()
    robot.mode(SetOperationMode.Request.ASSISTED_DRAG)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.DRAGGING)
    robot.call("stop")
    robot.ready()
    assert robot.calibration_events == 1
    assert robot.status.actual_force_n > 190.0
    robot.force = (0.0, 0.0, 0.0)
    robot.pump(0.2)
    robot.call("prepare")
    robot.wait(lambda: robot.calibration_events == 2)
    robot.wait(lambda: not robot.status.direction_available and not robot.status.direction_locked)
    assert not robot.status.direction_available
    assert not robot.status.direction_locked


def test_unload_can_interrupt_target_motion_and_stops_below_five_newtons(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (20.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n > 19.0)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    robot.call("unload")
    robot.wait(lambda: robot.status.state == TractionStatus.RELEASING)
    robot.wait(lambda: any(
        command.mode == TractionCommand.UNLOADING for _, command in list(robot.commands)
    ))
    robot.wait(lambda: bool(robot.twists) and robot.twists[-1][1].linear.x < -0.005)
    assert abs(robot.twists[-1][1].linear.y) < 1e-6
    assert robot.status.direction_locked
    robot.force = (4.5, 0.0, 0.0)
    robot.wait(lambda: robot.status.state == TractionStatus.READY)
    assert robot.status.stop_reason == "UNLOAD_BELOW_5N"
    assert robot.status.calibrated and robot.calibration_events == 1
    robot.pump(0.1)
    assert math.hypot(*vector(robot.twists[-1][1].linear)) < 1e-9


def test_unload_below_five_newtons_also_ends_active_target_traction(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (20.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n > 19.0)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    robot.force = (4.5, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n < 5.0)
    robot.call("unload")
    robot.wait(lambda: robot.status.state == TractionStatus.READY)
    assert robot.status.stop_reason == "UNLOAD_BELOW_5N"
    finished_at = time.monotonic()
    robot.pump(0.25)
    assert not any(
        stamp > finished_at and command.mode == TractionCommand.POSITIONING
        for stamp, command in list(robot.commands)
    )
    assert math.hypot(*vector(robot.twists[-1][1].linear)) < 1e-9


def test_stop_cancels_delayed_native_drag_to_unload_handoff(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (20.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.direction_available, timeout=3.0)
    robot.mode(SetOperationMode.Request.ASSISTED_DRAG)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.DRAGGING)
    robot.wait(lambda: robot.native_calls == ["start"])
    robot.native_stop_delay_s = 0.7
    robot.call("unload")
    robot.wait(robot.native_stop_entered.is_set)
    robot.call("stop")
    cancelled_at = time.monotonic()
    robot.wait(lambda: robot.status.state == TractionStatus.READY)
    assert robot.status.stop_reason == "UNLOAD_CANCELLED"
    robot.pump(0.3)
    assert not any(
        stamp > cancelled_at and command.mode == TractionCommand.UNLOADING
        for stamp, command in list(robot.commands)
    )
    assert not robot.switch_calls
    assert robot.native_calls == ["start", "stop"]


def test_target_limit_can_be_raised_and_removed_constant_force_mode_is_rejected(workstation):
    robot = workstation
    response = robot.request(robot.mode_client, SetOperationMode.Request(mode=0))
    assert not response.success
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    assert robot.status.target_force_max_n == 100.0
    response = robot.request(robot.target_client, SetTargetForce.Request(target_force_n=101.0))
    assert not response.success
    request = SetParametersAtomically.Request()
    request.parameters = [Parameter(
        name="target_force_max_n",
        value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE, double_value=200.0),
    )]
    response = robot.request(robot.parameters_client, request)
    assert response.result.successful, response.result.reason
    robot.wait(lambda: robot.status.target_force_max_n == 200.0)
    response = robot.request(robot.target_client, SetTargetForce.Request(target_force_n=180.0))
    assert response.success, response.message
    robot.force = (20.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n > 19.0)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    assert robot.status.target_force_n == 180.0
    robot.force = (200.0, 0.0, 0.0)
    robot.pump(0.4)
    assert robot.status.actual_force_n > 190.0
    assert not robot.status.fault_code


@pytest.mark.parametrize("settled_force_n", [7.6, 12.4])
def test_target_arrival_requires_one_and_half_seconds_inside_outer_window(workstation, settled_force_n):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (7.4, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n > 7.3)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    robot.pump(2.2)
    assert robot.status.state == TractionStatus.TRACTION
    robot.force = (settled_force_n, 0.0, 0.0)
    robot.wait(lambda: abs(robot.status.actual_force_n - settled_force_n) < 0.05)
    robot.pump(1.0)
    assert robot.status.state == TractionStatus.TRACTION
    robot.wait(lambda: robot.status.state == TractionStatus.POSITION_HOLD, timeout=1.5)
    entered_at = next(t for t, status in robot.status_times
                      if status.state == TractionStatus.TRACTION
                      and abs(status.actual_force_n - 10.0) <= 2.5)
    reached_at = next(t for t, status in robot.status_times
                      if status.state == TractionStatus.POSITION_HOLD)
    assert 1.3 <= reached_at - entered_at <= 1.8
    assert robot.status.position_reached
    assert robot.status.direction_locked
    assert abs(robot.status.actual_force_n - 10.0) <= 2.5
    robot.call("stop")
    robot.ready()
    assert not robot.status.direction_locked
    assert robot.calibration_events == 1


def test_target_arrival_restarts_confirmation_after_leaving_band(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (8.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n > 7.9)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    robot.pump(1.2)
    robot.force = (6.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n < 7.0)
    robot.force = (8.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n > 7.9)
    robot.pump(1.2)
    assert robot.status.state == TractionStatus.TRACTION
    robot.wait(lambda: robot.status.state == TractionStatus.POSITION_HOLD, timeout=1.5)
    assert robot.status.position_reached


def test_automatic_tool_z_search_stops_at_three_newtons_and_locks_target_direction(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    assert not robot.status.direction_available
    # A rotated tool catches an accidental use of base Z instead of Tool Z.
    robot.orientation = (0.0, math.sqrt(0.5), 0.0, math.sqrt(0.5))
    robot.pump(0.1)
    robot.call("start")
    robot.wait(lambda: bool(robot.commands) and (
        robot.commands[-1][1].mode == TractionCommand.DIRECTION_SEARCH
    ))
    command = robot.commands[-1][1]
    assert close_direction(vector(command.locked_direction_base), (-1.0, 0.0, 0.0))
    robot.wait(lambda: bool(robot.twists) and robot.twists[-1][1].linear.x < -0.005)
    robot.force = (0.0, 2.9, 0.0)
    robot.pump(0.15)
    assert robot.status.state == TractionStatus.PRETENSION
    robot.force = (0.0, 3.1, 0.0)
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    assert robot.status.direction_locked
    assert close_direction(vector(robot.status.locked_direction_base), (0.0, 1.0, 0.0))
    robot.force = (8.0, 0.0, 0.0)
    robot.pump(0.3)
    assert close_direction(vector(robot.status.locked_direction_base), (0.0, 1.0, 0.0))


@pytest.mark.parametrize("stop_by_travel", [False, True])
def test_empty_direction_search_has_one_attempt_and_never_faults(workstation, stop_by_travel):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.PRETENSION)
    robot.wait(lambda: any(
        command.mode == TractionCommand.DIRECTION_SEARCH
        for _, command in list(robot.commands)
    ))
    if stop_by_travel:
        robot.position = (0.4, 0.2, -0.001)
    robot.wait(lambda: robot.status.state == TractionStatus.READY, timeout=6.0)
    assert robot.status.stop_reason == "DIRECTION_SEARCH_NO_FORCE"
    assert not robot.status.direction_available
    assert robot.status.calibrated
    if not stop_by_travel:
        commands = list(robot.commands)
        distance = sum(
            (next_stamp - stamp) * command.linear_speed_mps
            for (stamp, command), (next_stamp, _) in zip(commands, commands[1:])
            if command.mode == TractionCommand.DIRECTION_SEARCH
        )
        assert abs(distance - 0.30) < 0.01
        assert max(command.linear_speed_mps for _, command in commands) <= 0.113
    finished_at = time.monotonic()
    robot.pump(2.2)
    assert not any(
        timestamp > finished_at and command.mode == TractionCommand.DIRECTION_SEARCH
        for timestamp, command in list(robot.commands)
    )
    assert robot.status.state == TractionStatus.READY
    assert robot.calibration_events == 1


def test_transient_search_contact_resumes_without_restarting_five_second_budget(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.call("start")
    robot.wait(lambda: any(
        command.mode == TractionCommand.DIRECTION_SEARCH
        for _, command in list(robot.commands)
    ))
    started_at = next(
        stamp for stamp, command in list(robot.commands)
        if command.mode == TractionCommand.DIRECTION_SEARCH
    )
    robot.pump(0.45)
    robot.force = (3.2, 0.0, 0.0)
    robot.pump(0.07)
    robot.force = (0.8, 0.0, 0.0)
    spike_ended_at = time.monotonic()
    robot.wait(lambda: any(
        stamp > spike_ended_at + 0.20 and command.mode == TractionCommand.DIRECTION_SEARCH
        for stamp, command in list(robot.commands)
    ), timeout=2.0)
    assert not robot.status.direction_locked
    robot.wait(lambda: robot.status.state == TractionStatus.READY, timeout=5.5)
    assert robot.status.stop_reason == "DIRECTION_SEARCH_NO_FORCE"
    assert 4.8 < time.monotonic() - started_at < 5.6
    assert len(robot.switch_calls) == 2  # One activation and one final stop.


def test_native_drag_requires_display_resultant_above_five_and_stops_at_five(workstation):
    robot = workstation
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.DRAGGING)
    robot.force = (2.0, 3.0, 0.0)
    robot.pump(0.35)
    assert robot.native_calls == []
    robot.force = (5.0, 0.0, 0.0)
    robot.pump(0.35)
    assert robot.native_calls == []
    robot.force = (3.6, 4.8, 0.0)  # total 6 N, each component below 5 N
    robot.wait(lambda: robot.native_calls == ["start"])
    robot.force = (5.0, 0.0, 0.0)
    robot.wait(lambda: robot.native_calls == ["start", "stop"])
    assert robot.status.state == TractionStatus.DRAGGING
    assert not robot.status.fault_code
    robot.pump(0.25)
    assert robot.native_calls == ["start", "stop"]
    robot.force = (0.0, 6.0, 0.0)
    robot.wait(lambda: robot.native_calls == ["start", "stop", "start"])
    robot.call("stop")
    robot.ready()
    assert robot.native_calls[-1] == "stop"
    assert robot.calibration_events == 1


def test_physical_teach_exit_recalibrates_loaded_force_once(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (9.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.direction_available)
    event = WrenchStamped()
    event.header.frame_id = "base_link"
    event.header.stamp = robot.node.get_clock().now().to_msg()
    event.wrench.force.x, event.wrench.force.y, event.wrench.force.z = (
        offset + force for offset, force in zip(robot.offset, robot.force)
    )
    robot.teach_exit_pub.publish(event)
    robot.wait(lambda: robot.calibration_events == 2 and robot.status.actual_force_n < 0.05)
    assert not robot.status.direction_available
    robot.teach_exit_pub.publish(event)
    robot.pump(0.2)
    assert robot.calibration_events == 2
    assert robot.status.state == TractionStatus.READY


def test_teach_exit_event_cannot_tare_active_traction(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (6.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.actual_force_n > 5.9)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    event = WrenchStamped()
    event.header.frame_id = "base_link"
    event.header.stamp = robot.node.get_clock().now().to_msg()
    event.wrench.force.x, event.wrench.force.y, event.wrench.force.z = (
        offset + force for offset, force in zip(robot.offset, robot.force)
    )
    robot.teach_exit_pub.publish(event)
    robot.pump(0.3)
    assert robot.status.actual_force_n > 5.9
    assert robot.calibration_events == 1
    assert robot.status.state == TractionStatus.TRACTION


def test_physical_button_interlocks_armed_drag_and_tares_before_reenabling(workstation):
    robot = workstation
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.DRAGGING)
    robot.physical_teach_pub.publish(Bool(data=True))
    robot.force = (9.0, 0.0, 0.0)
    robot.pump(0.4)
    assert robot.native_calls == []
    event = WrenchStamped()
    event.header.frame_id = "base_link"
    event.header.stamp = robot.node.get_clock().now().to_msg()
    event.wrench.force.x, event.wrench.force.y, event.wrench.force.z = (
        offset + force for offset, force in zip(robot.offset, robot.force)
    )
    # The two DDS topics may arrive out of order. A delayed exit wrench
    # must not permit software drag to re-enable before the tare is applied.
    robot.physical_teach_pub.publish(Bool(data=False))
    robot.pump(0.25)
    assert robot.native_calls == []
    assert robot.calibration_events == 1
    event.header.stamp = robot.node.get_clock().now().to_msg()
    robot.teach_exit_pub.publish(event)
    robot.wait(lambda: robot.calibration_events == 2 and robot.status.actual_force_n < 0.05)
    robot.pump(0.2)
    assert robot.native_calls == []
    assert robot.status.state == TractionStatus.DRAGGING
    robot.force = (15.0, 0.0, 0.0)  # 6 N above the new loaded baseline
    robot.wait(lambda: robot.native_calls == ["start"])


def test_mode_change_waits_for_native_stop_confirmation(workstation):
    robot = workstation
    robot.force = (6.0, 0.0, 0.0)
    robot.call("start")
    robot.wait(lambda: robot.native_calls == ["start"])
    robot.native_stop_delay_s = 0.7
    robot.call("stop")
    robot.wait(robot.native_stop_entered.is_set)
    robot.wait(lambda: robot.status.state == TractionStatus.DRAGGING and not robot.status.ready)
    response = robot.request(robot.mode_client, SetOperationMode.Request(mode=1))
    assert not response.success
    assert not robot.status.fault_code
    robot.wait(lambda: robot.status.ready)
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.pump(0.2)
    assert robot.status.state == TractionStatus.READY
    assert robot.status.operation_mode == 1
    assert not robot.switch_calls
    assert robot.calibration_events == 1


def test_software_timeout_reset_preserves_baseline_and_never_resumes(workstation):
    robot = workstation
    robot.mode(SetOperationMode.Request.POSITION_TRACTION)
    robot.force = (8.0, 0.0, 0.0)
    robot.wait(lambda: robot.status.direction_available)
    robot.call("start")
    robot.wait(lambda: robot.status.state == TractionStatus.TRACTION)
    robot.allow_fault = True
    robot.wrench_paused = True
    robot.wait(lambda: robot.status.state == TractionStatus.FAULT)
    assert robot.status.fault_code == "WRENCH_TIMEOUT"
    response = robot.request(robot.clients["reset_transient_fault"], Trigger.Request())
    assert not response.success
    robot.wrench_paused = False
    robot.pump(0.3)
    robot.call("reset_transient_fault")
    robot.wait(lambda: robot.status.state == TractionStatus.READY)
    switch_count = len(robot.switch_calls)
    robot.pump(0.3)
    assert len(robot.switch_calls) == switch_count
    assert robot.status.calibrated
    assert not robot.status.direction_locked
    assert robot.calibration_events == 1
    assert robot.status.operation_mode == SetOperationMode.Request.POSITION_TRACTION


def test_automatic_reset_cannot_acknowledge_an_emergency(workstation):
    robot = workstation
    robot.allow_fault = True
    robot.call("emergency_stop")
    robot.wait(lambda: robot.status.state == TractionStatus.EMERGENCY_STOP)
    robot.pump(0.15)
    response = robot.request(robot.clients["reset_transient_fault"], Trigger.Request())
    assert not response.success
    assert robot.status.state == TractionStatus.EMERGENCY_STOP
    assert robot.calibration_events == 1
