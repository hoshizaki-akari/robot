"""Offline ROS tests for the traction manager's stepwise zero-pose contract."""

from collections import deque
import csv
import math
import os
from pathlib import Path
import shutil
import subprocess
import tempfile
import threading
import time

import pytest
import rclpy
from controller_manager_msgs.srv import SwitchController
from fr_traction.msg import EndEffectorState, TractionCommand, TractionStatus
from fr_traction.srv import SetOperationMode, SetTargetForce
from geometry_msgs.msg import WrenchStamped
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool
from std_srvs.srv import Trigger


DOMAIN_ID = "175"
BASE_FRAME = "base_link"
WRENCH_TOPIC = "/test/stepwise_manager/wrench"
EE_TOPIC = "/test/stepwise_manager/ee"
JOINT_TOPIC = "/test/stepwise_manager/joints"
CONTROLLER_HEALTH_TOPIC = "/test/stepwise_manager/controller_health"
HARDWARE_HEALTH_TOPIC = "/test/stepwise_manager/hardware_health"
MOTION_IDLE_TOPIC = "/controller_manager/motion_idle"
SWITCH_SERVICE = "/test/stepwise_manager/switch_controller"
COMMAND_TOPIC = "/test/stepwise_manager/command"
STATUS_TOPIC = "/traction/status"
READY = TractionStatus.READY
RETURNING_ZERO = TractionStatus.RETURNING_ZERO


def _reliable_qos(depth=10, transient_local=False):
    """Return the reliable QoS used by manager input and status endpoints."""
    return QoSProfile(
        depth=depth,
        reliability=ReliabilityPolicy.RELIABLE,
        durability=(
            DurabilityPolicy.TRANSIENT_LOCAL
            if transient_local
            else DurabilityPolicy.VOLATILE
        ),
    )


class ManagerProcess:
    """Own one manager subprocess and remove only its temporary data."""

    def __init__(self, require_idle=True, extra_parameters=None):
        self.require_idle = require_idle
        self.extra_parameters = extra_parameters or []
        self.process = None
        self.data_directory = tempfile.mkdtemp(prefix="fr_traction_manager_ros_")

    def start(self):
        """Start the parent-provided manager executable on localhost only."""
        executable = os.environ.get("AUBO_TRACTION_MANAGER_EXECUTABLE")
        if not executable:
            shutil.rmtree(self.data_directory, ignore_errors=True)
            pytest.skip("AUBO_TRACTION_MANAGER_EXECUTABLE is not set")

        arguments = [
            str(Path(executable)),
            "--ros-args",
            "-p",
            "aubo_port_verified:=true",
            "-p",
            f"require_measured_motion_idle:={'true' if self.require_idle else 'false'}",
            "-p",
            f"wrench_topic:={WRENCH_TOPIC}",
            "-p",
            f"ee_state_topic:={EE_TOPIC}",
            "-p",
            f"joint_state_topic:={JOINT_TOPIC}",
            "-p",
            f"controller_health_topic:={CONTROLLER_HEALTH_TOPIC}",
            "-p",
            f"hardware_health_topic:={HARDWARE_HEALTH_TOPIC}",
            "-p",
            f"controller_manager_switch_service:={SWITCH_SERVICE}",
            "-p",
            f"command_topic:={COMMAND_TOPIC}",
            "-p",
            f"data_directory:={self.data_directory}",
            "-p",
            "control_rate_hz:=100.0",
            "-p",
            "status_rate_hz:=20.0",
            # This harness publishes at approximately 50 Hz, not the 100 Hz
            # hardware stream. Test stop/activation ordering with a realistic
            # sample window instead of racing the default 80-sample threshold.
            "-p",
            "calibration_window_s:=2.0",
            "-p",
            "calibration_min_samples:=40",
        ]
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

    def stop(self):
        """Terminate only this test-owned manager with bounded waits."""
        if self.process is not None and self.process.poll() is None:
            self.process.terminate()
            try:
                self.process.wait(timeout=3.0)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=3.0)
        shutil.rmtree(self.data_directory, ignore_errors=True)


class FakeSwitchService:
    """Record controller switches and optionally hold one start response."""

    def __init__(self, runtime, delay_start=False):
        self.node = Node("stepwise_manager_switch_fake", context=runtime)
        self.delay_start = delay_start
        self.start_entered = threading.Event()
        self.release_start = threading.Event()
        self.start_requests = 0
        self.stop_requests = 0
        self.late_start_responses = 0
        self._lock = threading.Lock()
        self.service = self.node.create_service(
            SwitchController,
            SWITCH_SERVICE,
            self._callback,
            callback_group=ReentrantCallbackGroup(),
        )

    def _callback(self, request, response):
        """Return healthy fake switch results, with one bounded delayed start."""
        is_start = bool(request.start_controllers)
        with self._lock:
            if is_start:
                self.start_requests += 1
            else:
                self.stop_requests += 1
        if is_start and self.delay_start and not self.start_entered.is_set():
            self.start_entered.set()
            self.release_start.wait(timeout=2.0)
            with self._lock:
                self.late_start_responses += 1
        response.ok = True
        return response

    def release(self):
        """Release any deliberately delayed callback during bounded cleanup."""
        self.release_start.set()


class ManagerHarness:
    """Publish fresh scripted manager inputs and collect status history."""

    def __init__(self, runtime, child, delay_start=False):
        self.child = child
        self.node = Node("stepwise_manager_ros_test", context=runtime)
        self.switch = FakeSwitchService(runtime, delay_start=delay_start)
        self.executor = MultiThreadedExecutor(num_threads=4, context=runtime)
        self.executor.add_node(self.node)
        self.executor.add_node(self.switch.node)
        input_qos = _reliable_qos()
        health_qos = _reliable_qos(depth=1, transient_local=True)
        self.wrench_pub = self.node.create_publisher(
            WrenchStamped, WRENCH_TOPIC, input_qos
        )
        self.ee_pub = self.node.create_publisher(
            EndEffectorState, EE_TOPIC, input_qos
        )
        self.joint_pub = self.node.create_publisher(JointState, JOINT_TOPIC, input_qos)
        self.controller_health_pub = self.node.create_publisher(
            Bool, CONTROLLER_HEALTH_TOPIC, health_qos
        )
        self.hardware_health_pub = self.node.create_publisher(
            Bool, HARDWARE_HEALTH_TOPIC, health_qos
        )
        self.motion_idle_pub = self.node.create_publisher(
            Bool, MOTION_IDLE_TOPIC, health_qos
        )
        self.status_history = deque(maxlen=2000)
        self.command_history = deque(maxlen=2000)
        self.motion_idle_history = deque(maxlen=200)
        self.node.create_subscription(
            TractionStatus,
            STATUS_TOPIC,
            lambda message: self.status_history.append(message),
            _reliable_qos(),
        )
        self.node.create_subscription(
            TractionCommand,
            COMMAND_TOPIC,
            lambda message: self.command_history.append(message),
            _reliable_qos(),
        )
        self.node.create_subscription(
            Bool,
            MOTION_IDLE_TOPIC,
            lambda message: self.motion_idle_history.append(message),
            health_qos,
        )
        self.publish_wrench = True
        self.publish_ee = True
        self.publish_joint = True
        self.publish_controller_health = True
        self.publish_hardware_health = True
        self.publish_motion_idle = True
        self.motion_idle = True
        self.wrench_force = (0.0, 0.0, 0.0)
        self.ee_position = (0.0, 0.0, 0.0)
        self.ee_quaternion = (0.0, 0.0, 0.0, 1.0)
        self.ee_frame = BASE_FRAME
        self.controller_health = True
        self.hardware_health = True
        self._clients = {}

    def _stamp(self, message):
        """Stamp an input from the test's current ROS clock."""
        message.header.stamp = self.node.get_clock().now().to_msg()

    def publish_inputs(self):
        """Publish only the currently enabled fresh scripted input streams."""
        if self.publish_wrench:
            message = WrenchStamped()
            self._stamp(message)
            message.header.frame_id = BASE_FRAME
            message.wrench.force.x, message.wrench.force.y, message.wrench.force.z = (
                self.wrench_force
            )
            self.wrench_pub.publish(message)
        if self.publish_ee:
            message = EndEffectorState()
            self._stamp(message)
            message.header.frame_id = self.ee_frame
            message.pose.position.x, message.pose.position.y, message.pose.position.z = (
                self.ee_position
            )
            (
                message.pose.orientation.x,
                message.pose.orientation.y,
                message.pose.orientation.z,
                message.pose.orientation.w,
            ) = self.ee_quaternion
            self.ee_pub.publish(message)
        if self.publish_joint:
            message = JointState()
            self._stamp(message)
            message.velocity = [0.0] * 6
            self.joint_pub.publish(message)
        if self.publish_controller_health:
            message = Bool()
            message.data = self.controller_health
            self.controller_health_pub.publish(message)
        if self.publish_hardware_health:
            message = Bool()
            message.data = self.hardware_health
            self.hardware_health_pub.publish(message)
        if self.publish_motion_idle:
            message = Bool()
            message.data = self.motion_idle
            self.motion_idle_pub.publish(message)

    def spin_once(self, timeout_s=0.02):
        """Publish fresh inputs and execute one bounded executor slice."""
        self.publish_inputs()
        self.executor.spin_once(timeout_sec=timeout_s)

    def mark(self):
        """Return a status-history cursor for assertions about new observations."""
        return len(self.status_history)

    def wait_until(self, predicate, timeout_s=3.0):
        """Wait with a monotonic deadline while keeping all inputs fresh."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_once(min(0.02, max(0.0, deadline - time.monotonic())))
            if predicate():
                return
        raise AssertionError("timed out waiting for bounded manager condition")

    def pump(self, duration_s=0.15):
        """Process a bounded interval so a changed input reaches the child."""
        deadline = time.monotonic() + duration_s
        while time.monotonic() < deadline:
            self.spin_once(min(0.02, max(0.0, deadline - time.monotonic())))

    def wait_status(self, state, timeout_s=3.0, after=0):
        """Wait for a status state observed after the supplied history cursor."""
        self.wait_until(
            lambda: any(message.state == state for message in list(self.status_history)[after:]),
            timeout_s=timeout_s,
        )

    def wait_motion_idle(self, expected, timeout_s=1.0):
        """Confirm the test node received the requested fresh idle feedback."""
        mark = len(self.motion_idle_history)
        self.wait_until(
            lambda: any(
                message.data == expected
                for message in list(self.motion_idle_history)[mark:]
            ),
            timeout_s=timeout_s,
        )

    def wait_ready(self, timeout_s=4.0):
        """Wait for manager readiness after fresh input discovery."""
        self.wait_until(
            lambda: bool(self.status_history)
            and self.status_history[-1].state == READY
            and self.status_history[-1].ready,
            timeout_s=timeout_s,
        )

    def wait_stable_state(self, state, timeout_s=1.0, samples=3):
        """Require repeated status samples in one state."""
        deadline = time.monotonic() + timeout_s
        while time.monotonic() < deadline:
            self.spin_once(0.02)
            recent = list(self.status_history)[-samples:]
            if len(recent) == samples and all(item.state == state for item in recent):
                return
        raise AssertionError(f"manager did not remain in state {state}")

    def trigger(self, service_name, timeout_s=3.0):
        """Call a manager Trigger service while spinning the test executor."""
        return self.call_service(service_name, Trigger, Trigger.Request(), timeout_s)

    def call_service(self, service_name, service_type, request, timeout_s=3.0):
        """Call a typed manager service while spinning bounded executor slices."""
        key = (service_name, service_type)
        client = self._clients.get(key)
        if client is None:
            client = self.node.create_client(service_type, service_name)
            self._clients[key] = client
        deadline = time.monotonic() + timeout_s
        while not client.wait_for_service(timeout_sec=0.05):
            if time.monotonic() >= deadline:
                raise AssertionError(f"service {service_name} did not become ready")
        future = client.call_async(request)
        while not future.done():
            if time.monotonic() >= deadline:
                raise AssertionError(f"service {service_name} did not answer")
            self.spin_once(0.02)
        return future.result()

    def set_operation_mode(self, mode):
        """Select a stopped manager operation mode through its real service."""
        request = SetOperationMode.Request()
        request.mode = mode
        return self.call_service(
            "/traction/set_operation_mode", SetOperationMode, request
        )

    def set_target_force(self, target_force_n):
        """Set a normal traction target through the real manager service."""
        request = SetTargetForce.Request()
        request.target_force_n = target_force_n
        return self.call_service(
            "/traction/set_target_force", SetTargetForce, request
        )

    def close(self):
        """Release fake callbacks and tear down only this test's ROS entities."""
        self.switch.release()
        self.child.stop()
        for client in self._clients.values():
            self.node.destroy_client(client)
        self.executor.remove_node(self.switch.node)
        self.executor.remove_node(self.node)
        self.switch.node.destroy_node()
        self.node.destroy_node()
        self.executor.shutdown()


@pytest.fixture(scope="session")
def ros_runtime():
    """Initialize one localhost-only ROS context for manager subprocess tests."""
    os.environ.update(
        {
            "ROS_DOMAIN_ID": DOMAIN_ID,
            "ROS_LOCALHOST_ONLY": "1",
            "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        }
    )
    if not rclpy.ok():
        rclpy.init(args=None)
    yield rclpy.get_default_context()
    if rclpy.ok():
        rclpy.shutdown()


@pytest.fixture
def manager(ros_runtime):
    """Run one measured-idle manager with a normal fake switch service."""
    child = ManagerProcess(require_idle=True).start()
    harness = ManagerHarness(ros_runtime, child)
    try:
        yield harness
    finally:
        harness.close()


def test_drag_limit_is_distinct_and_still_debounces_overforce(ros_runtime):
    """Ordinary hand force is permitted in drag, but a sustained 20 N trip remains."""
    child = ManagerProcess(extra_parameters=[
        "-p", "target_force_max_n:=5.0", "-p", "validated_target_max_n:=5.0",
        "-p", "target_force_n:=5.0", "-p", "hard_overforce_n:=8.0",
        "-p", "drag_hard_overforce_n:=20.0", "-p", "hard_overforce_confirm_s:=0.1",
    ]).start()
    harness = ManagerHarness(ros_runtime, child)
    try:
        harness.wait_ready()
        assert harness.set_operation_mode(2).success
        assert harness.trigger('/traction/prepare').success
        assert harness.trigger('/traction/start').success
        harness.wait_status(TractionStatus.DRAGGING)
        harness.wrench_force = (12.0, 0.0, 0.0)
        harness.pump(duration_s=0.5)
        assert harness.status_history[-1].state == TractionStatus.DRAGGING
        harness.wrench_force = (21.0, 0.0, 0.0)
        harness.wait_status(TractionStatus.FAULT)
        assert harness.status_history[-1].fault_code == 'HARD_OVERFORCE'
    finally:
        harness.close()


def test_session_bound_waits_for_actual_stop_and_allows_next_prepare(manager):
    """Reaching an operational session boundary is not an unrecoverable device fault."""
    manager.wait_ready()
    assert manager.set_operation_mode(2).success
    assert manager.trigger('/traction/prepare').success
    assert manager.trigger('/traction/start').success
    manager.wait_status(TractionStatus.DRAGGING)
    manager.motion_idle = False
    manager.wait_motion_idle(False)
    assert manager.trigger('/traction/session_limit_stop').success
    manager.pump(duration_s=0.15)
    assert manager.status_history[-1].state == TractionStatus.DRAGGING
    manager.motion_idle = True
    manager.wait_motion_idle(True)
    manager.wait_stable_state(READY)
    assert manager.status_history[-1].stop_reason == 'SESSION_TRAVEL_BOUND'
    assert manager.trigger('/traction/prepare').success
    manager.wait_stable_state(TractionStatus.MANUAL_SETUP)


@pytest.fixture
def default_manager(ros_runtime):
    """Run the manager's default mode without measured-idle opt-in."""
    child = ManagerProcess(require_idle=False).start()
    harness = ManagerHarness(ros_runtime, child)
    try:
        yield harness
    finally:
        harness.close()


@pytest.fixture
def delayed_manager(ros_runtime):
    """Run a manager whose first controller start response arrives late."""
    child = ManagerProcess(require_idle=True).start()
    harness = ManagerHarness(ros_runtime, child, delay_start=True)
    try:
        yield harness
    finally:
        harness.close()


def _capture_zero(harness):
    """Reach READY and save the current fresh idle pose."""
    harness.wait_ready()
    response = harness.trigger("/traction/set_zero_pose")
    assert response.success, response.message


def test_set_zero_default_mode_is_refused(default_manager):
    """The default manager mode refuses zero capture without measured idle data."""
    default_manager.wait_ready()
    response = default_manager.trigger("/traction/set_zero_pose")
    assert not response.success
    assert "unsupported" in response.message.lower()


@pytest.mark.parametrize("missing", ["fresh", "idle"])
def test_stepwise_set_zero_requires_fresh_idle_inputs(manager, missing):
    """Stepwise zero capture refuses either missing freshness or measured idle."""
    manager.wait_ready()
    if missing == "fresh":
        manager.publish_ee = False
        manager.pump(duration_s=0.35)
    else:
        manager.motion_idle = False
        manager.wait_motion_idle(False)
        manager.pump(duration_s=0.15)
    response = manager.trigger("/traction/set_zero_pose")
    assert not response.success
    assert "rejected" in response.message.lower()


def test_valid_zero_pose_is_captured_for_this_session(manager):
    """A fresh stationary pose is accepted and reported as non-persistent."""
    manager.wait_ready()
    response = manager.trigger("/traction/set_zero_pose")
    assert response.success
    assert "not persisted" in response.message.lower()


@pytest.mark.parametrize(
    ("invalid_case", "position", "force", "quaternion"),
    [
        ("distance", (0.10, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ("load", (0.0, 0.0, 0.0), (2.0, 0.0, 0.0), (0.0, 0.0, 0.0, 1.0)),
        ("orientation", (0.0, 0.0, 0.0), (0.0, 0.0, 0.0), (0.0, 0.0998, 0.0, 0.995)),
    ],
)
def test_return_zero_rejects_unbounded_or_loaded_pose(
    manager, invalid_case, position, force, quaternion
):
    """Return zero rejects distance, unloaded-force, and orientation violations."""
    _capture_zero(manager)
    manager.ee_position = position
    manager.wrench_force = force
    manager.ee_quaternion = quaternion
    manager.pump()
    response = manager.trigger("/traction/return_zero_pose")
    assert not response.success, invalid_case
    assert "rejected" in response.message.lower()
    manager.wait_stable_state(READY)


def test_return_state13_waits_for_feedback_and_stop_waits_for_idle(manager):
    """A legal return remains state 13, then stop completes only after idle feedback."""
    _capture_zero(manager)
    manager.ee_position = (0.02, 0.0, 0.0)
    manager.pump()
    command_mark = len(manager.command_history)
    response = manager.trigger("/traction/return_zero_pose")
    assert response.success
    manager.wait_status(RETURNING_ZERO)
    manager.wait_until(
        lambda: any(
            command.mode == TractionCommand.RELEASING
            and math.sqrt(
                command.lateral_velocity_base.x ** 2
                + command.lateral_velocity_base.y ** 2
                + command.lateral_velocity_base.z ** 2
            )
            > 1.0e-9
            for command in list(manager.command_history)[command_mark:]
        ),
        timeout_s=2.0,
    )
    active_command_mark = len(manager.command_history)
    manager.pump(duration_s=0.30)
    active_commands = list(manager.command_history)[active_command_mark:]
    assert active_commands
    assert all(command.mode != TractionCommand.DISABLED for command in active_commands)
    manager.wait_stable_state(RETURNING_ZERO, timeout_s=0.5)

    manager.motion_idle = False
    manager.wait_motion_idle(False)
    manager.pump(duration_s=0.15)
    response = manager.trigger("/traction/stop")
    assert response.success
    assert "waiting" in response.message.lower()
    manager.wait_stable_state(RETURNING_ZERO, timeout_s=0.5)
    assert manager.switch.stop_requests >= 1

    manager.motion_idle = True
    manager.wait_stable_state(READY, timeout_s=2.0)


def test_late_switch_start_cannot_revive_after_stop(delayed_manager):
    """A delayed start response is stale after stop and cannot revive state 13."""
    _capture_zero(delayed_manager)
    delayed_manager.ee_position = (0.02, 0.0, 0.0)
    delayed_manager.pump()
    response = delayed_manager.trigger("/traction/return_zero_pose")
    assert response.success
    delayed_manager.wait_until(
        lambda: delayed_manager.switch.start_entered.is_set(), timeout_s=2.0
    )
    delayed_manager.wait_status(RETURNING_ZERO)

    response = delayed_manager.trigger("/traction/stop")
    assert response.success
    stop_mark = delayed_manager.mark()
    delayed_manager.switch.release()
    delayed_manager.wait_until(
        lambda: delayed_manager.switch.late_start_responses == 1, timeout_s=2.0
    )
    delayed_manager.wait_stable_state(READY, timeout_s=1.0, samples=5)
    assert all(
        message.state != RETURNING_ZERO
        for message in list(delayed_manager.status_history)[stop_mark:]
    )


@pytest.mark.parametrize(
    ("mode", "active_state"),
    [
        (SetOperationMode.Request.ASSISTED_DRAG, TractionStatus.DRAGGING),
        (SetOperationMode.Request.CONSTANT_FORCE, TractionStatus.TRACTION),
    ],
)
def test_stop_invalidates_delayed_start_before_active_mode(
    delayed_manager, mode, active_state
):
    """A stop during delayed activation prevents drag or traction revival."""
    delayed_manager.wait_ready()
    mode_response = delayed_manager.set_operation_mode(mode)
    assert mode_response.success, mode_response.message
    prepare_response = delayed_manager.trigger("/traction/prepare")
    assert prepare_response.success, prepare_response.message
    delayed_manager.wait_status(TractionStatus.MANUAL_SETUP)

    if mode == SetOperationMode.Request.CONSTANT_FORCE:
        delayed_manager.wrench_force = (3.0, 0.0, 0.0)
        delayed_manager.pump(duration_s=0.20)
        calibrate_response = delayed_manager.trigger("/traction/calibrate_direction")
        assert calibrate_response.success, calibrate_response.message
        delayed_manager.wait_status(TractionStatus.DIRECTION_LOCKED, timeout_s=3.0)
        target_response = delayed_manager.set_target_force(5.0)
        assert target_response.success, target_response.message

    start_mark = len(delayed_manager.status_history)
    start_response = delayed_manager.trigger("/traction/start")
    assert start_response.success, start_response.message
    delayed_manager.wait_until(
        lambda: delayed_manager.switch.start_entered.is_set(), timeout_s=2.0
    )

    stop_response = delayed_manager.trigger("/traction/stop")
    assert stop_response.success, stop_response.message
    stop_mark = len(delayed_manager.status_history)
    delayed_manager.switch.release()
    delayed_manager.wait_until(
        lambda: delayed_manager.switch.late_start_responses == 1, timeout_s=2.0
    )
    delayed_manager.pump(duration_s=0.35)
    assert any(
        message.state in {
            TractionStatus.READY, TractionStatus.MANUAL_SETUP, TractionStatus.DIRECTION_LOCKED}
        for message in list(delayed_manager.status_history)[stop_mark:]
    )
    assert all(
        message.state != active_state
        for message in list(delayed_manager.status_history)[stop_mark:]
    )
    assert all(
        message.state != active_state
        for message in list(delayed_manager.status_history)[start_mark:stop_mark]
    )


@pytest.mark.parametrize('mode', [0, 1])
def test_repeated_traction_start_records_each_run(manager, mode):
    """Two starts with one direction calibration retain two real manager records."""
    manager.wait_ready()
    assert manager.set_operation_mode(mode).success
    assert manager.trigger('/traction/prepare').success
    manager.wrench_force = (3.0, 0.0, 0.0)
    manager.pump(duration_s=0.20)
    assert manager.trigger('/traction/calibrate_direction').success
    manager.wait_status(TractionStatus.DIRECTION_LOCKED, timeout_s=4.0)
    for _ in range(2):
        assert manager.set_target_force(5.0).success
        mark = manager.mark()
        assert manager.trigger('/traction/start').success
        manager.wait_status(TractionStatus.TRACTION, after=mark)
        manager.pump(duration_s=0.15)
        mark = manager.mark()
        assert manager.trigger('/traction/stop').success
        manager.wait_status(TractionStatus.DIRECTION_LOCKED, after=mark)
    records = sorted(Path(manager.child.data_directory).glob('session_*/traction.csv'))
    assert len(records) == 2
    with (Path(manager.child.data_directory) / 'sessions_multimode.csv').open(
        newline='', encoding='utf-8'
    ) as stream:
        summaries = {row['session_id']: row for row in csv.DictReader(stream)}
    for record in records:
        with record.open(newline='', encoding='utf-8') as stream:
            rows = list(csv.DictReader(stream))
        assert any(row['state'] == 'TRACTION' for row in rows)
        assert rows[-1]['state'] == 'COMPLETED'
        assert rows[-1]['measured_motion_idle'] == '1'
        assert rows[-1]['measured_idle_fresh'] == '1'
        assert 'fault_code' in rows[-1]
        assert 'ee_vx_mps' in rows[-1]
        summary = summaries[record.parent.name]
        session_start_ms = int(record.parent.name.removeprefix('session_'))
        assert abs(float(summary['start_time']) - session_start_ms / 1000.0) < 0.002
        assert abs(float(summary['end_time']) - float(rows[-1]['timestamp'])) < 0.000001
