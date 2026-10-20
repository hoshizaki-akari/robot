"""
Actual ROS node tests with a test-only SDK-constructor abort interposer.

No real SDK object can be constructed. Localhost-only ROS test domain, no
robot, serial, UI, fabricated operational feedback or real motion API calls.
"""
import os
import subprocess
import time

import rclpy
from controller_manager_msgs.srv import SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from fr_traction.msg import EndEffectorState
from geometry_msgs.msg import Twist
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool, String
from std_srvs.srv import Trigger


def environment():
    """Return an isolated localhost-only environment with SDK construction blocked."""
    env = os.environ.copy()
    env.update({
        "ROS_DOMAIN_ID": "172", "ROS_LOCALHOST_ONLY": "1",
        "RMW_IMPLEMENTATION": "rmw_cyclonedds_cpp",
        "LD_PRELOAD": os.environ["AUBO_SDK_GUARD"],
        "AUBO_V4_HOST": "127.0.0.1", "AUBO_V4_PORT": "9",
        "AUBO_V4_USERNAME": "offline-test", "AUBO_V4_PASSWORD": "offline-test",
    })
    return env


def test_sdk_constructor_guard_negative_control():
    """Prove the test guard intercepts the real binary before SDK construction."""
    # connect=true must hit our interposer and exit BEFORE the SDK constructor,
    # before login, before any controller network or motion operation.
    process = subprocess.run([
        os.environ["AUBO_STEPWISE_EXECUTABLE"], "--ros-args",
        "-p", "connect_hardware:=true", "-p", "enable_motion_interface:=false",
    ], env=environment(), stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=10)
    assert process.returncode == 91, process.stdout.decode(errors="replace")


def test_offline_node_refuses_motion_and_publishes_no_fake_pose(monkeypatch):
    """Check the offline motion boundary using the actual ROS executable."""
    env = environment()
    for name in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY", "RMW_IMPLEMENTATION"):
        monkeypatch.setenv(name, env[name])
    process = subprocess.Popen([
        os.environ["AUBO_STEPWISE_EXECUTABLE"], "--ros-args",
        "-p", "connect_hardware:=false", "-p", "enable_motion_interface:=true",
        "-p", "maximum_speed_mps:=0.05", "-p", "maximum_session_travel_m:=0.0",
    ], env=env, stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL)
    rclpy.init()
    node = rclpy.create_node("stepwise_explicit_offline_test")
    seen = {"joint": 0, "pose": 0}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def diagnostic(message):
        for status in message.status:
            if status.name == "aubo_v4_driver/state":
                seen["diagnostic"] = status

    subscriptions = [
        node.create_subscription(
            DiagnosticArray, "/diagnostics", diagnostic, qos_profile_sensor_data),
        node.create_subscription(
            Bool, "/controller_manager/healthy", lambda x: seen.update(healthy=x.data), qos),
        node.create_subscription(
            Bool, "/controller_manager/motion_idle", lambda x: seen.update(idle=x.data), qos),
        node.create_subscription(
            String, "/aubo_i5/driver_identity", lambda x: seen.update(identity=x.data), qos),
        node.create_subscription(
            JointState, "/joint_states", lambda x: seen.update(joint=seen["joint"] + 1), 10),
        node.create_subscription(
            EndEffectorState, "/controller_manager/ee_state",
            lambda x: seen.update(pose=seen["pose"] + 1), 10),
    ]
    velocity = node.create_publisher(Twist, "/controller_manager/command_cart_vel", 10)
    switch = node.create_client(SwitchController, "/controller_manager/switch_controller")
    stop = node.create_client(Trigger, "/aubo_v4_state_node/hardware_stop")

    def wait(predicate, seconds=5.0):
        deadline = time.monotonic() + seconds
        while not predicate() and time.monotonic() < deadline:
            assert process.poll() is None, "Offline node exited or touched SDK constructor"
            rclpy.spin_once(node, timeout_sec=0.02)
        assert predicate(), seen

    try:
        wait(lambda: all(key in seen for key in ("diagnostic", "healthy", "idle", "identity")))
        assert seen["healthy"] is False
        assert seen["idle"] is False
        assert seen["identity"] == ""
        assert "OFFLINE" in seen["diagnostic"].message
        values = {item.key: item.value for item in seen["diagnostic"].values}
        assert float(values["maximum_speed_mps"]) == 0.05
        assert float(values["maximum_session_travel_m"]) == 0.0
        assert "OFFLINE" in values["last_hardware_health_failure"]
        assert switch.wait_for_service(timeout_sec=2)
        request = SwitchController.Request()
        request.start_controllers = ["cartesian_velocity_controller"]
        future = switch.call_async(request)
        wait(future.done)
        assert future.result().ok is False
        command = Twist()
        command.linear.x = 0.002
        velocity.publish(command)
        assert stop.wait_for_service(timeout_sec=2)
        stopped = stop.call_async(Trigger.Request())
        wait(stopped.done)
        assert stopped.result().success is False
        assert seen["joint"] == 0 and seen["pose"] == 0
        assert process.poll() is None
    finally:
        _ = subscriptions
        node.destroy_node()
        rclpy.shutdown()
        process.terminate()
        try:
            process.wait(timeout=5)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)
