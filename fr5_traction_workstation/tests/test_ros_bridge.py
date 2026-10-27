import math
import unittest
from types import SimpleNamespace
from typing import ClassVar
from unittest.mock import Mock, patch

from backend.ros_bridge import (
    SERVICE_RESPONSE_TIMEOUT_S,
    STATE_NAMES,
    RosBridge,
    RosBridgeError,
)


class FakeHeader:
    class Stamp:
        sec = 1
        nanosec = 2

    stamp = Stamp()


class FakeVector:
    x = 1.0
    y = 2.0
    z = 3.0


class FakePoint(FakeVector):
    pass


class FakeTraction:
    header = FakeHeader()
    state = 6
    operation_mode = 1
    position_reached = False
    ready = True
    calibrated = True
    direction_available = True
    direction_locked = False
    target_force_n = 10.0
    target_force_max_n = 160.0
    actual_force_n = 9.5
    lateral_force_n = 0.2
    fx = 1.0
    fy = 2.0
    fz = 3.0
    locked_direction_base = FakeVector()
    ee_position_base = FakePoint()
    axis_displacement_m = 0.01
    velocity_cmd_mps = 0.001
    fault_code = ""
    stop_reason = ""


class FakeJoint:
    name: ClassVar[list[str]] = ["j3", "j1", "j6", "j2", "j5", "j4"]
    position: ClassVar[list[float]] = [0.0] * 6
    velocity: ClassVar[list[float]] = [0.1] * 6


class BridgeTest(unittest.TestCase):
    def test_hardware_recovery_waits_for_full_driver_sequence(self):
        self.assertGreaterEqual(SERVICE_RESPONSE_TIMEOUT_S["hardware_emergency_recover"], 15.0)

    def test_state_name_and_joint_conversion(self):
        bridge = RosBridge()
        bridge._on_joint_state(FakeJoint())
        bridge._on_traction(FakeTraction())
        snapshot = bridge.snapshot()
        self.assertEqual(STATE_NAMES[6], snapshot["traction"]["state_name"])
        self.assertEqual(1, snapshot["traction"]["operation_mode"])
        self.assertEqual(160.0, snapshot["traction"]["target_force_max_n"])
        self.assertTrue(snapshot["traction"]["calibrated"])
        self.assertTrue(snapshot["traction"]["direction_available"])
        self.assertFalse(snapshot["traction"]["direction_locked"])
        self.assertTrue(snapshot["fr5"]["valid"])
        self.assertAlmostEqual(math.degrees(0.1), snapshot["fr5"]["joint_velocity_deg_s"][0])

    def test_target_limit_parameter_is_confirmed_by_manager(self):
        bridge = RosBridge()
        bridge._started = True
        bridge._node = object()
        client = Mock()
        client.srv_type.Request = SimpleNamespace
        bridge._clients["set_target_force_limit"] = client
        interface = SimpleNamespace(
            Parameter=SimpleNamespace, ParameterValue=SimpleNamespace,
            ParameterType=SimpleNamespace(PARAMETER_DOUBLE=3),
        )
        with (
            patch.dict("sys.modules", {"rcl_interfaces.msg": interface}),
            patch.object(bridge, "_wait_for_future", return_value=SimpleNamespace(
                result=SimpleNamespace(successful=True, reason="")
            )),
        ):
            bridge.set_target_force_limit(155)
        request = client.call_async.call_args.args[0]
        self.assertEqual("target_force_max_n", request.parameters[0].name)
        self.assertEqual(155.0, request.parameters[0].value.double_value)
        self.assertEqual(3, request.parameters[0].value.type)
        with (
            patch.dict("sys.modules", {"rcl_interfaces.msg": interface}),
            patch.object(bridge, "_wait_for_future", return_value=SimpleNamespace(
                result=SimpleNamespace(successful=False, reason="motion is active")
            )),
            self.assertRaisesRegex(RosBridgeError, "motion is active"),
        ):
            bridge.set_target_force_limit(170)

    def test_snapshot_is_safe_before_ros_start(self):
        with patch("backend.ros_bridge.time.monotonic", return_value=1.0):
            snapshot = RosBridge().snapshot()
        self.assertFalse(snapshot["connected"])
        self.assertFalse(snapshot["traction"]["valid"])


if __name__ == "__main__":
    unittest.main()
