import math
import unittest
from typing import ClassVar
from unittest.mock import patch

from backend.ros_bridge import SERVICE_RESPONSE_TIMEOUT_S, STATE_NAMES, RosBridge


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
    target_force_n = 10.0
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
    def test_latched_driver_failure_survives_joint_feedback_timeout(self):
        from types import SimpleNamespace

        bridge = RosBridge()
        status = SimpleNamespace(
            name="aubo_v4_driver/state", level=b'\x02',
            message="V4 joint stream timing gap exceeded 30 ms",
            values=[SimpleNamespace(key="fail_closed", value="1")],
        )
        with patch("backend.ros_bridge.time.monotonic", return_value=10.0):
            bridge._on_diagnostics(SimpleNamespace(status=[status]))
        with patch("backend.ros_bridge.time.monotonic", return_value=12.0):
            snapshot = bridge.snapshot()
        self.assertFalse(snapshot["robot"]["connected"])
        self.assertEqual(2, snapshot["robot"]["diagnostic"]["level"])
        self.assertEqual(status.message, snapshot["robot"]["diagnostic"]["message"])
        self.assertEqual(2000, snapshot["robot"]["diagnostic"]["data_age_ms"])

    def test_unverified_hardware_recovery_has_no_timeout_or_client(self):
        self.assertNotIn("hardware_emergency_recover", SERVICE_RESPONSE_TIMEOUT_S)
        self.assertGreaterEqual(SERVICE_RESPONSE_TIMEOUT_S["hardware_emergency_stop"], 2.0)

    def test_state_name_and_joint_conversion(self):
        bridge = RosBridge()
        bridge._on_joint_state(FakeJoint())
        bridge._on_traction(FakeTraction())
        snapshot = bridge.snapshot()
        self.assertEqual(STATE_NAMES[6], snapshot["traction"]["state_name"])
        self.assertEqual(1, snapshot["traction"]["operation_mode"])
        self.assertTrue(snapshot["robot"]["valid"])
        self.assertAlmostEqual(math.degrees(0.1), snapshot["robot"]["joint_velocity_deg_s"][0])

    def test_snapshot_is_safe_before_ros_start(self):
        with patch("backend.ros_bridge.time.monotonic", return_value=1.0):
            snapshot = RosBridge().snapshot()
        self.assertFalse(snapshot["connected"])
        self.assertFalse(snapshot["traction"]["valid"])

    def test_fr5_graph_cannot_make_aubo_bridge_connected(self):
        bridge = RosBridge()
        bridge._on_joint_state(FakeJoint())
        bridge._on_traction(FakeTraction())
        self.assertFalse(bridge.snapshot()["connected"])
        bridge._on_driver_identity(type("Identity", (), {"data": "FR5"})())
        self.assertFalse(bridge.snapshot()["robot"]["identity_verified"])
        bridge._on_driver_identity(type("Identity", (), {"data": "AUBO_I5_V4"})())
        self.assertTrue(bridge.snapshot()["robot"]["identity_verified"])

    def test_motion_fault_does_not_masquerade_as_lost_robot_communication(self):
        from types import SimpleNamespace

        bridge = RosBridge()
        bridge._on_joint_state(FakeJoint())
        bridge._on_traction(FakeTraction())
        bridge._on_driver_identity(type("Identity", (), {"data": "AUBO_I5_V4"})())
        fault = SimpleNamespace(
            name="aubo_v4_driver/state",
            level=b"\x02",
            message="fixed-pose session orientation changed; re-calibration required",
            values=[
                SimpleNamespace(
                    key="fault_reason", value="fixed-pose session orientation changed"
                )
            ],
        )
        bridge._on_diagnostics(SimpleNamespace(status=[fault]))

        snapshot = bridge.snapshot()
        self.assertTrue(snapshot["robot"]["connected"])
        self.assertTrue(snapshot["robot"]["identity_verified"])
        self.assertTrue(snapshot["connected"])
        self.assertEqual(2, snapshot["robot"]["diagnostic"]["level"])
        self.assertEqual(fault.message, snapshot["robot"]["diagnostic"]["message"])


if __name__ == "__main__":
    unittest.main()
