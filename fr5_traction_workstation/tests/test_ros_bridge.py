import math
import unittest
from typing import ClassVar
from unittest.mock import patch

from backend.ros_bridge import STATE_NAMES, RosBridge


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
    def test_state_name_and_joint_conversion(self):
        bridge = RosBridge()
        bridge._on_joint_state(FakeJoint())
        bridge._on_traction(FakeTraction())
        snapshot = bridge.snapshot()
        self.assertEqual(STATE_NAMES[6], snapshot["traction"]["state_name"])
        self.assertTrue(snapshot["fr5"]["valid"])
        self.assertAlmostEqual(math.degrees(0.1), snapshot["fr5"]["joint_velocity_deg_s"][0])

    def test_snapshot_is_safe_before_ros_start(self):
        with patch("backend.ros_bridge.time.monotonic", return_value=1.0):
            snapshot = RosBridge().snapshot()
        self.assertFalse(snapshot["connected"])
        self.assertFalse(snapshot["traction"]["valid"])


if __name__ == "__main__":
    unittest.main()
