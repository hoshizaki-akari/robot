from __future__ import annotations

import io
import json
import math
import time
import unittest
from unittest.mock import patch

from force_demo.classifier import ForceTensionEngine
from force_demo.sources import (
    FallbackSource,
    _quaternion_rotate,
    _quaternion_to_rpy_deg,
    _result_values,
)


class SourceHelpersTest(unittest.TestCase):
    def test_quaternion_rotation_to_base(self):
        half = math.sqrt(0.5)
        rotated = _quaternion_rotate((1.0, 0.0, 0.0), (0.0, 0.0, half, half))
        self.assertAlmostEqual(rotated[0], 0.0, places=6)
        self.assertAlmostEqual(rotated[1], 1.0, places=6)
        self.assertAlmostEqual(rotated[2], 0.0, places=6)

    def test_quaternion_to_rpy(self):
        half = math.sqrt(0.5)
        rpy = _quaternion_to_rpy_deg((0.0, 0.0, half, half))
        self.assertAlmostEqual(rpy[2], 90.0, places=6)

    def test_sdk_result_parser_rejects_error(self):
        with self.assertRaises(RuntimeError):
            _result_values((5, [1, 2, 3]), 3)

    def test_valid_state_service_data_is_mapped_to_base_link(self):
        payload = {
            "source": "real",
            "kwr75d": {"valid": True, "wrench": [1, 2, 3, 0.1, 0.2, 0.3]},
            "fr5": {
                "valid": True,
                "joint_velocity_deg_s": [0, 0, 0.2, 0, 0, 0],
                "tcp_pose_mm_deg": [100, 200, 300, 0, 0, 0],
            },
        }
        response = io.BytesIO(json.dumps(payload).encode())
        engine = ForceTensionEngine()
        source = FallbackSource(engine, {"state_service_url": "http://example.invalid"})
        with patch("force_demo.sources.urlopen", return_value=response):
            self.assertTrue(source._poll_state_service())
        state = engine.snapshot()
        self.assertTrue(state["connected"])
        self.assertEqual(state["source"], "state_service")
        self.assertEqual(state["frame_id"], "base_link")
        self.assertTrue(state["moving"])

    def test_stale_sdk_frame_is_rejected(self):
        class State:
            frame_cnt = 7
            ft_sensor_active = 1
            ft_sensor_data = [0.0] * 6
            actual_qd = [0.0] * 6
            tl_cur_pos = [0.0] * 6

        class Robot:
            robot_state_pkg = State()
            closed = False

            def CloseRPC(self):
                self.closed = True

        engine = ForceTensionEngine()
        source = FallbackSource(engine, {})
        robot = Robot()
        source._robot = robot
        source._last_sdk_frame = 7
        source._last_sdk_frame_change = time.monotonic() - 1.0
        self.assertFalse(source._poll_sdk())
        self.assertTrue(robot.closed)
        self.assertFalse(engine.snapshot()["connected"])


if __name__ == "__main__":
    unittest.main()
