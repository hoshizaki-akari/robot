"""Offline orchestration regression; no SDK, ROS connection or motion."""

import unittest
from unittest.mock import Mock, patch

from backend.ros_bridge import RosBridge, RosBridgeError


class FaultRecoveryTest(unittest.TestCase):
    def bridge(self):
        value = RosBridge()
        value.snapshot = Mock(return_value={"traction": {"state": 9, "valid": True}})
        return value

    def test_driver_then_manager_reset_without_start_or_prepare(self):
        bridge = self.bridge()
        bridge._call_service = Mock(return_value={"success": True})
        result = bridge._recover_fault()
        self.assertTrue(result["success"])
        self.assertEqual(["driver_recover_fault", "reset_fault"],
                         [call.args[0] for call in bridge._call_service.call_args_list])
        self.assertFalse(bridge._recovery_lock.locked())

    def test_pending_stop_and_health_propagation_are_bounded_retries(self):
        bridge = self.bridge()
        bridge._call_service = Mock(side_effect=[
            RosBridgeError("RECOVERY_PENDING: stop"), {"success": True},
            RosBridgeError("Reset rejected: AUBO must be stationary"), {"success": True},
        ])
        with patch("backend.ros_bridge.time.sleep"):
            self.assertTrue(bridge._recover_fault()["success"])
        self.assertEqual(["driver_recover_fault", "driver_recover_fault", "reset_fault", "reset_fault"],
                         [call.args[0] for call in bridge._call_service.call_args_list])

    def test_hard_error_never_resets_manager_and_lock_is_released(self):
        bridge = self.bridge()
        bridge._call_service = Mock(side_effect=RosBridgeError("Driver recovery failed"))
        with self.assertRaises(RosBridgeError):
            bridge._recover_fault()
        bridge._call_service.assert_called_once_with("driver_recover_fault")
        self.assertFalse(bridge._recovery_lock.locked())

    def test_no_parallel_recovery_no_emergency_bypass(self):
        bridge = self.bridge()
        bridge._call_service = Mock()
        bridge.snapshot.return_value["traction"]["state"] = 10
        with self.assertRaises(RosBridgeError):
            bridge._recover_fault()
        bridge._call_service.assert_not_called()
        bridge._recovery_lock.acquire()
        with self.assertRaises(RosBridgeError):
            bridge._recover_fault()
        bridge._recovery_lock.release()

    def test_pending_stop_cannot_poll_forever(self):
        bridge = self.bridge()
        bridge._call_service = Mock(side_effect=RosBridgeError("RECOVERY_PENDING: stop"))
        with patch("backend.ros_bridge.time.monotonic", side_effect=[0.0, 0.1, 9.0]), \
                patch("backend.ros_bridge.time.sleep"), self.assertRaises(RosBridgeError):
            bridge._recover_fault()
        self.assertFalse(bridge._recovery_lock.locked())
