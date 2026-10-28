import unittest
from unittest.mock import Mock

from backend.auto_recovery import RECOVERABLE_FAULTS, TransientFaultRecovery
from backend.ros_bridge import RosBridgeError


def snapshot(code="WRENCH_TIMEOUT", state=9, connected=True):
    return {
        "connected": connected,
        "traction": {"state": state, "fault_code": code, "calibrated": True},
        "fr5": {"joint_velocity_rad_s": [0.0] * 6},
    }


class AutoRecoveryTest(unittest.TestCase):
    def test_only_software_timeouts_recover_without_start_or_hardware_reset(self):
        for code in RECOVERABLE_FAULTS:
            with self.subTest(code=code):
                bridge = Mock()
                bridge.snapshot.return_value = snapshot(code)
                recovery = TransientFaultRecovery()
                recovery.poll(bridge, 10)
                bridge.call.assert_not_called()
                recovery.poll(bridge, 11.1)
                self.assertEqual(
                    ["auto_recovery_check", "reset_transient_fault"],
                    [call.args[0] for call in bridge.call.call_args_list],
                )
                recovery.poll(bridge, 15)
                self.assertEqual(2, bridge.call.call_count)

    def test_hardware_fault_estop_and_stop_failure_never_auto_recover(self):
        for code, state in (
            ("ROS2_CONTROL_ERROR", 9), ("CARTESIAN_STOP_FAILED", 9),
            ("SOFTWARE_EMERGENCY_STOP", 10), ("WRENCH_INVALID", 9),
        ):
            bridge = Mock()
            bridge.snapshot.return_value = snapshot(code, state)
            recovery = TransientFaultRecovery()
            recovery.poll(bridge, 0)
            recovery.poll(bridge, 10)
            bridge.call.assert_not_called()

    def test_disconnection_or_motion_restarts_stable_feedback_wait(self):
        bridge = Mock()
        data = snapshot()
        bridge.snapshot.return_value = data
        recovery = TransientFaultRecovery()
        recovery.poll(bridge, 0)
        data["connected"] = False
        recovery.poll(bridge, 1)
        data["connected"] = True
        data["fr5"]["joint_velocity_rad_s"] = [0.1] * 6
        recovery.poll(bridge, 2)
        data["fr5"]["joint_velocity_rad_s"] = [0.0] * 6
        recovery.poll(bridge, 3)
        recovery.poll(bridge, 3.5)
        bridge.call.assert_not_called()
        recovery.poll(bridge, 4.1)
        self.assertEqual(2, bridge.call.call_count)

    def test_failed_passive_driver_check_never_clears_fault(self):
        bridge = Mock()
        bridge.snapshot.return_value = snapshot()
        bridge.call.side_effect = RosBridgeError("Physical protection still active")
        recovery = TransientFaultRecovery()
        recovery.poll(bridge, 0)
        for now in (1.1, 3.2, 5.3, 7.4, 10):
            recovery.poll(bridge, now)
        self.assertEqual(["auto_recovery_check"] * 3,
                         [call.args[0] for call in bridge.call.call_args_list])

    def test_new_estop_between_check_and_reset_is_not_acknowledged(self):
        bridge = Mock()
        data = snapshot()
        bridge.snapshot.return_value = data
        bridge.call.side_effect = lambda _: data["traction"].update(state=10)
        recovery = TransientFaultRecovery()
        recovery.poll(bridge, 0)
        recovery.poll(bridge, 1.1)
        bridge.call.assert_called_once_with("auto_recovery_check")
