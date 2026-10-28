import unittest
from unittest.mock import Mock, patch

from backend.ros_bridge import RosBridge, RosBridgeError


class EnterModeTest(unittest.TestCase):
    def setup_bridge(self, state=11, mode=2):
        bridge = RosBridge()
        data = {
            "connected": True,
            "traction": {"state": state, "operation_mode": mode, "ready": False},
            "fr5": {"joint_velocity_rad_s": [0.0] * 6},
        }
        bridge.snapshot = Mock(side_effect=lambda: data)
        bridge.call = Mock()
        bridge.set_operation_mode = Mock()
        return bridge, data

    def test_entering_waits_for_native_stop_ack_and_never_starts(self):
        bridge, data = self.setup_bridge()
        order = []

        def stop(name):
            order.append(name)
            data["traction"].update(state=1, ready=False)

        def confirmed_stop(_delay):
            data["traction"]["ready"] = True

        def select(mode):
            self.assertTrue(data["traction"]["ready"])
            order.append("mode")
            data["traction"]["operation_mode"] = mode

        bridge.call.side_effect = stop
        bridge.set_operation_mode.side_effect = select
        with patch("backend.ros_bridge.time.sleep", side_effect=confirmed_stop):
            result = bridge.enter_operation_mode(1)
        self.assertEqual(["stop", "mode"], order)
        self.assertTrue(result["success"])
        self.assertEqual(1, result["snapshot"]["traction"]["operation_mode"])

    def test_stop_failure_does_not_select_or_start(self):
        bridge, _data = self.setup_bridge()
        bridge.call.side_effect = RosBridgeError("stop unconfirmed")
        with self.assertRaisesRegex(RosBridgeError, "stop unconfirmed"):
            bridge.enter_operation_mode(1)
        bridge.set_operation_mode.assert_not_called()
        bridge.call.assert_called_once_with("stop")

    def test_estop_during_transition_cancels_select(self):
        bridge, data = self.setup_bridge(state=7, mode=3)
        bridge.call.side_effect = lambda _: data["traction"].update(state=10)
        with self.assertRaises(RosBridgeError):
            bridge.enter_operation_mode(2)
        bridge.set_operation_mode.assert_not_called()

    def test_stopped_entry_only_selects_and_invalid_modes_are_rejected(self):
        for state in (1, 5):
            with self.subTest(state=state):
                bridge, data = self.setup_bridge(state=state, mode=3)
                data["traction"]["ready"] = True
                bridge.set_operation_mode.side_effect = (
                    lambda mode, current=data: current["traction"].update(operation_mode=mode)
                )
                self.assertTrue(bridge.enter_operation_mode(2)["success"])
                bridge.call.assert_not_called()
                with self.assertRaises(RosBridgeError):
                    bridge.enter_operation_mode(3)
