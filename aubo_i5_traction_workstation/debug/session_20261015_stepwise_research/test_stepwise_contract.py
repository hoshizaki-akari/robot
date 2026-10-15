"""Pure scheduling/math contract tests, NOT a robot or SDK simulator.

These tests validate the proposed invariants only, not hardware timing or
closed-loop force stability. No sockets, serial ports, ROS, or SDK calls.
"""
import math
import unittest
from dataclasses import dataclass


def bounded_step(velocity, horizon=0.1, maximum=0.0005):
    if len(velocity) != 3 or not all(math.isfinite(x) for x in velocity):
        raise ValueError("invalid velocity")
    if not all(math.isfinite(x) and x > 0 for x in (horizon, maximum)):
        raise ValueError("invalid limits")
    delta = tuple(x * horizon for x in velocity)
    distance = math.hypot(*delta)
    scale = min(1.0, maximum / distance) if distance else 1.0
    return tuple(x * scale for x in delta)


def force_step(error, measured_gradient, gain=0.25, maximum=0.0002):
    if not all(math.isfinite(x) for x in (error, measured_gradient, gain, maximum)):
        raise ValueError("nonfinite force input")
    if abs(measured_gradient) < 1e-9 or not 0 < gain <= 1 or maximum <= 0:
        raise ValueError("unverified gradient or invalid limits")
    return max(-maximum, min(maximum, gain * error / measured_gradient))


@dataclass
class Gate:
    pending: bool = False
    acknowledged: bool = False
    fault: bool = False
    fresh_stopped_frames: int = 0
    last_sequence: int = -1

    def begin(self):
        if self.pending or self.fault:
            return False
        self.pending = True
        self.acknowledged = False
        self.fresh_stopped_frames = 0
        return True

    def acknowledge(self, success=True):
        if not success:
            self.fault = True
        elif not self.fault:
            self.acknowledged = True

    def feedback(self, sequence, fresh, stopped, at_target, quiet):
        if not self.pending or self.fault or not self.acknowledged:
            return False
        if sequence <= self.last_sequence:
            return False
        self.last_sequence = sequence
        confirmed = fresh and stopped and at_target and quiet
        self.fresh_stopped_frames = self.fresh_stopped_frames + 1 if confirmed else 0
        if self.fresh_stopped_frames >= 3:
            self.pending = False
            return True
        return False

    def timeout_or_stop(self):
        # An ambiguous acceptance or stop never permits automatic resending.
        self.fault = True


class ContractTests(unittest.TestCase):
    def test_no_second_command_while_pending(self):
        gate = Gate()
        self.assertTrue(gate.begin())
        self.assertFalse(gate.begin())

    def test_ack_is_not_completion(self):
        gate = Gate()
        gate.begin()
        gate.acknowledge()
        self.assertFalse(gate.begin())

    def test_feedback_before_ack_cannot_complete(self):
        gate = Gate()
        gate.begin()
        for seq in range(5):
            self.assertFalse(gate.feedback(seq, True, True, True, True))
        self.assertTrue(gate.pending)

    def test_three_distinct_fresh_frames_required(self):
        gate = Gate()
        gate.begin()
        gate.acknowledge()
        self.assertFalse(gate.feedback(1, True, True, True, True))
        self.assertFalse(gate.feedback(1, True, True, True, True))
        self.assertFalse(gate.feedback(2, True, True, True, True))
        self.assertTrue(gate.feedback(3, True, True, True, True))
        self.assertTrue(gate.begin())

    def test_stationary_wrong_position_not_complete(self):
        gate = Gate()
        gate.begin()
        gate.acknowledge()
        for seq in range(5):
            self.assertFalse(gate.feedback(seq, True, True, False, True))

    def test_stale_or_moving_feedback_not_complete(self):
        for flags in ((False, True, True, True), (True, False, True, True),
                      (True, True, True, False)):
            gate = Gate()
            gate.begin()
            gate.acknowledge()
            for seq in range(5):
                self.assertFalse(gate.feedback(seq, *flags))

    def test_timeout_and_late_ack_do_not_enable_retry(self):
        gate = Gate()
        gate.begin()
        gate.timeout_or_stop()
        gate.acknowledge()
        self.assertFalse(gate.begin())
        self.assertTrue(gate.fault)

    def test_failed_ack_does_not_enable_retry(self):
        gate = Gate()
        gate.begin()
        gate.acknowledge(False)
        self.assertFalse(gate.begin())

    def test_vector_step_cap_not_per_axis_cap(self):
        delta = bounded_step((1.0, 1.0, 1.0))
        self.assertAlmostEqual(math.hypot(*delta), 0.0005)

    def test_both_axis_signs_and_zero(self):
        self.assertEqual(bounded_step((0.0, 0.0, 0.0)), (0.0, 0.0, 0.0))
        self.assertEqual(bounded_step((0.002, 0.0, 0.0)), (0.0002, 0.0, 0.0))
        self.assertEqual(bounded_step((-0.002, 0.0, 0.0)), (-0.0002, 0.0, 0.0))

    def test_force_gradient_sign_and_step_cap(self):
        self.assertGreater(force_step(1.0, 1000.0), 0)
        self.assertLess(force_step(1.0, -1000.0), 0)
        self.assertAlmostEqual(abs(force_step(100.0, 1000.0)), 0.0002)

    def test_reject_unknown_gradient_and_nonfinite_input(self):
        with self.assertRaises(ValueError):
            force_step(1.0, 0.0)
        with self.assertRaises(ValueError):
            bounded_step((math.nan, 0.0, 0.0))
        with self.assertRaises(ValueError):
            force_step(math.inf, 1000.0)


if __name__ == "__main__":
    unittest.main()
