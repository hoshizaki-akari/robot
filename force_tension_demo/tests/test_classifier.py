from __future__ import annotations

import unittest

from force_demo.classifier import (
    EngineConfig,
    ForceTensionEngine,
    SensorSample,
    direction,
    dominant_axis,
)


class Clock:
    def __init__(self) -> None:
        self.value = 0.0

    def __call__(self) -> float:
        return self.value

    def step(self, seconds: float = 0.02) -> float:
        self.value += seconds
        return self.value


class EngineTest(unittest.TestCase):
    def setUp(self) -> None:
        self.clock = Clock()
        config = EngineConfig(
            baseline_duration_s=0.2,
            settle_after_motion_s=0.1,
            tension_confirm_s=0.1,
            slack_confirm_s=0.1,
            transition_confirm_s=0.05,
            stability_window_s=0.08,
            filter_cutoff_hz=20.0,
            data_stale_s=0.5,
        )
        self.engine = ForceTensionEngine(config=config, clock=self.clock)

    def feed(self, force, *, moving=False, source="ros2", detail="/wrench", priority=30, count=1, orientation=None):
        for _ in range(count):
            self.clock.step()
            self.engine.ingest(SensorSample.create(
                force,
                monotonic_time=self.clock.value,
                wall_time=f"t={self.clock.value:.2f}",
                source=source,
                source_detail=detail,
                priority=priority,
                motion_available=True,
                max_joint_speed_deg_s=1.0 if moving else 0.0,
                tcp_rpy_deg=orientation,
            ))

    def establish_baseline(self, baseline=(1.0, 2.0, 3.0)):
        self.feed(baseline)
        success, _ = self.engine.begin_baseline()
        self.assertTrue(success)
        self.feed(baseline, count=12)
        self.assertTrue(self.engine.snapshot()["baseline_ready"])

    def test_baseline_then_slack(self):
        self.establish_baseline()
        self.feed((1.0, 2.0, 3.0), count=14)
        state = self.engine.snapshot()
        self.assertEqual(state["phase"], "slack")
        self.assertLess(state["resultant_force_n"], 0.05)
        self.assertIsNone(state["actual_force_direction"])

    def test_rebaseline_resets_previous_confirmed_state(self):
        self.establish_baseline()
        self.feed((1.0, 2.0, 3.0), count=14)
        self.assertEqual(self.engine.snapshot()["confirmed_state"], "slack")
        success, _ = self.engine.begin_baseline()
        self.assertTrue(success)
        state = self.engine.snapshot(include_history=True)
        self.assertEqual(state["confirmed_state"], "unknown")
        self.assertEqual(state["resultant_force_n"], 0.0)
        self.assertEqual(state["history"], [])

    def test_cannot_zero_away_a_confirmed_tension(self):
        self.establish_baseline((0.0, 0.0, 0.0))
        self.feed((2.0, 0.0, 0.0), count=20)
        self.assertEqual(self.engine.snapshot()["confirmed_state"], "tension")
        success, message = self.engine.begin_baseline()
        self.assertFalse(success)
        self.assertIn("先真正松绳", message)

    def test_motion_force_is_not_classified_as_tension(self):
        self.establish_baseline((0.0, 0.0, 0.0))
        self.feed((5.0, 0.0, 0.0), moving=True, count=20)
        self.assertEqual(self.engine.snapshot()["phase"], "moving")
        self.feed((0.0, 0.0, 0.0), moving=False, count=18)
        state = self.engine.snapshot()
        self.assertEqual(state["phase"], "slack")
        self.assertEqual(state["confirmed_state"], "slack")

    def test_slack_state_does_not_flicker_inside_hysteresis_band(self):
        self.establish_baseline((0.0, 0.0, 0.0))
        self.feed((0.0, 0.0, 0.0), count=14)
        self.assertEqual(self.engine.snapshot()["phase"], "slack")
        for value in (0.45, 0.62, 0.52, 0.74, 0.48, 0.68):
            self.feed((value, 0.0, 0.0), count=3)
            self.assertEqual(self.engine.snapshot()["phase"], "slack")
            self.assertIsNone(self.engine.snapshot()["actual_force_direction"])

    def test_persistent_force_becomes_tension_after_stopping(self):
        self.establish_baseline((0.0, 0.0, 0.0))
        self.feed((2.0, 0.0, 0.0), moving=True, count=10)
        self.feed((2.0, 0.0, 0.0), moving=False, count=18)
        state = self.engine.snapshot()
        self.assertEqual(state["phase"], "tension")
        self.assertEqual(state["actual_force_axis"], "X+")
        self.assertEqual(state["increase_motion_axis"], "X-")

    def test_over_ten_newton_warning_is_immediate(self):
        self.establish_baseline((0.0, 0.0, 0.0))
        self.feed((11.2, 0.0, 0.0), count=3)
        self.assertTrue(self.engine.snapshot()["warning"])

    def test_source_change_invalidates_baseline(self):
        self.establish_baseline()
        self.feed((1.0, 2.0, 3.0), source="state_service", detail="RCS", priority=40)
        state = self.engine.snapshot()
        self.assertFalse(state["baseline_ready"])
        self.assertIn("来源发生变化", state["baseline_message"])
        self.assertEqual(state["resultant_force_n"], 0.0)
        self.assertEqual(self.engine.snapshot(include_history=True)["history"], [])

    def test_lower_priority_cannot_override_fresh_source(self):
        self.feed((0.0, 0.0, 0.0), priority=30)
        accepted = self.engine.ingest(SensorSample.create(
            (9.0, 0.0, 0.0), monotonic_time=self.clock.step(),
            source="sdk", source_detail="direct", priority=10,
        ))
        self.assertFalse(accepted)
        self.assertEqual(self.engine.snapshot()["source"], "ros2")

    def test_non_base_frame_is_rejected(self):
        accepted = self.engine.ingest(SensorSample.create(
            (1.0, 0.0, 0.0), monotonic_time=self.clock.step(),
            frame_id="tool0", source="bad", priority=99,
        ))
        self.assertFalse(accepted)
        self.assertIn("非 base_link", self.engine.snapshot()["last_error"])

    def test_invalid_threshold_configuration_is_rejected(self):
        with self.assertRaises(ValueError):
            EngineConfig(slack_threshold_n=1.0, tension_threshold_n=0.5)

    def test_non_finite_sensor_sample_is_rejected(self):
        with self.assertRaises(ValueError):
            SensorSample.create((float("nan"), 0.0, 0.0))

    def test_stale_data_clears_live_warning_and_shows_disconnected(self):
        self.establish_baseline((0.0, 0.0, 0.0))
        self.feed((12.0, 0.0, 0.0), count=3)
        self.assertTrue(self.engine.snapshot()["warning"])
        self.clock.step(0.6)
        state = self.engine.snapshot()
        self.assertFalse(state["connected"])
        self.assertFalse(state["warning"])
        self.assertEqual(state["phase"], "disconnected")

    def test_csv_failure_does_not_stop_live_processing(self):
        class BrokenRecorder:
            def write(self, row):
                raise OSError("disk full")

        engine = ForceTensionEngine(config=self.engine.config, recorder=BrokenRecorder(), clock=self.clock)
        accepted = engine.ingest(SensorSample.create(
            (1.0, 2.0, 3.0), monotonic_time=self.clock.step(),
            source="ros2", source_detail="/wrench", priority=30,
        ))
        self.assertTrue(accepted)
        self.assertTrue(engine.snapshot()["connected"])
        self.assertIn("CSV 记录失败", engine.snapshot()["last_error"])

    def test_orientation_change_pauses_classification(self):
        baseline = (0.0, 0.0, 0.0)
        self.feed(baseline, orientation=(10.0, 20.0, 30.0))
        success, _ = self.engine.begin_baseline()
        self.assertTrue(success)
        self.feed(baseline, count=12, orientation=(10.0, 20.0, 30.0))
        self.assertTrue(self.engine.snapshot()["baseline_ready"])
        self.feed((2.0, 0.0, 0.0), count=15, orientation=(12.0, 20.0, 30.0))
        state = self.engine.snapshot()
        self.assertEqual(state["phase"], "orientation_changed")
        self.assertGreater(state["orientation_change_deg"], 1.0)

    def test_all_six_base_axes_and_opposites(self):
        cases = [
            ((2.0, 0.0, 0.0), "X+", "X-"), ((-2.0, 0.0, 0.0), "X-", "X+"),
            ((0.0, 2.0, 0.0), "Y+", "Y-"), ((0.0, -2.0, 0.0), "Y-", "Y+"),
            ((0.0, 0.0, 2.0), "Z+", "Z-"), ((0.0, 0.0, -2.0), "Z-", "Z+"),
        ]
        for vector, measured_label, increase_label in cases:
            with self.subTest(vector=vector):
                measured = direction(vector, 1.0)
                self.assertEqual(dominant_axis(measured), measured_label)
                increase = tuple(-value for value in measured)
                self.assertEqual(dominant_axis(increase), increase_label)


if __name__ == "__main__":
    unittest.main()
