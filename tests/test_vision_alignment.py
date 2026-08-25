from __future__ import annotations

import unittest

import cv2
import numpy as np

from pry_buckle.heel_geometry import HeelGeometryEstimator
from pry_buckle.horizontal_diameter import CameraIntrinsics
from pry_buckle.measurement_stabilizer import MeasurementStabilizer
from scripts.clamp_auto_align import build_alignment_delta


class VisionAlignmentTests(unittest.TestCase):
    def _frame(self, angle_deg: float = 0.0, width_axis_px: int = 28) -> tuple[np.ndarray, np.ndarray, CameraIntrinsics]:
        mask = np.zeros((220, 260), dtype=np.uint8)
        cv2.ellipse(mask, (130, 110), (65, width_axis_px // 2), angle_deg, 0, 360, 1, -1)
        depth = np.full(mask.shape, 1000.0, dtype=np.float32)
        return mask.astype(bool), depth, CameraIntrinsics(500.0, 500.0, 130.0, 110.0)

    def test_principal_axis_width_survives_rotation(self) -> None:
        estimator = HeelGeometryEstimator()
        for angle in (-15.0, -5.0, 0.0, 10.0, 15.0):
            mask, depth, intrinsics = self._frame(angle)
            result = estimator.estimate(mask, depth, intrinsics)
            self.assertTrue(result["geometry_valid"], result["message"])
            self.assertTrue(result["within_expected_width_range"], result)
            self.assertLess(abs(float(result["width_mm"]) - 56.0), 6.0, result)

    def test_stabilizer_requires_fresh_stable_frames(self) -> None:
        stabilizer = MeasurementStabilizer()
        result = None
        for seq in range(1, 9):
            result = stabilizer.update({
                "valid": True,
                "geometry_valid": True,
                "within_expected_width_range": True,
                "width_mm": 56.0 + (0.1 if seq % 2 else -0.1),
                "center_camera_mm": [0.0, 0.0, 1000.0],
                "principal_angle_deg": 1.0,
                "depth_valid_ratio": 0.95,
            }, seq, float(seq))
        assert result is not None
        self.assertTrue(result["motion_grade"], result)
        held = dict(result)
        held["valid"] = False
        held["motion_grade"] = False
        held["display_only"] = True
        held["measurement_status"] = "held_last_valid_result"
        self.assertFalse(held["motion_grade"])

    def test_stable_out_of_range_is_not_motion_grade(self) -> None:
        stabilizer = MeasurementStabilizer()
        for seq in range(1, 9):
            result = stabilizer.update({
                "valid": False,
                "geometry_valid": True,
                "within_expected_width_range": False,
                "width_mm": 70.0,
                "center_camera_mm": [0.0, 0.0, 1000.0],
                "principal_angle_deg": 1.0,
                "depth_valid_ratio": 0.95,
            }, seq, float(seq))
        self.assertTrue(result["stable_valid"], result)
        self.assertFalse(result["motion_grade"], result)
        self.assertFalse(result["valid"], result)

    def test_alignment_delta_is_bounded(self) -> None:
        plan = {
            "valid": True,
            "motion_grade": True,
            "display_only": False,
            "measurement_status": "stabilized",
            "center_px": [322, 240],
            "image_width": 640,
            "image_height": 480,
            "center_camera_mm": [0.0, 0.0, 1000.0],
            "principal_angle_deg": 1.0,
            "color_intrinsics": {"fx": 500.0, "fy": 500.0},
        }
        result = build_alignment_delta(plan, {})
        self.assertTrue(result["safe_by_geometry"], result)
        self.assertAlmostEqual(result["predicted_translation_mm"], 4.0, places=3)
        self.assertAlmostEqual(result["predicted_rotation_deg"], 1.0, places=3)

        plan["center_px"] = [340, 240]
        result = build_alignment_delta(plan, {})
        self.assertFalse(result["safe_by_geometry"], result)


if __name__ == "__main__":
    unittest.main()
