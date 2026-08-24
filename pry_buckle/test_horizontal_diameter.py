from __future__ import annotations

import unittest

import cv2
import numpy as np

from pry_buckle.horizontal_diameter import CameraIntrinsics, HorizontalDiameterEstimator


class HorizontalDiameterTests(unittest.TestCase):
    def setUp(self) -> None:
        self.intrinsics = CameraIntrinsics(fx=600.0, fy=600.0, cx=160.0, cy=120.0)
        self.estimator = HorizontalDiameterEstimator()

    @staticmethod
    def _heel(width_px: int) -> np.ndarray:
        mask = np.zeros((240, 320), dtype=np.uint8)
        cv2.ellipse(mask, (160, 120), (width_px // 2, 46), 0, 0, 360, 255, -1)
        return mask.astype(bool)

    def test_55mm_horizontal_diameter_is_accepted(self) -> None:
        # At 600 mm with fx=600, a 55 px horizontal span is 55 mm.
        mask = self._heel(55)
        depth = np.full(mask.shape, 600.0, dtype=np.float32)
        result = self.estimator.estimate(mask, depth, self.intrinsics)
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["width_mm"], 55.0, delta=2.0)
        self.assertEqual(result["contact_left_px"][1], result["contact_right_px"][1])

    def test_106mm_is_rejected_not_silently_corrected(self) -> None:
        mask = self._heel(106)
        depth = np.full(mask.shape, 600.0, dtype=np.float32)
        result = self.estimator.estimate(mask, depth, self.intrinsics)
        self.assertFalse(result["valid"])
        self.assertGreater(result["width_mm"], 100.0)

    def test_metre_depth_is_converted_once(self) -> None:
        mask = self._heel(55)
        depth_m = np.full(mask.shape, 0.6, dtype=np.float32)
        result = self.estimator.estimate(mask, depth_m, self.intrinsics)
        self.assertTrue(result["valid"])
        self.assertAlmostEqual(result["foreground_layer_depth_mm"], 600.0, delta=0.1)


if __name__ == "__main__":
    unittest.main()
