import unittest

import numpy as np

from pry_buckle.depth_guided_heel import (
    estimate_depth_guided_target_chord,
    extract_depth_heel_candidate,
)
from pry_buckle.horizontal_diameter import CameraIntrinsics


class DepthGuidedHeelTests(unittest.TestCase):
    def test_selects_central_near_component_and_rejects_far_shelf(self):
        depth = np.full((240, 424), 800.0, dtype=np.float32)
        depth[70:235, 180:214] = 500.0
        depth[60:235, 300:424] = 720.0

        mask, diagnostics = extract_depth_heel_candidate(depth)

        self.assertIsNotNone(mask)
        self.assertEqual(diagnostics["component_bbox_px"][:2], [180, 70])
        self.assertLess(diagnostics["near_depth_threshold_mm"], 740.0)

    def test_target_chord_is_in_requested_width_range(self):
        depth = np.full((240, 424), 800.0, dtype=np.float32)
        depth[70:235, 180:214] = 500.0
        mask, _ = extract_depth_heel_candidate(depth)
        self.assertIsNotNone(mask)

        result = estimate_depth_guided_target_chord(
            mask,
            depth,
            CameraIntrinsics(fx=300.0, fy=300.0, cx=212.0, cy=120.0),
        )

        self.assertTrue(result["valid"])
        self.assertGreaterEqual(result["width_mm"], 50.0)
        self.assertLessEqual(result["width_mm"], 60.0)
        self.assertEqual(result["contact_left_px"][1], result["contact_right_px"][1])


if __name__ == "__main__":
    unittest.main()
