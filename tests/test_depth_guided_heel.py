import unittest

import numpy as np

from pry_buckle.depth_guided_heel import (
    build_target_display_mask,
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
        self.assertEqual(result["selected_chord_y_px"], 90)
        self.assertEqual(
            result["contact_left_px"][0] + result["contact_right_px"][0],
            2 * result["center_px"][0],
        )

        display_mask = build_target_display_mask(
            mask,
            result["target_circle_center_px"],
            result["target_circle_radius_px"],
        )
        self.assertGreater(int(np.count_nonzero(display_mask)), 0)
        ys, xs = np.nonzero(display_mask)
        cx, cy = result["target_circle_center_px"]
        self.assertLessEqual(int(np.max((xs - cx) ** 2 + (ys - cy) ** 2)), result["target_circle_radius_px"] ** 2)


if __name__ == "__main__":
    unittest.main()
