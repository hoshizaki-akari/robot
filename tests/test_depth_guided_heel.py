import unittest

import numpy as np

from pry_buckle.depth_guided_heel import (
    build_target_display_mask,
    estimate_depth_guided_target_chord,
    extract_depth_heel_candidate,
    refine_target_chord_at_center,
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
        self.assertEqual(result["selected_chord_y_px"], 122)
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

        refined = refine_target_chord_at_center(
            mask,
            depth,
            CameraIntrinsics(fx=300.0, fy=300.0, cx=212.0, cy=120.0),
            result,
        )
        self.assertTrue(refined["valid"])
        self.assertGreaterEqual(refined["width_mm"], 50.0)
        self.assertLessEqual(refined["width_mm"], 60.0)
        self.assertEqual(refined["center_px"][1], result["center_px"][1])

    def test_rotated_display_pixels_are_unrolled_for_camera_geometry(self):
        from pry_buckle.horizontal_diameter import HorizontalDiameterEstimator

        intrinsics = CameraIntrinsics(
            fx=300.0, fy=300.0, cx=212.0, cy=120.0,
            image_width=424, image_height=240, image_rotation_deg=180,
        )
        plane = np.asarray([0.0, 0.0, 500.0, 0.0, 0.0, -1.0])
        point = HorizontalDiameterEstimator._ray_plane((178, 127), plane, intrinsics)
        self.assertIsNotNone(point)
        # Display pixel (178,127) maps to native pixel (245,112).
        self.assertAlmostEqual(float(point[0]), 55.0, places=5)
        self.assertAlmostEqual(float(point[1]), -13.333333, places=5)
        self.assertAlmostEqual(float(point[2]), 500.0, places=5)

    def test_depth_fallback_center_uses_upper_heel_not_lower_right_extension(self):
        depth = np.full((240, 424), 800.0, dtype=np.float32)
        depth[70:130, 180:214] = 500.0
        depth[130:235, 180:260] = 500.0
        mask, _ = extract_depth_heel_candidate(depth)
        self.assertIsNotNone(mask)

        result = estimate_depth_guided_target_chord(
            mask,
            depth,
            CameraIntrinsics(fx=300.0, fy=300.0, cx=212.0, cy=120.0),
        )

        self.assertTrue(result["valid"])
        self.assertLessEqual(result["center_px"][0], 200)
        self.assertLess(
            result["upper_midpoint_px"][1], result["center_px"][1]
        )


if __name__ == "__main__":
    unittest.main()
