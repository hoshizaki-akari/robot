import unittest

import numpy as np

from platform_a.clamp_planner import build_clamp_plan


class ClampPlannerTests(unittest.TestCase):
    def test_frozen_center_keeps_gripper_bias_after_contact_average(self):
        vision = {
            "valid": True,
            "heel_center_camera_mm": [0.0, 0.0, 480.0],
            "clamp_contact_a_camera_mm": [-25.0, 0.0, 480.0],
            "clamp_contact_b_camera_mm": [25.0, 0.0, 480.0],
            "heel_plane_normal_camera": [0.0, 0.0, -1.0],
        }
        fr5 = {
            "valid": True,
            "age_ms": 0,
            "flange_pose_mm_deg": [
                -218.12,
                -417.21,
                207.38,
                -88.63,
                2.63,
                127.64,
            ],
        }

        result = build_clamp_plan(vision, fr5)

        self.assertTrue(result["valid"])
        self.assertEqual(result["clamp_contact_surface_center_camera_mm"], [0.0, 0.0, 480.0])
        self.assertEqual(result["clamp_contact_center_camera_mm"], [0.0, 0.0, 515.0])
        surface = np.asarray(result["clamp_contact_surface_center_base_mm"], dtype=float)
        center = np.asarray(result["clamp_contact_center_base_mm"], dtype=float)
        self.assertAlmostEqual(float(np.linalg.norm(center - surface)), 35.0, places=1)


if __name__ == "__main__":
    unittest.main()
