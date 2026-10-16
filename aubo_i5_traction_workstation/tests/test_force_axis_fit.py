"""Offline numerical tests only; never uses ROS or hardware."""

import copy
import runpy
import unittest
from pathlib import Path

import numpy as np

_module = runpy.run_path(str(Path(__file__).resolve().parents[1] / 'scripts/fit_force_axes.py'))
fit_force_axes = _module['fit_force_axes']
quaternion_xyzw = _module['quaternion_xyzw']


class ForceAxisFitTest(unittest.TestCase):
    def setUp(self):
        self.baseline = {
            'force_bias_sensor_n': [1, 2, 3],
            'tare_q_base_flange': [0, 0, 0, 1],
        }
        self.rotation = np.array([[0, -1, 0], [1, 0, 0], [0, 0, 1]])

    def samples(self, sign=1):
        return [{
            'source': 'REAL_ROS_SUBSCRIPTIONS_NO_COMMANDS',
            'force_bias_sensor_n': (np.array([1, 2, 3]) +
                                    sign * self.rotation.T[:, i] * 4).tolist(),
            'tare_q_base_flange': [0, 0, 0, 1],
            'standard_deviation_n_nm': [0.01] * 6,
        } for i in range(3)]

    def test_rotation_and_convention_sign(self):
        for sign in (-1, 1):
            result = fit_force_axes(self.baseline, self.samples(sign))
            self.assertEqual(result['force_sensor_sign_candidate'], [sign] * 3)
            np.testing.assert_allclose(result['q_flange_sensor_candidate'],
                                       [0, 0, 2 ** -0.5, 2 ** -0.5], atol=1e-12)
            self.assertFalse(result['mounting_verified'])
            self.assertFalse(result['drag_axes_verified'])

    def test_nonindependent_forces_are_rejected(self):
        samples = self.samples()
        samples[1] = copy.deepcopy(samples[0])
        with self.assertRaises(ValueError):
            fit_force_axes(self.baseline, samples)

    def test_two_axes_are_orthogonalized_and_unmeasured_z_is_disabled(self):
        samples = self.samples()[:2]
        samples[1]['force_bias_sensor_n'][1] += 0.4
        result = fit_force_axes(self.baseline, samples)
        self.assertTrue(result['z_axis_derived_not_force_verified'])
        self.assertEqual(result['stepwise_drag_axis_mask'], [1, 1, 0])
        self.assertLess(max(result['axis_fit_error_degrees']), 12)
        self.assertAlmostEqual(np.linalg.norm(result['q_flange_sensor_candidate']), 1)

    def test_pose_change_and_noisy_force_are_rejected(self):
        for change in ('pose', 'noise'):
            samples = self.samples()
            if change == 'pose':
                samples[0]['tare_q_base_flange'] = [0, 0, 0.1, (1 - 0.01) ** 0.5]
            else:
                samples[0]['standard_deviation_n_nm'] = [1] * 6
            with self.assertRaises(ValueError):
                fit_force_axes(self.baseline, samples)

    def test_pi_rotations_convert_without_trace_division_failure(self):
        for diagonal, expected in (([1, -1, -1], [1, 0, 0, 0]),
                                   ([-1, 1, -1], [0, 1, 0, 0]),
                                   ([-1, -1, 1], [0, 0, 1, 0])):
            np.testing.assert_allclose(quaternion_xyzw(np.diag(diagonal)), expected)


if __name__ == '__main__':
    unittest.main()
