#!/usr/bin/env python3
"""Fit operator-labelled real XYZ force samples; never enables robot motion."""

import argparse
import json
import math
from pathlib import Path

import numpy as np


def quaternion_xyzw(rotation):
    """Convert a proper rotation matrix using a stable largest-component branch."""
    r = np.asarray(rotation, dtype=float)
    trace = float(np.trace(r))
    if trace > 0:
        s = math.sqrt(trace + 1.0) * 2
        return [(r[2, 1] - r[1, 2]) / s, (r[0, 2] - r[2, 0]) / s,
                (r[1, 0] - r[0, 1]) / s, s / 4]
    i = int(np.argmax(np.diag(r)))
    j, k = (i + 1) % 3, (i + 2) % 3
    s = math.sqrt(1 + r[i, i] - r[j, j] - r[k, k]) * 2
    q = [0.0] * 4
    q[i] = s / 4
    q[j] = (r[i, j] + r[j, i]) / s
    q[k] = (r[i, k] + r[k, i]) / s
    q[3] = (r[k, j] - r[j, k]) / s
    return q


def fit_force_axes(baseline, samples):
    """Return a proper sensor rotation plus an explicit force convention sign."""
    if len(samples) not in (2, 3):
        raise ValueError('Two labelled X+/Y+ samples, or three XYZ samples required')
    bias = np.asarray(baseline['force_bias_sensor_n'], dtype=float)
    pose_q = np.asarray(baseline['tare_q_base_flange'], dtype=float)
    if (bias.shape != (3,) or pose_q.shape != (4,) or not np.isfinite(bias).all()
            or not np.isfinite(pose_q).all() or abs(np.linalg.norm(pose_q) - 1) > 0.001):
        raise ValueError('Invalid unloaded reference')
    columns = []
    for sample in samples:
        if sample.get('source') != 'REAL_ROS_SUBSCRIPTIONS_NO_COMMANDS':
            raise ValueError('Only recorded real ROS data may be fitted')
        vector = np.asarray(sample['force_bias_sensor_n'], dtype=float) - bias
        q = np.asarray(sample['tare_q_base_flange'], dtype=float)
        if vector.shape != (3,) or not np.isfinite(vector).all() or q.shape != (4,):
            raise ValueError('Invalid force sample')
        if not np.isfinite(q).all() or abs(np.linalg.norm(q) - 1) > 0.001:
            raise ValueError('Invalid measured quaternion')
        angle = 2 * math.acos(float(np.clip(abs(q @ pose_q) /
                                         (np.linalg.norm(q) * np.linalg.norm(pose_q)), 0, 1)))
        magnitude = float(np.linalg.norm(vector))
        noise = np.asarray(sample['standard_deviation_n_nm'][:3], dtype=float)
        if (angle > 0.01 or magnitude < 1.5 or magnitude > 15 or
                noise.shape != (3,) or not np.isfinite(noise).all() or
                np.linalg.norm(noise) > 0.15 * magnitude):
            raise ValueError('Pose changed, force too small/large, or force not steady')
        columns.append(vector / magnitude)
    z_derived = len(columns) == 2
    if z_derived:
        first, second = columns
        if abs(float(first @ second)) > 0.45:
            raise ValueError('X+/Y+ labels too far from orthogonal')
        middle = (first + second) / np.linalg.norm(first + second)
        difference = (first - second) / np.linalg.norm(first - second)
        x_axis = (middle + difference) / math.sqrt(2)
        y_axis = (middle - difference) / math.sqrt(2)
        z_axis = np.cross(x_axis, y_axis)
        basis = np.column_stack((x_axis, y_axis, z_axis))
    else:
        basis = np.column_stack(columns)
    determinant = float(np.linalg.det(basis))
    if abs(determinant) < 0.8:
        raise ValueError('Labelled axes are not sufficiently independent/orthogonal')
    force_sign = 1.0 if determinant > 0 else -1.0
    u, _, vt = np.linalg.svd(force_sign * basis)
    rotation = (u @ vt).T
    if np.linalg.det(rotation) < 0.999:
        raise ValueError('Fit did not produce a proper rotation')
    errors = [math.degrees(math.acos(float(np.clip(
        (rotation @ (force_sign * columns[i]))[i], -1, 1)))) for i in range(len(columns))]
    if max(errors) > 12:
        raise ValueError('Labelled axes disagree by more than 12 degrees')
    return {
        'q_flange_sensor_candidate': [float(v) for v in quaternion_xyzw(rotation)],
        'force_sensor_sign_candidate': [force_sign] * 3,
        'axis_fit_error_degrees': errors,
        'z_axis_derived_not_force_verified': z_derived,
        'stepwise_drag_axis_mask': [1.0, 1.0, 0.0] if z_derived else [1.0, 1.0, 1.0],
        'mounting_verified': False,
        'drag_axes_verified': False,
        'note': 'Candidate assumes operator tool axes equal measured flange axes; physical check pending',
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('baseline', type=Path)
    parser.add_argument('xyz', nargs='+', type=Path)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    result = fit_force_axes(json.loads(args.baseline.read_text(encoding='utf-8')),
                            [json.loads(path.read_text(encoding='utf-8')) for path in args.xyz])
    args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
    print(json.dumps(result, indent=2))


if __name__ == '__main__':
    main()
