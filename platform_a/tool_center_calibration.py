from __future__ import annotations

from itertools import combinations
from typing import Any, Sequence

import numpy as np

from .handeye_calibration import pose_to_matrix, rotation_angle_degrees


def matrix_to_rpy_degrees(rotation: np.ndarray) -> list[float]:
    """Inverse of the project's Rz @ Ry @ Rx pose convention."""
    sy = float(-rotation[2, 0])
    cy = float(np.sqrt(max(0.0, 1.0 - sy * sy)))
    if cy > 1e-8:
        rx = np.arctan2(rotation[2, 1], rotation[2, 2])
        ry = np.arctan2(sy, cy)
        rz = np.arctan2(rotation[1, 0], rotation[0, 0])
    else:
        rx = np.arctan2(-rotation[1, 2], rotation[1, 1])
        ry = np.arctan2(sy, cy)
        rz = 0.0
    return [round(float(np.degrees(value)), 6) for value in (rx, ry, rz)]


def flange_pose_for_center(
    center_base_mm: Sequence[float],
    rpy_deg: Sequence[float],
    offset_flange_mm: Sequence[float],
) -> list[float]:
    rotation = pose_to_matrix([0.0, 0.0, 0.0, *rpy_deg])[:3, :3]
    flange_mm = np.asarray(center_base_mm, dtype=np.float64) - rotation @ np.asarray(
        offset_flange_mm, dtype=np.float64
    )
    return [
        *[round(float(value), 6) for value in flange_mm],
        *[round(float(value), 6) for value in rpy_deg],
    ]


def solve_tool_center(flange_poses_mm_deg: Sequence[Sequence[float]]) -> dict[str, Any]:
    """Solve R_i * tool_center + flange_i = one fixed point."""
    if len(flange_poses_mm_deg) < 4:
        raise ValueError("至少需要4个已经重新对准尖锥的不同姿势")

    transforms = [pose_to_matrix(list(pose)) for pose in flange_poses_mm_deg]
    angle_spread = max(
        rotation_angle_degrees(a[:3, :3].T @ b[:3, :3])
        for a, b in combinations(transforms, 2)
    )
    if angle_spread < 15.0:
        raise ValueError("这些姿势的角度变化太小，至少需要相差15度")

    rows: list[np.ndarray] = []
    values: list[np.ndarray] = []
    for transform in transforms:
        rotation = transform[:3, :3]
        flange_position_mm = transform[:3, 3] * 1000.0
        rows.append(np.hstack((rotation, -np.eye(3, dtype=np.float64))))
        values.append(-flange_position_mm)
    matrix = np.vstack(rows)
    vector = np.concatenate(values)
    solution, _, rank, singular_values = np.linalg.lstsq(matrix, vector, rcond=None)
    if int(rank) < 6:
        raise ValueError("姿势方向不够丰富，无法唯一算出夹爪中心")

    offset_mm = solution[:3]
    fixed_point_mm = solution[3:]
    calculated_points = np.asarray(
        [
            transform[:3, :3] @ offset_mm + transform[:3, 3] * 1000.0
            for transform in transforms
        ],
        dtype=np.float64,
    )
    errors_mm = np.linalg.norm(calculated_points - fixed_point_mm, axis=1)
    condition_number = float(singular_values[0] / singular_values[-1])
    return {
        "flange_to_gripper_center_mm": [round(float(value), 4) for value in offset_mm],
        "fixed_cone_tip_base_mm": [round(float(value), 4) for value in fixed_point_mm],
        "sample_count": len(transforms),
        "angle_spread_deg": round(float(angle_spread), 3),
        "fit_rmse_mm": round(float(np.sqrt(np.mean(errors_mm**2))), 4),
        "fit_max_error_mm": round(float(np.max(errors_mm)), 4),
        "sample_errors_mm": [round(float(value), 4) for value in errors_mm],
        "condition_number": round(condition_number, 3),
        "validated": False,
    }


def point_from_pose(
    flange_pose_mm_deg: Sequence[float], offset_mm: Sequence[float]
) -> np.ndarray:
    transform = pose_to_matrix(list(flange_pose_mm_deg))
    return transform[:3, :3] @ np.asarray(offset_mm, dtype=np.float64) + transform[:3, 3] * 1000.0


def validate_tool_center(
    validation_poses_mm_deg: Sequence[Sequence[float]],
    offset_mm: Sequence[float],
    fixed_point_mm: Sequence[float],
) -> dict[str, Any]:
    if len(validation_poses_mm_deg) < 3:
        raise ValueError("独立验收至少需要3个新姿势")
    target = np.asarray(fixed_point_mm, dtype=np.float64)
    errors = [
        float(np.linalg.norm(point_from_pose(pose, offset_mm) - target))
        for pose in validation_poses_mm_deg
    ]
    maximum = max(errors)
    return {
        "sample_count": len(errors),
        "errors_mm": [round(value, 4) for value in errors],
        "rmse_mm": round(float(np.sqrt(np.mean(np.square(errors)))), 4),
        "max_error_mm": round(maximum, 4),
        "passed": maximum <= 3.0,
        "threshold_mm": 3.0,
    }
