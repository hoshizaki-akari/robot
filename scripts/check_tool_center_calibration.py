#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from platform_a.handeye_calibration import pose_to_matrix  # noqa: E402
from platform_a.tool_center_calibration import (  # noqa: E402
    solve_tool_center,
    validate_tool_center,
)


def pose_for_fixed_point(rpy_deg: list[float], offset_mm: np.ndarray, point_mm: np.ndarray) -> list[float]:
    rotation = pose_to_matrix([0.0, 0.0, 0.0, *rpy_deg])[:3, :3]
    flange_mm = point_mm - rotation @ offset_mm
    return [*flange_mm.tolist(), *rpy_deg]


def main() -> int:
    true_offset = np.asarray([1.8, -2.4, 219.2], dtype=np.float64)
    fixed_point = np.asarray([245.0, -248.0, 98.0], dtype=np.float64)
    orientations = [
        [-178.0, 0.0, -144.0],
        [-168.0, 0.0, -144.0],
        [-188.0, 0.0, -144.0],
        [-178.0, 10.0, -144.0],
        [-178.0, -10.0, -144.0],
        [-170.0, 8.0, -150.0],
    ]
    poses = [pose_for_fixed_point(value, true_offset, fixed_point) for value in orientations]
    result = solve_tool_center(poses)
    solved = np.asarray(result["flange_to_gripper_center_mm"], dtype=np.float64)
    assert np.linalg.norm(solved - true_offset) < 1e-3
    assert result["fit_max_error_mm"] < 1e-3

    validation_orientations = [
        [-186.0, 6.0, -138.0],
        [-170.0, -7.0, -148.0],
        [-182.0, -9.0, -140.0],
    ]
    validation_poses = [
        pose_for_fixed_point(value, true_offset, fixed_point)
        for value in validation_orientations
    ]
    validation = validate_tool_center(validation_poses, solved, fixed_point)
    assert validation["passed"]
    print("通过：多姿态尖锥标定能够反算夹爪中心三个方向的偏移。")
    print("通过：独立的新姿势能够检查标定结果，不会拿训练数据冒充验收。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
