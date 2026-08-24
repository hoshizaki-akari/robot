#!/usr/bin/env python3
from __future__ import annotations

import sys
from pathlib import Path

import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from platform_a.handeye_calibration import pose_to_matrix, rotation_angle_degrees, solve


def main() -> int:
    rng = np.random.default_rng(7)
    expected = pose_to_matrix([45, -22, 105, 3, -7, 12])
    fixed_board = pose_to_matrix([600, 100, 20, 0, 0, 0])
    samples = []
    for _ in range(20):
        pose = [
            350 + rng.uniform(-80, 80),
            rng.uniform(-120, 120),
            400 + rng.uniform(-80, 80),
            rng.uniform(-30, 30),
            rng.uniform(-35, 35),
            rng.uniform(-45, 45),
        ]
        base_t_flange = pose_to_matrix(pose)
        camera_t_board = np.linalg.inv(expected) @ np.linalg.inv(base_t_flange) @ fixed_board
        samples.append(
            {
                "flange_pose_mm_deg": pose,
                "camera_T_board": camera_t_board.tolist(),
            }
        )
    result = solve(samples)
    actual = np.asarray(result["flange_T_camera"], dtype=np.float64)
    translation_error_mm = float(np.linalg.norm(expected[:3, 3] - actual[:3, 3]) * 1000.0)
    rotation_error_deg = rotation_angle_degrees(expected[:3, :3].T @ actual[:3, :3])
    assert result["euler_convention"] == "rpy"
    assert translation_error_mm < 0.01
    assert rotation_error_deg < 0.01
    print("相机标定计算检查通过")
    print(f"模拟位置误差：{translation_error_mm:.6f} mm")
    print(f"模拟角度误差：{rotation_error_deg:.6f}°")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
