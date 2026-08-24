#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
import time
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from platform_a.handeye_calibration import (  # noqa: E402
    BoardSettings,
    estimate_board_pose,
    pose_to_matrix,
    rotation_angle_degrees,
    _mean_rotation,
)
from scripts.auto_calibrate_platform_a import (  # noqa: E402
    IMAGE_URL,
    STATE_URL,
    camera_parameters,
    fetch_json,
    safe_state,
    wait_camera_state,
    move_to,
)

RESULT_FILE = PROJECT / "platform_a" / "config" / "handeye_calibration.json"
SAMPLES_FILE = PROJECT / "platform_a" / "calibration_data" / "samples.json"
OUTPUT_FILE = PROJECT / "platform_a" / "calibration_data" / "validation.json"


def fetch_image() -> np.ndarray:
    with urlopen(IMAGE_URL, timeout=3) as response:
        image = cv2.imdecode(np.frombuffer(response.read(), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("相机图片读取失败")
    return image


def main() -> int:
    result = json.loads(RESULT_FILE.read_text(encoding="utf-8"))
    samples = json.loads(SAMPLES_FILE.read_text(encoding="utf-8"))
    X = np.asarray(result["flange_T_camera"], dtype=np.float64)
    settings = BoardSettings(55.0, 5.0)
    training_boards = []
    for sample in samples:
        training_boards.append(
            pose_to_matrix(sample["flange_pose_mm_deg"], result.get("euler_convention", "rpy"))
            @ X
            @ np.asarray(sample["camera_T_board"], dtype=np.float64)
        )
    board_center = np.mean(np.asarray([b[:3, 3] for b in training_boards]), axis=0)
    board_rotation = _mean_rotation([b[:3, :3] for b in training_boards])
    base_pose = [float(v) for v in samples[0]["flange_pose_mm_deg"]]
    offsets = [
        (20, 0, 15, 0, 6, 6),
        (-20, 0, 15, 0, -6, -6),
        (0, 20, 15, -6, 0, 6),
        (0, -20, 15, 0, 0, -6),
        (20, -20, 25, -6, 6, 0),
    ]
    validation = []
    robot = None
    try:
        from fairino import Robot

        robot = Robot.RPC("192.168.58.2")
        if robot is None or int(robot.Mode(0)) != 0 or int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法进入自动标定模式")
        move_to(robot, base_pose)
        for index, offset in enumerate(offsets, start=1):
            target = [base_pose[i] + offset[i] for i in range(6)]
            print(f"验收位置 {index}/5：偏移 {offset[:3]} mm，姿态 {offset[3:]}°")
            move_to(robot, target)
            snapshot = wait_camera_state()
            pose = safe_state(snapshot)
            matrix, distortion = camera_parameters(snapshot)
            image = fetch_image()
            camera_t_board, ids, annotated = estimate_board_pose(
                image, matrix, distortion, settings
            )
            base_t_board = pose_to_matrix(pose, result.get("euler_convention", "rpy")) @ X @ camera_t_board
            translation_error = float(np.linalg.norm(base_t_board[:3, 3] - board_center) * 1000.0)
            rotation_error = rotation_angle_degrees(board_rotation.T @ base_t_board[:3, :3])
            validation.append(
                {
                    "index": index,
                    "offset": offset,
                    "visible_marker_count": len(ids),
                    "translation_error_mm": round(translation_error, 3),
                    "rotation_error_deg": round(rotation_error, 3),
                }
            )
            print(f"  看到 {len(ids)} 个方块；位置误差 {translation_error:.2f} mm；角度误差 {rotation_error:.2f}°")
        max_translation = max(item["translation_error_mm"] for item in validation)
        max_rotation = max(item["rotation_error_deg"] for item in validation)
        passed = max_translation <= 10.0 and max_rotation <= 5.0
        output = {
            "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "sample_count": len(validation),
            "max_translation_error_mm": max_translation,
            "max_rotation_error_deg": max_rotation,
            "passed": passed,
            "checks": validation,
        }
        OUTPUT_FILE.write_text(json.dumps(output, ensure_ascii=False, indent=2), encoding="utf-8")
        if passed:
            result["validated"] = True
            result["validation"] = output
            RESULT_FILE.write_text(json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8")
            print("独立验收通过，标定结果已标记为可用。")
            return 0
        print("独立验收未通过，标定结果不会标记为可用。")
        return 2
    except Exception as error:
        print(f"验收停止：{error}", file=sys.stderr)
        return 3
    finally:
        if robot is not None:
            try:
                robot.StopMotion()
                robot.Mode(1)
                robot.CloseRPC()
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
