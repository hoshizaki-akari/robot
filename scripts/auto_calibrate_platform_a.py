#!/usr/bin/env python3
"""平台 A 安全的小范围自动相机标定。

只在固定标定板前采集相机和法兰位置；不执行夹爪、力控或足模型动作。
每个目标点都要确认标定板仍可见，不可见会退回上一个位置。
"""

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
    solve,
)
from scripts.calibrate_platform_a import (  # noqa: E402
    DATA_DIR,
    IMAGE_URL,
    RESULT_FILE,
    SAMPLES_FILE,
    STATE_URL,
    camera_parameters,
    read_samples,
    save_json,
)

ROBOT_IP = "192.168.58.2"
MARKER_LENGTH_MM = 55.0
MARKER_GAP_MM = 5.0


def fetch_json(url: str) -> dict:
    with urlopen(url, timeout=3) as response:
        return json.load(response)


def fetch_image() -> np.ndarray:
    with urlopen(IMAGE_URL, timeout=3) as response:
        image = cv2.imdecode(np.frombuffer(response.read(), dtype=np.uint8), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("相机图片读取失败")
    return image


def safe_state(snapshot: dict) -> list[float]:
    fr5 = snapshot.get("fr5") or {}
    errors = fr5.get("errors") or {}
    if not fr5.get("valid") or not fr5.get("connected"):
        raise RuntimeError("机械臂状态不可用")
    if errors.get("main") or errors.get("sub") or fr5.get("emergency_stop"):
        raise RuntimeError("机械臂有报警或急停")
    if any(int(value or 0) for value in (fr5.get("safety_stop") or [])):
        raise RuntimeError("机械臂处于安全停止")
    if not fr5.get("motion_done"):
        raise RuntimeError("机械臂还在运动")
    pose = fr5.get("flange_pose_mm_deg") or []
    if len(pose) != 6:
        raise RuntimeError("没有读到法兰位置")
    return [float(value) for value in pose]


def wait_camera_state(timeout: float = 20.0) -> dict:
    end = time.monotonic() + timeout
    last = {}
    while time.monotonic() < end:
        last = fetch_json(STATE_URL)
        d435 = last.get("d435") or {}
        if d435.get("valid") and d435.get("color_valid") and d435.get("color_intrinsics"):
            return last
        time.sleep(0.8)
    raise RuntimeError(f"D435 在 {timeout:.0f} 秒内没有恢复：{(last.get('d435') or {}).get('message', '')}")


def save_current_sample(settings: BoardSettings, samples: list[dict], number: int) -> bool:
    before = wait_camera_state()
    pose = safe_state(before)
    matrix, distortion = camera_parameters(before)
    image = fetch_image()
    after_pose = safe_state(wait_camera_state())
    before_matrix = pose_to_matrix(pose)
    after_matrix = pose_to_matrix(after_pose)
    if float(np.linalg.norm(before_matrix[:3, 3] - after_matrix[:3, 3]) * 1000.0) > 0.6:
        print("保存时机械臂位置变化，跳过这个点。")
        return False
    try:
        camera_t_board, ids, annotated = estimate_board_pose(image, matrix, distortion, settings)
    except ValueError as error:
        print(f"标定板检查失败：{error}")
        return False
    for old in samples:
        old_matrix = pose_to_matrix(old["flange_pose_mm_deg"])
        distance_mm = float(np.linalg.norm(old_matrix[:3, 3] - before_matrix[:3, 3]) * 1000.0)
        angle_deg = rotation_angle_degrees(old_matrix[:3, :3].T @ before_matrix[:3, :3])
        if distance_mm < 10.0 and angle_deg < 8.0:
            print("这个位置和已保存位置太近，跳过。")
            return False
    image_path = DATA_DIR / f"sample_{number:02d}.png"
    cv2.imwrite(str(image_path), annotated)
    samples.append(
        {
            "number": number,
            "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "flange_pose_mm_deg": pose,
            "camera_T_board": camera_t_board.tolist(),
            "visible_marker_ids": ids,
            "image": image_path.name,
        }
    )
    save_json(SAMPLES_FILE, samples)
    print(f"第 {number} 个位置已保存（看到 {len(ids)} 个方块）。")
    return True


def wait_stopped(timeout: float = 20.0) -> dict:
    end = time.monotonic() + timeout
    last = {}
    while time.monotonic() < end:
        last = fetch_json(STATE_URL)
        fr5 = last.get("fr5") or {}
        errors = fr5.get("errors") or {}
        if errors.get("main") or errors.get("sub") or fr5.get("emergency_stop"):
            raise RuntimeError(f"移动后出现报警：{errors}")
        if int(fr5.get("motion_done") or 0) == 1:
            time.sleep(0.8)
            return last
        time.sleep(0.25)
    raise TimeoutError("等待机械臂停稳超时")


def move_to(robot, pose: list[float]) -> None:
    # 目标限制在当前观察点的小范围内，减少碰撞风险。
    if not (240 <= pose[0] <= 340 and -220 <= pose[1] <= -105 and 350 <= pose[2] <= 470):
        raise ValueError(f"目标位置超出安全小范围：{pose[:3]}")
    if not (145 <= pose[3] <= 215 and -45 <= pose[4] <= 45 and -190 <= pose[5] <= -75):
        raise ValueError(f"目标姿态超出安全小范围：{pose[3:]}")
    code = robot.MoveL(
        pose,
        0,
        0,
        vel=20.0,
        acc=0.0,
        ovl=30.0,
        blendR=-1.0,
        overSpeedStrategy=2,
        speedPercent=10,
    )
    if int(code) != 0:
        raise RuntimeError(f"控制器拒绝移动，返回码：{code}")
    wait_stopped()


def main() -> int:
    print("平台 A 自动相机标定：只移动机械臂观察标定板，不夹取、不力控。")
    print("标定板必须固定；活动范围内不能有人。")
    snapshot = wait_camera_state()
    current_pose = safe_state(snapshot)
    matrix, _ = camera_parameters(snapshot)
    settings = BoardSettings(MARKER_LENGTH_MM, MARKER_GAP_MM)
    save_json(
        DATA_DIR / "board.json",
        {
            "dictionary": "DICT_5X5_50",
            "markers_x": 4,
            "markers_y": 3,
            "first_marker_id": 0,
            "marker_length_mm": MARKER_LENGTH_MM,
            "marker_gap_mm": MARKER_GAP_MM,
        },
    )
    samples = read_samples()
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    previous_pose = list(current_pose)
    try:
        from fairino import Robot

        robot = Robot.RPC(ROBOT_IP)
        if robot is None:
            raise RuntimeError("无法连接 FR5 控制器")
        if int(robot.Mode(0)) != 0:
            raise RuntimeError("无法切换自动模式")
        if int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法保持机械臂使能")
        time.sleep(1.0)
        if samples:
            # 续跑时回到第一个样本作为共同基准，避免重复采集和越界。
            current_pose = [float(value) for value in samples[0]["flange_pose_mm_deg"]]
            print("检测到已有样本，将回到第一个位置并继续，不会删除已有数据。")
            move_to(robot, current_pose)
            previous_pose = list(current_pose)
        elif not save_current_sample(settings, samples, len(samples) + 1):
            raise RuntimeError("当前起始画面不合格，请停止自动标定")
        # 相邻点只做小幅变化，同时改变位置和姿态，保证手眼计算有足够信息。
        offsets = [
            (45, 0, 0, 0, 0, 12), (-45, 0, 0, 0, 0, -12),
            (0, 45, 0, 0, 12, 0), (0, -45, 0, 0, -12, 0),
            (0, 0, 45, 0, 0, 12), (0, 0, -45, 0, 0, -12),
            (35, 35, 0, 0, -10, 10), (-35, 35, 0, 0, -10, -10),
            (35, -35, 0, 0, 10, -10), (-35, -35, 0, 0, 10, 10),
            (0, 0, 0, -12, 0, 12), (0, 0, 0, -24, 0, -12),
            (30, 0, 30, 0, 10, 0), (-30, 0, 30, 0, -10, 0),
            (0, 30, 30, 0, 12, 0), (0, -30, 30, 0, -12, 0),
            (25, 25, 30, 0, -8, 8), (-25, 25, 30, 0, -8, -8),
        ]
        for offset in offsets:
            if len(samples) >= 20:
                break
            target = [current_pose[index] + offset[index] for index in range(6)]
            target_matrix = pose_to_matrix(target)
            already_saved = False
            for old in samples:
                old_matrix = pose_to_matrix(old["flange_pose_mm_deg"])
                distance_mm = float(np.linalg.norm(old_matrix[:3, 3] - target_matrix[:3, 3]) * 1000.0)
                angle_deg = rotation_angle_degrees(old_matrix[:3, :3].T @ target_matrix[:3, :3])
                if distance_mm < 10.0 and angle_deg < 8.0:
                    already_saved = True
                    break
            if already_saved:
                print(f"跳过已保存的观察点：{offset[:3]} mm，{offset[3:]}°")
                continue
            print(f"移动到第 {len(samples) + 1} 个观察点：位置偏移 {offset[:3]} mm，姿态偏移 {offset[3:]}°")
            move_to(robot, target)
            if save_current_sample(settings, samples, len(samples) + 1):
                previous_pose = target
            else:
                print("这个位置没有通过画面检查，退回上一个安全位置。")
                move_to(robot, previous_pose)
        if len(samples) < 15:
            raise RuntimeError(f"只保存了 {len(samples)} 个位置，未达到 15 个")
        result = solve(samples)
        result["board_settings"] = json.loads((DATA_DIR / "board.json").read_text(encoding="utf-8"))
        result["created_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
        save_json(RESULT_FILE, result)
        print(f"标定计算完成：{RESULT_FILE}")
        print(f"位置一致性误差：{result['translation_rmse_mm']} mm")
        print(f"角度一致性误差：{result['rotation_rmse_deg']}°")
        print("结果仍需独立位置验收，验收前不会用于自动运动。")
        return 0
    except Exception as error:
        try:
            robot.StopMotion()
        except Exception:
            pass
        print(f"自动标定停止：{error}", file=sys.stderr)
        return 2
    finally:
        try:
            robot.Mode(1)
        except Exception:
            pass
        try:
            robot.CloseRPC()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
