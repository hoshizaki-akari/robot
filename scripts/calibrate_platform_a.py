#!/usr/bin/env python3
from __future__ import annotations

import json
import sys
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen

import cv2
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from platform_a.handeye_calibration import (  # noqa: E402
    BoardSettings,
    detect_markers,
    draw_detection,
    estimate_board_pose,
    pose_to_matrix,
    rotation_angle_degrees,
    solve,
)

STATE_URL = "http://127.0.0.1:8765/api/state"
IMAGE_URL = "http://127.0.0.1:8765/api/d435/color.png"
DATA_DIR = PROJECT / "platform_a" / "calibration_data"
SETTINGS_FILE = DATA_DIR / "board.json"
SAMPLES_FILE = DATA_DIR / "samples.json"
RESULT_FILE = PROJECT / "platform_a" / "config" / "handeye_calibration.json"


def fetch_json(url: str) -> dict:
    with urlopen(url, timeout=3) as response:
        return json.load(response)


def fetch_image() -> np.ndarray:
    with urlopen(IMAGE_URL, timeout=3) as response:
        data = np.frombuffer(response.read(), dtype=np.uint8)
    image = cv2.imdecode(data, cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError("相机图片读取失败")
    return image


def read_samples() -> list[dict]:
    if not SAMPLES_FILE.exists():
        return []
    return json.loads(SAMPLES_FILE.read_text(encoding="utf-8"))


def save_json(path: Path, value) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2), encoding="utf-8")


def get_settings() -> BoardSettings:
    if SETTINGS_FILE.exists():
        value = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
        return BoardSettings(float(value["marker_length_mm"]), float(value["marker_gap_mm"]))
    print("第一次使用，需要输入你用尺子量出的两个数。")
    marker = float(input("一个黑色方块的边长（毫米）：").strip())
    gap = float(input("两个黑色方块之间白缝的宽度（毫米）：").strip())
    if not (5.0 <= marker <= 100.0 and 0.5 <= gap <= 50.0):
        raise ValueError("测量值不合理，请检查单位是不是毫米")
    save_json(
        SETTINGS_FILE,
        {
            "dictionary": "DICT_5X5_50",
            "markers_x": 4,
            "markers_y": 3,
            "first_marker_id": 0,
            "marker_length_mm": marker,
            "marker_gap_mm": gap,
        },
    )
    return BoardSettings(marker, gap)


def check_board() -> None:
    image = fetch_image()
    corners, ids = detect_markers(image)
    output = draw_detection(image, corners, ids)
    DATA_DIR.mkdir(parents=True, exist_ok=True)
    path = DATA_DIR / "board_check.png"
    cv2.imwrite(str(path), output)
    values = [] if ids is None else [int(v) for v in ids.flatten()]
    print(f"看到了 {len(values)} 个完整方块：{values}")
    print(f"检查图片已保存：{path}")
    if len(values) < 6:
        print("现在不够。请让整张标定板进入画面，并避免反光、模糊和遮挡。")
    else:
        print("画面合格，可以保存这个位置。")


def camera_parameters(snapshot: dict) -> tuple[np.ndarray, np.ndarray]:
    d435 = snapshot.get("d435") or {}
    intrinsics = d435.get("color_intrinsics")
    if not d435.get("valid") or not intrinsics:
        raise RuntimeError("相机参数还没收到，请确认共同数据服务和 D435 正常")
    matrix = np.array(
        [
            [intrinsics["fx"], 0.0, intrinsics["cx"]],
            [0.0, intrinsics["fy"], intrinsics["cy"]],
            [0.0, 0.0, 1.0],
        ],
        dtype=np.float64,
    )
    distortion = np.asarray(d435.get("distortion_coefficients") or [0, 0, 0, 0, 0], dtype=np.float64)
    return matrix, distortion


def robot_pose(snapshot: dict) -> list[float]:
    fr5 = snapshot.get("fr5") or {}
    if not fr5.get("valid") or not fr5.get("connected"):
        raise RuntimeError("机械臂状态没有连接")
    errors = fr5.get("errors") or {}
    if errors.get("main") or errors.get("sub") or fr5.get("emergency_stop"):
        raise RuntimeError("机械臂有报警或急停，不能保存")
    if not fr5.get("motion_done"):
        raise RuntimeError("机械臂还在动，请停稳后再保存")
    pose = fr5.get("flange_pose_mm_deg") or []
    if len(pose) != 6:
        raise RuntimeError("没有读到机械臂末端位置")
    return [float(v) for v in pose]


def capture() -> None:
    settings = get_settings()
    before = fetch_json(STATE_URL)
    pose_before = robot_pose(before)
    matrix, distortion = camera_parameters(before)
    image = fetch_image()
    after = fetch_json(STATE_URL)
    pose_after = robot_pose(after)
    before_t = pose_to_matrix(pose_before)
    after_t = pose_to_matrix(pose_after)
    moved_mm = float(np.linalg.norm(before_t[:3, 3] - after_t[:3, 3]) * 1000.0)
    moved_deg = rotation_angle_degrees(before_t[:3, :3].T @ after_t[:3, :3])
    if moved_mm > 0.5 or moved_deg > 0.3:
        raise RuntimeError("保存图片时机械臂动了，请停稳后重试")
    camera_t_board, ids, annotated = estimate_board_pose(
        image, matrix, distortion, settings
    )
    samples = read_samples()
    for old in samples:
        old_t = pose_to_matrix(old["flange_pose_mm_deg"])
        distance = float(np.linalg.norm(old_t[:3, 3] - before_t[:3, 3]) * 1000.0)
        angle = rotation_angle_degrees(old_t[:3, :3].T @ before_t[:3, :3])
        if distance < 10.0 and angle < 8.0:
            raise RuntimeError("这个角度和之前太像，不需要重复保存；请明显换一个观察角度")
    number = len(samples) + 1
    image_path = DATA_DIR / f"sample_{number:02d}.png"
    cv2.imwrite(str(image_path), annotated)
    samples.append(
        {
            "number": number,
            "captured_at": datetime.now().astimezone().isoformat(timespec="seconds"),
            "flange_pose_mm_deg": pose_before,
            "camera_T_board": camera_t_board.tolist(),
            "visible_marker_ids": ids,
            "image": image_path.name,
        }
    )
    save_json(SAMPLES_FILE, samples)
    print(f"第 {number} 个位置保存成功，看到 {len(ids)} 个方块。")
    print(f"还需要至少 {max(0, 15 - number)} 个位置。")


def status() -> None:
    samples = read_samples()
    print(f"已经保存：{len(samples)} 个位置；至少需要：15 个。")
    print("合格的位置要有远近、左右、上下和倾斜角度的变化，不是只平移。")
    if RESULT_FILE.exists():
        result = json.loads(RESULT_FILE.read_text(encoding="utf-8"))
        print(
            f"已有计算结果：位置误差约 {result['translation_rmse_mm']} mm，"
            f"角度误差约 {result['rotation_rmse_deg']}°；尚未做独立验收。"
        )


def calculate() -> None:
    result = solve(read_samples())
    result["board_settings"] = json.loads(SETTINGS_FILE.read_text(encoding="utf-8"))
    result["created_at"] = datetime.now().astimezone().isoformat(timespec="seconds")
    save_json(RESULT_FILE, result)
    print(f"计算完成：{RESULT_FILE}")
    print(f"位置一致性误差：{result['translation_rmse_mm']} mm")
    print(f"角度一致性误差：{result['rotation_rmse_deg']}°")
    print("这还不是最终通过；下一步还要用 5 个没参与计算的位置检查。")


def main() -> int:
    print("\n平台 A 相机标定")
    print("只读取相机和机械臂位置，不会让机械臂运动。")
    print("标定板在整个过程中必须固定不动。\n")
    while True:
        print("1. 检查标定板是否看清")
        print("2. 保存当前观察位置")
        print("3. 查看已经保存多少个")
        print("4. 满 15 个后计算")
        print("0. 退出")
        choice = input("请输入序号：").strip()
        try:
            if choice == "1":
                check_board()
            elif choice == "2":
                capture()
            elif choice == "3":
                status()
            elif choice == "4":
                calculate()
            elif choice == "0":
                return 0
            else:
                print("请输入 0～4。")
        except Exception as error:
            print(f"没有完成：{error}")
        print()


if __name__ == "__main__":
    raise SystemExit(main())
