#!/usr/bin/env python3
"""Generate or execute one bounded clamp-camera alignment move.

The default is always dry-run. A real move needs both --execute and the
explicit Chinese confirmation phrase, and is limited to one small MoveL.
"""
from __future__ import annotations

import argparse
import json
import math
import os
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
STATE_URL = os.environ.get("FR5_STATE_URL", "http://127.0.0.1:8765")
CONFIG_FILE = ROOT / "platform_a" / "config" / "vision_alignment.json"
ROBOT_IP = os.environ.get("FR5_ROBOT_IP", "192.168.58.2")


def get_json(path: str) -> dict:
    with urlopen(f"{STATE_URL}{path}", timeout=3.0) as response:
        return json.load(response)


def pose_to_matrix(pose: list[float]) -> np.ndarray:
    from platform_a.handeye_calibration import pose_to_matrix as convert

    return convert(pose, "rpy")


def matrix_to_pose(matrix: np.ndarray) -> list[float]:
    rotation = np.asarray(matrix[:3, :3], dtype=np.float64)
    sy = float(np.clip(-rotation[2, 0], -1.0, 1.0))
    ry = math.asin(sy)
    cy = math.cos(ry)
    if abs(cy) > 1e-8:
        rx = math.atan2(rotation[2, 1], rotation[2, 2])
        rz = math.atan2(rotation[1, 0], rotation[0, 0])
    else:
        rx = math.atan2(-rotation[1, 2], rotation[1, 1])
        rz = 0.0
    return [
        *[round(float(value * 1000.0), 6) for value in matrix[:3, 3]],
        *[round(math.degrees(value), 6) for value in (rx, ry, rz)],
    ]


def _rotation_z(angle_deg: float) -> np.ndarray:
    angle = math.radians(angle_deg)
    result = np.eye(4, dtype=np.float64)
    result[:3, :3] = np.asarray([
        [math.cos(angle), -math.sin(angle), 0.0],
        [math.sin(angle), math.cos(angle), 0.0],
        [0.0, 0.0, 1.0],
    ])
    return result


def build_alignment_delta(plan: dict, config: dict) -> dict:
    if not plan.get("valid") or not plan.get("motion_grade") or plan.get("display_only"):
        raise ValueError(plan.get("message") or "视觉结果不是运动级有效结果")
    status = str(plan.get("measurement_status", ""))
    if status != "stabilized":
        raise ValueError(f"视觉结果尚未稳定：{status or 'unknown'}")
    center_px = plan.get("center_px") or []
    width = int(plan.get("image_width") or 0)
    height = int(plan.get("image_height") or 0)
    center_camera = np.asarray(plan.get("center_camera_mm") or [], dtype=np.float64)
    if len(center_px) != 2 or width <= 0 or height <= 0 or center_camera.shape != (3,):
        raise ValueError("视觉结果缺少图像中心、图像尺寸或相机三维中心")
    intrinsics = plan.get("color_intrinsics") or plan.get("intrinsics") or {}
    fx = float(intrinsics.get("fx") or 0.0)
    fy = float(intrinsics.get("fy") or 0.0)
    if fx <= 0.0 or fy <= 0.0:
        # D435 color intrinsics are not currently included in the plan. The
        # width estimate provides a safe fallback only for dry-run diagnostics;
        # real execution must supply intrinsics through the state service.
        raise ValueError("视觉规划中缺少相机焦距，不能生成自动对准位姿")
    error_px = np.asarray(center_px, dtype=np.float64) - np.asarray([width / 2.0, height / 2.0])
    delta_translation = np.asarray([
        error_px[0] * center_camera[2] / fx,
        error_px[1] * center_camera[2] / fy,
        0.0,
    ])
    principal_angle = float(plan.get("principal_angle_deg") or 0.0)
    delta_rotation = principal_angle - float(config.get("alignment_target_principal_angle_deg", 0.0))
    total_translation = float(np.linalg.norm(delta_translation))
    total_rotation = abs(delta_rotation)
    max_axis = float(config.get("max_axis_translation_mm", 5.0))
    max_total = float(config.get("max_total_translation_mm", 8.0))
    max_axis_rotation = float(config.get("max_axis_rotation_deg", 3.0))
    max_total_rotation = float(config.get("max_total_rotation_deg", 5.0))
    safe = bool(
        np.all(np.abs(delta_translation) <= max_axis)
        and total_translation <= max_total
        and total_rotation <= max_axis_rotation
        and total_rotation <= max_total_rotation
    )
    delta = _rotation_z(delta_rotation)
    delta[:3, 3] = delta_translation / 1000.0
    return {
        "safe_by_geometry": safe,
        "error_px": [round(float(value), 4) for value in error_px],
        "delta_camera_translation_mm": [round(float(value), 4) for value in delta_translation],
        "delta_camera_rotation_deg": round(delta_rotation, 4),
        "predicted_translation_mm": round(total_translation, 4),
        "predicted_rotation_deg": round(total_rotation, 4),
        "delta_camera": delta,
    }


def build_target_pose(plan: dict, state_snapshot: dict, config: dict) -> tuple[list[float], dict]:
    fr5 = state_snapshot.get("fr5") or {}
    current_pose = [float(value) for value in (fr5.get("flange_pose_mm_deg") or [])]
    if len(current_pose) != 6 or not fr5.get("valid") or int(fr5.get("motion_done", 0)) != 1:
        raise ValueError("机械臂实时位姿不可用或尚未停稳")
    if int((fr5.get("errors") or {}).get("main", 0)) or int((fr5.get("errors") or {}).get("sub", 0)):
        raise ValueError("机械臂存在报警")
    calibration = json.loads((ROOT / "platform_a/config/handeye_calibration.json").read_text(encoding="utf-8"))
    if not calibration.get("validated"):
        raise ValueError("手眼标定尚未通过验收")
    alignment = build_alignment_delta(plan, config)
    base_t_flange = pose_to_matrix(current_pose)
    flange_t_camera = np.asarray(calibration["flange_T_camera"], dtype=np.float64)
    base_t_camera_target = base_t_flange @ flange_t_camera @ alignment["delta_camera"]
    base_t_flange_target = base_t_camera_target @ np.linalg.inv(flange_t_camera)
    target_pose = matrix_to_pose(base_t_flange_target)
    alignment.pop("delta_camera")
    alignment["current_flange_pose_mm_deg"] = current_pose
    alignment["target_flange_pose_mm_deg"] = target_pose
    alignment["base_t_camera_target"] = base_t_camera_target.tolist()
    return target_pose, alignment


def wait_target(target: list[float], timeout: float = 60.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        snapshot = get_json("/api/state")
        fr5 = snapshot.get("fr5") or {}
        if int(fr5.get("emergency_stop", 0) or 0) or any(fr5.get("safety_stop") or []):
            raise RuntimeError("运动中触发急停或安全停止")
        actual = np.asarray(fr5.get("flange_pose_mm_deg") or [], dtype=np.float64)
        if actual.shape == (6,) and int(fr5.get("motion_done", 0)) == 1 and np.linalg.norm(actual[:3] - np.asarray(target[:3])) <= 2.0:
            return
        time.sleep(0.15)
    raise TimeoutError("自动对准目标位移动超时")


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true", help="只输出建议，不移动机器人（默认行为）")
    parser.add_argument("--execute", action="store_true", help="执行一次低速对准")
    parser.add_argument("--confirmed-clear", action="store_true")
    parser.add_argument("--confirm-text", default="")
    args = parser.parse_args()
    config = json.loads(CONFIG_FILE.read_text(encoding="utf-8"))
    plan = get_json("/api/platform-a/clamp/plan")
    snapshot = get_json("/api/state")
    target, alignment = build_target_pose(plan, snapshot, config)
    output = dict(alignment)
    output["status"] = "DRY_RUN"
    if not alignment["safe_by_geometry"]:
        output["status"] = "MANUAL_REQUIRED"
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 2
    if not args.execute:
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0
    if not config.get("clamp_auto_align_enabled"):
        raise RuntimeError("配置已关闭真实自动对准，请先人工确认并启用 clamp_auto_align_enabled")
    if not args.confirmed_clear or args.confirm_text.strip() != "确认一次自动对准":
        raise RuntimeError("真实自动对准需要 --confirmed-clear 和确认文字“确认一次自动对准”")
    from fairino import Robot

    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("无法连接FR5")
    try:
        if int(robot.Mode(0)) != 0 or int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法进入自动且使能状态")
        robot.ResumeMotion()
        robot.ProgramResume()
        code = robot.MoveL(target, 0, 0, vel=5.0, acc=0.0, ovl=5.0, blendR=-1.0, overSpeedStrategy=2, speedPercent=5)
        if int(code) != 0:
            raise RuntimeError(f"自动对准MoveL被控制器拒绝：{code}")
        wait_target(target)
        time.sleep(float(config.get("settle_seconds", 1.0)))
        output["status"] = "ALIGNMENT_COMPLETE_REMEASURE_REQUIRED"
        print(json.dumps(output, ensure_ascii=False, indent=2))
        return 0
    finally:
        try:
            robot.Mode(1)
            robot.CloseRPC()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
