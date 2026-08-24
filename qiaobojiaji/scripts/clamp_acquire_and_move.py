#!/usr/bin/env python3
"""Move to the clamp point using the shared pry observation result."""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
ROBOT_IP = "192.168.58.2"
STATE_URL = "http://127.0.0.1:8765/api/state"


def state() -> dict:
    with urlopen(STATE_URL, timeout=3.0) as response:
        return json.load(response)


def run(command: list[str], timeout: float) -> str:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    for attempt in range(5):
        completed = subprocess.run(command, cwd=ROOT, env=env, text=True,
                                   capture_output=True, timeout=timeout, check=False)
        output = "\n".join(part.strip() for part in (completed.stdout, completed.stderr) if part.strip())
        if completed.returncode == 0:
            return output
        if attempt < 4 and ("Resource temporarily unavailable" in output or "exclusive lock" in output):
            time.sleep(0.6)
            continue
        raise RuntimeError(output or f"command failed: {command}")
    raise RuntimeError("serial command retry limit reached")


def wait_target(target: np.ndarray, timeout: float = 180.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        fr5 = state().get("fr5") or {}
        actual = np.asarray(fr5.get("flange_pose_mm_deg") or [], dtype=float)
        if int(fr5.get("emergency_stop", 0) or 0) or any(fr5.get("safety_stop") or []):
            raise RuntimeError("emergency or safety stop is active")
        if actual.shape == (6,) and int(fr5.get("motion_done", 0)) == 1:
            if np.linalg.norm(actual[:3] - target[:3]) <= 2.0:
                return
        time.sleep(0.15)
    raise TimeoutError("clamp target motion timed out")


def acquire_result(args: argparse.Namespace) -> tuple[np.ndarray, float, float]:
    if args.center_camera_mm:
        center = np.asarray([float(v) for v in args.center_camera_mm.split(",")], dtype=float)
        width = float(args.width_mm)
        opening = width - float(args.clamp_mm)
        return center, width, opening

    run([sys.executable, str(ROOT / "scripts/pry_move_to_base.py")], 180)
    from platform_a.pry_buckle_vision import PryBuckleVisionWorker
    worker = PryBuckleVisionWorker()
    worker.start()
    try:
        deadline = time.monotonic() + 90.0
        result: dict = {}
        while time.monotonic() < deadline:
            result = worker.result
            if result.get("valid") and result.get("clamp_contact_center_camera_mm") and result.get("width_mm"):
                break
            time.sleep(0.25)
        if not result.get("valid"):
            raise RuntimeError(result.get("message", "no valid clamp vision result"))
        center = np.asarray(result["clamp_contact_center_camera_mm"], dtype=float)
        width = float(result["width_mm"])
        opening = width - float(args.clamp_mm)
        return center, width, opening
    finally:
        worker.stop()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--clamp-mm", type=float, required=True)
    parser.add_argument("--speed-mm-s", type=float, default=10.0)
    parser.add_argument("--center-camera-mm", type=str, default="")
    parser.add_argument("--width-mm", type=float, default=None)
    args = parser.parse_args()
    if not 0.0 <= args.clamp_mm <= 40.0:
        raise RuntimeError("clamp displacement must be between 0 and 40 mm")
    if args.center_camera_mm and args.width_mm is None:
        raise RuntimeError("--width-mm is required with --center-camera-mm")

    center_camera, width_mm, opening_mm = acquire_result(args)
    if center_camera.shape != (3,) or not np.all(np.isfinite(center_camera)):
        raise RuntimeError("clamp center depth is invalid")
    if not 0.0 <= opening_mm <= 95.0:
        raise RuntimeError(f"target gripper opening out of range: {opening_mm:.2f} mm")

    from platform_a.handeye_calibration import pose_to_matrix
    from platform_a.tool_center_calibration import flange_pose_for_center
    calibration = json.loads((ROOT / "platform_a/config/handeye_calibration.json").read_text())
    tcp = json.loads((ROOT / "platform_a/config/gripper_tcp_calibration.json").read_text())
    snapshot = state()
    current = [float(v) for v in (snapshot.get("fr5") or {})["flange_pose_mm_deg"]]
    base_t_flange = pose_to_matrix(current, calibration.get("euler_convention", "rpy"))
    flange_t_camera = np.asarray(calibration["flange_T_camera"], dtype=float)
    center_base = (base_t_flange @ flange_t_camera @ np.r_[center_camera / 1000.0, 1.0])[:3] * 1000.0
    target = np.asarray(flange_pose_for_center(center_base, current[3:], tcp["flange_to_gripper_center_mm"]), dtype=float)
    distance = float(np.linalg.norm(target[:3] - np.asarray(current[:3])))
    if distance > 320.0:
        raise RuntimeError(f"clamp target exceeds safe distance: {distance:.1f} mm")

    run([sys.executable, str(ROOT / "scripts/set_ag95_opening.py"),
         f"{opening_mm:.3f}", "--speed", "10", "--yes"], 30)
    from fairino import Robot
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("cannot connect to FR5")
    try:
        if int(robot.Mode(0)) != 0 or int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("cannot enable automatic motion")
        robot.ResumeMotion()
        robot.ProgramResume()
        code = robot.MoveL(target.tolist(), 0, 0, vel=float(args.speed_mm_s),
                           acc=0.0, ovl=float(args.speed_mm_s), blendR=-1.0,
                           overSpeedStrategy=2, speedPercent=max(5, int(args.speed_mm_s)))
        if int(code) != 0:
            raise RuntimeError(f"MoveL rejected: {code}")
        wait_target(target)
        print(json.dumps({"center_target": target.tolist(), "width_mm": width_mm,
                          "target_opening_mm": opening_mm, "status": "AT_CLAMP_POINT"},
                         ensure_ascii=False))
        return 0
    finally:
        try:
            robot.Mode(1)
            robot.CloseRPC()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
