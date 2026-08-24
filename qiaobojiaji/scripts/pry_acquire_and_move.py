#!/usr/bin/env python3
"""Real pry flow: return to base, acquire there, then make one target MoveL."""
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
import argparse
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
STATE_URL = "http://127.0.0.1:8765/api/state"
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))

# The platform service gets this from systemd; this standalone real-motion
# workflow must establish it before importing rclpy.
_ros_libs = "/opt/ros/humble/lib:/opt/ros/humble/local/lib"
os.environ["LD_LIBRARY_PATH"] = _ros_libs + (":" + os.environ["LD_LIBRARY_PATH"] if os.environ.get("LD_LIBRARY_PATH") else "")
os.environ["AMENT_PREFIX_PATH"] = "/opt/ros/humble" + (":" + os.environ["AMENT_PREFIX_PATH"] if os.environ.get("AMENT_PREFIX_PATH") else "")
for _ros_python in ("/opt/ros/humble/local/lib/python3.10/dist-packages", "/opt/ros/humble/lib/python3.10/site-packages"):
    if _ros_python not in sys.path:
        sys.path.insert(0, _ros_python)


def state() -> dict:
    from urllib.request import urlopen
    with urlopen(STATE_URL, timeout=3.0) as response:
        return json.load(response)


def run(command: list[str], timeout: float) -> str:
    env = os.environ.copy()
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    completed = subprocess.run(command, cwd=ROOT, env=env, text=True,
                               capture_output=True, timeout=timeout, check=False)
    output = "\n".join(x.strip() for x in (completed.stdout, completed.stderr) if x.strip())
    if completed.returncode:
        raise RuntimeError(output or f"command failed: {command}")
    return output


def ensure_auto() -> None:
    from fairino import Robot
    robot = Robot.RPC("192.168.58.2")
    if robot is None:
        raise RuntimeError("无法连接 FR5")
    try:
        if int(robot.Mode(0)) != 0:
            code = robot.Mode(0)
            if int(code) != 0:
                raise RuntimeError(f"无法切换 FR5 自动模式: {code}")
        if int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法使能 FR5")
        robot.ResumeMotion()
        robot.ProgramResume()
    finally:
        robot.CloseRPC()


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--dry-run", action="store_true")
    parser.add_argument("--direction", default="X_PLUS")
    parser.add_argument("--angle-deg", type=float, default=45.0)
    args = parser.parse_args()
    ensure_auto()
    print(run([sys.executable, str(ROOT / "scripts/pry_move_to_base.py")], 180))

    from platform_a.pry_buckle_vision import PryBuckleVisionWorker
    worker = PryBuckleVisionWorker()
    worker.start()
    try:
        deadline = time.monotonic() + 90.0
        result = {}
        while time.monotonic() < deadline:
            result = worker.result
            if result.get("valid") and result.get("clamp_contact_center_camera_mm"):
                break
            time.sleep(0.25)
        if not result.get("valid"):
            raise RuntimeError(f"基准位未得到有效撬拨目标: {result.get('message', result)}")
        center = result["clamp_contact_center_camera_mm"]
        surface_gap = float(result.get("surface_to_upper_midpoint_gap_mm", 0.0))
        if surface_gap <= 0.0:
            raise RuntimeError("未能计算夹持中心到足跟上端点距离")
        print(json.dumps({"vision_result": result, "planned_center_camera_mm": center}, ensure_ascii=False))
    finally:
        worker.stop()

    ensure_auto()
    print(run([
        sys.executable, str(ROOT / "scripts/pry_move_to_clamp.py"),
        "--center-camera-mm=" + ",".join(f"{float(v):.5f}" for v in center),
        "--surface-gap-mm", f"{surface_gap:.5f}",
        "--pry-position-mm", "100.0",
        "--close-after",
        "--pry-direction", args.direction,
        "--pry-angle-deg", f"{args.angle_deg:.5f}",
        "--confirmed-clear", "--experimental-first-six",
        *( ["--dry-run"] if args.dry_run else [] ),
    ], 180))
    final = state().get("fr5") or {}
    print(json.dumps({"final_flange_pose_mm_deg": final.get("flange_pose_mm_deg"),
                      "final_tcp_pose_mm_deg": final.get("tcp_pose_mm_deg"),
                      "motion_done": final.get("motion_done")}, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
