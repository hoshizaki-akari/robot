#!/usr/bin/env python3
"""Return the real robot to the saved pry-buckle base pose, gripper open."""
from __future__ import annotations

import json
import sys
import time
from pathlib import Path
from urllib.request import urlopen

import numpy as np

ROOT = Path(__file__).resolve().parents[1]
FAIRINO_SDK_ROOT = ROOT / "vendor" / "fairino-python-sdk" / "linux"
if FAIRINO_SDK_ROOT.is_dir() and str(FAIRINO_SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(FAIRINO_SDK_ROOT))

from fairino import Robot
STATE_URL = "http://127.0.0.1:8765/api/state"
HOME = ROOT / "platform_a/config/home_position.json"
PRY_HOME = ROOT / "platform_a/config/pry_home_position.json"
ROBOT_IP = "192.168.58.2"


def state() -> dict:
    with urlopen(STATE_URL, timeout=3.0) as response:
        return json.load(response)


def wait_target(target: list[float], timeout: float = 120.0) -> None:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        fr5 = state().get("fr5") or {}
        if int(fr5.get("emergency_stop", 0) or 0) or any(fr5.get("safety_stop") or []):
            raise RuntimeError("运动中触发急停或安全停止")
        actual = np.asarray(fr5.get("flange_pose_mm_deg") or [], dtype=np.float64)
        if actual.shape == (6,) and int(fr5.get("motion_done", 0)) == 1:
            if float(np.linalg.norm(actual[:3] - np.asarray(target[:3]))) <= 2.0:
                return
        time.sleep(0.15)
    raise TimeoutError("撬拨基准位移动超时")


def main() -> int:
    snapshot = state()
    fr5 = snapshot.get("fr5") or {}
    ag95 = snapshot.get("ag95") or {}
    if not fr5.get("valid") or int(fr5.get("motion_done", 0)) != 1:
        raise RuntimeError("机械臂实时状态不可用或尚未停稳")
    if int((fr5.get("errors") or {}).get("main", 0)) or int((fr5.get("errors") or {}).get("sub", 0)):
        raise RuntimeError("机械臂存在报警")
    # 回零流程里夹爪刚被张开脚本拉到全开，状态服务的缓存可能有极短延迟，
    # 这里重试等待状态追上真实夹爪开度，避免误报“开度不足”。
    grip_ok = False
    for _ in range(20):
        if int(ag95.get("position_raw") or 0) >= 450:
            grip_ok = True
            break
        time.sleep(0.2)
        ag95 = state().get("ag95") or {}
    if not grip_ok:
        raise RuntimeError("夹爪开度不足，至少需要保持标定开度")
    home_file = HOME if HOME.is_file() else PRY_HOME
    if not home_file.is_file():
        raise RuntimeError(
            "没有找到初始零点配置：platform_a/config/home_position.json"
        )
    target = json.loads(home_file.read_text(encoding="utf-8"))["flange_pose_mm_deg"]
    current = [float(v) for v in fr5["flange_pose_mm_deg"]]
    lift = current.copy(); lift[2] += 60.0
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("无法连接FR5控制器")
    try:
        if int(robot.Mode(0)) != 0 or int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法进入自动且使能状态")
        robot.ResumeMotion()
        robot.ProgramResume()
        for label, pose in (("抬高", lift), ("撬拨基准", target)):
            code = robot.MoveL(pose, 0, 0, vel=20.0, acc=0.0, ovl=20.0, blendR=-1.0, overSpeedStrategy=2, speedPercent=20)
            if int(code) != 0:
                raise RuntimeError(f"{label}移动被控制器拒绝：{code}")
            wait_target(pose)
            print(f"已完成：{label}")
        robot.Mode(1)
        print("已到达撬拨基准位，夹爪保持张开")
        return 0
    except Exception:
        try:
            robot.StopMotion(); robot.Mode(1); robot.RobotEnable(0)
        except Exception:
            pass
        raise
    finally:
        try:
            robot.CloseRPC()
        except Exception:
            pass


if __name__ == "__main__":
    raise SystemExit(main())
