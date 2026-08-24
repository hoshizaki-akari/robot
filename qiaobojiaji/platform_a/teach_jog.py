"""FR5 visual-observation pose teaching and TCP-local Cartesian jog helpers.

This module reuses the existing state-service reader and Fairino SDK.  It does
not create a second robot-control layer or a ROS node.
"""

from __future__ import annotations

import json
import math
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Sequence
from urllib.request import urlopen

import numpy as np
import yaml

from platform_a.handeye_calibration import pose_to_matrix

PROJECT = Path(__file__).resolve().parents[1]
POSE_FILE = PROJECT / "platform_a" / "config" / "taught_poses.yaml"
STATE_URL = "http://127.0.0.1:8765/api/state"
ROBOT_IP = "192.168.58.2"
TOOL = 0
USER = 0


def read_state() -> dict[str, Any]:
    with urlopen(STATE_URL, timeout=3) as response:
        return json.load(response)


def checked_stopped_state(snapshot: dict[str, Any]) -> dict[str, Any]:
    fr5 = snapshot.get("fr5") or {}
    errors = fr5.get("errors") or {}
    if not fr5.get("valid") or not fr5.get("connected"):
        raise RuntimeError("FR5 状态不可用")
    if int(errors.get("main") or 0) or int(errors.get("sub") or 0):
        raise RuntimeError(f"FR5 有报警：{errors}")
    if int(fr5.get("emergency_stop") or 0):
        raise RuntimeError("FR5 急停未释放")
    if any(int(value or 0) for value in (fr5.get("safety_stop") or [])):
        raise RuntimeError("FR5 处于安全停止")
    if int(fr5.get("motion_done") or 0) != 1:
        raise RuntimeError("FR5 正在运动，拒绝此命令")
    if len(fr5.get("joint_position_deg") or []) != 6 or len(fr5.get("tcp_pose_mm_deg") or []) != 6:
        raise RuntimeError("FR5 关节或 TCP 位姿长度异常")
    return fr5


def rpy_deg_to_quaternion(rpy_deg: Sequence[float]) -> list[float]:
    rotation = pose_to_matrix([0.0, 0.0, 0.0, *map(float, rpy_deg)])[:3, :3]
    trace = float(np.trace(rotation))
    if trace > 0.0:
        s = math.sqrt(trace + 1.0) * 2.0
        q = [(rotation[2, 1] - rotation[1, 2]) / s, (rotation[0, 2] - rotation[2, 0]) / s, (rotation[1, 0] - rotation[0, 1]) / s, 0.25 * s]
    else:
        i = int(np.argmax(np.diag(rotation)))
        if i == 0:
            s = math.sqrt(1.0 + rotation[0, 0] - rotation[1, 1] - rotation[2, 2]) * 2.0
            q = [0.25 * s, (rotation[0, 1] + rotation[1, 0]) / s, (rotation[0, 2] + rotation[2, 0]) / s, (rotation[2, 1] - rotation[1, 2]) / s]
        elif i == 1:
            s = math.sqrt(1.0 + rotation[1, 1] - rotation[0, 0] - rotation[2, 2]) * 2.0
            q = [(rotation[0, 1] + rotation[1, 0]) / s, 0.25 * s, (rotation[1, 2] + rotation[2, 1]) / s, (rotation[0, 2] - rotation[2, 0]) / s]
        else:
            s = math.sqrt(1.0 + rotation[2, 2] - rotation[0, 0] - rotation[1, 1]) * 2.0
            q = [(rotation[0, 2] + rotation[2, 0]) / s, (rotation[1, 2] + rotation[2, 1]) / s, 0.25 * s, (rotation[1, 0] - rotation[0, 1]) / s]
    return [float(value) for value in q]


def teach_pose(name: str, pose_file: Path = POSE_FILE) -> dict[str, Any]:
    if not name or not name.replace("_", "").isalnum():
        raise ValueError("位姿名称只能包含字母、数字和下划线")
    fr5 = checked_stopped_state(read_state())
    tcp = [float(value) for value in fr5["tcp_pose_mm_deg"]]
    pose = {
        "joints_deg": [float(value) for value in fr5["joint_position_deg"]],
        "tcp_pose": {
            "frame_id": str(fr5.get("frame_id") or "base"),
            "position_m": dict(zip("xyz", [value / 1000.0 for value in tcp[:3]])),
            "orientation": dict(zip(("qx", "qy", "qz", "qw"), rpy_deg_to_quaternion(tcp[3:]))),
            "controller_rpy_deg": dict(zip(("rx", "ry", "rz"), tcp[3:])),
        },
        "timestamp": datetime.now(timezone.utc).isoformat(timespec="seconds"),
        "source": "fr5-state-service",
    }
    poses = yaml.safe_load(pose_file.read_text(encoding="utf-8")) if pose_file.exists() else {}
    poses = poses or {}
    poses[name] = pose
    pose_file.parent.mkdir(parents=True, exist_ok=True)
    pose_file.write_text(yaml.safe_dump(poses, allow_unicode=True, sort_keys=False), encoding="utf-8")
    return pose


def load_taught_pose(name: str, pose_file: Path = POSE_FILE) -> dict[str, Any]:
    poses = yaml.safe_load(pose_file.read_text(encoding="utf-8")) if pose_file.exists() else {}
    pose = (poses or {}).get(name)
    if not isinstance(pose, dict) or len(pose.get("joints_deg") or []) != 6:
        raise KeyError(f"未找到完整的示教位姿：{name}")
    return pose


def _motion_ready() -> None:
    fr5 = checked_stopped_state(read_state())
    if fr5.get("mode") != "auto" or not bool(fr5.get("enabled")):
        raise RuntimeError("FR5 必须已由操作者置于自动且使能；本工具不会自行切换模式或使能")


def wait_motion_done(timeout_s: float = 60.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        snapshot = read_state()
        fr5 = snapshot.get("fr5") or {}
        if int(fr5.get("motion_done") or 0) == 1:
            checked_stopped_state(snapshot)
            return
        time.sleep(0.2)
    raise TimeoutError("等待 FR5 运动完成超时")


def move_to_taught_pose(name: str, velocity_percent: float = 5.0, *, confirmed_clear: bool = False) -> list[float]:
    if not confirmed_clear:
        raise RuntimeError("必须明确确认现场已清空且急停可用（confirmed_clear=True）")
    pose = load_taught_pose(name)
    _motion_ready()
    from fairino import Robot
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("无法连接 FR5 控制器")
    try:
        code = robot.MoveJ(pose["joints_deg"], TOOL, USER, vel=float(velocity_percent), blendT=-1.0)
        if int(code) != 0:
            raise RuntimeError(f"FR5 拒绝 MoveJ，错误码 {code}")
        wait_motion_done()
        return [float(value) for value in pose["joints_deg"]]
    finally:
        robot.CloseRPC()


def local_target_pose(current_tcp_mm_deg: Sequence[float], dx_m: float, dy_m: float, dz_m: float) -> list[float]:
    if len(current_tcp_mm_deg) != 6:
        raise ValueError("TCP 位姿必须有 6 个元素 [mm, deg]")
    current = [float(value) for value in current_tcp_mm_deg]
    rotation = pose_to_matrix([0.0, 0.0, 0.0, *current[3:]])[:3, :3]
    delta_base_mm = rotation @ (np.asarray([dx_m, dy_m, dz_m], dtype=np.float64) * 1000.0)
    return [*(np.asarray(current[:3]) + delta_base_mm).tolist(), *current[3:]]


def local_move(dx_m: float, dy_m: float, dz_m: float, velocity_mm_s: float = 10.0, *, confirmed_clear: bool = False) -> list[float]:
    if not confirmed_clear:
        raise RuntimeError("必须明确确认现场已清空且急停可用（confirmed_clear=True）")
    if float(np.linalg.norm([dx_m, dy_m, dz_m])) > 0.100:
        raise ValueError("单次 Local Jog 最大 100 mm；请分步移动")
    _motion_ready()
    fr5 = checked_stopped_state(read_state())
    target = local_target_pose(fr5["tcp_pose_mm_deg"], dx_m, dy_m, dz_m)
    from fairino import Robot
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("无法连接 FR5 控制器")
    try:
        code = robot.MoveL(target, TOOL, USER, vel=float(velocity_mm_s), acc=0.0, ovl=20.0, blendR=-1.0, overSpeedStrategy=2, speedPercent=10)
        if int(code) != 0:
            raise RuntimeError(f"FR5 拒绝 MoveL，错误码 {code}")
        wait_motion_done()
        return target
    finally:
        robot.CloseRPC()


def local_move_x_plus(distance_m: float, *, confirmed_clear: bool = False) -> list[float]: return local_move(abs(distance_m), 0.0, 0.0, confirmed_clear=confirmed_clear)
def local_move_x_minus(distance_m: float, *, confirmed_clear: bool = False) -> list[float]: return local_move(-abs(distance_m), 0.0, 0.0, confirmed_clear=confirmed_clear)
def local_move_y_plus(distance_m: float, *, confirmed_clear: bool = False) -> list[float]: return local_move(0.0, abs(distance_m), 0.0, confirmed_clear=confirmed_clear)
def local_move_y_minus(distance_m: float, *, confirmed_clear: bool = False) -> list[float]: return local_move(0.0, -abs(distance_m), 0.0, confirmed_clear=confirmed_clear)
def local_move_z_plus(distance_m: float, *, confirmed_clear: bool = False) -> list[float]: return local_move(0.0, 0.0, abs(distance_m), confirmed_clear=confirmed_clear)
def local_move_z_minus(distance_m: float, *, confirmed_clear: bool = False) -> list[float]: return local_move(0.0, 0.0, -abs(distance_m), confirmed_clear=confirmed_clear)
