#!/usr/bin/env python3
"""直连真机 FR5 的网页控制后端（platform_b gateway 调用）。

设计原则：
- 与 platform_a/calcaneus_robot/device.py 的 RealDeviceAdapter._robot_command 保持同一套
  Fairino SDK 调用路径，避免两套实现分叉。
- 所有“会真正让机械臂动起来”的动作都必须由调用方传入二次确认标记
  （confirm_text == "确认运动"），否则一律拒绝，避免网页误触真机。
- 急停 / 急停复位属于安全指令，但仍要求 confirm_text 以防误点。
- home（回零）复用 tk端已验证脚本：先张开夹爪再沿安全路线返回撬拨观察零点；
  set-zero（设置零点）复用 save_pry_home_position.py 仅写文件、不运动。
"""

from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from typing import Any

ROBOT_IP = os.environ.get("FR5_ROBOT_IP", "192.168.58.2")
ROOT = Path(__file__).resolve().parent.parent
FAIRINO_SDK_ROOT = ROOT / "vendor" / "fairino-python-sdk" / "linux"
if FAIRINO_SDK_ROOT.is_dir() and str(FAIRINO_SDK_ROOT) not in sys.path:
    sys.path.insert(0, str(FAIRINO_SDK_ROOT))

# 与 tk端 RealDeviceAdapter 对齐的安全阈值与默认值
CONFIRM_PHRASE = "确认运动"


class RobotControlError(RuntimeError):
    pass


def _robot_rpc() -> Any:
    """建立一次性的 Fairino RPC 连接；调用方负责 CloseRPC。"""
    try:
        from fairino import Robot
    except Exception as error:  # pragma: no cover - 依赖缺失
        raise RobotControlError(f"无法加载 Fairino SDK：{error}") from error
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RobotControlError("Robot.RPC() 没有返回 FR5 对象，控制器可能离线")
    return robot


def _require_confirm(confirm_text: str, action_label: str) -> None:
    text = (confirm_text or "").strip()
    if text != CONFIRM_PHRASE:
        raise RobotControlError(
            f"{action_label}被拒绝：请先输入“{CONFIRM_PHRASE}”后再提交"
        )


def _run_script(args: list[str], timeout: float = 240.0) -> str:
    env = os.environ.copy()
    env["FR5_PLATFORM_ROOT"] = str(ROOT)
    env["PYTHONPATH"] = str(ROOT) + os.pathsep + env.get("PYTHONPATH", "")
    completed = subprocess.run(
        [sys.executable, *args],
        cwd=ROOT,
        env=env,
        text=True,
        capture_output=True,
        timeout=timeout,
        check=False,
    )
    output = "\n".join(
        part.strip() for part in (completed.stdout, completed.stderr) if part.strip()
    )
    if completed.returncode != 0:
        detail = output.splitlines()[-1] if output else f"返回码{completed.returncode}"
        raise RobotControlError(f"脚本执行失败：{detail}")
    return output or "完成"


# ----------------------------- 运动控制指令 -----------------------------
# 这些指令直接下发到 FR5 控制器，行为与 tk端 _robot_command 完全一致。

def pause(confirm_text: str) -> dict[str, Any]:
    _require_confirm(confirm_text, "暂停")
    robot = _robot_rpc()
    try:
        ret = robot.StopMotion()
        if int(ret) != 0:
            raise RobotControlError(f"StopMotion 返回错误码 {ret}")
    finally:
        robot.CloseRPC()
    return {"action": "pause", "ok": True, "message": "已向 FR5 下发暂停（StopMotion）"}


def resume(confirm_text: str) -> dict[str, Any]:
    _require_confirm(confirm_text, "继续")
    robot = _robot_rpc()
    try:
        # Mode(0) 回到自动/可运行模式，RobotEnable(1) 重新使能
        robot.Mode(0)
        ret = robot.RobotEnable(1)
        if int(ret) != 0:
            raise RobotControlError(f"RobotEnable 返回错误码 {ret}")
    finally:
        robot.CloseRPC()
    return {"action": "resume", "ok": True, "message": "已向 FR5 下发继续（重新使能）"}


def emergency_stop(confirm_text: str) -> dict[str, Any]:
    _require_confirm(confirm_text, "急停")
    robot = _robot_rpc()
    try:
        ret = robot.StopMotion()
        if int(ret) != 0:
            raise RobotControlError(f"StopMotion 返回错误码 {ret}")
        robot.RobotEnable(0)
        robot.Mode(1)
    finally:
        robot.CloseRPC()
    return {"action": "emergency_stop", "ok": True, "message": "已触发急停：停止运动并撤销伺服使能"}


def emergency_reset(confirm_text: str) -> dict[str, Any]:
    _require_confirm(confirm_text, "急停复位")
    robot = _robot_rpc()
    try:
        robot.Mode(0)
        ret = robot.RobotEnable(1)
        if int(ret) != 0:
            raise RobotControlError(f"RobotEnable 返回错误码 {ret}")
        time.sleep(0.3)
        robot.Mode(1)
    finally:
        robot.CloseRPC()
    return {"action": "emergency_reset", "ok": True, "message": "急停已复位：伺服重新使能，进入可运行模式"}


def home(confirm_text: str) -> dict[str, Any]:
    """回零：先张开夹爪，再沿安全路线返回撬拨观察零点（复用 tk端脚本）。"""
    _require_confirm(confirm_text, "回零")
    out: list[str] = []
    # 1) 张开夹爪
    out.append(_run_script(
        [str(ROOT / "scripts/set_ag95_opening.py"), "95", "--speed", "20", "--yes"],
        timeout=60.0,
    ))
    # 2) 机械臂回撬拨观察零点（tk端“回零位置”默认与平台 A 相同）
    out.append(_run_script(
        [str(ROOT / "scripts/pry_move_to_base.py")],
        timeout=240.0,
    ))
    return {
        "action": "home",
        "ok": True,
        "message": "回零完成：夹爪全开，机械臂已返回撬拨观察零点",
        "detail": "\n".join(out),
    }


def set_zero(confirm_text: str) -> dict[str, Any]:
    """设置夹挤/撬拨共享零点：仅写入当前停稳法兰位姿，不运动。"""
    _require_confirm(confirm_text, "设置零点")
    _run_script(
        [str(ROOT / "scripts/save_pry_home_position.py")],
        timeout=30.0,
    )
    path = ROOT / "platform_a/config/pry_home_position.json"
    data: dict[str, Any] = {}
    if path.exists():
        try:
            data = json.loads(path.read_text(encoding="utf-8"))
        except Exception:
            data = {}
    return {
        "action": "set_zero",
        "ok": True,
        "message": "已记录当前停稳位置为新的夹挤/撬拨共享零点",
        "saved_at": data.get("saved_at"),
        "flange_pose_mm_deg": data.get("flange_pose_mm_deg"),
    }


# ----------------------------- 夹爪开度控制 -----------------------------
MAX_GRIPPER_OPENING_MM = 95.0


def set_gripper_opening(opening_mm: float, confirm_text: str) -> dict[str, Any]:
    """设置 AG95 夹爪目标开度（0~95 mm）。"""
    _require_confirm(confirm_text, "夹爪开度")
    if not 0.0 <= opening_mm <= MAX_GRIPPER_OPENING_MM:
        raise RobotControlError(f"夹爪开度 {opening_mm:.1f} mm 超出 0~{MAX_GRIPPER_OPENING_MM:.0f} mm 范围")
    _run_script(
        [str(ROOT / "scripts/set_ag95_opening.py"), f"{opening_mm:.2f}", "--speed", "10", "--yes"],
        timeout=60.0,
    )
    return {
        "action": "set-gripper-opening",
        "ok": True,
        "message": f"夹爪已调整至 {opening_mm:.1f} mm",
    }


# ----------------------------- 参数（仅存储，不下发） -----------------------------
_PARAM_FILE = ROOT / "platform_b/config/control_params.json"

_DEFAULT_PARAMS: dict[str, Any] = {
    "pry_displacement_mm": 100.0,   # 撬拨位移
    "clamp_displacement_mm": 5.0,   # 夹挤位移
    "speed_mm_s": 20.0,             # 速度
    "force_limit_n": 80.0,          # 力上限
    "torque_limit_nm": 8.0,         # 力矩上限
    "hold_seconds": 3.0,            # 保持时间
    "pry_direction": "X_PLUS",      # 撬拨方向
    "pry_angle_deg": 45.0,          # 撬拨角度
}


def load_params() -> dict[str, Any]:
    if _PARAM_FILE.exists():
        try:
            stored = json.loads(_PARAM_FILE.read_text(encoding="utf-8"))
            merged = dict(_DEFAULT_PARAMS)
            merged.update({k: v for k, v in stored.items() if k in _DEFAULT_PARAMS})
            return merged
        except Exception:
            return dict(_DEFAULT_PARAMS)
    return dict(_DEFAULT_PARAMS)


def save_params(params: dict[str, Any]) -> dict[str, Any]:
    cleaned = {k: v for k, v in params.items() if k in _DEFAULT_PARAMS}
    merged = load_params()
    merged.update(cleaned)
    _PARAM_FILE.parent.mkdir(parents=True, exist_ok=True)
    _PARAM_FILE.write_text(
        json.dumps(merged, ensure_ascii=False, indent=2) + "\n", encoding="utf-8"
    )
    return merged


# ----------------------------- 便捷映射 -----------------------------
def dispatch(action: str, confirm_text: str = "") -> dict[str, Any]:
    handlers = {
        "pause": pause,
        "resume": resume,
        "emergency-stop": emergency_stop,
        "emergency-reset": emergency_reset,
        "home": home,
        "set-zero": set_zero,
    }
    if action not in handlers:
        raise RobotControlError(f"未知动作：{action}")
    return handlers[action](confirm_text)


if __name__ == "__main__":
    # 仅供独立验证：python robot_control.py <action> [confirm_text]
    if len(sys.argv) < 2:
        print("usage: robot_control.py <action> [confirm_text]")
        raise SystemExit(1)
    act = sys.argv[1]
    ctext = sys.argv[2] if len(sys.argv) > 2 else ""
    try:
        result = dispatch(act, ctext)
        print(json.dumps(result, ensure_ascii=False, indent=2))
    except RobotControlError as exc:
        print(f"ERROR: {exc}")
        raise SystemExit(2)
