#!/usr/bin/env python3
"""轻量 FR5 模式和使能开关。

本程序只调用 Mode / RobotEnable，不发送任何移动指令。
"""

from __future__ import annotations

import argparse
import json
import os
import sys
import time
from dataclasses import dataclass
from urllib.error import URLError
from urllib.request import urlopen


ROBOT_IP = "192.168.58.2"


@dataclass
class State:
    mode: int
    enabled: int
    robot_state: int
    program_state: int
    main_error: int
    sub_error: int
    emergency_stop: int
    safety_stop_0: int
    safety_stop_1: int
    motion_done: int


def read_state(robot, timeout: float = 3.0) -> State:
    deadline = time.monotonic() + timeout
    while time.monotonic() < deadline:
        packet = getattr(robot, "robot_state_pkg", None)
        if packet is not None and not isinstance(packet, type):
            if getattr(packet, "frame_head", 0) == 0x5A5A:
                return State(
                    mode=int(packet.robot_mode),
                    enabled=int(packet.rbtEnableState),
                    robot_state=int(packet.robot_state),
                    program_state=int(packet.program_state),
                    main_error=int(packet.main_code),
                    sub_error=int(packet.sub_code),
                    emergency_stop=int(packet.EmergencyStop),
                    safety_stop_0=int(packet.safety_stop0_state),
                    safety_stop_1=int(packet.safety_stop1_state),
                    motion_done=int(packet.motion_done),
                )
        time.sleep(0.05)
    raise TimeoutError("等待机械臂状态超时")


def mode_text(mode: int) -> str:
    return {0: "自动模式", 1: "手动模式"}.get(mode, f"未知模式({mode})")


def read_state_service() -> State:
    try:
        with urlopen("http://127.0.0.1:8765/api/state", timeout=2) as response:
            snapshot = json.load(response)
    except (OSError, URLError, ValueError) as error:
        raise RuntimeError("共用状态服务不可用，请先运行 bash scripts/platforms.sh start") from error
    fr5 = snapshot.get("fr5") or {}
    if not fr5.get("valid"):
        raise RuntimeError("状态服务暂时没有有效的机械臂数据")
    errors = fr5.get("errors") or {}
    safety = fr5.get("safety_stop") or [0, 0]
    return State(
        mode=int(fr5.get("mode_raw", 0)),
        enabled=int(bool(fr5.get("enabled"))),
        robot_state=int(fr5.get("robot_state", 0)),
        program_state=int(fr5.get("program_state", 0)),
        main_error=int(errors.get("main", 0)),
        sub_error=int(errors.get("sub", 0)),
        emergency_stop=int(fr5.get("emergency_stop", 0)),
        safety_stop_0=int(safety[0]) if len(safety) > 0 else 0,
        safety_stop_1=int(safety[1]) if len(safety) > 1 else 0,
        motion_done=int(fr5.get("motion_done", 0)),
    )


def print_state(state: State) -> None:
    print(f"模式：{mode_text(state.mode)}")
    print(f"使能：{'已使能' if state.enabled else '未使能'}")
    print(f"运动：{'已停止' if state.motion_done else '仍在运动'}")
    print(f"报警：{state.main_error}/{state.sub_error}")
    print(f"急停：{state.emergency_stop}；安全停止：{state.safety_stop_0}/{state.safety_stop_1}")


def check_safe(state: State) -> None:
    problems: list[str] = []
    if state.main_error or state.sub_error:
        problems.append(f"存在报警 {state.main_error}/{state.sub_error}")
    if state.emergency_stop:
        problems.append("实体急停已触发")
    if state.safety_stop_0 or state.safety_stop_1:
        problems.append("安全停止已触发")
    if state.program_state != 1:
        problems.append(f"程序未停止（状态码 {state.program_state}）")
    if not state.motion_done:
        problems.append("机器人仍在运动")
    if problems:
        raise RuntimeError("不能修改状态：" + "；".join(problems))


def confirm(action: str, assume_yes: bool) -> None:
    if assume_yes:
        return
    print(f"准备执行：{action}。不会发送移动指令，但会改变机械臂控制状态。")
    if input("确认安全后输入 YES：").strip() != "YES":
        raise RuntimeError("已取消")


def main() -> int:
    parser = argparse.ArgumentParser(description="FR5 手动/自动模式与使能开关")
    parser.add_argument(
        "action",
        choices=("status", "manual", "auto", "enable", "disable", "on", "off"),
        nargs="?",
        default="status",
        help="status=查看；manual/auto=切换模式；enable/on=上使能；disable/off=下使能",
    )
    parser.add_argument("--yes", action="store_true", help="跳过确认，仅建议脚本自动调用时使用")
    parser.add_argument("--ip", default=ROBOT_IP, help="机械臂控制器IP")
    args = parser.parse_args()

    for name in ("NO_PROXY", "no_proxy"):
        values = [item.strip() for item in os.environ.get(name, "").split(",") if item.strip()]
        if args.ip not in values:
            values.append(args.ip)
        os.environ[name] = ",".join(values)

    try:
        from fairino import Robot
    except ModuleNotFoundError:
        print("找不到 fairino SDK，请先执行：source .venv/bin/activate", file=sys.stderr)
        return 1

    robot = None
    try:
        before = read_state_service()
        print_state(before)
        if args.action == "status":
            return 0

        check_safe(before)
        if args.action in ("manual", "auto"):
            expected = 1 if args.action == "manual" else 0
            action_text = "切换到手动模式" if expected else "切换到自动模式"
            confirm(action_text, args.yes)
            print(f"连接机械臂：{args.ip}")
            robot = Robot.RPC(args.ip)
            if robot is None:
                raise RuntimeError("SDK没有返回连接对象")
            code = robot.Mode(expected)
            field_value = expected
            field_name = "mode"
        else:
            expected = 1 if args.action in ("enable", "on") else 0
            action_text = "上使能" if expected else "下使能"
            confirm(action_text, args.yes)
            print(f"连接机械臂：{args.ip}")
            robot = Robot.RPC(args.ip)
            if robot is None:
                raise RuntimeError("SDK没有返回连接对象")
            code = robot.RobotEnable(expected)
            field_value = expected
            field_name = "enabled"

        print(f"SDK返回码：{code}")
        if code != 0:
            print("控制器拒绝了操作。", file=sys.stderr)
            return 2
        time.sleep(0.6)
        deadline = time.monotonic() + 5.0
        after = before
        while time.monotonic() < deadline:
            time.sleep(0.3)
            after = read_state_service()
            if getattr(after, field_name) == field_value:
                break
        print("操作后状态：")
        print_state(after)
        if getattr(after, field_name) != field_value:
            print("状态验证失败。", file=sys.stderr)
            return 3
        print("操作成功。")
        return 0
    except (RuntimeError, TimeoutError, ValueError) as error:
        print(f"操作未完成：{error}", file=sys.stderr)
        return 4
    except Exception as error:
        print(f"发生错误：{error}", file=sys.stderr)
        return 5
    finally:
        if robot is not None and hasattr(robot, "CloseRPC"):
            robot.CloseRPC()


if __name__ == "__main__":
    raise SystemExit(main())
