#!/usr/bin/env python3
"""平台A足模型夹挤：稳定取点和分段靠近。界面保持不变。"""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen

import numpy as np


ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT))

from platform_a.clamp_motion import ClampMotionConfig, build_motion_plan  # noqa: E402
from platform_a.handeye_calibration import pose_to_matrix, rotation_angle_degrees  # noqa: E402


STATE_URL = "http://127.0.0.1:8765/api/state"
VISION_URL = "http://127.0.0.1:8765/api/platform-a/clamp/plan"
ROBOT_IP = "192.168.58.2"
OBSERVATION_FILE = ROOT / "platform_a/calibration_data/clamp_camera_observation_pose.json"
TCP_FILE = ROOT / "platform_a/config/gripper_tcp_calibration.json"
HANDEYE_FILE = ROOT / "platform_a/config/handeye_calibration.json"
PLAN_FILE = ROOT / "platform_a/calibration_data/active_clamp_plan.json"
EXECUTION_FILE = ROOT / "platform_a/calibration_data/active_clamp_execution.json"
STAGE_ORDER = {"planned": 0, "pre": 1, "near": 2, "contact_center": 3}


def now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def load(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def save(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_url(url: str) -> dict:
    with urlopen(url, timeout=3.0) as response:
        return json.load(response)


def safe_state(require_open: bool = True) -> dict:
    state = read_url(STATE_URL)
    fr5 = state.get("fr5") or {}
    ag95 = state.get("ag95") or {}
    kwr = state.get("kwr75d") or {}
    if state.get("source") != "real":
        raise RuntimeError("当前不是实时设备数据")
    for name, device in (("机械臂", fr5), ("夹爪", ag95), ("力传感器", kwr)):
        if not device.get("valid") or int(device.get("age_ms", 999999)) > 1000:
            raise RuntimeError(f"{name}数据不可用或已经过期")
    errors = fr5.get("errors") or {}
    if errors.get("main") or errors.get("sub") or fr5.get("emergency_stop"):
        raise RuntimeError("机械臂存在报警或急停")
    if any(int(v or 0) for v in (fr5.get("safety_stop") or [])):
        raise RuntimeError("机械臂处于安全停止")
    if int(fr5.get("motion_done") or 0) != 1:
        raise RuntimeError("机械臂尚未停稳")
    if require_open and int(ag95.get("position_raw") or 0) < 950:
        raise RuntimeError("靠近足模型前夹爪必须基本完全张开")
    return state


def pose_errors(actual: list[float], target: list[float]) -> tuple[float, float]:
    position = float(np.linalg.norm(np.asarray(actual[:3]) - np.asarray(target[:3])))
    actual_r = pose_to_matrix(actual)[:3, :3]
    target_r = pose_to_matrix(target)[:3, :3]
    angle = rotation_angle_degrees(actual_r.T @ target_r)
    return position, angle


def wrench_delta(state: dict, baseline: np.ndarray) -> tuple[float, float]:
    current = np.asarray(state["kwr75d"]["wrench"], dtype=np.float64)
    delta = current - baseline
    return float(np.linalg.norm(delta[:3])), float(np.linalg.norm(delta[3:]))


def wait_control_ready(timeout_s: float = 8.0) -> None:
    deadline = time.monotonic() + timeout_s
    while time.monotonic() < deadline:
        state = read_url(STATE_URL)
        fr5 = state.get("fr5") or {}
        if fr5.get("valid") and fr5.get("mode") == "auto" and fr5.get("enabled"):
            time.sleep(0.5)
            return
        time.sleep(0.2)
    raise TimeoutError("机械臂没有真正进入自动且已使能状态")


def wait_pose(target: list[float], baseline: np.ndarray, timeout_s: float = 60.0) -> None:
    deadline = time.monotonic() + timeout_s
    last_position = float("inf")
    last_angle = float("inf")
    while time.monotonic() < deadline:
        state = read_url(STATE_URL)
        fr5 = state.get("fr5") or {}
        if not fr5.get("valid") or int(fr5.get("age_ms", 999999)) > 1000:
            raise RuntimeError("移动过程中机械臂实时数据中断")
        force_change, torque_change = wrench_delta(state, baseline)
        if force_change > 20.0 or torque_change > 3.0:
            raise RuntimeError(
                f"移动中受力突然变化：{force_change:.1f} N，{torque_change:.2f} Nm"
            )
        actual = [float(v) for v in fr5["flange_pose_mm_deg"]]
        last_position, last_angle = pose_errors(actual, target)
        if int(fr5.get("motion_done") or 0) == 1 and last_position <= 1.5 and last_angle <= 1.0:
            time.sleep(0.4)
            return
        time.sleep(0.2)
    raise TimeoutError(
        f"未到达目标：位置还差{last_position:.2f} mm，角度还差{last_angle:.2f}度"
    )


def capture_plan(clamp_mm: float) -> None:
    state = safe_state(require_open=True)
    observation = load(OBSERVATION_FILE)
    current_pose = [float(v) for v in state["fr5"]["flange_pose_mm_deg"]]
    position_error, angle_error = pose_errors(
        current_pose, [float(v) for v in observation["flange_pose_mm_deg"]]
    )
    if position_error > 10.0 or angle_error > 3.0:
        raise RuntimeError(
            f"机械臂不在已记录的摄像头观察位置：相差{position_error:.1f} mm、{angle_error:.1f}度"
        )

    samples: list[dict] = []
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline and len(samples) < 3:
        candidate = read_url(VISION_URL)
        if candidate.get("valid"):
            samples.append(candidate)
        else:
            samples.clear()
        time.sleep(0.8)
    if len(samples) < 3:
        raise RuntimeError("足跟和针没有连续稳定，请保持模型不动")

    centers = np.asarray([s["clamp_contact_center_base_mm"] for s in samples], dtype=np.float64)
    widths = np.asarray([s["heel_width_mm"] for s in samples], dtype=np.float64)
    if float(np.max(np.linalg.norm(centers - np.median(centers, axis=0), axis=1))) > 5.0:
        raise RuntimeError("连续三次夹取中心变化超过5 mm")
    if float(np.ptp(widths)) > 4.0:
        raise RuntimeError("连续三次足跟宽度变化超过4 mm")

    vision = samples[-1]
    plan = build_motion_plan(
        vision,
        observation["flange_pose_mm_deg"],
        load(TCP_FILE),
        load(HANDEYE_FILE),
        ClampMotionConfig(clamp_displacement_mm=clamp_mm),
    )
    plan.update(
        {
            "created_at": now(),
            "camera_frame_id": vision.get("camera_frame_id"),
            "observation_flange_pose_mm_deg": observation["flange_pose_mm_deg"],
        }
    )
    save(PLAN_FILE, plan)
    save(EXECUTION_FILE, {"created_at": now(), "stage": "planned", "history": []})
    print(f"路线已保存。足跟宽度约 {plan['heel_width_mm']:.1f} mm。")
    print(f"默认向内夹 {plan['requested_clamp_displacement_mm']:.1f} mm。")
    print("尚未发送机械臂运动命令。")


def refine_plan_at_pre() -> None:
    old_plan = load(PLAN_FILE)
    execution = load(EXECUTION_FILE)
    if execution.get("stage") != "pre":
        raise RuntimeError("只有到达60 mm外的pre位置后才能近距离更新路线")
    state = safe_state(require_open=True)
    current_pose = [float(v) for v in state["fr5"]["flange_pose_mm_deg"]]
    old_pre = [float(v) for v in old_plan["stages"]["pre"]["flange_pose_mm_deg"]]
    position_error, angle_error = pose_errors(current_pose, old_pre)
    if position_error > 3.0 or angle_error > 2.0:
        raise RuntimeError("机械臂已经离开原来的60 mm停靠位置")

    samples: list[dict] = []
    deadline = time.monotonic() + 20.0
    while time.monotonic() < deadline and len(samples) < 3:
        candidate = read_url(VISION_URL)
        if candidate.get("valid"):
            samples.append(candidate)
        else:
            samples.clear()
        time.sleep(0.8)
    if len(samples) < 3:
        raise RuntimeError("近距离画面中的足跟和针没有连续稳定")
    centers = np.asarray([s["clamp_contact_center_base_mm"] for s in samples], dtype=np.float64)
    widths = np.asarray([s["heel_width_mm"] for s in samples], dtype=np.float64)
    if float(np.max(np.linalg.norm(centers - np.median(centers, axis=0), axis=1))) > 4.0:
        raise RuntimeError("近距离连续三次夹取中心变化超过4 mm")
    if float(np.ptp(widths)) > 3.0:
        raise RuntimeError("近距离连续三次足跟宽度变化超过3 mm")

    vision = samples[-1]
    old_center = np.asarray(old_plan["clamp_center_base_mm"], dtype=np.float64)
    new_center = np.asarray(vision["clamp_contact_center_base_mm"], dtype=np.float64)
    shift = float(np.linalg.norm(new_center - old_center))
    if shift > 20.0:
        raise RuntimeError(f"足模型位置变化{shift:.1f} mm，超过20 mm，必须退回重新规划")
    observation = load(OBSERVATION_FILE)
    new_plan = build_motion_plan(
        vision,
        observation["flange_pose_mm_deg"],
        load(TCP_FILE),
        load(HANDEYE_FILE),
        ClampMotionConfig(
            clamp_displacement_mm=float(old_plan["requested_clamp_displacement_mm"])
        ),
    )
    new_plan.update(
        {
            "created_at": old_plan.get("created_at"),
            "revised_at_pre": now(),
            "revision_shift_mm": round(shift, 3),
            "camera_frame_id": vision.get("camera_frame_id"),
            "observation_flange_pose_mm_deg": observation["flange_pose_mm_deg"],
        }
    )
    save(PLAN_FILE, new_plan)
    execution.setdefault("history", []).append(
        {"stage": "pre_replan", "completed_at": now(), "shift_mm": round(shift, 3)}
    )
    save(EXECUTION_FILE, execution)
    print(f"近距离路线已更新，夹取中心修正了{shift:.1f} mm。")
    print(f"新足跟宽度约{new_plan['heel_width_mm']:.1f} mm。")
    print("机械臂没有移动。")


def move_stage(stage: str, confirmed_clear: bool) -> None:
    if not confirmed_clear:
        raise RuntimeError("必须使用--confirmed-clear确认现场无人、无障碍并可使用急停")
    plan = load(PLAN_FILE)
    execution = load(EXECUTION_FILE)
    current_stage = str(execution.get("stage", "planned"))
    required_previous = {"pre": "planned", "near": "pre", "contact_center": "near"}[stage]
    if current_stage != required_previous:
        raise RuntimeError(f"当前阶段是{current_stage}，不能直接执行{stage}")
    state = safe_state(require_open=True)
    baseline = np.asarray(state["kwr75d"]["wrench"], dtype=np.float64)
    target = [float(v) for v in plan["stages"][stage]["flange_pose_mm_deg"]]
    current = [float(v) for v in state["fr5"]["flange_pose_mm_deg"]]
    distance = float(np.linalg.norm(np.asarray(target[:3]) - np.asarray(current[:3])))
    limit = 250.0 if stage == "pre" else 80.0
    if distance > limit:
        raise RuntimeError(f"本段需要移动{distance:.1f} mm，超过{limit:.0f} mm限制")
    velocity = {"pre": 10.0, "near": 6.0, "contact_center": 3.0}[stage]

    from fairino import Robot

    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("无法连接FR5控制器")
    try:
        if int(robot.Mode(0)) != 0 or int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法进入自动且已使能状态")
        wait_control_ready()
        code = robot.MoveL(
            target, 0, 0, vel=velocity, acc=0.0, ovl=velocity * 2.0,
            blendR=-1.0, overSpeedStrategy=2, speedPercent=velocity,
        )
        if int(code) != 0:
            raise RuntimeError(f"控制器拒绝移动，返回码{code}")
        wait_pose(target, baseline)
        robot.Mode(1)
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

    execution["stage"] = stage
    execution.setdefault("history", []).append({"stage": stage, "completed_at": now()})
    save(EXECUTION_FILE, execution)
    distance_from_target = plan["stages"][stage]["distance_from_target_mm"]
    print(f"已到达{stage}阶段，夹爪中心距计划夹取中心约{distance_from_target:.0f} mm。")
    print("机械臂已停稳并回到手动模式。")


def clamp_silicone(confirmed_silicone: bool) -> None:
    if not confirmed_silicone:
        raise RuntimeError("必须使用--confirmed-silicone确认对象是硅胶足模型且手指位置正确")
    plan = load(PLAN_FILE)
    execution = load(EXECUTION_FILE)
    if execution.get("stage") != "contact_center":
        raise RuntimeError("机械臂还没有到达夹取中心")
    state = safe_state(require_open=True)
    actual_pose = [float(v) for v in state["fr5"]["flange_pose_mm_deg"]]
    target_pose = [float(v) for v in plan["stages"]["contact_center"]["flange_pose_mm_deg"]]
    position_error, angle_error = pose_errors(actual_pose, target_pose)
    if position_error > 3.0 or angle_error > 2.0:
        raise RuntimeError("机械臂已经离开夹取中心")

    from scripts.set_ag95_opening import (
        MAX_OPENING_MM,
        REG_FORCE,
        REG_INITIALIZE,
        REG_POSITION,
        REG_SPEED,
        wait_until,
        write_register,
    )
    from state_service.ag95_reader import (
        REG_ACTUAL_POSITION,
        REG_GRIP_STATUS,
        REG_INIT_STATUS,
        find_device,
        read_register,
    )
    import serial

    cfg = plan["config"]
    final_opening_mm = float(plan["predicted_final_opening_mm"])
    contact_opening_mm = float(plan["predicted_contact_opening_mm"])
    target_raw = round(final_opening_mm / MAX_OPENING_MM * 1000.0)
    baseline = np.asarray(state["kwr75d"]["wrench"], dtype=np.float64)
    force_limit = float(cfg["force_limit_n"])
    torque_limit = float(cfg["torque_limit_nm"])
    force_level = int(cfg.get("gripper_force_level", 80))
    speed = int(cfg.get("gripper_speed_percent", 10))
    last_raw = 1000

    device = find_device()
    with serial.Serial(
        device, baudrate=115200, bytesize=serial.EIGHTBITS,
        parity=serial.PARITY_NONE, stopbits=serial.STOPBITS_ONE,
        timeout=0.5, write_timeout=0.5, exclusive=True,
    ) as port:
        if read_register(port, REG_INIT_STATUS) != 1:
            write_register(port, REG_INITIALIZE, 1)
            wait_until(port, REG_INIT_STATUS, 1, 15.0)
        write_register(port, REG_FORCE, force_level)
        write_register(port, REG_SPEED, speed)
        write_register(port, REG_POSITION, target_raw)
        deadline = time.monotonic() + 25.0
        stable_count = 0
        previous_raw = None
        while time.monotonic() < deadline:
            last_raw = read_register(port, REG_ACTUAL_POSITION)
            grip_status = read_register(port, REG_GRIP_STATUS)
            stable_count = stable_count + 1 if last_raw == previous_raw else 0
            previous_raw = last_raw
            live = read_url(STATE_URL)
            kwr = live.get("kwr75d") or {}
            if not kwr.get("valid") or int(kwr.get("age_ms", 999999)) > 1000:
                write_register(port, REG_POSITION, last_raw)
                raise RuntimeError("夹挤过程中力传感器数据中断")
            delta = np.asarray(kwr["wrench"], dtype=np.float64) - baseline
            force_change = float(np.linalg.norm(delta[:3]))
            torque_change = float(np.linalg.norm(delta[3:]))
            if force_change > force_limit or torque_change > torque_limit:
                write_register(port, REG_POSITION, last_raw)
                raise RuntimeError(
                    f"夹挤受力超过上限：{force_change:.1f} N，{torque_change:.2f} Nm"
                )
            if abs(last_raw - target_raw) <= 5 and stable_count >= 2:
                break
            if grip_status == 2 and stable_count >= 4 and last_raw > target_raw + 5:
                write_register(port, REG_POSITION, last_raw)
                actual_mm = last_raw / 1000.0 * MAX_OPENING_MM
                raise RuntimeError(
                    f"硅胶阻力使夹爪提前停在{actual_mm:.1f} mm，未达到硬性目标{final_opening_mm:.1f} mm"
                )
            time.sleep(0.2)
        else:
            write_register(port, REG_POSITION, last_raw)
            raise TimeoutError("夹爪没有在25秒内到达硬性开度目标")

    actual_opening_mm = last_raw / 1000.0 * MAX_OPENING_MM
    achieved_mm = contact_opening_mm - actual_opening_mm
    if achieved_mm < float(plan["requested_clamp_displacement_mm"]) - 0.6:
        raise RuntimeError(f"实际只夹挤了{achieved_mm:.1f} mm，未达到12 mm硬性标准")
    execution["stage"] = "clamped"
    execution.setdefault("history", []).append(
        {
            "stage": "clamped",
            "completed_at": now(),
            "target_opening_mm": round(final_opening_mm, 3),
            "actual_opening_mm": round(actual_opening_mm, 3),
            "achieved_clamp_mm": round(achieved_mm, 3),
            "gripper_force_level": force_level,
        }
    )
    save(EXECUTION_FILE, execution)
    print(f"夹爪实际开度：{actual_opening_mm:.1f} mm。")
    print(f"相对视觉接触宽度，实际夹挤：{achieved_mm:.1f} mm。")
    print("12 mm硬性夹挤标准已达到。")


def retreat_all(confirmed_clear: bool) -> None:
    if not confirmed_clear:
        raise RuntimeError("必须使用--confirmed-clear确认退出路径无人、无障碍")
    plan = load(PLAN_FILE)
    execution = load(EXECUTION_FILE)
    stage = str(execution.get("stage"))
    targets: list[tuple[str, list[float], float]] = []
    if stage == "contact_center":
        targets.append(("near", [float(v) for v in plan["stages"]["near"]["flange_pose_mm_deg"]], 4.0))
        stage = "near"
    if stage == "near":
        targets.append(("pre", [float(v) for v in plan["stages"]["pre"]["flange_pose_mm_deg"]], 6.0))
        stage = "pre"
    if stage == "pre":
        observation = load(OBSERVATION_FILE)
        targets.append(("observation", [float(v) for v in observation["flange_pose_mm_deg"]], 8.0))
    if not targets:
        raise RuntimeError("当前阶段没有可执行的反向退出路线")

    state = safe_state(require_open=True)
    baseline = np.asarray(state["kwr75d"]["wrench"], dtype=np.float64)
    from fairino import Robot
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("无法连接FR5控制器")
    completed: list[str] = []
    try:
        if int(robot.Mode(0)) != 0 or int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法进入自动且已使能状态")
        wait_control_ready()
        for name, target, velocity in targets:
            code = robot.MoveL(
                target, 0, 0, vel=velocity, acc=0.0, ovl=velocity * 2.0,
                blendR=-1.0, overSpeedStrategy=2, speedPercent=velocity,
            )
            if int(code) != 0:
                raise RuntimeError(f"控制器拒绝退出到{name}，返回码{code}")
            wait_pose(target, baseline, timeout_s=80.0)
            completed.append(name)
        robot.Mode(1)
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
    execution["stage"] = "retreated"
    execution.setdefault("history", []).append(
        {"stage": "retreated", "completed_at": now(), "via": completed}
    )
    save(EXECUTION_FILE, execution)
    print("已沿原路线反向退出：" + " -> ".join(completed))
    print("机械臂已回到摄像头观察位置，夹爪保持完全张开。")


def status() -> None:
    if not PLAN_FILE.is_file():
        print("还没有保存夹挤路线。")
        return
    plan = load(PLAN_FILE)
    execution = load(EXECUTION_FILE) if EXECUTION_FILE.is_file() else {"stage": "unknown"}
    print(f"阶段：{execution.get('stage')}")
    print(f"足跟宽度：{plan['heel_width_mm']:.1f} mm")
    print(f"默认夹挤位移：{plan['requested_clamp_displacement_mm']:.1f} mm")
    print(f"预计最终开度：{plan['predicted_final_opening_mm']:.1f} mm")


def main() -> int:
    parser = argparse.ArgumentParser(description="平台A足模型夹挤分段执行")
    sub = parser.add_subparsers(dest="command", required=True)
    plan_parser = sub.add_parser("plan")
    plan_parser.add_argument("--clamp-mm", type=float, default=12.0)
    move_parser = sub.add_parser("move")
    move_parser.add_argument("stage", choices=("pre", "near", "contact_center"))
    move_parser.add_argument("--confirmed-clear", action="store_true")
    sub.add_parser("refine")
    clamp_parser = sub.add_parser("clamp")
    clamp_parser.add_argument("--confirmed-silicone", action="store_true")
    retreat_parser = sub.add_parser("retreat")
    retreat_parser.add_argument("--confirmed-clear", action="store_true")
    sub.add_parser("status")
    args = parser.parse_args()
    if args.command == "plan":
        capture_plan(args.clamp_mm)
    elif args.command == "refine":
        refine_plan_at_pre()
    elif args.command == "move":
        move_stage(args.stage, args.confirmed_clear)
    elif args.command == "clamp":
        clamp_silicone(args.confirmed_silicone)
    elif args.command == "retreat":
        retreat_all(args.confirmed_clear)
    else:
        status()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"夹挤流程停止：{error}", file=sys.stderr)
        raise SystemExit(2)
