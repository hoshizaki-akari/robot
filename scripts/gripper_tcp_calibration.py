#!/usr/bin/env python3
"""Guided multi-pose TCP calibration for the AG-160-95 gripper center."""

from __future__ import annotations

import argparse
import json
import sys
import time
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen

import numpy as np


PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))

from platform_a.tool_center_calibration import (  # noqa: E402
    flange_pose_for_center,
    point_from_pose,
    solve_tool_center,
    validate_tool_center,
)
from platform_a.handeye_calibration import pose_to_matrix, rotation_angle_degrees  # noqa: E402


STATE_URL = "http://127.0.0.1:8765/api/state"
ROBOT_IP = "192.168.58.2"
SESSION_FILE = PROJECT / "platform_a" / "calibration_data" / "gripper_tcp_session.json"
RESULT_FILE = PROJECT / "platform_a" / "config" / "gripper_tcp_calibration.json"
OPENING_RAW = 505
OPENING_TOLERANCE_RAW = 5
INITIAL_OFFSET_MM = [0.0, 0.0, 220.0]
ROUGH_CLEARANCE_MM = 15.0
RETREAT_MM = 35.0
CALIBRATION_MOVE_VEL = 25.0
CALIBRATION_MOVE_SPEED_PERCENT = 25
CALIBRATION_OFFSETS_DEG = [
    [12.0, 0.0, 0.0],
    [0.0, 12.0, 0.0],
    [0.0, -12.0, 0.0],
    [9.0, 9.0, 0.0],
    [9.0, -9.0, 0.0],
]
VALIDATION_OFFSETS_DEG = [
    [8.0, -2.0, 10.0],
    [3.0, 10.0, -8.0],
    [11.0, -4.0, -10.0],
]


def now() -> str:
    return datetime.now().astimezone().isoformat(timespec="seconds")


def load_json(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8"))


def save_json(path: Path, value: dict) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def read_live_state(require_stopped: bool = True) -> dict:
    with urlopen(STATE_URL, timeout=3) as response:
        state = json.load(response)
    fr5 = state.get("fr5") or {}
    ag95 = state.get("ag95") or {}
    errors = fr5.get("errors") or {}
    if state.get("source") != "real":
        raise RuntimeError("当前不是实时设备数据")
    if not fr5.get("valid") or int(fr5.get("age_ms", 999999999)) > 1000:
        raise RuntimeError("机械臂位置不是实时数据")
    if errors.get("main") or errors.get("sub") or fr5.get("emergency_stop"):
        raise RuntimeError("机械臂存在报警或急停")
    if any(int(value or 0) for value in (fr5.get("safety_stop") or [])):
        raise RuntimeError("机械臂处于安全停止")
    if require_stopped and int(fr5.get("motion_done") or 0) != 1:
        raise RuntimeError("机械臂尚未停止")
    if not ag95.get("valid") or int(ag95.get("age_ms", 999999999)) > 1000:
        raise RuntimeError("夹爪位置不是实时数据")
    raw = int(ag95.get("position_raw"))
    if abs(raw - OPENING_RAW) > OPENING_TOLERANCE_RAW:
        raise RuntimeError(f"夹爪必须保持48 mm；当前设备值是 {raw}/1000")
    pose = fr5.get("flange_pose_mm_deg") or []
    if len(pose) != 6:
        raise RuntimeError("没有读到完整法兰位置")
    return state


def current_sample(kind: str) -> dict:
    state = read_live_state()
    return {
        "captured_at": now(),
        "kind": kind,
        "flange_pose_mm_deg": [float(value) for value in state["fr5"]["flange_pose_mm_deg"]],
        "opening_raw": int(state["ag95"]["position_raw"]),
    }


def start_session() -> None:
    sample = current_sample("calibration")
    assumed_point = point_from_pose(sample["flange_pose_mm_deg"], INITIAL_OFFSET_MM)
    session = {
        "status": "collecting_calibration",
        "created_at": now(),
        "opening_raw_target": OPENING_RAW,
        "opening_mm_nominal": 47.975,
        "initial_offset_guess_mm": INITIAL_OFFSET_MM,
        "assumed_cone_tip_base_mm": [round(float(value), 4) for value in assumed_point],
        "initial_rpy_deg": sample["flange_pose_mm_deg"][3:],
        "calibration_samples": [sample],
        "validation_samples": [],
    }
    save_json(SESSION_FILE, session)
    print("已记录第1个尖锥对准姿势。")
    print("没有发送运动命令。")


def capture(kind: str) -> None:
    if not SESSION_FILE.is_file():
        raise RuntimeError("还没有开始标定会话")
    session = load_json(SESSION_FILE)
    sample = current_sample(kind)
    key = "calibration_samples" if kind == "calibration" else "validation_samples"
    current_transform = pose_to_matrix(sample["flange_pose_mm_deg"])
    comparison_samples = list(session[key])
    if kind == "validation":
        comparison_samples += list(session.get("calibration_samples") or [])
        comparison_samples += list(session.get("refinement_samples") or [])
    for old_sample in comparison_samples:
        old_transform = pose_to_matrix(old_sample["flange_pose_mm_deg"])
        angle_change = rotation_angle_degrees(
            old_transform[:3, :3].T @ current_transform[:3, :3]
        )
        if angle_change < 3.0:
            raise RuntimeError(
                f"当前角度与已有姿势只差 {angle_change:.2f} 度，不能作为新标定数据"
            )
    session[key].append(sample)
    count = len(session[key])
    session["status"] = "collecting_calibration" if kind == "calibration" else "collecting_validation"
    save_json(SESSION_FILE, session)
    print(f"已记录{kind}姿势第 {count} 个。")


def target_for_next(session: dict, kind: str) -> tuple[list[float], list[float]]:
    offsets = CALIBRATION_OFFSETS_DEG if kind == "calibration" else VALIDATION_OFFSETS_DEG
    samples = session["calibration_samples"] if kind == "calibration" else session["validation_samples"]
    completed_after_initial = len(samples) - 1 if kind == "calibration" else len(samples)
    if completed_after_initial >= len(offsets):
        raise RuntimeError(f"{kind}计划姿势已经全部完成")
    offset = offsets[completed_after_initial]
    initial_rpy = np.asarray(session["initial_rpy_deg"], dtype=np.float64)
    target_rpy = (initial_rpy + np.asarray(offset, dtype=np.float64)).tolist()
    if RESULT_FILE.is_file() and kind == "validation":
        result = load_json(RESULT_FILE)
        tool_offset = result["flange_to_gripper_center_mm"]
        cone_point = result["fixed_cone_tip_base_mm"]
    else:
        tool_offset = session["initial_offset_guess_mm"]
        cone_point = session["assumed_cone_tip_base_mm"]
    rough_center = np.asarray(cone_point, dtype=np.float64) + [0.0, 0.0, ROUGH_CLEARANCE_MM]
    target_pose = flange_pose_for_center(rough_center, target_rpy, tool_offset)
    return target_pose, target_rpy


def wait_pose_reached(target_pose: list[float], timeout_s: float = 45.0) -> None:
    target_transform = pose_to_matrix(target_pose)
    end = time.monotonic() + timeout_s
    last_position_error = float("inf")
    last_angle_error = float("inf")
    while time.monotonic() < end:
        state = read_live_state(require_stopped=False)
        actual_pose = [float(value) for value in state["fr5"]["flange_pose_mm_deg"]]
        actual_transform = pose_to_matrix(actual_pose)
        last_position_error = float(
            np.linalg.norm(np.asarray(actual_pose[:3]) - np.asarray(target_pose[:3]))
        )
        last_angle_error = rotation_angle_degrees(
            actual_transform[:3, :3].T @ target_transform[:3, :3]
        )
        if (
            int(state["fr5"].get("motion_done") or 0) == 1
            and last_position_error <= 1.0
            and last_angle_error <= 0.8
        ):
            time.sleep(0.5)
            return
        time.sleep(0.2)
    raise TimeoutError(
        "机械臂未真正到达目标；"
        f"位置还差 {last_position_error:.2f} mm，角度还差 {last_angle_error:.2f} 度"
    )


def wait_control_ready(timeout_s: float = 8.0) -> None:
    end = time.monotonic() + timeout_s
    while time.monotonic() < end:
        state = read_live_state(require_stopped=False)
        fr5 = state["fr5"]
        if fr5.get("mode") == "auto" and bool(fr5.get("enabled")):
            time.sleep(0.5)
            return
        time.sleep(0.2)
    raise TimeoutError("机械臂没有真正进入自动且已使能状态")


def move_linear(robot, pose: list[float]) -> None:
    code = robot.MoveL(
        pose,
        0,
        0,
        vel=CALIBRATION_MOVE_VEL,
        acc=0.0,
        ovl=CALIBRATION_MOVE_SPEED_PERCENT * 2.0,
        blendR=-1.0,
        overSpeedStrategy=2,
        speedPercent=CALIBRATION_MOVE_SPEED_PERCENT,
    )
    if int(code) != 0:
        raise RuntimeError(f"控制器拒绝移动，返回码 {code}")
    wait_pose_reached(pose)


def move_next(kind: str, confirmed: bool) -> None:
    if not confirmed:
        raise RuntimeError("必须使用 --confirmed 表示现场已经清空并可使用急停")
    session = load_json(SESSION_FILE)
    state = read_live_state()
    current_pose = [float(value) for value in state["fr5"]["flange_pose_mm_deg"]]
    target_pose, target_rpy = target_for_next(session, kind)
    start_position = np.asarray(session["calibration_samples"][0]["flange_pose_mm_deg"][:3])
    if float(np.linalg.norm(np.asarray(target_pose[:3]) - start_position)) > 65.0:
        raise RuntimeError("目标位置离起始位置超过65 mm，拒绝移动")
    retreat_pose = list(current_pose)
    retreat_pose[2] = max(float(current_pose[2]), float(start_position[2]) + RETREAT_MM)

    from fairino import Robot

    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("无法连接FR5控制器")
    try:
        if int(robot.Mode(0)) != 0 or int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法进入低速自动移动状态")
        wait_control_ready()
        print("先沿桌面上方退开35 mm。")
        move_linear(robot, retreat_pose)
        print(f"改变角度并停在尖锥顶上方 {ROUGH_CLEARANCE_MM:.0f} mm。")
        move_linear(robot, target_pose)
        robot.Mode(1)
        print(f"粗定位完成，目标角度：{[round(v, 2) for v in target_rpy]}")
        print("机械臂已回到手动模式。请现场小幅调整，使夹爪中心重新对准尖锥顶端。")
    except Exception:
        try:
            robot.StopMotion()
            robot.Mode(1)
            robot.RobotEnable(0)
        except Exception:
            pass
        raise
    finally:
        try:
            robot.CloseRPC()
        except Exception:
            pass


def solve() -> None:
    session = load_json(SESSION_FILE)
    poses = [sample["flange_pose_mm_deg"] for sample in session["calibration_samples"]]
    if len(poses) < 6:
        raise RuntimeError(f"需要6个标定姿势，现在只有 {len(poses)} 个")
    result = solve_tool_center(poses)
    difference = float(
        np.linalg.norm(
            np.asarray(result["flange_to_gripper_center_mm"]) - np.asarray(INITIAL_OFFSET_MM)
        )
    )
    result.update(
        {
            "created_at": now(),
            "opening_raw": OPENING_RAW,
            "opening_mm_nominal": 47.975,
            "difference_from_220mm_guess_mm": round(difference, 4),
            "motion_allowed": False,
        }
    )
    if difference > 40.0:
        raise RuntimeError("计算结果与220 mm初值相差超过40 mm，请检查对准过程")
    save_json(RESULT_FILE, result)
    session["status"] = "collecting_validation"
    save_json(SESSION_FILE, session)
    print(f"夹爪中心偏移：{result['flange_to_gripper_center_mm']} mm")
    print(f"拟合最大误差：{result['fit_max_error_mm']} mm")
    print("结果尚未通过独立验收，仍禁止自动夹挤。")


def refine_after_failed_validation() -> None:
    session = load_json(SESSION_FILE)
    if session.get("status") != "validation_failed":
        raise RuntimeError("只有独立验收未通过后才能重新计算")
    calibration = list(session.get("calibration_samples") or [])
    previous_validation = list(session.get("validation_samples") or [])
    all_samples = calibration + previous_validation
    if len(all_samples) < 7:
        raise RuntimeError("可用于重新计算的对准数据不足")

    best = None
    for excluded_index in range(len(all_samples)):
        used = [
            sample["flange_pose_mm_deg"]
            for index, sample in enumerate(all_samples)
            if index != excluded_index
        ]
        candidate = solve_tool_center(used)
        score = (candidate["fit_rmse_mm"], candidate["fit_max_error_mm"])
        if best is None or score < best[0]:
            best = (score, excluded_index, candidate)
    assert best is not None
    _, excluded_index, result = best
    if float(result["fit_max_error_mm"]) > 3.0:
        raise RuntimeError("剔除最明显的一次偏差后，剩余数据仍不一致，需要重新采集")

    previous_result = load_json(RESULT_FILE)
    difference = float(
        np.linalg.norm(
            np.asarray(result["flange_to_gripper_center_mm"])
            - np.asarray(INITIAL_OFFSET_MM)
        )
    )
    result.update(
        {
            "created_at": now(),
            "opening_raw": OPENING_RAW,
            "opening_mm_nominal": 47.975,
            "difference_from_220mm_guess_mm": round(difference, 4),
            "motion_allowed": False,
            "refinement": {
                "used_previous_validation_for_refit": True,
                "total_available_samples": len(all_samples),
                "excluded_sample_number": excluded_index + 1,
                "excluded_sample_group": (
                    "calibration" if excluded_index < len(calibration) else "previous_validation"
                ),
                "previous_validation": previous_result.get("validation"),
            },
        }
    )
    save_json(RESULT_FILE, result)
    rounds = list(session.get("previous_validation_rounds") or [])
    rounds.append(previous_validation)
    session["previous_validation_rounds"] = rounds
    session["refinement_samples"] = previous_validation
    session["validation_samples"] = []
    session["status"] = "collecting_validation"
    save_json(SESSION_FILE, session)
    print(f"已排除偏差最大的数据：总数据中的第 {excluded_index + 1} 个")
    print(f"重新计算的夹爪中心偏移：{result['flange_to_gripper_center_mm']} mm")
    print(f"剩余数据最大误差：{result['fit_max_error_mm']} mm")
    print("旧验收数据已用于重新计算，必须再采集3个全新姿势独立验收。")


def validate() -> None:
    session = load_json(SESSION_FILE)
    result = load_json(RESULT_FILE)
    poses = [sample["flange_pose_mm_deg"] for sample in session["validation_samples"]]
    validation = validate_tool_center(
        poses,
        result["flange_to_gripper_center_mm"],
        result["fixed_cone_tip_base_mm"],
    )
    result["validation"] = validation
    result["validated"] = bool(validation["passed"])
    result["motion_allowed"] = False
    save_json(RESULT_FILE, result)
    session["status"] = "validated" if validation["passed"] else "validation_failed"
    save_json(SESSION_FILE, session)
    print(f"独立验收最大误差：{validation['max_error_mm']} mm")
    print("验收通过。" if validation["passed"] else "验收未通过，不能开始夹挤。")


def show_status() -> None:
    if not SESSION_FILE.is_file():
        print("尚未开始夹爪中心标定。")
        return
    session = load_json(SESSION_FILE)
    print(f"状态：{session['status']}")
    print(f"标定姿势：{len(session['calibration_samples'])}/6")
    print(f"独立验收姿势：{len(session['validation_samples'])}/3")
    if RESULT_FILE.is_file():
        result = load_json(RESULT_FILE)
        print(f"当前偏移：{result['flange_to_gripper_center_mm']} mm")
        print(f"是否验收通过：{bool(result.get('validated'))}")


def main() -> int:
    parser = argparse.ArgumentParser(description="夹爪几何中心多姿态尖锥标定")
    sub = parser.add_subparsers(dest="command", required=True)
    sub.add_parser("start")
    capture_parser = sub.add_parser("capture")
    capture_parser.add_argument("--kind", choices=("calibration", "validation"), default="calibration")
    move_parser = sub.add_parser("move-next")
    move_parser.add_argument("--kind", choices=("calibration", "validation"), default="calibration")
    move_parser.add_argument("--confirmed", action="store_true")
    sub.add_parser("solve")
    sub.add_parser("refine")
    sub.add_parser("validate")
    sub.add_parser("status")
    args = parser.parse_args()
    if args.command == "start":
        start_session()
    elif args.command == "capture":
        capture(args.kind)
    elif args.command == "move-next":
        move_next(args.kind, args.confirmed)
    elif args.command == "solve":
        solve()
    elif args.command == "refine":
        refine_after_failed_validation()
    elif args.command == "validate":
        validate()
    else:
        show_status()
    return 0


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except Exception as error:
        print(f"标定停止：{error}", file=sys.stderr)
        raise SystemExit(2)
