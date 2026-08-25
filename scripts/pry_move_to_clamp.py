#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, subprocess, sys, time
from pathlib import Path
from urllib.request import urlopen
import numpy as np

ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))
from platform_a.handeye_calibration import pose_to_matrix
from platform_a.tool_center_calibration import flange_pose_for_center

STATE_URL = "http://127.0.0.1:8765/api/state"
ROBOT_IP = "192.168.58.2"

def state() -> dict:
    with urlopen(STATE_URL, timeout=3.0) as response:
        return json.load(response)

def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--center-camera-mm", required=True)
    parser.add_argument("--surface-gap-mm", type=float, required=True)
    parser.add_argument("--pry-position-mm", type=float, default=100.0)
    parser.add_argument("--confirmed-clear", action="store_true")
    parser.add_argument("--experimental-first-six", action="store_true")
    parser.add_argument("--close-after", action="store_true")
    parser.add_argument("--pry-direction", default="X_PLUS")
    parser.add_argument("--pry-angle-deg", type=float, default=0.0)
    parser.add_argument("--pry-lever-arm-mm", type=float, default=100.0)
    parser.add_argument("--speed-mm-s", type=float, default=40.0)
    parser.add_argument("--dry-run", action="store_true")
    args = parser.parse_args()
    if not args.confirmed_clear:
        raise RuntimeError("现场确认未通过")
    if not 0.0 <= args.surface_gap_mm <= 150.0 or not 0.0 < args.pry_position_mm <= 150.0:
        raise RuntimeError("间距或撬拨位置参数超出范围")
    center_camera = np.asarray([float(v) for v in args.center_camera_mm.split(",")], dtype=np.float64)
    if center_camera.shape != (3,) or not np.all(np.isfinite(center_camera)):
        raise RuntimeError("相机夹持中心无效")
    calibration = json.loads((ROOT / "platform_a/config/handeye_calibration.json").read_text())
    tcp = json.loads((ROOT / "platform_a/config/gripper_tcp_calibration.json").read_text())
    if not calibration.get("validated"):
        raise RuntimeError("手眼标定未通过")
    if not tcp.get("validated") and not args.experimental_first_six:
        raise RuntimeError("TCP标定未通过")
    if args.experimental_first_six and float(tcp.get("fit_max_error_mm", 999.0)) > 3.0:
        raise RuntimeError("前六姿势拟合误差超过3 mm")
    snapshot = state(); fr5 = snapshot.get("fr5") or {}; ag95 = snapshot.get("ag95") or {}
    if not fr5.get("valid") or int(fr5.get("motion_done", 0)) != 1:
        raise RuntimeError("FR5状态无效或未停止")
    if int((fr5.get("errors") or {}).get("main", 0)) or int((fr5.get("errors") or {}).get("sub", 0)):
        raise RuntimeError("FR5存在报警")
    if not ag95.get("valid") or int(ag95.get("position_raw") or 0) < 450:
        raise RuntimeError("夹爪没有保持张开")
    current = [float(v) for v in fr5["flange_pose_mm_deg"]]
    base_t_flange = pose_to_matrix(current, calibration.get("euler_convention", "rpy"))
    flange_t_camera = np.asarray(calibration["flange_T_camera"], dtype=np.float64)
    center_base = (base_t_flange @ flange_t_camera @ np.r_[center_camera / 1000.0, 1.0])[:3] * 1000.0
    target = np.asarray(flange_pose_for_center(center_base, current[3:], tcp["flange_to_gripper_center_mm"]), dtype=np.float64)
    total = float(args.surface_gap_mm + args.pry_position_mm)
    final_target = target.copy(); final_target[:3] += pose_to_matrix(target.tolist())[:3, :3][:, 1] * (-total)
    distance = float(np.linalg.norm(target[:3] - np.asarray(current[:3])))
    if distance > 320.0 or not 80.0 <= final_target[2] <= 700.0:
        raise RuntimeError(f"目标超出安全范围: distance={distance:.1f} final_z={final_target[2]:.1f}")
    print(json.dumps({"current": current, "center_camera_mm": center_camera.tolist(), "center_base_mm": center_base.tolist(), "center_target": target.tolist(), "final_tool_y_minus_target": final_target.tolist(), "surface_gap_mm": args.surface_gap_mm, "pry_position_mm": args.pry_position_mm, "tool_y_minus_total_mm": total, "pre_approach_tool_y_plus_retreat_mm": 10.0}, ensure_ascii=False))
    if args.dry_run:
        return 0
    from fairino import Robot
    robot = Robot.RPC(ROBOT_IP)
    if robot is None:
        raise RuntimeError("无法连接FR5")
    def move_wait(pose: np.ndarray, speed: int, timeout: float) -> None:
        code = robot.MoveL(pose.tolist(), 0, 0, vel=float(speed), acc=0.0, ovl=float(speed), blendR=-1.0, overSpeedStrategy=2, speedPercent=speed)
        if int(code) != 0:
            raise RuntimeError(f"MoveL rejected: {code}")
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            s = state().get("fr5") or {}; actual = np.asarray(s.get("flange_pose_mm_deg") or [], dtype=np.float64)
            if int(s.get("emergency_stop", 0) or 0) or any(s.get("safety_stop") or []):
                raise RuntimeError("触发急停或安全停止")
            if actual.shape == (6,) and int(s.get("motion_done", 0)) == 1 and np.linalg.norm(actual[:3] - pose[:3]) <= 2.0:
                return
            time.sleep(0.15)
        raise TimeoutError("MoveL超时")
    try:
        if int(robot.Mode(0)) != 0 or int(robot.RobotEnable(1)) != 0:
            raise RuntimeError("无法进入自动使能状态")
        robot.ResumeMotion()
        robot.ProgramResume()
        move_wait(target, int(args.speed_mm_s), 180)
        print("已到达夹持中心")

        # 撬拨专用安全顺序：到达夹持点后先沿 Tool Y+ 回退 10 mm，
        # 再执行原有的 Tool Y- 靠近，不影响夹挤工作流。
        tool_rotation = pose_to_matrix(target.tolist())[:3, :3]
        retreat_mm = 10.0
        retreat_target = target.copy()
        retreat_target[:3] += tool_rotation[:, 1] * retreat_mm
        move_wait(retreat_target, max(1, int(args.speed_mm_s * 0.75)), 60)
        print(f"撬拨安全回退：Tool Y+ {retreat_mm:.1f} mm")

        move_wait(final_target, max(1, int(args.speed_mm_s * 0.75)), 120)
        print(f"已完成Tool Y-移动 {total:.1f} mm")
        if args.close_after:
            close = subprocess.run([sys.executable, str(ROOT / "scripts/set_ag95_opening.py"), "0", "--speed", "5", "--force", "100", "--yes"], cwd=ROOT, text=True, capture_output=True, timeout=30, check=False)
            if close.returncode != 0 and "提前碰到物体" not in (close.stderr + close.stdout):
                raise RuntimeError((close.stderr or close.stdout or "夹爪闭合失败").strip())
            print("夹爪已缓慢闭合")
        if args.close_after and args.pry_angle_deg > 0.0:
            angle = float(args.pry_angle_deg)
            if not 0.0 < angle <= 89.0:
                raise RuntimeError("撬拨角度必须在0到89度之间")
            direction = str(args.pry_direction).upper()
            axes = {
                "X_PLUS": (0, 1.0), "X_MINUS": (0, -1.0),
                "Y_PLUS": (2, -1.0), "Y_MINUS": (2, 1.0),
            }
            if direction not in axes:
                raise RuntimeError(f"未知撬拨方向: {direction}")
            axis, sign = axes[direction]
            distance = float(args.pry_position_mm) * float(np.tan(np.deg2rad(angle)))
            pry_target = final_target.copy()
            pry_target[:3] += pose_to_matrix(final_target.tolist())[:3, :3][:, axis] * (sign * distance)
            move_wait(pry_target, max(1, int(args.speed_mm_s * 0.75)), 150)
            print(f"已完成笛卡尔撬拨: {direction}, {angle:.1f} deg, {sign*distance:.1f} mm")
        if False and args.pry_angle_deg > 0.0:
            trajectory = subprocess.run([
                sys.executable, str(ROOT / "scripts/pry_execute_arc.py"),
                "--direction", args.pry_direction,
                "--angle-deg", f"{args.pry_angle_deg:.5f}",
                "--lever-mm", f"{args.pry_lever_arm_mm:.5f}",
            ], cwd=ROOT, text=True, capture_output=True, timeout=180, check=False)
            if trajectory.returncode != 0:
                raise RuntimeError((trajectory.stderr or trajectory.stdout or "撬拨轨迹失败").strip())
            print(trajectory.stdout.strip())
        return 0
    except Exception:
        try: robot.StopMotion(); robot.Mode(1); robot.RobotEnable(0)
        except Exception: pass
        raise
    finally:
        try: robot.Mode(1); robot.CloseRPC()
        except Exception: pass

if __name__ == "__main__":
    raise SystemExit(main())
