from __future__ import annotations

import json
import math
import os
import random
import subprocess
import sys
import threading
import time
from dataclasses import replace
from pathlib import Path
from typing import Any, Protocol
from urllib.error import URLError
from urllib.parse import urlsplit, urlunsplit
from urllib.request import urlopen

from .models import ControlParameters, ControlState, ForceTorque, Pose
try:
    from platform_a.pry_buckle_vision import PryBuckleVisionWorker
except ImportError:
    from pry_buckle_vision import PryBuckleVisionWorker


class DeviceError(RuntimeError):
    pass


class RobotAdapter(Protocol):
    connected: bool
    servo_enabled: bool
    progress: float
    gripper_opening: float
    read_only: bool
    mode_label: str
    opening_unit: str
    status_text: str
    data_age_ms: int | None

    def connect(self) -> None: ...
    def disconnect(self) -> None: ...
    def enable(self) -> None: ...
    def disable(self) -> None: ...
    def reset(self) -> None: ...
    def reset_clamp_workflow(self) -> None: ...
    def start_clamp_workflow(self, clamp_mm: float, speed_mm_s: float = 10.0) -> None: ...
    def pause_motion(self) -> None: ...
    def resume_motion(self) -> None: ...
    def stop_motion(self) -> None: ...
    def emergency_stop(self) -> None: ...
    def reset_emergency(self) -> None: ...
    def jog_gripper(self, delta_mm: float) -> None: ...
    def set_gripper(self, opening: float) -> None: ...
    def step(
        self, state: ControlState, params: ControlParameters, dt: float
    ) -> tuple[Pose, ForceTorque, float, bool]: ...


class SimulatedRobotAdapter:
    """离线仿真适配器；只用于演示，不连接真实设备。"""

    read_only = False
    mode_label = "离线仿真"
    opening_unit = "mm"

    def __init__(self) -> None:
        self.connected = False
        self.servo_enabled = False
        self.pose = Pose()
        self.wrench = ForceTorque()
        self.progress = 0.0
        self.gripper_opening = 30.0
        self._hold_elapsed = 0.0
        self.status_text = "仿真设备未连接"
        self.data_age_ms: int | None = None

    def connect(self) -> None:
        self.connected = True
        self.status_text = "仿真设备已连接"

    def disconnect(self) -> None:
        self.servo_enabled = False
        self.connected = False
        self.status_text = "仿真设备已断开"

    def enable(self) -> None:
        if not self.connected:
            raise DeviceError("设备尚未连接")
        self.servo_enabled = True

    def disable(self) -> None:
        self.servo_enabled = False

    def reset(self) -> None:
        if not self.connected:
            raise DeviceError("设备尚未连接")
        self.pose = Pose()
        self.wrench = ForceTorque()
        self.progress = 0.0
        self.gripper_opening = 30.0
        self._hold_elapsed = 0.0

    def reset_clamp_workflow(self) -> None:
        self.reset()

    def start_clamp_workflow_v2(self, clamp_mm: float, speed_mm_s: float = 10.0) -> None:
        """Use the pry observation pose and horizontal-diameter detector for clamp."""
        if not 0.0 <= float(clamp_mm) <= 40.0:
            raise DeviceError("夹挤位移必须在0到40毫米之间")
        if not 0.1 <= float(speed_mm_s) <= 20.0:
            raise DeviceError("运动速度必须在0.1到20毫米/秒之间")
        root = Path(__file__).resolve().parents[2]
        env = os.environ.copy()
        env["FR5_PLATFORM_ROOT"] = str(root)
        env["PYTHONPATH"] = str(root) + os.pathsep + env.get("PYTHONPATH", "")
        # The script returns to the shared pry observation pose, reacquires the
        # target there, then moves to the target without the pry Y- offset.
        vision = dict(self.pry_vision.result)
        center = vision.get("clamp_contact_center_camera_mm")
        width = vision.get("width_mm")
        if not vision.get("valid") or not center or width is None:
            raise DeviceError(vision.get("message") or "夹挤视觉目标无效")
        command = [
            sys.executable, str(root / "scripts/clamp_acquire_and_move.py"),
            "--clamp-mm", f"{float(clamp_mm):.3f}",
            "--speed-mm-s", f"{float(speed_mm_s):.3f}",
            "--center-camera-mm", ",".join(f"{float(v):.5f}" for v in center),
            "--width-mm", f"{float(width):.5f}",
        ]
        # The standalone workflow owns the temporary ROS vision worker while
        # it returns to the observation pose and reacquires the target.
        self.pry_vision.stop()
        completed = subprocess.run(command, cwd=root, env=env, text=True,
                                   capture_output=True, timeout=300, check=False)
        output = "\n".join(part.strip() for part in (completed.stdout, completed.stderr) if part.strip())
        if completed.returncode != 0:
            raise DeviceError(output.splitlines()[-1] if output else "夹挤流程失败")
        self.status_text = "夹挤已按撬拨视觉流程到达夹持点"
        self.last_clamp_output = output

    def start_clamp_workflow(self, clamp_mm: float, speed_mm_s: float = 10.0) -> None:
        del clamp_mm, speed_mm_s
        self.begin_task()

    def begin_task(self) -> None:
        self.progress = 0.0
        self._hold_elapsed = 0.0

    def set_gripper(self, opening_mm: float) -> None:
        if not self.connected or not self.servo_enabled:
            raise DeviceError("设备未连接或伺服未使能")
        self.gripper_opening = max(0.0, min(40.0, opening_mm))

    def step(
        self, state: ControlState, p: ControlParameters, dt: float
    ) -> tuple[Pose, ForceTorque, float, bool]:
        if not self.connected:
            return self.pose, self.wrench, self.progress, False
        noise = lambda scale: random.uniform(-scale, scale)
        done = False
        if state == ControlState.POSITIONING:
            self.pose.z = min(p.target_pry_mm, self.pose.z + p.speed_mm_s * dt)
            self.progress = 100.0 if p.target_pry_mm == 0 else self.pose.z / p.target_pry_mm * 100.0
            self.wrench = ForceTorque(
                noise(0.8), 5 + self.pose.z * 1.7 + noise(1),
                9 + self.pose.z * 2.4 + noise(1.2), noise(.15),
                .4 + noise(.1), noise(.1)
            )
            done = self.pose.z >= p.target_pry_mm
        elif state == ControlState.CLAMPING:
            traveled = 30.0 - self.gripper_opening
            traveled = min(p.target_clamp_mm, traveled + p.speed_mm_s * dt)
            self.gripper_opening = 30.0 - traveled
            self.pose.x = traveled
            self.progress = 100.0 if p.target_clamp_mm == 0 else traveled / p.target_clamp_mm * 100.0
            self.wrench = ForceTorque(
                8 + traveled * 2.8 + noise(1.4), noise(.8),
                12 + traveled * 1.4 + noise(1), .3 + noise(.1),
                noise(.12), .6 + traveled * .12 + noise(.1)
            )
            done = traveled >= p.target_clamp_mm
        elif state == ControlState.HOLDING:
            self._hold_elapsed += dt
            self.progress = min(100.0, self._hold_elapsed / p.hold_seconds * 100.0)
            self.wrench = replace(
                self.wrench,
                fx=self.wrench.fx + noise(.5),
                fz=self.wrench.fz + noise(.5),
            )
            done = self._hold_elapsed >= p.hold_seconds
        elif state in (ControlState.IDLE, ControlState.PAUSED, ControlState.COMPLETED):
            self.wrench = ForceTorque(
                *(value * .92 for value in (
                    self.wrench.fx, self.wrench.fy, self.wrench.fz,
                    self.wrench.tx, self.wrench.ty, self.wrench.tz
                ))
            )
        return self.pose, self.wrench, min(100.0, self.progress), done


class RealDeviceAdapter:
    """后台读取统一状态服务；只额外开放夹挤复位这一项真机写操作。"""

    read_only = True
    mode_label = "真机只读"
    opening_unit = "原始值"

    def __init__(self, url: str | None = None) -> None:
        self.url = url or os.environ.get(
            "FR5_STATE_URL", "http://127.0.0.1:8765/api/state"
        )
        self.connected = False
        self.servo_enabled = False
        self.progress = 0.0
        self.gripper_opening = 0.0
        self.pose = Pose()
        self.wrench = ForceTorque()
        self.status_text = "真机状态服务未连接"
        self.data_age_ms: int | None = None
        self.snapshot: dict[str, Any] | None = None
        self.camera_frame_png: bytes | None = None
        self.vision_result: dict[str, Any] = {
            "valid": False,
            "motion_allowed": False,
            "message": "夹挤识别尚未连接",
        }
        self.pry_vision = PryBuckleVisionWorker()
        self.vision_mode = "idle"
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._lock = threading.Lock()
        parts = urlsplit(self.url)
        self.camera_url = urlunsplit(
            (parts.scheme, parts.netloc, "/api/d435/color.png", "", "")
        )
        self.vision_url = urlunsplit(
            (parts.scheme, parts.netloc, "/api/platform-a/clamp/plan", "", "")
        )

    def connect(self) -> None:
        if self._thread is None or not self._thread.is_alive():
            self._stop.clear()
            self._thread = threading.Thread(
                target=self._poll_loop,
                name="platform-a-state-poller",
                daemon=True,
            )
            self._thread.start()
        deadline = time.monotonic() + 2.5
        while time.monotonic() < deadline:
            with self._lock:
                if self.snapshot is not None:
                    return
            time.sleep(0.05)
        raise DeviceError(f"无法连接统一状态服务：{self.url}")

    def disconnect(self) -> None:
        self._stop.set()
        if self._thread is not None:
            self._thread.join(timeout=1.0)
        self._thread = None
        with self._lock:
            self.connected = False
            self.servo_enabled = False
            self.snapshot = None
            self.camera_frame_png = None
            self.pry_vision.stop()
            self.vision_mode = "idle"
            self.vision_result = {
                "valid": False,
                "motion_allowed": False,
                "message": "夹挤识别已断开",
            }
            self.status_text = "真机状态服务已断开"

    def _poll_loop(self) -> None:
        while not self._stop.is_set():
            try:
                with urlopen(self.url, timeout=0.8) as response:
                    snapshot = json.load(response)
                fr5 = snapshot["fr5"]
                kwr = snapshot["kwr75d"]
                ag95 = snapshot["ag95"]
                tcp = fr5.get("tcp_pose_mm_deg") or [0.0] * 6
                wrench = kwr.get("wrench") or [0.0] * 6
                if len(tcp) == 6 and len(wrench) == 6:
                    with self._lock:
                        self.snapshot = snapshot
                        self.connected = bool(fr5.get("valid"))
                        self.servo_enabled = bool(fr5.get("enabled"))
                        self.pose = Pose(*map(float, tcp))
                        self.wrench = ForceTorque(*map(float, wrench))
                        position = ag95.get("position_raw")
                        self.gripper_opening = float(position) if position is not None else 0.0
                        self.data_age_ms = max(
                            int(fr5.get("age_ms", 0)),
                            int(kwr.get("age_ms", 0)),
                        )
                        source = snapshot.get("source", "unknown")
                        mode = fr5.get("mode", "unknown")
                        self.status_text = (
                            f"数据源={source} FR5={mode} "
                            f"KWR75D={'有效' if kwr.get('valid') else '无效'} "
                            f"AG95={'有效' if ag95.get('valid') else '无效'} "
                            f"age={self.data_age_ms}ms"
                        )
                try:
                    with urlopen(self.vision_url, timeout=0.8) as response:
                        vision_result = json.load(response)
                    with self._lock:
                        self.vision_result = vision_result
                except (OSError, ValueError, URLError):
                    pass
                try:
                    with urlopen(self.camera_url, timeout=0.8) as response:
                        camera_frame = response.read()
                    if camera_frame.startswith(b"\x89PNG"):
                        with self._lock:
                            self.camera_frame_png = camera_frame
                except (OSError, URLError):
                    with self._lock:
                        self.camera_frame_png = None
            except (OSError, ValueError, KeyError, TypeError, URLError) as error:
                with self._lock:
                    self.connected = False
                    self.servo_enabled = False
                    self.data_age_ms = None
                    self.status_text = f"状态服务不可用：{error}"
            self._stop.wait(0.2)

    @staticmethod
    def _deny() -> None:
        raise DeviceError("真机模式除夹挤复位外，不发送其他使能、运动或夹爪命令")

    def start_pry_vision(self) -> None:
        with self._lock:
            if not self.connected:
                raise DeviceError("真实设备数据尚未连接")
        self.vision_mode = "pry"
        self.pry_vision.start()

    def stop_pry_vision(self) -> None:
        self.pry_vision.stop()
        if self.vision_mode in ("pry", "clamp"):
            self.vision_mode = "idle"

    def start_pry_workflow(self, result: dict[str, Any]) -> None:
        center = result.get("clamp_contact_center_camera_mm")
        if not result.get("valid") or not center or len(center) != 3:
            raise DeviceError("撬拨视觉目标无效，未发送运动命令")
        root = Path(__file__).resolve().parents[2]
        command = [
            sys.executable, str(root / "scripts/pry_move_to_clamp.py"),
            "--center-camera-mm", ",".join(f"{float(v):.5f}" for v in center),
            "--surface-gap-mm", f"{float(result.get('surface_to_upper_midpoint_gap_mm', 0.0)):.5f}",
            "--pry-position-mm", f"{float(result.get('pry_lever_arm_mm', 100.0)):.5f}",
            "--pry-direction", str(result.get("pry_direction", "X_PLUS")),
            "--pry-angle-deg", f"{float(result.get('pry_angle_deg', 0.0)):.5f}",
            "--pry-lever-arm-mm", f"{float(result.get('pry_lever_arm_mm', 100.0)):.5f}",
            "--speed-mm-s", f"{float(result.get('pry_speed_mm_s', 40.0)):.5f}",
            "--confirmed-clear",
            "--experimental-first-six",
            "--close-after",
        ]
        environment = os.environ.copy()
        environment["FR5_PLATFORM_ROOT"] = str(root)
        environment["PYTHONPATH"] = str(root) + os.pathsep + environment.get("PYTHONPATH", "")
        completed = subprocess.run(
            command, cwd=root, env=environment, text=True,
            capture_output=True, timeout=180, check=False,
        )
        output = "\n".join(part.strip() for part in (completed.stdout, completed.stderr) if part.strip())
        if completed.returncode != 0:
            raise DeviceError(output.splitlines()[-1] if output else "撬拨移动失败")
        self.status_text = "撬拨夹爪已到达夹持点并保持张开"
        self.last_pry_output = output

    def start_clamp_workflow_v2(self, clamp_mm: float, speed_mm_s: float = 10.0) -> None:
        """Run the new clamp flow from the shared pry observation pose."""
        with self._lock:
            connected = self.connected
            age = self.data_age_ms
        if not connected or age is None or age > 1500:
            raise DeviceError("真实设备数据未连接或已经过期")
        if not 0.0 <= float(clamp_mm) <= 40.0:
            raise DeviceError("夹挤位移必须在0到40毫米之间")
        if not 0.1 <= float(speed_mm_s) <= 20.0:
            raise DeviceError("移动速度必须在0.1到20毫米/秒之间")
        root = Path(__file__).resolve().parents[2]
        env = os.environ.copy()
        env["FR5_PLATFORM_ROOT"] = str(root)
        env["PYTHONPATH"] = str(root) + os.pathsep + env.get("PYTHONPATH", "")
        vision = dict(self.pry_vision.result)
        center = vision.get("clamp_contact_center_camera_mm")
        width = vision.get("width_mm")
        if not vision.get("valid") or not center or width is None:
            raise DeviceError(vision.get("message") or "夹挤视觉目标无效")
        command = [
            sys.executable, str(root / "scripts/clamp_acquire_and_move.py"),
            "--clamp-mm", f"{float(clamp_mm):.3f}",
            "--speed-mm-s", f"{float(speed_mm_s):.3f}",
            "--center-camera-mm", ",".join(f"{float(v):.5f}" for v in center),
            "--width-mm", f"{float(width):.5f}",
        ]
        self.pry_vision.stop()
        completed = subprocess.run(
            command, cwd=root, env=env, text=True,
            capture_output=True, timeout=300, check=False,
        )
        output = "\n".join(part.strip() for part in (completed.stdout, completed.stderr) if part.strip())
        if completed.returncode != 0:
            raise DeviceError(output.splitlines()[-1] if output else "夹挤流程失败")
        self.status_text = "夹挤已按撬拨视觉流程到达夹持点"
        self.last_clamp_output = output

    def enable(self) -> None:
        self._deny()

    def disable(self) -> None:
        self._deny()

    def reset(self) -> None:
        self._deny()

    def reset_clamp_workflow(self) -> None:
        """唯一允许的真机写操作：张开夹爪并沿已保存路线回初始位置。"""
        with self._lock:
            connected = self.connected
            age = self.data_age_ms
        if not connected or age is None or age > 1500:
            raise DeviceError("真实设备数据未连接或已经过期")
        root = Path(__file__).resolve().parents[2]
        env = os.environ.copy()
        env["FR5_PLATFORM_ROOT"] = str(root)
        env["PYTHONPATH"] = str(root) + os.pathsep + env.get("PYTHONPATH", "")
        commands = (
            ("张开夹爪", [sys.executable, str(root / "scripts/set_ag95_opening.py"), "95", "--speed", "20", "--yes"]),
            # Both branches now share the pry observation zero. Keep the old
            # clamp return script in the backup, but never use its old zero.
            ("机械臂回撬拨观察零点", [sys.executable, str(root / "scripts/pry_move_to_base.py")]),
        )
        output: list[str] = []
        for label, command in commands:
            attempts = 5 if label == "张开夹爪" else 1
            for attempt in range(1, attempts + 1):
                try:
                    result = subprocess.run(
                        command,
                        cwd=root,
                        env=env,
                        text=True,
                        capture_output=True,
                        timeout=210,
                        check=False,
                    )
                except subprocess.TimeoutExpired as error:
                    raise DeviceError("夹挤复位等待超时，机械臂已停止或需现场检查") from error
                text = "\n".join(part.strip() for part in (result.stdout, result.stderr) if part.strip())
                if result.returncode == 0:
                    if text:
                        output.append(text)
                    if label == "张开夹爪":
                        # The AG95 command can return before the shared state
                        # service has published the new position. Do not let
                        # the zero-pose executor reject a stale closed value.
                        open_deadline = time.monotonic() + 12.0
                        while time.monotonic() < open_deadline:
                            with self._lock:
                                ag95_state = (self.snapshot or {}).get("ag95") or {}
                                opened = bool(ag95_state.get("valid")) and int(ag95_state.get("position_raw") or 0) >= 450
                            if opened:
                                break
                            time.sleep(0.2)
                        else:
                            raise DeviceError("夹爪张开命令已返回，但实时开度仍未达到标定阈值")
                    break
                serial_busy = "Resource temporarily unavailable" in text or "exclusive lock" in text
                if serial_busy and attempt < attempts:
                    time.sleep(0.4)
                    continue
                detail = text.splitlines()[-1] if text else f"返回码{result.returncode}"
                raise DeviceError(f"夹挤复位失败（{label}）：{detail}")
        self.status_text = "夹挤复位完成：夹爪全开，机械臂位于初始观察位置"
        self.last_reset_output = "\n".join(output)

    def start_clamp_workflow(self, clamp_mm: float, speed_mm_s: float = 10.0) -> None:
        """从初始观察位置执行已标定的靠近路线，并按给定位移夹挤。"""
        with self._lock:
            connected = self.connected
            age = self.data_age_ms
        if not connected or age is None or age > 1500:
            raise DeviceError("真实设备数据未连接或已经过期")
        if not 1.0 <= float(clamp_mm) <= 20.0:
            raise DeviceError("夹挤位移必须在1到20毫米之间")
        if not 0.1 <= float(speed_mm_s) <= 20.0:
            raise DeviceError("移动速度必须在0.1到20毫米/秒之间")

        root = Path(__file__).resolve().parents[2]
        env = os.environ.copy()
        env["FR5_PLATFORM_ROOT"] = str(root)
        env["PYTHONPATH"] = str(root) + os.pathsep + env.get("PYTHONPATH", "")
        approach = root / "scripts/platform_a_plane_approach.py"
        clamp = root / "scripts/platform_a_execute_clamp.py"
        commands = [
            ("计算夹挤路线", [sys.executable, str(approach), "start", "--clamp-mm", f"{clamp_mm:.3f}"], 60),
            ("移动到对准位置", [sys.executable, str(approach), "move", "align", "--confirmed-clear", "--speed-mm-s", f"{speed_mm_s:.3f}"], 240),
            ("移动到足跟表面外60毫米", [sys.executable, str(approach), "move", "pre", "--confirmed-clear", "--speed-mm-s", f"{speed_mm_s:.3f}"], 240),
            ("移动到足跟表面外20毫米", [sys.executable, str(approach), "move", "near", "--confirmed-clear", "--speed-mm-s", f"{speed_mm_s:.3f}"], 240),
            ("移动到夹持深度", [sys.executable, str(approach), "move", "contact_center", "--confirmed-clear", "--speed-mm-s", f"{speed_mm_s:.3f}"], 240),
            (
                "执行夹挤",
                [sys.executable, str(clamp), "--execute", "--creep-retry", "--force-level", "100", "--speed", "5", "--accept-shortfall-mm", "2.0"],
                120,
            ),
        ]
        output: list[str] = []
        for label, command, timeout in commands:
            attempts = 5 if label == "执行夹挤" else 1
            for attempt in range(1, attempts + 1):
                self.status_text = label
                try:
                    result = subprocess.run(
                        command, cwd=root, env=env, text=True,
                        capture_output=True, timeout=timeout, check=False,
                    )
                except subprocess.TimeoutExpired as error:
                    raise DeviceError(f"{label}等待超时，流程已停止") from error
                text = "\n".join(
                    part.strip() for part in (result.stdout, result.stderr) if part.strip()
                )
                if result.returncode == 0:
                    if text:
                        output.append(text)
                    break
                serial_busy = (
                    "Resource temporarily unavailable" in text
                    or "exclusive lock" in text
                )
                if serial_busy and attempt < attempts:
                    time.sleep(0.4)
                    continue
                detail = text.splitlines()[-1] if text else f"返回码{result.returncode}"
                raise DeviceError(f"{label}失败：{detail}")
        self.status_text = f"夹挤完成：目标位移{clamp_mm:.1f}毫米"
        self.last_clamp_output = "\n".join(output)

    def set_gripper(self, opening: float) -> None:
        self._deny()

    def _robot_command(self, action: str) -> None:
        from fairino import Robot
        robot = Robot.RPC("192.168.58.2")
        if robot is None: raise DeviceError("无法连接FR5控制器")
        try:
            if action in ("pause", "stop", "emergency"): robot.StopMotion()
            if action == "emergency": robot.RobotEnable(0); robot.Mode(1)
            elif action == "reset": robot.Mode(0); robot.RobotEnable(1); time.sleep(0.3); robot.Mode(1)
            elif action == "resume": robot.Mode(0); robot.RobotEnable(1)
            else: robot.Mode(1)
        finally: robot.CloseRPC()

    def pause_motion(self) -> None: self._robot_command("pause")
    def resume_motion(self) -> None: self._robot_command("resume")
    def stop_motion(self) -> None: self._robot_command("stop")
    def emergency_stop(self) -> None: self._robot_command("emergency")
    def reset_emergency(self) -> None: self._robot_command("reset")

    def jog_gripper(self, delta_mm: float) -> None:
        with self._lock: raw = int((self.snapshot or {}).get("ag95", {}).get("position_raw") or 0)
        target = max(0, min(1000, int(round(raw + float(delta_mm) / 95.0 * 1000.0))))
        root = Path(__file__).resolve().parents[2]
        result = subprocess.run([sys.executable, str(root / "scripts/set_ag95_opening.py"), str(target), "--speed", "5", "--yes"], cwd=root, text=True, capture_output=True, timeout=30, check=False)
        if result.returncode != 0: raise DeviceError((result.stderr or result.stdout or "夹爪命令失败").strip().splitlines()[-1])

    def begin_task(self) -> None:
        self._deny()

    def step(
        self, state: ControlState, params: ControlParameters, dt: float
    ) -> tuple[Pose, ForceTorque, float, bool]:
        del state, params, dt
        with self._lock:
            pose = replace(self.pose)
            wrench = replace(self.wrench)
            connected = self.connected
            age = self.data_age_ms
        if age is not None and age > 1500:
            connected = False
            self.status_text = f"真机数据已过期：{age}ms"
        self.connected = connected
        return pose, wrench, 0.0, False


def make_adapter(mode: str) -> RobotAdapter:
    if mode == "real":
        return RealDeviceAdapter()
    if mode == "simulation":
        return SimulatedRobotAdapter()
    raise ValueError(f"未知设备模式：{mode}")
