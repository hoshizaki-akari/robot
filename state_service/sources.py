from __future__ import annotations

import math
import os
import time
from pathlib import Path
from typing import Any

from .ag95_reader import read_status as read_ag95_status

PROJECT_ROOT = Path(__file__).resolve().parents[1]
FAIRINO_SDK_ROOT = PROJECT_ROOT / "vendor" / "fairino-python-sdk" / "linux"
if FAIRINO_SDK_ROOT.is_dir() and str(FAIRINO_SDK_ROOT) not in os.sys.path:
    os.sys.path.insert(0, str(FAIRINO_SDK_ROOT))
from .d435_monitor import D435Monitor
from .schema import SCHEMA_VERSION, device_state, unavailable_snapshot, utc_now


def _as_floats(values: Any, expected: int) -> list[float]:
    result = [float(value) for value in values]
    if len(result) != expected:
        raise ValueError(f"期望 {expected} 个值，实际收到 {len(result)} 个")
    return result


def _query(result: Any, name: str, expected: int | None = None) -> Any:
    if not isinstance(result, (tuple, list)) or len(result) < 2:
        raise RuntimeError(f"{name} 返回格式异常：{result!r}")
    if int(result[0]) != 0:
        raise RuntimeError(f"{name} 失败，错误码 {result[0]}")
    value = result[1]
    return _as_floats(value, expected) if expected is not None else value


class ReplaySource:
    name = "replay"

    def __init__(self) -> None:
        self.started = time.monotonic()
        self.sequence = 0

    def close(self) -> None:
        return

    def sample(self) -> dict[str, Any]:
        self.sequence += 1
        elapsed = time.monotonic() - self.started
        wave = math.sin(elapsed * 0.8)
        joints = [round(base + wave * scale, 3) for base, scale in zip(
            [0, -35, 70, -45, 5, 10], [2, 1, 1.5, 1, 0.8, 1.2]
        )]
        tcp = [round(value, 3) for value in [320 + wave * 3, -40, 410 + wave * 2, 180, 0, 90]]
        wrench = [
            round(1.2 + wave * 0.4, 3),
            round(-0.8 + wave * 0.2, 3),
            round(12 + wave * 2.5, 3),
            round(0.08 + wave * 0.02, 3),
            round(-0.12 + wave * 0.03, 3),
            round(0.05 + wave * 0.01, 3),
        ]
        return {
            "schema_version": SCHEMA_VERSION,
            "sequence": self.sequence,
            "timestamp": utc_now(),
            "source": self.name,
            "system": device_state(True, message="回放数据运行中"),
            "fr5": device_state(
                True,
                connected=True,
                mode="manual",
                mode_raw=1,
                enabled=False,
                robot_state=1,
                program_state=1,
                errors={"main": 0, "sub": 0},
                emergency_stop=0,
                safety_stop=[0, 0],
                motion_done=1,
                joint_position_deg=joints,
                joint_velocity_deg_s=[round(wave * 0.2, 3)] * 6,
                flange_pose_mm_deg=tcp,
                tcp_pose_mm_deg=tcp,
                joint_torque_raw=[round(3 + index * 0.4 + wave * 0.1, 3) for index in range(6)],
                frame_id="base",
                message="确定性回放数据，不是真机",
            ),
            "kwr75d": device_state(
                True,
                connected=True,
                wrench=wrench,
                frame_id="RCS",
                units=["N", "N", "N", "Nm", "Nm", "Nm"],
                message="确定性回放数据，不是真机",
            ),
            "ag95": device_state(
                True,
                connected=True,
                initialized=True,
                init_status=1,
                motion_status=1,
                position_raw=int(500 + wave * 120),
                fault=0,
                timeout=False,
                frame_id="ag95",
                message="原始位置 0～1000，未换算毫米",
            ),
            "d435": device_state(
                True,
                connected=True,
                color_fps=30.0,
                depth_fps=30.0,
                last_color_frame=utc_now(),
                last_depth_frame=utc_now(),
                frame_id="camera_link",
                message="确定性回放数据，不是真机",
            ),
        }


class RealSource:
    name = "real"

    def __init__(self, robot_ip: str = "192.168.58.2") -> None:
        self.robot_ip = robot_ip
        self.sequence = 0
        self.robot: Any = None
        self.d435 = D435Monitor()
        self.d435.start()
        for key in ("NO_PROXY", "no_proxy"):
            values = [value for value in os.environ.get(key, "").split(",") if value]
            if robot_ip not in values:
                values.append(robot_ip)
            os.environ[key] = ",".join(values)

    def close(self) -> None:
        if self.robot is not None and hasattr(self.robot, "CloseRPC"):
            self.robot.CloseRPC()
        self.robot = None

    def _connect_robot(self) -> Any:
        if self.robot is None:
            from fairino import Robot

            self.robot = Robot.RPC(self.robot_ip)
            if self.robot is None:
                raise ConnectionError("Robot.RPC() 没有返回 FR5 对象")
        deadline = time.monotonic() + 3.0
        while time.monotonic() < deadline:
            state = getattr(self.robot, "robot_state_pkg", None)
            if state is not None and not isinstance(state, type):
                if int(getattr(state, "frame_head", 0)) == 0x5A5A:
                    return state
            time.sleep(0.05)
        raise TimeoutError("等待 FR5 实时状态帧超时")

    def _read_fr5(self) -> tuple[dict[str, Any], dict[str, Any]]:
        state = self._connect_robot()
        joints = _query(self.robot.GetActualJointPosDegree(), "关节角", 6)
        speeds = _query(self.robot.GetActualJointSpeedsDegree(), "关节速度", 6)
        tcp = _query(self.robot.GetActualTCPPose(), "TCP 位姿", 6)
        flange = _query(self.robot.GetActualToolFlangePose(), "法兰位姿", 6)
        torques = _query(self.robot.GetJointTorques(), "关节力矩", 6)
        wrench = _query(self.robot.FT_GetForceTorqueRCS(), "KWR75D 六维力", 6)
        raw_wrench = _query(
            self.robot.FT_GetForceTorqueOrigin(), "KWR75D 原始六维力", 6
        )
        force_config = _query(self.robot.FT_GetConfig(), "力传感器配置")
        force_active = bool(int(state.ft_sensor_active))
        fr5 = device_state(
            True,
            connected=True,
            mode={0: "auto", 1: "manual"}.get(int(state.robot_mode), "unknown"),
            mode_raw=int(state.robot_mode),
            enabled=bool(state.rbtEnableState),
            robot_state=int(state.robot_state),
            program_state=int(state.program_state),
            errors={"main": int(state.main_code), "sub": int(state.sub_code)},
            emergency_stop=int(state.EmergencyStop),
            safety_stop=[int(state.safety_stop0_state), int(state.safety_stop1_state)],
            motion_done=int(state.motion_done),
            joint_position_deg=joints,
            joint_velocity_deg_s=speeds,
            flange_pose_mm_deg=flange,
            tcp_pose_mm_deg=tcp,
            joint_torque_raw=torques,
            frame_id="base",
            message="FR5 SDK 只读状态",
        )
        kwr = device_state(
            force_active,
            connected=True,
            wrench=wrench,
            raw_wrench=raw_wrench,
            active=force_active,
            config=force_config,
            frame_id="RCS",
            units=["N", "N", "N", "Nm", "Nm", "Nm"],
            message=(
                "力传感器已启用"
                if force_active
                else "控制器中力传感器尚未启用"
            ),
        )
        return fr5, kwr

    @staticmethod
    def _read_ag95() -> dict[str, Any]:
        try:
            status = read_ag95_status()
            return device_state(
                True,
                connected=True,
                initialized=status["init_status"] == 1,
                init_status=status["init_status"],
                motion_status=status["motion_status"],
                position_raw=status["position_raw"],
                fault=0,
                timeout=False,
                frame_id="ag95",
                serial_device=status["device"],
                message="原始位置 0～1000，未换算毫米",
            )
        except Exception as error:
            return device_state(
                False,
                connected=False,
                initialized=False,
                init_status=None,
                motion_status=None,
                position_raw=None,
                fault=str(error),
                timeout=isinstance(error, TimeoutError),
                frame_id="ag95",
                message=str(error),
            )

    def sample(self) -> dict[str, Any]:
        self.sequence += 1
        base = unavailable_snapshot(self.name, "正在读取真机")
        try:
            fr5, kwr = self._read_fr5()
            base["fr5"], base["kwr75d"] = fr5, kwr
        except Exception as error:
            self.close()
            message = f"FR5 读取失败：{error}"
            base["fr5"]["message"] = message
            base["kwr75d"]["message"] = message

        base["ag95"] = self._read_ag95()
        d435 = self.d435.snapshot()
        base["d435"] = device_state(
            bool(d435.pop("valid")),
            frame_id="camera_link",
            **d435,
        )
        valid_count = sum(bool(base[name]["valid"]) for name in ("fr5", "kwr75d", "ag95", "d435"))
        base["system"] = device_state(
            valid_count > 0,
            message=f"4 类设备中 {valid_count} 类数据有效；无效设备不会伪装成正常",
        )
        base["sequence"] = self.sequence
        base["timestamp"] = utc_now()
        return base


def build_source(name: str) -> ReplaySource | RealSource:
    if name == "real":
        return RealSource()
    if name == "replay":
        return ReplaySource()
    raise ValueError(f"不支持的数据源：{name}")
