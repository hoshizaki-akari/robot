from __future__ import annotations

from dataclasses import asdict, dataclass, field
from datetime import datetime
from enum import Enum
from math import sqrt
from typing import Any


class ControlState(str, Enum):
    DISCONNECTED = "设备未连接"
    IDLE = "空闲"
    POSITIONING = "撬拨定位"
    CLAMPING = "夹挤复位"
    HOLDING = "保持"
    PAUSED = "已暂停"
    EMERGENCY = "急停锁定"
    COMPLETED = "流程完成"


@dataclass
class ForceTorque:
    fx: float = 0.0
    fy: float = 0.0
    fz: float = 0.0
    tx: float = 0.0
    ty: float = 0.0
    tz: float = 0.0

    @property
    def force_norm(self) -> float:
        return sqrt(self.fx**2 + self.fy**2 + self.fz**2)

    @property
    def torque_norm(self) -> float:
        return sqrt(self.tx**2 + self.ty**2 + self.tz**2)


@dataclass
class Pose:
    x: float = 0.0
    y: float = 0.0
    z: float = 0.0
    rx: float = 0.0
    ry: float = 0.0
    rz: float = 0.0


@dataclass
class PatientCase:
    case_id: str
    patient_code: str
    side: str
    operator: str
    note: str = ""
    created_at: str = field(default_factory=lambda: datetime.now().isoformat(timespec="seconds"))

    def to_dict(self) -> dict[str, Any]:
        return asdict(self)


@dataclass
class ControlParameters:
    target_pry_mm: float = 8.0
    target_clamp_mm: float = 12.0
    speed_mm_s: float = 20.0
    force_limit_n: float = 80.0
    torque_limit_nm: float = 8.0
    hold_seconds: float = 3.0

    def validate(self) -> list[str]:
        errors: list[str] = []
        if not 0.0 <= self.target_pry_mm <= 150.0:
            errors.append("撬拨位移应为 0～150 mm")
        if not 0.0 <= self.target_clamp_mm <= 40.0:
            errors.append("夹挤位移应为 0～40 mm")
        if not 0.1 <= self.speed_mm_s <= 20.0:
            errors.append("运动速度应为 0.1～20.0 mm/s")
        if not 5.0 <= self.force_limit_n <= 150.0:
            errors.append("力上限应为 5～150 N")
        if not 0.5 <= self.torque_limit_nm <= 15.0:
            errors.append("力矩上限应为 0.5～15 Nm")
        if not 1.0 <= self.hold_seconds <= 30.0:
            errors.append("保持时间应为 1～30 s")
        return errors


@dataclass
class Sample:
    timestamp: str
    state: str
    pose: Pose
    wrench: ForceTorque
    progress: float

    def to_row(self) -> list[str]:
        return [
            self.timestamp, self.state,
            f"{self.pose.x:.3f}", f"{self.pose.y:.3f}", f"{self.pose.z:.3f}",
            f"{self.pose.rx:.3f}", f"{self.pose.ry:.3f}", f"{self.pose.rz:.3f}",
            f"{self.wrench.fx:.3f}", f"{self.wrench.fy:.3f}", f"{self.wrench.fz:.3f}",
            f"{self.wrench.tx:.3f}", f"{self.wrench.ty:.3f}", f"{self.wrench.tz:.3f}",
            f"{self.wrench.force_norm:.3f}", f"{self.progress:.1f}",
        ]
