from __future__ import annotations

from datetime import datetime
from typing import Callable

from .device import RobotAdapter
from .models import ControlParameters, ControlState, Sample
from .storage import RecordStore


class SafetyError(RuntimeError):
    pass


class RobotController:
    def __init__(self, device: RobotAdapter, store: RecordStore, notify: Callable[[str], None]) -> None:
        self.device = device
        self.store = store
        self.notify = notify
        self.state = ControlState.DISCONNECTED
        self.previous_state = ControlState.IDLE
        self.params = ControlParameters()
        self.last_sample_second = -1

    def _set_state(self, state: ControlState, message: str) -> None:
        self.state = state
        self.store.event("INFO", message)
        self.notify(message)

    def connect(self) -> None:
        self.device.connect()
        if not self.device.read_only:
            self.device.enable()
            message = "仿真设备连接成功，仿真伺服已使能"
        else:
            message = "真机状态服务连接成功；已开放状态监测与夹挤复位"
        self._set_state(ControlState.IDLE, message)

    def disconnect(self) -> None:
        if self.state not in (ControlState.IDLE, ControlState.COMPLETED, ControlState.DISCONNECTED):
            raise SafetyError("请先停止当前任务再断开设备")
        self.device.disconnect()
        self._set_state(ControlState.DISCONNECTED, "设备已断开")

    def update_parameters(self, params: ControlParameters) -> None:
        errors = params.validate()
        if errors:
            raise ValueError("；".join(errors))
        preview_parameters = (
            self.device.read_only
            and self.state in (ControlState.POSITIONING, ControlState.CLAMPING)
            and getattr(self.device, "vision_mode", "idle") in ("pry", "clamp")
        )
        if not preview_parameters and self.state not in (ControlState.IDLE, ControlState.COMPLETED):
            raise SafetyError("仅可在空闲或完成状态修改参数")
        self.params = params
        self.store.event("INFO", "控制参数校验通过")

    def start_positioning(self) -> None:
        if self.device.read_only:
            if self.state not in (ControlState.IDLE, ControlState.COMPLETED):
                raise SafetyError("请先连接设备并保持空闲")
            if self.store.current_case is None:
                raise SafetyError("请先建立病例")
            if not self.device.connected:
                raise SafetyError("真实设备数据尚未连接")
            self.device.start_pry_vision()
            self._set_state(ControlState.POSITIONING, "开始撬拨：启动独立水平直径视觉")
            return
        self._require_writable()
        self._require_ready()
        self.device.begin_task()
        self._set_state(ControlState.POSITIONING, "开始撬拨定位")

    def begin_clamp_preview(self) -> None:
        if not self.device.read_only:
            raise SafetyError("夹挤预览仅用于真机视觉分支")
        if self.state not in (ControlState.IDLE, ControlState.COMPLETED):
            raise SafetyError("请先保持空闲")
        if self.store.current_case is None or not self.device.connected:
            raise SafetyError("请先建立病例并连接真实设备")
        self.device.vision_mode = "clamp"
        # Clamp preview uses the same horizontal-diameter detector as pry.
        self.device.pry_vision.start()
        self._set_state(ControlState.CLAMPING, "进入夹挤视觉分支：等待启动开关")

    def start_clamping(self) -> None:
        if self.device.read_only:
            preview_ready = (
                self.state == ControlState.CLAMPING
                and getattr(self.device, "vision_mode", "idle") == "clamp"
            )
            if not preview_ready and self.state not in (ControlState.IDLE, ControlState.COMPLETED):
                raise SafetyError("请先连接设备并保持空闲")
            if self.store.current_case is None:
                raise SafetyError("请先建立病例")
            if not self.device.connected:
                raise SafetyError("真实设备数据尚未连接")
            self._set_state(ControlState.CLAMPING, "夹挤开始：正在识别足跟并计算完整路线")
            self.device.vision_mode = "clamp"
            try:
                workflow = getattr(self.device, "start_clamp_workflow_v2", self.device.start_clamp_workflow)
                workflow(self.params.target_clamp_mm, self.params.speed_mm_s)
            except Exception:
                if self.state != ControlState.EMERGENCY:
                    self._set_state(ControlState.IDLE, "夹挤没有完成，机械臂已停止")
                raise
            self._set_state(
                ControlState.COMPLETED,
                f"夹挤完成：目标位移 {self.params.target_clamp_mm:.1f} mm",
            )
            return
        self._require_writable()
        if self.state not in (ControlState.IDLE, ControlState.COMPLETED):
            raise SafetyError("当前状态不能启动夹挤复位")
        self._require_case()
        self.device.begin_task()
        self._set_state(ControlState.CLAMPING, "开始夹挤复位")

    def complete_pry_workflow(self) -> None:
        """Commit the UI-controlled real pry workflow completion state."""
        if self.state == ControlState.POSITIONING:
            self._set_state(ControlState.COMPLETED, "撬拨夹持点定位完成")

    def _preview_clamping(self) -> None:
        if self.state not in (ControlState.IDLE, ControlState.COMPLETED):
            raise SafetyError("请先连接设备并保持空闲")
        if self.store.current_case is None:
            raise SafetyError("请先建立病例")
        if not self.device.connected:
            raise SafetyError("真实设备数据尚未连接")
        result = getattr(self.device, "vision_result", {})
        if not result.get("valid"):
            raise SafetyError(result.get("message") or "尚未找到足跟和针孔")
        center = result.get("heel_center_camera_mm")
        puncture = result.get("puncture_camera_mm")
        width = result.get("heel_width_mm")
        details = (
            f"足跟中心={center or '等待深度'}，针孔={puncture or '等待深度'}，"
            f"足跟宽度={width if width is not None else '等待深度'} mm"
        )
        base_center = result.get("clamp_contact_center_base_mm")
        base_puncture = result.get("puncture_base_mm")
        if base_center:
            details += f"；基座夹取中心={base_center} mm"
        if base_puncture:
            details += f"；基座针孔={base_puncture} mm"
        self.store.event("INFO", f"夹挤路线预检查：{details}")
        self.notify(
            f"已找到夹挤位置。接触后计划再夹 {self.params.target_clamp_mm:.1f} mm；"
            "已完成坐标转换；没有向机械臂发送运动命令。"
        )

    def pause(self) -> None:
        if self.state not in (ControlState.POSITIONING, ControlState.CLAMPING, ControlState.HOLDING):
            raise SafetyError("当前没有可暂停的任务")
        if self.device.read_only: self.device.pause_motion()
        self.previous_state = self.state
        self._set_state(ControlState.PAUSED, "任务已暂停")

    def resume(self) -> None:
        if self.state != ControlState.PAUSED:
            raise SafetyError("当前任务未暂停")
        if self.device.read_only: self.device.resume_motion()
        self._set_state(self.previous_state, "任务继续运行")

    def stop(self) -> None:
        if self.state == ControlState.EMERGENCY:
            raise SafetyError("急停状态需执行急停复位")
        if self.state == ControlState.DISCONNECTED:
            return
        if self.device.read_only:
            if self.state in (ControlState.POSITIONING, ControlState.CLAMPING):
                self.device.stop_pry_vision()
            else:
                self.device.stop_motion()
        self._set_state(ControlState.IDLE, "当前任务已安全停止")

    def emergency_stop(self) -> None:
        if self.device.read_only: self.device.emergency_stop()
        else: self.device.disable()
        self.store.event("ALARM", "触发软件急停并撤销伺服使能")
        self.state = ControlState.EMERGENCY
        self.notify("急停已触发：运动命令被锁定")

    def reset_emergency(self) -> None:
        if self.state != ControlState.EMERGENCY:
            raise SafetyError("当前不在急停状态")
        if self.device.read_only: self.device.reset_emergency()
        else: self.device.reset(); self.device.enable()
        self._set_state(ControlState.IDLE, "急停复位完成，进入空闲状态")

    def home(self) -> None:
        # A clamp preview is not a running motion task. Allow the operator to
        # leave that preview and return to the independent pry zero pose.
        if (
            self.device.read_only
            and self.state in (ControlState.POSITIONING, ControlState.CLAMPING)
            and getattr(self.device, "vision_mode", "idle") in ("pry", "clamp")
        ):
            self.device.stop_pry_vision()
            self._set_state(ControlState.IDLE, "已退出视觉预览，可以执行软件回零")
        if self.state not in (ControlState.IDLE, ControlState.COMPLETED):
            raise SafetyError("当前状态不能执行软件回零")
        if self.device.read_only:
            if not self.device.connected:
                raise SafetyError("请先连接真实设备")
            self.notify("安全回零开始：先张开夹爪，再沿撬拨观察位姿安全路线返回")
            self.device.reset_clamp_workflow()
            self._set_state(ControlState.IDLE, "安全回零完成：夹爪全开，机械臂已回撬拨观察零点")
            return
        self._require_writable()
        self.device.reset()
        self._set_state(ControlState.IDLE, "软件坐标与仿真夹爪已回零")

    def jog_gripper(self, delta_mm: float) -> None:
        if self.state not in (ControlState.IDLE, ControlState.PAUSED):
            raise SafetyError("仅可在空闲或暂停状态点动夹爪")
        if self.device.read_only: self.device.jog_gripper(delta_mm)
        else: self.device.set_gripper(self.device.gripper_opening + delta_mm)
        self.store.event("INFO", f"夹爪点动至 {self.device.gripper_opening:.1f} mm")

    def tick(self, dt: float = 0.1) -> Sample:
        pose, wrench, progress, done = self.device.step(self.state, self.params, dt)
        if (
            not self.device.read_only
            and (
                wrench.force_norm > self.params.force_limit_n
                or wrench.torque_norm > self.params.torque_limit_nm
            )
        ):
            self.store.event("ALARM", f"安全阈值越限：F={wrench.force_norm:.2f}N, T={wrench.torque_norm:.2f}Nm")
            self.emergency_stop()
        if done:
            if self.state in (ControlState.POSITIONING, ControlState.CLAMPING):
                self.device.progress = 0.0
                self._set_state(ControlState.HOLDING, "目标位移到达，进入保持阶段")
            elif self.state == ControlState.HOLDING:
                self._set_state(ControlState.COMPLETED, "复位辅助流程完成")
        sample = Sample(datetime.now().isoformat(timespec="milliseconds"), self.state.value, pose, wrench, progress)
        if self.store.current_case is not None:
            self.store.sample(sample)
        return sample

    def _require_ready(self) -> None:
        if self.state not in (ControlState.IDLE, ControlState.COMPLETED):
            raise SafetyError("当前状态不能启动撬拨定位")
        self._require_case()

    def _require_case(self) -> None:
        if self.store.current_case is None:
            raise SafetyError("请先建立病例")
        if not self.device.connected or not self.device.servo_enabled:
            raise SafetyError("设备未连接或伺服未使能")

    def _require_writable(self) -> None:
        if self.device.read_only:
            raise SafetyError("真机模式目前只开放夹挤复位，其他控制命令仍被锁定")
