#!/usr/bin/env python3
"""不连接真机，检查平台A夹挤复位按钮背后的控制逻辑。"""

from platform_a.calcaneus_robot.controller import RobotController
from platform_a.calcaneus_robot.models import ControlState
from platform_a.calcaneus_robot.ui import CalcaneusRobotApp


class FakeDevice:
    read_only = True
    connected = True
    servo_enabled = True
    progress = 0.0
    gripper_opening = 1000.0
    mode_label = "test"
    opening_unit = "raw"
    status_text = "test"
    data_age_ms = 0

    def __init__(self):
        self.reset_calls = 0
        self.clamp_calls = []
        self.control_calls = []

    def reset_clamp_workflow(self):
        self.reset_calls += 1

    def start_clamp_workflow(self, clamp_mm, speed_mm_s):
        self.clamp_calls.append((clamp_mm, speed_mm_s))

    def pause_motion(self): self.control_calls.append("pause")
    def resume_motion(self): self.control_calls.append("resume")
    def stop_motion(self): self.control_calls.append("stop")
    def emergency_stop(self): self.control_calls.append("emergency")
    def reset_emergency(self): self.control_calls.append("reset")
    def jog_gripper(self, delta_mm): self.control_calls.append(("jog", delta_mm))


class FakeStore:
    def __init__(self):
        self.events = []
        self.current_case = object()

    def event(self, level, message):
        self.events.append((level, message))


def main() -> int:
    device = FakeDevice()
    messages = []
    controller = RobotController(device, FakeStore(), messages.append)
    controller.state = ControlState.IDLE
    controller.home()
    assert device.reset_calls == 1
    assert controller.state == ControlState.IDLE
    assert any("夹挤复位完成" in message for message in messages)
    controller.start_clamping()
    assert device.clamp_calls == [(12.0, 10.0)]
    assert controller.state == ControlState.COMPLETED
    assert any("夹挤完成" in message for message in messages)
    controller.state = ControlState.CLAMPING
    controller.pause(); controller.resume(); controller.stop()
    controller.state = ControlState.IDLE
    controller.jog_gripper(1.0)
    controller.emergency_stop(); controller.reset_emergency()
    assert device.control_calls == ["pause", "resume", "stop", ("jog", 1.0), "emergency", "reset"]
    assert callable(CalcaneusRobotApp.start_home)
    assert callable(CalcaneusRobotApp.start_clamp)
    print("通过：真机软件回零会调用夹挤复位，完成后回到空闲状态。")
    print("通过：平台A保留现有软件回零按钮，没有增加界面控件。")
    print("通过：开始夹挤复位按钮会调用完整真机夹挤流程。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
