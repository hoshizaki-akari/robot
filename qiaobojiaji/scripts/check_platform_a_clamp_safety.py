#!/usr/bin/env python3
from __future__ import annotations

import tempfile
from pathlib import Path
from time import monotonic

from platform_a.calcaneus_robot.controller import RobotController
from platform_a.calcaneus_robot.device import DeviceError, RealDeviceAdapter
from platform_a.calcaneus_robot.models import ControlState, PatientCase
from platform_a.calcaneus_robot.storage import RecordStore
from state_service.d435_monitor import D435Monitor


def main() -> None:
    adapter = RealDeviceAdapter("http://127.0.0.1:1/api/state")
    adapter.connected = True
    adapter.vision_result = {
        "valid": True,
        "motion_allowed": False,
        "heel_center_camera_mm": [1, 2, 500],
        "puncture_camera_mm": [1, -18, 500],
        "heel_width_mm": 55,
    }
    messages: list[str] = []
    with tempfile.TemporaryDirectory() as folder:
        store = RecordStore(Path(folder))
        store.begin_case(PatientCase("TEST", "TEST", "左", "tester"))
        controller = RobotController(adapter, store, messages.append)
        controller.state = ControlState.IDLE
        controller.start_clamping()
        assert controller.state == ControlState.IDLE
        assert messages and "没有向机械臂发送运动命令" in messages[-1]
    try:
        adapter.begin_task()
    except DeviceError:
        pass
    else:
        raise AssertionError("真机写操作没有被拦截")
    monitor = D435Monitor()
    monitor._color_times.append(monotonic())
    monitor._vision_result = {"valid": True, "motion_allowed": False}
    monitor._vision_result_at = monotonic() - 3
    assert monitor.vision_snapshot()["valid"] is False
    print("通过：夹挤按钮只做位置预检查，真实机械臂和夹爪写操作仍被拦截。")
    print("通过：相机中断或结果超过2秒后，旧夹挤位置立即作废。")


if __name__ == "__main__":
    main()
