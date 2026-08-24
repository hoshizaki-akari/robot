#!/usr/bin/env python3
"""在 D435 有设备但长时间没有图像时自动重启相机服务。"""

from __future__ import annotations

import json
import subprocess
import time
from urllib.error import URLError
from urllib.request import urlopen


STATE_URL = "http://127.0.0.1:8765/api/state"
CHECK_INTERVAL_SECONDS = 5
FAILURES_BEFORE_RESTART = 4
RESTART_SETTLE_SECONDS = 15
# USB 2.0（480M）及以下带宽不足以传输 D435 深度流，重启 ROS 程序无法恢复，
# 此时应停止无意义重启，改为告警并拉长间隔，等待用户更换 USB 3.0 端口。
SLOW_USB_SPEEDS = {"480", "12", "1.5"}
WARN_INTERVAL_SECONDS = 60


def read_camera_state() -> dict | None:
    try:
        with urlopen(STATE_URL, timeout=2) as response:
            return (json.load(response).get("d435") or {})
    except (OSError, URLError, ValueError):
        # 状态服务本身不可用时，不误判为摄像头故障。
        return None


def main() -> None:
    failures = 0
    last_warn_at = 0.0
    while True:
        camera = read_camera_state()
        now = time.time()
        if camera is None:
            failures = 0
        elif camera.get("color_valid") and camera.get("depth_valid"):
            failures = 0
        elif camera.get("connected"):
            usb_speed = str(camera.get("usb_speed_mbps") or "")
            if usb_speed in SLOW_USB_SPEEDS:
                # 物理带宽不足：重启 ROS 服务无效，只告警一次/分钟，避免刷日志。
                if now - last_warn_at >= WARN_INTERVAL_SECONDS:
                    print(
                        f"[watchdog] D435 连接在 USB {usb_speed}Mbps 端口，带宽不足，"
                        "重启相机无效。请将相机换到 USB 3.0（SuperSpeed）端口。",
                        flush=True,
                    )
                    last_warn_at = now
                failures = 0
                time.sleep(CHECK_INTERVAL_SECONDS)
                continue
            failures += 1
            if failures >= FAILURES_BEFORE_RESTART:
                print(
                    "[watchdog] D435 已连接但长时间无有效画面，尝试重启相机服务。",
                    flush=True,
                )
                subprocess.run(
                    ["systemctl", "--user", "restart", "d435-camera.service"],
                    check=False,
                )
                failures = 0
                time.sleep(RESTART_SETTLE_SECONDS)
        else:
            # USB设备不在WSL中，单纯重启ROS程序无法恢复。
            failures = 0
        time.sleep(CHECK_INTERVAL_SECONDS)


if __name__ == "__main__":
    main()
