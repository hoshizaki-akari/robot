#!/usr/bin/env python3
from __future__ import annotations

import json
import os
import subprocess
import sys
import time
from pathlib import Path
from urllib.request import Request, urlopen


ROOT = Path(__file__).resolve().parents[1]
PORT = 18081


def request_json(path: str, body: dict | None = None) -> dict:
    encoded = json.dumps(body).encode() if body is not None else None
    request = Request(
        f"http://127.0.0.1:{PORT}{path}",
        data=encoded,
        headers={"Content-Type": "application/json"},
        method="POST" if body is not None or "stop" in path else "GET",
    )
    with urlopen(request, timeout=2) as response:
        return json.load(response)


def wait_ready() -> None:
    deadline = time.monotonic() + 6
    while time.monotonic() < deadline:
        try:
            if request_json("/health").get("ok"):
                return
        except OSError:
            pass
        time.sleep(0.2)
    raise RuntimeError("测试网关没有按时启动")


def main() -> int:
    environment = os.environ.copy()
    environment["FR5_STATE_URL"] = "http://127.0.0.1:8765/api/state"
    process = subprocess.Popen(
        [
            sys.executable,
            "-m",
            "uvicorn",
            "platform_b.gateway:app",
            "--host",
            "127.0.0.1",
            "--port",
            str(PORT),
            "--log-level",
            "warning",
        ],
        cwd=ROOT,
        env=environment,
    )
    try:
        wait_ready()
        time.sleep(2)
        started = request_json("/api/traction/start", {"target_force_n": 10})
        assert started["active"]
        assert started["target_force_n"] == 10
        assert not started["direction_locked"]
        assert started["message"] == "请轻拉以确定牵引方向"
        assert not started["send_robot_motion"]

        state = request_json("/api/state")
        assert state["traction"]["active"]
        stopped = request_json("/api/traction/stop", {})
        assert not stopped["active"]
        print("PASS：工作站后台可开始、等待方向、读取状态并停止恒力牵引")
        print("PASS：测试全过程没有向真实机械臂发送运动")
        return 0
    finally:
        process.terminate()
        try:
            process.wait(timeout=3)
        except subprocess.TimeoutExpired:
            process.kill()
            process.wait(timeout=2)


if __name__ == "__main__":
    raise SystemExit(main())
