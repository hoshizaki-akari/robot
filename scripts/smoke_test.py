#!/usr/bin/env python3
from __future__ import annotations

import asyncio
import json
import os
import subprocess
import sys
import time
from pathlib import Path
from urllib.error import URLError
from urllib.request import urlopen

import websockets


ROOT = Path(__file__).resolve().parents[1]
PYTHON = sys.executable


def wait_json(url: str, timeout: float = 8.0) -> dict:
    deadline = time.monotonic() + timeout
    last_error: Exception | None = None
    while time.monotonic() < deadline:
        try:
            with urlopen(url, timeout=0.8) as response:
                return json.load(response)
        except (OSError, ValueError, URLError) as error:
            last_error = error
            time.sleep(0.15)
    raise RuntimeError(f"等待 {url} 超时：{last_error}")


async def read_websocket(url: str) -> dict:
    async with websockets.connect(url, open_timeout=2) as websocket:
        return json.loads(await asyncio.wait_for(websocket.recv(), timeout=2))


def start_process(command: list[str], env: dict[str, str]) -> subprocess.Popen:
    return subprocess.Popen(
        command,
        cwd=ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
    )


def stop_process(process: subprocess.Popen) -> None:
    if process.poll() is not None:
        return
    process.terminate()
    try:
        process.wait(timeout=3)
    except subprocess.TimeoutExpired:
        process.kill()
        process.wait(timeout=2)


def assert_schema(state: dict) -> None:
    assert state["schema_version"] == "1.0"
    assert state["sequence"] > 0
    for name in ("fr5", "kwr75d", "ag95", "d435"):
        assert name in state
        assert isinstance(state[name]["valid"], bool)
        assert isinstance(state[name]["age_ms"], int)
        assert state[name]["timestamp"]
    assert len(state["fr5"]["joint_position_deg"]) == 6
    assert len(state["fr5"]["tcp_pose_mm_deg"]) == 6
    assert len(state["kwr75d"]["wrench"]) == 6
    assert 0 <= state["ag95"]["position_raw"] <= 1000


def main() -> int:
    env = os.environ.copy()
    env["FR5_STATE_SOURCE"] = "replay"
    env["PYTHONPATH"] = str(ROOT)
    state_process = start_process(
        [PYTHON, "-m", "uvicorn", "state_service.app:app", "--host", "127.0.0.1", "--port", "8765"],
        env,
    )
    gateway_process: subprocess.Popen | None = None
    adapter = None
    try:
        state = wait_json("http://127.0.0.1:8765/api/state")
        assert_schema(state)
        print("PASS 1/8：统一状态服务和数据结构")

        first_sequence = state["sequence"]
        time.sleep(0.35)
        next_state = wait_json("http://127.0.0.1:8765/api/state")
        assert next_state["sequence"] > first_sequence
        print("PASS 2/8：回放数据持续更新")

        sys.path.insert(0, str(ROOT / "platform_a"))
        from calcaneus_robot.device import RealDeviceAdapter

        adapter = RealDeviceAdapter()
        adapter.connect()
        time.sleep(0.3)
        pose, wrench, progress, done = adapter.step(
            None, None, 0.1  # type: ignore[arg-type]
        )
        assert adapter.connected
        assert all(math_value == math_value for math_value in (pose.x, wrench.fz))
        assert progress == 0 and done is False
        print("PASS 3/8：平台1后台适配器读取且不阻塞界面线程")

        gateway_process = start_process([PYTHON, str(ROOT / "platform_b" / "gateway.py")], env)
        gateway = wait_json("http://127.0.0.1:8080/api/state")
        assert gateway["gateway"]["valid"]
        assert_schema(gateway)
        print("PASS 4/8：平台2 REST 网关")

        websocket_state = asyncio.run(read_websocket("ws://127.0.0.1:8080/ws"))
        assert websocket_state["gateway"]["valid"]
        assert_schema(websocket_state)
        assert adapter.connected
        print("PASS 5/8：两个平台同时读取，平台2 WebSocket 正常")

        html = (ROOT / "platform_b" / "090105.html").read_text(encoding="utf-8")
        assert "Math.random" not in html
        assert "connectStateStream()" in html
        with urlopen("http://127.0.0.1:8080", timeout=2) as response:
            served_html = response.read().decode("utf-8")
        assert 'id="tareBtn"' in served_html
        assert "将当前力设为零点" in served_html
        assert "三方向合力" in served_html
        assert "真机运动发送：关闭" in served_html
        assert "Math.random" not in served_html
        print("PASS 6/8：网页使用真实力向量，置零和力控计算已就绪")

        stop_process(state_process)
        time.sleep(2.0)
        disconnected = wait_json("http://127.0.0.1:8080/api/state")
        assert not disconnected["gateway"]["valid"]
        assert not adapter.connected
        print("PASS 7/8：上游停止后两个平台明确显示断线")

        state_process = start_process(
            [PYTHON, "-m", "uvicorn", "state_service.app:app", "--host", "127.0.0.1", "--port", "8765"],
            env,
        )
        wait_json("http://127.0.0.1:8765/api/state")
        deadline = time.monotonic() + 4
        while time.monotonic() < deadline:
            recovered = wait_json("http://127.0.0.1:8080/api/state")
            if recovered["gateway"]["valid"] and adapter.connected:
                break
            time.sleep(0.2)
        assert recovered["gateway"]["valid"] and adapter.connected
        print("PASS 8/8：上游恢复后两个平台自动重连")
        print("\n全部冒烟测试通过。")
        return 0
    finally:
        if adapter is not None:
            adapter.disconnect()
        if gateway_process is not None:
            stop_process(gateway_process)
        stop_process(state_process)


if __name__ == "__main__":
    raise SystemExit(main())
