#!/usr/bin/env python3
from __future__ import annotations

import os
import sys
from pathlib import Path

import uvicorn


ROOT = Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path:
    sys.path.insert(0, str(ROOT))


def choose_source() -> str:
    configured = os.environ.get("FR5_STATE_SOURCE")
    if configured in {"replay", "real"}:
        return configured
    print(
        "\n请选择数据从哪里来：\n"
        "1. 使用电脑生成的演示数字（不用打开真机）\n"
        "2. 读取真实设备的数字（只看数据，不控制设备）\n"
        "0. 退出\n"
    )
    choice = input("请输入序号：").strip()
    if choice == "1":
        return "replay"
    if choice == "2":
        return "real"
    raise SystemExit("已退出，未启动服务。")


def main() -> None:
    source = choose_source()
    os.environ["FR5_STATE_SOURCE"] = source
    source_name = "电脑演示数字" if source == "replay" else "真实设备数字"
    print(f"\n已选择：{source_name}")
    print("这个窗口需要保持打开。")
    print("下面出现 http://127.0.0.1:8765 就表示启动成功。")
    print("检查地址：http://127.0.0.1:8765/api/state\n")
    from state_service.app import app

    uvicorn.run(
        app,
        host="127.0.0.1",
        port=8765,
        reload=False,
        log_level="info",
    )


if __name__ == "__main__":
    main()
