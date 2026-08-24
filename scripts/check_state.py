#!/usr/bin/env python3
from __future__ import annotations

import json
from urllib.request import urlopen


def main() -> None:
    with urlopen("http://127.0.0.1:8765/api/state", timeout=2) as response:
        state = json.load(response)
    print(f"数据源：{state['source']}  序号：{state['sequence']}")
    for name in ("fr5", "kwr75d", "ag95", "d435"):
        item = state[name]
        print(
            f"{name:8s} valid={item['valid']!s:5s} "
            f"age_ms={item['age_ms']:4d} connected={item.get('connected')} "
            f"{item.get('message', '')}"
        )
    print("FR5 TCP：", state["fr5"]["tcp_pose_mm_deg"])
    print("KWR75D：", state["kwr75d"]["wrench"])
    print("AG95 原始位置：", state["ag95"]["position_raw"])


if __name__ == "__main__":
    main()

