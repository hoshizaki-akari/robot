#!/usr/bin/env python3
"""Record the currently centered gripper as a safe reference snapshot."""

from __future__ import annotations

import json
import urllib.request
from datetime import datetime, timezone
from pathlib import Path


PROJECT = Path(__file__).resolve().parents[1]
OUTPUT = PROJECT / "platform_a" / "config" / "gripper_center_calibration.json"
STATE_URL = "http://127.0.0.1:8765/api/state"


def read_state() -> dict:
    with urllib.request.urlopen(STATE_URL, timeout=3) as response:
        return json.load(response)


def main() -> int:
    state = read_state()
    fr5 = state.get("fr5", {})
    ag95 = state.get("ag95", {})
    if state.get("source") != "real":
        raise RuntimeError("当前不是实时设备数据，不能记录真机中心")
    if not fr5.get("valid") or not ag95.get("valid"):
        raise RuntimeError("机械臂或夹爪数据无效")
    if not ag95.get("initialized"):
        raise RuntimeError("夹爪尚未初始化")
    if fr5.get("motion_done") != 1:
        raise RuntimeError("机械臂尚未停止，不能记录中心")

    raw = int(ag95["position_raw"])
    opening_mm = raw / 1000.0 * 95.0
    previous = {}
    if OUTPUT.is_file():
        try:
            previous = json.loads(OUTPUT.read_text(encoding="utf-8"))
        except (OSError, ValueError):
            previous = {}
    result = {
        "status": "reference_captured_only",
        "motion_allowed": False,
        "center_definition": "当前开度下，两根手指宽度方向和高度方向的几何正中间",
        "opening_raw": raw,
        "opening_mm_nominal": round(opening_mm, 3),
        "flange_pose_mm_deg": fr5.get("flange_pose_mm_deg"),
        "tcp_pose_mm_deg": fr5.get("tcp_pose_mm_deg"),
        "frame_id": "base",
        "created_at": datetime.now(timezone.utc).isoformat(),
        "note": "单个姿态只能保存中心参考；正式TCP偏移仍需已知固定点或实测法兰到尖端距离。",
    }
    for key in (
        "measured_flange_to_center_mm",
        "measured_offset_frame",
        "measured_offset_vector_mm",
        "measurement_note",
    ):
        if key in previous:
            result[key] = previous[key]
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(result, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已记录夹爪中心参考：{OUTPUT}")
    print(f"当前开度：{opening_mm:.1f} mm")
    print(f"法兰位置：{fr5.get('flange_pose_mm_deg')}")
    print("安全状态：只记录位置，没有发送任何运动命令。")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
