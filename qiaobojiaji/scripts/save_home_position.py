#!/usr/bin/env python3
"""保存当前机械臂停稳位置为平台 A 的软件零点；只读状态，不发送运动命令。"""

from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen


ROOT = Path(os.environ.get("FR5_PLATFORM_ROOT", Path(__file__).resolve().parents[1]))
STATE_URL = os.environ.get("FR5_STATE_URL", "http://127.0.0.1:8765/api/state")
OUTPUT = ROOT / "platform_a/config/home_position.json"


def main() -> int:
    with urlopen(STATE_URL, timeout=5.0) as response:
        snapshot = json.load(response)
    fr5 = snapshot.get("fr5") or {}
    if not fr5.get("valid") or int(fr5.get("age_ms", 999999)) > 1000:
        raise RuntimeError("机械臂实时状态不可用，不能记录零点")
    if int(fr5.get("motion_done", 0)) != 1:
        raise RuntimeError("机械臂还在运动，请停稳后再记录零点")
    errors = fr5.get("errors") or {}
    if int(errors.get("main", 0) or 0) or int(errors.get("sub", 0) or 0):
        raise RuntimeError(f"机械臂存在报警，不能记录零点：{errors}")
    pose = fr5.get("flange_pose_mm_deg") or []
    if len(pose) != 6:
        raise RuntimeError("没有读到完整的机械臂位姿")
    ag95 = snapshot.get("ag95") or {}
    data = {
        "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
        "meaning": "用户确认的默认软件零点；回零时先张开夹爪，再返回此法兰位姿",
        "flange_pose_mm_deg": [round(float(value), 6) for value in pose],
        "ag95_opening_raw": ag95.get("position_raw"),
        "source": "fr5-state-service",
    }
    OUTPUT.parent.mkdir(parents=True, exist_ok=True)
    OUTPUT.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"已保存软件零点：{OUTPUT}")
    print(json.dumps(data, ensure_ascii=False))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
