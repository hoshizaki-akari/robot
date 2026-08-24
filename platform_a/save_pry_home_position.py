#!/usr/bin/env python3
"""Save the independent pry-buckle zero pose from the read-only state service."""
from __future__ import annotations

import json
import os
from datetime import datetime
from pathlib import Path
from urllib.request import urlopen

ROOT = Path(os.environ.get("FR5_PLATFORM_ROOT", Path(__file__).resolve().parents[1]))
OUTPUT = ROOT / "platform_a/config/pry_home_position.json"
URL = os.environ.get("FR5_STATE_URL", "http://127.0.0.1:8765/api/state")

with urlopen(URL, timeout=5.0) as response:
    snapshot = json.load(response)
fr5 = snapshot.get("fr5") or {}
if not fr5.get("valid") or int(fr5.get("age_ms", 999999)) > 1000 or int(fr5.get("motion_done", 0)) != 1:
    raise RuntimeError("机械臂实时状态不可用或尚未停稳，不能记录撬拨零点")
pose = fr5.get("flange_pose_mm_deg") or []
if len(pose) != 6:
    raise RuntimeError("没有读到完整的机械臂位姿")
data = {
    "saved_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    "meaning": "撬拨分支独立软件零点；不与夹挤零点共用",
    "flange_pose_mm_deg": [round(float(value), 6) for value in pose],
    "source": "fr5-state-service",
}
OUTPUT.parent.mkdir(parents=True, exist_ok=True)
OUTPUT.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"已保存撬拨零点：{OUTPUT}")
