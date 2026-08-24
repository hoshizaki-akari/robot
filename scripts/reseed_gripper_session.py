#!/usr/bin/env python3
"""Start a fresh TCP calibration using the user's three accepted poses."""
from __future__ import annotations

import json
import shutil
from datetime import datetime
from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
SESSION = ROOT / "platform_a/calibration_data/gripper_tcp_session.json"
BACKUP = ROOT / "platform_a/calibration_data/gripper_tcp_session_before_reseed.json"

source_session = BACKUP if BACKUP.is_file() else SESSION
old = json.loads(source_session.read_text(encoding="utf-8"))
accepted = list(old.get("validation_samples") or [])
if len(accepted) != 3:
    raise RuntimeError(f"需要恰好3个用户确认的姿势，当前为{len(accepted)}个")
if not BACKUP.is_file():
    shutil.copy2(SESSION, BACKUP)
first_pose = accepted[0]["flange_pose_mm_deg"]
initial_offset = [0.0, 0.0, 220.0]
old_result = json.loads((ROOT / "platform_a/config/gripper_tcp_calibration.json").read_text(encoding="utf-8"))
cone_point = old_result.get("fixed_cone_tip_base_mm")
if not cone_point or len(cone_point) != 3:
    raise RuntimeError("旧标定结果没有固定尖锥坐标")
new_session = {
    "status": "collecting_calibration",
    "created_at": datetime.now().astimezone().isoformat(timespec="seconds"),
    "reseeded_from": str(BACKUP),
    "opening_raw_target": 505,
    "opening_mm_nominal": 47.975,
    "initial_offset_guess_mm": initial_offset,
    "assumed_cone_tip_base_mm": [float(value) for value in cone_point],
    "initial_rpy_deg": list(first_pose[3:]),
    "calibration_samples": accepted,
    "validation_samples": [],
    "reseed_note": "用户确认的三个原验证姿势作为新原始六姿势中的前三个",
}
SESSION.write_text(json.dumps(new_session, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
print(f"已重建标定会话，原会话备份：{BACKUP}")
print("当前原始标定姿势：3/6；等待新增3个原始姿势")
