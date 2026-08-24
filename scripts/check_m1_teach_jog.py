#!/usr/bin/env python3
"""Offline verification for M1 pose serialization and local-frame jog math."""
from __future__ import annotations
import sys
import tempfile
from pathlib import Path
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from platform_a.handeye_calibration import pose_to_matrix  # noqa: E402
import platform_a.teach_jog as teach_jog  # noqa: E402
from platform_a.teach_jog import local_target_pose, rpy_deg_to_quaternion  # noqa: E402

current = [100.0, 200.0, 300.0, 20.0, -15.0, 35.0]
target = local_target_pose(current, 0.05, 0.0, 0.0)
rotation = pose_to_matrix([0.0, 0.0, 0.0, *current[3:]])[:3, :3]
assert np.allclose(np.asarray(target[:3]) - np.asarray(current[:3]), rotation[:, 0] * 50.0, atol=1e-9)
assert target[3:] == current[3:]
assert abs(float(np.linalg.norm(rpy_deg_to_quaternion(current[3:]))) - 1.0) < 1e-12
snapshot = {"fr5": {"valid": True, "connected": True, "errors": {"main": 0, "sub": 0}, "emergency_stop": 0, "safety_stop": [0, 0], "motion_done": 1, "joint_position_deg": [1, 2, 3, 4, 5, 6], "tcp_pose_mm_deg": current, "frame_id": "base"}}
original_reader = teach_jog.read_state
try:
    teach_jog.read_state = lambda: snapshot
    with tempfile.TemporaryDirectory() as folder:
        saved = teach_jog.teach_pose("wire_a", Path(folder) / "taught_poses.yaml")
        assert saved["joints_deg"] == [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        assert teach_jog.load_taught_pose("wire_a", Path(folder) / "taught_poses.yaml")["tcp_pose"]["position_m"]["x"] == 0.1
finally:
    teach_jog.read_state = original_reader
print("M1 math PASS: local X+ 50 mm follows current TCP X axis; orientation unchanged.")
