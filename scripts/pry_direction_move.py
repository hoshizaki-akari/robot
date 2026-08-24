#!/usr/bin/env python3
"""Execute one coarse tool-frame pry displacement after gripping."""
from __future__ import annotations
import argparse, json, time
from urllib.request import urlopen
from pathlib import Path
import numpy as np
from fairino import Robot
from platform_a.handeye_calibration import pose_to_matrix

ROOT = Path(__file__).resolve().parents[1]
def state():
    with urlopen("http://127.0.0.1:8765/api/state", timeout=3) as r: return json.load(r)

def compute(pose, direction, angle_deg, lever_mm):
    if direction not in {"X_PLUS", "X_MINUS", "Y_PLUS", "Y_MINUS"}: raise ValueError("invalid pry direction")
    if not 0.0 <= angle_deg < 89.0: raise ValueError("angle must be in [0,89) degrees")
    distance = float(lever_mm * np.tan(np.radians(angle_deg)))
    R = pose_to_matrix(pose)[:3, :3]
    axis = {"X_PLUS": R[:, 0], "X_MINUS": -R[:, 0], "Y_PLUS": -R[:, 2], "Y_MINUS": R[:, 2]}[direction]
    target = np.asarray(pose, dtype=float); target[:3] += axis * distance
    return target, distance

def main():
    p=argparse.ArgumentParser(); p.add_argument("--direction", required=True); p.add_argument("--angle-deg", type=float, required=True); p.add_argument("--lever-arm-mm", type=float, default=100.0); p.add_argument("--dry-run", action="store_true"); a=p.parse_args()
    s=state(); fr=s.get("fr5") or {}; pose=list(fr["flange_pose_mm_deg"]); target,d=compute(pose,a.direction,a.angle_deg,a.lever_arm_mm)
    print(json.dumps({"direction":a.direction,"angle_deg":a.angle_deg,"lever_arm_mm":a.lever_arm_mm,"translation_mm":d,"start":pose,"target":target.tolist()},ensure_ascii=False))
    if a.dry_run: return 0
    r=Robot.RPC("192.168.58.2")
    try:
        if int(r.Mode(0)) != 0 or int(r.RobotEnable(1)) != 0: raise RuntimeError("FR5 not auto-enabled")
        code=r.MoveL(target.tolist(),0,0,vel=10.0,acc=0.0,ovl=15.0,blendR=-1.0,overSpeedStrategy=2,speedPercent=15)
        if int(code)!=0: raise RuntimeError(f"MoveL rejected: {code}")
        deadline=time.monotonic()+120
        while time.monotonic()<deadline:
            x=state().get("fr5") or {}; actual=np.asarray(x.get("flange_pose_mm_deg") or [],dtype=float)
            if int(x.get("emergency_stop",0) or 0) or any(x.get("safety_stop") or []): raise RuntimeError("safety stop")
            if actual.shape==(6,) and int(x.get("motion_done",0))==1 and np.linalg.norm(actual[:3]-target[:3])<=2: return 0
            time.sleep(.15)
        raise TimeoutError("pry trajectory timeout")
    finally:
        try: r.Mode(1); r.CloseRPC()
        except Exception: pass
if __name__ == "__main__": raise SystemExit(main())
