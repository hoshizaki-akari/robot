#!/usr/bin/env python3
"""K-wire task flow; does not modify or call platform_a clamp state files."""
from __future__ import annotations
import argparse,json
from pathlib import Path
import sys
ROOT=Path(__file__).resolve().parents[1]; sys.path.insert(0,str(ROOT))
from platform_a.k_wire_motion import KWireMotionConfig,build_plan,unit_test

def main():
    ap=argparse.ArgumentParser(); sub=ap.add_subparsers(dest='cmd',required=True)
    sub.add_parser('unit-test')
    p=sub.add_parser('plan');p.add_argument('--vision-json',type=Path,required=True);p.add_argument('--observation-pose',type=Path,required=True);p.add_argument('--tcp-json',type=Path,required=True);p.add_argument('--out',type=Path,required=True);p.add_argument('--a-mm',type=float,default=40);p.add_argument('--midpoint-to-grip-mm',type=float,default=40);p.add_argument('--z-plus-mm',type=float,default=10)
    a=ap.parse_args()
    if a.cmd=='unit-test': unit_test(); return
    v=json.loads(a.vision_json.read_text()); obs=json.loads(a.observation_pose.read_text()) if a.observation_pose.suffix=='.json' else json.loads(a.observation_pose.read_text()); tcp=json.loads(a.tcp_json.read_text()); pose=obs.get('flange_pose_mm_deg') or obs.get('tcp_pose_mm_deg') or obs
    plan=build_plan(v,pose,tcp['flange_to_gripper_center_mm'],KWireMotionConfig(needle_offset_mm=a.a_mm,midpoint_to_grip_mm=a.midpoint_to_grip_mm,z_plus_mm=a.z_plus_mm)); a.out.mkdir(parents=True,exist_ok=True); (a.out/'k_wire_motion_plan.json').write_text(json.dumps(plan,indent=2),encoding='utf-8'); print(json.dumps(plan,indent=2))
if __name__=='__main__':main()
