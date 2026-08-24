#!/usr/bin/env python3
from __future__ import annotations
import argparse, json, time, subprocess
from pathlib import Path
from urllib.request import urlopen
import numpy as np
from fairino import Robot

ROOT=Path(__file__).resolve().parents[1]
def state():
    with urlopen('http://127.0.0.1:8765/api/state',timeout=3) as r:return json.load(r)
def wait(pose,limit=120):
    end=time.monotonic()+limit
    while time.monotonic()<end:
        s=state().get('fr5') or {}; actual=np.asarray(s.get('flange_pose_mm_deg') or [],float)
        if int(s.get('emergency_stop',0) or 0) or any(s.get('safety_stop') or []):raise RuntimeError('safety stop')
        if actual.shape==(6,) and int(s.get('motion_done',0))==1 and np.linalg.norm(actual[:3]-pose[:3])<=2:return
        time.sleep(.15)
    raise TimeoutError('reverse motion timeout')
def main():
    p=argparse.ArgumentParser();p.add_argument('--total-tool-y-mm',type=float,required=True);p.add_argument('--csv',type=Path,default=ROOT/'debug/pry_last_arc.csv');p.add_argument('--dry-run',action='store_true');p.add_argument('--skip-reverse',action='store_true');a=p.parse_args()
    path=np.loadtxt(a.csv,delimiter=',',skiprows=1); grasp=path[0].copy()
    from platform_a.handeye_calibration import pose_to_matrix
    release=grasp.copy();release[:3]+=pose_to_matrix(grasp.tolist())[:3,:3][:,1]*a.total_tool_y_mm
    print(json.dumps({'reverse_to_grasp':grasp.tolist(),'release_pose':release.tolist()},ensure_ascii=False))
    if a.dry_run:return 0
    r=Robot.RPC('192.168.58.2')
    try:
        if int(r.Mode(0))!=0 or int(r.RobotEnable(1))!=0:raise RuntimeError('cannot auto-enable')
        r.ResumeMotion(); r.ProgramResume()
        for pose in ([] if a.skip_reverse else path[-2::-1]):
            code=r.MoveL(pose.tolist(),0,0,vel=5.0,acc=0.0,ovl=8.0,blendR=-1.0,overSpeedStrategy=2,speedPercent=8)
            if int(code)!=0:raise RuntimeError(f'MoveL rejected {code}')
            wait(pose,30)
        code=r.MoveL(release.tolist(),0,0,vel=8.0,acc=0.0,ovl=10.0,blendR=-1.0,overSpeedStrategy=2,speedPercent=10)
        if int(code)!=0:raise RuntimeError(f'release MoveL rejected {code}')
        wait(release);print('returned to clamp center')
    finally:
        try:r.Mode(1);r.CloseRPC()
        except Exception:pass
    result=subprocess.run(['python3',str(ROOT/'scripts/set_ag95_opening.py'),'95','--speed','10','--yes'],cwd=ROOT,text=True,capture_output=True,timeout=30)
    if result.returncode!=0:raise RuntimeError(result.stderr or result.stdout)
    print('gripper released')
if __name__=='__main__':main()
