#!/usr/bin/env python3
"""Execute a closed-gripper pivot arc planned in the current Tool frame."""
from __future__ import annotations
import argparse, json, sys, time
from pathlib import Path
from urllib.request import urlopen
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
if str(ROOT) not in sys.path: sys.path.insert(0,str(ROOT))
if str(Path(__file__).resolve().parent) not in sys.path: sys.path.insert(0, str(Path(__file__).resolve().parent))
from pry_arc_trajectory import plan

def state():
    with urlopen('http://127.0.0.1:8765/api/state',timeout=3) as r: return json.load(r)

def main():
    p=argparse.ArgumentParser(); p.add_argument('--direction',required=True); p.add_argument('--angle-deg',type=float,required=True); p.add_argument('--lever-mm',type=float,default=100.0); p.add_argument('--dry-run',action='store_true'); a=p.parse_args()
    s=state(); fr=s.get('fr5') or {}; ag=s.get('ag95') or {}
    if not fr.get('valid') or int(fr.get('motion_done',0))!=1: raise RuntimeError('FR5 not stopped')
    if not ag.get('valid') or int(ag.get('position_raw',1000))>100: raise RuntimeError('gripper must be closed before an arc')
    pose=[float(x) for x in fr['flange_pose_mm_deg']]; pivot,path,_=plan(pose,a.direction,a.angle_deg,a.lever_mm,1.0)
    np.savetxt(ROOT / 'debug/pry_last_arc.csv', path, delimiter=',', header='x_mm,y_mm,z_mm,rx_deg,ry_deg,rz_deg', comments='')
    print(json.dumps({'pivot_base_mm':pivot.tolist(),'waypoint_count':len(path),'direction':a.direction,'angle_deg':a.angle_deg,'final_pose':path[-1].tolist()},ensure_ascii=False))
    if a.dry_run: return 0
    from fairino import Robot
    robot=Robot.RPC('192.168.58.2')
    try:
        if int(robot.Mode(0))!=0 or int(robot.RobotEnable(1))!=0: raise RuntimeError('cannot auto-enable FR5')
        robot.ResumeMotion(); robot.ProgramResume()
        for i,pose_i in enumerate(path[1:],1):
            code=robot.MoveL(pose_i.tolist(),0,0,vel=5.0,acc=0.0,ovl=8.0,blendR=-1.0,overSpeedStrategy=2,speedPercent=8)
            if int(code)!=0: raise RuntimeError(f'waypoint {i} rejected: {code}')
            deadline=time.monotonic()+30
            while time.monotonic()<deadline:
                now=state().get('fr5') or {}; actual=np.asarray(now.get('flange_pose_mm_deg') or [],float)
                if int(now.get('emergency_stop',0) or 0) or any(now.get('safety_stop') or []): raise RuntimeError('safety stop')
                if actual.shape==(6,) and int(now.get('motion_done',0))==1 and np.linalg.norm(actual[:3]-pose_i[:3])<=2: break
                time.sleep(.12)
            else: raise TimeoutError(f'waypoint {i} timeout')
        print('arc complete')
    except Exception:
        try: robot.StopMotion(); robot.Mode(1); robot.RobotEnable(0)
        except Exception: pass
        raise
    finally:
        try: robot.Mode(1); robot.CloseRPC()
        except Exception: pass
if __name__=='__main__': raise SystemExit(main())
