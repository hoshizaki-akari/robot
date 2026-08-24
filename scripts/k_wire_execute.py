#!/usr/bin/env python3
"""Execute a previously reviewed K-wire plan using platform_a safety helpers."""
import argparse,json,subprocess,sys,time
from pathlib import Path
import numpy as np
ROOT=Path(__file__).resolve().parents[1];sys.path.insert(0,str(ROOT))
from scripts.platform_a_clamp import ROBOT_IP,read_url,wrench_delta,wait_control_ready,wait_pose

STAGES=('align','pre','near','midpoint','tool_y_minus','tool_z_plus')
PREV={'align':None,'pre':'align','near':'pre','midpoint':'near','tool_y_minus':'midpoint','tool_z_plus':'tool_y_minus'}
def state_path(plan): return plan.parent/'k_wire_execution.json'
def load_state(plan):
 p=state_path(plan)
 if not p.exists(): return {'stage':'open','history':[]}
 return json.loads(p.read_text())
def save_state(plan,s): state_path(plan).write_text(json.dumps(s,indent=2),encoding='utf-8')
def open_gripper(): subprocess.run([sys.executable,str(ROOT/'scripts/set_ag95_opening.py'),'95','--speed','10','--yes'],check=True)
def move(plan_path,stage,confirmed_clear):
 if not confirmed_clear: raise RuntimeError('需要 --confirmed-clear 才允许真实 MoveL')
 plan=json.loads(plan_path.read_text()); state=load_state(plan_path); expected=PREV[stage]
 if expected is not None and state.get('stage')!=expected: raise RuntimeError(f'当前阶段 {state.get("stage")}，不能进入 {stage}')
 target=plan['stages'][stage]['flange_pose_mm_deg']; live=read_url(); fr5=live.get('fr5') or {}; baseline=np.asarray((live.get('kwr75d') or {}).get('wrench'),float)
 if not fr5.get('valid') or int(fr5.get('motion_done',0))!=1: raise RuntimeError('机械臂没有稳定停止')
 from fairino import Robot
 robot=Robot.RPC(ROBOT_IP)
 if robot is None: raise RuntimeError('无法连接 FR5')
 try:
  wait_control_ready(); code=robot.MoveL(target,0,0,vel=5.0,acc=0.0,ovl=10.0,blendR=-1.0,overSpeedStrategy=2,speedPercent=5)
  if int(code)!=0: raise RuntimeError(f'MoveL rejected: {code}')
  wait_pose(target,baseline,timeout_s=90.0); robot.Mode(1)
 finally: robot.CloseRPC()
 state['stage']=stage;state.setdefault('history',[]).append({'stage':stage,'timestamp':time.time()});save_state(plan_path,state);print(f'completed {stage}')
def main():
 ap=argparse.ArgumentParser();sub=ap.add_subparsers(dest='cmd',required=True);sub.add_parser('open');p=sub.add_parser('move');p.add_argument('stage',choices=STAGES);p.add_argument('--plan',type=Path,required=True);p.add_argument('--confirmed-clear',action='store_true');sub.add_parser('status');a=ap.parse_args()
 if a.cmd=='open': open_gripper(); return
 if a.cmd=='status': print('K-wire executor ready; plan-specific state is separate from platform_a'); return
 move(a.plan,a.stage,a.confirmed_clear)
if __name__=='__main__':main()
