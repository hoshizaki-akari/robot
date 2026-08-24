#!/usr/bin/env python3
"""M8 local-tool correction planner; planning only, no robot motion by default."""
import argparse,json,math
from pathlib import Path
import numpy as np

def unit(x):
    n=np.linalg.norm(x)
    if n<1e-12: raise ValueError('degenerate axis')
    return x/n

def semantic_frame(wire_v, sole_n):
    z=unit(sole_n); y=unit(np.cross(z,unit(wire_v))); x=unit(np.cross(y,z)); return np.column_stack((x,y,z))

def plan(midpoint_m, wire_v, sole_n, a_mm=40., y_base_mm=40., z_plus_mm=10.):
    """Return midpoint -> tool Y- correction -> tool Z+ correction.

    `midpoint_m` is the visual object midpoint. The final point is obtained
    only through the current tool frame, so no Base-X assumption is made.
    """
    p=np.asarray(midpoint_m,float); R=semantic_frame(wire_v,sole_n); y=R[:,1]; z=R[:,2]
    p_y=p-y*((y_base_mm+a_mm)/1000.); p_f=p_y+z*(z_plus_mm/1000.)
    return {'midpoint_base_m':p.tolist(),'tool_rotation_base':R.tolist(),'y_minus_distance_mm':y_base_mm+a_mm,'z_plus_distance_mm':z_plus_mm,'waypoints_base_m':[p.tolist(),p_y.tolist(),p_f.tolist()],'final_base_m':p_f.tolist()}

def tests():
    d=plan([0,0,0],[1,0,0],[0,0,1],40,40,10); assert np.allclose(d['tool_rotation_base'],np.eye(3)); assert np.allclose(d['final_base_m'],[0,-.08,.01]); R=np.array(d['tool_rotation_base']); assert np.max(abs(R.T@R-np.eye(3)))<1e-12; assert abs(R[:,1]@np.array([1,0,0]))<1e-12; print('M8 tool-local tests PASS')

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--m6',type=Path); ap.add_argument('--m5',type=Path); ap.add_argument('--midpoint',required=False); ap.add_argument('--a-mm',type=float,default=40.); ap.add_argument('--y-base-mm',type=float,default=40.); ap.add_argument('--z-plus-mm',type=float,default=10.); ap.add_argument('--out',type=Path); ap.add_argument('--unit-test',action='store_true'); a=ap.parse_args()
    if a.unit_test: tests(); return
    d=json.loads(a.m6.read_text()); m5=json.loads(a.m5.read_text()); mid=np.asarray(json.loads(a.midpoint) if a.midpoint else d['sole_centroid_base_m'],float); result=plan(mid,m5['wire_direction_v0'],d['sole_normal_base'],a.a_mm,a.y_base_mm,a.z_plus_mm); result.update({'source':'M6 sole centroid unless --midpoint supplied','status':'PLAN_ONLY_NO_ROBOT_MOTION'})
    a.out.mkdir(parents=True,exist_ok=True); (a.out/'m8_tool_local_plan.json').write_text(json.dumps(result,indent=2),encoding='utf-8'); print(json.dumps(result,indent=2))
if __name__=='__main__':main()
