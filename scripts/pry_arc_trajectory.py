#!/usr/bin/env python3
"""Plan a pivot-centred pry arc in the current Tool coordinate system."""
from __future__ import annotations
import argparse, csv, math
from pathlib import Path
import numpy as np
from platform_a.handeye_calibration import pose_to_matrix
from platform_a.tool_center_calibration import matrix_to_rpy_degrees

def axis_angle(axis: np.ndarray, angle: float) -> np.ndarray:
    x, y, z = axis / np.linalg.norm(axis)
    skew = np.array([[0.0, -z, y], [z, 0.0, -x], [-y, x, 0.0]])
    return np.eye(3) + math.sin(angle) * skew + (1.0 - math.cos(angle)) * (skew @ skew)

def plan(start_pose: list[float], direction: str, angle_deg: float, lever_mm: float = 100.0, step_deg: float = 1.0):
    if direction not in {"X_PLUS","X_MINUS","Y_PLUS","Y_MINUS"}: raise ValueError("invalid direction")
    if not 0 < angle_deg <= 60: raise ValueError("angle must be in (0,60]")
    T=pose_to_matrix(start_pose); R0=T[:3,:3]; g0=np.asarray(start_pose[:3],float)
    tool_y=R0[:,1]; r0=-lever_mm*tool_y; pivot=g0-r0
    d={"X_PLUS":R0[:,0],"X_MINUS":-R0[:,0],"Y_PLUS":-R0[:,2],"Y_MINUS":R0[:,2]}[direction]
    u=d-np.dot(d,r0/lever_mm)*(r0/lever_mm); u/=np.linalg.norm(u)
    rot_axis=np.cross(r0/lever_mm,u); rot_axis/=np.linalg.norm(rot_axis)
    count=max(1,int(math.ceil(angle_deg/step_deg))); result=[]
    for phi_deg in np.linspace(0,angle_deg,count+1):
        phi=math.radians(float(phi_deg)); Q=axis_angle(rot_axis,phi); p=pivot+Q@r0; R=Q@R0
        result.append([*p.tolist(),*matrix_to_rpy_degrees(R)])
    return np.asarray(pivot),np.asarray(result),R0

def main():
    p=argparse.ArgumentParser(); p.add_argument('--pose',required=True); p.add_argument('--direction',required=True); p.add_argument('--angle-deg',type=float,required=True); p.add_argument('--lever-mm',type=float,default=100); p.add_argument('--out',type=Path,required=True); a=p.parse_args()
    start=[float(x) for x in a.pose.split(',')]; pivot,path,R0=plan(start,a.direction,a.angle_deg,a.lever_mm)
    radius=np.linalg.norm(path[:,:3]-pivot,axis=1); ortho=max(np.linalg.norm(pose_to_matrix(row.tolist())[:3,:3].T@pose_to_matrix(row.tolist())[:3,:3]-np.eye(3)) for row in path)
    a.out.parent.mkdir(parents=True,exist_ok=True)
    with a.out.open('w',newline='') as f:
        w=csv.writer(f); w.writerow(['x_mm','y_mm','z_mm','rx_deg','ry_deg','rz_deg']); w.writerows(path)
    print({'pivot_base_mm':pivot.tolist(),'waypoints':len(path),'max_radius_error_mm':float(np.max(abs(radius-a.lever_mm))),'rotation_orthogonality':float(ortho),'final_pose':path[-1].tolist()})
if __name__=='__main__': main()
