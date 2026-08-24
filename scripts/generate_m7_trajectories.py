#!/usr/bin/env python3
"""M7 pure geometry: four Pivot-centred wire rotation arcs."""
import argparse,csv,json,math
from pathlib import Path
import numpy as np

DIRS=('X_PLUS','X_MINUS','Y_PLUS','Y_MINUS')
def unit(x):
    n=np.linalg.norm(x)
    if n<1e-12: raise ValueError('degenerate vector')
    return x/n

def semantic_rotation(v, sole):
    z=unit(sole); y0=np.cross(z,v)
    if np.linalg.norm(y0)<1e-10:
        ref=np.array([1.,0.,0.]) if abs(z[0])<.9 else np.array([0.,1.,0.])
        y0=np.cross(z,ref)
    y=unit(y0); x=unit(np.cross(y,z)); return np.column_stack((x,y,z))

def make_arc(O,v0,X,Y,S,L,deg,step):
    O= np.asarray(O,float); v0=unit(np.asarray(v0,float)); X=unit(np.asarray(X,float)); Y=unit(np.asarray(Y,float)); S=unit(np.asarray(S,float))
    ds={'X_PLUS':X,'X_MINUS':-X,'Y_PLUS':Y,'Y_MINUS':-Y}; out=[]; n=max(1,int(math.ceil(abs(deg)/step)))
    for direction in DIRS:
        d=ds[direction]; u0=d-(d@v0)*v0; u=unit(u0); rows=[]
        for i in range(n+1):
            phi=math.radians(deg)*i/n; v=np.cos(phi)*v0+np.sin(phi)*u; v=unit(v); G=O+L*v; R=semantic_rotation(v,S)
            rows.append({'direction':direction,'index':i,'angle_deg':math.degrees(phi),'x_m':G[0],'y_m':G[1],'z_m':G[2],'wire_vx':v[0],'wire_vy':v[1],'wire_vz':v[2],'R00':R[0,0],'R01':R[0,1],'R02':R[0,2],'R10':R[1,0],'R11':R[1,1],'R12':R[1,2],'R20':R[2,0],'R21':R[2,1],'R22':R[2,2]})
        out.append(rows)
    return out

def tests(arcs,O,v0,S,L,deg):
    max_radius=0.; max_angle=0.; max_ortho=0.; max_close=0.; max_z=0.
    for rows in arcs:
        for r in rows:
            G=np.array([r['x_m'],r['y_m'],r['z_m']]); v=np.array([r['wire_vx'],r['wire_vy'],r['wire_vz']]); R=np.array([[r['R00'],r['R01'],r['R02']],[r['R10'],r['R11'],r['R12']],[r['R20'],r['R21'],r['R22']]])
            max_radius=max(max_radius,abs(np.linalg.norm(G-O)-L)); max_ortho=max(max_ortho,float(np.max(abs(R.T@R-np.eye(3))))); max_close=max(max_close,abs(R[:,1]@v)); max_z=max(max_z,1-abs(R[:,2]@S))
        vf=np.array([rows[-1]['wire_vx'],rows[-1]['wire_vy'],rows[-1]['wire_vz']]); max_angle=max(max_angle,abs(math.degrees(math.acos(np.clip(v0@vf,-1,1)))-abs(deg)))
    return dict(max_radius_error_m=max_radius,final_angle_error_deg=max_angle,max_rotation_orthogonality_error=max_ortho,max_closing_axis_wire_dot=max_close,max_tcp_z_sole_normal_error=max_z)

def unit_tests():
    O=np.zeros(3); v=np.array([0.,0.,1.]); X=np.array([1.,0.,0.]);Y=np.array([0.,1.,0.]);S=np.array([0.,0.,1.]); arcs=make_arc(O,v,X,Y,S,.04,45,1); a=arcs[0][-1]; b=arcs[2][-1]; assert np.allclose([a['x_m'],a['y_m'],a['z_m']],[.028284271,0,.028284271],atol=2e-6); assert np.allclose([b['x_m'],b['y_m'],b['z_m']],[0,.028284271,.028284271],atol=2e-6); t=tests(arcs,O,v,S,.04,45); assert t['max_radius_error_m']<1e-12 and t['final_angle_error_deg']<1e-10 and t['max_rotation_orthogonality_error']<1e-12 and t['max_closing_axis_wire_dot']<1e-12 and t['max_tcp_z_sole_normal_error']<1e-12; print('M7 unit tests PASS',t)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--m6',type=Path); ap.add_argument('--m5',type=Path); ap.add_argument('--out',type=Path); ap.add_argument('--lever-arm-mm',type=float,default=40); ap.add_argument('--angle-deg',type=float,default=45); ap.add_argument('--angular-step-deg',type=float,default=1); ap.add_argument('--unit-test',action='store_true'); a=ap.parse_args()
    if a.unit_test: unit_tests(); return
    d=json.loads(a.m6.read_text()); m5p=a.m5 or (a.m6.parent.parent/'m5_debug/m5_pivot.json'); m5=json.loads(m5p.read_text()); O=np.array(d['pivot_base_xyz_m']); v=unit(np.array(m5['wire_direction_v0'])); X=np.array(d['heel_x_plus']);Y=np.array(d['heel_y_plus']);S=np.array(d['sole_normal_base']);L=a.lever_arm_mm/1000.; arcs=make_arc(O,v,X,Y,S,L,a.angle_deg,a.angular_step_deg); t=tests(arcs,O,v,S,L,a.angle_deg); a.out.mkdir(parents=True,exist_ok=True)
    with (a.out/'trajectory.csv').open('w',newline='') as f:
        fields=list(arcs[0][0].keys());w=csv.DictWriter(f,fieldnames=fields);w.writeheader();[w.writerow(r) for rows in arcs for r in rows]
    summary={'lever_arm_m':L,'angle_deg':a.angle_deg,'angular_step_deg':a.angular_step_deg,'pivot_base_xyz_m':O.tolist(),'wire_direction_v0':v.tolist(),'sole_normal':S.tolist(),'directions':{rows[0]['direction']:{'waypoint_count':len(rows),'start_point_m':[rows[0][k] for k in ('x_m','y_m','z_m')],'final_point_m':[rows[-1][k] for k in ('x_m','y_m','z_m')] } for rows in arcs},'checks':t,'status':'WAITING_CP5'}
    (a.out/'m7_trajectory_summary.json').write_text(json.dumps(summary,indent=2),encoding='utf-8')
    report='===== M7 TRAJECTORIES =====\n\n'+''.join(f'{k}:\n  waypoint count = {v["waypoint_count"]}\n  max radius error = {t["max_radius_error_m"]:.12g} m\n  final angle = {a.angle_deg:.6f} deg\n  start point = {v["start_point_m"]}\n  final point = {v["final_point_m"]}\n\n' for k,v in summary['directions'].items())+f'checks = {t}\nstatus = WAITING_CP5\n'
    (a.out/'stage_report.txt').write_text(report,encoding='utf-8');print(report)
if __name__=='__main__':main()
