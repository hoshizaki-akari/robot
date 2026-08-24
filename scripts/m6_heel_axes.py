#!/usr/bin/env python3
"""M6 sole plane and Heel local axes; includes safe right-hint recording."""
import argparse, json, math, sys
from datetime import datetime, timezone
from pathlib import Path
import cv2
import numpy as np

ROOT=Path(__file__).resolve().parents[1]
RIGHT_FILE=ROOT/'data/session_m2_20260820/right_hint.json'

def unit(x):
    n=np.linalg.norm(x)
    if n<1e-12: raise ValueError('degenerate vector')
    return x/n

def ransac_plane(P, threshold=.008, iterations=3000, seed=19):
    rng=np.random.default_rng(seed); best=np.zeros(len(P),bool)
    for _ in range(iterations):
        a,b,c=P[rng.choice(len(P),3,replace=False)]; n=np.cross(b-a,c-a); nn=np.linalg.norm(n)
        if nn<1e-9: continue
        n/=nn; d=-n@a; m=np.abs(P@n+d)<threshold
        if m.sum()>best.sum(): best=m
    if best.sum()<100: raise RuntimeError('insufficient sole plane inliers')
    Q=P[best]; C=Q.mean(0); _,_,vh=np.linalg.svd(Q-C,full_matrices=False); n=unit(vh[-1]); d=-n@C
    return C,n,d,best,np.abs(Q@n+d)

def state_snapshot():
    sys.path.insert(0,str(ROOT)); from platform_a.teach_jog import read_state,checked_stopped_state
    fr5=checked_stopped_state(read_state())
    return {'tcp_pose_mm_deg':[float(x) for x in fr5['tcp_pose_mm_deg']], 'joints_deg':[float(x) for x in fr5['joint_position_deg']], 'timestamp':datetime.now(timezone.utc).isoformat(timespec='milliseconds')}

def record(kind):
    d=json.loads(RIGHT_FILE.read_text()) if RIGHT_FILE.exists() else {}
    key={'record_right_start':'start','record_right_end':'end'}.get(kind,kind)
    d[key]=state_snapshot(); RIGHT_FILE.parent.mkdir(parents=True,exist_ok=True); RIGHT_FILE.write_text(json.dumps(d,indent=2),encoding='utf-8'); print(json.dumps(d[key],indent=2))

def main():
    ap=argparse.ArgumentParser(); sub=ap.add_subparsers(dest='cmd',required=True)
    for k in ('record_right_start','record_right_end'): sub.add_parser(k)
    p=sub.add_parser('compute'); p.add_argument('--session',type=Path,required=True); p.add_argument('--out',type=Path,required=True); p.add_argument('--sole-roi',default='130,70,100,170'); p.add_argument('--max-depth-m',type=float,default=.60); p.add_argument('--sole-normal-sign',type=float,default=1.0); p.add_argument('--yolo-model',type=Path,default=None); p.add_argument('--yolo-conf',type=float,default=.25)
    a=ap.parse_args()
    if a.cmd.startswith('record_'): return record(a.cmd)
    root=a.session; a.out.mkdir(parents=True,exist_ok=True); x0,y0,w,h=map(int,a.sole_roi.split(','))
    dep=np.load(root/'sole/depth_median.npy'); cam=json.loads((root/'sole/camerainfo.json').read_text()); K=np.asarray(cam['k'],float).reshape(3,3)
    T=np.asarray(json.loads((root/'sole/tf_samples.json').read_text())[7]['base_T_camera'],float); yy,xx=np.indices(dep.shape); z=dep/1000.; X=(xx-K[0,2])/K[0,0]*z; Y=(yy-K[1,2])/K[1,1]*z
    sel=(xx>=x0)&(xx<x0+w)&(yy>=y0)&(yy<y0+h)&(dep>0)&(z<a.max_depth_m); yolo_info=None
    if a.yolo_model:
        from ultralytics import YOLO
        image=cv2.imread(str(root/'sole/rgb.png')); model=YOLO(str(a.yolo_model)); result=model.predict(image,conf=a.yolo_conf,retina_masks=True,verbose=False)[0]
        if result.masks is None or len(result.masks)==0: raise RuntimeError('YOLO did not detect heel')
        scores=result.boxes.conf.cpu().numpy(); idx=int(np.argmax(scores)); mask=np.zeros(dep.shape,dtype=np.uint8)
        poly=result.masks.xy[idx].astype(np.int32).reshape(-1,1,2); cv2.fillPoly(mask,[poly],255); sel &= mask>0
        box=result.boxes.xyxy[idx].cpu().numpy().tolist(); yolo_info={'model':str(a.yolo_model),'class':'heel','confidence':float(scores[idx]),'box_xyxy':box,'mask_path':str(a.out/'sole_heel_yolo_mask.png')}; cv2.imwrite(str(a.out/'sole_heel_yolo_mask.png'),mask)
    P=np.stack([X,Y,z],-1)[sel]
    C,n,d,mask,res=ransac_plane(P)
    n=unit(a.sole_normal_sign*n); d=float(-n@C)
    m5=json.loads((root/'m5_debug/m5_pivot.json').read_text()); O=np.asarray(m5['pivot_base_xyz_m']); nH=unit(np.asarray(m5['heel_plane_base_normal'])); RB,t=T[:3,:3],T[:3,3]; CS_B=RB@C+t; nS_B=unit(RB@n)
    r=CS_B-O; y0v=r-(r@nH)*nH; Yp=unit(y0v); right=None; Xp=None
    if RIGHT_FILE.exists():
        rh=json.loads(RIGHT_FILE.read_text())
        # Accept the pre-fix key names from the first recording attempt.
        if 'start' not in rh and 'record_right_start' in rh: rh['start']=rh['record_right_start']
        if 'end' not in rh and 'record_right_end' in rh: rh['end']=rh['record_right_end']
        if 'start' in rh and 'end' in rh:
            p0=np.asarray(rh['start']['tcp_pose_mm_deg'][:3])/1000.; p1=np.asarray(rh['end']['tcp_pose_mm_deg'][:3])/1000.; right=unit(p1-p0); xc=unit(right-(right@nH)*nH); Xp=xc if xc@right>=0 else -xc
    status='WAITING_CP4' if Xp is not None else 'WAITING_RIGHT_HINT'
    out={'algorithm':'sole aligned depth -> local depth ROI -> RANSAC plane -> PCA refine; Y+=project(C_S-O, heel plane); X+ sign from right_hint','sole_roi_xywh':[x0,y0,w,h],'sole_point_count':int(len(P)),'sole_inlier_count':int(mask.sum()),'sole_inlier_ratio':float(mask.mean()),'sole_centroid_base_m':CS_B.tolist(),'sole_normal_base':nS_B.tolist(),'sole_rms_m':float(np.sqrt(np.mean(res**2))),'pivot_base_xyz_m':O.tolist(),'heel_normal_base':nH.tolist(),'heel_y_plus':Yp.tolist(),'heel_y_minus':(-Yp).tolist(),'right_hint':None if right is None else right.tolist(),'heel_x_plus':None if Xp is None else Xp.tolist(),'heel_x_minus':None if Xp is None else (-Xp).tolist(),'status':status}
    out['yolo_heel_detection']=yolo_info; out['sole_mask_point_count']=int(len(P)); (a.out/'m6_axes.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    report=f'''===== M6 HEEL AXES =====\n\nsole RMS mm = {out["sole_rms_m"]*1000:.6f}\nsole normal = {out["sole_normal_base"]}\nheel X+ = {out["heel_x_plus"]}\nheel Y+ = {out["heel_y_plus"]}\nheel normal = {out["heel_normal_base"]}\nright_hint = {out["right_hint"]}\nstatus = {status}\n'''
    (a.out/'stage_report.txt').write_text(report,encoding='utf-8'); print(report)
if __name__=='__main__': main()
