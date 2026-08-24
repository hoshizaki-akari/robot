#!/usr/bin/env python3
"""M5 heel local plane and pivot from the saved wire_a RGB/depth session."""
import argparse, json, math
from pathlib import Path
import cv2
import numpy as np

def unit(x):
    n=np.linalg.norm(x)
    if n < 1e-12: raise ValueError('degenerate vector')
    return x/n

def ransac_plane(P, threshold=0.008, iterations=2500, seed=7):
    rng=np.random.default_rng(seed); best=np.zeros(len(P),bool)
    for _ in range(iterations):
        ids=rng.choice(len(P),3,replace=False); a,b,c=P[ids]
        n=np.cross(b-a,c-a); nn=np.linalg.norm(n)
        if nn<1e-8: continue
        n=n/nn; d=-n@a
        mask=np.abs(P@n+d)<threshold
        if mask.sum()>best.sum(): best=mask
    if best.sum()<30: raise RuntimeError('insufficient plane inliers')
    Q=P[best]; centroid=Q.mean(axis=0); _,_,vh=np.linalg.svd(Q-centroid,full_matrices=False)
    n=unit(vh[-1]); d=-n@centroid
    if n[2]<0: n=-n; d=-d
    residual=np.abs(Q@n+d)
    return centroid,n,d,best,residual

def main():
    ap=argparse.ArgumentParser()
    ap.add_argument('--session',type=Path,required=True); ap.add_argument('--out',type=Path,required=True)
    ap.add_argument('--roi',default='121,35,99,188'); ap.add_argument('--entry',default='185,180')
    ap.add_argument('--plane-threshold-mm',type=float,default=8.0)
    args=ap.parse_args(); root=args.session; args.out.mkdir(parents=True,exist_ok=True)
    x0,y0,w,h=map(int,args.roi.split(',')); eu,ev=map(float,args.entry.split(','))
    rgb=cv2.imread(str(root/'wire_a/rgb.png')); depth=np.load(root/'wire_a/depth_median.npy')
    cam=json.loads((root/'wire_a/camerainfo.json').read_text()); K=np.asarray(cam['k'],float).reshape(3,3)
    D=np.asarray(cam['d'],float); T=np.asarray(json.loads((root/'wire_a/tf_samples.json').read_text())[7]['base_T_camera'],float)
    m3=json.loads((root/'m3_wire_results_independent.json').read_text())['wire_a']
    x1,y1,x2,y2=m3['visible_segment_px']; yy,xx=np.indices(depth.shape)
    roi=(xx>=x0)&(xx<x0+w)&(yy>=y0)&(yy<y0+h)
    local=((xx-eu)**2+(yy-ev)**2 <= 72**2)
    # Keep the foot-side portion of the yellow ROI; this avoids the distant wall/box.
    local &= (yy >= ev-72)
    dist_line=np.abs((x2-x1)*(yy-y1)-(y2-y1)*(xx-x1))/max(math.hypot(x2-x1,y2-y1),1e-9)
    needle=dist_line <= 8.0
    valid=roi&local&(~needle)&(depth>0)
    z=depth/1000.0; X=(xx-K[0,2])/K[0,0]*z; Y=(yy-K[1,2])/K[1,1]*z
    Pcam=np.stack([X,Y,z],axis=-1)[valid]
    # Use approximate entry only to select the nearby heel depth layer.
    entry_depth=float(depth[int(round(ev)),int(round(eu))])/1000.0
    if entry_depth <= 0: entry_depth=float(np.median(Pcam[:,2]))
    Pcam=Pcam[np.abs(Pcam[:,2]-entry_depth)<0.20]
    if len(Pcam)<100: raise RuntimeError('too few local depth points')
    C,n,d,inlier,res=ransac_plane(Pcam,args.plane_threshold_mm/1000.0)
    # M4 line in Base.
    m4=json.loads((root/'m4_debug/m4_wire_3d.json').read_text()); P0=np.asarray(m4['P0_m']); v=unit(np.asarray(m4['wire_direction_unsigned']))
    den=n @ (T[:3,:3] @ v)  # plane in camera coords, line is first transformed to camera below
    # Transform heel plane to Base and intersect there.
    RB,tb=T[:3,:3],T[:3,3]; nB=unit(RB@n); dB=float(d-nB@tb)
    th=-(nB@P0+dB)/(nB@v); O=P0+th*v
    # Resolve sign toward the visible endpoint farther from the computed pivot.
    TCB=np.linalg.inv(T)
    def pix_of(P):
        q=TCB[:3,:3]@P+TCB[:3,3]; return (K@(q/q[2]))[:2]
    qO=TCB[:3,:3]@O+TCB[:3,3]; uvO=(K@(qO/qO[2]))[:2]
    measured=np.array([[x1,y1],[x2,y2]],float)
    target=measured[np.argmax(np.linalg.norm(measured-uvO,axis=1))]
    v0=v if np.linalg.norm(pix_of(P0+0.5*v)-target) <= np.linalg.norm(pix_of(P0-0.5*v)-target) else -v
    # Project pivot and draw overlay.
    q=(TCB[:3,:3]@O+TCB[:3,3]); uv=(K@(q/q[2])).astype(float); uv=uv[:2]
    overlay=rgb.copy(); cv2.rectangle(overlay,(x0,y0),(x0+w,y0+h),(0,255,255),2)
    cv2.line(overlay,(int(x1),int(y1)),(int(x2),int(y2)),(255,180,0),2,cv2.LINE_AA)
    ip=tuple(np.round(uv).astype(int)); cv2.circle(overlay,ip,11,(255,255,255),-1); cv2.circle(overlay,ip,8,(0,0,255),-1); cv2.drawMarker(overlay,ip,(0,0,255),cv2.MARKER_CROSS,26,2)
    cv2.putText(overlay,f'FINAL PIVOT ({uv[0]:.1f},{uv[1]:.1f})',tuple(np.round(uv+np.array([8,-10])).astype(int)),cv2.FONT_HERSHEY_SIMPLEX,.48,(0,0,255),2)
    cv2.imwrite(str(args.out/'wire_a_pivot_overlay.png'),overlay)
    rms=float(np.sqrt(np.mean(res**2))); ratio=float(inlier.sum()/len(Pcam))
    out={'algorithm':'aligned depth -> needle mask exclusion -> RANSAC plane -> PCA refine -> 3D wire/heel-plane intersection','roi_xywh':[x0,y0,w,h],'approx_entry_pixel':[eu,ev],'heel_point_count':int(len(Pcam)),'inlier_count':int(inlier.sum()),'inlier_ratio':ratio,'heel_centroid_camera_m':C.tolist(),'heel_normal_camera':n.tolist(),'heel_rms_m':rms,'heel_plane_base_normal':nB.tolist(),'heel_plane_base_d':dB,'pivot_base_xyz_m':O.tolist(),'wire_direction_v0':v0.tolist(),'pivot_pixel':[float(uv[0]),float(uv[1])],'debug_image':str(args.out/'wire_a_pivot_overlay.png'),'status':'WAITING_CP3'}
    (args.out/'m5_pivot.json').write_text(json.dumps(out,indent=2),encoding='utf-8')
    report=f'''===== M5 HEEL PIVOT =====\n\nheel point count = {len(Pcam)}\ninlier ratio = {ratio:.6f}\nheel RMS mm = {rms*1000:.6f}\npivot Base XYZ = {O.tolist()}\nwire direction v0 = {v0.tolist()}\ndebug image = {args.out/'wire_a_pivot_overlay.png'}\nstatus = WAITING_CP3\n'''
    (args.out/'stage_report.txt').write_text(report,encoding='utf-8'); print(report)
if __name__=='__main__': main()
