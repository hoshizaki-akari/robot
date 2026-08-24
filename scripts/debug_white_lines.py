import cv2
import numpy as np
from ultralytics import YOLO

im=cv2.imread('/home/zhj/projects/fr5_platform_ws/current_clamp_pose.png')
r=YOLO('/home/zhj/projects/fr5_platform_ws/platform_a/models/heel_seg.pt').predict(im,conf=.2,imgsz=640,verbose=False,device='cpu')[0]
raw=r.masks.data[0].cpu().numpy(); mask=cv2.resize(raw,(im.shape[1],im.shape[0]),interpolation=cv2.INTER_NEAREST)>.5
gray=cv2.createCLAHE(2,(8,8)).apply(cv2.cvtColor(im,cv2.COLOR_BGR2GRAY)); near=cv2.dilate(mask.astype(np.uint8),np.ones((3,3),np.uint8),iterations=70)
top=cv2.morphologyEx(gray,cv2.MORPH_TOPHAT,cv2.getStructuringElement(cv2.MORPH_ELLIPSE,(11,11)))
lim=max(12,float(np.percentile(top[near>0],88))); bright=((top>=lim)&(gray>=105)&(near>0)).astype(np.uint8)*255
bright=cv2.morphologyEx(bright,cv2.MORPH_CLOSE,np.ones((3,3),np.uint8))
lines=cv2.HoughLinesP(bright,1,np.pi/180,5,minLineLength=10,maxLineGap=26)
ys,xs=np.nonzero(mask); cx=xs.mean(); width=xs.max()-xs.min(); topy=ys.min(); bot=ys.max(); out=[]
for x1,y1,x2,y2 in lines[:,0,:]:
 dx=x2-x1;dy=y2-y1;length=np.hypot(dx,dy)
 if length<14 or abs(dy)<abs(dx)*1.8:continue
 lo=min(y1,y2);hi=max(y1,y2);by=None
 if lo<=topy-12 and hi>=topy-8:by=topy
 elif hi>=bot+12 and lo<=bot+8:by=bot
 if by is None:continue
 xb=x1+(by-y1)*dx/dy
 if abs(xb-cx)>width*.5:continue
 pts=np.rint(np.linspace((x1,y1),(x2,y2),100)).astype(int);pts[:,0]=np.clip(pts[:,0],0,im.shape[1]-1);pts[:,1]=np.clip(pts[:,1],0,im.shape[0]-1)
 out.append((round(float(length),1),round(float(xb),1),round(float(abs(xb-cx)),1),round(float(gray[pts[:,1],pts[:,0]].mean()),1),[int(x1),int(y1),int(x2),int(y2)]))
print('center',cx,'width',width,'boundary',topy,bot,'limit',lim)
for x in sorted(out,reverse=True):print(x)
cv2.imwrite('/home/zhj/projects/fr5_platform_ws/bright_mask.png',bright)
