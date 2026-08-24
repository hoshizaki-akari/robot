import cv2
import numpy as np
from ultralytics import YOLO

image = cv2.imread("/home/zhj/projects/fr5_platform_ws/current_clamp_pose.png")
model = YOLO("/mnt/c/Users/zhj/Desktop/ji_cheng_YOLO/deeplearning/ultralytics-8.3.163/results/compare2/YOLO11n-seg_20260329_131918/weights/best.pt")
result = model.predict(image, conf=0.2, imgsz=640, verbose=False, device="cpu")[0]
raw = result.masks.data[0].cpu().numpy()
mask = cv2.resize(raw, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST) > .5
distance = cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 3)
gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
edges = cv2.Canny(gray, 35, 120)
lines = cv2.HoughLinesP(edges, 1, np.pi/180, 14, minLineLength=25, maxLineGap=18)
candidates=[]
for x1,y1,x2,y2 in lines[:,0,:]:
    length=float(np.hypot(x2-x1,y2-y1))
    pts=np.rint(np.linspace((x1,y1),(x2,y2),120)).astype(int)
    pts[:,0]=np.clip(pts[:,0],0,image.shape[1]-1); pts[:,1]=np.clip(pts[:,1],0,image.shape[0]-1)
    d=distance[pts[:,1],pts[:,0]]
    if float(d.min())>8: continue
    nearest=int(np.argmin(d)); p=pts[nearest]
    inside=float(mask[pts[:,1],pts[:,0]].mean())
    angle=float(np.degrees(np.arctan2(y2-y1,x2-x1)))
    score=length-40*inside-2*float(d.min())
    candidates.append((score,length,angle,inside,int(p[0]),int(p[1]),int(x1),int(y1),int(x2),int(y2)))
candidates.sort(reverse=True)
debug=image.copy()
for i,item in enumerate(candidates[:20]):
    _,_,_,_,px,py,x1,y1,x2,y2=item
    cv2.line(debug,(x1,y1),(x2,y2),(0,255,255),1)
    cv2.putText(debug,str(i),(px,py),cv2.FONT_HERSHEY_SIMPLEX,.35,(0,0,255),1)
for i,item in enumerate(candidates[:30]): print(i,item)
cv2.imwrite("/home/zhj/projects/fr5_platform_ws/needle_debug.png",debug)
