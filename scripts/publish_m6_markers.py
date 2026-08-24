#!/usr/bin/env python3
import argparse,json
from pathlib import Path
import numpy as np
import cv2
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
from sensor_msgs.msg import PointCloud2
from sensor_msgs_py import point_cloud2

class Axes(Node):
    def __init__(self,d,session):
        super().__init__('m6_heel_axes_markers'); q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub=self.create_publisher(MarkerArray,'/m6/heel_axes',q); self.arr=MarkerArray(); self.make(d); self.timer=self.create_timer(1.,self.pub_once); self.pub_once()
        self.cloud_pub=self.create_publisher(PointCloud2,'/m6/sole_pointcloud',q)
        self.cloud=self.make_cloud(d,Path(session)); self.cloud_timer=self.create_timer(1.,self.cloud_once); self.cloud_once()
    def make_cloud(self,d,root):
        cam=json.load(open(root/'sole/camerainfo.json')); K=np.asarray(cam['k'],float).reshape(3,3)
        T=np.asarray(json.load(open(root/'sole/tf_samples.json'))[7]['base_T_camera'],float)
        dep=np.load(root/'sole/depth_median.npy'); x0,y0,w,h=d['sole_roi_xywh']; yy,xx=np.indices(dep.shape); z=dep/1000.
        sel=(xx>=x0)&(xx<x0+w)&(yy>=y0)&(yy<y0+h)&(dep>0)&(z<0.60)
        info=d.get('yolo_heel_detection') or {}; mp=Path(info.get('mask_path',''))
        if mp:
            if not mp.exists(): mp=root.parent.parent/mp
            if mp.exists(): sel &= cv2.imread(str(mp),cv2.IMREAD_GRAYSCALE)>0
        # Downsample the saved point cloud for responsive RViz display.
        ys,xs=np.where(sel); ys,xs=ys[::2],xs[::2]; zs=z[ys,xs]
        pc=np.column_stack([(xs-K[0,2])/K[0,0]*zs,(ys-K[1,2])/K[1,1]*zs,zs])
        pb=(T[:3,:3]@pc.T+T[:3,3:4]).T
        from std_msgs.msg import Header
        hmsg=Header(); hmsg.frame_id='base'; return point_cloud2.create_cloud_xyz32(hmsg,pb.tolist())
    def cloud_once(self):
        self.cloud.header.stamp=self.get_clock().now().to_msg(); self.cloud_pub.publish(self.cloud)
    def arrow(self,i,name,p,v,color):
        m=Marker(); m.header.frame_id='base'; m.ns='m6'; m.id=i; m.type=Marker.ARROW; m.action=Marker.ADD; m.scale.x=.006; m.scale.y=.012; m.scale.z=.018; m.color.r=color[0]; m.color.g=color[1]; m.color.b=color[2]; m.color.a=1.; m.points=[Point(x=float(p[0]),y=float(p[1]),z=float(p[2])),Point(x=float(p[0]+.12*v[0]),y=float(p[1]+.12*v[1]),z=float(p[2]+.12*v[2]))]; self.arr.markers.append(m)
    def make(self,d):
        O=d['pivot_base_xyz_m']; X=d['heel_x_plus']; Y=d['heel_y_plus']; H=d['heel_normal_base']; S=d['sole_normal_base']
        m=Marker(); m.header.frame_id='base'; m.ns='m6'; m.id=0; m.type=Marker.SPHERE; m.action=Marker.ADD; m.pose.position.x=O[0];m.pose.position.y=O[1];m.pose.position.z=O[2];m.scale.x=m.scale.y=m.scale.z=.025;m.color.r=1.;m.color.a=1.;self.arr.markers.append(m)
        self.arrow(1,'X+',O,X,(1.,0.,0.)); self.arrow(2,'Y+',O,Y,(0.,1.,0.)); self.arrow(3,'heel_normal',O,H,(0.,0.,1.)); self.arrow(4,'sole_normal',d['sole_centroid_base_m'],S,(1.,1.,0.))
    def pub_once(self):
        now=self.get_clock().now().to_msg()
        for m in self.arr.markers:m.header.stamp=now
        self.pub.publish(self.arr)
def main():
    ap=argparse.ArgumentParser();ap.add_argument('--input',required=True);ap.add_argument('--session',required=True);a=ap.parse_args();rclpy.init();n=Axes(json.load(open(a.input)),a.session);print('publishing /m6/heel_axes and /m6/sole_pointcloud in frame base',flush=True)
    try:rclpy.spin(n)
    except KeyboardInterrupt:pass
    n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
