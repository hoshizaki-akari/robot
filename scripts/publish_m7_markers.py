#!/usr/bin/env python3
import argparse,csv
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile,ReliabilityPolicy,DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker,MarkerArray
class Pub(Node):
 def __init__(self,path):
  super().__init__('m7_trajectory_markers');q=QoSProfile(depth=1,reliability=ReliabilityPolicy.RELIABLE,durability=DurabilityPolicy.TRANSIENT_LOCAL);self.p=self.create_publisher(MarkerArray,'/m7/trajectories',q);self.a=MarkerArray();self.load(path);self.t=self.create_timer(1.,self.go);self.go()
 def load(self,path):
  groups={}
  for r in csv.DictReader(open(path)):groups.setdefault(r['direction'],[]).append(r)
  colors={'X_PLUS':(1.,0.,0.),'X_MINUS':(1.,.4,0.),'Y_PLUS':(0.,1.,0.),'Y_MINUS':(0.,.6,1.)};mid=0
  for name,rows in groups.items():
   m=Marker();m.header.frame_id='base';m.ns='m7';m.id=mid;mid+=1;m.type=Marker.LINE_STRIP;m.action=Marker.ADD;m.scale.x=.004;m.color.r,m.color.g,m.color.b=colors[name];m.color.a=1.;m.points=[Point(x=float(r['x_m']),y=float(r['y_m']),z=float(r['z_m'])) for r in rows];self.a.markers.append(m)
   e=Marker();e.header.frame_id='base';e.ns='m7_final';e.id=mid;mid+=1;e.type=Marker.SPHERE;e.action=Marker.ADD;e.scale.x=e.scale.y=e.scale.z=.012;e.color.r,e.color.g,e.color.b=colors[name];e.color.a=1.;r=rows[-1];e.pose.position.x=float(r['x_m']);e.pose.position.y=float(r['y_m']);e.pose.position.z=float(r['z_m']);self.a.markers.append(e)
  r=next(iter(groups.values()))[0];m=Marker();m.header.frame_id='base';m.ns='m7_pivot';m.id=99;m.type=Marker.SPHERE;m.action=Marker.ADD;m.scale.x=m.scale.y=m.scale.z=.025;m.color.r=1.;m.color.a=1.;m.pose.position.x=float(r['x_m'])-float(r['wire_vx'])*.04;m.pose.position.y=float(r['y_m'])-float(r['wire_vy'])*.04;m.pose.position.z=float(r['z_m'])-float(r['wire_vz'])*.04;self.a.markers.append(m)
 def go(self):
  n=self.get_clock().now().to_msg()
  for m in self.a.markers:m.header.stamp=n
  self.p.publish(self.a)
def main():
 ap=argparse.ArgumentParser();ap.add_argument('--csv',required=True);a=ap.parse_args();rclpy.init();n=Pub(a.csv);print('publishing /m7/trajectories in frame base',flush=True)
 try:rclpy.spin(n)
 except KeyboardInterrupt:pass
 n.destroy_node();rclpy.shutdown()
if __name__=='__main__':main()
