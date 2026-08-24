#!/usr/bin/env python3
import argparse, json
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from geometry_msgs.msg import Point
from visualization_msgs.msg import Marker

class WireMarker(Node):
    def __init__(self, data):
        super().__init__('m4_wire_marker')
        qos = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.pub = self.create_publisher(Marker, '/m4/wire_3d', qos)
        self.msg = Marker(); self.msg.header.frame_id = 'base'; self.msg.ns = 'm4_wire'; self.msg.id = 1
        self.msg.type = Marker.LINE_STRIP; self.msg.action = Marker.ADD
        self.msg.scale.x = 0.004; self.msg.color.r = 1.0; self.msg.color.g = 0.15; self.msg.color.b = 0.05; self.msg.color.a = 1.0
        p = data['P0_m']; v = data['wire_direction_unsigned']; extent = 0.5
        for s in (-extent, extent):
            q = [p[i] + s*v[i] for i in range(3)]; self.msg.points.append(Point(x=q[0], y=q[1], z=q[2]))
        self.timer = self.create_timer(1.0, self.publish)
        self.publish()
    def publish(self):
        self.msg.header.stamp = self.get_clock().now().to_msg(); self.pub.publish(self.msg)

def main():
    ap=argparse.ArgumentParser(); ap.add_argument('--input', required=True); a=ap.parse_args()
    data=json.load(open(a.input)); rclpy.init(); n=WireMarker(data); print('publishing /m4/wire_3d in frame base', flush=True)
    try: rclpy.spin(n)
    except KeyboardInterrupt: pass
    n.destroy_node(); rclpy.shutdown()
if __name__ == '__main__': main()
