#!/usr/bin/env python3
"""Capture real ROS force/pose data without any robot or sensor commands."""

import argparse
import json
import math
import statistics
import time
from collections import deque
from pathlib import Path

import rclpy
from fr_traction.msg import EndEffectorState
from geometry_msgs.msg import WrenchStamped
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import JointState
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=3.0)
    parser.add_argument('--label', choices=('unloaded', 'tool_x_push', 'tool_y_push', 'tool_z_push'),
                        default='unloaded')
    args = parser.parse_args()
    if not math.isfinite(args.seconds) or not 1.0 <= args.seconds <= 10.0:
        parser.error('seconds must be in [1, 10]')
    rclpy.init()
    node = Node('capture_real_force_checkpoint')
    samples = deque(maxlen=20000)
    poses = deque(maxlen=2000)
    joints = deque(maxlen=2000)
    idle = deque(maxlen=2000)

    def force(message):
        values = tuple(getattr(vector, axis) for vector in (
            message.wrench.force, message.wrench.torque) for axis in 'xyz')
        source = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - source
        if (message.header.frame_id == 'kwr75_sensor' and -0.05 <= age <= 0.25
                and all(math.isfinite(value) for value in values)):
            samples.append((time.monotonic(), values))

    def pose(message):
        source = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - source
        q = message.pose.orientation
        p = message.pose.position
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if (message.header.frame_id == 'base_link' and -0.05 <= age <= 0.5
                and all(math.isfinite(value) for value in values)):
            poses.append((time.monotonic(), values))

    node.create_subscription(WrenchStamped, '/aubo/kwr75/raw_wrench', force,
                             qos_profile_sensor_data)
    node.create_subscription(EndEffectorState, '/controller_manager/ee_state', pose, 10)
    node.create_subscription(JointState, '/joint_states',
                             lambda message: joints.append((time.monotonic(),
                                                            tuple(message.velocity))), 10)
    node.create_subscription(Bool, '/controller_manager/motion_idle',
                             lambda message: idle.append((time.monotonic(), message.data)), 10)
    deadline = time.monotonic() + args.seconds + 1.0
    try:
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        now = time.monotonic()
        selected = [values for receipt, values in samples if now - receipt <= args.seconds]
        recent_poses = [values for receipt, values in poses if now - receipt <= args.seconds]
        recent_joints = [values for receipt, values in joints if now - receipt <= args.seconds]
        recent_idle = [value for receipt, value in idle if now - receipt <= args.seconds]
        if len(selected) < 100 or not recent_poses or not recent_joints or not recent_idle:
            raise RuntimeError('Missing real force, pose, joint, or idle samples; no baseline saved')
        if any(len(values) != 6 or not all(math.isfinite(v) and abs(v) < 0.02
                                         for v in values) for values in recent_joints):
            raise RuntimeError('Joint feedback not stationary; no baseline saved')
        if (not all(recent_idle) or now - poses[-1][0] > 0.5 or now - samples[-1][0] > 0.25
                or now - joints[-1][0] > 0.5 or now - idle[-1][0] > 0.5):
            raise RuntimeError('Measured idle/freshness not confirmed; no baseline saved')
        mean = [statistics.fmean(values[index] for values in selected) for index in range(6)]
        deviation = [statistics.pstdev(values[index] for values in selected) for index in range(6)]
        result = {
            'source': 'REAL_ROS_SUBSCRIPTIONS_NO_COMMANDS',
            'sample_kind': args.label,
            'sample_count': len(selected),
            'force_bias_sensor_n': mean[:3],
            'torque_bias_sensor_nm': mean[3:],
            'standard_deviation_n_nm': deviation,
            'position_base_m': recent_poses[-1][:3],
            'tare_q_base_flange': recent_poses[-1][3:],
            'position_range_m': [max(v[i] for v in recent_poses) - min(v[i] for v in recent_poses)
                                 for i in range(3)],
            'measured_idle': True,
            'mounting_verified': False,
            'force_polarity_verified': False,
            'note': ('Operator-confirmed unloaded sample; fixed-pose tare only' if
                     args.label == 'unloaded' else
                     'Operator-labelled applied force; NOT an unloaded tare'),
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n', encoding='utf-8')
        print(json.dumps(result, indent=2))
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
