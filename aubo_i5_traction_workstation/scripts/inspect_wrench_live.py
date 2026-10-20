#!/usr/bin/env python3
"""Read-only sensor/pose/transform continuity report; never requests motion."""

import argparse
import json
import math
import time
from pathlib import Path

import rclpy
import yaml
from fr_traction.msg import EndEffectorState
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--calibration', type=Path, required=True)
    parser.add_argument('--seconds', type=float, default=15.0)
    parser.add_argument('--output', type=Path, required=True)
    args = parser.parse_args()
    config = yaml.safe_load(args.calibration.read_text())['aubo_wrench_transform']['ros__parameters']
    reference = config['tare_q_base_flange']
    stats = {}
    pose = {}
    observations = []
    rclpy.init()
    node = rclpy.create_node('inspect_wrench_readonly')

    def accept(name, message):
        now = time.monotonic()
        item = stats.setdefault(name, {'count': 0, 'max_gap_s': 0.0, 'last': now})
        item['max_gap_s'] = max(item['max_gap_s'], now - item['last'])
        item['last'] = now
        item['count'] += 1
        if name == 'pose':
            q = message.pose.orientation
            values = [q.x, q.y, q.z, q.w]
            dot = abs(sum(a * b for a, b in zip(values, reference)))
            norm = math.sqrt(sum(x * x for x in values) * sum(x * x for x in reference))
            pose['q_xyzw'] = values
            pose['tare_angle_rad'] = 2 * math.acos(min(1.0, dot / norm))
        if name == 'raw_valid':
            item['valid'] = message.data
        if name in ('raw', 'transformed', 'corrected'):
            force = message.wrench.force
            item['last_force_n'] = [force.x, force.y, force.z]

    subscriptions = []
    for name, topic, kind, qos in (
        ('raw', '/aubo/kwr75/raw_wrench', WrenchStamped, qos_profile_sensor_data),
        ('raw_valid', '/aubo/kwr75/raw_wrench_valid', Bool, 10),
        ('pose', '/controller_manager/ee_state', EndEffectorState, 10),
        ('transformed', '/controller_manager/wrench', WrenchStamped, 10),
        ('corrected', '/traction/corrected_wrench', WrenchStamped, 10),
    ):
        subscriptions.append(node.create_subscription(
            kind, topic, lambda msg, name=name: accept(name, msg), qos))
    began = time.monotonic()
    next_sample = began
    try:
        while time.monotonic() - began < args.seconds:
            rclpy.spin_once(node, timeout_sec=0.02)
            now = time.monotonic()
            if now >= next_sample:
                observations.append({
                    'elapsed_s': now - began,
                    'ages_s': {key: now - item['last'] for key, item in stats.items()},
                    'tare_angle_rad': pose.get('tare_angle_rad'),
                    'raw_valid': stats.get('raw_valid', {}).get('valid'),
                })
                next_sample = now + 0.1
        now = time.monotonic()
        result = {
            'calibration': str(args.calibration),
            'duration_s': now - began,
            'configured_tare_tolerance_rad': config['tare_pose_tolerance_rad'],
            'pose': pose,
            'topics': {
                key: {**item, 'age_s': now - item['last']} for key, item in stats.items()
            },
            'observations': observations,
        }
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(result, indent=2) + '\n')
        print(json.dumps({key: value for key, value in result.items() if key != 'observations'}, indent=2))
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
