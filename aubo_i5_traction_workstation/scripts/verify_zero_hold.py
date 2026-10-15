#!/usr/bin/env python3
"""Bounded live driver test: stream zero velocity for two seconds, then stop."""

import argparse
import time

import rclpy
from controller_manager_msgs.srv import SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from fr_traction.msg import EndEffectorState
from geometry_msgs.msg import Twist
from rclpy.qos import QoSProfile, DurabilityPolicy


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-zero-hold', action='store_true', required=True)
    parser.parse_args()
    rclpy.init()
    node = rclpy.create_node('aubo_bounded_zero_hold_verifier')
    seen = {}
    qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def diagnostic(message):
        for status in message.status:
            if status.name == 'aubo_v4_driver/state':
                seen['diagnostic'] = status
                seen['level'] = (
                    int.from_bytes(status.level, 'little')
                    if isinstance(status.level, bytes) else int(status.level)
                )

    def pose(message):
        p = message.pose.position
        seen['position'] = (p.x, p.y, p.z)
        seen['pose_at'] = time.monotonic()

    subscriptions = [
        node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, qos),
        node.create_subscription(EndEffectorState, '/controller_manager/ee_state', pose, 10),
    ]
    publisher = node.create_publisher(Twist, '/controller_manager/command_cart_vel', 10)
    client = node.create_client(SwitchController, '/controller_manager/switch_controller')
    zero = Twist()

    def pump(duration, publish=True):
        deadline = time.monotonic() + duration
        while time.monotonic() < deadline:
            if publish:
                publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.005)

    def switch(start):
        request = SwitchController.Request()
        names = ['cartesian_velocity_controller']
        request.start_controllers = names if start else []
        request.stop_controllers = [] if start else names
        request.strictness = SwitchController.Request.STRICT
        request.start_asap = start
        request.activate_asap = start
        future = client.call_async(request)
        deadline = time.monotonic() + 2.0
        while not future.done() and time.monotonic() < deadline:
            publisher.publish(zero)
            rclpy.spin_once(node, timeout_sec=0.005)
        return future.done() and future.result() is not None and future.result().ok

    start_attempted = False
    try:
        pump(5.0, publish=False)
        status = seen.get('diagnostic')
        if not status or seen['level'] != 0 or 'position' not in seen:
            print(f'driver_error={status.message if status else "no diagnostic received"}', flush=True)
            print(f'diagnostic_level={status.level if status else "missing"}; pose_received={"position" in seen}', flush=True)
            print('status=REJECTED_NO_HEALTHY_FEEDBACK', flush=True)
            return 2
        if not client.wait_for_service(timeout_sec=1.0):
            print('status=REJECTED_NO_SWITCH_SERVICE', flush=True)
            return 2
        origin = seen['position']
        start_attempted = True
        if not switch(True):
            print('status=STREAM_ENTRY_FAILED', flush=True)
            return 2
        deadline = time.monotonic() + 2.0
        max_drift = 0.0
        while time.monotonic() < deadline:
            pump(0.01)
            max_drift = max(max_drift, sum(
                (a - b) ** 2 for a, b in zip(seen['position'], origin)
            ) ** 0.5)
            status = seen['diagnostic']
            if seen['level'] != 0:
                print(f'driver_error={status.message}', flush=True)
                print('status=STREAM_FAILED', flush=True)
                return 1
            if time.monotonic() - seen['pose_at'] > 0.25 or max_drift > 0.0005:
                print('status=FEEDBACK_OR_HOLD_FAILED', flush=True)
                return 1
        print(f'max_hold_drift_m={max_drift:.9f}', flush=True)
        print('status=ZERO_HOLD_PASS', flush=True)
        return 0
    finally:
        if start_attempted:
            print(f'stream_stop_acknowledged={switch(False)}', flush=True)
        _ = subscriptions
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    raise SystemExit(main())
