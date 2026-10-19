#!/usr/bin/env python3
"""One explicitly requested, measured-envelope tool-X move; no automatic retries."""

import argparse
import csv
import json
import math
import time
from pathlib import Path

import rclpy
from controller_manager_msgs.srv import SwitchController
from diagnostic_msgs.msg import DiagnosticArray
from fr_traction.msg import EndEffectorState
from geometry_msgs.msg import Twist, WrenchStamped
from live_motion_bounds import (
    best_effort_stop,
    pose_is_after_fence,
    require_target_reached,
    require_within_envelope,
)
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from std_msgs.msg import Bool


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--execute-tool-x', action='store_true', required=True)
    parser.add_argument('--distance-m', type=float, required=True)
    parser.add_argument('--baseline', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--continue-result', type=Path,
                        help='Finish only the remaining distance inside a previous real envelope')
    args = parser.parse_args()
    if not math.isfinite(args.distance_m) or not 0.0005 <= args.distance_m <= 0.05:
        parser.error('distance must be between 0.5 mm and 50 mm')
    baseline = json.loads(args.baseline.read_text(encoding='utf-8'))
    bias = baseline['force_bias_sensor_n']
    if len(bias) != 3 or not all(math.isfinite(v) for v in bias):
        parser.error('invalid measured baseline')
    rclpy.init()
    node = rclpy.create_node('single_bounded_stepwise_tool_x_verifier')
    data = {}
    rows = []
    latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)

    def pose(message):
        p, q = message.pose.position, message.pose.orientation
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        values = (p.x, p.y, p.z, q.x, q.y, q.z, q.w)
        if (message.header.frame_id != 'base_link' or not -0.05 <= age <= 0.5 or
                not all(math.isfinite(v) for v in values)):
            data.pop('pose', None)
            return
        data.update(pose=values, pose_at=time.monotonic(),
                    pose_source_ns=message.header.stamp.sec * 1_000_000_000 +
                    message.header.stamp.nanosec)

    def force(message):
        f = message.wrench.force
        stamp = message.header.stamp.sec + message.header.stamp.nanosec * 1e-9
        age = node.get_clock().now().nanoseconds * 1e-9 - stamp
        values = (f.x, f.y, f.z)
        if (message.header.frame_id == 'kwr75_sensor' and -0.05 <= age <= 0.25 and
                all(math.isfinite(v) for v in values)):
            data.update(force=values, force_at=time.monotonic())

    def diagnostic(message):
        for status in message.status:
            if status.name == 'aubo_v4_driver/state':
                data['driver'] = {item.key: item.value for item in status.values}
                data['reason'] = status.message
                data['diagnostic_at'] = time.monotonic()

    def idle(message):
        data.update(idle=message.data, idle_at=time.monotonic(),
                    idle_sequence=data.get('idle_sequence', 0) + 1)

    node.create_subscription(EndEffectorState, '/controller_manager/ee_state', pose, 10)
    node.create_subscription(WrenchStamped, '/aubo/kwr75/raw_wrench', force,
                             qos_profile_sensor_data)
    node.create_subscription(DiagnosticArray, '/diagnostics', diagnostic, latched)
    node.create_subscription(Bool, '/controller_manager/motion_idle', idle, latched)
    publisher = node.create_publisher(Twist, '/controller_manager/command_cart_vel', 1)
    client = node.create_client(SwitchController, '/controller_manager/switch_controller')
    zero = Twist()
    activated = False
    summary = {'status': 'NOT_STARTED', 'requested_distance_m': args.distance_m}

    def switch(start):
        request = SwitchController.Request()
        request.start_controllers = ['cartesian_velocity_controller'] if start else []
        request.stop_controllers = [] if start else ['cartesian_velocity_controller']
        request.strictness = SwitchController.Request.STRICT
        request.start_asap = start
        request.activate_asap = start
        future = client.call_async(request)
        rclpy.spin_until_future_complete(node, future, timeout_sec=3.0)
        return future.done() and future.result() is not None and future.result().ok

    def require_inputs():
        now = time.monotonic()
        if ('pose' not in data or 'force' not in data or
                now - data.get('pose_at', 0) > 0.5 or
                now - data.get('force_at', 0) > 0.25 or
                now - data.get('diagnostic_at', 0) > 0.5):
            raise RuntimeError('Missing/stale real feedback')
        driver = data.get('driver', {})
        if driver.get('backend') != 'stepwise' or driver.get('connected') != '1':
            raise RuntimeError('Expected connected stepwise backend')
        if driver.get('stepwise_phase') == 'FAULT':
            raise RuntimeError('Driver fault: ' + data.get('reason', 'unknown'))
        external = math.dist(data['force'], bias)
        if external > 3.0:
            raise RuntimeError(f'Unexpected external force {external:.3f} N')
        return external

    try:
        deadline = time.monotonic() + 3
        while time.monotonic() < deadline:
            rclpy.spin_once(node, timeout_sec=0.02)
        require_inputs()
        if (not data.get('idle') or data['driver'].get('motion_interface_enabled') != '1' or
                node.count_publishers('/controller_manager/command_cart_vel') != 1):
            raise RuntimeError('Motion not enabled/idle, or conflicting command publisher')
        origin = data['pose'][:3]
        horizon = float(data['driver']['planning_horizon_s'])
        maximum_step = float(data['driver']['maximum_step_m'])
        if not 0 < horizon <= 0.5 or not 0 < maximum_step <= 0.001:
            raise RuntimeError('Invalid reported driver planning bounds')
        target_tolerance = min(0.0005, max(0.0003, maximum_step * 0.5))
        summary['target_acceptance_tolerance_m'] = target_tolerance
        q = data['pose'][3:]
        if abs(math.sqrt(sum(v * v for v in q)) - 1) > 0.001:
            raise RuntimeError('Invalid measured orientation')
        x, y, z, w = q
        direction = (1 - 2 * (y * y + z * z), 2 * (x * y + w * z),
                     2 * (x * z - w * y))
        if args.continue_result:
            previous = json.loads(args.continue_result.read_text(encoding='utf-8'))
            if (previous.get('requested_distance_m') != args.distance_m or
                    previous.get('status') != 'STOPPED_NOT_ACCEPTED' or
                    math.dist(origin, previous['final_position_m']) > 0.001):
                raise RuntimeError('Continuation does not match the current measured stopped pose')
            origin = tuple(previous['origin_m'])
            direction = tuple(previous['commanded_direction_base'])
            require_within_envelope(data['pose'][:3], origin, direction, args.distance_m)
            summary['continued_original_result'] = str(args.continue_result)
        if not client.wait_for_service(timeout_sec=2):
            raise RuntimeError('Controller switch unavailable')
        activated = True
        if not switch(True):
            raise RuntimeError('Controller activation refused')
        started = time.monotonic()
        previous_phase = None
        summary.update(origin_m=origin, commanded_direction_base=direction)
        while time.monotonic() - started < 240:
            rclpy.spin_once(node, timeout_sec=0.01)
            external = require_inputs()
            delta = tuple(a - b for a, b in zip(data['pose'][:3], origin))
            projection = sum(a * b for a, b in zip(delta, direction))
            distance = math.sqrt(sum(v * v for v in delta))
            lateral = math.sqrt(max(0, distance * distance - projection * projection))
            phase = data['driver'].get('stepwise_phase')
            if phase != previous_phase:
                print(f'phase={phase} displacement_mm={1000 * projection:.3f}', flush=True)
                previous_phase = phase
            rows.append((time.monotonic() - started, *data['pose'][:3], projection,
                         lateral, external, phase))
            if distance > args.distance_m + 0.0001 or lateral > 0.001 or projection < -0.0005:
                raise RuntimeError('Measured displacement exceeded requested direction/envelope')
            if projection >= args.distance_m - target_tolerance:
                summary['status'] = 'TARGET_REACHED'
                break
            command = Twist()
            # Correct actual lateral deviation instead of allowing IK error
            # from each measured endpoint to accumulate into a curved path.
            speed = min(0.002, max(0.0, args.distance_m - projection - 0.0001) / horizon)
            velocity = tuple(speed * axis - 2.0 * (offset - projection * axis)
                             for axis, offset in zip(direction, delta))
            magnitude = math.sqrt(sum(v * v for v in velocity))
            scale = min(1.0, 0.002 / magnitude) if magnitude else 1.0
            command.linear.x, command.linear.y, command.linear.z = (v * scale for v in velocity)
            publisher.publish(command)
        else:
            raise RuntimeError('Single move timed out; no automatic retry')
    except (Exception, KeyboardInterrupt) as error:  # noqa: BLE001 - always enter explicit stop
        summary.update(status='STOPPED_NOT_ACCEPTED', reason=str(error))
    finally:
        if activated:
            def wait_for_measured_stop():
                deadline = time.monotonic() + 6
                stopped_frames = 0
                previous_idle_sequence = data.get('idle_sequence', 0)
                while time.monotonic() < deadline and stopped_frames < 3:
                    rclpy.spin_once(node, timeout_sec=0.02)
                    if data.get('idle_sequence', 0) != previous_idle_sequence:
                        previous_idle_sequence = data['idle_sequence']
                        stopped_frames = stopped_frames + 1 if (
                            data.get('idle') and time.monotonic() - data.get('idle_at', 0) < 0.25
                        ) else 0
                return stopped_frames >= 3

            summary.update(best_effort_stop(lambda: publisher.publish(zero),
                                            lambda: switch(False), wait_for_measured_stop))
            if summary['stop_errors']:
                summary['status'] = 'STOPPED_NOT_ACCEPTED'
            # A fresh pose AFTER stop confirmation is mandatory, not a cached
            # pre-stop position or a Boolean idle alone.
            final_fence = time.monotonic()
            final_source_fence = node.get_clock().now().nanoseconds
            deadline = final_fence + 1.0
            while time.monotonic() < deadline and not pose_is_after_fence(
                    data.get('pose_source_ns'), final_source_fence):
                rclpy.spin_once(node, timeout_sec=0.02)
            if not pose_is_after_fence(data.get('pose_source_ns'), final_source_fence):
                summary.update(status='STOPPED_NOT_ACCEPTED', reason='No fresh final pose')
        if 'origin_m' in summary and 'pose' in data:
            summary['final_position_m'] = data['pose'][:3]
            summary['final_displacement_m'] = math.dist(data['pose'][:3], summary['origin_m'])
            try:
                _, projection, lateral = require_within_envelope(
                    data['pose'][:3], summary['origin_m'], summary['commanded_direction_base'],
                    args.distance_m)
                summary.update(final_projected_m=projection, final_lateral_m=lateral)
                if summary['status'] == 'TARGET_REACHED':
                    require_target_reached(
                        data['pose'][:3], summary['origin_m'],
                        summary['commanded_direction_base'], args.distance_m,
                        summary['target_acceptance_tolerance_m'])
            except ValueError as error:
                summary.update(status='STOPPED_NOT_ACCEPTED', reason=str(error))
        args.output.parent.mkdir(parents=True, exist_ok=True)
        args.output.write_text(json.dumps(summary, indent=2) + '\n', encoding='utf-8')
        with args.output.with_suffix('.csv').open('w', newline='', encoding='utf-8') as stream:
            writer = csv.writer(stream)
            writer.writerow(('elapsed_s', 'x_m', 'y_m', 'z_m', 'projected_m',
                             'lateral_m', 'external_force_n', 'phase'))
            writer.writerows(rows)
        print(json.dumps(summary, indent=2), flush=True)
        node.destroy_node()
        rclpy.shutdown()
    return 0 if summary['status'] == 'TARGET_REACHED' and summary.get(
        'measured_stop_confirmed') else 1


if __name__ == '__main__':
    raise SystemExit(main())
