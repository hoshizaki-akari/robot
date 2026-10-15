#!/usr/bin/env python3
"""Summarize live AUBO and KWR75D ROS data without issuing motion commands."""

import math
import time

import rclpy
from diagnostic_msgs.msg import DiagnosticArray
from fr_traction.msg import EndEffectorState
from geometry_msgs.msg import WrenchStamped
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import Bool


def main() -> int:
    rclpy.init()
    node = rclpy.create_node("aubo_i5_readonly_verifier")
    seen = {}
    subscriptions = [
        node.create_subscription(Bool, "/controller_manager/healthy", lambda x: seen.__setitem__("robot_health", x), 10),
        node.create_subscription(EndEffectorState, "/controller_manager/ee_state", lambda x: seen.__setitem__("robot_pose", x), 10),
        node.create_subscription(Bool, "/aubo/kwr75/raw_wrench_valid", lambda x: seen.__setitem__("sensor_health", x), 10),
        node.create_subscription(WrenchStamped, "/aubo/kwr75/raw_wrench", lambda x: seen.__setitem__("sensor_wrench", x), qos_profile_sensor_data),
        node.create_subscription(DiagnosticArray, "/diagnostics", lambda x: seen.__setitem__("diagnostics", x), 10),
    ]
    _ = subscriptions
    try:
        deadline = time.monotonic() + 12.0
        while time.monotonic() < deadline and len(seen) < 5:
            rclpy.spin_once(node, timeout_sec=0.1)
        if "robot_pose" in seen:
            p = seen["robot_pose"].pose.position
            print(f"robot_pose_m={p.x:.6f},{p.y:.6f},{p.z:.6f}")
        if "sensor_wrench" in seen:
            w = seen["sensor_wrench"].wrench
            values = (w.force.x, w.force.y, w.force.z, w.torque.x, w.torque.y, w.torque.z)
            print("sensor_wrench_si=" + ",".join(f"{x:.6f}" for x in values))
            if not all(math.isfinite(x) for x in values):
                print("sensor_values_finite=false")
                return 2
        for key in ("robot_health", "sensor_health"):
            if key in seen:
                print(f"{key}={str(seen[key].data).lower()}")
        if "diagnostics" in seen:
            for status in seen["diagnostics"].status:
                if status.name == "aubo_v4_driver/state":
                    print(f"robot_diagnostic={status.message}")
        expected = {
            "robot_health", "robot_pose", "sensor_health", "sensor_wrench", "diagnostics"
        }
        missing = sorted(expected - seen.keys())
        print("missing_topics=" + (",".join(missing) if missing else "none"))
        return 0 if not missing else 1
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
