#!/usr/bin/env python3
"""Reject a traction launch if a second FR SDK owner or Wrench publisher is present."""

import subprocess
import sys


FORBIDDEN_MARKERS = ("fr_robot_driver", "servoj", "ros2_cmd_server")


def run(*args: str) -> str:
    return subprocess.check_output(args, text=True, stderr=subprocess.STDOUT)


def main() -> int:
    try:
        nodes = run("ros2", "node", "list").splitlines()
        topic_info = run(
            "ros2", "topic", "info", "-v",
            "/force_torque_sensor_broadcaster/wrench",
        )
    except (FileNotFoundError, subprocess.CalledProcessError) as exc:
        print(f"traction runtime check could not query ROS2: {exc}", file=sys.stderr)
        return 2

    forbidden = [
        node for node in nodes if any(marker in node for marker in FORBIDDEN_MARKERS)
    ]
    if forbidden:
        message = "forbidden duplicate SDK nodes are running: " + ", ".join(forbidden)
        print(message, file=sys.stderr)
        return 1
    publisher_lines = [line for line in topic_info.splitlines() if "Publisher count:" in line]
    if not publisher_lines or publisher_lines[0].strip() != "Publisher count: 1":
        print("Wrench topic must have exactly one publisher", file=sys.stderr)
        return 1
    print("traction runtime check passed: no duplicate SDK owner and one Wrench publisher")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
