#!/usr/bin/env python3
"""Save one read-only, aligned D435 frame for independent pry-buckle checks."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class OneFrameCapture(Node):
    def __init__(self, output: Path) -> None:
        super().__init__("pry_buckle_one_frame_capture")
        self.output = output
        self.bridge = CvBridge()
        self.color: np.ndarray | None = None
        self.depth: np.ndarray | None = None
        self.info: CameraInfo | None = None
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.on_color, qos_profile_sensor_data)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.on_depth, qos_profile_sensor_data)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.on_info, qos_profile_sensor_data)

    def on_color(self, message: Image) -> None:
        self.color = self.bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")

    def on_depth(self, message: Image) -> None:
        depth = self.bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
        self.depth = depth.astype(np.float32) * (1000.0 if message.encoding.lower() == "32fc1" else 1.0)

    def on_info(self, message: CameraInfo) -> None:
        self.info = message

    def ready(self) -> bool:
        return bool(
            self.color is not None and self.depth is not None and self.info is not None
            and self.color.shape[:2] == self.depth.shape[:2]
        )

    def save(self) -> None:
        assert self.color is not None and self.depth is not None and self.info is not None
        self.output.mkdir(parents=True, exist_ok=True)
        cv2.imwrite(str(self.output / "color.png"), self.color)
        np.save(self.output / "aligned_depth_mm.npy", self.depth)
        (self.output / "camera_info.json").write_text(json.dumps({
            "fx": self.info.k[0], "fy": self.info.k[4], "cx": self.info.k[2], "cy": self.info.k[5],
            "width": self.info.width, "height": self.info.height, "frame_id": self.info.header.frame_id,
        }, indent=2), encoding="utf-8")


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--output-dir", type=Path, default=Path("debug/pry_buckle/live_frame"))
    parser.add_argument("--timeout-s", type=float, default=8.0)
    args = parser.parse_args()
    rclpy.init()
    node = OneFrameCapture(args.output_dir)
    try:
        deadline = node.get_clock().now().nanoseconds + int(args.timeout_s * 1e9)
        while rclpy.ok() and node.get_clock().now().nanoseconds < deadline and not node.ready():
            rclpy.spin_once(node, timeout_sec=0.2)
        if not node.ready():
            raise RuntimeError("timed out waiting for matching color, aligned depth, and CameraInfo")
        node.save()
        print(args.output_dir)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
