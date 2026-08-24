#!/usr/bin/env python3
"""检查足跟识别需要的彩色图、对齐深度图和相机参数。"""

from __future__ import annotations

import sys
import time

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image


class CameraCheck(Node):
    def __init__(self) -> None:
        super().__init__("heel_camera_check")
        self.color_count = 0
        self.depth_count = 0
        self.color_size: tuple[int, int] | None = None
        self.depth_size: tuple[int, int] | None = None
        self.info_size: tuple[int, int] | None = None
        self.create_subscription(
            Image,
            "/camera/camera/color/image_raw",
            self.on_color,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            Image,
            "/camera/camera/aligned_depth_to_color/image_raw",
            self.on_depth,
            qos_profile_sensor_data,
        )
        self.create_subscription(
            CameraInfo,
            "/camera/camera/color/camera_info",
            self.on_info,
            qos_profile_sensor_data,
        )

    def on_color(self, message: Image) -> None:
        self.color_count += 1
        self.color_size = (message.width, message.height)

    def on_depth(self, message: Image) -> None:
        self.depth_count += 1
        self.depth_size = (message.width, message.height)

    def on_info(self, message: CameraInfo) -> None:
        self.info_size = (message.width, message.height)


def main() -> int:
    rclpy.init()
    node = CameraCheck()
    deadline = time.monotonic() + 8
    while time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.2)
    ok = (
        node.color_count >= 10
        and node.depth_count >= 10
        and node.info_size is not None
        and node.color_size == node.depth_size == node.info_size
    )
    print(f"彩色画面：{node.color_count} 帧，尺寸 {node.color_size}")
    print(f"对齐深度：{node.depth_count} 帧，尺寸 {node.depth_size}")
    print(f"相机参数：{'已收到' if node.info_size else '未收到'}，尺寸 {node.info_size}")
    print("检查结果：可以进行足跟检测" if ok else "检查结果：尚不满足足跟检测条件")
    node.destroy_node()
    rclpy.shutdown()
    return 0 if ok else 1


if __name__ == "__main__":
    sys.exit(main())
