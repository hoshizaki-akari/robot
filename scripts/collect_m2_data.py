#!/usr/bin/env python3
"""M2: collect RGB, raw/median depth, CameraInfo and live eye-in-hand TF.

The collector subscribes to the already-running RealSense ROS topics.  It
never starts a librealsense pipeline and never commands robot motion.
"""

from __future__ import annotations

import argparse
import json
import sys
import time
import warnings
from datetime import datetime, timezone
from pathlib import Path
from typing import Any
from urllib.request import urlopen

import cv2
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(PROJECT))
from platform_a.handeye_calibration import pose_to_matrix  # noqa: E402

STATE_URL = "http://127.0.0.1:8765/api/state"
HANDEYE_FILE = PROJECT / "platform_a" / "config" / "handeye_calibration.json"
COLOR_TOPIC = "/camera/camera/color/image_raw"
DEPTH_TOPIC = "/camera/camera/aligned_depth_to_color/image_raw"
INFO_TOPIC = "/camera/camera/color/camera_info"


def utc_now() -> str:
    return datetime.now(timezone.utc).isoformat(timespec="milliseconds")


def stamp_to_ns(stamp: Any) -> int:
    return int(stamp.sec) * 1_000_000_000 + int(stamp.nanosec)


def read_state() -> dict[str, Any]:
    with urlopen(STATE_URL, timeout=3) as response:
        return json.load(response)


def stable_robot_state(timeout_s: float = 3.0) -> dict[str, Any]:
    deadline = time.monotonic() + timeout_s
    previous: np.ndarray | None = None
    last: dict[str, Any] | None = None
    while time.monotonic() < deadline:
        last = read_state()
        fr5 = last.get("fr5") or {}
        errors = fr5.get("errors") or {}
        if not fr5.get("valid") or not fr5.get("connected"):
            raise RuntimeError("FR5 状态不可用")
        if int(errors.get("main") or 0) or int(errors.get("sub") or 0) or int(fr5.get("emergency_stop") or 0):
            raise RuntimeError(f"FR5 有报警或急停：{errors}")
        if any(int(value or 0) for value in (fr5.get("safety_stop") or [])):
            raise RuntimeError("FR5 处于安全停止")
        if int(fr5.get("motion_done") or 0) != 1:
            previous = None
            time.sleep(0.1)
            continue
        joints = np.asarray(fr5.get("joint_position_deg") or [], dtype=np.float64)
        tcp = np.asarray(fr5.get("flange_pose_mm_deg") or [], dtype=np.float64)
        if joints.size != 6 or tcp.size != 6:
            raise RuntimeError("FR5 状态长度异常")
        if previous is not None and float(np.max(np.abs(joints - previous))) < 0.05:
            return last
        previous = joints
        time.sleep(0.1)
    raise TimeoutError(f"FR5 在 {timeout_s:.1f}s 内没有稳定停止：{last}")


def live_tf_record(snapshot: dict[str, Any], ros_stamp_ns: int) -> dict[str, Any]:
    calibration = json.loads(HANDEYE_FILE.read_text(encoding="utf-8"))
    if not calibration.get("validated"):
        raise RuntimeError("手眼标定文件未标记 validated，拒绝记录 Base→Camera TF")
    flange_pose = (snapshot.get("fr5") or {}).get("flange_pose_mm_deg") or []
    if len(flange_pose) != 6:
        raise RuntimeError("没有实时 FR5 法兰位姿")
    base_t_flange = pose_to_matrix([float(value) for value in flange_pose], calibration.get("euler_convention", "rpy"))
    flange_t_camera = np.asarray(calibration["flange_T_camera"], dtype=np.float64)
    base_t_camera = base_t_flange @ flange_t_camera
    fr5 = snapshot["fr5"]
    return {
        "ros_image_stamp_ns": int(ros_stamp_ns),
        "state_service_timestamp": snapshot.get("timestamp"),
        "captured_at": utc_now(),
        "base_frame": "base",
        "flange_frame": "flange",
        "camera_frame": "camera_color_optical_frame",
        "source": "live_fr5_state_service_and_validated_handeye",
        "base_T_flange": base_t_flange.tolist(),
        "flange_T_camera": flange_t_camera.tolist(),
        "base_T_camera": base_t_camera.tolist(),
        "joint_positions_deg": [float(value) for value in fr5["joint_position_deg"]],
        "flange_pose_mm_deg": [float(value) for value in fr5["flange_pose_mm_deg"]],
        "tcp_pose_mm_deg": [float(value) for value in fr5.get("tcp_pose_mm_deg") or []],
    }


def image_to_bgr(message: Any) -> np.ndarray:
    height, width, step = int(message.height), int(message.width), int(message.step)
    raw = np.frombuffer(bytes(message.data), dtype=np.uint8).reshape(height, step)
    encoding = str(message.encoding).lower()
    if encoding in ("rgb8", "bgr8"):
        image = raw[:, : width * 3].reshape(height, width, 3).copy()
        return cv2.cvtColor(image, cv2.COLOR_RGB2BGR) if encoding == "rgb8" else image
    if encoding in ("rgba8", "bgra8"):
        image = raw[:, : width * 4].reshape(height, width, 4).copy()
        return cv2.cvtColor(image, cv2.COLOR_RGBA2BGR if encoding == "rgba8" else cv2.COLOR_BGRA2BGR)
    raise ValueError(f"不支持 RGB encoding：{message.encoding}")


def depth_to_array(message: Any) -> np.ndarray:
    height, width, step = int(message.height), int(message.width), int(message.step)
    encoding = str(message.encoding).lower()
    if encoding == "16uc1":
        return np.frombuffer(bytes(message.data), dtype=np.uint16).reshape(height, step // 2)[:, :width].copy()
    if encoding == "32fc1":
        return np.frombuffer(bytes(message.data), dtype=np.float32).reshape(height, step // 4)[:, :width].copy()
    raise ValueError(f"不支持 Depth encoding：{message.encoding}")


def camera_info_dict(message: Any) -> dict[str, Any]:
    return {
        "header": {"stamp_ns": stamp_to_ns(message.header.stamp), "frame_id": message.header.frame_id},
        "height": int(message.height),
        "width": int(message.width),
        "distortion_model": message.distortion_model,
        "d": [float(value) for value in message.d],
        "k": [float(value) for value in message.k],
        "r": [float(value) for value in message.r],
        "p": [float(value) for value in message.p],
        "binning_x": int(message.binning_x),
        "binning_y": int(message.binning_y),
        "roi": {
            "x_offset": int(message.roi.x_offset), "y_offset": int(message.roi.y_offset),
            "height": int(message.roi.height), "width": int(message.roi.width),
            "do_rectify": bool(message.roi.do_rectify),
        },
    }


class Collector:
    def __init__(self) -> None:
        import rclpy
        from rclpy.node import Node
        from rclpy.qos import qos_profile_sensor_data
        from sensor_msgs.msg import CameraInfo, Image

        rclpy.init(args=None)
        self.node = Node("m2_d435_collector")
        self.color: tuple[int, np.ndarray, str] | None = None
        self.depth: tuple[int, np.ndarray, str] | None = None
        self.info: Any | None = None
        self.node.create_subscription(Image, COLOR_TOPIC, self.on_color, qos_profile_sensor_data)
        self.node.create_subscription(Image, DEPTH_TOPIC, self.on_depth, qos_profile_sensor_data)
        self.node.create_subscription(CameraInfo, INFO_TOPIC, self.on_info, qos_profile_sensor_data)
        self.rclpy = rclpy

    def on_color(self, message: Any) -> None:
        self.color = (stamp_to_ns(message.header.stamp), image_to_bgr(message), str(message.encoding))

    def on_depth(self, message: Any) -> None:
        self.depth = (stamp_to_ns(message.header.stamp), depth_to_array(message), str(message.encoding))

    def on_info(self, message: Any) -> None:
        self.info = message

    def close(self) -> None:
        self.node.destroy_node()
        self.rclpy.shutdown()

    def collect(self, frames: int, settle_s: float) -> tuple[list[tuple[int, np.ndarray, str]], list[tuple[int, np.ndarray, str]], Any, list[dict[str, Any]]]:
        stable_robot_state()
        deadline = time.monotonic() + settle_s
        while time.monotonic() < deadline:
            self.rclpy.spin_once(self.node, timeout_sec=0.1)
            stable_robot_state(timeout_s=0.5)
        colors: list[tuple[int, np.ndarray, str]] = []
        depths: list[tuple[int, np.ndarray, str]] = []
        tf_samples: list[dict[str, Any]] = []
        last_pair: tuple[int, int] | None = None
        timeout = time.monotonic() + max(20.0, frames * 2.0)
        while len(colors) < frames and time.monotonic() < timeout:
            self.rclpy.spin_once(self.node, timeout_sec=0.2)
            if self.color is None or self.depth is None or self.info is None:
                continue
            pair = (self.color[0], self.depth[0])
            if pair == last_pair:
                continue
            last_pair = pair
            snapshot = stable_robot_state(timeout_s=1.0)
            colors.append((self.color[0], self.color[1].copy(), self.color[2]))
            depths.append((self.depth[0], self.depth[1].copy(), self.depth[2]))
            tf_samples.append(live_tf_record(snapshot, self.color[0]))
            print(f"captured {len(colors)}/{frames}: rgb={self.color[1].shape} depth={self.depth[1].shape} encoding={self.depth[2]}")
        if len(colors) < frames:
            raise TimeoutError(f"只采集到 {len(colors)}/{frames} 帧")
        return colors, depths, self.info, tf_samples


def save_observation(output: Path, name: str, colors: list[tuple[int, np.ndarray, str]], depths: list[tuple[int, np.ndarray, str]], info: Any, tf_samples: list[dict[str, Any]]) -> Path:
    observation = output / name
    (observation / "rgb_frames").mkdir(parents=True, exist_ok=True)
    (observation / "depth_raw_frames").mkdir(parents=True, exist_ok=True)
    for index, (_, image, _) in enumerate(colors):
        if not cv2.imwrite(str(observation / "rgb_frames" / f"frame_{index:03d}.png"), image):
            raise IOError("RGB PNG 保存失败")
    for index, (_, depth, _) in enumerate(depths):
        np.save(observation / "depth_raw_frames" / f"frame_{index:03d}.npy", depth, allow_pickle=False)
    raw = np.stack([depth for _, depth, _ in depths])
    valid = raw > 0
    values = np.where(valid, raw.astype(np.float32), np.nan)
    with warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)
        median = np.nanmedian(values, axis=0).astype(np.float32)
    median[np.all(~valid, axis=0)] = 0.0
    np.save(observation / "depth_raw_stack.npy", raw, allow_pickle=False)
    np.save(observation / "depth_median.npy", median, allow_pickle=False)
    median_png = np.nan_to_num(median, nan=0.0).clip(0, 65535).astype(np.uint16)
    cv2.imwrite(str(observation / "depth_median.png"), median_png)
    cv2.imwrite(str(observation / "rgb.png"), colors[len(colors) // 2][1])
    info_dict = camera_info_dict(info)
    (observation / "camerainfo.json").write_text(json.dumps(info_dict, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    (observation / "tf_samples.json").write_text(json.dumps(tf_samples, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    manifest = {
        "observation": name, "created_at": utc_now(), "frame_count": len(colors),
        "rgb_topic": COLOR_TOPIC, "depth_topic": DEPTH_TOPIC, "camera_info_topic": INFO_TOPIC,
        "rgb_shape": list(colors[0][1].shape), "depth_shape": list(depths[0][1].shape),
        "rgb_encoding": colors[0][2], "depth_encoding": depths[0][2],
        "depth_median_rule": "pixelwise median of depth values > 0; all-invalid pixels are 0",
        "raw_depth_files": "depth_raw_frames/frame_*.npy and depth_raw_stack.npy",
        "tf_file": "tf_samples.json; one live Base->Camera transform per captured RGB frame",
        "camera_info_file": "camerainfo.json",
        "image_frame_id": info.header.frame_id,
    }
    (observation / "manifest.json").write_text(json.dumps(manifest, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return observation


def main() -> int:
    parser = argparse.ArgumentParser(description="M2 真实 D435 数据采集（不运动机器人）")
    parser.add_argument("name", choices=("wire_a", "wire_b", "sole"))
    parser.add_argument("--frames", type=int, default=15)
    parser.add_argument("--settle-s", type=float, default=1.0)
    parser.add_argument("--output", type=Path, default=PROJECT / "data")
    parser.add_argument("--session", type=Path, default=None, help="已有 session 目录；不传则新建")
    args = parser.parse_args()
    if args.frames < 1:
        raise SystemExit("--frames 必须大于0")
    session = args.session or (args.output / f"session_{datetime.now().astimezone().strftime('%Y%m%d_%H%M%S')}" )
    collector = Collector()
    try:
        colors, depths, info, tf_samples = collector.collect(args.frames, args.settle_s)
        path = save_observation(session, args.name, colors, depths, info, tf_samples)
        print(f"saved: {path}")
        return 0
    finally:
        collector.close()


if __name__ == "__main__":
    raise SystemExit(main())
