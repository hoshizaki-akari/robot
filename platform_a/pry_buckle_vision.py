"""Live pry-buckle vision worker.

This is deliberately separate from clamp_vision.py.  It subscribes to the
already running D435 ROS topics and performs no robot or gripper operation.
"""
from __future__ import annotations

import sys
import threading
import os
import ctypes
import time
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    from pry_buckle.heel_geometry import HeelGeometryEstimator
    from pry_buckle.horizontal_diameter import CameraIntrinsics
    from pry_buckle.measurement_stabilizer import MeasurementStabilizer
except ImportError:
    from pry_buckle.heel_geometry import HeelGeometryEstimator
    from pry_buckle.horizontal_diameter import CameraIntrinsics
    from pry_buckle.measurement_stabilizer import MeasurementStabilizer


def _load_runtime_dependencies() -> None:
    """Allow the system ROS Python to see the existing vision venv packages."""
    ros_prefix = "/opt/ros/humble"
    ros_site = Path(ros_prefix + "/local/lib/python3.10/dist-packages")
    ros_site_system = Path(ros_prefix + "/lib/python3.10/site-packages")
    venv_site = Path(__file__).resolve().parents[1] / ".venv/lib/python3.10/site-packages"
    for path in (ros_site, ros_site_system, venv_site):
        if path.exists() and str(path) not in sys.path:
            sys.path.insert(0, str(path))
    old_ld = os.environ.get("LD_LIBRARY_PATH", "")
    ld_paths = [ros_prefix + "/lib", ros_prefix + "/local/lib"]
    for p in ld_paths:
        if p not in old_ld.split(":"):
            old_ld = p + ((":" + old_ld) if old_ld else "")
    os.environ["LD_LIBRARY_PATH"] = old_ld
    old_ament = os.environ.get("AMENT_PREFIX_PATH", "")
    if ros_prefix not in old_ament.split(":"):
        os.environ["AMENT_PREFIX_PATH"] = ros_prefix + ((":" + old_ament) if old_ament else "")
    # When the GUI is started directly from the venv, the dynamic loader has
    # not seen ROS' environment setup. Preload ROS shared libraries so rclpy
    # can still subscribe to the already running camera topics.
    ros_root = Path(ros_prefix + "/lib")
    ros_shared = sorted(ros_root.glob("*.so*"))
    # rclpy is imported from the venv process, so the dynamic loader may not
    # re-read LD_LIBRARY_PATH after Python has started. Load the core ROS
    # dependency chain by absolute path first, then retry the complete set.
    priority = [
        ros_root / "librcutils.so",
        ros_root / "librmw.so",
        ros_root / "librcl.so",
        ros_root / "librcl_action.so",
    ]
    for _ in range(5):
        for library in priority + ros_shared:
            try:
                ctypes.CDLL(str(library), mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass


class PryBuckleVisionWorker:
    # The aligned depth is measured on the visible heel surface. The gripper
    # centre is approximately 35 mm farther along the physical optical axis
    # in this setup; keep this as an explicit task parameter, not a hidden
    # change to hand-eye calibration.
    CAMERA_DEPTH_TO_GRIPPER_CENTER_BIAS_MM = 35.0
    def __init__(self, model_path: Path | None = None) -> None:
        self.model_path = model_path or Path(__file__).resolve().parent / "models" / "heel_seg.pt"
        self._lock = threading.Lock()
        self._stop = threading.Event()
        self._thread: threading.Thread | None = None
        self._result: dict[str, Any] = {"valid": False, "message": "撬拨视觉尚未启动"}
        self._frame_png: bytes | None = None

    @property
    def result(self) -> dict[str, Any]:
        with self._lock:
            return dict(self._result)

    @property
    def frame_png(self) -> bytes | None:
        with self._lock:
            return self._frame_png

    def start(self) -> None:
        if self._thread is not None and self._thread.is_alive():
            return
        self._stop.clear()
        self._thread = threading.Thread(target=self._run, name="platform-a-pry-vision", daemon=True)
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        if self._thread is not None:
            # CPU YOLO inference can outlive a single GUI poll interval.  Do
            # not detach that native torch thread and then let the process
            # exit; that was the source of intermittent abort/segfaults when
            # switching branches or reconnecting the service.
            self._thread.join(timeout=15.0)
        if self._thread is not None and self._thread.is_alive():
            with self._lock:
                self._result = {"valid": False, "message": "撬拨视觉正在停止，请稍候"}
            return
        self._thread = None
        with self._lock:
            self._result = {"valid": False, "message": "撬拨视觉已停止"}
            self._frame_png = None

    def _run(self) -> None:
        try:
            _load_runtime_dependencies()
            import rclpy
            from cv_bridge import CvBridge
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CameraInfo, Image
            from ultralytics import YOLO

            try:
                rclpy.init(args=None)
            except Exception:
                # 同一进程里 ROS 上下文已由其它节点（如共同状态服务
                # D435Monitor）初始化过，复用即可，不要重复 init。
                pass
            node = Node("platform_a_pry_buckle_vision")
            bridge = CvBridge()
            detector = YOLO(str(self.model_path))
            estimator = HeelGeometryEstimator()
            stabilizer = MeasurementStabilizer()
            state: dict[str, Any] = {"image": None, "depth": None, "intrinsics": None}
            processed = -1
            frame_id = 0
            last_inference_time = 0.0
            last_valid_result: dict[str, Any] | None = None
            last_valid_overlay: bytes | None = None
            last_valid_at = 0.0

            def on_color(message: Image) -> None:
                nonlocal frame_id
                image = bridge.imgmsg_to_cv2(message, desired_encoding="bgr8")
                state["image"] = image
                frame_id += 1

            def on_depth(message: Image) -> None:
                depth = bridge.imgmsg_to_cv2(message, desired_encoding="passthrough")
                state["depth"] = depth.astype(np.float32) * (1000.0 if message.encoding.lower() == "32fc1" else 1.0)

            def on_info(message: CameraInfo) -> None:
                state["intrinsics"] = CameraIntrinsics(float(message.k[0]), float(message.k[4]), float(message.k[2]), float(message.k[5]))

            node.create_subscription(Image, "/camera/camera/color/image_raw", on_color, qos_profile_sensor_data)
            node.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", on_depth, qos_profile_sensor_data)
            node.create_subscription(CameraInfo, "/camera/camera/color/camera_info", on_info, qos_profile_sensor_data)
            while not self._stop.is_set():
                rclpy.spin_once(node, timeout_sec=0.1)
                image_raw = state["image"]
                if image_raw is None or frame_id == processed:
                    continue
                processed = frame_id
                # Keep the live preview responsive on CPU-only machines. The
                # camera remains live at its native rate; segmentation is a
                # bounded-rate estimator, not a frame-by-frame controller.
                now = time.monotonic()
                if now - last_inference_time < 0.25:
                    continue
                last_inference_time = now
                try:
                    # The D435 image stream is rotated 180 deg.  Rotate the
                    # color frame back before YOLO so the model (trained on
                    # upright images) can detect the heel reliably.
                    image = cv2.rotate(image_raw, cv2.ROTATE_180)
                    depth_raw = state["depth"]
                    depth = None if depth_raw is None else cv2.rotate(depth_raw, cv2.ROTATE_180)
                    intr_raw = state["intrinsics"]
                    intr = None
                    if intr_raw is not None:
                        ih, iw = image.shape[:2]
                        intr = CameraIntrinsics(intr_raw.fx, intr_raw.fy, iw - intr_raw.cx, ih - intr_raw.cy)
                    prediction = detector.predict(source=image, imgsz=640, conf=0.10, retina_masks=True, verbose=False, device="cpu")[0]
                    if prediction.masks is None or prediction.boxes is None:
                        raise RuntimeError("视野内未识别到足跟，请调整足部位置或相机角度后重试")
                    index = int(np.argmax(prediction.boxes.conf.detach().cpu().numpy()))
                    raw_mask = prediction.masks.data[index].detach().cpu().numpy()
                    mask = cv2.resize(raw_mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST) > 0.5
                    result = estimator.estimate(mask, depth, intr)
                    result = stabilizer.update(result, frame_id, now)
                    result.update({
                        "image_width": int(image.shape[1]), "image_height": int(image.shape[0]),
                        "heel_detected": True, "heel_outline_px": [],
                        "heel_center_px": result.get("center_px"),
                        "clamp_contact_a_px": result.get("contact_left_px"),
                        "clamp_contact_b_px": result.get("contact_right_px"),
                        "heel_width_mm": result.get("width_mm"),
                    })
                    center_camera = result.get("center_camera_mm")
                    if center_camera and len(center_camera) == 3:
                        # Image+depth were rotated back and the principal
                        # point adjusted, so the coordinates are already in
                        # the physical (unrotated) optical frame.  Only add
                        # the gripper depth bias along the optical axis.
                        result["clamp_contact_center_camera_mm"] = [
                            round(float(center_camera[0]), 3),
                            round(float(center_camera[1]), 3),
                            round(float(center_camera[2] + self.CAMERA_DEPTH_TO_GRIPPER_CENTER_BIAS_MM), 3),
                        ]
                    top_camera = result.get("upper_midpoint_camera_mm")
                    if top_camera and len(top_camera) == 3:
                        result["heel_upper_midpoint_camera_mm"] = [
                            round(float(top_camera[0]), 3),
                            round(float(top_camera[1]), 3),
                            round(float(top_camera[2] + self.CAMERA_DEPTH_TO_GRIPPER_CENTER_BIAS_MM), 3),
                        ]
                        result["surface_to_upper_midpoint_gap_mm"] = float(result.get("surface_to_upper_midpoint_gap_mm", 0.0))
                    if result.get("motion_grade"):
                        last_valid_result = dict(result)
                    elif last_valid_result is not None and result.get("center_px"):
                        jump = float(np.linalg.norm(
                            np.asarray(result["center_px"], dtype=np.float32)
                            - np.asarray(last_valid_result.get("center_px"), dtype=np.float32)
                        ))
                        if jump <= 30.0:
                            held = dict(last_valid_result)
                            held["valid"] = False
                            held["motion_grade"] = False
                            held["stable_valid"] = False
                            held["display_only"] = True
                            held["measurement_status"] = "held_last_valid_result"
                            held["message"] = "当前帧深度/分割异常，沿用最近有效夹持点"
                            result = held
                    # Segmentation/depth can fail intermittently while the
                    # camera is live. Hold the last geometrically valid
                    # result briefly so the UI and the operator see a stable
                    # target instead of alternating valid/rejected frames.
                    if (
                        not result.get("valid")
                        and last_valid_result is not None
                        and time.monotonic() - last_valid_at <= 3.0
                    ):
                        held = dict(last_valid_result)
                        held["valid"] = False
                        held["motion_grade"] = False
                        held["stable_valid"] = False
                        held["display_only"] = True
                        held["measurement_status"] = "held_last_valid_result"
                        held["message"] = "当前帧短暂无效，保持最近有效夹持点"
                        result = held
                    overlay = estimator.draw_overlay(image, mask, result)
                    if not result.get("valid"):
                        overlay = image.copy()
                    ok, encoded = cv2.imencode(".png", overlay)
                    if not ok:
                        raise RuntimeError("撬拨视觉结果编码失败")
                    if result.get("motion_grade"):
                        last_valid_result = dict(result)
                        last_valid_overlay = encoded.tobytes()
                        last_valid_at = time.monotonic()
                    with self._lock:
                        self._result = result
                        self._frame_png = encoded.tobytes()
                except Exception as error:
                    if (
                        last_valid_result is not None
                        and last_valid_overlay is not None
                        and time.monotonic() - last_valid_at <= 3.0
                    ):
                        held = dict(last_valid_result)
                        held["valid"] = False
                        held["motion_grade"] = False
                        held["stable_valid"] = False
                        held["display_only"] = True
                        held["measurement_status"] = "held_last_valid_result"
                        held["message"] = "当前帧短暂无效，保持最近有效夹持点"
                        with self._lock:
                            self._result = held
                            self._frame_png = last_valid_overlay
                        continue
                    # Keep the latest raw frame visible even when YOLO has no
                    # mask, depth is temporarily invalid, or PNG overlay
                    # drawing fails.  A single bad frame must not stop the
                    # preview stream.
                    fallback = state.get("image")
                    encoded_fallback = None
                    if fallback is not None:
                        preview = fallback.copy()
                        ok, encoded = cv2.imencode(".jpg", preview, [cv2.IMWRITE_JPEG_QUALITY, 85])
                        if ok:
                            encoded_fallback = encoded.tobytes()
                    with self._lock:
                        self._result = {"valid": False, "heel_detected": False, "message": f"撬拨视觉不可用：{error}"}
                        if encoded_fallback is not None:
                            self._frame_png = encoded_fallback
            node.destroy_node()
            rclpy.shutdown()
        except Exception as error:
            with self._lock:
                self._result = {"valid": False, "heel_detected": False, "message": f"撬拨视觉线程启动失败：{error}"}
