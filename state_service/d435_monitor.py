from __future__ import annotations

import glob
import math
import os
import struct
import threading
import zlib
from collections import deque
from datetime import datetime, timezone
from time import monotonic
from pathlib import Path
from typing import Any

import cv2
import numpy as np

try:
    from pry_buckle.horizontal_diameter import CameraIntrinsics, HorizontalDiameterEstimator
except ImportError:  # 允许脚本直接以模块方式运行
    from horizontal_diameter import CameraIntrinsics, HorizontalDiameterEstimator

try:
    from pry_buckle.depth_guided_heel import (
        build_target_display_mask,
        estimate_depth_guided_target_chord,
        extract_depth_heel_candidate,
    )
except ImportError:  # 允许脚本直接以模块方式运行
    from depth_guided_heel import (
        build_target_display_mask,
        estimate_depth_guided_target_chord,
        extract_depth_heel_candidate,
    )


class D435Monitor:
    """可选 ROS 2 图像订阅器；没有启动相机节点时返回明确的无效状态。"""

    def __init__(self) -> None:
        self._lock = threading.Lock()
        self._color_times: deque[float] = deque(maxlen=90)
        self._depth_times: deque[float] = deque(maxlen=90)
        self._color_stamp: str | None = None
        self._depth_stamp: str | None = None
        self._error = ""
        self._color_png: bytes | None = None
        self._color_jpeg: bytes | None = None
        self._color_jpeg_sequence = 0
        self._annotated_png: bytes | None = None
        self._latest_color: np.ndarray | None = None
        self._aligned_depth_mm: np.ndarray | None = None
        self._intrinsics: CameraIntrinsics | None = None
        self._distortion_coefficients: list[float] = []
        self._vision_result: dict[str, Any] = {
            "valid": False,
            "motion_allowed": False,
            "message": "夹挤识别正在启动",
        }
        self._vision_frame_id = 0
        self._vision_processed_id = -1
        self._vision_result_at = 0.0
        self._last_color_encode_at = 0.0
        self._last_color_jpeg_encode_at = 0.0
        self._thread: threading.Thread | None = None
        self._vision_thread: threading.Thread | None = None
        # USB 链路自诊断缓存：D435 必须接在 USB 3.0 才能稳定传输深度流。
        self._usb_speed_mbps: str | None = None
        self._usb_speed_at = 0.0

    def start(self) -> None:
        if self._thread is not None:
            return
        self._thread = threading.Thread(target=self._run, name="d435-monitor", daemon=True)
        self._thread.start()
        self._vision_thread = threading.Thread(
            target=self._run_clamp_vision, name="heel-clamp-vision", daemon=True
        )
        self._vision_thread.start()

    @staticmethod
    def _fps(values: deque[float]) -> float:
        if len(values) < 2:
            return 0.0
        elapsed = values[-1] - values[0]
        return round((len(values) - 1) / elapsed, 2) if elapsed > 0 else 0.0

    @staticmethod
    def _read_sys(path: str) -> str:
        try:
            with open(path, "r") as handle:
                return handle.read().strip()
        except OSError:
            return ""

    def _d435_usb_speed_mbps(self) -> str | None:
        """读取 D435 当前 USB 协商速度（Mbps）。

        480 = USB 2.0（带宽不足以稳定传输深度流，会超时/丢帧）；
        5000 = USB 3.0 / SuperSpeed（正常）。结果每 10 秒刷新一次。
        """
        now = monotonic()
        if self._usb_speed_mbps is not None and now - self._usb_speed_at < 10.0:
            return self._usb_speed_mbps
        speed: str | None = None
        try:
            base = "/sys/bus/usb/devices"
            for dev in os.listdir(base):
                if self._read_sys(os.path.join(base, dev, "idVendor")) != "8086":
                    continue
                if self._read_sys(os.path.join(base, dev, "idProduct")) != "0b07":
                    continue
                raw = self._read_sys(os.path.join(base, dev, "speed"))
                if raw:
                    speed = raw
                break
        except OSError:
            speed = None
        with self._lock:
            self._usb_speed_mbps = speed
            self._usb_speed_at = now
        return speed

    def snapshot(self) -> dict[str, Any]:
        devices = sorted(glob.glob("/dev/video*"))
        usb_speed = self._d435_usb_speed_mbps()
        with self._lock:
            color_fps = self._fps(self._color_times)
            depth_fps = self._fps(self._depth_times)
            now = monotonic()
            color_receiving = bool(self._color_times) and now - self._color_times[-1] < 2.0
            depth_receiving = bool(self._depth_times) and now - self._depth_times[-1] < 2.0
            # 足跟定位同时依赖彩色图和深度图，缺少任意一路都不能算可用。
            receiving = color_receiving and depth_receiving
            # D435 在 USB 2.0（480M）下带宽不足，深度流必然超时/丢帧。
            usb_too_slow = usb_speed in ("480", "12", "1.5")
            if usb_too_slow:
                diag = (
                    "D435 当前连接在 USB 2.0 端口（协商速率 "
                    f"{usb_speed}Mbps），带宽不足以稳定传输深度流，导致设备无效、无画面。"
                    "请将相机换到主机的 USB 3.0（SuperSpeed，蓝色 SS 标识）端口，并改用 USB 3.0 数据线。"
                )
            else:
                diag = ""
            return {
                "connected": bool(devices),
                "valid": receiving,
                "color_valid": color_receiving,
                "depth_valid": depth_receiving,
                "usb_speed_mbps": usb_speed,
                "color_fps": color_fps,
                "depth_fps": depth_fps,
                "last_color_frame": self._color_stamp,
                "last_depth_frame": self._depth_stamp,
                "video_devices": devices,
                "color_intrinsics": (
                    {
                        "fx": self._intrinsics.fx,
                        "fy": self._intrinsics.fy,
                        "cx": self._intrinsics.cx,
                        "cy": self._intrinsics.cy,
                    }
                    if self._intrinsics is not None
                    else None
                ),
                "distortion_coefficients": list(self._distortion_coefficients),
                "message": (
                    "正在接收 D435 彩色和深度图像"
                    if receiving
                    else diag
                    or self._error
                    or (
                        "D435 彩色画面中断"
                        if not color_receiving and depth_receiving
                        else "D435 深度画面中断"
                        if color_receiving and not depth_receiving
                        else "发现 D435，但尚未收到彩色和深度画面"
                    )
                    if devices
                    else "WSL 中未发现 D435 视频设备"
                ),
                "clamp_vision": dict(self._vision_result),
            }

    def _record(self, target: deque[float], stamp_name: str) -> None:
        now_text = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
        with self._lock:
            target.append(monotonic())
            setattr(self, stamp_name, now_text)

    @staticmethod
    def _png_chunk(chunk_type: bytes, data: bytes) -> bytes:
        return (
            struct.pack(">I", len(data))
            + chunk_type
            + data
            + struct.pack(">I", zlib.crc32(chunk_type + data) & 0xFFFFFFFF)
        )

    @classmethod
    def _image_to_png(cls, message: Any) -> bytes:
        width = int(message.width)
        height = int(message.height)
        step = int(message.step)
        encoding = str(message.encoding).lower()
        source = bytes(message.data)
        rows: list[bytes] = []
        for row_index in range(height):
            row = source[row_index * step : (row_index + 1) * step]
            if encoding == "rgb8":
                rgb = row[: width * 3]
            elif encoding == "bgr8":
                pixels = row[: width * 3]
                converted = bytearray(len(pixels))
                converted[0::3] = pixels[2::3]
                converted[1::3] = pixels[1::3]
                converted[2::3] = pixels[0::3]
                rgb = bytes(converted)
            elif encoding in ("rgba8", "bgra8"):
                pixels = row[: width * 4]
                converted = bytearray(width * 3)
                if encoding == "rgba8":
                    converted[0::3] = pixels[0::4]
                    converted[1::3] = pixels[1::4]
                    converted[2::3] = pixels[2::4]
                else:
                    converted[0::3] = pixels[2::4]
                    converted[1::3] = pixels[1::4]
                    converted[2::3] = pixels[0::4]
                rgb = bytes(converted)
            else:
                raise ValueError(f"暂不支持 D435 彩色格式：{message.encoding}")
            rows.append(b"\x00" + rgb)
        header = struct.pack(">IIBBBBB", width, height, 8, 2, 0, 0, 0)
        return (
            b"\x89PNG\r\n\x1a\n"
            + cls._png_chunk(b"IHDR", header)
            + cls._png_chunk(b"IDAT", zlib.compress(b"".join(rows), 3))
            + cls._png_chunk(b"IEND", b"")
        )

    @staticmethod
    def _image_to_bgr(message: Any) -> np.ndarray:
        height = int(message.height)
        width = int(message.width)
        step = int(message.step)
        encoding = str(message.encoding).lower()
        raw = np.frombuffer(message.data, dtype=np.uint8).reshape(height, step)
        if encoding in ("bgr8", "rgb8"):
            image = raw[:, : width * 3].reshape(height, width, 3).copy()
            return image if encoding == "bgr8" else cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
        if encoding in ("bgra8", "rgba8"):
            image = raw[:, : width * 4].reshape(height, width, 4).copy()
            code = cv2.COLOR_BGRA2BGR if encoding == "bgra8" else cv2.COLOR_RGBA2BGR
            return cv2.cvtColor(image, code)
        raise ValueError(f"暂不支持 D435 彩色格式：{message.encoding}")

    def _record_color(self, message: Any) -> None:
        self._record(self._color_times, "_color_stamp")
        try:
            image = self._image_to_bgr(message)
            with self._lock:
                self._latest_color = image
                self._vision_frame_id += 1
        except Exception as error:
            with self._lock:
                self._error = f"D435 彩色画面读取失败：{error}"
            return
        now = monotonic()
        with self._lock:
            if now - self._last_color_encode_at < 0.1:
                return
            self._last_color_encode_at = now
        try:
            png = self._image_to_png(message)
            with self._lock:
                self._color_png = png
        except Exception as error:
            with self._lock:
                self._error = f"D435 彩色画面转换失败：{error}"
            # 兜底：用 OpenCV 现场编码，避免画面缓存为空导致黑屏
            try:
                ok, encoded = cv2.imencode(".png", image)
                if ok:
                    with self._lock:
                        self._color_png = encoded.tobytes()
            except Exception:
                pass

        # UI 实时预览使用 JPEG/MJPEG，避免每次刷新都重新下载与解码 PNG。
        # 这是显示层编码，不参与 HeelClampVision 的输入和夹持点计算。
        with self._lock:
            if now - self._last_color_jpeg_encode_at < 0.08:
                return
            self._last_color_jpeg_encode_at = now
        try:
            encoded_ok, encoded = cv2.imencode(
                ".jpg", image, [cv2.IMWRITE_JPEG_QUALITY, 82]
            )
            if not encoded_ok:
                raise ValueError("JPEG 编码失败")
            with self._lock:
                self._color_jpeg = encoded.tobytes()
                self._color_jpeg_sequence += 1
        except Exception as error:
            with self._lock:
                self._error = f"D435 彩色预览编码失败：{error}"

    def color_png(self) -> bytes | None:
        with self._lock:
            if not self._color_times or monotonic() - self._color_times[-1] >= 2.0:
                return None
            return self._color_png

    def color_jpeg_snapshot(self) -> tuple[int, bytes] | None:
        """返回最新预览帧；供 HTTP MJPEG 长连接使用。"""
        with self._lock:
            if not self._color_times or monotonic() - self._color_times[-1] >= 2.0:
                return None
            if self._color_jpeg is None:
                return None
            return self._color_jpeg_sequence, self._color_jpeg

    def annotated_png(self) -> bytes | None:
        with self._lock:
            fresh = bool(self._color_times) and monotonic() - self._color_times[-1] < 2.0
            if not fresh:
                return None
            return self._annotated_png or self._color_png

    def vision_snapshot(self) -> dict[str, Any]:
        with self._lock:
            now = monotonic()
            color_fresh = bool(self._color_times) and now - self._color_times[-1] < 2.0
            result_fresh = self._vision_result_at > 0 and now - self._vision_result_at < 2.0
            if not color_fresh or not result_fresh:
                return {
                    "valid": False,
                    "heel_detected": False,
                    "puncture_detected": False,
                    "motion_allowed": False,
                    "message": "相机画面刚刚中断或重启，旧的夹挤位置已经作废",
                }
            return dict(self._vision_result)

    def _record_aligned_depth(self, message: Any) -> None:
        self._record(self._depth_times, "_depth_stamp")
        try:
            height = int(message.height)
            width = int(message.width)
            step = int(message.step)
            encoding = str(message.encoding).lower()
            if encoding == "16uc1":
                row_values = step // 2
                depth = np.frombuffer(message.data, dtype=np.uint16).reshape(height, row_values)
                depth_mm = depth[:, :width].astype(np.float32)
            elif encoding == "32fc1":
                row_values = step // 4
                depth = np.frombuffer(message.data, dtype=np.float32).reshape(height, row_values)
                depth_mm = depth[:, :width].copy() * 1000.0
            else:
                raise ValueError(f"暂不支持深度格式：{message.encoding}")
            with self._lock:
                self._aligned_depth_mm = depth_mm
        except Exception as error:
            with self._lock:
                self._error = f"D435 深度画面读取失败：{error}"

    def _record_camera_info(self, message: Any) -> None:
        matrix = list(message.k)
        if len(matrix) != 9:
            return
        with self._lock:
            self._intrinsics = CameraIntrinsics(
                fx=float(matrix[0]), fy=float(matrix[4]),
                cx=float(matrix[2]), cy=float(matrix[5]),
            )
            self._distortion_coefficients = [float(value) for value in message.d]

    def _run_clamp_vision(self) -> None:
        """Extract a validated current-view heel candidate.

        The camera launch already enables the RealSense rotation filter.  The
        previous implementation rotated the color/depth pair a second time,
        then ran a small full-frame inference.  The current view contains a
        much smaller heel than the model's training crops, so a low-confidence
        background box could be mistaken for the target while the real heel
        was rejected.  We now require a usable mask and geometry; if YOLO does
        not provide one, a coherent near-depth component is used as a
        display-only candidate and is still checked by the same plane/width
        estimator.
        """
        default_model = Path(__file__).resolve().parents[1] / "platform_a" / "models" / "heel_seg.pt"
        model_path = Path(os.environ.get("PLATFORM_A_HEEL_MODEL", str(default_model)))
        vision_threads = max(1, int(os.environ.get("FR5_VISION_TORCH_THREADS", "1")))
        os.environ.setdefault("OMP_NUM_THREADS", str(vision_threads))
        os.environ.setdefault("MKL_NUM_THREADS", str(vision_threads))
        os.environ.setdefault("OPENBLAS_NUM_THREADS", str(vision_threads))
        from ultralytics import YOLO

        try:
            import torch

            torch.set_num_threads(vision_threads)
            try:
                torch.set_num_interop_threads(vision_threads)
            except RuntimeError:
                # Another already-running torch component may have fixed the
                # inter-op pool; the intra-op limit above is still effective.
                pass
        except (ImportError, ValueError, RuntimeError):
            pass
        detector = YOLO(str(model_path))
        estimator = HorizontalDiameterEstimator()
        last_inference_time = 0.0
        last_yolo_inference_at = 0.0
        inference_period_s = float(os.environ.get("FR5_VISION_INFERENCE_PERIOD_S", "0.75"))
        yolo_check_period_s = float(
            os.environ.get("FR5_VISION_YOLO_CHECK_PERIOD_S", "5.0")
        )
        # 夹持中心比可见足跟表面沿光轴远约 35mm，与 PryBuckleVisionWorker 一致。
        GRIPPER_BIAS_MM = 35.0
        while True:
            with self._lock:
                frame_id = self._vision_frame_id
                image = None if self._latest_color is None else self._latest_color.copy()
                depth = None if self._aligned_depth_mm is None else self._aligned_depth_mm.copy()
                intrinsics = self._intrinsics
            if image is None or frame_id == self._vision_processed_id:
                threading.Event().wait(0.05)
                continue
            self._vision_processed_id = frame_id
            now = monotonic()
            if now - last_inference_time < inference_period_s:
                threading.Event().wait(0.05)
                continue
            last_inference_time = now
            try:
                # rotation_filter.rotation:=180.0 is already applied by the
                # camera node.  Keep RGB, aligned depth and intrinsics in the
                # same published orientation; do not rotate a second time.
                img = image
                dep = depth
                ih, iw = img.shape[:2]
                intr = intrinsics
                result: dict[str, Any] | None = None
                detection_method = "yolo"
                yolo_confidence: float | None = None
                selected_mask: np.ndarray | None = None
                depth_candidate_result: dict[str, Any] | None = None
                depth_mask: np.ndarray | None = None

                # Reject tiny/background YOLO masks before geometry.  This is
                # deliberately stricter than the model confidence threshold:
                # a 7x8 px shelf fragment is not a heel candidate.
                if now - last_yolo_inference_at >= yolo_check_period_s:
                    last_yolo_inference_at = now
                    prediction = detector.predict(
                        source=img, imgsz=640, conf=0.10,
                        retina_masks=True, verbose=False, device="cpu",
                    )[0]
                    if prediction.masks is not None and prediction.boxes is not None:
                        confidences = prediction.boxes.conf.detach().cpu().numpy()
                        for index in np.argsort(confidences)[::-1]:
                            raw_mask = prediction.masks.data[int(index)].detach().cpu().numpy()
                            mask = cv2.resize(
                                raw_mask, (iw, ih), interpolation=cv2.INTER_NEAREST
                            ) > 0.5
                            if int(np.count_nonzero(mask)) < 300:
                                continue
                            candidate = estimator.estimate(mask, dep, intr)
                            if candidate.get("valid"):
                                result = candidate
                                selected_mask = mask
                                yolo_confidence = float(confidences[int(index)])
                                break

                # Current-view depth consensus: when the aligned depth image
                # can identify the single central near object, prefer its
                # fixed upper-heel chord even if YOLO also returned a valid
                # but different mask.  This prevents a low-quality YOLO
                # fragment from moving the contacts between unrelated rows.
                depth_diag: dict[str, Any] = {}
                if dep is not None and intr is not None:
                    depth_mask, depth_diag = extract_depth_heel_candidate(dep)
                    if depth_mask is not None:
                        candidate = estimate_depth_guided_target_chord(
                            depth_mask, dep, intr, estimator
                        )
                        depth_diag.update(
                            {
                                "depth_candidate_width_mm": candidate.get("width_mm"),
                                "depth_candidate_valid": bool(candidate.get("valid")),
                                "depth_candidate_message": candidate.get("message"),
                            }
                        )
                        if candidate.get("valid"):
                            result = candidate
                            selected_mask = build_target_display_mask(
                                depth_mask,
                                candidate.get("target_circle_center_px")
                                or candidate.get("center_px"),
                                candidate.get("target_circle_radius_px", 10),
                            )
                            detection_method = (
                                "depth_guided_consensus"
                                if yolo_confidence is not None
                                else "depth_guided_fallback"
                            )
                        else:
                            depth_candidate_result = candidate

                if result is None:
                    if depth_candidate_result is not None:
                        result = depth_candidate_result
                        detection_method = "depth_guided_fallback"
                        selected_mask = build_target_display_mask(
                            depth_mask,
                            result.get("target_circle_center_px")
                            or result.get("center_px"),
                            result.get("target_circle_radius_px", 10),
                        )
                    else:
                        raise RuntimeError(
                            "视野内未识别到满足几何约束的足跟："
                            + str(depth_diag.get("message", "YOLO 与深度候选均失败"))
                        )

                result.update({
                    "image_width": int(iw), "image_height": int(ih),
                    "heel_detected": True, "heel_outline_px": [],
                    "heel_center_px": result.get("center_px"),
                    "clamp_contact_a_px": result.get("contact_left_px"),
                    "clamp_contact_b_px": result.get("contact_right_px"),
                    "heel_width_mm": result.get("width_mm"),
                    "detection_method": detection_method,
                    "yolo_confidence": yolo_confidence,
                    **depth_diag,
                    # clamp_planner 期望的字段名
                    "heel_center_camera_mm": result.get("center_camera_mm"),
                    "clamp_contact_a_camera_mm": result.get("contact_left_camera_mm"),
                    "clamp_contact_b_camera_mm": result.get("contact_right_camera_mm"),
                })
                center_camera = result.get("center_camera_mm")
                if center_camera and len(center_camera) == 3:
                    result["clamp_contact_center_camera_mm"] = [
                        round(float(center_camera[0]), 3),
                        round(float(center_camera[1]), 3),
                        round(float(center_camera[2] + GRIPPER_BIAS_MM), 3),
                    ]
                top_camera = result.get("upper_midpoint_camera_mm")
                if top_camera and len(top_camera) == 3:
                    result["heel_upper_midpoint_camera_mm"] = [
                        round(float(top_camera[0]), 3),
                        round(float(top_camera[1]), 3),
                        round(float(top_camera[2] + GRIPPER_BIAS_MM), 3),
                    ]
                    result["surface_to_upper_midpoint_gap_mm"] = float(
                        result.get("surface_to_upper_midpoint_gap_mm", 0.0)
                    )
                # A depth fallback is useful for the current-view display but
                # must not be promoted to robot-motion grade without a later
                # human checkpoint and multi-frame confirmation.
                if detection_method == "depth_guided_fallback":
                    result["motion_grade"] = False
                    result["motion_allowed"] = False
                    result["display_only"] = True
                    result["message"] = (
                        "深度引导足跟候选：已得到夹持点与宽度，等待人工确认"
                        if result.get("valid")
                        else "深度候选已找到，但夹持宽度/深度几何未通过"
                    )
                overlay = estimator.draw_overlay(
                    img,
                    selected_mask if selected_mask is not None else np.zeros((ih, iw), dtype=bool),
                    result,
                )
                ok, encoded = cv2.imencode(".png", overlay)
                if not ok:
                    raise ValueError("识别画面编码失败")
                with self._lock:
                    self._vision_result = result
                    self._annotated_png = encoded.tobytes()
                    self._vision_result_at = monotonic()
            except Exception as error:
                with self._lock:
                    self._vision_result = {
                        "valid": False,
                        "motion_allowed": False,
                        "message": f"夹挤识别不可用：{error}",
                    }
                    self._vision_result_at = monotonic()
            threading.Event().wait(0.05)

    def _run(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CameraInfo, Image

            try:
                rclpy.init(args=None)
            except RuntimeError:
                # 同一进程内 PryBuckleVisionWorker 已初始化过 ROS 上下文，复用即可。
                pass
            node = Node("fr5_platform_d435_monitor")
            node.create_subscription(
                Image,
                "/camera/camera/color/image_raw",
                self._record_color,
                qos_profile_sensor_data,
            )
            node.create_subscription(
                Image,
                "/camera/camera/aligned_depth_to_color/image_raw",
                self._record_aligned_depth,
                qos_profile_sensor_data,
            )
            node.create_subscription(
                CameraInfo,
                "/camera/camera/color/camera_info",
                self._record_camera_info,
                qos_profile_sensor_data,
            )
            rclpy.spin(node)
        except Exception as error:
            with self._lock:
                self._error = f"ROS 2 图像监测未启动：{error}"
