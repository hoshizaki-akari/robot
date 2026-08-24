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

from platform_a.clamp_vision import CameraIntrinsics, HeelClampVision


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
        self._puncture_history: deque[tuple[list[float], list[int]]] = deque(maxlen=7)
        self._stable_puncture_3d: list[float] | None = None
        self._stable_puncture_px: list[int] | None = None
        self._last_puncture_stable_at = 0.0
        self._clamp_history: deque[dict[str, np.ndarray]] = deque(maxlen=11)
        self._last_heel_center: list[float] | None = None
        self._stable_clamp_axis: np.ndarray | None = None
        self._stable_clamp_center: np.ndarray | None = None
        self._stable_clamp_half_length: float | None = None
        self._stable_clamp_a3d: np.ndarray | None = None
        self._stable_clamp_b3d: np.ndarray | None = None
        self._last_color_encode_at = 0.0
        self._thread: threading.Thread | None = None
        self._vision_thread: threading.Thread | None = None

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

    def snapshot(self) -> dict[str, Any]:
        devices = sorted(glob.glob("/dev/video*"))
        with self._lock:
            color_fps = self._fps(self._color_times)
            depth_fps = self._fps(self._depth_times)
            now = monotonic()
            color_receiving = bool(self._color_times) and now - self._color_times[-1] < 2.0
            depth_receiving = bool(self._depth_times) and now - self._depth_times[-1] < 2.0
            # 足跟定位同时依赖彩色图和深度图，缺少任意一路都不能算可用。
            receiving = color_receiving and depth_receiving
            return {
                "connected": bool(devices),
                "valid": receiving,
                "color_valid": color_receiving,
                "depth_valid": depth_receiving,
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
                    else self._error
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

    def color_png(self) -> bytes | None:
        with self._lock:
            if not self._color_times or monotonic() - self._color_times[-1] >= 2.0:
                return None
            return self._color_png

    def annotated_png(self) -> bytes | None:
        with self._lock:
            if not self._color_times or monotonic() - self._color_times[-1] >= 2.0:
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
        default_model = Path(__file__).resolve().parents[1] / "platform_a" / "models" / "heel_seg.pt"
        model_path = Path(os.environ.get("PLATFORM_A_HEEL_MODEL", str(default_model)))
        detector = HeelClampVision(model_path)
        while True:
            with self._lock:
                frame_id = self._vision_frame_id
                image = None if self._latest_color is None else self._latest_color.copy()
                depth = None if self._aligned_depth_mm is None else self._aligned_depth_mm.copy()
                intrinsics = self._intrinsics
            if image is None or frame_id == self._vision_processed_id:
                threading.Event().wait(0.1)
                continue
            try:
                result, annotated = detector.analyze(image, depth, intrinsics)
                single_frame_clamp_valid = bool(result.get("valid"))
                # 只有多帧夹持结果稳定后才对外宣告可用。
                result["valid"] = False
                clamp_a = result.get("clamp_contact_a_px")
                clamp_b = result.get("clamp_contact_b_px")
                clamp_a_3d = result.get("clamp_contact_a_camera_mm")
                clamp_b_3d = result.get("clamp_contact_b_camera_mm")
                heel_center = result.get("heel_center_camera_mm")
                if result.get("heel_detected") and clamp_a and clamp_b:
                    clamp_a_array = np.asarray(clamp_a, dtype=np.float32)
                    clamp_b_array = np.asarray(clamp_b, dtype=np.float32)
                    current_center = (clamp_a_array + clamp_b_array) * 0.5
                    current_vector = clamp_b_array - clamp_a_array
                    current_length = float(np.linalg.norm(current_vector))
                    current_axis = current_vector / max(current_length, 1e-6)
                    if current_axis[0] < 0.0:
                        current_axis = -current_axis
                    if self._last_heel_center is not None and heel_center:
                        moved = np.linalg.norm(np.asarray(heel_center, dtype=np.float32) - np.asarray(self._last_heel_center, dtype=np.float32))
                        # D435 在硅胶曲面上会有厘米级深度波动；只有目标真的移动
                        # 很明显时才清空历史，避免累计帧数反复从零开始。
                        if moved > 35.0:
                            self._clamp_history.clear()
                            self._stable_clamp_axis = None
                            self._stable_clamp_center = None
                            self._stable_clamp_half_length = None
                            self._stable_clamp_a3d = None
                            self._stable_clamp_b3d = None
                            self._stable_puncture_3d = None
                            self._stable_puncture_px = None
                            self._last_puncture_stable_at = 0.0
                    # 已经取得针方向后，偶尔漏检针时继续沿用最近的可靠方向，
                    # 不让接近圆形的足跟轮廓重新把夹持线带偏。
                    if (
                        result.get("clamp_direction_source") == "needle_perpendicular"
                        or len(self._clamp_history) < 5
                    ):
                        self._clamp_history.append({
                            "a": np.asarray(clamp_a, dtype=np.float32),
                            "b": np.asarray(clamp_b, dtype=np.float32),
                            "center": current_center,
                            "axis": current_axis,
                            "half_length": current_length * 0.5,
                            "a3d": np.asarray(clamp_a_3d, dtype=np.float32) if clamp_a_3d else None,
                            "b3d": np.asarray(clamp_b_3d, dtype=np.float32) if clamp_b_3d else None,
                        })
                    centers = np.asarray([item["center"] for item in self._clamp_history], dtype=np.float32)
                    median_center = np.median(centers, axis=0)
                    distances = np.linalg.norm(centers - median_center, axis=1)
                    mad = float(np.median(np.abs(distances - np.median(distances))))
                    reject_limit = max(4.0, 3.0 * 1.4826 * mad)
                    # 夹持方向是“没有箭头的轴”，0° 与 180° 等价。用双角度
                    # 求多数方向，并排除突然旋转的结果，避免圆形足跟导致 90° 翻转。
                    axes = np.asarray([item["axis"] for item in self._clamp_history], dtype=np.float32)
                    angles = np.arctan2(axes[:, 1], axes[:, 0])
                    # 先在历史结果中找“最接近大多数方向”的那一条。普通平均
                    # 会被金属反光产生的错误直线拉偏，导致所有帧都被排除。
                    differences = angles[:, None] - angles[None, :]
                    pair_errors = np.abs(np.arctan2(np.sin(differences), np.cos(differences)))
                    pair_errors = np.minimum(pair_errors, np.pi - pair_errors)
                    medoid_index = int(np.argmin(np.median(pair_errors, axis=1)))
                    mean_angle = float(angles[medoid_index])
                    angle_errors = np.abs(np.arctan2(
                        np.sin(angles - mean_angle), np.cos(angles - mean_angle)
                    ))
                    angle_errors = np.minimum(angle_errors, np.pi - angle_errors)
                    direction_inliers = angle_errors <= np.deg2rad(25.0)
                    inliers = (distances <= reject_limit) & direction_inliers
                    if int(np.count_nonzero(inliers)) >= 3:
                        kept = [item for item, ok in zip(self._clamp_history, inliers) if ok]
                        kept_axes = np.asarray([item["axis"] for item in kept], dtype=np.float32)
                        kept_angles = np.arctan2(kept_axes[:, 1], kept_axes[:, 0])
                        stable_angle = 0.5 * math.atan2(
                            float(np.mean(np.sin(2.0 * kept_angles))),
                            float(np.mean(np.cos(2.0 * kept_angles))),
                        )
                        stable_axis = np.asarray(
                            [math.cos(stable_angle), math.sin(stable_angle)], dtype=np.float32
                        )
                        if stable_axis[0] < 0.0:
                            stable_axis = -stable_axis
                        measured_center = np.median(
                            np.asarray([item["center"] for item in kept]), axis=0
                        )
                        measured_half_length = float(np.median(
                            np.asarray([item["half_length"] for item in kept])
                        ))
                        output_step_deg = 0.0
                        if self._stable_clamp_axis is None:
                            self._stable_clamp_axis = stable_axis
                        else:
                            previous_angle = math.atan2(
                                float(self._stable_clamp_axis[1]),
                                float(self._stable_clamp_axis[0]),
                            )
                            measured_angle = math.atan2(float(stable_axis[1]), float(stable_axis[0]))
                            delta = math.atan2(
                                math.sin(measured_angle - previous_angle),
                                math.cos(measured_angle - previous_angle),
                            )
                            if delta > math.pi / 2:
                                delta -= math.pi
                            elif delta < -math.pi / 2:
                                delta += math.pi
                            step = float(np.clip(delta, -math.radians(2.0), math.radians(2.0)))
                            output_step_deg = abs(math.degrees(step))
                            output_angle = previous_angle + step
                            self._stable_clamp_axis = np.asarray(
                                [math.cos(output_angle), math.sin(output_angle)], dtype=np.float32
                            )
                        if self._stable_clamp_center is None:
                            self._stable_clamp_center = measured_center
                            self._stable_clamp_half_length = measured_half_length
                        else:
                            self._stable_clamp_center = (
                                self._stable_clamp_center * 0.8 + measured_center * 0.2
                            )
                            self._stable_clamp_half_length = float(
                                self._stable_clamp_half_length * 0.8 + measured_half_length * 0.2
                            )
                        stable_axis = self._stable_clamp_axis
                        stable_center = self._stable_clamp_center
                        stable_half_length = float(self._stable_clamp_half_length)
                        a_px = stable_center - stable_axis * stable_half_length
                        b_px = stable_center + stable_axis * stable_half_length
                        result["clamp_contact_a_px"] = [int(round(float(v))) for v in a_px]
                        result["clamp_contact_b_px"] = [int(round(float(v))) for v in b_px]
                        result["heel_center_px"] = [int(round(float(v))) for v in (a_px + b_px) * 0.5]
                        if all(item["a3d"] is not None and item["b3d"] is not None for item in kept):
                            measured_a3d = np.median(np.asarray([item["a3d"] for item in kept]), axis=0)
                            measured_b3d = np.median(np.asarray([item["b3d"] for item in kept]), axis=0)
                            if self._stable_clamp_a3d is None or self._stable_clamp_b3d is None:
                                self._stable_clamp_a3d = measured_a3d
                                self._stable_clamp_b3d = measured_b3d
                            else:
                                self._stable_clamp_a3d = self._stable_clamp_a3d * 0.8 + measured_a3d * 0.2
                                self._stable_clamp_b3d = self._stable_clamp_b3d * 0.8 + measured_b3d * 0.2
                            result["clamp_contact_a_camera_mm"] = [round(float(v), 2) for v in self._stable_clamp_a3d]
                            result["clamp_contact_b_camera_mm"] = [round(float(v), 2) for v in self._stable_clamp_b3d]
                        result["clamp_point_frame_count"] = len(kept)
                        result["clamp_point_outlier_count"] = len(self._clamp_history) - len(kept)
                        result["clamp_point_jitter_px"] = round(float(np.median(distances[inliers])), 2)
                        result["clamp_direction_jitter_deg"] = round(
                            float(np.degrees(np.median(angle_errors[inliers]))), 2
                        )
                        result["clamp_direction_output_step_deg"] = round(output_step_deg, 2)
                        result["clamp_points_stable"] = bool(
                            len(kept) >= 5
                            and result["clamp_point_jitter_px"] <= 3.0
                            and output_step_deg <= 2.0
                        )
                        result["valid"] = bool(single_frame_clamp_valid) and bool(result["clamp_points_stable"])
                        stable_a = tuple(result["clamp_contact_a_px"])
                        stable_b = tuple(result["clamp_contact_b_px"])
                        stable_c = tuple(result["heel_center_px"])
                        cv2.line(annotated, stable_a, stable_b, (0, 210, 255), 2)
                        cv2.circle(annotated, stable_a, 6, (0, 210, 255), -1)
                        cv2.circle(annotated, stable_b, 6, (0, 210, 255), -1)
                        cv2.drawMarker(
                            annotated, stable_c, (255, 255, 255), cv2.MARKER_CROSS, 18, 2
                        )
                point_3d = result.get("puncture_camera_mm")
                point_px = result.get("puncture_px")
                if result.get("heel_detected") and heel_center:
                    if (
                        self._last_heel_center is not None
                        and np.linalg.norm(
                            np.asarray(heel_center, dtype=np.float32)
                            - np.asarray(self._last_heel_center, dtype=np.float32)
                        ) > 35.0
                    ):
                        self._puncture_history.clear()
                    self._last_heel_center = list(heel_center)
                if result.get("heel_detected") and point_3d and point_px:
                    self._puncture_history.append((list(point_3d), list(point_px)))
                    points_3d = np.asarray([item[0] for item in self._puncture_history], dtype=np.float32)
                    points_px = np.asarray([item[1] for item in self._puncture_history], dtype=np.float32)
                    median_3d = np.median(points_3d, axis=0)
                    median_px = np.median(points_px, axis=0)
                    distances = np.linalg.norm(points_3d - median_3d, axis=1)
                    jitter_mm = float(np.median(distances))
                    stable = len(self._puncture_history) >= 5 and jitter_mm <= 3.0
                    result["puncture_camera_mm"] = [round(float(v), 2) for v in median_3d]
                    result["puncture_px"] = [int(round(float(v))) for v in median_px]
                    result["puncture_jitter_mm"] = round(jitter_mm, 2)
                    result["puncture_stable"] = stable
                    if stable:
                        self._stable_puncture_3d = result["puncture_camera_mm"]
                        self._stable_puncture_px = result["puncture_px"]
                        self._last_puncture_stable_at = monotonic()
                    elif (
                        self._stable_puncture_3d is not None
                        and monotonic() - self._last_puncture_stable_at <= 3.0
                    ):
                        result["puncture_camera_mm"] = list(self._stable_puncture_3d)
                        result["puncture_px"] = list(self._stable_puncture_px or [])
                        result["puncture_stable"] = True
                        result["puncture_held_from_last_frame"] = True
                    if stable:
                        needle_line = result.get("visible_needle_line_px")
                        if needle_line and len(needle_line) == 4:
                            x1, y1, x2, y2 = map(int, needle_line)
                            cv2.line(annotated, (x1, y1), (x2, y2), (40, 40, 240), 2)
                        else:
                            result["visible_needle_line_px"] = None
                        stable_x, stable_y = result["puncture_px"]
                        cv2.circle(
                            annotated, (stable_x, stable_y), 8, (40, 40, 240), 2
                        )
                        cv2.drawMarker(
                            annotated,
                            (stable_x, stable_y),
                            (40, 40, 240),
                            cv2.MARKER_CROSS,
                            12,
                            2,
                        )
                    if not stable:
                        result["puncture_message"] = "正在连续确认针孔"
                elif not result.get("heel_detected"):
                    self._puncture_history.clear()
                    self._stable_puncture_3d = None
                    self._stable_puncture_px = None
                    self._last_puncture_stable_at = 0.0
                    self._clamp_history.clear()
                    self._stable_clamp_axis = None
                    self._stable_clamp_center = None
                    self._stable_clamp_half_length = None
                    self._stable_clamp_a3d = None
                    self._stable_clamp_b3d = None
                    self._last_heel_center = None
                    result["puncture_stable"] = False
                    result["valid"] = False
                else:
                    result["puncture_stable"] = False
                    if (
                        result.get("heel_detected")
                        and self._stable_puncture_3d is not None
                        and monotonic() - self._last_puncture_stable_at <= 3.0
                    ):
                        result["puncture_camera_mm"] = list(self._stable_puncture_3d)
                        result["puncture_px"] = list(self._stable_puncture_px or [])
                        result["puncture_stable"] = True
                        result["puncture_held_from_last_frame"] = True
                result["camera_frame_id"] = frame_id
                result["updated_at"] = datetime.now(timezone.utc).isoformat(timespec="milliseconds")
                encoded_ok, encoded = cv2.imencode(".png", annotated)
                if not encoded_ok:
                    raise ValueError("识别画面编码失败")
                with self._lock:
                    self._vision_result = result
                    self._annotated_png = encoded.tobytes()
                    self._vision_processed_id = frame_id
                    self._vision_result_at = monotonic()
            except Exception as error:
                with self._lock:
                    self._vision_result = {
                        "valid": False,
                        "motion_allowed": False,
                        "message": f"夹挤识别不可用：{error}",
                    }
                    self._vision_processed_id = frame_id
                    self._vision_result_at = monotonic()
            # 足模型在规划时保持静止；每秒识别约一次即可，原始视频仍独立流畅刷新。
            threading.Event().wait(0.65)

    def _run(self) -> None:
        try:
            import rclpy
            from rclpy.node import Node
            from rclpy.qos import qos_profile_sensor_data
            from sensor_msgs.msg import CameraInfo, Image

            rclpy.init(args=None)
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
