from __future__ import annotations

import math
import os
import threading
from dataclasses import dataclass
from pathlib import Path
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float


class HeelClampVision:
    """第一步夹挤的视觉计算。

    输出仍然是相机坐标，不能直接当作机械臂运动坐标。
    """

    def __init__(self, model_path: str | Path) -> None:
        self.model_path = Path(model_path)
        self._model: Any | None = None
        self._model_lock = threading.Lock()
        self.confidence = float(os.environ.get("HEEL_VISION_CONFIDENCE", "0.25"))
        self.needle_to_sole_mm = float(os.environ.get("HEEL_NEEDLE_TO_SOLE_MM", "20"))

    def _load_model(self) -> Any:
        with self._model_lock:
            if self._model is None:
                if not self.model_path.is_file():
                    raise FileNotFoundError(f"未找到足跟模型：{self.model_path}")
                from ultralytics import YOLO

                self._model = YOLO(str(self.model_path))
            return self._model

    @staticmethod
    def _point_from_depth(
        pixel: tuple[int, int],
        depth_mm: np.ndarray | None,
        intrinsics: CameraIntrinsics | None,
        valid_mask: np.ndarray | None = None,
        radius: int = 4,
    ) -> list[float] | None:
        if depth_mm is None or intrinsics is None:
            return None
        u, v = pixel
        h, w = depth_mm.shape[:2]
        x0, x1 = max(0, u - radius), min(w, u + radius + 1)
        y0, y1 = max(0, v - radius), min(h, v + radius + 1)
        values = depth_mm[y0:y1, x0:x1].astype(np.float32)
        usable = np.isfinite(values) & (values > 100) & (values < 3000)
        if valid_mask is not None:
            usable &= valid_mask[y0:y1, x0:x1]
        values = values[usable]
        if values.size < 3:
            return None
        z = float(np.median(values))
        return [
            round((u - intrinsics.cx) * z / intrinsics.fx, 2),
            round((v - intrinsics.cy) * z / intrinsics.fy, 2),
            round(z, 2),
        ]

    @staticmethod
    def _project_with_depth(
        pixel: tuple[int, int], z_mm: float, intrinsics: CameraIntrinsics
    ) -> list[float]:
        u, v = pixel
        return [
            round((u - intrinsics.cx) * z_mm / intrinsics.fx, 2),
            round((v - intrinsics.cy) * z_mm / intrinsics.fy, 2),
            round(z_mm, 2),
        ]

    @staticmethod
    def _fit_local_plane(
        mask: np.ndarray,
        depth_mm: np.ndarray | None,
        intrinsics: CameraIntrinsics | None,
        center_px: tuple[int, int],
    ) -> dict[str, Any] | None:
        """用夹持带附近的多点深度拟合足跟局部平面。"""
        if depth_mm is None or intrinsics is None:
            return None
        ys, xs = np.nonzero(mask)
        if len(xs) < 150:
            return None
        height = max(1, int(ys.max() - ys.min()))
        half_band = max(8, int(round(height * 0.18)))
        yy, xx = np.indices(depth_mm.shape[:2])
        inner = cv2.erode(mask.astype(np.uint8), np.ones((5, 5), np.uint8), iterations=1) > 0
        region = inner & (np.abs(yy - int(center_px[1])) <= half_band)
        valid = (
            region
            & np.isfinite(depth_mm)
            & (depth_mm > 100.0)
            & (depth_mm < 3000.0)
        )
        v, u = np.nonzero(valid)
        if len(u) < 120:
            return None
        u = u[::2]
        v = v[::2]
        z = depth_mm[v, u].astype(np.float64)
        median_z = float(np.median(z))
        mad_z = float(np.median(np.abs(z - median_z)))
        z_limit = max(12.0, 3.0 * 1.4826 * mad_z)
        keep = np.abs(z - median_z) <= z_limit
        u, v, z = u[keep], v[keep], z[keep]
        if len(z) < 80:
            return None
        points = np.column_stack(
            ((u - intrinsics.cx) * z / intrinsics.fx,
             (v - intrinsics.cy) * z / intrinsics.fy, z)
        )
        center = points.mean(axis=0)
        _, _, vt = np.linalg.svd(points - center, full_matrices=False)
        normal = vt[-1]
        normal /= np.linalg.norm(normal)
        residual = np.abs((points - center) @ normal)
        residual_median = float(np.median(residual))
        inlier_limit = max(2.5, 3.0 * 1.4826 * residual_median)
        inliers = residual <= inlier_limit
        if int(np.count_nonzero(inliers)) < 70:
            return None
        fit_points = points[inliers]
        center = fit_points.mean(axis=0)
        _, _, vt = np.linalg.svd(fit_points - center, full_matrices=False)
        normal = vt[-1]
        normal /= np.linalg.norm(normal)
        if normal[2] < 0.0:
            normal = -normal
        errors = np.abs((fit_points - center) @ normal)
        return {
            "point_camera_mm": [round(float(x), 3) for x in center],
            "normal_camera": [round(float(x), 7) for x in normal],
            "sample_count": int(len(points)),
            "inlier_count": int(len(fit_points)),
            "inlier_ratio": round(float(len(fit_points) / len(points)), 4),
            "rmse_mm": round(float(np.sqrt(np.mean(errors**2))), 3),
            "p95_error_mm": round(float(np.percentile(errors, 95)), 3),
            "band_half_height_px": int(half_band),
        }

    @staticmethod
    def _ray_plane_intersection(
        pixel: tuple[int, int], plane: dict[str, Any] | None,
        intrinsics: CameraIntrinsics | None,
    ) -> list[float] | None:
        if plane is None or intrinsics is None:
            return None
        u, v = pixel
        ray = np.array([(u - intrinsics.cx) / intrinsics.fx,
                        (v - intrinsics.cy) / intrinsics.fy, 1.0], dtype=np.float64)
        point = np.asarray(plane["point_camera_mm"], dtype=np.float64)
        normal = np.asarray(plane["normal_camera"], dtype=np.float64)
        denominator = float(np.dot(normal, ray))
        if abs(denominator) < 1e-6:
            return None
        scale = float(np.dot(normal, point) / denominator)
        if not 100.0 <= scale <= 3000.0:
            return None
        result = ray * scale
        return [round(float(x), 2) for x in result]

    @staticmethod
    def _mask_geometry(
        mask: np.ndarray,
        preferred_clamp_axis: np.ndarray | None = None,
        preferred_center: tuple[int, int] | None = None,
    ) -> dict[str, Any]:
        ys, xs = np.nonzero(mask)
        if len(xs) < 100:
            raise ValueError("足跟轮廓太小")
        points = np.column_stack((xs, ys)).astype(np.float32)
        center = points.mean(axis=0)
        covariance = np.cov(points - center, rowvar=False)
        values, vectors = np.linalg.eigh(covariance)
        if preferred_clamp_axis is not None:
            clamp_axis = np.asarray(preferred_clamp_axis, dtype=np.float32)
        else:
            # 仅作为后备。足跟接近圆形时，单靠轮廓主轴可能翻转 90°。
            clamp_axis = vectors[:, int(np.argmin(values))].astype(np.float32)
        clamp_axis /= max(float(np.linalg.norm(clamp_axis)), 1e-6)
        if clamp_axis[0] < 0.0:
            clamp_axis = -clamp_axis
        if np.linalg.norm(clamp_axis) < 1e-6:
            raise ValueError("足跟左右方向不稳定")
        # 旧原型把“最宽的横截面”当成夹持位置，这会随着足跟斜放、遮挡
        # 或分割边缘的小变化而跳动。原始项目的定义是：YOLO 掩码质心
        # 就是目标中心，足跟表面方向再由深度点云 PCA/SVD 求出。因此
        # 这里固定使用掩码质心，不再沿长轴寻找最宽行。
        if preferred_center is not None:
            target = np.asarray(preferred_center, dtype=np.float32)
            # 只在目标所在的横截面取左右边缘，不能把整个足跟轮廓的
            # 上下范围混进来。
            longitudinal = np.asarray([-clamp_axis[1], clamp_axis[0]], dtype=np.float32)
            along_local = (points - target) @ longitudinal
            band_half = max(3.0, float(np.ptp(points @ longitudinal)) * 0.035)
            band = np.abs(along_local) <= band_half
            if int(np.count_nonzero(band)) >= 16:
                center = target
                across = (points[band] - center) @ clamp_axis
            else:
                center = target
                across = (points - center) @ clamp_axis
        else:
            across = (points - center) @ clamp_axis
        low = float(np.percentile(across, 10))
        high = float(np.percentile(across, 90))
        contact_a = center + clamp_axis * low
        contact_b = center + clamp_axis * high
        return {
            "center": center,
            "axis": clamp_axis,
            "contact_a": contact_a,
            "contact_b": contact_b,
            "width_px": float(high - low),
            "contact_angle_deg": round(float(np.degrees(np.arctan2(clamp_axis[1], clamp_axis[0]))), 2),
        }

    @staticmethod
    def _foot_long_axis(image_bgr: np.ndarray, heel_mask: np.ndarray) -> np.ndarray | None:
        """用足部的大肉色区域估计足部前后方向。

        这不是新的模型：只在足跟掩码附近取肉色连通区域，并用 PCA 求长轴。
        夹爪连线使用该长轴的垂线。若肉色区域不可靠，则返回 None，交给足跟
        掩码的几何后备方案。
        """
        ys, xs = np.nonzero(heel_mask)
        if len(xs) < 100:
            return None
        h, w = image_bgr.shape[:2]
        pad_x = max(40, int((xs.max() - xs.min()) * 0.9))
        pad_y = max(40, int((ys.max() - ys.min()) * 1.2))
        x0, x1 = max(0, int(xs.min()) - pad_x), min(w, int(xs.max()) + pad_x + 1)
        y0, y1 = max(0, int(ys.min()) - pad_y), min(h, int(ys.max()) + pad_y + 1)
        roi = image_bgr[y0:y1, x0:x1]
        hsv = cv2.cvtColor(roi, cv2.COLOR_BGR2HSV)
        # 肉色范围允许随光照变化，但排除蓝色垫子和白色桌面。
        skin = (((hsv[:, :, 0] <= 25) | (hsv[:, :, 0] >= 165)) &
                (hsv[:, :, 1] >= 25) & (hsv[:, :, 1] <= 210) &
                (hsv[:, :, 2] >= 45)).astype(np.uint8) * 255
        kernel = np.ones((5, 5), np.uint8)
        skin = cv2.morphologyEx(skin, cv2.MORPH_OPEN, kernel)
        skin = cv2.morphologyEx(skin, cv2.MORPH_CLOSE, kernel)
        heel_local = heel_mask[y0:y1, x0:x1].astype(np.uint8)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(skin, 8)
        candidates = []
        for label in range(1, count):
            area = int(stats[label, cv2.CC_STAT_AREA])
            if area < 400:
                continue
            overlap = int(np.count_nonzero((labels == label) & (heel_local > 0)))
            if overlap:
                candidates.append((overlap, area, label))
        if not candidates:
            return None
        _, _, label = max(candidates, key=lambda item: (item[0], item[1]))
        py, px = np.nonzero(labels == label)
        if len(px) < 400:
            return None
        points = np.column_stack((px, py)).astype(np.float32)
        heel_py, heel_px = np.nonzero(heel_local)
        heel_center = np.asarray([heel_px.mean(), heel_py.mean()], dtype=np.float32)

        # 不能把整条小腿也拿来做 PCA：脚踝处会形成 V 形，主轴会偏向小腿。
        # 当前工作站中脚趾始终位于足跟的画面上方，因此只从足跟上方、距离最远
        # 的一簇肉色点取“朝脚趾方向”。这一簇是脚掌和脚趾，而不是小腿或手臂。
        relative = points - heel_center
        distance = np.linalg.norm(relative, axis=1)
        upper = relative[:, 1] < -10.0
        if int(np.count_nonzero(upper)) >= 120:
            threshold = float(np.percentile(distance[upper], 68))
            toe_points = points[upper & (distance >= threshold)]
        else:
            toe_points = np.empty((0, 2), dtype=np.float32)
        if len(toe_points) >= 30:
            toe_center = np.median(toe_points, axis=0)
            axis = toe_center - heel_center
        else:
            # 画面不满足“脚趾在上方”时才退回普通主轴。
            center = points.mean(axis=0)
            covariance = np.cov(points - center, rowvar=False)
            values, vectors = np.linalg.eigh(covariance)
            axis = vectors[:, int(np.argmax(values))].astype(np.float32)
        norm = float(np.linalg.norm(axis))
        if norm < 1e-6:
            return None
        axis /= norm
        # 夹爪方向是足部长轴的垂线。
        clamp_axis = np.asarray([-axis[1], axis[0]], dtype=np.float32)
        if clamp_axis[0] < 0:
            clamp_axis = -clamp_axis
        return clamp_axis

    @staticmethod
    def _clamp_axis_from_needle(line: list[int] | None) -> np.ndarray | None:
        """针的方向近似足跟纵向；夹爪两指连线应与针垂直。"""
        if line is None or len(line) != 4:
            return None
        x1, y1, x2, y2 = (float(value) for value in line)
        needle = np.asarray([x2 - x1, y2 - y1], dtype=np.float32)
        length = float(np.linalg.norm(needle))
        if length < 12.0:
            return None
        axis = np.asarray([-needle[1], needle[0]], dtype=np.float32) / length
        if axis[0] < 0.0:
            axis = -axis
        return axis

    @staticmethod
    def _needle_inward_direction(
        line: list[int] | None,
        boundary: tuple[int, int],
        mask: np.ndarray,
    ) -> np.ndarray | None:
        """返回从针孔/边界指向足跟内部的图像方向。"""
        if line is None or len(line) != 4:
            return None
        x1, y1, x2, y2 = (float(value) for value in line)
        direction = np.asarray([x2 - x1, y2 - y1], dtype=np.float32)
        norm = float(np.linalg.norm(direction))
        if norm < 12.0:
            return None
        direction /= norm
        u, v = boundary
        h, w = mask.shape[:2]
        for candidate in (direction, -direction):
            probe = np.asarray([u, v], dtype=np.float32) + candidate * 5.0
            px, py = int(round(float(probe[0]))), int(round(float(probe[1])))
            if 0 <= px < w and 0 <= py < h and bool(mask[py, px]):
                return candidate
        return None

    @staticmethod
    def _detect_puncture(image: np.ndarray, heel_mask: np.ndarray) -> dict[str, Any]:
        """在足跟轮廓内部寻找针孔样的局部暗点，不训练额外模型。"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8)).apply(gray)
        # 针在 D435 彩色画面中表现为细亮白线。使用“局部亮线”而不是普通边缘，
        # 避免把手部、衣物或足跟轮廓误认成针。
        near = cv2.dilate(heel_mask.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=70)
        top_hat = cv2.morphologyEx(
            gray, cv2.MORPH_TOPHAT,
            cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (11, 11)),
        )
        local_values = top_hat[near > 0]
        bright_limit = max(12.0, float(np.percentile(local_values, 88))) if local_values.size else 12.0
        bright_line = ((top_hat >= bright_limit) & (gray >= 105) & (near > 0)).astype(np.uint8) * 255
        bright_line = cv2.morphologyEx(
            bright_line, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8)
        )
        lines = cv2.HoughLinesP(
            bright_line, 1, np.pi / 180, threshold=5, minLineLength=10, maxLineGap=26
        )
        line_best: tuple[float, tuple[int, int], list[int]] | None = None
        mask_y, mask_x = np.nonzero(heel_mask)
        mask_center_x = float(mask_x.mean())
        mask_width = max(1.0, float(mask_x.max() - mask_x.min()))
        if lines is not None:
            for x1, y1, x2, y2 in lines[:, 0, :]:
                dx, dy = int(x2 - x1), int(y2 - y1)
                length = math.hypot(dx, dy)
                if length < 14 or abs(dy) < abs(dx) * 0.35:
                    continue
                samples = np.rint(np.linspace((x1, y1), (x2, y2), 100)).astype(int)
                samples[:, 0] = np.clip(samples[:, 0], 0, gray.shape[1] - 1)
                samples[:, 1] = np.clip(samples[:, 1], 0, gray.shape[0] - 1)
                inside = heel_mask[samples[:, 1], samples[:, 0]]
                inside_indices = np.flatnonzero(inside)
                outside_indices = np.flatnonzero(~inside)
                if len(inside_indices) < 3 or len(outside_indices) < 8:
                    continue
                transitions = np.flatnonzero(inside[1:] != inside[:-1]) + 1
                if len(transitions) == 0:
                    continue
                transition = min(transitions, key=lambda index: abs(float(samples[index, 0]) - mask_center_x))
                point = (int(samples[transition, 0]), int(samples[transition, 1]))
                if abs(point[0] - mask_center_x) > mask_width * 0.55:
                    continue
                mean_brightness = float(gray[samples[:, 1], samples[:, 0]].mean())
                score = mean_brightness + 0.05 * length + 0.1 * len(outside_indices) - 0.15 * abs(point[0] - mask_center_x)
                if line_best is None or score > line_best[0]:
                    line_best = (score, point, [int(x1), int(y1), int(x2), int(y2)])
        if line_best is not None:
            return {
                "detected": True,
                "point_px": list(line_best[1]),
                "contrast": None,
                "visible_line_px": line_best[2],
                "method": "由露出的针与足跟边缘交点确定",
                "message": "已由露出的针确定针孔",
            }

        inner = cv2.dilate(heel_mask.astype(np.uint8), np.ones((3, 3), np.uint8), iterations=1)
        values = gray[inner > 0]
        if values.size < 100:
            return {"detected": False, "message": "足跟有效区域太小，无法寻找针孔"}
        dark_limit = min(float(np.percentile(values, 12)), float(np.median(values) - 18))
        dark = ((gray <= dark_limit) & (inner > 0)).astype(np.uint8) * 255
        dark = cv2.morphologyEx(dark, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        count, labels, stats, centroids = cv2.connectedComponentsWithStats(dark)
        ys, xs = np.nonzero(heel_mask)
        center = np.array([xs.mean(), ys.mean()], dtype=np.float32)
        image_area = image.shape[0] * image.shape[1]
        min_area = max(3, int(image_area * 0.00002))
        max_area = max(80, int(image_area * 0.004))
        best: tuple[float, tuple[int, int], float] | None = None
        for label in range(1, count):
            x, y, width, height, area = map(int, stats[label])
            if area < min_area or area > max_area or width > height * 5 or height > width * 5:
                continue
            component = labels == label
            mean_dark = float(gray[component].mean())
            pad = 5
            y0, y1 = max(0, y - pad), min(gray.shape[0], y + height + pad)
            x0, x1 = max(0, x - pad), min(gray.shape[1], x + width + pad)
            ring = gray[y0:y1, x0:x1][inner[y0:y1, x0:x1] > 0]
            contrast = float(np.median(ring) - mean_dark) if ring.size else 0.0
            fill = area / max(1.0, float(width * height))
            point = np.array(centroids[label], dtype=np.float32)
            center_distance = float(np.linalg.norm(point - center))
            if abs(float(point[0] - center[0])) > float(xs.max() - xs.min()) * 0.30:
                continue
            score = contrast + 12.0 * fill - 0.025 * center_distance
            if contrast >= 10 and (best is None or score > best[0]):
                best = (score, (int(round(point[0])), int(round(point[1]))), contrast)
        if best is None:
            return {"detected": False, "message": "足跟内没有找到针孔，边缘也没有找到插入的针"}
        return {
            "detected": True,
            "point_px": list(best[1]),
            "contrast": round(best[2], 1),
            "visible_line_px": None,
            "method": "针孔黑点",
            "message": "已找到针孔黑点",
        }

    def analyze(
        self,
        image_bgr: np.ndarray,
        depth_mm: np.ndarray | None = None,
        intrinsics: CameraIntrinsics | None = None,
    ) -> tuple[dict[str, Any], np.ndarray]:
        model = self._load_model()
        # 与 ji_cheng_YOLO/deeplearning/ultralytics-8.3.163/mypredict_seg.py 保持一致：
        # 使用分割掩码轮廓质心；针线只作辅助显示，不能改变目标点。
        prediction = model.predict(
            image_bgr,
            imgsz=640,
            conf=max(0.5, self.confidence),
            retina_masks=True,
            verbose=False,
            device="cpu",
        )[0]
        if prediction.masks is None or prediction.boxes is None or len(prediction.boxes) == 0:
            return ({
                "valid": False,
                "heel_detected": False,
                "puncture_detected": False,
                "motion_allowed": False,
                "message": "没有识别到足跟",
            }, image_bgr.copy())

        confidences = prediction.boxes.conf.detach().cpu().numpy()
        index = int(np.argmax(confidences))
        raw_mask = prediction.masks.data[index].detach().cpu().numpy()
        h, w = image_bgr.shape[:2]
        mask = cv2.resize(raw_mask, (w, h), interpolation=cv2.INTER_NEAREST) > 0.5
        # 原项目把足跟轮廓质心作为机械臂目标点。
        contour_list, _ = cv2.findContours(
            mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE
        )
        contour = max(contour_list, key=cv2.contourArea) if contour_list else None
        source_center_px = None
        if contour is not None:
            moments = cv2.moments(contour)
            if abs(float(moments["m00"])) > 1e-6:
                source_center_px = (
                    int(round(float(moments["m10"] / moments["m00"]))),
                    int(round(float(moments["m01"] / moments["m00"]))),
                )
        puncture = self._detect_puncture(image_bgr, mask)
        puncture_px = puncture.get("point_px")
        # 针线不是识别输入。夹爪方向改为足部肉色区域长轴的垂线。
        needle_axis = self._foot_long_axis(image_bgr, mask)
        inward = None
        target_center_px: tuple[int, int] | None = None
        # 针孔到足底的已知距离是20 mm。夹持位置取针孔沿针的“进入足跟”
        # 方向前进20 mm处，而不是取整块掩码的质心。
        if puncture_px and inward is not None:
            puncture_point = self._point_from_depth(
                tuple(puncture_px), depth_mm, intrinsics, mask, radius=12
            )
            if puncture_point is not None and intrinsics is not None:
                z = max(100.0, float(puncture_point[2]))
                pixel_step = self.needle_to_sole_mm * float(intrinsics.fx) / z
                candidate = np.asarray(puncture_px, dtype=np.float32) + inward * pixel_step
                target_center_px = (
                    int(round(float(candidate[0]))),
                    int(round(float(candidate[1]))),
                )
            else:
                target_center_px = tuple(
                    int(round(float(v)))
                    for v in np.asarray(puncture_px, dtype=np.float32) + inward * 14.0
                )
        # 不论针孔检测结果如何，最终目标点回到原项目的掩码质心。
        # 这样针线反光、斜放和单帧误检不会把夹持点推走。
        target_center_px = source_center_px
        geometry = self._mask_geometry(mask, needle_axis, target_center_px)
        center = tuple(int(round(v)) for v in geometry["center"])
        contact_a = tuple(int(round(v)) for v in geometry["contact_a"])
        contact_b = tuple(int(round(v)) for v in geometry["contact_b"])

        plane = self._fit_local_plane(mask, depth_mm, intrinsics, center)
        center_3d = self._ray_plane_intersection(center, plane, intrinsics)
        a_3d = None
        b_3d = None
        width_mm = None
        if center_3d is not None and plane is not None and intrinsics is not None:
            # 足跟是弧面，左右两侧不能共用中心点的深度。极边缘常混入背景，
            # 因此在轮廓内侧约20%的位置取各自深度，再投影回真实接触像素。
            a_3d = self._ray_plane_intersection(contact_a, plane, intrinsics)
            b_3d = self._ray_plane_intersection(contact_b, plane, intrinsics)
            if a_3d is not None and b_3d is not None:
                width_mm = round(
                    float(
                        np.linalg.norm(
                            np.asarray(b_3d, dtype=np.float64)
                            - np.asarray(a_3d, dtype=np.float64)
                        )
                    ),
                    2,
                )
                if not 20.0 <= width_mm <= 120.0:
                    width_mm = None
        puncture_3d = (
            self._point_from_depth(tuple(puncture_px), depth_mm, intrinsics, mask, radius=12)
            if puncture_px else None
        )
        if center_3d is not None and puncture_3d is not None:
            if abs(puncture_3d[2] - center_3d[2]) > 50.0:
                puncture_3d = None

        annotated = image_bgr.copy()
        overlay = annotated.copy()
        overlay[mask] = (40, 190, 70)
        annotated = cv2.addWeighted(overlay, 0.28, annotated, 0.72, 0)
        # 夹持线由上层监视器在多帧筛选后绘制，避免单帧方向跳动。
        # 针的位置需要由监视器做多帧稳定处理后再绘制。这里不显示单帧原始点，
        # 避免一帧反光误识别导致红线在画面中乱跳。

        result = {
            "valid": bool(
                center_3d is not None
                and width_mm is not None
                and plane is not None
                and int(plane.get("inlier_count", 0)) >= 70
                and float(plane.get("rmse_mm", 999.0)) <= 5.0
            ),
            "heel_detected": True,
            "heel_confidence": round(float(confidences[index]), 4),
            "image_width": int(w),
            "image_height": int(h),
            "heel_center_px": list(center),
            "clamp_target_rule": "针孔沿针进入足跟方向20毫米处",
            "needle_inward_direction_px": None if inward is None else [round(float(v), 5) for v in inward],
            "heel_center_camera_mm": center_3d,
            "clamp_contact_a_px": list(contact_a),
            "clamp_contact_b_px": list(contact_b),
            "clamp_contact_a_camera_mm": a_3d,
            "clamp_contact_b_camera_mm": b_3d,
            "heel_width_mm": width_mm,
            "clamp_direction_source": "foot_skin_long_axis_perpendicular" if needle_axis is not None else "heel_mask_fallback",
            "clamp_direction_angle_deg": geometry["contact_angle_deg"],
            "heel_plane_point_camera_mm": None if plane is None else plane["point_camera_mm"],
            "heel_plane_normal_camera": None if plane is None else plane["normal_camera"],
            "heel_plane_sample_count": 0 if plane is None else plane["sample_count"],
            "heel_plane_inlier_count": 0 if plane is None else plane["inlier_count"],
            "heel_plane_inlier_ratio": 0.0 if plane is None else plane["inlier_ratio"],
            "heel_plane_rmse_mm": None if plane is None else plane["rmse_mm"],
            "heel_plane_p95_error_mm": None if plane is None else plane["p95_error_mm"],
            "puncture_detected": bool(puncture.get("detected")),
            "puncture_px": puncture_px,
            "puncture_camera_mm": puncture_3d,
            "puncture_contrast": puncture.get("contrast"),
            "puncture_method": puncture.get("method"),
            "visible_needle_line_px": puncture.get("visible_line_px"),
            "needle_to_sole_expected_mm": self.needle_to_sole_mm,
            "coordinate_system": "camera",
            "motion_allowed": False,
            "message": (
                "足跟夹持位置已找到；针孔同时可见"
                if puncture.get("detected")
                else "足跟夹持位置已找到；当前帧没有看清针孔，沿用已确认的夹持方向"
            ),
        }
        # 清除旧版本遗留的针孔规则说明；针线不属于本算法输入。
        result["clamp_target_rule"] = "足跟分割轮廓质心；夹爪方向取足部长轴的垂线"
        return result, annotated
