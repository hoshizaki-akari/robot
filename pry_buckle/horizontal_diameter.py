"""Independent horizontal-diameter measurement for the pry-buckle task."""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np


@dataclass(frozen=True)
class CameraIntrinsics:
    fx: float
    fy: float
    cx: float
    cy: float
    # The RealSense launch file rotates the published color/depth images by
    # 180 degrees for the upright UI. CameraInfo still describes the native
    # optical frame, so geometry maps display pixels back to native pixels.
    image_width: int | None = None
    image_height: int | None = None
    image_rotation_deg: int = 0


class HorizontalDiameterEstimator:
    """Use only the image-horizontal chord of a semantic heel mask."""

    def __init__(self, expected_min_mm: float = 45.0, expected_max_mm: float = 65.0) -> None:
        if not 0.0 < expected_min_mm < expected_max_mm:
            raise ValueError("invalid expected diameter range")
        self.expected_min_mm = float(expected_min_mm)
        self.expected_max_mm = float(expected_max_mm)

    @staticmethod
    def _span(row: np.ndarray, x: int) -> tuple[int, int] | None:
        if x < 0 or x >= row.size or not bool(row[x]):
            return None
        left = right = x
        while left > 0 and bool(row[left - 1]):
            left -= 1
        while right + 1 < row.size and bool(row[right + 1]):
            right += 1
        return left, right

    @classmethod
    def horizontal_endpoints(cls, heel_mask: np.ndarray) -> tuple[tuple[int, int], tuple[int, int], tuple[int, int]]:
        mask = np.asarray(heel_mask, dtype=bool)
        ys, xs = np.nonzero(mask)
        if xs.size < 80:
            raise ValueError("heel mask has too few pixels")
        cx, cy = int(round(float(np.median(xs)))), int(round(float(np.median(ys))))
        spans: list[tuple[int, int, int]] = []
        for y in range(max(0, cy - 3), min(mask.shape[0], cy + 4)):
            span = cls._span(mask[y], cx)
            if span is not None and span[1] - span[0] >= 8:
                spans.append((span[0], span[1], y))
        if not spans:
            raise ValueError("centroid has no horizontal heel span")
        left = int(round(float(np.median([p[0] for p in spans]))))
        right = int(round(float(np.median([p[1] for p in spans]))))
        y = int(round(float(np.median([p[2] for p in spans]))))
        return (left, y), (right, y), (cx, y)

    @staticmethod
    def upper_midpoint(heel_mask: np.ndarray, center_x: int) -> tuple[int, int]:
        mask = np.asarray(heel_mask, dtype=bool)
        ys, xs = np.nonzero(mask)
        if xs.size < 80:
            raise ValueError("heel mask has too few pixels")
        # Use the upper point on the central vertical section of the heel,
        # avoiding a stray silhouette pixel at the lateral edge.
        band = max(3, int(round(mask.shape[1] * 0.04)))
        selected = ys[(xs >= center_x - band) & (xs <= center_x + band)]
        if selected.size == 0:
            raise ValueError("heel mask has no central upper point")
        top_y = int(np.min(selected))
        return int(center_x), top_y

    @staticmethod
    def _depth_mm(depth: np.ndarray) -> np.ndarray:
        values = np.asarray(depth, dtype=np.float32)
        positive = values[np.isfinite(values) & (values > 0)]
        return values * 1000.0 if positive.size and float(np.median(positive)) < 10.0 else values

    @staticmethod
    def _foreground_plane(depth: np.ndarray, mask: np.ndarray, intrinsics: CameraIntrinsics) -> tuple[np.ndarray, float, int] | None:
        """Fit nearest populated depth layer, excluding silhouette background.

        D435 depth at a colour-mask contour can be a far background sample.
        The horizontal endpoints stay on the colour contour, but their 3-D
        positions are obtained by ray/plane intersection with this locally
        coherent, nearest heel layer.
        """
        valid = mask & np.isfinite(depth) & (depth > 100.0) & (depth < 3000.0)
        samples = depth[valid]
        if samples.size < 70:
            return None
        edges = np.arange(100.0, 3010.0, 10.0)
        counts, _ = np.histogram(samples, bins=edges)
        populated = np.flatnonzero(counts >= max(60, int(samples.size * 0.04)))
        if populated.size == 0:
            return None
        mode = float((edges[int(populated[0])] + edges[int(populated[0]) + 1]) * 0.5)
        layer = valid & (np.abs(depth - mode) <= 25.0)
        ys, xs = np.nonzero(layer)
        if xs.size < 70:
            return None
        z = depth[ys, xs].astype(np.float64)
        native_xs, native_ys = HorizontalDiameterEstimator._native_pixels(xs, ys, intrinsics)
        points = np.column_stack(((native_xs - intrinsics.cx) * z / intrinsics.fx, (native_ys - intrinsics.cy) * z / intrinsics.fy, z))
        center = np.mean(points, axis=0)
        _, _, vt = np.linalg.svd(points - center, full_matrices=False)
        normal = vt[-1]
        residual = np.abs((points - center) @ normal)
        inliers = residual <= max(3.0, float(np.percentile(residual, 80)))
        if int(np.count_nonzero(inliers)) < 60:
            return None
        point = np.mean(points[inliers], axis=0)
        _, _, vt = np.linalg.svd(points[inliers] - point, full_matrices=False)
        normal = vt[-1]
        if normal[2] > 0.0:
            normal = -normal
        return np.r_[point, normal], float(np.median(z)), int(np.count_nonzero(inliers))

    @staticmethod
    def _native_pixels(
        xs: np.ndarray | float,
        ys: np.ndarray | float,
        intrinsics: CameraIntrinsics,
    ) -> tuple[np.ndarray | float, np.ndarray | float]:
        rotation = int(intrinsics.image_rotation_deg) % 360
        if rotation == 180 and intrinsics.image_width and intrinsics.image_height:
            return (
                intrinsics.image_width - 1 - np.asarray(xs),
                intrinsics.image_height - 1 - np.asarray(ys),
            )
        if rotation == 0:
            return xs, ys
        raise ValueError(f"暂不支持的图像几何旋转：{intrinsics.image_rotation_deg}°")

    @staticmethod
    def _ray_plane(pixel: tuple[int, int], plane: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray | None:
        point, normal = plane[:3], plane[3:]
        native_x, native_y = HorizontalDiameterEstimator._native_pixels(float(pixel[0]), float(pixel[1]), intrinsics)
        ray = np.asarray([(native_x - intrinsics.cx) / intrinsics.fx, (native_y - intrinsics.cy) / intrinsics.fy, 1.0])
        denominator = float(normal @ ray)
        if abs(denominator) < 1e-7:
            return None
        distance = float(normal @ point) / denominator
        return ray * distance if 100.0 < distance < 3000.0 else None

    def estimate(self, heel_mask: np.ndarray, depth: np.ndarray | None, intrinsics: CameraIntrinsics | None) -> dict[str, Any]:
        mask = np.asarray(heel_mask, dtype=bool)
        left, right, center = self.horizontal_endpoints(mask)
        top = self.upper_midpoint(mask, center[0])
        result: dict[str, Any] = {
            "horizontal_diameter": True,
            "center_px": list(center), "contact_left_px": list(left), "contact_right_px": list(right),
            "upper_midpoint_px": list(top),
            "image_axis": [1.0, 0.0], "target_width_range_mm": [50.0, 60.0],
            "accepted_width_range_mm": [self.expected_min_mm, self.expected_max_mm],
            "valid": False, "message": "waiting for aligned depth and CameraInfo",
        }
        if depth is None or intrinsics is None:
            return result
        depth_mm = self._depth_mm(depth)
        if depth_mm.shape != mask.shape:
            result["message"] = "rejected: aligned depth size does not match color mask"
            return result
        foreground = self._foreground_plane(depth_mm, mask, intrinsics)
        if foreground is None:
            result["message"] = "rejected: no coherent foreground heel depth layer"
            return result
        plane, layer_depth, inliers = foreground
        p_left = self._ray_plane(left, plane, intrinsics)
        p_right = self._ray_plane(right, plane, intrinsics)
        p_center = self._ray_plane(center, plane, intrinsics)
        p_top = self._ray_plane(top, plane, intrinsics)
        if p_left is None or p_right is None or p_center is None or p_top is None:
            result["message"] = "rejected: horizontal endpoints cannot meet foreground plane"
            return result
        width = float(np.linalg.norm(p_right - p_left))
        accepted = self.expected_min_mm <= width <= self.expected_max_mm
        point = plane[:3]
        normal = plane[3:]
        result.update({
            "foreground_layer_depth_mm": round(layer_depth, 2), "foreground_plane_inlier_count": inliers,
            "heel_plane_point_camera_mm": [round(float(v), 3) for v in point],
            "heel_plane_normal_camera": [round(float(v), 7) for v in normal],
            "contact_left_camera_mm": [round(float(v), 3) for v in p_left],
            "contact_right_camera_mm": [round(float(v), 3) for v in p_right],
            "center_camera_mm": [round(float(v), 3) for v in p_center],
            "upper_midpoint_camera_mm": [round(float(v), 3) for v in p_top],
            "surface_to_upper_midpoint_gap_mm": round(float(np.linalg.norm(p_center - p_top)), 3),
            "width_mm": round(width, 2), "valid": accepted,
            "message": "horizontal diameter valid" if accepted else "rejected: horizontal diameter is outside 45-65 mm tolerance",
        })
        return result

    @staticmethod
    def draw_overlay(image_bgr: np.ndarray, heel_mask: np.ndarray, result: dict[str, Any]) -> np.ndarray:
        # 仅保留轻量足跟掩膜着色，不绘制青色水平线、端点圆、中心十字等标识
        image = image_bgr.copy(); overlay = image.copy()
        overlay[np.asarray(heel_mask, dtype=bool)] = (40, 190, 70)
        image = cv2.addWeighted(overlay, 0.25, image, 0.75, 0)
        return image
