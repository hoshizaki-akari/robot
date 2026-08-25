"""Pose-robust heel width geometry based on mask PCA and a local depth plane."""
from __future__ import annotations

from typing import Any

import cv2
import numpy as np

from .horizontal_diameter import CameraIntrinsics


class HeelGeometryEstimator:
    """Estimate a width chord perpendicular to the mask principal axis."""

    def __init__(self, expected_min_mm: float = 45.0, expected_max_mm: float = 65.0) -> None:
        if not 0.0 < expected_min_mm < expected_max_mm:
            raise ValueError("invalid expected diameter range")
        self.expected_min_mm = float(expected_min_mm)
        self.expected_max_mm = float(expected_max_mm)

    @staticmethod
    def _largest_component(mask: np.ndarray) -> np.ndarray:
        binary = (np.asarray(mask, dtype=np.uint8) * 255)
        kernel = np.ones((3, 3), np.uint8)
        binary = cv2.morphologyEx(binary, cv2.MORPH_CLOSE, kernel, iterations=1)
        count, labels, stats, _ = cv2.connectedComponentsWithStats(binary, 8)
        if count <= 1:
            return binary > 0
        index = 1 + int(np.argmax(stats[1:, cv2.CC_STAT_AREA]))
        return labels == index

    @staticmethod
    def _axes(mask: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, np.ndarray]:
        ys, xs = np.nonzero(mask)
        if xs.size < 80:
            raise ValueError("heel mask has too few pixels")
        points = np.column_stack((xs.astype(np.float64), ys.astype(np.float64)))
        center = np.mean(points, axis=0)
        covariance = np.cov(points - center, rowvar=False)
        values, vectors = np.linalg.eigh(covariance)
        u = vectors[:, int(np.argmax(values))]
        u = u / max(float(np.linalg.norm(u)), 1e-12)
        if u[0] < 0.0 or (abs(u[0]) < 1e-9 and u[1] < 0.0):
            u = -u
        v = np.asarray([-u[1], u[0]], dtype=np.float64)
        projections = (points - center) @ np.column_stack((u, v))
        return center, u, v, projections

    @staticmethod
    def _candidate_pixels(mask: np.ndarray, projections: np.ndarray) -> list[tuple[tuple[int, int], tuple[int, int], tuple[int, int]]]:
        ys, xs = np.nonzero(mask)
        length = max(float(np.ptp(projections[:, 0])), 1.0)
        band = max(2.0, length * 0.025)
        candidates = []
        for fraction in (0.35, 0.425, 0.5, 0.575, 0.65):
            target = float(np.min(projections[:, 0]) + length * fraction)
            selected = np.abs(projections[:, 0] - target) <= band
            if int(np.count_nonzero(selected)) < 10:
                continue
            selected_v = projections[selected, 1]
            low = float(np.quantile(selected_v, 0.03))
            high = float(np.quantile(selected_v, 0.97))
            pixel_points = np.column_stack((xs[selected], ys[selected])).astype(np.float64)
            low_px = pixel_points[np.argmin(np.abs(selected_v - low))]
            high_px = pixel_points[np.argmin(np.abs(selected_v - high))]
            center_px = (low_px + high_px) * 0.5
            candidates.append((
                (int(round(low_px[0])), int(round(low_px[1]))),
                (int(round(high_px[0])), int(round(high_px[1]))),
                (int(round(center_px[0])), int(round(center_px[1]))),
            ))
        if not candidates:
            raise ValueError("heel mask has no principal-axis width section")
        return candidates

    @staticmethod
    def _depth_mm(depth: np.ndarray) -> np.ndarray:
        values = np.asarray(depth, dtype=np.float32)
        positive = values[np.isfinite(values) & (values > 0)]
        return values * 1000.0 if positive.size and float(np.median(positive)) < 10.0 else values

    @staticmethod
    def _fit_local_plane(depth_mm: np.ndarray, mask: np.ndarray, intrinsics: CameraIntrinsics, center_px: tuple[int, int], width_px: float) -> tuple[np.ndarray, float, int, float] | None:
        yy, xx = np.indices(mask.shape)
        radius = max(width_px * 1.4, 35.0)
        local = ((xx - center_px[0]) ** 2 + (yy - center_px[1]) ** 2) <= radius ** 2
        mask_count = int(np.count_nonzero(mask & local))
        valid = mask & local & np.isfinite(depth_mm) & (depth_mm > 100.0) & (depth_mm < 3000.0)
        depth_valid_ratio = float(np.count_nonzero(valid) / max(mask_count, 1))
        ys, xs = np.nonzero(valid)
        if xs.size < 70:
            return None
        z = depth_mm[ys, xs].astype(np.float64)
        # D435 contour samples frequently contain a second, farther depth
        # layer. Select the nearest coherent local layer before fitting; a
        # single SVD over mixed foreground/background points can turn a
        # 55-mm heel width into hundreds of millimetres.
        edges = np.arange(100.0, 3010.0, 10.0)
        counts, _ = np.histogram(z, bins=edges)
        populated = np.flatnonzero(counts >= max(30, int(z.size * 0.04)))
        if populated.size == 0:
            return None
        layer_depth = float((edges[int(populated[0])] + edges[int(populated[0]) + 1]) * 0.5)
        layer = np.abs(z - layer_depth) <= 35.0
        if int(np.count_nonzero(layer)) < 70:
            return None
        xs, ys, z = xs[layer], ys[layer], z[layer]
        points = np.column_stack(((xs - intrinsics.cx) * z / intrinsics.fx, (ys - intrinsics.cy) * z / intrinsics.fy, z))
        seed = np.mean(points, axis=0)
        _, _, vt = np.linalg.svd(points - seed, full_matrices=False)
        normal = vt[-1]
        residual = np.abs((points - seed) @ normal)
        inliers = residual <= max(3.0, float(np.percentile(residual, 80)))
        if int(np.count_nonzero(inliers)) < 60:
            return None
        point = np.mean(points[inliers], axis=0)
        _, _, vt = np.linalg.svd(points[inliers] - point, full_matrices=False)
        normal = vt[-1]
        if normal[2] > 0.0:
            normal = -normal
        plane = np.r_[point, normal / max(float(np.linalg.norm(normal)), 1e-12)]
        return plane, float(np.median(z[inliers])), int(np.count_nonzero(inliers)), depth_valid_ratio

    @staticmethod
    def _ray_plane(pixel: tuple[int, int], plane: np.ndarray, intrinsics: CameraIntrinsics) -> np.ndarray | None:
        point, normal = plane[:3], plane[3:]
        ray = np.asarray([(pixel[0] - intrinsics.cx) / intrinsics.fx, (pixel[1] - intrinsics.cy) / intrinsics.fy, 1.0])
        denominator = float(normal @ ray)
        if abs(denominator) < 1e-7:
            return None
        distance = float(normal @ point) / denominator
        return ray * distance if 100.0 < distance < 3000.0 else None

    def estimate(self, heel_mask: np.ndarray, depth: np.ndarray | None, intrinsics: CameraIntrinsics | None) -> dict[str, Any]:
        mask = self._largest_component(heel_mask)
        center_2d, u, v, projections = self._axes(mask)
        candidates = self._candidate_pixels(mask, projections)
        first_a, first_b, first_c = candidates[len(candidates) // 2]
        ys = np.nonzero(mask)[0]
        result: dict[str, Any] = {
            "horizontal_diameter": False,
            "center_px": list(first_c),
            "contact_left_px": list(first_a),
            "contact_right_px": list(first_b),
            "upper_midpoint_px": [int(round(center_2d[0])), int(np.min(ys))],
            "image_axis": [round(float(u[0]), 7), round(float(u[1]), 7)],
            "principal_axis_image": [round(float(u[0]), 7), round(float(u[1]), 7)],
            "width_axis_image": [round(float(v[0]), 7), round(float(v[1]), 7)],
            "principal_angle_deg": round(float(np.degrees(np.arctan2(u[1], u[0]))), 4),
            "target_width_range_mm": [50.0, 60.0],
            "accepted_width_range_mm": [self.expected_min_mm, self.expected_max_mm],
            "geometry_valid": False,
            "stable_valid": False,
            "within_expected_width_range": False,
            "motion_grade": False,
            "valid": False,
            "display_only": False,
            "measurement_status": "raw_invalid",
            "message": "等待对齐深度和相机内参",
        }
        if depth is None or intrinsics is None:
            return result
        depth_mm = self._depth_mm(depth)
        if depth_mm.shape != mask.shape:
            result["message"] = "拒绝：深度尺寸与彩色掩膜不一致"
            return result
        width_px = float(np.ptp(projections[:, 1]))
        plane_info = self._fit_local_plane(depth_mm, mask, intrinsics, first_c, width_px)
        if plane_info is None:
            result["message"] = "拒绝：夹持中心附近没有足够的有效局部深度平面"
            return result
        plane, layer_depth, inliers, depth_valid_ratio = plane_info
        measured = []
        for pixel_a, pixel_b, pixel_c in candidates:
            p_a = self._ray_plane(pixel_a, plane, intrinsics)
            p_b = self._ray_plane(pixel_b, plane, intrinsics)
            p_c = self._ray_plane(pixel_c, plane, intrinsics)
            if p_a is None or p_b is None or p_c is None:
                continue
            measured.append((float(np.linalg.norm(p_b - p_a)), pixel_a, pixel_b, pixel_c, p_a, p_b, p_c))
        if not measured:
            result["message"] = "拒绝：主轴宽度端点无法与局部深度平面求交"
            return result
        in_range = [item for item in measured if self.expected_min_mm <= item[0] <= self.expected_max_mm]
        width, pixel_a, pixel_b, pixel_c, p_a, p_b, p_c = min(in_range or measured, key=lambda item: abs(item[0] - 57.5))
        top_px = result["upper_midpoint_px"]
        p_top = self._ray_plane((int(top_px[0]), int(top_px[1])), plane, intrinsics)
        if p_top is None:
            p_top = p_c
        accepted = self.expected_min_mm <= width <= self.expected_max_mm
        result.update({
            "center_px": list(pixel_c), "contact_left_px": list(pixel_a), "contact_right_px": list(pixel_b),
            "foreground_layer_depth_mm": round(layer_depth, 2), "foreground_plane_inlier_count": inliers,
            "plane_inlier_ratio": round(float(inliers / max(np.count_nonzero(mask), 1)), 5),
            "depth_valid_ratio": round(depth_valid_ratio, 5),
            "heel_plane_point_camera_mm": [round(float(vv), 3) for vv in plane[:3]],
            "heel_plane_normal_camera": [round(float(vv), 7) for vv in plane[3:]],
            "contact_left_camera_mm": [round(float(vv), 3) for vv in p_a],
            "contact_right_camera_mm": [round(float(vv), 3) for vv in p_b],
            "center_camera_mm": [round(float(vv), 3) for vv in p_c],
            "upper_midpoint_camera_mm": [round(float(vv), 3) for vv in p_top],
            "surface_to_upper_midpoint_gap_mm": round(float(np.linalg.norm(p_c - p_top)), 3),
            "width_mm": round(width, 2), "within_expected_width_range": accepted,
            "geometry_valid": True, "valid": accepted,
            "message": "主轴宽度几何有效" if accepted else "几何有效，但宽度超出45～65mm预期范围",
        })
        return result

    @staticmethod
    def draw_overlay(image_bgr: np.ndarray, heel_mask: np.ndarray, result: dict[str, Any]) -> np.ndarray:
        image = image_bgr.copy()
        overlay = image.copy()
        overlay[np.asarray(heel_mask, dtype=bool)] = (40, 190, 70)
        return cv2.addWeighted(overlay, 0.25, image, 0.75, 0)
