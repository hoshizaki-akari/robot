"""Depth-guided heel candidate extraction for the live D435 view.

The deployed heel model was trained on larger heel crops than the current
424x240 full frame.  This module does not invent a semantic result from
depth alone: it only proposes a coherent, near, central component which is
then checked by the existing horizontal-diameter geometry and depth plane
fit.  Motion remains gated by the planner until the physical result is
accepted.
"""
from __future__ import annotations

from dataclasses import dataclass
from typing import Any

import cv2
import numpy as np

try:
    from .horizontal_diameter import CameraIntrinsics, HorizontalDiameterEstimator
except ImportError:  # allow direct script/module execution
    from horizontal_diameter import CameraIntrinsics, HorizontalDiameterEstimator


@dataclass(frozen=True)
class DepthHeelConfig:
    min_component_area_px: int = 1200
    min_component_width_px: int = 30
    min_component_height_px: int = 60
    # The current scene has a 480--650 mm heel and a roughly 750 mm shelf.
    # A 50 mm gap admits the shelf into the same component when the border
    # estimate is around 800 mm, so keep a conservative separation.
    min_depth_gap_mm: float = 80.0
    max_depth_gap_mm: float = 140.0
    centrality_limit: float = 0.42


def _valid_depth(depth_mm: np.ndarray) -> np.ndarray:
    depth = np.asarray(depth_mm, dtype=np.float32)
    return np.isfinite(depth) & (depth >= 100.0) & (depth <= 3000.0)


def _background_depth(depth_mm: np.ndarray, valid: np.ndarray) -> float | None:
    height, width = depth_mm.shape
    edge = max(8, int(round(width * 0.12)))
    top = max(8, int(round(height * 0.10)))
    border = np.concatenate(
        (
            depth_mm[:, :edge][valid[:, :edge]],
            depth_mm[:, width - edge :][valid[:, width - edge :]],
            depth_mm[:top, :][valid[:top, :]],
        )
    )
    if border.size < 100:
        return None
    # The background is the farther, stable layer.  A high percentile keeps
    # a near object at an image edge from lowering the threshold too much.
    return float(np.percentile(border, 70.0))


def extract_depth_heel_candidate(
    depth_mm: np.ndarray,
    config: DepthHeelConfig | None = None,
) -> tuple[np.ndarray | None, dict[str, Any]]:
    """Return one coherent central near-depth component and diagnostics."""

    cfg = config or DepthHeelConfig()
    depth = np.asarray(depth_mm, dtype=np.float32)
    if depth.ndim != 2:
        return None, {"message": "aligned depth must be a 2-D image"}
    valid = _valid_depth(depth)
    background = _background_depth(depth, valid)
    if background is None:
        return None, {"message": "not enough valid border depth"}
    gap = float(np.clip(background * 0.08, cfg.min_depth_gap_mm, cfg.max_depth_gap_mm))
    threshold = background - gap
    near = valid & (depth <= threshold)
    kernel = np.ones((3, 3), dtype=np.uint8)
    near = cv2.morphologyEx(near.astype(np.uint8), cv2.MORPH_OPEN, kernel)
    near = cv2.morphologyEx(near, cv2.MORPH_CLOSE, kernel)

    count, labels, stats, _ = cv2.connectedComponentsWithStats(near, 8)
    height, width = depth.shape
    candidates: list[tuple[float, int, tuple[int, int, int, int]]] = []
    image_center = width * 0.5
    for index in range(1, count):
        x, y, w, h, area = [int(value) for value in stats[index]]
        if area < cfg.min_component_area_px:
            continue
        if w < cfg.min_component_width_px or h < cfg.min_component_height_px:
            continue
        center_x = x + w * 0.5
        centrality = abs(center_x - image_center) / max(1.0, image_center)
        if centrality > cfg.centrality_limit:
            continue
        fill_ratio = area / float(w * h)
        if fill_ratio < 0.25:
            continue
        # Prefer a large, compact, central near component.  The score is only
        # for selecting a candidate; it is never exposed as a confidence.
        score = float(area) * (1.0 - centrality) * min(1.0, fill_ratio * 1.5)
        candidates.append((score, index, (x, y, w, h)))

    if not candidates:
        return None, {
            "message": "no central coherent near-depth component",
            "background_depth_mm": round(background, 2),
            "near_depth_threshold_mm": round(threshold, 2),
        }
    score, index, bbox = max(candidates, key=lambda item: item[0])
    selected = labels == index
    x, y, w, h = bbox
    area = int(np.count_nonzero(selected))
    return selected, {
        "message": "depth-guided heel candidate",
        "background_depth_mm": round(background, 2),
        "near_depth_threshold_mm": round(threshold, 2),
        "component_area_px": area,
        "component_bbox_px": [x, y, w, h],
        "component_score": round(score, 2),
    }


def estimate_depth_guided_target_chord(
    heel_mask: np.ndarray,
    depth_mm: np.ndarray,
    intrinsics: CameraIntrinsics,
    estimator: HorizontalDiameterEstimator | None = None,
    target_min_mm: float = 50.0,
    target_max_mm: float = 60.0,
) -> dict[str, Any]:
    """Estimate a usable upper heel chord instead of the whole-foot width.

    A depth component can contain the full foot, whose middle row is much
    wider than the intended clamp section.  Use a fixed heel-relative row and
    a symmetric chord around the component median, matching the round heel
    location learned by the trained detector.  This prevents the contacts
    from jumping between unrelated rows when a depth hole or a nearby
    background object changes by one frame.
    """

    geometry = estimator or HorizontalDiameterEstimator()
    base = geometry.estimate(heel_mask, depth_mm, intrinsics)
    mask = np.asarray(heel_mask, dtype=bool)
    ys, xs = np.nonzero(mask)
    if xs.size < 80:
        return base
    foreground = geometry._foreground_plane(depth_mm, mask, intrinsics)
    if foreground is None:
        return base
    plane = foreground[0]
    top_y = int(np.min(ys))
    bottom_y = int(np.max(ys))
    component_height = max(1, bottom_y - top_y)
    # The full depth component contains the foot below the heel.  The actual
    # round heel in the current view is around one third of the component
    # height, not at its top edge.  Use the component median x and a fixed
    # relative y so the fallback agrees with the trained YOLO target.
    center_x = int(round(float(np.median(xs))))
    y = top_y + int(round(component_height * 0.32))
    point_center = geometry._ray_plane((center_x, y), plane, intrinsics)
    if point_center is None:
        return base
    max_half_px = min(center_x, mask.shape[1] - 1 - center_x, 40)
    if max_half_px < 8:
        return base
    target_mid_mm = (target_min_mm + target_max_mm) * 0.5
    chord_candidates: list[tuple[float, int, np.ndarray, np.ndarray, float]] = []
    for half_px in range(8, max_half_px + 1):
        point_left = geometry._ray_plane((center_x - half_px, y), plane, intrinsics)
        point_right = geometry._ray_plane((center_x + half_px, y), plane, intrinsics)
        if point_left is None or point_right is None:
            continue
        width = float(np.linalg.norm(point_right - point_left))
        in_target = target_min_mm <= width <= target_max_mm
        score = abs(width - target_mid_mm) + (0.0 if in_target else 100.0)
        chord_candidates.append((score, half_px, point_left, point_right, width))
    if not chord_candidates:
        return base
    _, half_px, point_left, point_right, width = min(
        chord_candidates, key=lambda item: item[0]
    )
    center_pixel = (center_x, y)
    point_center = geometry._ray_plane(center_pixel, plane, intrinsics)
    if point_center is None:
        return base
    top_pixel = geometry.upper_midpoint(mask, center_pixel[0])
    point_top = geometry._ray_plane(top_pixel, plane, intrinsics)
    if point_top is None:
        return base
    width = float(np.linalg.norm(point_right - point_left))
    result = dict(base)
    result.update(
        {
            "center_px": [center_pixel[0], center_pixel[1]],
            "contact_left_px": [center_x - half_px, y],
            "contact_right_px": [center_x + half_px, y],
            "center_camera_mm": [round(float(v), 3) for v in point_center],
            "contact_left_camera_mm": [round(float(v), 3) for v in point_left],
            "contact_right_camera_mm": [round(float(v), 3) for v in point_right],
            "width_mm": round(width, 2),
            "valid": bool(geometry.expected_min_mm <= width <= geometry.expected_max_mm),
            "target_width_range_mm": [target_min_mm, target_max_mm],
            "selected_chord_y_px": y,
            "selected_chord_width_mm": round(width, 2),
            "target_circle_center_px": [center_x, y],
            "target_circle_radius_px": half_px,
            "surface_to_upper_midpoint_gap_mm": round(
                float(np.linalg.norm(point_center - point_top)), 3
            ),
            "message": (
                "depth-guided target chord valid"
                if geometry.expected_min_mm <= width <= geometry.expected_max_mm
                else "rejected: depth-guided target chord is outside 45-65 mm tolerance"
            ),
        }
    )
    return result


def build_target_display_mask(
    heel_mask: np.ndarray,
    center_px: list[int] | tuple[int, int] | None,
    radius_px: int | float,
) -> np.ndarray:
    """Limit the annotated overlay to the selected, circular heel target."""

    mask = np.asarray(heel_mask, dtype=bool)
    if mask.ndim != 2 or center_px is None or len(center_px) < 2:
        return np.zeros(mask.shape, dtype=bool)
    cx, cy = int(round(float(center_px[0]))), int(round(float(center_px[1])))
    radius = max(1, int(round(float(radius_px))))
    circle = np.zeros(mask.shape, dtype=np.uint8)
    cv2.circle(circle, (cx, cy), radius, 1, thickness=-1)
    return mask & circle.astype(bool)


def refine_target_chord_at_center(
    heel_mask: np.ndarray,
    depth_mm: np.ndarray,
    intrinsics: CameraIntrinsics,
    result: dict[str, Any],
    estimator: HorizontalDiameterEstimator | None = None,
    target_min_mm: float = 50.0,
    target_max_mm: float = 60.0,
) -> dict[str, Any]:
    """Keep a YOLO center/row but choose a symmetric 50--60 mm chord."""

    geometry = estimator or HorizontalDiameterEstimator()
    center = result.get("center_px")
    mask = np.asarray(heel_mask, dtype=bool)
    if center is None or mask.ndim != 2:
        return result
    foreground = geometry._foreground_plane(depth_mm, mask, intrinsics)
    if foreground is None:
        return result
    plane = foreground[0]
    center_x, center_y = int(center[0]), int(center[1])
    point_center = geometry._ray_plane((center_x, center_y), plane, intrinsics)
    if point_center is None:
        return result
    max_half_px = min(center_x, mask.shape[1] - 1 - center_x, 40)
    candidates: list[tuple[float, int, np.ndarray, np.ndarray, float]] = []
    target_mid_mm = (target_min_mm + target_max_mm) * 0.5
    for half_px in range(8, max_half_px + 1):
        point_left = geometry._ray_plane(
            (center_x - half_px, center_y), plane, intrinsics
        )
        point_right = geometry._ray_plane(
            (center_x + half_px, center_y), plane, intrinsics
        )
        if point_left is None or point_right is None:
            continue
        width = float(np.linalg.norm(point_right - point_left))
        in_target = target_min_mm <= width <= target_max_mm
        score = abs(width - target_mid_mm) + (0.0 if in_target else 100.0)
        candidates.append((score, half_px, point_left, point_right, width))
    if not candidates:
        return result
    _, half_px, point_left, point_right, width = min(
        candidates, key=lambda item: item[0]
    )
    top_pixel = result.get("upper_midpoint_px") or geometry.upper_midpoint(
        mask, center_x
    )
    point_top = geometry._ray_plane(tuple(top_pixel), plane, intrinsics)
    if point_top is None:
        return result
    refined = dict(result)
    refined.update(
        {
            "contact_left_px": [center_x - half_px, center_y],
            "contact_right_px": [center_x + half_px, center_y],
            "contact_left_camera_mm": [round(float(v), 3) for v in point_left],
            "contact_right_camera_mm": [round(float(v), 3) for v in point_right],
            "center_camera_mm": [round(float(v), 3) for v in point_center],
            "width_mm": round(width, 2),
            "valid": bool(geometry.expected_min_mm <= width <= geometry.expected_max_mm),
            "target_width_range_mm": [target_min_mm, target_max_mm],
            "target_circle_center_px": [center_x, center_y],
            "target_circle_radius_px": half_px,
            "surface_to_upper_midpoint_gap_mm": round(
                float(np.linalg.norm(point_center - point_top)), 3
            ),
        }
    )
    return refined
