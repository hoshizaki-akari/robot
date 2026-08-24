#!/usr/bin/env python3
"""M3 traditional OpenCV K-wire centreline fit from saved RGB frames only."""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

import cv2
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]


def unit(vector: np.ndarray) -> np.ndarray:
    length = float(np.linalg.norm(vector))
    if length < 1e-9:
        raise ValueError("零长度种子线")
    return vector / length


def line_distance(points: np.ndarray, origin: np.ndarray, direction: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    delta = points - origin
    projection = delta @ direction
    residual = np.abs(delta[:, 0] * direction[1] - delta[:, 1] * direction[0])
    return projection, residual


def robust_fit(points: np.ndarray, seed_direction: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    if len(points) < 12:
        raise RuntimeError(f"有效边缘点太少：{len(points)}")
    active = points.copy()
    direction = seed_direction.copy()
    center = np.mean(active, axis=0)
    for _ in range(8):
        center = np.mean(active, axis=0)
        _, _, vt = np.linalg.svd(active - center, full_matrices=False)
        direction = unit(vt[0])
        if float(direction @ seed_direction) < 0:
            direction = -direction
        _, residual = line_distance(active, center, direction)
        median = float(np.median(residual))
        mad = float(np.median(np.abs(residual - median)))
        threshold = max(1.25, median + 2.8 * max(mad, 0.25))
        next_active = active[residual <= threshold]
        if len(next_active) == len(active):
            break
        if len(next_active) < 12:
            break
        active = next_active
    _, residual = line_distance(active, center, direction)
    return center, direction, active, float(np.sqrt(np.mean(np.square(residual))))


def candidate_segments(edges: np.ndarray) -> list[tuple[np.ndarray, np.ndarray, str]]:
    segments: list[tuple[np.ndarray, np.ndarray, str]] = []
    lsd = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    result = lsd.detect(edges)
    if result[0] is not None:
        for x1, y1, x2, y2 in result[0].reshape(-1, 4):
            segments.append((np.array([x1, y1], dtype=np.float64), np.array([x2, y2], dtype=np.float64), "LSD"))
    hough = cv2.HoughLinesP(edges, 1, np.pi / 180.0, threshold=18, minLineLength=20, maxLineGap=8)
    if hough is not None:
        for x1, y1, x2, y2 in hough.reshape(-1, 4):
            segments.append((np.array([x1, y1], dtype=np.float64), np.array([x2, y2], dtype=np.float64), "Hough"))
    return segments


def fit_wire(image: np.ndarray, annotation: dict) -> tuple[dict, np.ndarray]:
    x, y, w, h = map(int, annotation["roi_xywh"])
    seed = np.asarray(annotation["seed_line_px"], dtype=np.float64).reshape(2, 2)
    seed_origin, seed_end = seed
    seed_direction = unit(seed_end - seed_origin)
    seed_length = float(np.linalg.norm(seed_end - seed_origin))
    roi = image[y:y+h, x:x+w]
    gray = cv2.cvtColor(roi, cv2.COLOR_BGR2GRAY)
    clahe = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
    scharr_x = cv2.Scharr(clahe, cv2.CV_32F, 1, 0)
    scharr_y = cv2.Scharr(clahe, cv2.CV_32F, 0, 1)
    magnitude = cv2.magnitude(scharr_x, scharr_y)
    magnitude_u8 = cv2.normalize(magnitude, None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
    edges = cv2.Canny(clahe, 35, 100, apertureSize=3, L2gradient=True)
    roi_seed_origin = seed_origin - np.array([x, y], dtype=np.float64)
    roi_seed_end = seed_end - np.array([x, y], dtype=np.float64)
    segment_scores: list[tuple[float, np.ndarray, np.ndarray, str]] = []
    for a, b, source in candidate_segments(edges):
        direction = unit(b - a)
        angle_penalty = 1.0 - abs(float(direction @ seed_direction))
        midpoint = (a + b) * 0.5
        _, distance = line_distance(midpoint.reshape(1, 2), roi_seed_origin, seed_direction)
        length = float(np.linalg.norm(b - a))
        if angle_penalty < 0.08 and float(distance[0]) < 8.0:
            segment_scores.append((length - 50.0 * angle_penalty - float(distance[0]), a, b, source))
    if segment_scores:
        _, _, _, candidate_source = max(segment_scores, key=lambda item: item[0])
    else:
        candidate_source = "seed_fallback"
    yy, xx = np.nonzero(edges)
    points = np.column_stack((xx, yy)).astype(np.float64)
    # Keep all edge support along the manually marked visible wire span.  A
    # short Hough/LSD candidate only validates direction; it must not truncate
    # the edge-point set used for the final robust centreline fit.
    projections, distances = line_distance(points, roi_seed_origin, seed_direction)
    keep = (distances <= 4.0) & (projections >= -10.0) & (projections <= seed_length + 10.0)
    points = points[keep]
    center, direction, inliers, rms = robust_fit(points, seed_direction)
    projection, _ = line_distance(inliers, center, direction)
    # The line itself comes only from true Canny edge inliers.  The manually
    # clicked points delimit the visible needle ends, which avoids losing a
    # low-contrast needle tip to a percentile cut while avoiding a Hough
    # segment as the fitted axis.
    seed_projection, _ = line_distance(np.vstack((roi_seed_origin, roi_seed_end)), center, direction)
    lo, hi = float(np.min(seed_projection)), float(np.max(seed_projection))
    endpoint_a = center + lo * direction + np.array([x, y])
    endpoint_b = center + hi * direction + np.array([x, y])
    center_global = center + np.array([x, y])
    inliers_global = inliers + np.array([x, y])
    overlay = image.copy()
    cv2.rectangle(overlay, (x, y), (x+w, y+h), (0, 255, 255), 1)
    for px, py in inliers_global.astype(np.int32):
        cv2.circle(overlay, (int(px), int(py)), 1, (255, 255, 0), -1)
    a = tuple(np.rint(endpoint_a).astype(int)); b = tuple(np.rint(endpoint_b).astype(int))
    cv2.line(overlay, a, b, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.circle(overlay, a, 4, (0, 255, 0), -1); cv2.circle(overlay, b, 4, (0, 255, 0), -1)
    cv2.putText(overlay, f"wire: {float(hi-lo):.1f}px  rms: {rms:.2f}px", (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(overlay, f"wire: {float(hi-lo):.1f}px  rms: {rms:.2f}px", (12, 25), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (255, 255, 255), 1, cv2.LINE_AA)
    result = {
        "roi_xywh": [x, y, w, h], "seed_line_px": seed.reshape(-1).tolist(), "entry_pixel": annotation.get("entry_pixel", [int(seed_end[0]), int(seed_end[1])]),
        "candidate_source": candidate_source, "algorithm": "Gray + CLAHE + Scharr/Canny + LSD/Hough candidate + edge-point iterative TLS robust fit",
        "inlier_count": int(len(inliers)), "line_center_px": [round(float(v), 3) for v in center_global],
        "line_direction_px": [round(float(v), 7) for v in direction],
        "visible_segment_px": [int(a[0]), int(a[1]), int(b[0]), int(b[1])], "segment_extent_source": "manual_seed_endpoints_on_robust_fit",
        "visible_length_px": round(float(hi - lo), 3), "fit_rms_px": round(rms, 3),
    }
    return result, overlay


def main() -> int:
    parser = argparse.ArgumentParser(description="M3 K-wire 2D centreline detection from saved RGB")
    parser.add_argument("--session", type=Path, required=True)
    args = parser.parse_args()
    annotations = json.loads((args.session / "m3_wire_annotations.json").read_text(encoding="utf-8"))
    output: dict[str, dict] = {}
    for name in ("wire_a", "wire_b"):
        image = cv2.imread(str(args.session / name / "rgb.png"), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(name)
        result, overlay = fit_wire(image, annotations[name])
        file = args.session / f"{name}_overlay.png"
        if not cv2.imwrite(str(file), overlay):
            raise IOError(file)
        output[name] = result
    (args.session / "m3_wire_results.json").write_text(json.dumps(output, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    for name, result in output.items():
        print(f"{name}: length={result['visible_length_px']:.1f}px rms={result['fit_rms_px']:.3f}px inliers={result['inlier_count']}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
