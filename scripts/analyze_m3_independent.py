#!/usr/bin/env python3
"""Independent M3 wire detector.

This file intentionally does not read m3_wire_annotations.json or any marked
image.  It detects long, thin, straight image structures from the original
RGB frame and uses only generic image evidence to choose a candidate.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2
import numpy as np

PROJECT = Path(__file__).resolve().parents[1]


def unit(v: np.ndarray) -> np.ndarray:
    n = float(np.linalg.norm(v))
    if n < 1e-9:
        raise ValueError("zero direction")
    return v / n


def fit_trimmed(points: np.ndarray, seed: np.ndarray) -> tuple[np.ndarray, np.ndarray, np.ndarray, float]:
    if len(points) < 12:
        raise ValueError("too few edge points")
    active = points.copy()
    direction = seed.copy()
    for _ in range(8):
        center = np.mean(active, axis=0)
        _, _, vt = np.linalg.svd(active - center, full_matrices=False)
        direction = unit(vt[0])
        if float(direction @ seed) < 0:
            direction = -direction
        residual = np.abs((active - center)[:, 0] * direction[1] - (active - center)[:, 1] * direction[0])
        med = float(np.median(residual))
        mad = float(np.median(np.abs(residual - med)))
        keep = residual <= max(1.5, med + 2.8 * max(mad, 0.25))
        if int(np.count_nonzero(keep)) < 12 or int(np.count_nonzero(keep)) == len(active):
            break
        active = active[keep]
    center = np.mean(active, axis=0)
    residual = np.abs((active - center)[:, 0] * direction[1] - (active - center)[:, 1] * direction[0])
    return center, direction, active, float(np.sqrt(np.mean(residual * residual)))


def line_points(edges: np.ndarray, origin: np.ndarray, direction: np.ndarray) -> tuple[np.ndarray, np.ndarray]:
    yy, xx = np.nonzero(edges)
    points = np.column_stack((xx, yy)).astype(np.float64)
    delta = points - origin
    projection = delta @ direction
    residual = np.abs(delta[:, 0] * direction[1] - delta[:, 1] * direction[0])
    h, w = edges.shape
    corners = np.array([[0, 0], [w - 1, 0], [0, h - 1], [w - 1, h - 1]], dtype=np.float64)
    corner_proj = (corners - origin) @ direction
    keep = (residual <= 3.0) & (projection >= float(corner_proj.min()) - 2) & (projection <= float(corner_proj.max()) + 2)
    return points[keep], projection[keep]


def coverage_score(projection: np.ndarray, length: float) -> float:
    if len(projection) < 2 or length < 1:
        return 0.0
    bins = max(8, int(length / 8.0))
    lo, hi = float(projection.min()), float(projection.max())
    occupied, _ = np.histogram(projection, bins=bins, range=(lo, hi))
    return float(np.count_nonzero(occupied)) / bins


def contrast_score(gray: np.ndarray, center: np.ndarray, direction: np.ndarray, lo: float, hi: float) -> float:
    normal = np.array([-direction[1], direction[0]])
    values_center: list[float] = []
    values_side: list[float] = []
    h, w = gray.shape
    for projection in np.linspace(lo, hi, max(20, int((hi - lo) / 3.0))):
        point = center + direction * projection
        px, py = np.rint(point).astype(int)
        if not (4 <= px < w - 4 and 4 <= py < h - 4):
            continue
        center_samples = [gray[int(np.clip(py + normal[1] * offset, 0, h - 1)), int(np.clip(px + normal[0] * offset, 0, w - 1))] for offset in (-1, 0, 1)]
        left = gray[int(np.clip(py + normal[1] * -4, 0, h - 1)), int(np.clip(px + normal[0] * -4, 0, w - 1))]
        right = gray[int(np.clip(py + normal[1] * 4, 0, h - 1)), int(np.clip(px + normal[0] * 4, 0, w - 1))]
        values_center.append(float(np.mean(center_samples)))
        # Both sides must be brighter than the narrow centre. This rejects
        # one-sided table/box boundaries that otherwise look like long wires.
        values_side.append(float(min(left, right) - np.mean(center_samples)))
    if not values_center:
        return 0.0
    return float(np.clip(np.mean(values_side) / 50.0, -1.0, 1.0))


def skin_mask(image: np.ndarray) -> np.ndarray:
    b, g, r = [channel.astype(np.int16) for channel in cv2.split(image)]
    hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    # Broad but chromatic flesh cue; the green background and grey box do not
    # pass the red-over-green and low-hue tests. No fixed image coordinates.
    mask = (((hsv[:, :, 0] <= 20) | (hsv[:, :, 0] >= 170)) & (hsv[:, :, 1] >= 35) & (hsv[:, :, 2] >= 70) & (r > g + 15) & (g > b + 3)).astype(np.uint8)
    kernel = np.ones((5, 5), np.uint8)
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, kernel, iterations=2)


def skin_support(mask: np.ndarray, center: np.ndarray, direction: np.ndarray, lo: float, hi: float) -> tuple[float, int]:
    h, w = mask.shape
    count = 0
    samples = max(20, int(abs(hi - lo)))
    values: list[int] = []
    for projection in np.linspace(lo, hi, samples):
        px, py = np.rint(center + direction * projection).astype(int)
        if 0 <= px < w and 0 <= py < h:
            values.append(int(mask[py, px]))
    if not values:
        return 0.0, 0
    for before, after in zip(values, values[1:]):
        count += int(before != after)
    return float(np.mean(values)), count


def clip_at_flesh_boundary(mask: np.ndarray, center: np.ndarray, direction: np.ndarray, lo: float, hi: float) -> tuple[float, float]:
    """Remove the hidden-in-flesh continuation from a visible wire segment."""
    count = max(40, int(abs(hi - lo) * 1.5))
    projections = np.linspace(lo, hi, count)
    h, w = mask.shape
    values = []
    for projection in projections:
        px, py = np.rint(center + direction * projection).astype(int)
        values.append(int(mask[np.clip(py, 0, h - 1), np.clip(px, 0, w - 1)]))
    values = np.asarray(values, dtype=np.uint8)
    # If flesh occupies one end of the line, use the first background/flesh
    # boundary as the visible entry point. Interior flesh runs are not used.
    if float(np.mean(values[:min(12, count)])) > 0.5:
        transitions = np.where((values[:-1] == 1) & (values[1:] == 0))[0]
        if len(transitions):
            lo = float(projections[int(transitions[0] + 1)])
    elif float(np.mean(values[-min(12, count):])) > 0.5:
        transitions = np.where((values[:-1] == 0) & (values[1:] == 1))[0]
        if len(transitions):
            hi = float(projections[int(transitions[-1])])
    return min(lo, hi), max(lo, hi)


def candidates(edges: np.ndarray) -> list[tuple[np.ndarray, np.ndarray, str]]:
    result: list[tuple[np.ndarray, np.ndarray, str]] = []
    detector = cv2.createLineSegmentDetector(cv2.LSD_REFINE_STD)
    detected = detector.detect(edges)[0]
    if detected is not None:
        for x1, y1, x2, y2 in detected.reshape(-1, 4):
            a, b = np.array([x1, y1]), np.array([x2, y2])
            if np.linalg.norm(b - a) >= 18:
                result.append((a, b, "LSD"))
    hough = cv2.HoughLinesP(edges, 1, np.pi / 360.0, threshold=16, minLineLength=18, maxLineGap=12)
    if hough is not None:
        for x1, y1, x2, y2 in hough.reshape(-1, 4):
            a, b = np.array([x1, y1], dtype=np.float64), np.array([x2, y2], dtype=np.float64)
            if np.linalg.norm(b - a) >= 18:
                result.append((a, b, "Hough"))
    return result


def detect(image: np.ndarray) -> tuple[dict, np.ndarray]:
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    enhanced = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
    edges = cv2.Canny(enhanced, 30, 95, apertureSize=3, L2gradient=True)
    flesh = skin_mask(image)
    scored: list[tuple[float, dict, np.ndarray]] = []
    for a, b, source in candidates(edges):
        direction = unit(b - a)
        origin = (a + b) * 0.5
        points, projection = line_points(edges, origin, direction)
        if len(points) < 20:
            continue
        try:
            center, direction, inliers, rms = fit_trimmed(points, direction)
        except ValueError:
            continue
        fitted_projection = (inliers - center) @ direction
        lo, hi = float(np.percentile(fitted_projection, 2)), float(np.percentile(fitted_projection, 98))
        length = hi - lo
        coverage = coverage_score(fitted_projection, length)
        contrast = contrast_score(gray, center, direction, lo, hi)
        flesh_fraction, transitions = skin_support(flesh, center, direction, lo, hi)
        # Generic physical prior: a K-wire is long, straight, narrow, and
        # continuously supported by two nearby image edges. No image location
        # or annotation coordinate is used here.
        # A true wire should enter/terminate at the flesh region, unlike a
        # tabletop or storage-box edge.  The transition term rewards a line
        # crossing between background and flesh, without using the annotation.
        score = 0.020 * length + 35.0 * coverage + 180.0 * contrast + 35.0 * flesh_fraction + 8.0 * min(transitions, 4) - 12.0 * min(rms, 4.0)
        lo, hi = clip_at_flesh_boundary(flesh, center, direction, lo, hi)
        visible_mask = (fitted_projection >= lo) & (fitted_projection <= hi)
        visible_inliers = inliers[visible_mask] if int(np.count_nonzero(visible_mask)) >= 12 else inliers
        visible_residual = np.abs((visible_inliers - center)[:, 0] * direction[1] - (visible_inliers - center)[:, 1] * direction[0])
        visible_rms = float(np.sqrt(np.mean(visible_residual * visible_residual)))
        pa = np.rint(center + lo * direction).astype(int); pb = np.rint(center + hi * direction).astype(int)
        result = {"candidate_source": source, "score": round(float(score), 3), "inlier_count": int(len(inliers)), "line_center_px": [round(float(v), 3) for v in center], "line_direction_px": [round(float(v), 7) for v in direction], "visible_segment_px": [int(pa[0]), int(pa[1]), int(pb[0]), int(pb[1])], "flesh_fraction": round(flesh_fraction, 3), "flesh_transitions": transitions}
        result["visible_segment_px"] = [int(pa[0]), int(pa[1]), int(pb[0]), int(pb[1])]
        result["visible_length_px"] = round(float(np.linalg.norm(pb - pa)), 3); result["fit_rms_px"] = round(visible_rms, 3); result["coverage"] = round(coverage, 3); result["contrast"] = round(contrast, 3)
        overlay = image.copy()
        cv2.line(overlay, tuple(pa), tuple(pb), (0, 0, 255), 2, cv2.LINE_AA)
        for px, py in visible_inliers.astype(int):
            cv2.circle(overlay, (int(px), int(py)), 1, (255, 255, 0), -1)
        scored.append((score, result, overlay))
    if not scored:
        raise RuntimeError("未找到满足连续直线约束的候选")
    score, result, overlay = max(scored, key=lambda item: item[0])
    result["algorithm"] = "full-frame Gray + CLAHE + Canny + LSD/Hough candidates + edge-point trimmed TLS robust fit"
    result["annotation_independent"] = True
    cv2.putText(overlay, f"independent wire {result['visible_length_px']:.1f}px rms {result['fit_rms_px']:.2f}px", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(overlay, f"independent wire {result['visible_length_px']:.1f}px rms {result['fit_rms_px']:.2f}px", (8, 22), cv2.FONT_HERSHEY_SIMPLEX, 0.48, (255, 255, 255), 1, cv2.LINE_AA)
    return result, overlay


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("--session", type=Path, required=True)
    args = parser.parse_args()
    results = {}
    for name in ("wire_a", "wire_b"):
        image = cv2.imread(str(args.session / name / "rgb.png"), cv2.IMREAD_COLOR)
        if image is None:
            raise FileNotFoundError(name)
        result, overlay = detect(image)
        results[name] = result
        cv2.imwrite(str(args.session / f"{name}_overlay_independent.png"), overlay)
        print(name, result)
    (args.session / "m3_wire_results_independent.json").write_text(json.dumps(results, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
