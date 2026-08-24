"""撬拨观察位的独立二维针线检测。

这个模块不控制机器人，也不参与夹挤流程。输入一张普通 RGB 图像，
输出足跟区域、针孔二维点和针线二维线段，并可生成调试图。
"""

from __future__ import annotations

import math
from pathlib import Path
from typing import Any

import cv2
import numpy as np


class PivotNeedleVision:
    def __init__(self, model_path: str | Path, confidence: float = 0.35) -> None:
        self.model_path = Path(model_path)
        self.confidence = confidence
        self._model: Any | None = None

    def _heel_mask(self, image: np.ndarray) -> np.ndarray:
        if self._model is None:
            from ultralytics import YOLO

            self._model = YOLO(str(self.model_path))
        prediction = self._model.predict(
            image, imgsz=640, conf=self.confidence,
            retina_masks=True, verbose=False, device="cpu"
        )[0]
        if prediction.masks is None or len(prediction.boxes) == 0:
            # 撬拨观察角度与夹挤角度不同，模型可能没有足跟类别结果。
            # 此时只用肤色连通区作为实验性候选，不影响原平台。
            hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
            skin = (((hsv[:, :, 0] <= 25) | (hsv[:, :, 0] >= 165)) &
                    (hsv[:, :, 1] >= 18) & (hsv[:, :, 1] <= 210) &
                    (hsv[:, :, 2] >= 45)).astype(np.uint8)
            skin = cv2.morphologyEx(skin, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
            count, labels, stats, _ = cv2.connectedComponentsWithStats(skin, 8)
            if count <= 1:
                raise ValueError("没有找到足跟区域")
            label = max(range(1, count), key=lambda i: int(stats[i, cv2.CC_STAT_AREA]))
            return labels == label
        scores = prediction.boxes.conf.detach().cpu().numpy()
        index = int(np.argmax(scores))
        raw = prediction.masks.data[index].detach().cpu().numpy()
        h, w = image.shape[:2]
        return cv2.resize(raw, (w, h), interpolation=cv2.INTER_NEAREST) > 0.5

    @staticmethod
    def _line_candidates(image: np.ndarray, mask: np.ndarray, puncture: tuple[int, int]) -> list[tuple[float, tuple[int, int, int, int]]]:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        roi = cv2.dilate(mask.astype(np.uint8), np.ones((5, 5), np.uint8), iterations=18)
        # 金属针可能比皮肤亮，也可能因反光呈暗线；两种响应都保留。
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (13, 13))
        bright = cv2.morphologyEx(gray, cv2.MORPH_TOPHAT, kernel)
        dark = cv2.morphologyEx(gray, cv2.MORPH_BLACKHAT, kernel)
        response = np.maximum(bright, dark)
        response[roi == 0] = 0
        response = cv2.normalize(response, None, 0, 255, cv2.NORM_MINMAX)
        _, binary = cv2.threshold(response, 0, 255, cv2.THRESH_BINARY + cv2.THRESH_OTSU)
        binary = cv2.morphologyEx(binary, cv2.MORPH_OPEN, np.ones((2, 2), np.uint8))
        edges = cv2.Canny(gray, 45, 130)
        edges[roi == 0] = 0
        combined = cv2.bitwise_or(binary, edges)
        lines = cv2.HoughLinesP(combined, 1, np.pi / 180, 12, minLineLength=18, maxLineGap=20)
        if lines is None:
            return []
        ys, xs = np.nonzero(mask)
        center = np.array([xs.mean(), ys.mean()], dtype=np.float32)
        width = max(10.0, float(xs.max() - xs.min()))
        result: list[tuple[float, tuple[int, int, int, int]]] = []
        for x1, y1, x2, y2 in lines[:, 0, :]:
            length = math.hypot(x2 - x1, y2 - y1)
            if length < 18:
                continue
            # 足跟/小腿的主边缘通常近似竖直；针线在该视角下应明显偏离竖直。
            if abs(x2 - x1) < abs(y2 - y1) * 0.38:
                continue
            samples = np.rint(np.linspace((x1, y1), (x2, y2), 120)).astype(int)
            inside = mask[samples[:, 1], samples[:, 0]]
            transitions = np.flatnonzero(inside[1:] != inside[:-1]) + 1
            if len(transitions) == 0:
                continue
            # 取最靠近足跟中心的进出边界，作为针孔候选。
            transition = min(transitions, key=lambda i: float(np.linalg.norm(samples[i] - center)))
            point = samples[transition]
            if float(np.linalg.norm(point - np.asarray(puncture))) > 32.0:
                continue
            if float(np.linalg.norm(point - center)) > width * 1.8:
                continue
            outside = int(np.count_nonzero(~inside))
            score = length + outside * 0.25 - float(np.linalg.norm(point - center)) * 0.15
            result.append((score, (int(x1), int(y1), int(x2), int(y2))))
        return sorted(result, key=lambda item: item[0], reverse=True)

    @staticmethod
    def _fallback_line(image: np.ndarray, mask: np.ndarray, puncture: tuple[int, int]) -> tuple[int, int, int, int]:
        """针线不可直接拟合时，沿针孔附近最强的细长边缘给出短线，不使用足部长边。"""
        x, y = puncture
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        edges = cv2.Canny(gray, 35, 110)
        h, w = gray.shape
        best: tuple[float, tuple[int, int, int, int]] | None = None
        for angle in np.linspace(0, math.pi, 36, endpoint=False):
            direction = np.array([math.cos(angle), math.sin(angle)], dtype=np.float32)
            length = 45
            p1 = np.asarray([x, y], dtype=np.float32) - direction * 8
            p2 = np.asarray([x, y], dtype=np.float32) + direction * length
            samples = np.rint(np.linspace(p1, p2, 40)).astype(int)
            valid = ((samples[:, 0] >= 0) & (samples[:, 0] < w) &
                     (samples[:, 1] >= 0) & (samples[:, 1] < h))
            score = float(np.count_nonzero(edges[samples[valid, 1], samples[valid, 0]]))
            candidate = (int(p1[0]), int(p1[1]), int(p2[0]), int(p2[1]))
            if best is None or score > best[0]:
                best = (score, candidate)
        return best[1] if best is not None else (x, y, x + 1, y + 1)

    @staticmethod
    def _trace_needle_from_puncture(
        image: np.ndarray, mask: np.ndarray, puncture: tuple[int, int]
    ) -> tuple[tuple[int, int, int, int], float]:
        """从候选针孔向所有方向追踪连续细边缘，返回最可信的线。"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        enhanced = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
        enhanced = cv2.GaussianBlur(enhanced, (3, 3), 0)
        edges = cv2.Canny(enhanced, 22, 70)
        x, y = puncture
        height, width = gray.shape
        best: tuple[float, float] | None = None
        for degree in range(0, 360, 2):
            radian = math.radians(degree)
            direction = np.asarray([math.cos(radian), math.sin(radian)], dtype=np.float32)
            distance = np.arange(6, 156)
            samples = np.rint(np.asarray([x, y]) + np.outer(distance, direction)).astype(int)
            valid = ((samples[:, 0] >= 0) & (samples[:, 0] < width) &
                     (samples[:, 1] >= 0) & (samples[:, 1] < height))
            count = int(np.count_nonzero(valid))
            if count < 20:
                continue
            hits = edges[samples[valid, 1], samples[valid, 0]] > 0
            outside = ~mask[samples[valid, 1], samples[valid, 0]]
            # 针线应从足跟内部的小孔延伸到足跟外；足部纹理或轮廓通常不能满足这一点。
            if int(np.count_nonzero(outside)) < 12:
                continue
            weights = np.linspace(0.6, 1.5, count)
            score = float(np.sum(hits * weights) + 0.15 * np.count_nonzero(outside))
            if best is None or score > best[0]:
                best = (score, float(degree))
        if best is None:
            return (x, y, x, y), 0.0
        degree = best[1]
        direction = np.asarray([math.cos(math.radians(degree)), math.sin(math.radians(degree))])
        end = np.rint(np.asarray([x, y]) + direction * 145.0).astype(int)
        end[0] = int(np.clip(end[0], 0, width - 1))
        end[1] = int(np.clip(end[1], 0, height - 1))
        return (x, y, int(end[0]), int(end[1])), float(best[0])

    def analyze(self, image: np.ndarray) -> tuple[dict[str, Any], np.ndarray]:
        mask = self._heel_mask(image)
        candidates = self._dark_puncture_candidates(image, mask)
        if not candidates:
            raise ValueError("足跟区域没有找到足够清晰的深色针孔候选")
        choices = []
        for darkness, candidate in candidates:
            line, line_score = self._trace_needle_from_puncture(image, mask, candidate)
            choices.append((darkness + line_score * 3.0, candidate, line, line_score))
        _, puncture, line, line_score = max(choices, key=lambda item: item[0])
        x1, y1, x2, y2 = line
        ys, xs = np.nonzero(mask)
        center = np.array([xs.mean(), ys.mean()], dtype=np.float32)
        result = {
            "valid": True,
            "heel_center_px": [int(round(float(center[0]))), int(round(float(center[1])))],
            "puncture_px": list(puncture),
            "needle_line_px": list(line),
            "needle_line_score": round(float(line_score), 2),
            "method": "global_dark_candidates_plus_connected_line",
            "note": "仅二维检测；不输出深度，不控制机器人",
        }
        debug = image.copy()
        # 只画足跟轮廓，保留原始颜色和暗点，便于人眼复核。
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (80, 180, 80), 1)
        cv2.line(debug, (x1, y1), (x2, y2), (0, 220, 255), 3)
        cv2.circle(debug, puncture, 6, (0, 0, 255), -1)
        return result, debug

    @staticmethod
    def _dark_puncture_candidates(image: np.ndarray, mask: np.ndarray) -> list[tuple[float, tuple[int, int]]]:
        """全局寻找足跟内部的局部深色小点，不使用固定位置或固定方向。"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        ys, xs = np.nonzero(mask)
        if len(xs) < 100:
            return None
        x_min, x_max = int(xs.min()), int(xs.max())
        y_min, y_max = int(ys.min()), int(ys.max())
        height = max(1, y_max - y_min)
        yy, xx = np.indices(gray.shape)
        # 只排除明显的小腿下段，不指定上部的某一个固定点。
        upper_heel = yy < y_min + int(height * 0.58)
        inside = cv2.distanceTransform(mask.astype(np.uint8), cv2.DIST_L2, 3) > 5.0
        search = mask & inside & upper_heel
        smooth = cv2.GaussianBlur(gray, (31, 31), 0)
        darkness = np.maximum(smooth.astype(np.float32) - gray.astype(np.float32), 0.0)
        darkness[~search] = 0.0
        peaks = darkness == cv2.dilate(darkness, np.ones((9, 9), np.uint8))
        ys2, xs2 = np.nonzero(peaks & search & (darkness > 4.0))
        ranked = sorted(
            ((float(darkness[y, x]), (int(x), int(y))) for y, x in zip(ys2, xs2)),
            reverse=True,
        )
        return ranked[:16]
