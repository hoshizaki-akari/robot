"""撬拨第 2 视角的二维针孔/针线检测。

此视角中足跟分割区域在画面上方，针线从足跟下缘附近向下延伸，
因此不能套用第 1 视角的规则。
"""

from __future__ import annotations

from pathlib import Path
from typing import Any

import cv2
import numpy as np


class PivotViewBVision:
    def __init__(self, model_path: str | Path) -> None:
        from ultralytics import YOLO

        self.model = YOLO(str(model_path))

    def _heel_mask(self, image: np.ndarray) -> np.ndarray:
        result = self.model.predict(
            image, imgsz=640, conf=0.25, retina_masks=True,
            verbose=False, device="cpu"
        )[0]
        if result.masks is None or len(result.boxes) == 0:
            raise ValueError("没有找到足跟区域")
        index = int(np.argmax(result.boxes.conf.detach().cpu().numpy()))
        raw = result.masks.data[index].detach().cpu().numpy()
        h, w = image.shape[:2]
        return cv2.resize(raw, (w, h), interpolation=cv2.INTER_NEAREST) > 0.5

    @staticmethod
    def _needle_line_from_edges(image: np.ndarray, mask: np.ndarray) -> tuple[int, int, int, int] | None:
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        gray = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(gray)
        edges = cv2.Canny(gray, 25, 90)
        ys, xs = np.nonzero(mask)
        if len(xs) < 30:
            return None
        x_center = float(xs.mean())
        y_start = int(ys.max())
        h, w = gray.shape
        roi = np.zeros_like(edges)
        roi[max(0, y_start - 4):min(h, y_start + 170),
            max(0, int(x_center - 75)):min(w, int(x_center + 75))] = edges[
                max(0, y_start - 4):min(h, y_start + 170),
                max(0, int(x_center - 75)):min(w, int(x_center + 75))]
        lines = cv2.HoughLinesP(roi, 1, np.pi / 360, 10, minLineLength=45, maxLineGap=22)
        if lines is None:
            return None
        candidates = []
        for x1, y1, x2, y2 in lines[:, 0, :]:
            if y1 > y2:
                x1, y1, x2, y2 = x2, y2, x1, y1
            length = float(np.hypot(x2 - x1, y2 - y1))
            if length < 45 or y1 < y_start - 12 or y1 > y_start + 45:
                continue
            if abs(x2 - x1) > abs(y2 - y1) * 0.45:
                continue
            center_penalty = abs((x1 + x2) * 0.5 - x_center) * 0.3
            candidates.append((length - center_penalty, (int(x1), int(y1), int(x2), int(y2))))
        return max(candidates, default=(0.0, None))[1]

    @staticmethod
    def _view_b_puncture(image: np.ndarray, mask: np.ndarray) -> tuple[int, int] | None:
        """第二视角：在完整肉色色块内部，按局部颜色寻找针孔。"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        skin = (((hsv[:, :, 0] <= 25) | (hsv[:, :, 0] >= 165)) &
                (hsv[:, :, 1] >= 18) & (hsv[:, :, 1] <= 210) &
                (hsv[:, :, 2] >= 45)).astype(np.uint8)
        # 从足跟模型向外扩展，只保留同一块脚/小腿肉色区域，排除桌面和杂物。
        support = cv2.dilate(mask.astype(np.uint8), np.ones((9, 9), np.uint8), iterations=18)
        region = (skin & support).astype(np.uint8)
        region = cv2.morphologyEx(region, cv2.MORPH_CLOSE, np.ones((9, 9), np.uint8))
        count, labels, stats, _ = cv2.connectedComponentsWithStats(region, 8)
        if count > 1:
            overlap = [
                (int(np.count_nonzero((labels == label) & (mask > 0))),
                 int(stats[label, cv2.CC_STAT_AREA]), label)
                for label in range(1, count)
            ]
            label = max(overlap)[2]
            region = (labels == label).astype(np.uint8)
        ys, xs = np.nonzero(region)
        if len(xs) < 30:
            return None
        center_x = float(xs.mean())
        top, bottom = int(ys.min()), int(ys.max())
        height = max(1, bottom - top)
        yy, xx = np.indices(gray.shape)
        valid = (region > 0) & (cv2.distanceTransform(region, cv2.DIST_L2, 3) > 4.0)
        mask_top = int(np.nonzero(mask)[0].min()) if np.count_nonzero(mask) else top
        valid &= yy >= mask_top
        # 保留足跟和针孔附近的完整肉色区域，不把最下端手臂杂色纳入候选。
        valid &= yy <= top + int(height * 0.72)
        blur = cv2.GaussianBlur(gray, (21, 21), 0)
        response = np.maximum(blur.astype(np.float32) - gray.astype(np.float32), 0.0)
        response[~valid] = 0.0
        peaks = response == cv2.dilate(response, np.ones((7, 7), np.uint8))
        ys2, xs2 = np.nonzero(peaks & valid & (response > 3.0))
        if len(xs2) == 0:
            return None
        order = np.argsort(response[ys2, xs2])[::-1]
        return int(xs2[order[0]]), int(ys2[order[0]])

    @staticmethod
    def _trace_view_b_line(image: np.ndarray, mask: np.ndarray, point: tuple[int, int]) -> tuple[int, int, int, int]:
        """从针孔出发，在整张画面里找与其相连的细暗线。"""
        gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
        enhanced = cv2.createCLAHE(2.5, (8, 8)).apply(gray)
        edges = cv2.Canny(enhanced, 22, 70)
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
        # 图上人工画的红/黄标记不是真实针线，验收截图中也不应让它们参与判断。
        markup = (hsv[:, :, 1] > 115) & (
            ((hsv[:, :, 0] >= 15) & (hsv[:, :, 0] <= 42)) |
            (hsv[:, :, 0] <= 8) | (hsv[:, :, 0] >= 170)
        )
        # 边缘检测会在黄线边缘产生两条强边，因此屏蔽区必须比黄线本身稍宽。
        markup = cv2.dilate(markup.astype(np.uint8), np.ones((13, 13), np.uint8)).astype(bool)
        outside_heel = cv2.distanceTransform((~mask).astype(np.uint8), cv2.DIST_L2, 3) > 1.5
        # 针线在此图里是一根比周边略暗的细线。使用局部亮度差，而非固定颜色阈值。
        local_background = cv2.GaussianBlur(enhanced, (31, 31), 0)
        dark_ridge = np.maximum(local_background.astype(np.float32) - enhanced.astype(np.float32), 0.0)
        ridge_cut = float(np.percentile(dark_ridge[~markup], 88.0))
        ridge = (dark_ridge >= max(5.0, ridge_cut)) & ~markup
        x, y = point
        heel_y, heel_x = np.nonzero(mask)
        heel_center = np.asarray([float(heel_x.mean()), float(heel_y.mean())])
        outward = np.asarray([float(x), float(y)]) - heel_center
        outward_norm = float(np.linalg.norm(outward))
        if outward_norm > 1e-6:
            outward /= outward_norm
        best: tuple[float, np.ndarray] | None = None
        # 从针孔向所有 360 个方向逐像素追踪；没有预先写死“向左/向右/向下”。
        # 真实针线的特点是：从针孔离开肉色足部后，沿一条细暗脊持续存在。
        for degree in range(360):
            direction = np.asarray([np.cos(np.radians(degree)), np.sin(np.radians(degree))])
            outward_alignment = 1.0 if outward_norm <= 1e-6 else float(direction @ outward)
            if outward_alignment < 0.70:
                continue
            distance = np.arange(6, 165)
            samples = np.rint(np.asarray([x, y]) + np.outer(distance, direction)).astype(int)
            valid = ((samples[:, 0] >= 1) & (samples[:, 0] < gray.shape[1] - 1) &
                     (samples[:, 1] >= 1) & (samples[:, 1] < gray.shape[0] - 1))
            samples = samples[valid]
            if len(samples) < 28:
                continue
            px, py = samples[:, 0], samples[:, 1]
            usable = outside_heel[py, px] & ~markup[py, px]
            # 真正的针线是“中间细暗、两侧相近”的脊线；足部边缘则一边亮一边暗。
            # 这一项能把小腿轮廓、桌沿等粗边从候选中剔除。
            normal = np.asarray([-direction[1], direction[0]])
            left = np.rint(samples + normal * 4.0).astype(int)
            right = np.rint(samples - normal * 4.0).astype(int)
            side_valid = ((left[:, 0] >= 0) & (left[:, 0] < gray.shape[1]) &
                          (left[:, 1] >= 0) & (left[:, 1] < gray.shape[0]) &
                          (right[:, 0] >= 0) & (right[:, 0] < gray.shape[1]) &
                          (right[:, 1] >= 0) & (right[:, 1] < gray.shape[0]))
            if not np.any(side_valid):
                continue
            px, py, left, right, usable = (
                px[side_valid], py[side_valid], left[side_valid], right[side_valid], usable[side_valid]
            )
            left_value = enhanced[left[:, 1], left[:, 0]].astype(np.float32)
            right_value = enhanced[right[:, 1], right[:, 0]].astype(np.float32)
            centre_value = enhanced[py, px].astype(np.float32)
            thin_dark = np.maximum(
                (left_value + right_value) * 0.5 - centre_value -
                0.30 * np.abs(left_value - right_value), 0.0
            )
            thin_dark[~usable] = 0.0
            hit = thin_dark > max(2.0, float(np.percentile(thin_dark[usable], 65.0)) if np.any(usable) else 2.0)
            continuous = float(np.max(np.convolve(hit.astype(np.float32), np.ones(11), mode="same")))
            score = float(np.sum(thin_dark)) + 5.0 * continuous + 2.0 * outward_alignment
            if best is None or score > best[0]:
                best = (score, direction)
        if best is None:
            raise ValueError("没有找到与针孔连续相连的针线")
        direction = best[1]
        end = np.rint(np.asarray([x, y]) + direction * 145).astype(int)
        end[0] = int(np.clip(end[0], 0, gray.shape[1] - 1))
        end[1] = int(np.clip(end[1], 0, gray.shape[0] - 1))
        return x, y, int(end[0]), int(end[1])

    def analyze(self, image: np.ndarray) -> tuple[dict[str, Any], np.ndarray]:
        mask = self._heel_mask(image)
        puncture = self._view_b_puncture(image, mask)
        source = "d435_enhanced_edges"
        line = None if puncture is None else self._trace_view_b_line(image, mask, puncture)
        if line is None:
            raise ValueError("第二视角没有找到从足跟下缘向下延伸的针线")
        x1, y1, x2, y2 = line
        puncture = puncture or (x1, y1)
        ys, xs = np.nonzero(mask)
        result = {
            "valid": True,
            "heel_center_px": [int(round(float(xs.mean()))), int(round(float(ys.mean())))],
            "puncture_px": list(puncture),
            "needle_line_px": list(line),
            "needle_vector_2d": [x2 - x1, y2 - y1],
            "source": source,
            "note": "第二视角二维结果；尚未转换为三维",
        }
        debug = image.copy()
        contours, _ = cv2.findContours(mask.astype(np.uint8), cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cv2.drawContours(debug, contours, -1, (80, 180, 80), 1)
        cv2.line(debug, (x1, y1), (x2, y2), (0, 220, 255), 3)
        cv2.circle(debug, puncture, 6, (0, 0, 255), -1)
        return result, debug
