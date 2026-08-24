"""AG-160-95 实测开度换算。

控制器给出的 0--1000 是内部行程值，并不等同于两根手指之间的毫米数。
本模块只使用现场量得的数值做分段直线换算，避免把内部行程直接当作开度。
"""

from __future__ import annotations

import json
from pathlib import Path


CALIBRATION_FILE = Path(__file__).resolve().parent / "config" / "ag95_opening_calibration.json"


def _points() -> list[tuple[float, float]]:
    data = json.loads(CALIBRATION_FILE.read_text(encoding="utf-8"))
    points = [(float(item["raw"]), float(item["gap_mm"])) for item in data["points"]]
    if len(points) < 2 or points != sorted(points):
        raise RuntimeError("AG95 开度标定表无效")
    return points


def _interpolate(value: float, pairs: list[tuple[float, float]]) -> float:
    """在实测点之间插值；端点外仅作线性延伸。"""
    for (x0, y0), (x1, y1) in zip(pairs, pairs[1:]):
        if x0 <= value <= x1:
            return y0 + (value - x0) * (y1 - y0) / (x1 - x0)
    (x0, y0), (x1, y1) = pairs[:2] if value < pairs[0][0] else pairs[-2:]
    return y0 + (value - x0) * (y1 - y0) / (x1 - x0)


def gap_mm_from_raw(raw: int | float) -> float:
    return max(0.0, _interpolate(float(raw), _points()))


def raw_from_gap_mm(gap_mm: int | float) -> int:
    points = _points()
    raw = _interpolate(float(gap_mm), [(gap, raw) for raw, gap in points])
    return max(0, min(1000, int(round(raw))))
