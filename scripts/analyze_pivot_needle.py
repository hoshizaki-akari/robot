#!/usr/bin/env python3
"""用一张图片单独测试撬拨视角下的针孔和针线检测。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from platform_a.pivot_needle_vision import PivotNeedleVision


def main() -> int:
    parser = argparse.ArgumentParser(description="撬拨观察位二维针线检测")
    parser.add_argument("image", type=Path, help="原始摄像头图片")
    parser.add_argument(
        "--output", type=Path, default=Path("pivot_needle_debug.png"),
        help="输出红点/黄线调试图片",
    )
    parser.add_argument(
        "--model", type=Path,
        default=Path(__file__).resolve().parents[1] / "platform_a/models/heel_seg.pt",
        help="足跟分割模型路径",
    )
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit(f"无法读取图片：{args.image}")
    result, debug = PivotNeedleVision(args.model).analyze(image)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), debug)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"调试图已保存：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
