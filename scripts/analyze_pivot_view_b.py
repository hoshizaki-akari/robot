#!/usr/bin/env python3
"""单独测试撬拨第 2 视角的二维针孔和针线。"""

from __future__ import annotations

import argparse
import json
import sys
from pathlib import Path

import cv2

sys.path.insert(0, str(Path(__file__).resolve().parents[1]))
from platform_a.pivot_view_b_vision import PivotViewBVision


def main() -> int:
    parser = argparse.ArgumentParser()
    parser.add_argument("image", type=Path)
    parser.add_argument("--output", type=Path, default=Path("pivot_view_b_debug.png"))
    parser.add_argument(
        "--model", type=Path,
        default=Path(__file__).resolve().parents[1] / "platform_a/models/heel_seg.pt",
    )
    args = parser.parse_args()
    image = cv2.imread(str(args.image))
    if image is None:
        raise SystemExit(f"无法读取图片：{args.image}")
    result, debug = PivotViewBVision(args.model).analyze(image)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output), debug)
    print(json.dumps(result, ensure_ascii=False, indent=2))
    print(f"调试图已保存：{args.output}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
