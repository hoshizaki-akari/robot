#!/usr/bin/env python3
"""Interactive M3 annotation: ROI plus two points on the K-wire centreline."""
from __future__ import annotations

import argparse
import json
from pathlib import Path

import cv2

PROJECT = Path(__file__).resolve().parents[1]


def read_annotations(path: Path) -> dict:
    return json.loads(path.read_text(encoding="utf-8")) if path.exists() else {}


def main() -> int:
    parser = argparse.ArgumentParser(description="框选克氏针 ROI，并在线上点击两个点")
    parser.add_argument("name", choices=("wire_a", "wire_b"))
    parser.add_argument("--session", type=Path, required=True)
    parser.add_argument("--roi", help="非交互 ROI：x,y,w,h")
    parser.add_argument("--line", help="非交互中心线种子：x1,y1,x2,y2")
    args = parser.parse_args()
    image_path = args.session / args.name / "rgb.png"
    image = cv2.imread(str(image_path), cv2.IMREAD_COLOR)
    if image is None:
        raise FileNotFoundError(image_path)
    parse = lambda value: [int(part) for part in value.split(",")]
    roi = tuple(parse(args.roi)) if args.roi else cv2.selectROI(f"{args.name}: drag wire ROI then Enter", image, fromCenter=False, showCrosshair=True)
    cv2.destroyAllWindows()
    if roi[2] < 8 or roi[3] < 8:
        raise RuntimeError("ROI 太小或已取消")
    points: list[tuple[int, int]] = [] if not args.line else [tuple(parse(args.line)[:2]), tuple(parse(args.line)[2:])]
    display = image.copy()
    def click(event, x, y, *_):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 2:
            points.append((x, y))
    if not args.line:
        cv2.namedWindow("click two points on wire centreline; Enter confirms")
        cv2.setMouseCallback("click two points on wire centreline; Enter confirms", click)
        while True:
            canvas = display.copy()
            x, y, w, h = map(int, roi)
            cv2.rectangle(canvas, (x, y), (x + w, y + h), (0, 255, 255), 2)
            for point in points:
                cv2.circle(canvas, point, 4, (0, 0, 255), -1)
            if len(points) == 2:
                cv2.line(canvas, points[0], points[1], (0, 0, 255), 1)
            cv2.imshow("click two points on wire centreline; Enter confirms", canvas)
            key = cv2.waitKey(30) & 0xFF
            if key in (13, 10) and len(points) == 2:
                break
            if key == 27:
                raise RuntimeError("标注取消")
        cv2.destroyAllWindows()
    file = args.session / "m3_wire_annotations.json"
    data = read_annotations(file)
    data[args.name] = {"roi_xywh": list(map(int, roi)), "seed_line_px": [*points[0], *points[1]], "entry_pixel": list(map(int, points[1])), "source": "manual_opencv"}
    file.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    print(f"saved: {file}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
