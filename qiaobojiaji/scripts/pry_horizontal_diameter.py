#!/usr/bin/env python3
"""Independent, read-only horizontal-diameter check for the pry-buckle task.

It subscribes to the already running D435 ROS topics.  It does not start a
librealsense pipeline and it never sends a robot or gripper command.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
from threading import Event

import cv2
import numpy as np

from pry_buckle.horizontal_diameter import CameraIntrinsics, HorizontalDiameterEstimator


def segment_heel(image: np.ndarray, model_path: Path) -> np.ndarray:
    from ultralytics import YOLO

    prediction = YOLO(str(model_path)).predict(
        source=image, imgsz=640, conf=0.5, retina_masks=True, verbose=False, device="cpu"
    )[0]
    if prediction.masks is None or prediction.boxes is None:
        raise RuntimeError("YOLO did not return a heel mask")
    confidences = prediction.boxes.conf.detach().cpu().numpy()
    index = int(np.argmax(confidences))
    mask = prediction.masks.data[index].detach().cpu().numpy()
    return cv2.resize(mask, (image.shape[1], image.shape[0]), interpolation=cv2.INTER_NEAREST) > 0.5


def main() -> None:
    parser = argparse.ArgumentParser()
    parser.add_argument("--color", type=Path, help="saved BGR/RGB image")
    parser.add_argument("--depth-npy", type=Path, help="aligned depth array in mm or m")
    parser.add_argument("--camera-info", type=Path, help="JSON saved by capture_pry_d435_frame.py")
    parser.add_argument("--model", type=Path, default=Path("platform_a/models/heel_seg.pt"))
    parser.add_argument("--output-dir", type=Path, default=Path("debug/pry_buckle"))
    args = parser.parse_args()
    required = (args.color, args.depth_npy, args.camera_info)
    if not all(value is not None for value in required):
        parser.error("this offline checker requires --color --depth-npy --camera-info")
    image = cv2.imread(str(args.color), cv2.IMREAD_COLOR)
    if image is None:
        raise RuntimeError(f"cannot load color image: {args.color}")
    depth = np.load(args.depth_npy)
    camera_info = json.loads(args.camera_info.read_text(encoding="utf-8"))
    intrinsics = CameraIntrinsics(
        float(camera_info["fx"]), float(camera_info["fy"]),
        float(camera_info["cx"]), float(camera_info["cy"]),
    )
    mask = segment_heel(image, args.model)
    estimator = HorizontalDiameterEstimator()
    result = estimator.estimate(mask, depth, intrinsics)
    args.output_dir.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(args.output_dir / "horizontal_diameter_overlay.png"), estimator.draw_overlay(image, mask, result))
    (args.output_dir / "horizontal_diameter_result.json").write_text(
        json.dumps(result, ensure_ascii=False, indent=2), encoding="utf-8"
    )
    print(json.dumps(result, ensure_ascii=False, indent=2))


if __name__ == "__main__":
    main()
