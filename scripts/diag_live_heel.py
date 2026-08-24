"""Diagnose why live heel detection fails while the saved sample frame works.

Captures one live frame, then tries multiple orientations / preprocessing
variants and reports what YOLO sees on each.  For the best variant it also
runs the full HorizontalDiameterEstimator pipeline and saves the overlay.
"""
import sys
import os
import time
import json
from pathlib import Path

PROJ = Path("/home/zhj/projects/fr5_platform_ws")
sys.path.insert(0, str(PROJ))
VENV_SITE = PROJ / ".venv/lib/python3.10/site-packages"
ROS_SITES = [
    Path("/opt/ros/humble/local/lib/python3.10/dist-packages"),
    Path("/opt/ros/humble/lib/python3.10/site-packages"),
]
for p in (VENV_SITE, *ROS_SITES):
    if p.exists() and str(p) not in sys.path:
        sys.path.insert(0, str(p))

ros_lib = "/opt/ros/humble/lib"
old_ld = os.environ.get("LD_LIBRARY_PATH", "")
if ros_lib not in old_ld.split(":"):
    os.environ["LD_LIBRARY_PATH"] = ros_lib + ((":" + old_ld) if old_ld else "")
try:
    import ctypes
    priority = [f"{ros_lib}/librcutils.so", f"{ros_lib}/librmw.so",
                f"{ros_lib}/librcl.so", f"{ros_lib}/librcl_action.so"]
    shared = sorted(Path(ros_lib).glob("*.so*"))
    for _ in range(3):
        for lib in [Path(x) for x in priority] + shared:
            try:
                ctypes.CDLL(str(lib), mode=ctypes.RTLD_GLOBAL)
            except OSError:
                pass
except Exception:
    pass

import numpy as np
import cv2
import rclpy
from cv_bridge import CvBridge
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import CameraInfo, Image
from ultralytics import YOLO
from pry_buckle.horizontal_diameter import CameraIntrinsics, HorizontalDiameterEstimator

DEBUG = PROJ / "debug" / "diag"
DEBUG.mkdir(parents=True, exist_ok=True)


class Diag(Node):
    def __init__(self):
        super().__init__("diag_live_heel")
        self.bridge = CvBridge()
        self.detector = YOLO(str(PROJ / "platform_a/models/heel_seg.pt"))
        self.estimator = HorizontalDiameterEstimator()
        self.state = {"image": None, "depth": None, "intrinsics": None}
        self.done = False
        q = qos_profile_sensor_data
        self.create_subscription(Image, "/camera/camera/color/image_raw", self.cb_color, q)
        self.create_subscription(Image, "/camera/camera/aligned_depth_to_color/image_raw", self.cb_depth, q)
        self.create_subscription(CameraInfo, "/camera/camera/color/camera_info", self.cb_info, q)

    def cb_color(self, m):
        self.state["image"] = self.bridge.imgmsg_to_cv2(m, desired_encoding="bgr8")

    def cb_depth(self, m):
        d = self.bridge.imgmsg_to_cv2(m, desired_encoding="passthrough")
        self.state["depth"] = d.astype(np.float32) * (1000.0 if m.encoding.lower() == "32fc1" else 1.0)

    def cb_info(self, m):
        self.state["intrinsics"] = CameraIntrinsics(float(m.k[0]), float(m.k[4]), float(m.k[2]), float(m.k[5]))

    def _detect(self, img, tag):
        pred = self.detector.predict(source=img, imgsz=640, conf=0.01, retina_masks=True, verbose=False, device="cpu")[0]
        boxes = pred.boxes
        masks = pred.masks
        report = {"tag": tag, "boxes": [], "best_mask": None, "best_conf": 0.0}
        if boxes is None or len(boxes) == 0:
            return report
        confs = boxes.conf.detach().cpu().numpy()
        clss = boxes.cls.detach().cpu().numpy().astype(int)
        xyxy = boxes.xyxy.detach().cpu().numpy()
        names = pred.names
        for i in range(len(confs)):
            report["boxes"].append({
                "cls": str(names.get(int(clss[i]), clss[i])),
                "conf": round(float(confs[i]), 3),
                "xyxy": [round(float(v), 1) for v in xyxy[i]],
            })
        best = int(np.argmax(confs))
        report["best_conf"] = round(float(confs[best]), 3)
        if masks is not None and len(masks) > 0:
            raw = masks.data[best].detach().cpu().numpy()
            mask = cv2.resize(raw, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0.5
            report["best_mask"] = mask
            report["mask_area_px"] = int(np.count_nonzero(mask))
        return report

    def run_diag(self):
        s = self.state
        img = s["image"]
        depth = s["depth"]
        intr = s["intrinsics"]
        cv2.imwrite(str(DEBUG / "00_raw.png"), img)
        variants = {
            "original": img,
            "rot180": cv2.rotate(img, cv2.ROTATE_180),
        }
        # sharpen
        blur = cv2.GaussianBlur(img, (0, 0), 3)
        variants["sharpen"] = cv2.addWeighted(img, 1.6, blur, -0.6, 0)
        # CLAHE on LAB L channel
        lab = cv2.cvtColor(img, cv2.COLOR_BGR2LAB)
        lab[:, :, 0] = cv2.createCLAHE(clipLimit=2.5, tileGridSize=(8, 8)).apply(lab[:, :, 0])
        variants["clahe"] = cv2.cvtColor(lab, cv2.COLOR_LAB2BGR)

        reports = []
        best = None
        for tag, v in variants.items():
            r = self._detect(v, tag)
            reports.append({k: val for k, val in r.items() if k != "best_mask"})
            if r["best_mask"] is not None and (best is None or r["best_conf"] > best[0]["best_conf"]):
                best = (r, v, tag)

        print("=== DETECTION REPORTS (conf=0.01) ===")
        for r in reports:
            print(json.dumps(r, ensure_ascii=False))

        if best is None:
            print("NO_MASK_ON_ANY_VARIANT")
            self.done = True
            return

        r, best_img, best_tag = best
        print(f"=== BEST VARIANT: {best_tag} conf={r['best_conf']} mask_area={r.get('mask_area_px')} ===")
        mask = r["best_mask"]
        # NOTE: depth is aligned to the ORIGINAL image.  If the best variant is
        # rot180, we must rotate depth back to match the mask coordinates.
        depth_for_est = depth
        if best_tag == "rot180":
            depth_for_est = cv2.rotate(depth, cv2.ROTATE_180)
        result = self.estimator.estimate(mask, depth_for_est, intr)
        result["diag_best_variant"] = best_tag
        result["diag_best_conf"] = r["best_conf"]
        result["heel_detected"] = True
        result["heel_width_mm"] = result.get("width_mm")
        overlay = self.estimator.draw_overlay(best_img, mask, result)
        cv2.imwrite(str(DEBUG / "01_best_overlay.png"), overlay)
        cv2.imwrite(str(DEBUG / "01_best_variant.png"), best_img)
        print("=== ESTIMATE RESULT ===")
        printable = {k: v for k, v in result.items() if k != "image_axis"}
        print(json.dumps(printable, ensure_ascii=False, indent=2, default=str))
        self.done = True


def main():
    rclpy.init(args=None)
    node = Diag()
    deadline = time.time() + 15.0
    while time.time() < deadline and not node.done:
        if node.state["image"] is not None and node.state["depth"] is not None and node.state["intrinsics"] is not None:
            node.run_diag()
            break
        rclpy.spin_once(node, timeout_sec=0.2)
    sys.stdout.flush()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
