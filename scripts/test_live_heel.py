"""Standalone live test of the pry-buckle heel recognition algorithm.

Subscribes to the already-running D435 ROS2 topics, runs YOLO (heel_seg.pt)
to get the heel mask, feeds it to HorizontalDiameterEstimator, draws the
overlay, and writes one annotated PNG plus the result JSON.

Diagnostics: always saves the raw colour frame, reports YOLO top boxes even
when nothing passes the threshold, and lowers conf to 0.01 so we can tell
whether the model fires at all.

Run inside WSL (Ubuntu-22.04-F), with ROS + the project venv available:
  /home/zhj/projects/fr5_platform_ws/.venv/bin/python test_live_heel.py
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

# Preload ROS shared libs so rclpy imports cleanly from the venv process.
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

DEBUG = PROJ / "debug"
DEBUG.mkdir(parents=True, exist_ok=True)


class LiveHeelTester(Node):
    def __init__(self):
        super().__init__("live_heel_tester")
        self.bridge = CvBridge()
        self.detector = YOLO(str(PROJ / "platform_a/models/heel_seg.pt"))
        self.estimator = HorizontalDiameterEstimator()
        self.state = {"image": None, "depth": None, "intrinsics": None}
        self.processed = False
        self.result = None
        self.out_path = None
        self.top_boxes = []
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

    def _save_raw(self, img):
        cv2.imwrite(str(DEBUG / "live_raw.png"), img)

    def try_process(self):
        if self.processed:
            return
        s = self.state
        if s["image"] is None or s["depth"] is None or s["intrinsics"] is None:
            return
        img_raw = s["image"]
        self._save_raw(img_raw)
        # Rotate the D435 stream back 180 deg so the model (trained on
        # upright images) can detect the heel reliably.  Rotate depth too
        # and adjust the principal point to match.
        img = cv2.rotate(img_raw, cv2.ROTATE_180)
        depth = cv2.rotate(s["depth"], cv2.ROTATE_180)
        ih, iw = img.shape[:2]
        intr = CameraIntrinsics(s["intrinsics"].fx, s["intrinsics"].fy, iw - s["intrinsics"].cx, ih - s["intrinsics"].cy)
        # Low conf so we can see whether the model fires at all.
        pred = self.detector.predict(source=img, imgsz=640, conf=0.01, retina_masks=True, verbose=False, device="cpu")[0]
        boxes = pred.boxes
        masks = pred.masks
        top = []
        if boxes is not None and len(boxes):
            confs = boxes.conf.detach().cpu().numpy()
            clss = boxes.cls.detach().cpu().numpy().astype(int)
            names = pred.names
            order = np.argsort(-confs)[:5]
            for i in order:
                top.append((str(names.get(int(clss[i]), clss[i])), round(float(confs[i]), 3)))
        self.top_boxes = top
        if masks is None or len(masks) == 0:
            self.get_logger().warn("no heel mask; top boxes: %s" % top)
            preview = img.copy()
            cv2.putText(preview, "NO HEEL MASK (top: %s)" % top, (16, 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2, cv2.LINE_AA)
            out = DEBUG / "live_heel_overlay.png"
            cv2.imwrite(str(out), preview)
            self.out_path = out
            self.processed = True
            return
        idx = int(np.argmax(boxes.conf.detach().cpu().numpy()))
        raw = masks.data[idx].detach().cpu().numpy()
        mask = cv2.resize(raw, (img.shape[1], img.shape[0]), interpolation=cv2.INTER_NEAREST) > 0.5
        res = self.estimator.estimate(mask, depth, intr)
        res.update({
            "heel_detected": True,
            "yolo_top_boxes": top,
            "heel_center_px": res.get("center_px"),
            "clamp_contact_a_px": res.get("contact_left_px"),
            "clamp_contact_b_px": res.get("contact_right_px"),
            "heel_width_mm": res.get("width_mm"),
        })
        overlay = self.estimator.draw_overlay(img, mask, res)
        out = DEBUG / "live_heel_overlay.png"
        cv2.imwrite(str(out), overlay)
        self.result = res
        self.out_path = out
        self.processed = True
        self.get_logger().info("overlay written: %s" % out)


def main():
    rclpy.init(args=None)
    node = LiveHeelTester()
    deadline = time.time() + 25.0
    while time.time() < deadline and not node.processed:
        rclpy.spin_once(node, timeout_sec=0.2)
        node.try_process()
    print("=== YOLO TOP BOXES (lowest conf) ===")
    print(json.dumps(node.top_boxes, ensure_ascii=False))
    if node.result is not None:
        printable = {k: v for k, v in node.result.items() if k != "image_axis"}
        print("=== HEEL RESULT ===")
        print(json.dumps(printable, ensure_ascii=False, indent=2, default=str))
    else:
        print("NO_HEEL_RESULT within timeout")
        print("frames seen -> color:%s depth:%s intrinsics:%s" % (
            node.state["image"] is not None,
            node.state["depth"] is not None,
            node.state["intrinsics"] is not None,
        ))
    print("OVERLAY_PATH=" + str(node.out_path))
    sys.stdout.flush()
    node.destroy_node()
    rclpy.shutdown()


if __name__ == "__main__":
    main()
