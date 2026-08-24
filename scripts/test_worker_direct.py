"""直接验证 PryBuckleVisionWorker 在当前相机下能否检出足跟。

绕过 d435_monitor，单独实例化 worker、start()，轮询 .result 与 .frame_png，
打印原始 message / heel_width_mm，并把标注图存盘。用于隔离 worker 本体问题。
"""
import sys, os, time, json
from pathlib import Path

PROJ = Path("/home/zhj/projects/fr5_platform_ws")
for p in (str(PROJ), str(PROJ / ".venv/lib/python3.10/site-packages")):
    if p not in sys.path:
        sys.path.insert(0, p)

# ROS 依赖
for lib in ("/opt/ros/humble/local/lib/python3.10/dist-packages",
            "/opt/ros/humble/lib/python3.10/site-packages"):
    if lib not in sys.path:
        sys.path.insert(0, lib)


def main():
    from platform_a.pry_buckle_vision import PryBuckleVisionWorker
    w = PryBuckleVisionWorker()
    w.start()
    print("worker started", flush=True)
    out = PROJ / "debug" / "worker_direct"
    out.mkdir(parents=True, exist_ok=True)
    last = None
    for i in range(24):
        time.sleep(0.5)
        r = w.result
        png = w.frame_png
        if png:
            with open(str(out / "annotated.png"), "wb") as f:
                f.write(png)
        sig = (r.get("valid"), r.get("heel_width_mm"), r.get("message"))
        if sig != last:
            print(f"[{i*0.5:4.1f}s] valid={r.get('valid')} width={r.get('heel_width_mm')} "
                  f"center={r.get('heel_center_px')} a={r.get('clamp_contact_a_px')} "
                  f"b={r.get('clamp_contact_b_px')} msg={r.get('message')}", flush=True)
            last = sig
    print("FINAL=" + json.dumps(w.result, ensure_ascii=False), flush=True)
    print("ANNOTATED=" + str(out / "annotated.png"), flush=True)
    w.stop()
    os._exit(0)


if __name__ == "__main__":
    main()
