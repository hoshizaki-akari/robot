"""验证夹持分支（D435Monitor）已改用 PryBuckleVisionWorker。

直接实例化 state_service.D435Monitor，启动其视觉线程（现在内部委托
PryBuckleVisionWorker），读取 vision_snapshot() / annotated_png() 确认
夹持视觉结果与撬拨算法一致（valid / heel_width_mm / 标注图）。
"""
from __future__ import annotations
import os, sys, time, threading
from pathlib import Path

PROJ = Path("/home/zhj/projects/fr5_platform_ws")
for p in (str(PROJ), str(PROJ / ".venv/lib/python3.10/site-packages")):
    if p not in sys.path:
        sys.path.insert(0, p)

# 让 ROS 依赖可见
for lib in ("/opt/ros/humble/local/lib/python3.10/dist-packages",
            "/opt/ros/humble/lib/python3.10/site-packages"):
    if lib not in sys.path:
        sys.path.insert(0, lib)


def main() -> None:
    from state_service.d435_monitor import D435Monitor
    mon = D435Monitor()
    mon.start()
    print("D435Monitor started; polling vision_snapshot for ~8s ...", flush=True)
    last = None
    out_dir = PROJ / "debug" / "clamp_via_monitor"
    out_dir.mkdir(parents=True, exist_ok=True)
    for i in range(16):
        time.sleep(0.5)
        snap = mon.vision_snapshot()
        r = mon.annotated_png()
        if r is not None:
            with open(str(out_dir / "annotated.png"), "wb") as f:
                f.write(r)
        msg = snap.get("message", "")
        if snap != last:
            print(f"[{i*0.5:4.1f}s] valid={snap.get('valid')} "
                  f"heel_detected={snap.get('heel_detected')} "
                  f"width_mm={snap.get('heel_width_mm')} "
                  f"center_px={snap.get('heel_center_px')} "
                  f"a_px={snap.get('clamp_contact_a_px')} b_px={snap.get('clamp_contact_b_px')} "
                  f"msg={msg}", flush=True)
            last = snap
    final = mon.vision_snapshot()
    print("FINAL_SNAPSHOT=" + __import__("json").dumps(final, ensure_ascii=False), flush=True)
    print("ANNOTATED_PATH=" + str(out_dir / "annotated.png"), flush=True)
    mon._stop.set() if hasattr(mon, "_stop") else None
    os._exit(0)


if __name__ == "__main__":
    main()
