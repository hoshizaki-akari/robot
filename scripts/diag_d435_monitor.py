import sys
from pathlib import Path

sys.path.insert(0, str(Path("/home/zhj/projects/fr5_platform_ws")))

from state_service.d435_monitor import D435Monitor
import time

mon = D435Monitor()
mon.start()
for i in range(15):
    time.sleep(1.0)
    s = mon.snapshot()
    print(f"{i+1:02d}s valid={s['valid']} color_fps={s['color_fps']:.1f} depth_fps={s['depth_fps']:.1f} msg={s.get('message')!r}")
    if s['valid']:
        break
print("vision:", mon.vision_snapshot())
mon.stop()
