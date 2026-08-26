# Platform B ROS2 traction UI

`gateway.py` is the REST/WebSocket gateway. It runs an `rclpy` bridge that
subscribes to `/traction/status` and `/traction/history`, publishes the
optional `/traction/ui_heartbeat`, and forwards explicit manager services. It
does not calculate force, store fake history, or send SDK motion commands.

The active page is `control.html`; `090105.html` remains a compatibility page
and uses the same ROS2 traction endpoints. Both pages show only values received
from the ROS2 manager. Authentication/case display state may remain in browser
session storage, but traction target/history are not stored in local storage.

Start the ROS workspace first:

```bash
source /opt/ros/humble/setup.bash
source /home/zhj/projects/fr5_learning/robot_ws_backup/new_fairino_ws/install/setup.bash
ros2 launch fr_traction traction_system.launch.py robot_ip:=192.168.58.2
```

Then start this gateway from an environment that has ROS Humble sourced:

```bash
cd /home/zhj/projects/fr5_platform_ws
source .venv/bin/activate
python platform_b/gateway.py
```

Open `http://127.0.0.1:8080`. The UI sequence is `准备` → manual coarse
setup on the teach pendant → `方向标定` → verify `DIRECTION_LOCKED` → set a
1–20 N target → `开始恒力` → `正常释放`. The software emergency-stop button
does not replace the physical E-stop.

If the ROS manager is not running, the bridge reports `ROS2 牵引管理器不可用`
and the operation buttons remain disabled. This is intentional; the UI never
falls back to simulated force or browser-local records.
