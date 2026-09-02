#!/usr/bin/env bash
set -eo pipefail

# ROS 2 Humble's setup script reads AMENT_TRACE_SETUP_FILES while nounset is
# active. Source the ROS environments without nounset, then restore strict
# mode for the actual launch and parameter handling below.
source /opt/ros/humble/setup.bash
ROS_WS="${FR5_ROS_WS:-/home/zhj/projects/fr5_learning/robot_ws_backup/new_fairino_ws}"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "FR5 ROS2 工作区不存在或尚未编译：$ROS_WS" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"
set -u

# A real FR5 accepts only one direct-driver connection at a time. Starting a
# second launch while an old stack is still alive makes the second driver look
# connected at the SDK level but unable to receive realtime state packets.
if pgrep -u "$(id -u)" -f '[f]r5_direct_driver_node.py' >/dev/null 2>&1 || \
   pgrep -u "$(id -u)" -f '[t]raction_manager_node' >/dev/null 2>&1 || \
   pgrep -u "$(id -u)" -f '[t]raction_controller_node' >/dev/null 2>&1; then
  echo "FR5 ROS 控制栈已经在运行，拒绝重复启动。请先在原终端按 Ctrl+C 停止旧栈，再重新运行本脚本。" >&2
  exit 3
fi

exec ros2 launch fr_traction traction_system.launch.py \
  robot_ip:="${FR5_ROBOT_IP:-192.168.58.2}" \
  zero_sensor_on_activate:="${FR5_ZERO_SENSOR_ON_ACTIVATE:-true}" \
  use_web_bridge:=false \
  use_rviz:="${FR5_USE_RVIZ:-false}" \
  data_directory:="${FR5_TRACTION_DATA_DIR:-debug/traction_sessions}"
