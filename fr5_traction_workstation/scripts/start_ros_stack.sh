#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
ROS_WS="${FR5_ROS_WS:-/home/zhj/projects/fr5_learning/robot_ws_backup/new_fairino_ws}"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "FR5 ROS2 工作区不存在或尚未编译：$ROS_WS" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"

exec ros2 launch fr_traction traction_system.launch.py \
  robot_ip="${FR5_ROBOT_IP:-192.168.58.2}" \
  zero_sensor_on_activate="${FR5_ZERO_SENSOR_ON_ACTIVATE:-true}" \
  use_web_bridge=false \
  use_rviz="${FR5_USE_RVIZ:-false}" \
  data_directory="${FR5_TRACTION_DATA_DIR:-debug/traction_sessions}"
