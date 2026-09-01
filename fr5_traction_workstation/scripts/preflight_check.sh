#!/usr/bin/env bash
set -eo pipefail

# ROS 2 Humble's setup script reads AMENT_TRACE_SETUP_FILES while nounset is
# active. Source both environments first, then restore nounset for checks.
source /opt/ros/humble/setup.bash
ROS_WS="${FR5_ROS_WS:-/home/zhj/projects/fr5_learning/robot_ws_backup/new_fairino_ws}"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "FR5 ROS2 工作区不存在或尚未编译：$ROS_WS" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"
set -u

echo "=== FR5 牵引系统预检 ==="
echo "节点："
ros2 node list | sed -n '1,80p'
echo "关键话题："
for topic in /joint_states /controller_manager/wrench /traction/corrected_wrench \
  /traction/status /traction_controller/healthy /controller_manager/healthy; do
  if ros2 topic list | grep -Fxq "$topic"; then
    echo "  [OK] $topic"
  else
    echo "  [缺少] $topic"
  fi
done
echo "关键服务："
for service in /traction/set_zero_pose /traction/return_zero_pose /traction/prepare \
  /traction/calibrate_direction /traction/set_target_force /traction/start \
  /traction/stop /traction/emergency_stop /traction/reset_fault \
  /traction/auto_tension_tool_y_minus; do
  if ros2 service list | grep -Fxq "$service"; then
    echo "  [OK] $service"
  else
    echo "  [缺少] $service"
  fi
done
