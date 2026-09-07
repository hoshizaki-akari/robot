#!/usr/bin/env bash
set -eo pipefail

# ROS 2 Humble's setup script reads AMENT_TRACE_SETUP_FILES while nounset is
# active. Source both environments first, then restore nounset for checks.
source /opt/ros/humble/setup.bash
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="${FR5_ROS_WS:-$PROJECT_DIR/ros2_overlay}"
FR5_MODEL_WS="${FR5_MODEL_WS:-/home/zhj/projects/fr5_platform_ws/runtimes/directional_correction_v1}"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "FR5 ROS2 工作区不存在或尚未编译：$ROS_WS" >&2
  exit 2
fi
if [[ "$ROS_WS" == "$PROJECT_DIR/ros2_overlay" ]]; then
  if [[ ! -f "$FR5_MODEL_WS/install/setup.bash" ]]; then
    echo "FR5机器人模型基础工作区不存在：$FR5_MODEL_WS" >&2
    exit 2
  fi
  source "$FR5_MODEL_WS/install/setup.bash"
  source "$ROS_WS/install/local_setup.bash"
else
  source "$ROS_WS/install/setup.bash"
fi
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
  /traction/set_operation_mode \
  /traction/stop /traction/emergency_stop /traction/reset_fault \
  /traction/auto_tension_tool_y_minus; do
  if ros2 service list | grep -Fxq "$service"; then
    echo "  [OK] $service"
  else
    echo "  [缺少] $service"
  fi
done
