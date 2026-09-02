#!/usr/bin/env bash
set -eo pipefail

# ROS 2 Humble's setup script reads AMENT_TRACE_SETUP_FILES while nounset is
# active. Source the ROS environments without nounset, then restore strict
# mode for the actual launch and parameter handling below.
source /opt/ros/humble/setup.bash
VERSION_CHOICE="${1:-}"

if [[ -z "$VERSION_CHOICE" ]]; then
  if [[ -t 0 ]]; then
    echo "请选择版本：1=第一版稳定基线，2=当前方向纠偏版（默认2）"
    read -r VERSION_CHOICE
  else
    VERSION_CHOICE=2
  fi
fi
VERSION_CHOICE="${VERSION_CHOICE:-2}"

case "$VERSION_CHOICE" in
  1)
    ROS_WS="/home/zhj/projects/fr5_platform_ws/runtimes/force_stable_v1"
    VERSION_LABEL="版本1：稳定基线"
    ;;
  2)
    ROS_WS="/home/zhj/projects/fr5_platform_ws/runtimes/directional_correction_v1"
    VERSION_LABEL="版本2：方向纠偏"
    ;;
  *)
    echo "版本选择无效，请输入 1 或 2。" >&2
    exit 2
    ;;
esac

echo "准备启动 $VERSION_LABEL。"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "FR5 ROS2 工作区不存在或尚未编译：$ROS_WS" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"
set -u

# A real FR5 accepts only one direct-driver connection at a time. Stop an
# earlier stack cleanly before starting the selected version.
collect_stack_pids() {
  {
    pgrep -u "$(id -u)" -f '[r]os2 launch fr_traction traction_system.launch.py' || true
    pgrep -u "$(id -u)" -f '[f]r5_direct_driver_node.py' || true
    pgrep -u "$(id -u)" -f '[t]raction_manager_node' || true
    pgrep -u "$(id -u)" -f '[t]raction_controller_node' || true
  } | sort -nu
}

existing_pids="$(collect_stack_pids)"
if [[ -n "$existing_pids" ]]; then
  echo "发现旧的 FR5 ROS 控制栈，先正常停止它。"
  while read -r pid; do
    kill -INT "$pid" 2>/dev/null || true
  done <<< "$existing_pids"

  remaining="$existing_pids"
  for _ in {1..30}; do
    sleep 0.1
    remaining="$(collect_stack_pids)"
    [[ -z "$remaining" ]] && break
  done

  if [[ -n "$remaining" ]]; then
    echo "旧控制栈未及时退出，发送终止信号。" >&2
    while read -r pid; do
      kill -TERM "$pid" 2>/dev/null || true
    done <<< "$remaining"
    sleep 0.5
    remaining="$(collect_stack_pids)"
  fi

  if [[ -n "$remaining" ]]; then
    echo "旧 FR5 ROS 控制栈仍未退出，取消本次启动。" >&2
    exit 4
  fi
fi

exec ros2 launch fr_traction traction_system.launch.py \
  robot_ip:="${FR5_ROBOT_IP:-192.168.58.2}" \
  zero_sensor_on_activate:="${FR5_ZERO_SENSOR_ON_ACTIVATE:-true}" \
  use_web_bridge:=false \
  use_rviz:="${FR5_USE_RVIZ:-false}" \
  data_directory:="${FR5_TRACTION_DATA_DIR:-debug/traction_sessions}"
