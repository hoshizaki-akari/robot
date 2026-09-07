#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
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
source /home/zhj/projects/fr5_platform_ws/.venv/bin/activate
WORKSTATION_PORT="${WORKSTATION_PORT:-8081}"
exec python -m uvicorn app:app --host 127.0.0.1 --port "$WORKSTATION_PORT"
