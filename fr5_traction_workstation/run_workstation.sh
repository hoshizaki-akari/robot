#!/usr/bin/env bash
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
ROS_WS="${FR5_ROS_WS:-/home/zhj/projects/fr5_learning/robot_ws_backup/new_fairino_ws}"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "FR5 ROS2 工作区不存在或尚未编译：$ROS_WS" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"
source /home/zhj/projects/fr5_platform_ws/.venv/bin/activate
WORKSTATION_PORT="${WORKSTATION_PORT:-8081}"
exec python -m uvicorn app:app --host 127.0.0.1 --port "$WORKSTATION_PORT"
