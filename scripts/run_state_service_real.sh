#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
realsense_setup="/home/zhj/projects/fr5_learning/robot_ws_backup/ros2_realsense/install/setup.bash"
if [[ -f "$realsense_setup" ]]; then
  source "$realsense_setup"
fi

cd /home/zhj/projects/fr5_platform_ws
export FR5_STATE_SOURCE=real
exec .venv/bin/python -m uvicorn state_service.app:app \
  --host 127.0.0.1 \
  --port 8765
