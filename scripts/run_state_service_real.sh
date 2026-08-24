#!/usr/bin/env bash
set -eo pipefail

PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/humble/setup.bash
if [[ -n "${REALSENSE_ROS_SETUP:-}" && -f "$REALSENSE_ROS_SETUP" ]]; then
  source "$REALSENSE_ROS_SETUP"
fi

cd "$PROJECT"
export FR5_STATE_SOURCE=real
exec "$PROJECT/.venv/bin/python" -m uvicorn state_service.app:app \
  --host 127.0.0.1 \
  --port 8765

