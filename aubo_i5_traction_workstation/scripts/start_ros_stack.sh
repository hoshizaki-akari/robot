#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$PROJECT_DIR/ros2_overlay"
source /opt/ros/humble/setup.bash
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "AUBO overlay is not built: $ROS_WS" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"
export ROS_DOMAIN_ID="${AUBO_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${AUBO_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
echo "Read-only AUBO status launch; robot and sensor connections are opt-in."
exec ros2 launch fr_traction aubo_i5_readonly.launch.py "$@"
