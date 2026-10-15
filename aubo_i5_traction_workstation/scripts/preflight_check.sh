#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
ROS_WS="$PROJECT_DIR/ros2_overlay"
source /opt/ros/humble/setup.bash
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "AUBO ROS2 workspace is not built: $ROS_WS" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"
export ROS_DOMAIN_ID="${AUBO_ROS_DOMAIN_ID:-42}"

echo "=== AUBO i5 traction workstation (ROS domain $ROS_DOMAIN_ID) ==="
for topic in /joint_states /controller_manager/ee_state \
  /aubo/kwr75/raw_wrench /aubo/kwr75/raw_wrench_valid \
  /controller_manager/wrench /traction/corrected_wrench \
  /traction/status /controller_manager/healthy; do
  if ros2 topic list | grep -Fxq "$topic"; then
    echo "[topic OK] $topic"
  else
    echo "[topic missing] $topic"
  fi
done
echo "Read-only checks do not certify motion, force direction, or traction safety."
