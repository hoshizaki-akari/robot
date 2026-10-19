#!/usr/bin/env bash
set -eo pipefail

# The web server only subscribes to the isolated AUBO ROS graph. It never
# starts the controller or opens a vendor SDK connection by itself.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
ROS_WS="$PROJECT_DIR/ros2_overlay"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "AUBO ROS2 workspace is not built: $ROS_WS" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
VENV_DIR="${AUBO_VENV_DIR:-$PROJECT_DIR/.venv}"
if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
  echo "AUBO workstation Python environment is missing: $VENV_DIR" >&2
  echo "Create it with: bash scripts/tablet_setup_env.sh" >&2
  exit 2
fi
source "$VENV_DIR/bin/activate"
set -u
export ROS_DOMAIN_ID="${AUBO_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${AUBO_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${AUBO_ROS_LOCALHOST_ONLY:-1}"
WORKSTATION_PORT="${WORKSTATION_PORT:-8082}"
cd "$PROJECT_DIR"
echo "WEB-ONLY: this starts the browser UI, not the AUBO SDK driver or COM4 sensor bridge."
echo "If the robot card says disconnected, that is expected in this mode."
echo "Do not use AUBO_TRACTION_ENABLED as a hardware launcher; see README.md for the approved live-workstation checkpoint."
exec python -m uvicorn app:app --host 127.0.0.1 --port "$WORKSTATION_PORT"
