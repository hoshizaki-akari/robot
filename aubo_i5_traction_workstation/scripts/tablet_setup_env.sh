#!/usr/bin/env bash
set -eo pipefail

# Deliberately does not install a vendor SDK, connect hardware, or enable motion.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
VENV_DIR="${AUBO_VENV_DIR:-$PROJECT_DIR/.venv}"
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required at /opt/ros/humble/setup.bash" >&2
  exit 2
fi
if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
  python3 -m venv --system-site-packages "$VENV_DIR"
fi
source "$VENV_DIR/bin/activate"
if [[ -n "${AUBO_WHEEL_DIR:-}" ]]; then
  python -m pip install --no-index --find-links "$AUBO_WHEEL_DIR" -r "$PROJECT_DIR/requirements.txt"
else
  python -m pip install -r "$PROJECT_DIR/requirements.txt"
fi
echo "AUBO-only Python environment ready: $VENV_DIR"
echo "Build the ROS overlay separately; the external V4 SDK is required for aubo_v4_driver."
