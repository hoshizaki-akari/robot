#!/usr/bin/env bash
set -eo pipefail

# Launch only the AUBO web UI. It does not start a robot connection or motion.
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
echo "AUBO i5 web UI is read-only until the AUBO ROS graph is separately validated."
echo "Open http://127.0.0.1:${WORKSTATION_PORT:-8082}/ in a browser."
exec bash "$PROJECT_DIR/run_workstation.sh"
