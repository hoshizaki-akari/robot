#!/usr/bin/env bash
set -euo pipefail

PROJECT="/home/zhj/projects/fr5_platform_ws"
source "$PROJECT/.venv/bin/activate"
exec python "$PROJECT/scripts/robot_switch.py" "$@"
