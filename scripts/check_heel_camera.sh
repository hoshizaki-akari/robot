#!/usr/bin/env bash
set -euo pipefail

PROJECT="/home/zhj/projects/fr5_platform_ws"

source /opt/ros/humble/setup.bash
source /home/zhj/projects/fr5_learning/robot_ws_backup/ros2_realsense/install/setup.bash
source "$PROJECT/.venv/bin/activate"

exec python "$PROJECT/scripts/check_heel_camera.py"
