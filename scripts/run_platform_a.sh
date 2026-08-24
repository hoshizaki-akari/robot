#!/usr/bin/env bash
set -euo pipefail

source /opt/ros/humble/setup.bash
source /home/zhj/projects/fr5_platform_ws/.venv/bin/activate
cd /home/zhj/projects/fr5_platform_ws
exec env PYTHONPATH=. python platform_a/main.py
