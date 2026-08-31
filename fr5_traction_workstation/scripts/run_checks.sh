#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
set -u
python -m compileall -q app.py backend tests
ruff check app.py backend tests
python -m unittest discover -s tests -v
echo "FR5 TRACTION WORKSTATION CHECKS PASSED"
