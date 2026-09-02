#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
set -u
PROJECT_VENV_BIN="$(dirname "$PROJECT_DIR")/.venv/bin"
if [[ -x "$PROJECT_VENV_BIN/python3" ]]; then
  export PATH="$PROJECT_VENV_BIN:$PATH"
fi
python3 -m compileall -q app.py backend tests
ruff check app.py backend tests
python3 -m unittest discover -s tests -v
echo "FR5 TRACTION WORKSTATION CHECKS PASSED"
