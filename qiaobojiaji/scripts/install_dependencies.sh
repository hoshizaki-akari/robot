#!/usr/bin/env bash
set -euo pipefail

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PYTHON_BIN="${PYTHON_BIN:-python3}"

if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "ROS 2 Humble is required: /opt/ros/humble/setup.bash not found." >&2
  exit 1
fi

"$PYTHON_BIN" -m venv "$PROJECT_ROOT/.venv"
source "$PROJECT_ROOT/.venv/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "$PROJECT_ROOT/requirements.txt"
python -m pip install -r "$PROJECT_ROOT/requirements-vision.txt"

echo "Python dependencies installed. Install the vendor fairino SDK separately."
