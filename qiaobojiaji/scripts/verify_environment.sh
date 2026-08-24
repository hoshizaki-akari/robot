#!/usr/bin/env bash
# Read-only preflight check for the standalone Platform A package.
set -eo pipefail

# ROS Humble's setup scripts access optional variables, so source it before
# enabling nounset in this shell.
source /opt/ros/humble/setup.bash
set -u

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$PROJECT_ROOT/.venv/bin/activate" ]]; then
  VENV_ACTIVATE="$PROJECT_ROOT/.venv/bin/activate"
elif [[ -f "$PROJECT_ROOT/../.venv/bin/activate" ]]; then
  VENV_ACTIVATE="$PROJECT_ROOT/../.venv/bin/activate"
else
  echo "FAIL: no virtual environment found. Run: bash scripts/install_dependencies.sh" >&2
  exit 1
fi

source "$VENV_ACTIVATE"
cd "$PROJECT_ROOT"
export PYTHONPATH=".:${PYTHONPATH:-}"
export LD_LIBRARY_PATH="/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"

python - <<'PY'
import importlib

modules = ("cv2", "numpy", "serial", "ultralytics", "fairino")
for module in modules:
    importlib.import_module(module)

from platform_a.calcaneus_robot.ui import CalcaneusRobotApp  # noqa: F401
from platform_a.pry_buckle_vision import PryBuckleVisionWorker  # noqa: F401
print("PASS: ROS, Python dependencies, Platform A UI, and pry/buckle vision imports are available.")
PY
