#!/usr/bin/env bash
set -eo pipefail

DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${DEMO_DIR}/.." && pwd)"
if [[ -f /opt/ros/humble/setup.bash ]]; then source /opt/ros/humble/setup.bash; fi
if [[ -f "${WORKSPACE_DIR}/.venv/bin/activate" ]]; then source "${WORKSPACE_DIR}/.venv/bin/activate"; fi
set -u
cd "${DEMO_DIR}"

python -m compileall -q app.py force_demo tests
ruff check app.py force_demo tests
python -m unittest discover -s tests -v
if command -v node >/dev/null 2>&1; then
  node --check static/app.js
else
  echo "JS syntax check skipped: node is not installed in this WSL distribution"
fi
echo "ALL FORCE TENSION DEMO CHECKS PASSED"
