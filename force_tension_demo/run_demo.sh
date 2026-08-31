#!/usr/bin/env bash
set -eo pipefail

DEMO_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_DIR="$(cd "${DEMO_DIR}/.." && pwd)"

if [[ -f /opt/ros/humble/setup.bash ]]; then
  source /opt/ros/humble/setup.bash
fi
if [[ -f "${WORKSPACE_DIR}/.venv/bin/activate" ]]; then
  source "${WORKSPACE_DIR}/.venv/bin/activate"
fi
set -u

cd "${DEMO_DIR}"
export PYTHONUNBUFFERED=1
exec python -m uvicorn app:app --host 127.0.0.1 --port "${FORCE_DEMO_PORT:-8092}"
