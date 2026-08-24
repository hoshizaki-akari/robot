#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u
PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
if [[ -f "$PROJECT_ROOT/.venv/bin/activate" ]]; then
  VENV_ACTIVATE="$PROJECT_ROOT/.venv/bin/activate"
elif [[ -f "$PROJECT_ROOT/../.venv/bin/activate" ]]; then
  # Development convenience: reuse the existing parent workspace venv until
  # this standalone package has installed its own dependencies.
  VENV_ACTIVATE="$PROJECT_ROOT/../.venv/bin/activate"
  echo "Using parent workspace virtual environment: $VENV_ACTIVATE" >&2
else
  echo "No Python virtual environment found." >&2
  echo "Run: bash scripts/install_dependencies.sh" >&2
  exit 1
fi
source "$VENV_ACTIVATE"
cd "$PROJECT_ROOT"
if [[ "${PLATFORM_A_SKIP_SERVICES:-0}" != "1" ]]; then
  bash "$PROJECT_ROOT/scripts/start_runtime_services.sh"
fi
exec env \
  PYTHONPATH=".:${PYTHONPATH:-}" \
  LD_LIBRARY_PATH="/opt/ros/humble/lib${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}" \
  python platform_a/main.py
