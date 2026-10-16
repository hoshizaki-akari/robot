#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/.venv/bin/activate"
python -m compileall -q app.py backend tests scripts
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python -m pytest -q tests
python -m ruff check app.py backend \
  ros2_overlay/src/fr_traction/scripts/aubo_wrench_transform_node.py \
  tests/test_control_config_contract.py tests/test_emergency_recover.py \
  scripts/capture_force_checkpoint.py scripts/fit_force_axes.py \
  scripts/live_motion_bounds.py scripts/verify_stepwise_live.py \
  tests/test_force_axis_fit.py tests/test_live_motion_bounds.py
echo "AUBO i5 workstation offline Python checks passed"
