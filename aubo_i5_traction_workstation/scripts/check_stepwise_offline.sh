#!/usr/bin/env bash
# Offline acceptance only: no robot login, serial sensor, or operational web launch.
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
SDK_ROOT="${AUBO_V4_SDK_ROOT:-/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit}"
PROTOBUF_DIR="${AUBO_V4_PROTOBUF_DIR:-/mnt/c/Users/zhj/AppData/Local/Temp/aubo-v4-runtime-audit/extracted/usr/lib/x86_64-linux-gnu}"
test -f "$SDK_ROOT/include/aubo_driver/serviceinterface.h"
test -e "$PROTOBUF_DIR/libprotobuf.so.9"
export AUBO_V4_SDK_ROOT="$SDK_ROOT"
export LD_LIBRARY_PATH="$PROTOBUF_DIR:$SDK_ROOT/lib/lib64/aubocontroller:$SDK_ROOT/lib/lib64/config:$SDK_ROOT/lib/lib64/log4cplus:${LD_LIBRARY_PATH:-}"
source /opt/ros/humble/setup.bash
cd "$PROJECT_DIR/ros2_overlay"
colcon build --packages-select fr_traction aubo_v4_driver --symlink-install \
  --cmake-args "-DAUBO_V4_SDK_ROOT=$SDK_ROOT"
source install/setup.bash
export PYTEST_DISABLE_PLUGIN_AUTOLOAD=1
colcon test --packages-select fr_traction aubo_v4_driver
colcon test-result --test-result-base build/fr_traction --verbose
colcon test-result --test-result-base build/aubo_v4_driver --verbose
ctest --test-dir build/aubo_v4_driver \
  -R 'test_stepwise_scheduler|test_stepwise_transport' \
  --repeat until-fail:20 --output-on-failure
bash "$PROJECT_DIR/scripts/run_checks.sh"
echo 'Offline acceptance passed. Hardware motion has NOT been validated.'
