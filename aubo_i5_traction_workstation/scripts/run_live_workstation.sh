#!/usr/bin/env bash
# Explicit hardware launch; never starts a traction session or moves on startup.
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PROFILE="${1:?Pass the locally verified wrench/commissioning YAML}"
SESSION_DIR="${2:?Pass the log directory}"
test -f "$PROFILE"
test -n "${AUBO_V4_USERNAME:-}"
test -n "${AUBO_V4_PASSWORD:-}"
mkdir -p "$SESSION_DIR"
exec 9>"$PROJECT_DIR/debug/live_workstation.lock"
flock -n 9 || { echo 'Another live workstation supervisor owns this checkout.' >&2; exit 2; }
if pgrep -f '/aubo_v4_(stepwise|state)_node( |$)' >/dev/null; then
  echo 'An AUBO SDK owner is already running; refusing a duplicate connection.' >&2
  exit 2
fi
if [[ -n "$(ss -Hltn 'sport = :8082')" ]]; then
  echo 'WSL port 8082 is already owned; stop the previous web process first.' >&2
  exit 2
fi

SDK_ROOT="${AUBO_V4_SDK_ROOT:-/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit}"
PROTOBUF_DIR="${AUBO_V4_PROTOBUF_DIR:-/mnt/c/Users/zhj/AppData/Local/Temp/aubo-v4-runtime-audit/extracted/usr/lib/x86_64-linux-gnu}"
export LD_LIBRARY_PATH="$PROTOBUF_DIR:$SDK_ROOT/lib/lib64/aubocontroller:$SDK_ROOT/lib/lib64/config:$SDK_ROOT/lib/lib64/log4cplus:${LD_LIBRARY_PATH:-}"
source /opt/ros/humble/setup.bash
source "$PROJECT_DIR/ros2_overlay/install/setup.bash"
export ROS_DOMAIN_ID=42 RMW_IMPLEMENTATION=rmw_cyclonedds_cpp ROS_LOCALHOST_ONLY=1
export AUBO_TRACTION_ENABLED=true
cd "$PROJECT_DIR"

pids=()
names=()
launch() {
  local name="$1"
  shift
  "$@" >"$SESSION_DIR/$name.log" 2>&1 &
  pids+=("$!")
  names+=("$name")
  printf '%s %s\n' "$!" "$name" >>"$SESSION_DIR/processes.txt"
}
cleanup() {
  trap - EXIT INT TERM
  # Request the existing measured-stop path before terminating the SDK owner.
  curl -s --max-time 3 -H 'Content-Type: application/json' -d '{}' \
    http://127.0.0.1:8082/api/traction/stop >/dev/null 2>&1 || true
  for pid in "${pids[@]}"; do kill -INT "$pid" 2>/dev/null || true; done
  for attempt in {1..50}; do
    local alive=false
    for pid in "${pids[@]}"; do kill -0 "$pid" 2>/dev/null && alive=true; done
    if ! $alive; then break; fi
    sleep 0.1
  done
  for pid in "${pids[@]}"; do kill -TERM "$pid" 2>/dev/null || true; done
  wait 2>/dev/null || true
}
trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

launch sensor_pty python3 scripts/kwr75_wsl_pty_bridge.py
for attempt in {1..50}; do
  if [[ -e /tmp/kwr75d-bridge-tty ]]; then break; fi
  kill -0 "${pids[0]}" 2>/dev/null || { echo 'Sensor PTY failed; see sensor_pty.log.' >&2; exit 2; }
  sleep 0.1
done
test -e /tmp/kwr75d-bridge-tty
BIN="$PROJECT_DIR/ros2_overlay/install"
launch robot "$BIN/aubo_v4_driver/lib/aubo_v4_driver/aubo_v4_stepwise_node" \
  --ros-args -p connect_hardware:=true -p enable_motion_interface:=true \
  -p maximum_step_m:=0.001 -p planning_horizon_s:=0.5 -p arrival_tolerance_m:=0.0002
launch sensor "$BIN/aubo_kwr75_sensor/lib/aubo_kwr75_sensor/kwr75_sensor_node" \
  --ros-args -p serial_path:=/tmp/kwr75d-bridge-tty
launch wrench "$BIN/fr_traction/lib/fr_traction/aubo_wrench_transform_node.py" \
  --ros-args --params-file "$PROFILE"
CONFIG="$PROJECT_DIR/ros2_overlay/src/fr_traction/config/traction_params.yaml"
launch controller "$BIN/fr_traction/lib/fr_traction/traction_controller_node" \
  --ros-args -r __node:=traction_controller --params-file "$CONFIG" --params-file "$PROFILE"
launch manager "$BIN/fr_traction/lib/fr_traction/traction_manager_node" \
  --ros-args -r __node:=traction_manager --params-file "$CONFIG" --params-file "$PROFILE"
launch web bash run_workstation.sh
printf 'Workstation supervisor started; no traction session started. Logs: %s\n' "$SESSION_DIR"
# A failed component must not leave an apparently healthy partial workstation.
set +e
wait -n "${pids[@]}"
component_status=$?
set -e
printf 'A component exited (status %s); stopping this supervisor.\n' "$component_status" >&2
exit "$component_status"
