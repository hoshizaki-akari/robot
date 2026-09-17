#!/usr/bin/env bash
# One-click native Ubuntu supervisor: ROS 2 + FastAPI + dedicated Firefox.
# Closing the dedicated Firefox window stops both background services.
set -Eeuo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PROJECT_DIR/.." && pwd)"
RUN_DIR="${XDG_RUNTIME_DIR:-/tmp}/fr5-traction-$UID"
LOG_DIR="$HOME/.local/state/fr5-traction/logs"
PROFILE_DIR="$HOME/.local/share/fr5-traction/firefox-profile"
PID_FILE="$RUN_DIR/supervisor.pid"
PORT="${WORKSTATION_PORT:-8081}"
ROBOT_IP="${FR5_ROBOT_IP:-192.168.58.2}"
URL="http://127.0.0.1:$PORT/"
ROS_PID=""
WEB_PID=""
FIREFOX_PID=""

mkdir -p "$RUN_DIR" "$LOG_DIR" "$PROFILE_DIR"
START_STAMP="$(date +%Y%m%d_%H%M%S)"
ROS_LOG="$LOG_DIR/ros_$START_STAMP.log"
WEB_LOG="$LOG_DIR/web_$START_STAMP.log"

show_error() {
  local message="$1"
  zenity --error --title="骨伤牵引机器人工作站" --text="$message" 2>/dev/null || true
  printf '%s\n' "$message" >&2
}

stop_group() {
  local process_id="$1"
  [[ -n "$process_id" ]] || return 0
  if kill -0 "$process_id" 2>/dev/null; then
    kill -INT -- "-$process_id" 2>/dev/null || kill -INT "$process_id" 2>/dev/null || true
    for _ in {1..30}; do
      kill -0 "$process_id" 2>/dev/null || return 0
      sleep 0.1
    done
    kill -TERM -- "-$process_id" 2>/dev/null || kill -TERM "$process_id" 2>/dev/null || true
  fi
}

cleanup() {
  trap - EXIT INT TERM
  [[ -n "$FIREFOX_PID" ]] && kill -TERM "$FIREFOX_PID" 2>/dev/null || true
  stop_group "$WEB_PID"
  stop_group "$ROS_PID"
  rm -f "$PID_FILE"
}
trap cleanup EXIT INT TERM

# A second double-click replaces only the previous workstation supervisor.
if [[ -f "$PID_FILE" ]]; then
  OLD_PID="$(cat "$PID_FILE" 2>/dev/null || true)"
  if [[ "$OLD_PID" =~ ^[0-9]+$ ]] && kill -0 "$OLD_PID" 2>/dev/null; then
    kill -TERM "$OLD_PID" 2>/dev/null || true
    for _ in {1..50}; do
      kill -0 "$OLD_PID" 2>/dev/null || break
      sleep 0.1
    done
  fi
fi
printf '%s\n' "$$" > "$PID_FILE"

for required in \
  /opt/ros/humble/setup.bash \
  "$PROJECT_DIR/ros2_overlay/install/setup.bash" \
  "$WORKSPACE_ROOT/.venv/bin/activate" \
  "$PROJECT_DIR/vendor/fairino-python-sdk/linux/fairino/Robot.py"; do
  if [[ ! -f "$required" ]]; then
    show_error "部署不完整，缺少：$required\n请重新运行 scripts/tablet_setup_env.sh。"
    exit 2
  fi
done

if ! command -v firefox >/dev/null 2>&1; then
  show_error "没有找到 Firefox，请先安装 Firefox。"
  exit 2
fi
if command -v snap >/dev/null 2>&1 && snap list firefox >/dev/null 2>&1; then
  # Ubuntu 22.04 normally ships Firefox as a confined snap. Keep the dedicated
  # profile in the snap-writable area so kiosk startup is reliable.
  PROFILE_DIR="$HOME/snap/firefox/common/fr5-traction-profile"
  mkdir -p "$PROFILE_DIR"
fi

# The controller's XML-RPC endpoint is a more useful preflight than ping.
if ! timeout 2 bash -c "</dev/tcp/$ROBOT_IP/20003" 2>/dev/null; then
  show_error "无法连接机械臂 $ROBOT_IP:20003。\n请检查网线和机器人网口的静态 IP 设置。"
  exit 3
fi

export FR5_SDK_PYTHON_PATH="$PROJECT_DIR/vendor/fairino-python-sdk/linux"
export FR5_TRACTION_DATA_DIR="$PROJECT_DIR/debug/traction_sessions"

setsid bash "$PROJECT_DIR/scripts/start_ros_stack.sh" 2 >"$ROS_LOG" 2>&1 &
ROS_PID=$!

# Give the driver enough time for its startup RPC and force reference setup.
DRIVER_READY=0
for _ in {1..60}; do
  kill -0 "$ROS_PID" 2>/dev/null || {
    show_error "ROS 2 控制服务启动失败。\n日志：$ROS_LOG"
    exit 4
  }
  if grep -q "FR5 direct driver connected" "$ROS_LOG" 2>/dev/null; then
    DRIVER_READY=1
    break
  fi
  sleep 0.5
done
if [[ "$DRIVER_READY" -ne 1 ]]; then
  show_error "机械臂驱动在规定时间内没有就绪。\n日志：$ROS_LOG"
  exit 4
fi

setsid bash "$PROJECT_DIR/run_workstation.sh" >"$WEB_LOG" 2>&1 &
WEB_PID=$!
for _ in {1..60}; do
  kill -0 "$WEB_PID" 2>/dev/null || {
    show_error "网页服务启动失败。\n日志：$WEB_LOG"
    exit 5
  }
  curl -fsS "$URL" >/dev/null 2>&1 && break
  sleep 0.25
done
if ! curl -fsS "$URL" >/dev/null 2>&1; then
  show_error "网页服务在规定时间内没有就绪。\n日志：$WEB_LOG"
  exit 5
fi

notify-send "骨伤牵引机器人工作站" "系统已启动；关闭此专用浏览器窗口将同时停止控制服务。" 2>/dev/null || true
firefox --kiosk --no-remote --profile "$PROFILE_DIR" "$URL" &
FIREFOX_PID=$!
wait "$FIREFOX_PID" || true
