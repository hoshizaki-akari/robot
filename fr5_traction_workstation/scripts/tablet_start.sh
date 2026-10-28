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
DRIVER_CONNECTED=0
CONNECTING=0
ROS_ATTEMPT=0
NEXT_ROS_ATTEMPT=0
DRIVER_DEADLINE=0

mkdir -p "$RUN_DIR" "$LOG_DIR" "$PROFILE_DIR"
START_STAMP="$(date +%Y%m%d_%H%M%S)"
ROS_LOG="$LOG_DIR/ros_$START_STAMP.log"
WEB_LOG="$LOG_DIR/web_$START_STAMP.log"
STARTUP_LOG="$LOG_DIR/startup_$START_STAMP.log"

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
  if [[ "$(cat "$PID_FILE" 2>/dev/null || true)" == "$$" ]]; then
    rm -f "$PID_FILE"
  fi
}
trap cleanup EXIT
trap 'exit 0' INT TERM

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

export FR5_SDK_PYTHON_PATH="$PROJECT_DIR/vendor/fairino-python-sdk/linux"
export FR5_TRACTION_DATA_DIR="$PROJECT_DIR/debug/traction_sessions"
# The web shutdown endpoint signals this supervisor. Its cleanup trap then
# closes Firefox and stops both FastAPI and the ROS 2 control process groups.
export FR5_WORKSTATION_SUPERVISOR_PID="$$"

# Render the local UI even while the controller is still booting. RosBridge
# discovers live ROS publishers later; it never synthesizes robot feedback.
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

notify-send "骨伤牵引机器人工作站" "界面已启动，正在等待机械臂连接。" 2>/dev/null || true
firefox --kiosk --no-remote --profile "$PROFILE_DIR" "$URL" &
FIREFOX_PID=$!

log_startup() {
  printf '%s %s\n' "$(date --iso-8601=seconds)" "$1" >> "$STARTUP_LOG"
}

log_startup "网页已启动，等待机械臂 $ROBOT_IP:20003；关闭界面将取消等待。"
while kill -0 "$FIREFOX_PID" 2>/dev/null; do
  if ! kill -0 "$WEB_PID" 2>/dev/null; then
    show_error "网页服务已退出。\n日志：$WEB_LOG"
    exit 5
  fi

  if [[ "$CONNECTING" -eq 1 ]]; then
    # A connected driver may already have acquired control/calibrated. Never
    # restart it automatically after a runtime fault or dropped connection.
    if grep -q "FR5 direct driver connected" "$ROS_LOG" 2>/dev/null; then
      DRIVER_CONNECTED=1
      CONNECTING=0
      log_startup "机械臂驱动已连接；后续运行故障不自动重启控制栈。"
    elif ! kill -0 "$ROS_PID" 2>/dev/null ||
         grep -qE 'fr5_direct_driver_node.py.*process has died|Traceback \(most recent call last\)' "$ROS_LOG" 2>/dev/null ||
         (( SECONDS >= DRIVER_DEADLINE )); then
      log_startup "机械臂尚未完成初始化，保留网页，清理本次启动后重试。日志：$ROS_LOG"
      stop_group "$ROS_PID"
      wait "$ROS_PID" 2>/dev/null || true
      ROS_PID=""
      CONNECTING=0
      NEXT_ROS_ATTEMPT=$((SECONDS + 3))
    fi
  elif [[ "$DRIVER_CONNECTED" -eq 0 ]] && (( SECONDS >= NEXT_ROS_ATTEMPT )); then
    # Probe only the command endpoint, never claim the single-owner 20004
    # state stream. No SDK process exists until the controller port is up.
    if timeout 2 bash -c 'exec 3<>"/dev/tcp/$1/20003"' _ "$ROBOT_IP" 2>/dev/null; then
      # Closing Firefox during a probe must not start a late hardware owner.
      kill -0 "$FIREFOX_PID" 2>/dev/null || break
      ROS_ATTEMPT=$((ROS_ATTEMPT + 1))
      ROS_LOG="$LOG_DIR/ros_${START_STAMP}_attempt${ROS_ATTEMPT}.log"
      setsid bash "$PROJECT_DIR/scripts/start_ros_stack.sh" 2 >"$ROS_LOG" 2>&1 &
      ROS_PID=$!
      CONNECTING=1
      DRIVER_DEADLINE=$((SECONDS + 45))
      log_startup "启动机械臂连接，第${ROS_ATTEMPT}次。日志：$ROS_LOG"
    else
      NEXT_ROS_ATTEMPT=$((SECONDS + 3))
    fi
  elif [[ "$DRIVER_CONNECTED" -eq 1 && -n "$ROS_PID" ]] &&
       ! kill -0 "$ROS_PID" 2>/dev/null; then
    wait "$ROS_PID" 2>/dev/null || true
    ROS_PID=""
    log_startup "运行中的控制栈已退出，保留网页；请检查故障后手动重新启动软件。"
  fi
  sleep 0.5
done
wait "$FIREFOX_PID" 2>/dev/null || true
