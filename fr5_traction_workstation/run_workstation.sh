#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WORKSPACE_ROOT="$(cd "$PROJECT_DIR/.." && pwd)"
cd "$PROJECT_DIR"
source /opt/ros/humble/setup.bash
ROS_WS="${FR5_ROS_WS:-$PROJECT_DIR/ros2_overlay}"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "FR5 ROS2 工作区不存在或尚未编译：$ROS_WS" >&2
  exit 2
fi
source "$ROS_WS/install/setup.bash"
set -u
VENV_DIR="${FR5_VENV_DIR:-$WORKSPACE_ROOT/.venv}"
if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
  echo "Python 环境不存在：$VENV_DIR" >&2
  echo "请先运行：bash $PROJECT_DIR/scripts/tablet_setup_env.sh" >&2
  exit 2
fi
source "$VENV_DIR/bin/activate"
WORKSTATION_PORT="${WORKSTATION_PORT:-8081}"

collect_port_pids() {
  fuser -n tcp "$WORKSTATION_PORT" 2>/dev/null |
    tr ' ' '\n' |
    grep -E '^[0-9]+$' || true
}

# Re-running this launcher should replace the previous web process instead of
# failing with "address already in use". Limit termination to processes that
# actually own the selected TCP port; never use a broad Python/uvicorn match.
existing_pids="$(collect_port_pids | sort -nu)"
if [[ -n "$existing_pids" ]]; then
  echo "发现端口 $WORKSTATION_PORT 已被旧进程占用，正在停止旧网页服务。"
  while read -r pid; do
    [[ "$pid" == "$$" ]] || kill -INT "$pid" 2>/dev/null || true
  done <<< "$existing_pids"

  for _ in {1..30}; do
    [[ -z "$(collect_port_pids)" ]] && break
    sleep 0.1
  done

  remaining_pids="$(collect_port_pids | sort -nu)"
  if [[ -n "$remaining_pids" ]]; then
    echo "旧网页服务未及时退出，发送终止信号。"
    while read -r pid; do
      [[ "$pid" == "$$" ]] || kill -TERM "$pid" 2>/dev/null || true
    done <<< "$remaining_pids"
    sleep 0.5
  fi

  remaining_pids="$(collect_port_pids | sort -nu)"
  if [[ -n "$remaining_pids" ]]; then
    echo "端口 $WORKSTATION_PORT 仍未释放，无法安全启动网页服务。" >&2
    exit 4
  fi
fi

exec python -m uvicorn app:app --host 127.0.0.1 --port "$WORKSTATION_PORT"
