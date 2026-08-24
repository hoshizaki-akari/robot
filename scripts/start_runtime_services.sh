#!/usr/bin/env bash
# Start only the services required by the standalone Platform A GUI.
# This intentionally does not start Platform A a second time and does not
# send any robot motion command.
set -eo pipefail

source /opt/ros/humble/setup.bash
set -u

PROJECT_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="${FR5_PLATFORM_WS_ROOT:-$(cd "$PROJECT_ROOT/.." && pwd)}"
PYTHON_BIN="${WORKSPACE_ROOT}/.venv/bin/python"

if [[ ! -x "$PYTHON_BIN" ]]; then
  echo "找不到父工作区 Python 环境：$PYTHON_BIN" >&2
  echo "请先确认 /home/zhj/projects/fr5_platform_ws/.venv 存在，或设置 FR5_PLATFORM_WS_ROOT。" >&2
  exit 1
fi

start_unit() {
  local unit="$1"
  shift
  if systemctl --user is-active --quiet "$unit"; then
    return 0
  fi
  systemd-run --user \
    --unit="${unit%.service}" \
    --collect \
    --property=Restart=on-failure \
    "$@" >/dev/null
}

if [[ ! -x "$WORKSPACE_ROOT/scripts/run_d435_camera.sh" ]]; then
  echo "找不到 D435 启动脚本：$WORKSPACE_ROOT/scripts/run_d435_camera.sh" >&2
  exit 1
fi

start_unit d435-camera.service \
  "$WORKSPACE_ROOT/scripts/run_d435_camera.sh"

if ! systemctl --user is-active --quiet fr5-state.service; then
  systemd-run --user \
    --unit=fr5-state \
    --collect \
    --property=Restart=on-failure \
    --setenv=FR5_STATE_SOURCE=real \
    --setenv=PYTHONPATH="$PROJECT_ROOT" \
    --setenv=LD_LIBRARY_PATH="/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}" \
    --working-directory="$PROJECT_ROOT" \
    "$PYTHON_BIN" -m uvicorn state_service.app:app --host 127.0.0.1 --port 8765 >/dev/null
fi

if [[ -f "$WORKSPACE_ROOT/scripts/watch_d435_camera.py" ]] && \
   [[ ! -x "$WORKSPACE_ROOT/.venv/bin/python" ]]; then
  echo "找不到 D435 watchdog Python 环境。" >&2
  exit 1
fi
if [[ -f "$WORKSPACE_ROOT/scripts/watch_d435_camera.py" ]]; then
  start_unit d435-watchdog.service \
    "$PYTHON_BIN" "$WORKSPACE_ROOT/scripts/watch_d435_camera.py"
fi

for _ in {1..30}; do
  if curl --silent --fail --max-time 1 http://127.0.0.1:8765/health >/dev/null; then
    echo "Platform A runtime services are ready."
    exit 0
  fi
  sleep 1
done

echo "状态服务启动超时，请检查：systemctl --user status fr5-state.service" >&2
exit 1
