#!/usr/bin/env bash
set -euo pipefail

PROJECT="/home/zhj/projects/fr5_platform_ws"

say() {
  printf '%s\n' "$*"
}

# Ctrl+C / 终止时自动清理，避免留下半成品 / 残留单元
trap 'say "收到中断信号，正在清理服务…"; stop_all' INT TERM

is_running() {
  systemctl --user is-active --quiet "$1"
}

unit_exists() {
  [[ "$(systemctl --user show --property=LoadState --value "$1" 2>/dev/null || true)" != "not-found" ]]
}

start_unit() {
  local unit="$1"
  shift
  if is_running "$unit"; then
    say "已在运行：$unit"
    return
  fi
  # transient unit 停止后可能仍短暂存在。复用它的启动配置，而不是
  # 再次以相同名称 systemd-run 创建，避免 "Unit ... already exists"。
  if unit_exists "$unit"; then
    systemctl --user reset-failed "$unit" >/dev/null 2>&1 || true
    systemctl --user start "$unit"
    say "已重新启动：$unit"
    return
  fi

  systemd-run --user \
    --unit="${unit%.service}" \
    --collect \
    --property=Restart=on-failure \
    "$@" >/dev/null
  say "已启动：$unit"
}

stop_unit() {
  local unit="$1"
  if systemctl --user status "$unit" >/dev/null 2>&1; then
    systemctl --user stop "$unit" >/dev/null 2>&1 || true
  fi
  systemctl --user reset-failed "$unit" >/dev/null 2>&1 || true
  say "已停止：$unit"
}

wait_for_url() {
  local name="$1"
  local url="$2"
  local attempts="${3:-20}"
  local i
  for ((i = 1; i <= attempts; i++)); do
    if curl --silent --fail --max-time 1 "$url" >/dev/null; then
      say "$name：可以使用"
      return 0
    fi
    sleep 1
  done
  say "$name：启动超时，请运行 bash scripts/platforms.sh status 查看"
  return 1
}

stop_conflicting_reader() {
  local pids
  pids="$(pgrep -f '[p]ython(3)? .*scripts/02_robot_mode_and_enable\.py' || true)"
  if [[ -n "$pids" ]]; then
    say "发现会抢占机械臂连接的旧程序，正在停止：$pids"
    kill $pids
    sleep 1
  fi
}

ensure_usb_in_wsl() {
  local name="$1"
  local hardware_id="$2"
  local line
  local busid

  if lsusb | grep -q "$hardware_id"; then
    say "$name：已连接到WSL"
    return 0
  fi

  if ! command -v usbipd.exe >/dev/null 2>&1; then
    say "$name：WSL中未发现设备，且无法调用Windows的USB连接工具"
    return 1
  fi

  line="$(usbipd.exe list 2>/dev/null | tr -d '\r' | grep "$hardware_id" | head -n 1 || true)"
  busid="${line%% *}"
  if [[ -z "$line" || -z "$busid" ]]; then
    say "$name：Windows也没有发现设备，请检查USB线和供电"
    return 1
  fi

  say "$name：正在重新接入WSL"
  usbipd.exe attach --wsl Ubuntu-22.04-F --busid "$busid" >/dev/null 2>&1 || true
  sleep 2
  if lsusb | grep -q "$hardware_id"; then
    say "$name：已重新接入WSL"
    return 0
  fi

  say "$name：Windows尚未允许共享。请用管理员PowerShell运行："
  say "usbipd bind --busid $busid"
  return 1
}

start_all() {
  cd "$PROJECT"
  stop_conflicting_reader
  ensure_usb_in_wsl "AG95" "0403:6001"
  ensure_usb_in_wsl "D435" "8086:0b07"

  start_unit d435-camera.service \
    "$PROJECT/scripts/run_d435_camera.sh"

  start_unit fr5-state.service \
    "$PROJECT/scripts/run_state_service_real.sh"

  wait_for_url "共同数据服务" "http://127.0.0.1:8765/health" 20

  start_unit d435-watchdog.service \
    "$PROJECT/.venv/bin/python" "$PROJECT/scripts/watch_d435_camera.py"

  start_unit fr5-platform-b.service \
    "$PROJECT/.venv/bin/python" "$PROJECT/platform_b/gateway.py"

  wait_for_url "Web 控制台" "http://127.0.0.1:8080/health" 15

  if is_running fr5-platform-a.service; then
    say "已在运行：ROS2 后端服务（platform-a）"
  elif unit_exists fr5-platform-a.service; then
    systemctl --user reset-failed fr5-platform-a.service >/dev/null 2>&1 || true
    systemctl --user start fr5-platform-a.service
    say "已重新启动：ROS2 后端服务（platform-a）"
  else
    systemd-run --user \
      --unit=fr5-platform-a \
      --collect \
      --property=Restart=no \
      --setenv="DISPLAY=${DISPLAY:-:0}" \
      --setenv="WAYLAND_DISPLAY=${WAYLAND_DISPLAY:-wayland-0}" \
      --setenv="XDG_RUNTIME_DIR=${XDG_RUNTIME_DIR:-/run/user/$(id -u)}" \
      --setenv="LANG=C.UTF-8" \
      --setenv="LC_ALL=C.UTF-8" \
      --setenv="PYTHONUTF8=1" \
      --setenv="LD_LIBRARY_PATH=/opt/ros/humble/lib:${LD_LIBRARY_PATH:-}" \
      --setenv="AMENT_PREFIX_PATH=/opt/ros/humble:${AMENT_PREFIX_PATH:-}" \
      "$PROJECT/.venv/bin/python" "$PROJECT/platform_a/main.py" >/dev/null
    say "已启动：ROS2 后端服务（platform-a）"
  fi

  if command -v powershell.exe >/dev/null 2>&1; then
    powershell.exe -NoProfile -Command \
      "Start-Process 'http://127.0.0.1:8080/'" >/dev/null 2>&1 || true
  fi

  say ""
  say "启动完成：Web 控制台地址为 http://127.0.0.1:8080/"
  say "检查状态：bash scripts/platforms.sh status"
  say "全部停止：bash scripts/platforms.sh stop"
}

stop_all() {
  stop_unit d435-watchdog.service
  stop_unit fr5-platform-a.service
  stop_unit fr5-platform-b.service
  stop_unit fr5-state.service
  stop_unit d435-camera.service
  say "ROS2 后端、Web 控制台、共同数据服务和相机服务均已停止。"
}

show_status() {
  local unit
  for unit in d435-camera.service d435-watchdog.service fr5-state.service fr5-platform-b.service fr5-platform-a.service; do
    if is_running "$unit"; then
      say "$unit：运行中"
    else
      say "$unit：未运行"
    fi
  done

  if curl --silent --fail --max-time 2 "http://127.0.0.1:8765/health" >/dev/null; then
    say ""
    curl --silent --max-time 2 "http://127.0.0.1:8765/health"
    say ""
  fi
  if curl --silent --fail --max-time 2 "http://127.0.0.1:8080/health" >/dev/null; then
    curl --silent --max-time 2 "http://127.0.0.1:8080/health"
    say ""
  fi
}

open_official() {
  stop_all
  if command -v powershell.exe >/dev/null 2>&1; then
    powershell.exe -NoProfile -Command \
      "Start-Process 'http://192.168.58.2/index.html#/'" >/dev/null 2>&1 || true
  fi
  say "已停止A、B相关服务，并打开法奥官方网页。"
}

case "${1:-start}" in
  start)
    start_all
    ;;
  stop)
    stop_all
    ;;
  restart)
    stop_all
    start_all
    ;;
  status)
    show_status
    ;;
  official)
    open_official
    ;;
  *)
    say "用法：bash scripts/platforms.sh start|stop|restart|status|official"
    exit 2
    ;;
esac
