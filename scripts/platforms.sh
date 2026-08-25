#!/usr/bin/env bash
set -euo pipefail

PROJECT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
OTHER_STACK_UNITS=(
  d435-camera.service
  d435-watchdog.service
  fr5-state.service
  fr5-platform-b.service
  fr5-platform-a.service
)
PYTHON="$PROJECT/.venv/bin/python"
CAMERA_UNIT="fr5-release-d435-camera.service"
STATE_UNIT="fr5-release-state.service"
WATCHDOG_UNIT="fr5-release-d435-watchdog.service"
GATEWAY_UNIT="fr5-release-platform-b.service"

say() { printf "%s\n" "$*"; }
trap 'say "收到中断信号，正在清理发布版服务…"; stop_all' INT TERM

is_running() { systemctl --user is-active --quiet "$1"; }
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
  if unit_exists "$unit"; then
    systemctl --user reset-failed "$unit" >/dev/null 2>&1 || true
    systemctl --user start "$unit"
  else
    systemd-run --user --unit="${unit%.service}" --collect \
      --property=Restart=on-failure "$@" >/dev/null
  fi
  say "已启动：$unit"
}

stop_unit() {
  local unit="$1"
  systemctl --user stop "$unit" >/dev/null 2>&1 || true
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

ensure_http_unit() {
  local unit="$1"
  local command="$2"
  local name="$3"
  local url="$4"

  if is_running "$unit" && ! curl --silent --fail --max-time 1 "$url" >/dev/null; then
    say "$name：服务状态存在但端口无响应，正在重启"
    systemctl --user kill "$unit" >/dev/null 2>&1 || true
    systemctl --user stop --no-block "$unit" >/dev/null 2>&1 || true
    sleep 1
  fi

  start_unit "$unit" "$command"
  wait_for_url "$name" "$url" 20
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
  stop_conflicting_stack
  assert_port_available_for "$GATEWAY_UNIT" 8080
  assert_port_available_for "$STATE_UNIT" 8765
  ensure_usb_in_wsl "AG95" "0403:6001"
  ensure_usb_in_wsl "D435" "8086:0b07"
  start_unit "$CAMERA_UNIT" "$PROJECT/scripts/run_d435_camera.sh"
  ensure_http_unit "$STATE_UNIT" "共同数据服务" "http://127.0.0.1:8765/health" "$PROJECT/scripts/run_state_service_real.sh"
  start_unit "$WATCHDOG_UNIT" "$PYTHON" "$PROJECT/scripts/watch_d435_camera.py"
  ensure_http_unit "$GATEWAY_UNIT" "Web 控制台" "http://127.0.0.1:8080/health" "$PYTHON" "$PROJECT/platform_b/gateway.py"
  say ""
  say "发布版启动完成：http://127.0.0.1:8080/"
  say "状态检查：bash scripts/platforms.sh status"
  say "全部停止：bash scripts/platforms.sh stop"
}

stop_all() {
  stop_unit "$WATCHDOG_UNIT"
  stop_unit "$GATEWAY_UNIT"
  stop_unit "$STATE_UNIT"
  stop_unit "$CAMERA_UNIT"
  say "发布版相机、状态服务、视觉网关已停止。"
}

show_status() {
  local unit
  for unit in "$CAMERA_UNIT" "$WATCHDOG_UNIT" "$STATE_UNIT" "$GATEWAY_UNIT"; do
    if is_running "$unit"; then
      say "$unit：运行中"
    else
      say "$unit：未运行"
    fi
  done
  if curl --silent --fail --max-time 2 "http://127.0.0.1:8765/health" >/dev/null; then
    curl --silent --fail --max-time 2 "http://127.0.0.1:8765/health"
    say ""
  fi
  if curl --silent --fail --max-time 2 "http://127.0.0.1:8080/health" >/dev/null; then
    curl --silent --fail --max-time 2 "http://127.0.0.1:8080/health"
    say ""
  fi
}


# 启动前自检与自愈：USB 透传、两套项目端口冲突、以及服务假启动。
wait_for_http_unit() {
  local unit="$1" name="$2" url="$3" attempts="${4:-20}" i
  for ((i = 1; i <= attempts; i++)); do
    if is_running "$unit" && curl --silent --fail --max-time 1 "$url" >/dev/null; then
      say "$name：可以使用"
      return 0
    fi
    sleep 1
  done
  say "$name：未能通过服务运行和 HTTP 响应自检"
  journalctl --user -u "$unit" --no-pager -n 20 2>&1 || true
  return 1
}


ensure_http_unit() {
  local unit="$1" name="$2" url="$3"
  shift 3
  if is_running "$unit" && ! curl --silent --fail --max-time 1 "$url" >/dev/null; then
    say "$name：服务显示运行但健康检查失败，正在停止并重启"
    systemctl --user kill "$unit" >/dev/null 2>&1 || true
    systemctl --user stop --no-block "$unit" >/dev/null 2>&1 || true
    sleep 1
  fi
  start_unit "$unit" "$@"
  wait_for_http_unit "$unit" "$name" "$url" 20
}

stop_conflicting_stack() {
  local unit active=0
  for unit in "${OTHER_STACK_UNITS[@]}"; do
    if is_running "$unit"; then active=1; break; fi
  done
  (( active == 0 )) && return 0
  say "检测到原项目正在占用同一组 USB/端口；本次启动当前项目，正在停止冲突服务"
  for unit in "${OTHER_STACK_UNITS[@]}"; do
    systemctl --user stop "$unit" >/dev/null 2>&1 || true
    systemctl --user reset-failed "$unit" >/dev/null 2>&1 || true
  done
  sleep 1
}

assert_port_available_for() {
  local unit="$1" port="$2" owners
  if is_running "$unit"; then
    return 0
  fi
  owners="$(ss -ltnp "sport = :$port" 2>/dev/null | sed -n '2,$p' || true)"
  if [[ -n "$owners" ]]; then
    say "端口 $port 仍被未知进程占用；为避免误杀进程，停止启动："
    say "$owners"
    return 1
  fi
}

ensure_usb_in_wsl() {
  local name="$1" hardware_id="$2" line output busid
  if lsusb | grep -q "$hardware_id"; then
    say "$name：已连接到WSL"
    return 0
  fi
  if ! command -v usbipd.exe >/dev/null 2>&1; then
    say "$name：WSL 中未发现设备，且找不到 usbipd.exe"
    return 1
  fi
  if ! output="$(usbipd.exe list 2>&1)"; then
    say "$name：WSL 无法执行 usbipd.exe，不能误报为 Windows 未发现设备"
    say "请在 Windows PowerShell 执行：wsl --shutdown；随后重新打开 Ubuntu-22.04-F，再运行本脚本"
    return 1
  fi
  line="$(printf '%s\n' "$output" | tr -d '\r' | grep "$hardware_id" | head -n 1 || true)"
  busid="${line%% *}"
  if [[ -z "$line" || -z "$busid" ]]; then
    say "$name：Windows 未发现设备，请检查 USB 线和供电"
    return 1
  fi
  if [[ "$line" == *"Not shared"* ]]; then
    say "$name：Windows 已发现设备但未共享。请用管理员 PowerShell 执行：usbipd bind --busid $busid"
    return 1
  fi
  say "$name：正在重新接入 WSL"
  usbipd.exe attach --wsl Ubuntu-22.04-F --busid "$busid" >/dev/null 2>&1 || true
  for _ in 1 2 3; do
    sleep 1
    if lsusb | grep -q "$hardware_id"; then
      say "$name：已重新接入 WSL"
      return 0
    fi
  done
  say "$name：Windows 已共享但未能附加到 WSL。请在 Windows PowerShell 执行：usbipd attach --wsl Ubuntu-22.04-F --busid $busid"
  return 1
}

case "${1:-start}" in
  start) start_all ;;
  stop) stop_all ;;
  restart) stop_all; start_all ;;
  status) show_status ;;
  *) say "用法：bash scripts/platforms.sh start|stop|restart|status"; exit 2 ;;
esac

