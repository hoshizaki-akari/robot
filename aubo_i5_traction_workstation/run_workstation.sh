#!/usr/bin/env bash
set -eo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
mode="${1:-}"
restart=false
if [[ -z "$mode" ]]; then
  if [[ "${AUBO_TRACTION_ENABLED:-false}" == true ]]; then mode=--live; else mode=--web-only; fi
fi
if [[ "$mode" == --restart-live ]]; then
  restart=true
  mode=--live
fi
if [[ "$mode" == --check-config ]]; then
  launcher="$(wslpath -w "$PROJECT_DIR/scripts/start_live_workstation.ps1")"
  powershell="$(command -v powershell.exe || true)"
  powershell="${powershell:-/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe}"
  exec "$powershell" -NoProfile -ExecutionPolicy Bypass -File "$launcher" -CheckLocalConfig
fi
if [[ "$mode" == --live ]]; then
  launcher="$(wslpath -w "$PROJECT_DIR/scripts/start_live_workstation.ps1")"
  powershell="$(command -v powershell.exe || true)"
  powershell="${powershell:-/mnt/c/Windows/System32/WindowsPowerShell/v1.0/powershell.exe}"
  arguments=(-NoProfile -ExecutionPolicy Bypass -File "$launcher")
  if $restart; then
    # Validate before stopping a working instance. The second invocation uses
    # the same project-local reference and Windows credential file.
    if config_report="$("$powershell" "${arguments[@]}" -CheckLocalConfig)"; then
      echo 'Local connection and calibration configuration OK.'
    else
      printf '%s\n' "$config_report" >&2
      exit 2
    fi
    bash "$PROJECT_DIR/scripts/stop_live_workstation.sh"
    arguments+=(-WaitForPreviousShutdown)
  fi
  exec "$powershell" "${arguments[@]}"
fi
if [[ "$mode" != --web-only ]]; then
  echo 'Usage: bash run_workstation.sh [--live|--restart-live|--web-only|--check-config]' >&2
  exit 2
fi
ROS_WS="$PROJECT_DIR/ros2_overlay"
if [[ ! -f "$ROS_WS/install/setup.bash" ]]; then
  echo "AUBO ROS2 workspace is not built: $ROS_WS" >&2
  exit 2
fi
source /opt/ros/humble/setup.bash
source "$ROS_WS/install/setup.bash"
VENV_DIR="${AUBO_VENV_DIR:-$PROJECT_DIR/.venv}"
if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
  echo "AUBO workstation Python environment is missing: $VENV_DIR" >&2
  echo "Create it with: bash scripts/tablet_setup_env.sh" >&2
  exit 2
fi
source "$VENV_DIR/bin/activate"
set -u
export ROS_DOMAIN_ID="${AUBO_ROS_DOMAIN_ID:-42}"
export RMW_IMPLEMENTATION="${AUBO_RMW_IMPLEMENTATION:-rmw_cyclonedds_cpp}"
export ROS_LOCALHOST_ONLY="${AUBO_ROS_LOCALHOST_ONLY:-1}"
WORKSTATION_PORT="${WORKSTATION_PORT:-8082}"
cd "$PROJECT_DIR"
echo "AUBO web component: http://127.0.0.1:$WORKSTATION_PORT/"
exec python -m uvicorn app:app --host 127.0.0.1 --port "$WORKSTATION_PORT"
