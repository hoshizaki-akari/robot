#!/usr/bin/env bash
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
PID_FILE="$PROJECT_DIR/debug/live_workstation.pid"
if [[ ! -f "$PID_FILE" ]]; then
  echo 'No managed live-workstation PID file; no process was terminated.'
  exit 0
fi
read -r runtime_pid < "$PID_FILE"
if [[ ! "$runtime_pid" =~ ^[1-9][0-9]*$ ]]; then echo 'Invalid supervisor PID file.' >&2; exit 2; fi
if [[ ! -d "/proc/$runtime_pid" ]]; then exit 0; fi
runtime_command="$(tr '\0' ' ' < "/proc/$runtime_pid/cmdline")"
if [[ "$runtime_command" != *"$PROJECT_DIR/scripts/run_live_workstation.sh "* ]]; then
  echo 'PID no longer belongs to this workstation; no process was terminated.' >&2
  exit 2
fi
kill -TERM "$runtime_pid"
flock -w 15 "$PROJECT_DIR/debug/live_workstation.lock" true
echo 'Live workstation stopped.'
