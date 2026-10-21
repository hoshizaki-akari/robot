#!/usr/bin/env bash
# Full live workstation, including restart; no shell environment setup required.
set -eo pipefail
PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec bash "$PROJECT_DIR/run_workstation.sh" --restart-live "$@"
