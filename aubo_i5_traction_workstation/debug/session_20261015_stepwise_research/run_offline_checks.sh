#!/usr/bin/env bash
set -euo pipefail
cd "$(dirname "${BASH_SOURCE[0]}")"
sdk_dir="${AUBO_V4_SDK_ROOT:-/mnt/c/Users/zhj/AppData/Local/Temp/aubo-legacy-v4-audit}"
protobuf_dir="${AUBO_PROTOBUF9_DIR:-/mnt/c/Users/zhj/AppData/Local/Temp/aubo-v4-runtime-audit/extracted/usr/lib/x86_64-linux-gnu}"
export LD_LIBRARY_PATH="$protobuf_dir:$sdk_dir/lib/lib64/aubocontroller:$sdk_dir/lib/lib64/config:$sdk_dir/lib/lib64/log4cplus${LD_LIBRARY_PATH:+:$LD_LIBRARY_PATH}"
g++ -std=c++17 -O0 -Wall -Wextra sdk_offline_contract.cpp \
  -I"$sdk_dir/include" -L"$sdk_dir/lib/lib64/aubocontroller" \
  -lauborobotcontroller -Wl,-rpath-link,"$protobuf_dir" \
  -Wl,-rpath-link,"$sdk_dir/lib/lib64/config" \
  -Wl,-rpath-link,"$sdk_dir/lib/lib64/log4cplus" -o sdk_offline_contract
./sdk_offline_contract
loader_report="$(ldd -r ./sdk_offline_contract 2>&1)"
if [[ "$loader_report" == *"not found"* || "$loader_report" == *"undefined symbol"* ]]; then
  printf '%s\n' "$loader_report"
  exit 1
fi
echo "dynamic_dependencies=RESOLVED"
python3 test_stepwise_contract.py
echo "scope=OFFLINE_API_AND_DESIGN_CONTRACT_ONLY; live_force_control=NOT_TESTED"
