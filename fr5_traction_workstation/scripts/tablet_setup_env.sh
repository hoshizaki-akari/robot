#!/usr/bin/env bash
# Native Ubuntu 22.04 deployment setup for the FR5 traction workstation.
# Run once after cloning the repository; the script is safe to run again.
set -euo pipefail

PROJECT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
WORKSPACE_ROOT="$(cd "$PROJECT_DIR/.." && pwd)"
ROS_WS="$PROJECT_DIR/ros2_overlay"
VENV_DIR="$WORKSPACE_ROOT/.venv"
SDK_ROOT="$PROJECT_DIR/vendor/fairino-python-sdk"
DESCRIPTION_ROOT="$ROS_WS/src/fairino_description"
DESKTOP_DIR="$(xdg-user-dir DESKTOP 2>/dev/null || true)"
if [[ -z "$DESKTOP_DIR" || "$DESKTOP_DIR" == "$HOME" ]]; then
  DESKTOP_DIR="$HOME/Desktop"
fi
DESKTOP_FILE="$DESKTOP_DIR/骨伤牵引机器人工作站.desktop"
# The target machine is deployed on a mainland-China network. Prefix public
# GitHub URLs with a streaming mirror by default; set GITHUB_MIRROR_PREFIX to
# an empty string before running this script when direct GitHub is available.
GITHUB_MIRROR_PREFIX="${GITHUB_MIRROR_PREFIX-https://gh-proxy.com/}"
if [[ -n "$GITHUB_MIRROR_PREFIX" ]]; then
  OFFICIAL_ROS2_REPO="${GITHUB_MIRROR_PREFIX}https://github.com/FAIR-INNOVATION/frcobot_ros2.git"
  OFFICIAL_SDK_REPO="${GITHUB_MIRROR_PREFIX}https://github.com/FAIR-INNOVATION/fairino-python-sdk.git"
else
  # An empty mirror prefix explicitly selects the user's authenticated SSH
  # connection (normally github.com -> ssh.github.com:443 in ~/.ssh/config).
  OFFICIAL_ROS2_REPO="git@github.com:FAIR-INNOVATION/frcobot_ros2.git"
  OFFICIAL_SDK_REPO="git@github.com:FAIR-INNOVATION/fairino-python-sdk.git"
fi

if [[ ! -r /etc/os-release ]]; then
  echo "无法识别操作系统。" >&2
  exit 1
fi
# shellcheck disable=SC1091
source /etc/os-release
if [[ "${VERSION_ID:-}" != "22.04" ]]; then
  echo "当前系统是 ${PRETTY_NAME:-未知}，本项目要求 Ubuntu 22.04。" >&2
  exit 1
fi
if [[ "$(uname -m)" != "x86_64" ]]; then
  echo "当前架构为 $(uname -m)，本项目要求 x86_64。" >&2
  exit 1
fi
if [[ ! -f /opt/ros/humble/setup.bash ]]; then
  echo "没有找到 ROS 2 Humble，请先完成 Humble 安装。" >&2
  exit 1
fi

echo "[1/6] 安装编译和桌面启动所需工具"
sudo apt-get update
sudo apt-get install -y \
  build-essential curl git psmisc python3-pip python3-rosdep python3-venv \
  xdg-user-dirs \
  python3-colcon-common-extensions zenity \
  ros-humble-ament-cmake ros-humble-ament-index-python \
  ros-humble-controller-manager-msgs ros-humble-geometry-msgs \
  ros-humble-launch ros-humble-launch-ros ros-humble-rclcpp \
  ros-humble-robot-state-publisher ros-humble-rosidl-default-generators \
  ros-humble-rosidl-default-runtime ros-humble-sensor-msgs \
  ros-humble-std-msgs ros-humble-std-srvs ros-humble-urdf ros-humble-xacro

echo "[2/6] 获取法奥官方机器人描述（稀疏下载）"
if [[ ! -f "$DESCRIPTION_ROOT/package.xml" ]]; then
  rm -rf "$DESCRIPTION_ROOT"
  git clone --depth 1 --filter=blob:none --sparse \
    "$OFFICIAL_ROS2_REPO" "$DESCRIPTION_ROOT"
  git -C "$DESCRIPTION_ROOT" sparse-checkout set --no-cone \
    /fairino_description/CMakeLists.txt \
    /fairino_description/package.xml \
    /fairino_description/urdf/fairino5_v6.urdf \
    '/fairino_description/meshes/fairino5_v6/*'
  mv "$DESCRIPTION_ROOT/fairino_description" "$ROS_WS/src/.fairino_description.tmp"
  rm -rf "$DESCRIPTION_ROOT"
  mv "$ROS_WS/src/.fairino_description.tmp" "$DESCRIPTION_ROOT"
fi

echo "[3/6] 获取法奥官方 Python SDK（稀疏下载）"
if [[ ! -f "$SDK_ROOT/linux/fairino/Robot.py" ]]; then
  rm -rf "$SDK_ROOT"
  git clone --depth 1 --filter=blob:none --sparse \
    "$OFFICIAL_SDK_REPO" "$SDK_ROOT"
  git -C "$SDK_ROOT" sparse-checkout set --no-cone \
    /linux/fairino/Robot.py /LICENSE /README.md
fi

echo "[4/6] 创建网页服务 Python 环境"
if [[ ! -f "$VENV_DIR/bin/activate" ]]; then
  python3 -m venv --system-site-packages "$VENV_DIR"
fi
# shellcheck disable=SC1091
source "$VENV_DIR/bin/activate"
python -m pip install --upgrade pip
python -m pip install -r "$PROJECT_DIR/requirements.txt"

echo "[5/6] 编译 ROS 2 工作区"
# shellcheck disable=SC1091
# Humble's generated setup reads AMENT_TRACE_SETUP_FILES before defining it.
# Temporarily disable nounset exactly as the runtime launchers do.
set +u
source /opt/ros/humble/setup.bash
set -u
if [[ ! -e /etc/ros/rosdep/sources.list.d/20-default.list ]]; then
  sudo rosdep init
fi
rosdep update
rosdep install --from-paths \
  "$ROS_WS/src/fairino_msgs" "$ROS_WS/src/fr_traction" \
  --ignore-src --rosdistro humble -r -y
(
  cd "$ROS_WS"
  colcon build --symlink-install --parallel-workers 2 \
    --packages-select fairino_description fairino_msgs \
      fairino5_v6_moveit2_config fr_traction \
    --cmake-args -DBUILD_TESTING=OFF
)

echo "[6/6] 创建桌面双击启动图标"
mkdir -p "$DESKTOP_DIR"
cat > "$DESKTOP_FILE" <<EOF
[Desktop Entry]
Type=Application
Name=骨伤牵引机器人工作站
Comment=启动 FR5 控制、牵引服务和触摸屏操作页面
Exec=$PROJECT_DIR/scripts/tablet_start.sh
Icon=applications-engineering
Terminal=false
Categories=Utility;
StartupNotify=true
EOF
chmod +x "$DESKTOP_FILE" "$PROJECT_DIR/scripts/tablet_start.sh"
gio set "$DESKTOP_FILE" metadata::trusted true >/dev/null 2>&1 || true

echo
echo "部署准备完成。"
echo "请先按部署指南配置机器人网口，然后双击桌面的‘骨伤牵引机器人工作站’。"
