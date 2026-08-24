#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/humble/setup.bash
source /home/zhj/projects/fr5_learning/robot_ws_backup/ros2_realsense/install/setup.bash

exec ros2 launch realsense2_camera rs_launch.py \
  enable_color:=true \
  enable_depth:=true \
  enable_infra1:=false \
  enable_infra2:=false \
  align_depth.enable:=true \
  rotation_filter.enable:=true \
  rotation_filter.rotation:=180.0 \
  rgb_camera.color_profile:=424x240x15 \
  depth_module.depth_profile:=480x270x6
