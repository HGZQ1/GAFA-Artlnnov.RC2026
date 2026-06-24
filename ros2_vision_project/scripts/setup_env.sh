#!/bin/bash
# 一键 source 所有 ROS2 工作空间（按依赖顺序）
# 用法: source scripts/setup_env.sh

source /opt/ros/humble/setup.bash
source /home/hgzq/ros2_external_ws/install/setup.bash
source /home/hgzq/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws/install/setup.bash

export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
echo "[RC2026] 环境已加载: ROS2 Humble + livox_ros_driver2 + fast_lio + 项目包"
