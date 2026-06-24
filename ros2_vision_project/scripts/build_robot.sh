#!/bin/bash
# build_robot.sh — 只编译 ros2_ws（部署到 Jetson 用）
# 排除仿真相关包

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WS_DIR="$SCRIPT_DIR/../ros2_ws"

echo "========================================"
echo "  RC2026 Robot Build (ros2_ws only)"
echo "========================================"

cd "$WS_DIR"

colcon build --symlink-install \
  --packages-ignore rc2026_simulation \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

echo ""
echo "Build complete. Source with:"
echo "  source $WS_DIR/install/setup.bash"
