#!/bin/bash
# build_sim.sh — 编译 ros2_ws + simulation_ws（开发机用）
# simulation_ws overlay ros2_ws

set -e

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
ROBOT_WS="$SCRIPT_DIR/../ros2_ws"
SIM_WS="$SCRIPT_DIR/../simulation_ws"

echo "========================================"
echo "  RC2026 Full Build (robot + simulation)"
echo "========================================"

# 1. 编译上位机工作空间
echo "[1/2] Building ros2_ws ..."
cd "$ROBOT_WS"
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release
source "$ROBOT_WS/install/setup.bash"

# 2. 编译仿真工作空间 (overlay)
if [ -d "$SIM_WS/src" ]; then
  echo "[2/2] Building simulation_ws (overlay) ..."
  cd "$SIM_WS"
  colcon build --symlink-install \
    --cmake-args -DCMAKE_BUILD_TYPE=Release
  echo ""
  echo "Build complete. Source with:"
  echo "  source $ROBOT_WS/install/setup.bash"
  echo "  source $SIM_WS/install/setup.bash"
else
  echo "[2/2] simulation_ws/src not found, skipping."
  echo ""
  echo "Build complete. Source with:"
  echo "  source $ROBOT_WS/install/setup.bash"
fi
