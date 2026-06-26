#!/bin/bash
# sync_dp.sh — 将 decision_processor 源码同步到 install 目录
#
# 用法:
#   bash scripts/sync_dp.sh              # 同步全部 .py 文件
#   bash scripts/sync_dp.sh config.py    # 只同步指定文件
#   bash scripts/sync_dp.sh config.py game_controller.py  # 同步多个文件

SRC="$(cd "$(dirname "$0")/.." && pwd)/ros2_ws/src/decision_processor/decision_processor"
DST="$(cd "$(dirname "$0")/.." && pwd)/ros2_ws/install/decision_processor/lib/python3.10/site-packages/decision_processor"

if [ ! -d "$SRC" ]; then
    echo "[ERROR] 源目录不存在: $SRC"
    exit 1
fi
if [ ! -d "$DST" ]; then
    echo "[ERROR] install目录不存在: $DST"
    echo "        请先执行 colcon build 生成 install 目录"
    exit 1
fi

if [ $# -eq 0 ]; then
    # 无参数: 同步全部 .py 文件
    count=0
    for f in "$SRC"/*.py; do
        fname="$(basename "$f")"
        cp "$f" "$DST/$fname"
        echo "[OK] $fname"
        count=$((count + 1))
    done
    echo "--- 共同步 $count 个文件 ---"
else
    # 有参数: 只同步指定文件
    for fname in "$@"; do
        src_file="$SRC/$fname"
        dst_file="$DST/$fname"
        if [ ! -f "$src_file" ]; then
            echo "[SKIP] 文件不存在: $fname"
            continue
        fi
        cp "$src_file" "$dst_file"
        echo "[OK] $fname"
    done
fi
