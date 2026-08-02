#!/bin/bash
# sync_dp.sh — decision_processor 使用 egg-link 安装（软链接到 src）
# 修改 src/ 下的 .py 文件后直接重启节点即可生效，无需手动同步。
echo "[INFO] decision_processor 已通过 egg-link 安装，src 修改直接生效。"
echo "       重启节点: python3 scripts/launch_rc2026.py"
