#!/bin/bash

# ============================================
# 配置
# ============================================
JETSON_IP="192.168.37.107"
JETSON_USER="nvidia"
PROJECT_DIR="$HOME/ros2_vision_project"
REMOTE_DIR="~/ros2_vision_project"

GREEN='\033[0;32m'
NC='\033[0m'

echo -e "${GREEN}==>${NC} Syncing code to Jetson..."

# 同步整个项目
rsync -avz --delete \
    --exclude '.git/' \
    --exclude '__pycache__/' \
    --exclude '*.pyc' \
    --exclude 'build/' \
    --exclude 'install/' \
    --exclude 'log/' \
    --exclude 'data/' \
    --progress \
    $PROJECT_DIR/ \
    $JETSON_USER@$JETSON_IP:$REMOTE_DIR/

echo -e "${GREEN}✅ Sync complete!${NC}"