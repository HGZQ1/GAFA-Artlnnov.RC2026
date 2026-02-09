#!/bin/bash

set -e

# ============================================
# 配置
# ============================================
JETSON_IP="192.168.37.107"
JETSON_USER="nvidia"
PROJECT_NAME="ros2-vision"
CONTAINER_NAME="${PROJECT_NAME}-container"

GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

# ============================================
# 1. 同步代码
# ============================================
echo -e "${GREEN}==>${NC} Step 1/4: Syncing code..."
./scripts/sync_to_jetson.sh

# ============================================
# 2. 构建镜像
# ============================================
echo -e "${GREEN}==>${NC} Step 2/4: Building Docker image on Jetson..."
ssh $JETSON_USER@$JETSON_IP << 'ENDSSH'
cd ~/ros2_vision_project
docker build -f docker/Dockerfile.jetson -t ros2-vision-jetson:latest .
ENDSSH

# ============================================
# 3. 停止旧容器
# ============================================
echo -e "${GREEN}==>${NC} Step 3/4: Stopping old container..."
ssh $JETSON_USER@$JETSON_IP << ENDSSH
docker stop $CONTAINER_NAME 2>/dev/null || true
docker rm $CONTAINER_NAME 2>/dev/null || true
ENDSSH

# ============================================
# 4. 启动新容器
# ============================================
echo -e "${GREEN}==>${NC} Step 4/4: Starting new container..."
ssh $JETSON_USER@$JETSON_IP << ENDSSH
docker run -d \
    --name $CONTAINER_NAME \
    --runtime nvidia \
    --network host \
    --privileged \
    --restart unless-stopped \
    -v /dev:/dev \
    -v ~/ros2_vision_project/models:/ros2_ws/models \
    -v ~/ros2_vision_project/data:/ros2_ws/data \
    -e DISPLAY=:0 \
    ros2-vision-jetson:latest
ENDSSH

echo -e "${GREEN}✅ Deployment complete!${NC}"
echo -e "${YELLOW}Check logs:${NC} ssh $JETSON_USER@$JETSON_IP 'docker logs -f $CONTAINER_NAME'"