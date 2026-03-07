# Docker配置完整对比指南

## 🎯 快速决策树

```
您的项目需要什么？
│
├─ 只需要目标检测，不需要ROS2
│  │
│  ├─ x86_64 PC
│  │  ├─ 有NVIDIA GPU → 使用 Dockerfile.gpu
│  │  └─ 无GPU → 使用 Dockerfile
│  │
│  └─ Jetson Orin Nano → 使用 Dockerfile.jetson
│
└─ 需要ROS2集成（机器人系统）
   │
   ├─ x86_64 PC
   │  ├─ 有NVIDIA GPU → 使用 Dockerfile.ros2.gpu
   │  └─ 无GPU → 使用 Dockerfile.ros2
   │
   └─ Jetson Orin Nano → 使用 Dockerfile.ros2.jetson
```

---

## 📦 完整文件清单（24个文件）

### 共用文件（5个）

| 文件 | 说明 | 所有版本都需要 |
|-----|------|------------|
| `.dockerignore` | Docker构建忽略 | ✅ |
| `.env.example` | 环境变量模板 | ✅ |
| `requirements.txt` | Python依赖 | ✅ |
| `test_docker_env.py` | 环境测试脚本 | ✅ |
| `PLATFORM_GUIDE.md` | 架构选择指南 | ✅ |

### Standalone版本（9个文件）

**用途**: 纯检测系统，不需要ROS2

#### x86_64 (6个)
| 文件 | 平台 | 说明 |
|-----|------|------|
| `Dockerfile` | x86_64 CPU | 无GPU版本 |
| `Dockerfile.gpu` | x86_64 GPU | NVIDIA GPU版本 |
| `docker-compose.yml` | x86_64 | 编排配置 |
| `run_docker.sh` | x86_64 | 启动脚本 |
| `Makefile` | x86_64 | Make命令 |
| `README_DOCKER.md` | x86_64 | 详细文档 |
| `DOCKER_README.md` | x86_64 | 快速指南 |

#### Jetson (3个)
| 文件 | 平台 | 说明 |
|-----|------|------|
| `Dockerfile.jetson` | ARM64 | Jetson专用 |
| `docker-compose.jetson.yml` | ARM64 | Jetson编排 |
| `run_docker_jetson.sh` | ARM64 | Jetson脚本 |
| `README_JETSON.md` | ARM64 | Jetson文档 |

### ROS2版本（7个文件）

**用途**: ROS2机器人系统集成

| 文件 | 平台 | 说明 |
|-----|------|------|
| `Dockerfile.ros2` | x86_64 CPU | ROS2 CPU版本 |
| `Dockerfile.ros2.gpu` | x86_64 GPU | ROS2 GPU版本 |
| `Dockerfile.ros2.jetson` | ARM64 | ROS2 Jetson版本 |
| `docker-compose.ros2.yml` | 全平台 | ROS2编排配置 |
| `run_docker_ros2.sh` | 全平台 | ROS2启动脚本 |
| `README_ROS2_DOCKER.md` | 全平台 | ROS2 Docker文档 |

### ROS2集成文档（1个）

| 文件 | 说明 |
|-----|------|
| `ROS2_YOLOV8_D435i_Integration_Guide.md` | ROS2包创建完整指南 |

---

## 📊 配置对比表

### 特性对比

| 特性 | Standalone | ROS2 |
|-----|-----------|------|
| **镜像大小** | 5-8GB | 8-11GB |
| **内存使用** | ~1.5GB | ~2.5GB |
| **启动时间** | ~7秒 | ~10秒 |
| **ROS2支持** | ❌ | ✅ |
| **话题通信** | ❌ | ✅ |
| **RViz2可视化** | ❌ | ✅ |
| **TF坐标变换** | ❌ | ✅ |
| **适用场景** | 单机检测 | 机器人系统 |
| **学习曲线** | 简单 | 需要ROS2知识 |

### 基础镜像对比

| 版本 | 基础镜像 | 说明 |
|-----|---------|------|
| **Standalone x86 CPU** | `ubuntu:22.04` | 标准Ubuntu |
| **Standalone x86 GPU** | `nvidia/cuda:11.8.0-cudnn8-devel-ubuntu22.04` | NVIDIA CUDA |
| **Standalone Jetson** | `nvcr.io/nvidia/l4t-pytorch:r35.2.1-pth2.0-py3` | L4T + PyTorch |
| **ROS2 x86 CPU** | `osrf/ros:humble-desktop-full` | ROS2官方镜像 |
| **ROS2 x86 GPU** | `nvidia/cuda:11.8.0` + ROS2安装 | CUDA + 手动ROS2 |
| **ROS2 Jetson** | `dustynv/ros:humble-pytorch-l4t` | L4T + ROS2 + PyTorch |

---

## 🔧 使用场景详解

### Scenario 1: 简单演示/测试

**需求**: 快速测试YOLOv8检测功能

**推荐**: Standalone版本
```bash
./run_docker.sh build cpu
./run_docker.sh start cpu
./run_docker.sh exec cpu
python3 main_debug.py
```

**优点**:
- 最快部署
- 最小资源占用
- 最简单操作

---

### Scenario 2: 高性能检测（x86 PC + GPU）

**需求**: 实时检测，需要GPU加速

**推荐**: Standalone GPU版本
```bash
./run_docker.sh build gpu
./run_docker.sh start gpu
./run_docker.sh exec gpu
python3 main_debug.py
```

**性能**: 80-100 FPS @ 640x480

---

### Scenario 3: 边缘部署（Jetson Orin Nano）

**需求**: 嵌入式设备，功耗受限

**推荐**: Standalone Jetson版本
```bash
./run_docker_jetson.sh build
./run_docker_jetson.sh perf  # 设置性能模式
./run_docker_jetson.sh start
./run_docker_jetson.sh exec
python3 main_debug.py
```

**性能**: 15-60 FPS @ 640x480（TensorRT优化）

---

### Scenario 4: 机器人视觉系统（x86 + ROS2）

**需求**: 
- 检测结果发布到ROS2话题
- 与导航系统集成
- RViz2可视化

**推荐**: ROS2 GPU版本
```bash
./run_docker_ros2.sh build gpu
./run_docker_ros2.sh start gpu
./run_docker_ros2.sh exec gpu

# 在容器内
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch vision_detector detector.launch.py
```

**功能**:
- 发布`/detections`话题
- 发布`/detected_objects_3d`话题
- 发布TF变换
- RViz2实时显示

---

### Scenario 5: 移动机器人（Jetson + ROS2）

**需求**:
- Jetson Orin Nano主控
- ROS2导航系统
- 功耗和性能平衡

**推荐**: ROS2 Jetson版本
```bash
./run_docker_ros2.sh build jetson
./run_docker_ros2.sh start jetson
./run_docker_ros2.sh exec jetson

# 在容器内
cd ~/ros2_ws
colcon build
source install/setup.bash
ros2 launch vision_detector detector.launch.py
```

**优化**:
- MAXN性能模式
- TensorRT加速
- 与Nav2集成

---

## 📝 配置文件使用矩阵

### x86_64 Ubuntu 22.04

| 需求 | Dockerfile | Docker Compose | 启动脚本 | 文档 |
|-----|-----------|---------------|---------|------|
| **CPU检测** | Dockerfile | docker-compose.yml | run_docker.sh | README_DOCKER.md |
| **GPU检测** | Dockerfile.gpu | docker-compose.yml | run_docker.sh | README_DOCKER.md |
| **ROS2+CPU** | Dockerfile.ros2 | docker-compose.ros2.yml | run_docker_ros2.sh | README_ROS2_DOCKER.md |
| **ROS2+GPU** | Dockerfile.ros2.gpu | docker-compose.ros2.yml | run_docker_ros2.sh | README_ROS2_DOCKER.md |

### Jetson Orin Nano

| 需求 | Dockerfile | Docker Compose | 启动脚本 | 文档 |
|-----|-----------|---------------|---------|------|
| **Standalone** | Dockerfile.jetson | docker-compose.jetson.yml | run_docker_jetson.sh | README_JETSON.md |
| **ROS2集成** | Dockerfile.ros2.jetson | docker-compose.ros2.yml | run_docker_ros2.sh | README_ROS2_DOCKER.md |

---

## 🚀 快速命令参考

### Standalone版本

```bash
# x86 CPU
./run_docker.sh build cpu && ./run_docker.sh start cpu

# x86 GPU
./run_docker.sh build gpu && ./run_docker.sh start gpu

# Jetson
./run_docker_jetson.sh build && ./run_docker_jetson.sh start
```

### ROS2版本

```bash
# x86 CPU
./run_docker_ros2.sh build cpu && ./run_docker_ros2.sh start cpu

# x86 GPU
./run_docker_ros2.sh build gpu && ./run_docker_ros2.sh start gpu

# Jetson
./run_docker_ros2.sh build jetson && ./run_docker_ros2.sh start jetson
```

---

## ⚠️ 常见错误

### 错误1: 混用不同版本的文件

**症状**: 构建或运行失败

**解决**: 确保使用完整的一套配置

✅ **正确组合**:
- Standalone x86: Dockerfile + docker-compose.yml + run_docker.sh
- ROS2 x86: Dockerfile.ros2.gpu + docker-compose.ros2.yml + run_docker_ros2.sh
- Jetson: Dockerfile.jetson + docker-compose.jetson.yml + run_docker_jetson.sh

❌ **错误组合**:
- Dockerfile + docker-compose.ros2.yml
- Dockerfile.jetson + docker-compose.yml
- Dockerfile.ros2 + run_docker.sh

---

### 错误2: 在错误平台上构建

**症状**: 
```
exec format error
manifest not found
```

**解决**: 
- x86镜像只能在x86系统上构建
- Jetson镜像只能在Jetson上构建

---

### 错误3: ROS2容器中缺少工作空间

**症状**: 找不到ROS2包

**解决**: ROS2版本需要手动创建工作空间

```bash
# 在ROS2容器内
cd ~/ros2_ws/src
# 按照ROS2_YOLOV8_D435i_Integration_Guide.md创建包
```

---

## 📈 性能基准对比

### 推理速度（YOLOv8n @ 640x480）

| 配置 | FPS | 延迟 | 功耗 |
|-----|-----|------|------|
| **x86 CPU** | 6-8 | 140ms | ~65W |
| **x86 GPU (RTX 3060)** | 80-100 | 11ms | ~180W |
| **x86 GPU + TensorRT** | 150-200 | 6ms | ~200W |
| **Jetson PyTorch** | 15-20 | 55ms | 15-25W |
| **Jetson TensorRT** | 40-60 | 18ms | 20-25W |

### 资源占用

| 配置 | 镜像大小 | 内存使用 | 存储需求 |
|-----|---------|---------|---------|
| **Standalone x86 CPU** | ~5GB | ~1.5GB | 15GB |
| **Standalone x86 GPU** | ~7GB | ~2GB | 20GB |
| **Standalone Jetson** | ~8GB | ~3GB | 25GB |
| **ROS2 x86 CPU** | ~8GB | ~2.5GB | 25GB |
| **ROS2 x86 GPU** | ~10GB | ~3GB | 30GB |
| **ROS2 Jetson** | ~11GB | ~3.5GB | 35GB |

---

## 📚 文档索引

### 按平台

**x86_64**:
- Standalone: README_DOCKER.md, DOCKER_README.md
- ROS2: README_ROS2_DOCKER.md

**Jetson**:
- Standalone: README_JETSON.md
- ROS2: README_ROS2_DOCKER.md

### 按功能

**快速开始**: DOCKER_README.md  
**详细部署**: README_DOCKER.md  
**Jetson专用**: README_JETSON.md  
**ROS2集成**: README_ROS2_DOCKER.md  
**ROS2开发**: ROS2_YOLOV8_D435i_Integration_Guide.md  
**架构选择**: PLATFORM_GUIDE.md  
**完整对比**: 本文档  

---

## 💡 推荐实践

### 开发到部署流程

```
1. 开发阶段 → Standalone x86 GPU版本
   ├─ 快速迭代
   ├─ 算法调试
   └─ 性能测试

2. 集成阶段 → ROS2 x86 GPU版本
   ├─ ROS2包开发
   ├─ 话题测试
   └─ 系统集成

3. 部署阶段
   ├─ 实验室/云端 → ROS2 x86 GPU版本
   └─ 边缘/移动 → ROS2 Jetson版本
```

---

## 🎓 学习路径

### 新手路径

```
1. 阅读 DOCKER_README.md（5分钟）
2. 运行 Standalone CPU版本（15分钟）
3. 测试检测功能（10分钟）
4. 如需GPU，尝试GPU版本
5. 如需ROS2，学习ROS2集成文档
```

### 进阶路径

```
1. 掌握Standalone版本
2. 学习ROS2基础
3. 阅读 ROS2_YOLOV8_D435i_Integration_Guide.md
4. 部署ROS2版本
5. 优化TensorRT
6. 部署到Jetson
```

---

## 📞 获取帮助

### 命令行帮助

```bash
# Standalone
./run_docker.sh help
make help

# Standalone Jetson
./run_docker_jetson.sh help

# ROS2
./run_docker_ros2.sh help
```

### 环境测试

```bash
# 所有版本都可以运行
python3 /workspace/test_docker_env.py
```

---

**文档版本**: v2.0  
**包含配置**: Standalone + ROS2 (x86_64 + Jetson)  
**总文件数**: 24个  
**最后更新**: 2026年2月
