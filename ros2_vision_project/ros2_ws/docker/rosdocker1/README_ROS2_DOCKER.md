# YOLOv8-D435i + ROS2 Docker部署指南

## 📋 目录

1. [为什么需要ROS2版本](#为什么需要ros2版本)
2. [与标准版本的区别](#与标准版本的区别)
3. [系统要求](#系统要求)
4. [快速开始](#快速开始)
5. [ROS2工作空间配置](#ros2工作空间配置)
6. [使用方法](#使用方法)
7. [常见问题](#常见问题)

---

## 为什么需要ROS2版本

### Standalone版本 vs ROS2版本

| 特性 | Standalone版本 | ROS2版本 |
|-----|---------------|---------|
| **部署场景** | 单机检测系统 | 机器人/多节点系统 |
| **通信方式** | 无 | ROS2话题/服务 |
| **集成难度** | 简单 | 需要ROS2知识 |
| **镜像大小** | ~5GB | ~8GB |
| **适用项目** | 演示、测试 | 生产机器人系统 |

### 选择ROS2版本的场景

✅ **需要ROS2集成**: 项目已使用ROS2框架  
✅ **多节点通信**: 检测结果需要发布给其他节点  
✅ **机器人系统**: 导航、控制、感知需要集成  
✅ **TF坐标变换**: 需要发布目标的3D坐标  
✅ **RViz2可视化**: 需要在RViz2中查看检测结果  

---

## 与标准版本的区别

### 文件对比

| 用途 | Standalone版本 | ROS2版本 |
|-----|---------------|---------|
| CPU Dockerfile | `Dockerfile` | `Dockerfile.ros2` |
| GPU Dockerfile | `Dockerfile.gpu` | `Dockerfile.ros2.gpu` |
| Jetson Dockerfile | `Dockerfile.jetson` | `Dockerfile.ros2.jetson` |
| Docker Compose | `docker-compose.yml` | `docker-compose.ros2.yml` |
| 启动脚本 | `run_docker.sh` | `run_docker_ros2.sh` |

### 额外功能

ROS2版本包含：
- ✅ ROS2 Humble完整安装
- ✅ vision_msgs和cv_bridge
- ✅ RealSense ROS2驱动
- ✅ RViz2可视化工具
- ✅ TF2坐标变换支持
- ✅ 预配置的ROS2工作空间

---

## 系统要求

### 硬件要求

同Standalone版本，但建议：
- **内存**: 16GB+（ROS2需要更多内存）
- **存储**: 30GB+（ROS2镜像更大）

### 软件要求

| 组件 | 版本 |
|-----|------|
| Ubuntu | 22.04 LTS |
| Docker | ≥ 20.10 |
| Docker Compose | ≥ 1.29 |
| NVIDIA Driver | ≥ 520（GPU版本） |
| JetPack | 5.1.2+（Jetson版本） |

---

## 快速开始

### x86_64系统（CPU版本）

```bash
# 1. 进入项目目录
cd /path/to/yolov5-D435i-main

# 2. 给脚本执行权限
chmod +x run_docker_ros2.sh

# 3. 构建镜像（首次需要20-30分钟）
./run_docker_ros2.sh build cpu

# 4. 启动容器
./run_docker_ros2.sh start cpu

# 5. 进入容器
./run_docker_ros2.sh exec cpu

# 6. 在容器内：创建ROS2包（参考集成文档）
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python vision_detector
# ... 复制代码文件 ...

# 7. 构建工作空间
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash

# 8. 运行ROS2节点
ros2 run vision_detector detector_node
```

### x86_64系统（GPU版本）

```bash
# 构建GPU版本
./run_docker_ros2.sh build gpu

# 启动容器
./run_docker_ros2.sh start gpu

# 进入容器
./run_docker_ros2.sh exec gpu

# 在容器内运行（GPU加速）
cd ~/ros2_ws
source install/setup.bash
ros2 run vision_detector detector_node
```

### Jetson Orin Nano

```bash
# 确认在Jetson设备上
cat /etc/nv_tegra_release

# 构建Jetson版本（首次需要60-90分钟）
./run_docker_ros2.sh build jetson

# 启动容器
./run_docker_ros2.sh start jetson

# 进入容器
./run_docker_ros2.sh exec jetson
```

---

## ROS2工作空间配置

### 项目结构

```
yolov5-D435i-main/
├── ros2_ws/                    # ROS2工作空间（Docker挂载）
│   ├── src/
│   │   ├── vision_detector/    # 检测包
│   │   │   ├── vision_detector/
│   │   │   │   ├── __init__.py
│   │   │   │   ├── detector_node.py
│   │   │   │   ├── yolov8_detector.py
│   │   │   │   └── utils.py
│   │   │   ├── launch/
│   │   │   │   └── detector.launch.py
│   │   │   ├── config/
│   │   │   │   └── detector_params.yaml
│   │   │   ├── package.xml
│   │   │   └── setup.py
│   │   │
│   │   └── vision_msgs_custom/ # 自定义消息
│   │       ├── msg/
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   │
│   ├── build/
│   ├── install/
│   └── log/
│
├── Dockerfile.ros2            # ROS2 CPU版本
├── Dockerfile.ros2.gpu        # ROS2 GPU版本
├── Dockerfile.ros2.jetson     # ROS2 Jetson版本
├── docker-compose.ros2.yml    # ROS2编排配置
└── run_docker_ros2.sh         # ROS2启动脚本
```

### 创建ROS2包（在容器内）

按照之前的`ROS2_YOLOV8_D435i_Integration_Guide.md`文档创建包：

```bash
# 1. 进入容器
./run_docker_ros2.sh exec gpu

# 2. 在容器内创建工作空间
cd ~/ros2_ws/src

# 3. 创建检测包
ros2 pkg create --build-type ament_python \
    --dependencies rclpy sensor_msgs vision_msgs cv_bridge \
    --node-name detector_node \
    vision_detector

# 4. 复制代码文件（参考ROS2集成文档）
# 将detector_node.py, yolov8_detector.py, utils.py等文件
# 复制到 vision_detector/vision_detector/

# 5. 构建
cd ~/ros2_ws
colcon build --symlink-install

# 6. Source工作空间
source install/setup.bash
```

---

## 使用方法

### 基础命令

```bash
# 构建镜像
./run_docker_ros2.sh build <cpu|gpu|jetson>

# 启动容器
./run_docker_ros2.sh start <cpu|gpu|jetson>

# 进入容器
./run_docker_ros2.sh exec <cpu|gpu|jetson>

# 停止容器
./run_docker_ros2.sh stop <cpu|gpu|jetson|all>

# 查看日志
./run_docker_ros2.sh logs [cpu|gpu|jetson|all]

# 清理
./run_docker_ros2.sh clean
```

### ROS2专用命令

```bash
# 构建ROS2工作空间
./run_docker_ros2.sh build-ws gpu

# 运行检测节点
./run_docker_ros2.sh run-node gpu

# 启动launch文件
./run_docker_ros2.sh launch gpu detector.launch.py

# 查看ROS2话题
./run_docker_ros2.sh topics gpu

# 查看ROS2节点
./run_docker_ros2.sh nodes gpu

# 启动RViz2
./run_docker_ros2.sh rviz gpu
```

### 在容器内工作

```bash
# 进入容器
./run_docker_ros2.sh exec gpu

# === 在容器内 ===

# Source ROS2环境
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# 查看话题
ros2 topic list

# 查看话题数据
ros2 topic echo /detections

# 运行节点
ros2 run vision_detector detector_node

# 启动launch文件
ros2 launch vision_detector detector.launch.py

# 启动RViz2
rviz2

# 查看节点信息
ros2 node info /vision_detector
```

---

## 完整工作流程

### 1. 首次设置

```bash
# A. 构建Docker镜像
./run_docker_ros2.sh build gpu

# B. 启动容器
./run_docker_ros2.sh start gpu

# C. 进入容器
./run_docker_ros2.sh exec gpu

# D. 在容器内创建ROS2包（参考ROS2集成文档）
# ... 按照文档创建vision_detector包 ...

# E. 构建工作空间
cd ~/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

### 2. 启动系统

```bash
# 终端1: 启动RealSense相机
ros2 launch realsense2_camera rs_launch.py

# 终端2: 启动检测节点
ros2 run vision_detector detector_node

# 终端3: 启动RViz2（可视化）
rviz2
```

或使用launch文件一键启动：

```bash
ros2 launch vision_detector detector.launch.py
```

### 3. 测试系统

```bash
# 查看发布的话题
ros2 topic list

# 查看检测结果
ros2 topic echo /detections

# 查看图像
rqt_image_view

# 查看节点图
rqt_graph
```

---

## 常见问题

### Q1: 为什么需要单独的ROS2版本Dockerfile？

A: ROS2需要大量额外依赖（~3GB），将它们与Standalone版本分离可以：
- 保持Standalone版本轻量
- 允许用户选择是否需要ROS2
- 避免不必要的复杂度

### Q2: 可以在ROS2容器中运行Standalone脚本吗？

A: 可以。ROS2版本包含所有Standalone功能：

```bash
# 在ROS2容器内
cd /workspace
python3 main_debug.py  # 照常运行
```

### Q3: ROS2工作空间的数据会持久化吗？

A: 会。`ros2_ws`目录已挂载到宿主机：

```yaml
volumes:
  - ./ros2_ws:/home/developer/ros2_ws
```

容器删除后，代码和编译结果都会保留。

### Q4: 如何在多台机器之间共享ROS2话题？

A: 确保ROS_DOMAIN_ID相同：

```bash
# 在.env文件中设置
ROS_DOMAIN_ID=42

# 或在容器内
export ROS_DOMAIN_ID=42
```

### Q5: ROS2版本的性能会比Standalone版本差吗？

A: 理论上相同，但：
- ROS2话题发布有小幅延迟（~1-2ms）
- 内存占用更高（+500MB）
- 如果只需要检测，Standalone版本更轻量

### Q6: 可以同时运行Standalone和ROS2容器吗？

A: 可以，但需要注意：
- 使用不同的容器名
- 避免端口冲突
- RealSense相机只能被一个容器访问

### Q7: Jetson上构建ROS2镜像很慢怎么办？

A: 正常现象。优化方法：
- 使用预构建的基础镜像（`dustynv/ros:humble-pytorch-l4t`）
- 配置SWAP（8GB）
- 使用`--no-cache`重建前先清理空间

### Q8: 如何更新ROS2包的代码？

A: 由于使用了`--symlink-install`：

```bash
# 修改代码后，无需重新构建
# 直接重启节点即可

# 如果修改了package.xml或setup.py
cd ~/ros2_ws
colcon build --symlink-install --packages-select vision_detector
```

---

## 性能对比

### 镜像大小

| 版本 | Standalone | ROS2 |
|-----|-----------|------|
| CPU | ~5GB | ~8GB |
| GPU | ~7GB | ~10GB |
| Jetson | ~8GB | ~11GB |

### 内存使用

| 组件 | Standalone | ROS2 |
|-----|-----------|------|
| 基础系统 | ~1GB | ~1.5GB |
| YOLOv8推理 | ~500MB | ~500MB |
| ROS2运行时 | N/A | ~500MB |
| **总计** | **~1.5GB** | **~2.5GB** |

### 启动时间

| 阶段 | Standalone | ROS2 |
|-----|-----------|------|
| 容器启动 | ~2秒 | ~3秒 |
| 模型加载 | ~5秒 | ~5秒 |
| ROS2节点启动 | N/A | ~2秒 |
| **总计** | **~7秒** | **~10秒** |

---

## 最佳实践

### 1. 开发流程

```
本地开发 (Standalone) → ROS2集成测试 → 部署 (ROS2)
```

- 先用Standalone版本快速迭代算法
- 确认功能后再集成到ROS2
- 使用ROS2版本进行系统集成测试

### 2. 资源管理

```bash
# 定期清理未使用的镜像
docker system prune -a

# 查看镜像大小
docker images | grep yolov8

# 只保留当前使用的版本
```

### 3. 多机部署

```bash
# 机器A: 相机+检测节点
ros2 launch vision_detector detector.launch.py

# 机器B: 控制节点（订阅检测结果）
ros2 run my_controller controller_node
```

---

## 总结

### 何时使用ROS2版本

✅ **使用ROS2版本**:
- 已有ROS2机器人系统
- 需要多节点通信
- 需要TF坐标变换
- 需要RViz2可视化

❌ **使用Standalone版本**:
- 单机检测应用
- 快速原型开发
- 资源受限环境
- 不需要ROS2功能

### 文件清单

**ROS2专用文件（4个）**:
- `Dockerfile.ros2` - CPU版本
- `Dockerfile.ros2.gpu` - GPU版本
- `Dockerfile.ros2.jetson` - Jetson版本
- `docker-compose.ros2.yml` - 编排配置
- `run_docker_ros2.sh` - 启动脚本

**配合使用**:
- `ROS2_YOLOV8_D435i_Integration_Guide.md` - ROS2包创建指南
- `requirements.txt` - Python依赖
- `test_docker_env.py` - 环境测试

---

**文档版本**: v1.0  
**适用平台**: x86_64 + Jetson Orin Nano  
**ROS2版本**: Humble  
**最后更新**: 2026年2月
