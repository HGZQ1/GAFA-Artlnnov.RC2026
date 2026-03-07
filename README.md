# GAFA-Artlnnov.RC2026
## 广州美术学院RC2026艺创战队视觉组仓库  
本仓库用于存放广州美术学院RC2026艺创战队视觉组的相关资料和项目文件。
# 项目结构目录

## 目录树

```
ros2_vision_project
docker/
│   ├── docker-compose.ros2.yml       # Docker Compose 配置文件
│   ├── Dockerfile.ros2               # ROS2 CPU 版本 Dockerfile
│   ├── Dockerfile.ros2.gpu           # ROS2 GPU 版本 Dockerfile
│   └── Dockerfile.ros2.jetson        # ROS2 Jetson 版本 Dockerfile
ros2_ws/
├── .vscode/                              # VSCode 编辑器配置目录
├── src/                                  # ROS2 功能包源码目录
│   ├── vision_detector/                  # 视觉检测功能包
│   │   ├── CMakeLists.txt               # CMake 构建配置文件
│   │   ├── package.xml                   # ROS2 包描述文件
│   │   ├── setup.cfg                    # Python 包配置文件
│   │   ├── setup.py                     # Python 包安装脚本
│   │   ├── launch/                      # 启动文件目录
│   │   │   ├── detector.launch.py       # 检测器启动文件*
│   │   │   └── detector_rviz.launch.py  # 检测器 RViz 启动文件*
│   │   ├── rviz/                        # RViz 配置文件目录
│   │   │   └── detector_view.rviz       # 检测器可视化配置
│   │   ├── vision_detector/             # Python 源代码目录
│   │   │   ├── __init__.py              # Python 包初始化文件
│   │   │   ├── detector_node.py         # 检测器ROS2节点主程序*
│   │   │   ├── yolov8_detector.py       # YOLOv8 检测器实现*
│   │   │   ├── utils.py                 # 工具函数*
│   │   │   ├── config/                  # 配置文件目录
│   │   │   └── rviz/                    # RViz 相关文件目录
│   │   ├── test/                        # 测试文件目录
│   │   │   ├── test_copyright.py        # 版权测试
│   │   │   ├── test_flake8.py           # 代码风格测试
│   │   │   └── test_pep257.py           # 文档风格测试
│   │   ├── resource/                    # 资源文件目录
│   │   │   └── vision_detector          # 包标识文件
│   │   └── weights/                     # 模型权重目录
│   │       └── yolov8n.pt               # YOLOv8n 模型权重文件
│   └── vision_msgs_custom/              # 自定义消息功能包
│       ├── CMakeLists.txt               # CMake 构建配置文件
│       ├── package.xml                   # ROS2 包描述文件
│       ├── msg/                          # 自定义消息定义目录
│       │   ├── Detection2DExtended.msg   # 扩展的2D检测消息
│       │   └── ObjectDistance.msg       # 物体距离消息
│       ├── include/                      # C++ 头文件目录
│       │   └── vision_msgs_custom/      # 自定义消息头文件
│       └── src/                         # C++ 源代码目录
├── yolov8n.pt                            # YOLOv8n 模型权重文件
├── test.jpg                              # 视觉测试图片1
├── test1.jpg                             # 视觉测试图片2
├── test2.jpg                             # 视觉测试图片3
├── test_detector.py                      # 检测器测试脚本
├── ROS2_YOLOV8_D435i_Integration_Guide.md  # 项目集成指南
└── 启的.md                               # 项目说明文档（待完善命名）
```

## 目录说明

### 根目录文件

| 文件/目录 | 说明 |
|---------|------|
| `docker-compose.yml` | Docker Compose 配置文件，用于定义和运行多容器 Docker 应用 |
| `yolov8n.pt` | YOLOv8n 模型权重文件，用于目标检测 |
| `test.jpg`, `test1.jpg`, `test2.jpg` | 视觉检测测试图片（组长可爱照片） |
| `test_detector.py` | 检测器测试脚本 |
| `ROS2_YOLOV8_D435i_Integration_Guide.md` | 项目集成指南文档 |
| `启的.md` | 项目启动命令 |

### Docker 相关目录

| 目录 | 说明 |
|-----|------|
| `docker/` | Docker 环境配置相关目录 |
| `docker/rosdocker1/` | Docker ROS2 配置目录 |
| `docker/rosdocker1/docker-compose.ros2.yml` | Docker Compose 配置文件，定义了 CPU、GPU 和 Jetson 三个版本的服务 |
| `docker/rosdocker1/Dockerfile.ros2` | ROS2 CPU 版本 Dockerfile |
| `docker/rosdocker1/Dockerfile.ros2.gpu` | ROS2 GPU 版本 Dockerfile |
| `docker/rosdocker1/Dockerfile.ros2.jetson` | ROS2 Jetson 版本 Dockerfile |

### ROS2 源代码目录

| 目录 | 说明 |
|-----|------|
| `src/` | ROS2 功能包源码目录 |
| `src/vision_detector/` | 视觉检测功能包，包含 YOLOv8 检测器实现 |
| `src/vision_msgs_custom/` | 自定义消息功能包，定义了扩展的检测消息类型 |

### vision_detector 功能包

| 目录/文件 | 说明 |
|----------|------|
| `CMakeLists.txt` | CMake 构建配置文件 |
| `package.xml` | ROS2 包描述文件，定义了包的依赖关系 |
| `setup.cfg` | Python 包配置文件 |
| `setup.py` | Python 包安装脚本 |
| `launch/` | 启动文件目录，包含检测器和 RViz 的启动配置 |
| `rviz/` | RViz 配置文件目录 |
| `vision_detector/` | Python 源代码目录 |
| `test/` | 测试文件目录 |
| `resource/` | 资源文件目录 |
| `weights/` | 模型权重目录 |

### vision_msgs_custom 功能包

| 目录/文件 | 说明 |
|----------|------|
| `CMakeLists.txt` | CMake 构建配置文件 |
| `package.xml` | ROS2 包描述文件 |
| `msg/` | 自定义消息定义目录 |
| `include/` | C++ 头文件目录 |
| `src/` | C++ 源代码目录 |

## Docker 服务说明

### yolov8-ros2-cpu
- **说明**: ROS2 CPU 版本服务
- **特点**: 不需要 GPU 支持，适合在没有 GPU 的机器上运行
- **镜像**: yolov8-d435i:ros2-cpu-latest
- **容器名**: yolov8-ros2-cpu

### yolov8-ros2-gpu
- **说明**: ROS2 GPU 版本服务
- **特点**: 需要 NVIDIA GPU 支持，性能更高
- **镜像**: yolov8-d435i:ros2-gpu-latest
- **容器名**: yolov8-ros2-gpu
- **GPU 配置**: 使用 NVIDIA 运行时和 GPU 资源预留

### yolov8-ros2-jetson
- **说明**: ROS2 Jetson 版本服务
- **特点**: 专为 NVIDIA Jetson 系列设备优化
- **镜像**: yolov8-d435i:ros2-jetson-latest
- **容器名**: yolov8-ros2-jetson
- **特殊配置**: 
  - 挂载 Jetson CUDA 库
  - 配置 Jetson GPU 设备
  - 设置内存限制和预留

## 卷挂载说明

| 宿主机路径 | 容器路径 | 说明 |
|----------|---------|------|
| `../../` | `/workspace` | 项目代码目录 |
| `../../src` | `/ros2_ws/src` | ROS2 工作空间源码目录 |
| `../../yolov8n.pt` | `/workspace/yolov8n.pt` | 模型权重文件 |
| `/tmp/.X11-unix` | `/tmp/.X11-unix` | X11 socket，用于图形界面显示 |
| `../../runs` | `/workspace/runs` | 运行结果目录 |
| `../../data` | `/workspace/data` | 数据集目录 |
| `/usr/local/cuda` | `/usr/local/cuda` (Jetson) | Jetson CUDA 库（只读） |

## 环境变量说明

| 变量名 | 说明 |
|-------|------|
| `DISPLAY` | X11 显示环境变量 |
| `QT_X11_NO_MITSHM` | QT X11 配置 |
| `PYTHONUNBUFFERED` | Python 输出不缓冲 |
| `ROS_DOMAIN_ID` | ROS2 域 ID，用于 ROS2 网络隔离 |
| `NVIDIA_VISIBLE_DEVICES` | 可见的 NVIDIA 设备 |
| `NVIDIA_DRIVER_CAPABILITIES` | NVIDIA 驱动能力 |
| `OPENBLAS_CORETYPE` | OpenBLAS 核心类型（Jetson） |

## 设备挂载说明

| 设备 | 说明 |
|-----|------|
| `/dev/bus/usb` | USB 设备，用于连接相机等设备 |
| `/dev/video0-3` | 视频设备，用于相机输入 |
| `/dev/nvhost-*` | Jetson GPU 相关设备 |
| `/dev/nvmap` | Jetson 内存映射设备 |
| `/tmp/.X11-unix` | X11 socket，用于图形界面显示 |

## 使用说明

1. **构建镜像**:
   ```bash
   cd docker/rosdocker1
   docker-compose -f docker-compose.ros2.yml build
   ```

2. **启动服务**:
   ```bash
   # CPU 版本
   docker-compose -f docker-compose.ros2.yml up yolov8-ros2-cpu

   # GPU 版本
   docker-compose -f docker-compose.ros2.yml up yolov8-ros2-gpu

   # Jetson 版本
   docker-compose -f docker-compose.ros2.yml up yolov8-ros2-jetson
   ```

3. **进入容器**:
   ```bash
   docker exec -it yolov8-ros2-cpu bash
   ```

4. **编译 ROS2 工作空间**:
   ```bash
   cd /ros2_ws
   colcon build
   source install/setup.bash
   ```

5. **运行检测器**:
   ```bash
   ros2 launch vision_detector detector.launch.py
   ```

6. **运行 RViz**:
   ```bash
   ros2 run rviz2 rviz2 -d /ros2_ws/install/vision_detector/share/vision_detector/rviz/detector_view.rviz
   ```

## 注意事项

1. 确保 Docker 和 Docker Compose 已正确安装
2. GPU 版本需要安装 NVIDIA Docker 运行时
3. Jetson 版本需要在 NVIDIA Jetson 设备上运行
4. 确保 USB 设备权限正确配置
5. 首次运行需要构建镜像，可能需要较长时间
6. 修改源代码后需要重新编译 ROS2 工作空间
7. 确保 X11 转发配置正确，以便显示图形界面

## 许可证

本项目遵循相应的开源许可证。

