# ROS2 + RealSense + YOLOv8 完整开发部署指南
## Ubuntu x86 开发 → Jetson Orin Nano ARM64 部署

---

## 📋 项目概述

**技术栈：**
- **ROS2 Humble**（Ubuntu 22.04）
- **Intel RealSense D400 系列深度相机**
- **YOLOv8s 目标检测模型**
- **Docker 容器化部署**
- **开发环境：** Ubuntu 22.04 x86_64
- **目标平台：** Jetson Orin Nano ARM64 (L4T 35.4.1)

---

## 目录

1. [项目结构设计](#1-项目结构设计)
2. [环境准备](#2-环境准备)
3. [Docker 镜像构建策略](#3-docker-镜像构建策略)
4. [ROS2 工作空间配置](#4-ros2-工作空间配置)
5. [开发工作流](#5-开发工作流)
6. [部署到 Jetson](#6-部署到-jetson)
7. [性能优化](#7-性能优化)
8. [常见问题](#8-常见问题)

---

## 1. 项目结构设计

### 1.1 完整目录结构

```
ros2_vision_project/
├── docker/
│   ├── Dockerfile.dev              # x86 开发环境
│   ├── Dockerfile.jetson           # Jetson ARM64 生产环境
│   ├── docker-compose.dev.yml      # 本地开发
│   └── entrypoint.sh               # 容器启动脚本
├── ros2_ws/                        # ROS2 工作空间
│   └── src/
│       ├── vision_detection/       # 主检测节点包
│       │   ├── vision_detection/
│       │   │   ├── __init__.py
│       │   │   ├── detector_node.py
│       │   │   ├── yolov8_detector.py
│       │   │   └── utils.py
│       │   ├── launch/
│       │   │   └── detection.launch.py
│       │   ├── config/
│       │   │   ├── yolov8_config.yaml
│       │   │   └── camera_config.yaml
│       │   ├── resource/
│       │   ├── package.xml
│       │   └── setup.py
│       └── realsense_wrapper/      # RealSense 封装包
│           ├── realsense_wrapper/
│           │   ├── __init__.py
│           │   └── realsense_node.py
│           ├── launch/
│           │   └── realsense.launch.py
│           ├── config/
│           │   └── camera_params.yaml
│           ├── package.xml
│           └── setup.py
├── models/
│   ├── yolov8s.pt                  # 原始模型
│   ├── yolov8s.onnx                # ONNX 模型
│   └── yolov8s_fp16.trt            # TensorRT 引擎
├── scripts/
│   ├── setup_dev_env.sh            # 开发环境设置
│   ├── sync_to_jetson.sh           # 同步到 Jetson
│   ├── build_and_deploy.sh         # 构建部署
│   ├── export_tensorrt.py          # 导出 TensorRT
│   └── benchmark.sh                # 性能测试
├── data/                           # 测试数据
│   └── test_images/
├── requirements.txt
├── .dockerignore
├── .gitignore
└── README.md
```

---

## 2. 环境准备

### 2.1 Ubuntu x86 开发机准备

```bash
# ============================================
# 系统更新
# ============================================
sudo apt update && sudo apt upgrade -y

# ============================================
# 安装 Docker
# ============================================
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER
newgrp docker

# 安装 Docker Compose
sudo apt install docker-compose-plugin -y

# 启用 Buildx（交叉编译支持）
docker buildx create --name multiarch --driver docker-container --use
docker buildx inspect --bootstrap

# ============================================
# 安装 NVIDIA Container Toolkit（如果有 GPU）
# ============================================
distribution=$(. /etc/os-release;echo $ID$VERSION_ID)
curl -fsSL https://nvidia.github.io/libnvidia-container/gpgkey | sudo gpg --dearmor -o /usr/share/keyrings/nvidia-container-toolkit-keyring.gpg
curl -s -L https://nvidia.github.io/libnvidia-container/$distribution/libnvidia-container.list | \
    sed 's#deb https://#deb [signed-by=/usr/share/keyrings/nvidia-container-toolkit-keyring.gpg] https://#g' | \
    sudo tee /etc/apt/sources.list.d/nvidia-container-toolkit.list

sudo apt-get update
sudo apt-get install -y nvidia-container-toolkit
sudo systemctl restart docker

# ============================================
# 安装开发工具
# ============================================
sudo apt install -y \
    git \
    vim \
    curl \
    wget \
    rsync \
    python3-pip \
    python3-venv

# ============================================
# （可选）本地安装 ROS2 Humble 用于开发测试
# ============================================
sudo apt install software-properties-common -y
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

sudo apt update
sudo apt install -y ros-humble-desktop python3-colcon-common-extensions

# 设置环境
echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc
source ~/.bashrc
```

### 2.2 Jetson Orin Nano 准备

```bash
# SSH 登录到 Jetson
ssh jetson@<jetson_ip>

# ============================================
# 检查 JetPack 版本
# ============================================
cat /etc/nv_tegra_release
# 预期输出：R35 (release), REVISION: 4.1

# ============================================
# 更新系统
# ============================================
sudo apt update && sudo apt upgrade -y

# ============================================
# 安装 Docker（如果未安装）
# ============================================
curl -fsSL https://get.docker.com -o get-docker.sh
sudo sh get-docker.sh
sudo usermod -aG docker $USER

# ============================================
# 安装 NVIDIA Container Runtime
# ============================================
sudo apt-get install -y nvidia-docker2
sudo systemctl restart docker

# 配置默认 runtime
sudo tee /etc/docker/daemon.json > /dev/null <<EOF
{
    "default-runtime": "nvidia",
    "runtimes": {
        "nvidia": {
            "path": "nvidia-container-runtime",
            "runtimeArgs": []
        }
    }
}
EOF

sudo systemctl restart docker

# ============================================
# 验证 GPU 可用
# ============================================
docker run --rm --runtime nvidia nvcr.io/nvidia/l4t-base:r35.4.1 nvidia-smi

# ============================================
# 设置最大性能模式
# ============================================
sudo nvpmodel -m 0        # MAXN 模式
sudo jetson_clocks         # 锁定最高频率

# ============================================
# 创建项目目录
# ============================================
mkdir -p ~/ros2_vision_project
```

### 2.3 SSH 免密登录配置

```bash
# 在开发机上
ssh-keygen -t rsa -b 4096
ssh-copy-id jetson@<jetson_ip>

# 测试连接
ssh jetson@<jetson_ip> "echo 'Connection successful!'"
```

---

## 3. Docker 镜像构建策略

### 3.1 开发环境 Dockerfile（x86）

**docker/Dockerfile.dev：**
```dockerfile
# ============================================
# 开发环境镜像（x86_64）
# ============================================
FROM osrf/ros:humble-desktop-full

ENV DEBIAN_FRONTEND=noninteractive
SHELL ["/bin/bash", "-c"]

# ============================================
# 安装系统依赖
# ============================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # 编译工具
    build-essential \
    cmake \
    git \
    wget \
    curl \
    # Python 开发
    python3-pip \
    python3-dev \
    python3-opencv \
    # ROS2 工具
    python3-colcon-common-extensions \
    python3-rosdep \
    # RealSense 依赖
    libusb-1.0-0-dev \
    libssl-dev \
    libgtk-3-dev \
    libglfw3-dev \
    # 其他工具
    vim \
    htop \
    tmux \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# 安装 RealSense SDK
# ============================================
WORKDIR /tmp
RUN git clone --depth 1 --branch v2.54.2 https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=false \
        -DBUILD_GRAPHICAL_EXAMPLES=false \
        -DBUILD_PYTHON_BINDINGS=true \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 && \
    make -j$(nproc) && \
    make install && \
    ldconfig && \
    cd ../.. && rm -rf librealsense

# ============================================
# 安装 Python 依赖
# ============================================
COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir -r /tmp/requirements.txt

# ============================================
# 设置工作空间
# ============================================
WORKDIR /ros2_ws

# ROS2 环境设置
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "if [ -f /ros2_ws/install/setup.bash ]; then source /ros2_ws/install/setup.bash; fi" >> ~/.bashrc

# 复制启动脚本
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

ENTRYPOINT ["/entrypoint.sh"]
CMD ["bash"]
```

### 3.2 Jetson 生产环境 Dockerfile（ARM64）

**docker/Dockerfile.jetson：**
```dockerfile
# ============================================
# Stage 1: 基础环境（使用 NVIDIA L4T 基础镜像）
# ============================================
FROM dustynv/ros:humble-pytorch-l4t-r35.4.1 AS base

ENV DEBIAN_FRONTEND=noninteractive \
    PYTHONUNBUFFERED=1 \
    ROS_DISTRO=humble \
    CUDA_HOME=/usr/local/cuda \
    PATH=/usr/local/cuda/bin:$PATH \
    LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH

SHELL ["/bin/bash", "-c"]

# ============================================
# 安装系统依赖
# ============================================
RUN apt-get update && apt-get install -y --no-install-recommends \
    # ROS2 工具
    python3-colcon-common-extensions \
    python3-rosdep \
    ros-humble-cv-bridge \
    ros-humble-vision-msgs \
    ros-humble-image-transport \
    # RealSense 依赖
    libusb-1.0-0-dev \
    libssl-dev \
    # OpenCV
    python3-opencv \
    libopencv-dev \
    # 工具
    wget \
    curl \
    vim \
    htop \
    && rm -rf /var/lib/apt/lists/*

# ============================================
# Stage 2: 编译 RealSense SDK
# ============================================
FROM base AS realsense-builder

WORKDIR /tmp
RUN git clone --depth 1 --branch v2.54.2 https://github.com/IntelRealSense/librealsense.git && \
    cd librealsense && \
    mkdir build && cd build && \
    cmake .. \
        -DCMAKE_BUILD_TYPE=Release \
        -DBUILD_EXAMPLES=false \
        -DBUILD_GRAPHICAL_EXAMPLES=false \
        -DBUILD_PYTHON_BINDINGS=true \
        -DPYTHON_EXECUTABLE=/usr/bin/python3 \
        -DFORCE_RSUSB_BACKEND=ON && \
    make -j$(nproc) && \
    make install && \
    ldconfig

# ============================================
# Stage 3: Python 依赖层
# ============================================
FROM base AS dependencies

# 复制 RealSense SDK
COPY --from=realsense-builder /usr/local/lib/librealsense* /usr/local/lib/
COPY --from=realsense-builder /usr/local/lib/python3.10/dist-packages/pyrealsense2* /usr/local/lib/python3.10/dist-packages/
COPY --from=realsense-builder /usr/local/include/librealsense2 /usr/local/include/librealsense2
RUN ldconfig

# 安装 Python 依赖
COPY requirements.txt /tmp/
RUN pip3 install --no-cache-dir --break-system-packages -r /tmp/requirements.txt

# ============================================
# Stage 4: 应用层
# ============================================
FROM dependencies AS application

# 创建工作空间
WORKDIR /ros2_ws

# 复制 ROS2 工作空间源码
COPY ros2_ws/src /ros2_ws/src

# 初始化 rosdep
RUN source /opt/ros/humble/setup.bash && \
    rosdep update && \
    rosdep install --from-paths src --ignore-src -r -y || true

# 编译 ROS2 工作空间
RUN source /opt/ros/humble/setup.bash && \
    colcon build \
        --cmake-args -DCMAKE_BUILD_TYPE=Release \
        --parallel-workers $(nproc)

# 创建模型目录
RUN mkdir -p /ros2_ws/models

# 设置环境
RUN echo "source /opt/ros/humble/setup.bash" >> ~/.bashrc && \
    echo "source /ros2_ws/install/setup.bash" >> ~/.bashrc

# 复制启动脚本
COPY docker/entrypoint.sh /entrypoint.sh
RUN chmod +x /entrypoint.sh

# 复制 udev 规则
COPY docker/99-realsense-libusb.rules /etc/udev/rules.d/

# 健康检查
HEALTHCHECK --interval=30s --timeout=10s --start-period=10s --retries=3 \
    CMD ros2 node list || exit 1

ENTRYPOINT ["/entrypoint.sh"]
CMD ["ros2", "launch", "vision_detection", "detection.launch.py"]
```

### 3.3 Docker Entrypoint 脚本

**docker/entrypoint.sh：**
```bash
#!/bin/bash
set -e

# Source ROS2 environment
source /opt/ros/humble/setup.bash

# Source workspace if built
if [ -f /ros2_ws/install/setup.bash ]; then
    source /ros2_ws/install/setup.bash
fi

# Execute command
exec "$@"
```

### 3.4 RealSense udev 规则

**docker/99-realsense-libusb.rules：**
```
# Intel RealSense D400 Series
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0ad*", MODE:="0666", GROUP:="plugdev"
SUBSYSTEMS=="usb", ATTRS{idVendor}=="8086", ATTRS{idProduct}=="0b*", MODE:="0666", GROUP:="plugdev"
```

### 3.5 Docker Compose（开发环境）

**docker/docker-compose.dev.yml：**
```yaml
version: '3.8'

services:
  ros2_dev:
    build:
      context: ..
      dockerfile: docker/Dockerfile.dev
    image: ros2-vision-dev:latest
    container_name: ros2_vision_dev
    privileged: true
    network_mode: host
    environment:
      - DISPLAY=${DISPLAY}
      - QT_X11_NO_MITSHM=1
    volumes:
      # 挂载代码
      - ../ros2_ws:/ros2_ws
      - ../models:/ros2_ws/models
      - ../data:/ros2_ws/data
      # X11 显示
      - /tmp/.X11-unix:/tmp/.X11-unix:rw
      # USB 设备（RealSense）
      - /dev:/dev
    devices:
      - /dev/bus/usb:/dev/bus/usb
    runtime: nvidia
    command: bash
```

---

## 4. ROS2 工作空间配置

### 4.1 视觉检测包（vision_detection）

#### package.xml

**ros2_ws/src/vision_detection/package.xml：**
```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vision_detection</name>
  <version>1.0.0</version>
  <description>YOLOv8 vision detection with RealSense</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>cv_bridge</depend>
  <depend>std_msgs</depend>

  <test_depend>ament_copyright</test_depend>
  <test_depend>ament_flake8</test_depend>
  <test_depend>ament_pep257</test_depend>
  <test_depend>python3-pytest</test_depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

#### setup.py

**ros2_ws/src/vision_detection/setup.py：**
```python
from setuptools import setup
import os
from glob import glob

package_name = 'vision_detection'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your@email.com',
    description='YOLOv8 vision detection',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = vision_detection.detector_node:main',
        ],
    },
)
```

#### YOLOv8 检测器类

**ros2_ws/src/vision_detection/vision_detection/yolov8_detector.py：**
```python
import cv2
import numpy as np
from ultralytics import YOLO
import torch
from typing import List, Tuple


class YOLOv8Detector:
    """YOLOv8 目标检测器封装"""
    
    def __init__(
        self,
        model_path: str = 'models/yolov8s.pt',
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        device: str = 'cuda',
        use_tensorrt: bool = False
    ):
        """
        初始化 YOLOv8 检测器
        
        Args:
            model_path: 模型路径
            conf_threshold: 置信度阈值
            iou_threshold: NMS IOU 阈值
            device: 设备 ('cuda' or 'cpu')
            use_tensorrt: 是否使用 TensorRT
        """
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        
        # 加载模型
        if use_tensorrt and model_path.endswith('.trt'):
            # TensorRT 引擎
            from ultralytics.engine.exporter import TensorRTPredictor
            self.model = YOLO(model_path, task='detect')
        else:
            # PyTorch 或 ONNX
            self.model = YOLO(model_path)
        
        # 预热
        self._warmup()
    
    def _warmup(self, size=(640, 640)):
        """模型预热"""
        dummy_img = np.zeros((size[1], size[0], 3), dtype=np.uint8)
        for _ in range(3):
            _ = self.model(dummy_img, verbose=False)
    
    def detect(
        self, 
        image: np.ndarray,
        classes: List[int] = None
    ) -> List[Tuple]:
        """
        执行目标检测
        
        Args:
            image: BGR 图像
            classes: 要检测的类别列表
        
        Returns:
            检测结果列表 [(x1, y1, x2, y2, conf, cls), ...]
        """
        # 推理
        results = self.model(
            image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            classes=classes,
            verbose=False
        )
        
        # 解析结果
        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                
                detections.append((x1, y1, x2, y2, conf, cls))
        
        return detections
    
    def get_class_name(self, class_id: int) -> str:
        """获取类别名称"""
        return self.model.names[class_id]


# ============================================
# 测试代码
# ============================================
if __name__ == "__main__":
    detector = YOLOv8Detector(
        model_path='models/yolov8s.pt',
        device='cuda'
    )
    
    # 测试图像
    img = cv2.imread('test.jpg')
    detections = detector.detect(img)
    
    # 可视化
    for x1, y1, x2, y2, conf, cls in detections:
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        label = f"{detector.get_class_name(cls)}: {conf:.2f}"
        cv2.putText(img, label, (int(x1), int(y1)-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow('Detections', img)
    cv2.waitKey(0)
```

#### ROS2 检测节点

**ros2_ws/src/vision_detection/vision_detection/detector_node.py：**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import numpy as np
import cv2
from .yolov8_detector import YOLOv8Detector


class DetectorNode(Node):
    """YOLOv8 ROS2 检测节点"""
    
    def __init__(self):
        super().__init__('yolov8_detector')
        
        # 声明参数
        self.declare_parameter('model_path', '/ros2_ws/models/yolov8s.pt')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('use_tensorrt', False)
        self.declare_parameter('publish_viz', True)
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('conf_threshold').value
        iou_threshold = self.get_parameter('iou_threshold').value
        device = self.get_parameter('device').value
        use_tensorrt = self.get_parameter('use_tensorrt').value
        self.publish_viz = self.get_parameter('publish_viz').value
        
        # 初始化检测器
        self.get_logger().info(f'Loading model: {model_path}')
        self.detector = YOLOv8Detector(
            model_path=model_path,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            device=device,
            use_tensorrt=use_tensorrt
        )
        self.get_logger().info('Model loaded successfully')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 订阅相机话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # 发布检测结果
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        # 发布可视化图像
        if self.publish_viz:
            self.viz_pub = self.create_publisher(
                Image,
                '/detections/visualization',
                10
            )
        
        # 缓存
        self.latest_depth = None
        self.camera_info = None
        
        self.get_logger().info('Detector node initialized')
    
    def depth_callback(self, msg):
        """深度图回调"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def camera_info_callback(self, msg):
        """相机信息回调"""
        self.camera_info = msg
    
    def image_callback(self, msg):
        """图像回调和检测"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 执行检测
            detections = self.detector.detect(cv_image)
            
            # 发布检测结果
            self.publish_detections(detections, msg.header)
            
            # 发布可视化
            if self.publish_viz:
                viz_image = self.visualize_detections(cv_image, detections)
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                viz_msg.header = msg.header
                self.viz_pub.publish(viz_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')
    
    def publish_detections(self, detections, header):
        """发布检测结果为 ROS2 消息"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        for x1, y1, x2, y2, conf, cls in detections:
            detection = Detection2D()
            
            # 边界框
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            
            # 类别和置信度
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(cls)
            hypothesis.hypothesis.score = conf
            detection.results.append(hypothesis)
            
            # 如果有深度信息，计算距离
            if self.latest_depth is not None:
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                if 0 <= cy < self.latest_depth.shape[0] and 0 <= cx < self.latest_depth.shape[1]:
                    depth = self.latest_depth[cy, cx] / 1000.0  # mm to m
                    detection.bbox.center.position.z = float(depth)
            
            detection_array.detections.append(detection)
        
        self.detection_pub.publish(detection_array)
    
    def visualize_detections(self, image, detections):
        """可视化检测结果"""
        viz_image = image.copy()
        
        for x1, y1, x2, y2, conf, cls in detections:
            # 绘制边界框
            color = (0, 255, 0)
            cv2.rectangle(viz_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            
            # 标签
            class_name = self.detector.get_class_name(cls)
            label = f'{class_name}: {conf:.2f}'
            
            # 如果有深度信息
            if self.latest_depth is not None:
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                if 0 <= cy < self.latest_depth.shape[0] and 0 <= cx < self.latest_depth.shape[1]:
                    depth = self.latest_depth[cy, cx] / 1000.0
                    label += f' | {depth:.2f}m'
            
            # 绘制文本
            cv2.putText(viz_image, label, (int(x1), int(y1) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return viz_image


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### Launch 文件

**ros2_ws/src/vision_detection/launch/detection.launch.py：**
```python
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/ros2_ws/models/yolov8s.pt',
        description='Path to YOLOv8 model'
    )
    
    use_tensorrt_arg = DeclareLaunchArgument(
        'use_tensorrt',
        default_value='false',
        description='Use TensorRT engine'
    )
    
    # RealSense Launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense_wrapper'),
                'launch',
                'realsense.launch.py'
            )
        ])
    )
    
    # 检测节点
    detector_node = Node(
        package='vision_detection',
        executable='detector_node',
        name='yolov8_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': 0.25,
            'iou_threshold': 0.45,
            'device': 'cuda',
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
            'publish_viz': True
        }]
    )
    
    return LaunchDescription([
        model_path_arg,
        use_tensorrt_arg,
        realsense_launch,
        detector_node
    ])
```

### 4.2 RealSense 封装包

#### RealSense 节点

**ros2_ws/src/realsense_wrapper/realsense_wrapper/realsense_node.py：**
```python
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class RealSenseNode(Node):
    """RealSense 相机 ROS2 节点"""
    
    def __init__(self):
        super().__init__('realsense_camera')
        
        # 参数
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('align_depth', True)
        
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        align_depth = self.get_parameter('align_depth').value
        
        # 初始化 RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        
        # 启动 pipeline
        profile = self.pipeline.start(config)
        
        # 对齐
        if align_depth:
            self.align = rs.align(rs.stream.color)
        else:
            self.align = None
        
        # 获取相机内参
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_profile.get_intrinsics()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 发布器
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        
        # 定时器
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        
        self.get_logger().info('RealSense camera node initialized')
    
    def timer_callback(self):
        """定时发布图像"""
        try:
            # 等待帧
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            # 对齐
            if self.align:
                frames = self.align.process(frames)
            
            # 获取彩色和深度
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
            
            # 转换为 numpy
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 时间戳
            timestamp = self.get_clock().now().to_msg()
            
            # 发布彩色图
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = 'camera_color_optical_frame'
            self.color_pub.publish(color_msg)
            
            # 发布深度图
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='passthrough')
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            self.depth_pub.publish(depth_msg)
            
            # 发布相机信息
            camera_info = self.create_camera_info_msg(timestamp)
            self.camera_info_pub.publish(camera_info)
        
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
    
    def create_camera_info_msg(self, timestamp):
        """创建相机信息消息"""
        camera_info = CameraInfo()
        camera_info.header.stamp = timestamp
        camera_info.header.frame_id = 'camera_color_optical_frame'
        
        camera_info.width = self.intrinsics.width
        camera_info.height = self.intrinsics.height
        
        camera_info.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        camera_info.d = list(self.intrinsics.coeffs)
        camera_info.distortion_model = 'plumb_bob'
        
        return camera_info
    
    def destroy_node(self):
        """停止相机"""
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
```

#### RealSense Launch

**ros2_ws/src/realsense_wrapper/launch/realsense.launch.py：**
```python
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    realsense_node = Node(
        package='realsense_wrapper',
        executable='realsense_node',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 30,
            'align_depth': True
        }]
    )
    
    return LaunchDescription([realsense_node])
```

#### package.xml 和 setup.py

**package.xml：**
```xml
<?xml version="1.0"?>
<package format="3">
  <name>realsense_wrapper</name>
  <version>1.0.0</version>
  <description>RealSense camera wrapper</description>
  <maintainer email="your@email.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>cv_bridge</depend>

  <export>
    <build_type>ament_python</build_type>
  </export>
</package>
```

**setup.py：**
```python
from setuptools import setup
import os
from glob import glob

package_name = 'realsense_wrapper'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    entry_points={
        'console_scripts': [
            'realsense_node = realsense_wrapper.realsense_node:main',
        ],
    },
)
```

---

## 5. 开发工作流

### 5.1 本地开发（Docker 方式）

```bash
# ============================================
# 1. 构建开发环境镜像
# ============================================
cd ~/ros2_vision_project
docker build -f docker/Dockerfile.dev -t ros2-vision-dev:latest .

# ============================================
# 2. 启动开发容器
# ============================================
# 允许 X11 转发
xhost +local:docker

docker-compose -f docker/docker-compose.dev.yml up -d

# 进入容器
docker exec -it ros2_vision_dev bash

# ============================================
# 3. 在容器内编译工作空间
# ============================================
cd /ros2_ws
colcon build --symlink-install

# ============================================
# 4. 运行节点测试
# ============================================
source install/setup.bash

# 启动 RealSense
ros2 launch realsense_wrapper realsense.launch.py

# 新终端：启动检测
ros2 launch vision_detection detection.launch.py

# 新终端：查看话题
ros2 topic list
ros2 topic echo /detections

# 可视化（rqt_image_view）
ros2 run rqt_image_view rqt_image_view
```

### 5.2 本地开发（非 Docker 方式）

```bash
# ============================================
# 1. 编译工作空间
# ============================================
cd ~/ros2_vision_project/ros2_ws
source /opt/ros/humble/setup.bash
colcon build --symlink-install

# ============================================
# 2. 运行
# ============================================
source install/setup.bash
ros2 launch vision_detection detection.launch.py
```

---

## 6. 部署到 Jetson

### 6.1 同步代码脚本

**scripts/sync_to_jetson.sh：**
```bash
#!/bin/bash

# ============================================
# 配置
# ============================================
JETSON_IP="192.168.1.100"
JETSON_USER="jetson"
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
```

### 6.2 构建并部署脚本

**scripts/build_and_deploy.sh：**
```bash
#!/bin/bash

set -e

# ============================================
# 配置
# ============================================
JETSON_IP="192.168.1.100"
JETSON_USER="jetson"
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
```

### 6.3 TensorRT 导出脚本

**scripts/export_tensorrt.py：**
```python
#!/usr/bin/env python3

from ultralytics import YOLO
import argparse


def export_tensorrt(
    model_path='models/yolov8s.pt',
    output_path='models/yolov8s_fp16.trt',
    imgsz=640,
    half=True
):
    """
    导出 YOLOv8 模型为 TensorRT 引擎
    
    在 Jetson 上运行此脚本以获得最佳性能
    """
    print(f"Loading model: {model_path}")
    model = YOLO(model_path)
    
    print("Exporting to TensorRT...")
    model.export(
        format='engine',  # TensorRT
        imgsz=imgsz,
        half=half,        # FP16
        simplify=True,
        workspace=4       # GB
    )
    
    print(f"✅ TensorRT engine saved to: {output_path}")


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--model', type=str, default='models/yolov8s.pt')
    parser.add_argument('--output', type=str, default='models/yolov8s_fp16.trt')
    parser.add_argument('--imgsz', type=int, default=640)
    parser.add_argument('--half', action='store_true', default=True)
    
    args = parser.parse_args()
    
    export_tensorrt(
        model_path=args.model,
        output_path=args.output,
        imgsz=args.imgsz,
        half=args.half
    )
```

**在 Jetson 上执行导出：**
```bash
ssh jetson@<jetson_ip>
cd ~/ros2_vision_project

# 在容器内执行
docker exec -it ros2-vision-container python3 scripts/export_tensorrt.py \
    --model /ros2_ws/models/yolov8s.pt \
    --output /ros2_ws/models/yolov8s_fp16.trt \
    --imgsz 640 \
    --half
```

---

## 7. 性能优化

### 7.1 性能测试脚本

**scripts/benchmark.sh：**
```bash
#!/bin/bash

JETSON_IP="192.168.1.100"
JETSON_USER="jetson"
CONTAINER_NAME="ros2-vision-container"

echo "🔍 Running performance benchmark..."

ssh $JETSON_USER@$JETSON_IP << 'ENDSSH'
# GPU 状态
echo "=== GPU Status ==="
nvidia-smi

# 系统状态
echo -e "\n=== Jetson Stats ==="
sudo tegrastats --interval 1000 --logfile /tmp/tegrastats.log &
TEGRA_PID=$!
sleep 5
kill $TEGRA_PID
cat /tmp/tegrastats.log

# ROS2 节点性能
echo -e "\n=== ROS2 Topics ==="
docker exec ros2-vision-container bash -c "source /opt/ros/humble/setup.bash && ros2 topic hz /detections"

ENDSSH

echo "✅ Benchmark complete!"
```

### 7.2 优化建议

**模型优化：**
1. **使用 TensorRT FP16**：2-3x 加速
2. **减小输入尺寸**：640 → 416
3. **使用 YOLOv8n**：更快但精度稍低

**系统优化：**
```bash
# Jetson 性能模式
sudo nvpmodel -m 0
sudo jetson_clocks

# 增加 swap（如果 OOM）
sudo fallocate -l 8G /swapfile
sudo chmod 600 /swapfile
sudo mkswap /swapfile
sudo swapon /swapfile
```

**ROS2 优化：**
```python
# 在节点中使用 QoS 优化
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy

qos = QoSProfile(
    reliability=QoSReliabilityPolicy.BEST_EFFORT,
    durability=QoSDurabilityPolicy.VOLATILE,
    depth=1
)

self.image_sub = self.create_subscription(
    Image, '/camera/color/image_raw', self.callback, qos
)
```

---

## 8. 常见问题

### 8.1 RealSense 问题

**问题：No device connected**
```bash
# 检查 USB 连接
lsusb | grep Intel

# 检查 udev 规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 在 Docker 中需要 privileged 和 /dev 挂载
docker run --privileged -v /dev:/dev ...
```

**问题：权限问题**
```bash
# 添加用户到 plugdev 组
sudo usermod -aG plugdev $USER
```

### 8.2 Docker 问题

**问题：无法访问 GPU**
```bash
# 确保使用 nvidia runtime
docker run --runtime nvidia --gpus all ...

# 检查 CUDA
docker run --rm --runtime nvidia nvcr.io/nvidia/l4t-base:r35.4.1 nvidia-smi
```

### 8.3 ROS2 问题

**问题：找不到包**
```bash
# 确保 source 了环境
source /opt/ros/humble/setup.bash
source /ros2_ws/install/setup.bash

# 重新编译
colcon build --symlink-install
```

**问题：话题无数据**
```bash
# 检查节点运行
ros2 node list

# 检查话题
ros2 topic list
ros2 topic echo /camera/color/image_raw

# 检查 QoS 兼容性
ros2 topic info /camera/color/image_raw -v
```

---

## 9. 完整部署流程总结

### 开发机（x86 Ubuntu）

```bash
# 1. 开发代码
cd ~/ros2_vision_project/ros2_ws
colcon build --symlink-install

# 2. 本地测试
source install/setup.bash
ros2 launch vision_detection detection.launch.py

# 3. 提交代码（可选）
git add .
git commit -m "Update detection algorithm"
git push
```

### 部署到 Jetson

```bash
# 一键部署
cd ~/ros2_vision_project
chmod +x scripts/build_and_deploy.sh
./scripts/build_and_deploy.sh

# 或手动步骤：
# 1. 同步代码
./scripts/sync_to_jetson.sh

# 2. SSH 到 Jetson 构建
ssh jetson@<jetson_ip>
cd ~/ros2_vision_project
docker build -f docker/Dockerfile.jetson -t ros2-vision-jetson:latest .

# 3. 运行
docker run -d \
    --name ros2-vision-container \
    --runtime nvidia \
    --network host \
    --privileged \
    -v /dev:/dev \
    -v ~/ros2_vision_project/models:/ros2_ws/models \
    ros2-vision-jetson:latest

# 4. 查看日志
docker logs -f ros2-vision-container

# 5. 查看话题
docker exec -it ros2-vision-container bash
source /opt/ros/humble/setup.bash
ros2 topic list
ros2 topic hz /detections
```

---

## 10. requirements.txt

**requirements.txt：**
```txt
# Deep Learning
ultralytics==8.0.196      # YOLOv8
torch>=2.0.0              # 由基础镜像提供（Jetson）
torchvision               # 由基础镜像提供（Jetson）

# Computer Vision
opencv-python-headless==4.8.1.78
Pillow==10.0.0

# RealSense
# pyrealsense2            # 通过源码编译安装

# ROS2 (由基础镜像提供)
# rclpy
# cv_bridge
# sensor_msgs
# vision_msgs

# Utils
numpy==1.24.3
pyyaml==6.0.1
```

---

## 总结

这套方案的优势：

✅ **ROS2 原生支持**：标准消息、Launch 系统、参数服务器  
✅ **模块化设计**：相机节点独立、检测节点独立  
✅ **高性能**：TensorRT 加速、RGBD 融合  
✅ **易于开发**：x86 开发、Docker 隔离环境  
✅ **一键部署**：自动化脚本、容器化部署  
✅ **生产就绪**：健康检查、日志管理、自动重启  

你现在可以开始开发了！有任何问题随时问我。🚀  







sudo tee /etc/docker/daemon.json <<-EOF
>{ 
  "registry-mirrors" : 
    [ 
      "https://docker.m.daocloud.io",
      "https://docker.xuanyuan.me", 
      "https://docker.1ms.run"
    ] 
}
>EOF

sudo systemctl daemon-reload
sudo systemctl restart docker