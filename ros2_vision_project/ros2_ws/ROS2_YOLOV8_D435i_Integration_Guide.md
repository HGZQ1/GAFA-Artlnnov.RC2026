# YOLOv8-D435i ROS2 集成开发指南

## 📋 目录

1. [项目概述](#项目概述)
2. [系统架构设计](#系统架构设计)
3. [开发环境准备](#开发环境准备)
4. [ROS2 Package创建](#ros2-package创建)
5. [自定义消息定义](#自定义消息定义)
6. [核心节点实现](#核心节点实现)
7. [Launch文件配置](#launch文件配置)
8. [编译与部署](#编译与部署)
9. [测试与验证](#测试与验证)
10. [性能优化](#性能优化)
11. [故障排查](#故障排查)

---

## 项目概述

### 目标
将现有的YOLOv5-D435i项目升级为YOLOv8并集成到ROS2系统，实现：
- 实时目标检测与深度测距
- 发布检测结果到ROS2 topic
- 发布3D点云/TF坐标变换
- 与导航/控制系统通信

### 技术栈
- **硬件**: Jetson Orin Nano + Intel RealSense D435i
- **操作系统**: Ubuntu 22.04 (推荐)
- **ROS版本**: ROS2 Humble
- **模型**: YOLOv8n (Ultralytics)
- **语言**: Python 3.10+

---

## 系统架构设计

### ROS2节点架构图

```
┌─────────────────────────────────────────────────────────────┐
│                    ROS2 系统架构                             │
└─────────────────────────────────────────────────────────────┘

┌──────────────────┐         ┌──────────────────────────┐
│  D435i Camera    │────────>│  vision_detector_node    │
│  (硬件)          │  RGB+D  │  (检测节点)              │
└──────────────────┘         └──────────────────────────┘
                                       │
                    ┌──────────────────┼──────────────────┐
                    │                  │                  │
                    ▼                  ▼                  ▼
         /camera/color/image_raw  /detections    /detected_objects_3d
         (sensor_msgs/Image)      (自定义消息)   (vision_msgs/Detection3DArray)
                    │                  │                  │
                    ▼                  ▼                  ▼
         ┌─────────────────┐  ┌──────────────┐  ┌─────────────┐
         │  Image View      │  │ 控制节点     │  │ 导航节点    │
         │  (可视化)        │  │ (下游应用)   │  │ (路径规划)  │
         └─────────────────┘  └──────────────┘  └─────────────┘
```

### Topic设计

| Topic名称 | 消息类型 | 频率 | 说明 |
|----------|---------|------|------|
| `/camera/color/image_raw` | `sensor_msgs/Image` | 30Hz | 原始RGB图像 |
| `/camera/depth/image_rect_raw` | `sensor_msgs/Image` | 30Hz | 对齐深度图 |
| `/camera/color/camera_info` | `sensor_msgs/CameraInfo` | 30Hz | 相机内参 |
| `/detections` | `vision_msgs/Detection2DArray` | 30Hz | 2D检测结果 |
| `/detected_objects_3d` | `vision_msgs/Detection3DArray` | 30Hz | 3D位置信息 |
| `/detection_image` | `sensor_msgs/Image` | 30Hz | 可视化图像 |

---

## 开发环境准备

### 1. 安装ROS2 Humble (Jetson Orin Nano)

```bash
# 设置locale
sudo apt update && sudo apt install locales
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# 添加ROS2 APT源
sudo apt install software-properties-common
sudo add-apt-repository universe
sudo apt update && sudo apt install curl -y

sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg

echo "deb [arch=$(dpkg --print-architecture) \
signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] \
http://packages.ros.org/ros2/ubuntu \
$(. /etc/os-release && echo $UBUNTU_CODENAME) main" | \
sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# 安装ROS2 Humble
sudo apt update
sudo apt upgrade
sudo apt install ros-humble-desktop
sudo apt install ros-humble-ros-base

# 安装开发工具
sudo apt install python3-colcon-common-extensions
sudo apt install python3-rosdep python3-vcstool
```

### 2. 安装RealSense ROS2包装器

```bash
# 安装librealsense2
sudo apt install ros-humble-librealsense2*
sudo apt install ros-humble-realsense2-camera
sudo apt install ros-humble-realsense2-description
```

### 3. 安装视觉相关ROS2包

```bash
# 安装vision_msgs
sudo apt install ros-humble-vision-msgs

# 安装图像处理工具
sudo apt install ros-humble-cv-bridge
sudo apt install ros-humble-image-transport
sudo apt install ros-humble-image-transport-plugins
```

### 4. 安装Python依赖

```bash
# 创建虚拟环境（可选但推荐）
python3 -m pip install --upgrade pip

# 安装YOLOv8和相关依赖
pip3 install ultralytics>=8.0.0
pip3 install torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu118
pip3 install opencv-python>=4.5.0
pip3 install numpy>=1.21.0
pip3 install pyrealsense2>=2.50.0
```

### 5. 环境配置

在 `~/.bashrc` 中添加：

```bash
# ROS2 Humble
source /opt/ros/humble/setup.bash

# 自定义工作空间（后续创建）
source ~/ros2_ws/install/setup.bash

# 设置ROS_DOMAIN_ID（避免多机干扰）
export ROS_DOMAIN_ID=42

# CUDA路径（Jetson）
export PATH=/usr/local/cuda/bin:$PATH
export LD_LIBRARY_PATH=/usr/local/cuda/lib64:$LD_LIBRARY_PATH
```

应用配置：
```bash
source ~/.bashrc
```

---

## ROS2 Package创建

### 1. 创建工作空间

```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

### 2. 创建主检测包

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_python \
    --dependencies rclpy sensor_msgs vision_msgs cv_bridge \
    --node-name detector_node \
    vision_detector
```

### 3. 创建自定义消息包

```bash
cd ~/ros2_ws/src
ros2 pkg create --build-type ament_cmake vision_msgs_custom
```

### 4. 最终目录结构

```
~/ros2_ws/
├── src/
│   ├── vision_detector/              # 主检测包
│   │   ├── vision_detector/
│   │   │   ├── __init__.py
│   │   │   ├── detector_node.py      # 检测节点
│   │   │   ├── yolov8_detector.py    # YOLOv8封装
│   │   │   └── utils.py              # 工具函数
│   │   ├── launch/
│   │   │   ├── detector.launch.py    # 启动文件
│   │   │   └── detector_rviz.launch.py
│   │   ├── config/
│   │   │   ├── detector_params.yaml  # 参数配置
│   │   │   └── camera.yaml
│   │   ├── rviz/
│   │   │   └── detector_view.rviz
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   └── vision_msgs_custom/           # 自定义消息包
│       ├── msg/
│       │   ├── Detection2DExtended.msg
│       │   └── ObjectDistance.msg
│       ├── CMakeLists.txt
│       └── package.xml
│
├── build/
├── install/
└── log/
```

---

## 自定义消息定义

### 1. Detection2DExtended.msg

创建 `~/ros2_ws/src/vision_msgs_custom/msg/Detection2DExtended.msg`:

```
# 扩展的2D检测消息，包含深度信息

std_msgs/Header header

# 2D边界框
vision_msgs/BoundingBox2D bbox

# 检测结果
vision_msgs/ObjectHypothesisWithPose[] results

# 深度信息（米）
float32 distance

# 置信度
float32 confidence

# 类别ID和名称
int32 class_id
string class_name

# 3D位置（相机坐标系）
geometry_msgs/Point position_3d
```

### 2. ObjectDistance.msg

创建 `~/ros2_ws/src/vision_msgs_custom/msg/ObjectDistance.msg`:

```
# 目标距离信息

std_msgs/Header header

# 目标ID
int32 tracking_id

# 类别
string class_name

# 距离（米）
float32 distance

# 方位角（弧度，相对于相机中心）
float32 azimuth

# 俯仰角（弧度）
float32 elevation

# 置信度
float32 confidence
```

### 3. 配置CMakeLists.txt

编辑 `~/ros2_ws/src/vision_msgs_custom/CMakeLists.txt`:

```cmake
cmake_minimum_required(VERSION 3.8)
project(vision_msgs_custom)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# 依赖
find_package(ament_cmake REQUIRED)
find_package(std_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(vision_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)

# 生成消息
rosidl_generate_interfaces(${PROJECT_NAME}
  "msg/Detection2DExtended.msg"
  "msg/ObjectDistance.msg"
  DEPENDENCIES std_msgs geometry_msgs vision_msgs
)

ament_package()
```

### 4. 配置package.xml

编辑 `~/ros2_ws/src/vision_msgs_custom/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vision_msgs_custom</name>
  <version>1.0.0</version>
  <description>Custom vision messages for YOLOv8-D435i integration</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <buildtool_depend>ament_cmake</buildtool_depend>
  <buildtool_depend>rosidl_default_generators</buildtool_depend>

  <depend>std_msgs</depend>
  <depend>geometry_msgs</depend>
  <depend>vision_msgs</depend>

  <exec_depend>rosidl_default_runtime</exec_depend>

  <member_of_group>rosidl_interface_packages</member_of_group>

  <export>
    <build_type>ament_cmake</build_type>
  </export>
</package>
```

---

## 核心节点实现

### 1. YOLOv8检测器封装类

创建 `~/ros2_ws/src/vision_detector/vision_detector/yolov8_detector.py`:

```python
#!/usr/bin/env python3
"""
YOLOv8检测器封装类
"""

import numpy as np
from ultralytics import YOLO
import cv2
from typing import List, Tuple, Optional


class YOLOv8Detector:
    """YOLOv8检测器封装"""
    
    def __init__(self, 
                 model_path: str = 'yolov8n.pt',
                 conf_threshold: float = 0.5,
                 iou_threshold: float = 0.45,
                 device: str = 'cuda'):
        """
        初始化YOLOv8检测器
        
        Args:
            model_path: 模型权重路径
            conf_threshold: 置信度阈值
            iou_threshold: NMS的IoU阈值
            device: 推理设备 ('cuda' or 'cpu')
        """
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        
        # 预热模型
        dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.model(dummy_img, verbose=False, device=self.device)
        
    def detect(self, image: np.ndarray) -> List[dict]:
        """
        执行目标检测
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            检测结果列表，每个元素包含:
                - bbox: [x1, y1, x2, y2]
                - confidence: float
                - class_id: int
                - class_name: str
        """
        results = self.model(
            image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            verbose=False,
            device=self.device
        )[0]
        
        detections = []
        
        if results.boxes is not None and len(results.boxes) > 0:
            boxes = results.boxes
            
            for i in range(len(boxes)):
                bbox = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                cls_name = results.names[cls_id]
                
                detections.append({
                    'bbox': bbox.tolist(),
                    'confidence': conf,
                    'class_id': cls_id,
                    'class_name': cls_name
                })
        
        return detections
    
    def visualize(self, 
                  image: np.ndarray, 
                  detections: List[dict],
                  distances: Optional[List[float]] = None) -> np.ndarray:
        """
        可视化检测结果
        
        Args:
            image: 原始图像
            detections: 检测结果
            distances: 可选的距离信息
            
        Returns:
            绘制了检测框的图像
        """
        vis_img = image.copy()
        
        for i, det in enumerate(detections):
            x1, y1, x2, y2 = map(int, det['bbox'])
            conf = det['confidence']
            cls_name = det['class_name']
            
            # 绘制边界框
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 准备标签
            if distances is not None and i < len(distances):
                label = f"{cls_name} {conf:.2f} {distances[i]:.2f}m"
            else:
                label = f"{cls_name} {conf:.2f}"
            
            # 绘制标签背景
            (label_w, label_h), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
            )
            cv2.rectangle(vis_img, (x1, y1 - label_h - 10), 
                         (x1 + label_w, y1), (0, 255, 0), -1)
            
            # 绘制标签文字
            cv2.putText(vis_img, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return vis_img
```

### 2. 深度处理工具函数

创建 `~/ros2_ws/src/vision_detector/vision_detector/utils.py`:

```python
#!/usr/bin/env python3
"""
深度处理和坐标转换工具函数
"""

import numpy as np
import random
from typing import Tuple, Optional


def get_object_distance(bbox: list, 
                       depth_image: np.ndarray,
                       sample_points: int = 24) -> float:
    """
    使用随机采样+中值滤波获取目标距离
    
    Args:
        bbox: [x1, y1, x2, y2] 边界框
        depth_image: 深度图像 (单位: 毫米)
        sample_points: 采样点数量
        
    Returns:
        距离（米），如果无效则返回0.0
    """
    x1, y1, x2, y2 = map(int, bbox)
    
    # 计算中心点
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    
    # 计算采样范围
    width = abs(x2 - x1)
    height = abs(y2 - y1)
    min_dim = min(width, height)
    sample_range = min_dim // 4
    
    # 随机采样
    distance_list = []
    for _ in range(sample_points):
        offset_x = random.randint(-sample_range, sample_range)
        offset_y = random.randint(-sample_range, sample_range)
        
        sample_x = np.clip(cx + offset_x, 0, depth_image.shape[1] - 1)
        sample_y = np.clip(cy + offset_y, 0, depth_image.shape[0] - 1)
        
        depth_value = depth_image[sample_y, sample_x]
        
        if depth_value > 0:  # 有效深度值
            distance_list.append(depth_value)
    
    if len(distance_list) == 0:
        return 0.0
    
    # 中值滤波
    distance_array = np.array(distance_list)
    distance_array = np.sort(distance_array)
    
    # 取中间50%的数据
    start_idx = len(distance_array) // 4
    end_idx = start_idx + len(distance_array) // 2
    filtered_distances = distance_array[start_idx:end_idx]
    
    # 返回平均值（转换为米）
    return float(np.mean(filtered_distances)) / 1000.0 if len(filtered_distances) > 0 else 0.0


def pixel_to_3d_point(u: int, v: int, 
                      depth: float,
                      camera_matrix: np.ndarray) -> Tuple[float, float, float]:
    """
    将像素坐标和深度转换为3D坐标（相机坐标系）
    
    Args:
        u, v: 像素坐标
        depth: 深度值（米）
        camera_matrix: 3x3相机内参矩阵
        
    Returns:
        (x, y, z) 3D坐标（米）
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
    return x, y, z


def calculate_azimuth_elevation(x: float, y: float, z: float) -> Tuple[float, float]:
    """
    计算方位角和俯仰角
    
    Args:
        x, y, z: 3D坐标（相机坐标系）
        
    Returns:
        (azimuth, elevation) 方位角和俯仰角（弧度）
    """
    # 方位角（水平角度）
    azimuth = np.arctan2(x, z)
    
    # 俯仰角（垂直角度）
    horizontal_dist = np.sqrt(x**2 + z**2)
    elevation = np.arctan2(-y, horizontal_dist)
    
    return azimuth, elevation
```

### 3. 主检测节点

创建 `~/ros2_ws/src/vision_detector/vision_detector/detector_node.py`:

```python
#!/usr/bin/env python3
"""
YOLOv8-D435i ROS2检测节点
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D, Pose2D
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import Optional

from .yolov8_detector import YOLOv8Detector
from .utils import get_object_distance, pixel_to_3d_point, calculate_azimuth_elevation


class VisionDetectorNode(Node):
    """视觉检测ROS2节点"""
    
    def __init__(self):
        super().__init__('vision_detector_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', 'yolov8n.pt'),
                ('conf_threshold', 0.5),
                ('iou_threshold', 0.45),
                ('device', 'cuda'),
                ('publish_visualization', True),
                ('depth_sample_points', 24),
                ('camera_topic', '/camera/color/image_raw'),
                ('depth_topic', '/camera/depth/image_rect_raw'),
                ('camera_info_topic', '/camera/color/camera_info'),
            ]
        )
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('conf_threshold').value
        iou_threshold = self.get_parameter('iou_threshold').value
        device = self.get_parameter('device').value
        self.publish_vis = self.get_parameter('publish_visualization').value
        self.depth_sample_points = self.get_parameter('depth_sample_points').value
        
        # 初始化检测器
        self.get_logger().info(f'Initializing YOLOv8 detector with model: {model_path}')
        self.detector = YOLOv8Detector(
            model_path=model_path,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            device=device
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 相机内参（延迟初始化）
        self.camera_matrix: Optional[np.ndarray] = None
        
        # 缓存深度图像
        self.latest_depth_image: Optional[np.ndarray] = None
        
        # 订阅器
        camera_topic = self.get_parameter('camera_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        
        self.color_sub = self.create_subscription(
            Image,
            camera_topic,
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # 发布器
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        if self.publish_vis:
            self.vis_pub = self.create_publisher(
                Image,
                '/detection_image',
                10
            )
        
        # 统计信息
        self.frame_count = 0
        self.detection_count = 0
        
        # 定时器：打印统计信息
        self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('Vision Detector Node initialized successfully')
    
    def camera_info_callback(self, msg: CameraInfo):
        """接收相机内参"""
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = K
            self.get_logger().info(f'Received camera intrinsics:\n{K}')
    
    def depth_callback(self, msg: Image):
        """接收深度图像"""
        try:
            # 转换为numpy数组（单位：毫米）
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = depth_image
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
    
    def color_callback(self, msg: Image):
        """接收彩色图像并执行检测"""
        try:
            # 转换图像
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 执行检测
            detections = self.detector.detect(color_image)
            
            # 计算距离
            distances = []
            if self.latest_depth_image is not None:
                for det in detections:
                    dist = get_object_distance(
                        det['bbox'],
                        self.latest_depth_image,
                        self.depth_sample_points
                    )
                    distances.append(dist)
            
            # 发布检测结果
            self.publish_detections(msg.header, detections, distances)
            
            # 发布可视化
            if self.publish_vis:
                vis_image = self.detector.visualize(color_image, detections, distances)
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
                vis_msg.header = msg.header
                self.vis_pub.publish(vis_msg)
            
            # 更新统计
            self.frame_count += 1
            self.detection_count += len(detections)
            
        except Exception as e:
            self.get_logger().error(f'Error in color_callback: {e}')
    
    def publish_detections(self, 
                          header: Header,
                          detections: list,
                          distances: list):
        """发布检测结果"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        for i, det in enumerate(detections):
            detection_msg = Detection2D()
            detection_msg.header = header
            
            # 边界框
            bbox = BoundingBox2D()
            x1, y1, x2, y2 = det['bbox']
            bbox.center = Pose2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            detection_msg.bbox = bbox
            
            # 检测结果
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score = det['confidence']
            detection_msg.results.append(hypothesis)
            
            # 源ID（可选）
            detection_msg.source_img = header
            
            detection_array.detections.append(detection_msg)
        
        self.detection_pub.publish(detection_array)
    
    def print_stats(self):
        """打印统计信息"""
        if self.frame_count > 0:
            avg_detections = self.detection_count / self.frame_count
            self.get_logger().info(
                f'Stats - Frames: {self.frame_count}, '
                f'Total Detections: {self.detection_count}, '
                f'Avg: {avg_detections:.2f} det/frame'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectorNode()
    
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

### 4. 配置setup.py

编辑 `~/ros2_ws/src/vision_detector/setup.py`:

```python
from setuptools import setup
import os
from glob import glob

package_name = 'vision_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), 
            glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), 
            glob('config/*.yaml')),
        (os.path.join('share', package_name, 'rviz'), 
            glob('rviz/*.rviz')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='YOLOv8-D435i ROS2 Vision Detector',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'detector_node = vision_detector.detector_node:main',
        ],
    },
)
```

### 5. 配置package.xml

编辑 `~/ros2_ws/src/vision_detector/package.xml`:

```xml
<?xml version="1.0"?>
<?xml-model href="http://download.ros.org/schema/package_format3.xsd" schematypens="http://www.w3.org/2001/XMLSchema"?>
<package format="3">
  <name>vision_detector</name>
  <version>1.0.0</version>
  <description>YOLOv8-D435i ROS2 Vision Detection Package</description>
  <maintainer email="your_email@example.com">Your Name</maintainer>
  <license>Apache-2.0</license>

  <depend>rclpy</depend>
  <depend>sensor_msgs</depend>
  <depend>vision_msgs</depend>
  <depend>geometry_msgs</depend>
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

---

## Launch文件配置

### 1. 主启动文件

创建 `~/ros2_ws/src/vision_detector/launch/detector.launch.py`:

```python
#!/usr/bin/env python3
"""
YOLOv8-D435i检测系统启动文件
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 参数声明
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='yolov8n.pt',
        description='Path to YOLOv8 model weights'
    )
    
    conf_threshold_arg = DeclareLaunchArgument(
        'conf_threshold',
        default_value='0.5',
        description='Confidence threshold for detection'
    )
    
    device_arg = DeclareLaunchArgument(
        'device',
        default_value='cuda',
        description='Device for inference (cuda or cpu)'
    )
    
    # RealSense相机启动
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('realsense2_camera'),
                'launch',
                'rs_launch.py'
            ])
        ]),
        launch_arguments={
            'align_depth.enable': 'true',
            'enable_color': 'true',
            'enable_depth': 'true',
            'depth_module.profile': '640x480x30',
            'rgb_camera.profile': '640x480x30',
        }.items()
    )
    
    # 检测节点
    detector_node = Node(
        package='vision_detector',
        executable='detector_node',
        name='vision_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': LaunchConfiguration('conf_threshold'),
            'device': LaunchConfiguration('device'),
            'publish_visualization': True,
            'depth_sample_points': 24,
            'camera_topic': '/camera/camera/color/image_raw',
            'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
            'camera_info_topic': '/camera/camera/color/camera_info',
        }],
        remappings=[
            ('/camera/color/image_raw', '/camera/camera/color/image_raw'),
            ('/camera/depth/image_rect_raw', '/camera/camera/aligned_depth_to_color/image_raw'),
        ]
    )
    
    return LaunchDescription([
        model_path_arg,
        conf_threshold_arg,
        device_arg,
        realsense_launch,
        detector_node,
    ])
```

### 2. RViz可视化启动文件

创建 `~/ros2_ws/src/vision_detector/launch/detector_rviz.launch.py`:

```python
#!/usr/bin/env python3
"""
带RViz可视化的启动文件
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 包含主检测启动文件
    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vision_detector'),
                'launch',
                'detector.launch.py'
            ])
        ])
    )
    
    # RViz节点
    rviz_config = PathJoinSubstitution([
        FindPackageShare('vision_detector'),
        'rviz',
        'detector_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        detector_launch,
        rviz_node,
    ])
```

### 3. 参数配置文件

创建 `~/ros2_ws/src/vision_detector/config/detector_params.yaml`:

```yaml
vision_detector:
  ros__parameters:
    # 模型配置
    model_path: "yolov8n.pt"
    conf_threshold: 0.5
    iou_threshold: 0.45
    device: "cuda"
    
    # 功能开关
    publish_visualization: true
    
    # 深度处理
    depth_sample_points: 24
    
    # Topic配置
    camera_topic: "/camera/camera/color/image_raw"
    depth_topic: "/camera/camera/aligned_depth_to_color/image_raw"
    camera_info_topic: "/camera/camera/color/camera_info"
```

### 4. RViz配置文件

创建 `~/ros2_ws/src/vision_detector/rviz/detector_view.rviz`:

```yaml
Panels:
  - Class: rviz_common/Displays
    Name: Displays
  - Class: rviz_common/Views
    Name: Views

Visualization Manager:
  Displays:
    - Class: rviz_default_plugins/Image
      Name: RGB Image
      Topic:
        Value: /camera/camera/color/image_raw
      
    - Class: rviz_default_plugins/Image
      Name: Detection Result
      Topic:
        Value: /detection_image
      
    - Class: rviz_default_plugins/Image
      Name: Depth Image
      Topic:
        Value: /camera/camera/aligned_depth_to_color/image_raw

  Global Options:
    Fixed Frame: camera_link

  Views:
    Current:
      Class: rviz_default_plugins/Orbit
```

---

## 编译与部署

### 1. 编译消息包

```bash
cd ~/ros2_ws

# 先编译自定义消息包
colcon build --packages-select vision_msgs_custom

# source新消息
source install/setup.bash
```

### 2. 编译检测包

```bash
cd ~/ros2_ws

# 编译主检测包
colcon build --packages-select vision_detector

# source
source install/setup.bash
```

### 3. 完整编译（推荐）

```bash
cd ~/ros2_ws

# 清理之前的编译
rm -rf build/ install/ log/

# 完整编译
colcon build --symlink-install

# source
source install/setup.bash
```

### 4. 下载模型权重

```bash
cd ~/ros2_ws/src/vision_detector

# 创建weights目录
mkdir -p weights
cd weights

# 下载YOLOv8n权重（~6MB）
wget https://github.com/ultralytics/assets/releases/download/v0.0.0/yolov8n.pt

# 或者使用Python下载
python3 -c "from ultralytics import YOLO; YOLO('yolov8n.pt')"
```

---

## 测试与验证

### 1. 测试RealSense相机

```bash
# 启动RealSense查看器
realsense-viewer

# 或使用ROS2命令
ros2 launch realsense2_camera rs_launch.py

# 查看topic列表
ros2 topic list

# 查看图像（使用rqt）
rqt_image_view
```

### 2. 测试检测节点（不带相机）

创建测试脚本 `~/ros2_ws/test_detector.py`:

```bash
#!/usr/bin/env python3
import cv2
import numpy as np
from vision_detector.yolov8_detector import YOLOv8Detector

# 初始化检测器
detector = YOLOv8Detector('yolov8n.pt')

# 读取测试图像
test_image = cv2.imread('test.jpg')

# 执行检测
detections = detector.detect(test_image)

# 可视化
vis_image = detector.visualize(test_image, detections)

cv2.imshow('Detection', vis_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(f"Detected {len(detections)} objects:")
for det in detections:
    print(f"  {det['class_name']}: {det['confidence']:.2f}")
```

运行测试：
```bash
python3 ~/ros2_ws/test_detector.py
```

### 3. 启动完整系统

```bash
# 方式1：使用launch文件
ros2 launch vision_detector detector.launch.py

# 方式2：带RViz
ros2 launch vision_detector detector_rviz.launch.py

# 方式3：自定义参数
ros2 launch vision_detector detector.launch.py \
    model_path:=yolov8s.pt \
    conf_threshold:=0.6 \
    device:=cuda
```

### 4. 查看检测结果

```bash
# 查看topic
ros2 topic list

# 监听检测结果
ros2 topic echo /detections

# 查看发布频率
ros2 topic hz /detections

# 查看消息详情
ros2 topic info /detections
```

### 5. 录制数据包（调试用）

```bash
# 录制所有相关topic
ros2 bag record -o detector_test \
    /camera/camera/color/image_raw \
    /camera/camera/aligned_depth_to_color/image_raw \
    /detections \
    /detection_image

# 回放
ros2 bag play detector_test
```

### 6. 性能测试

创建性能测试脚本 `~/ros2_ws/benchmark_node.py`:

```python
#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from vision_msgs.msg import Detection2DArray
import time

class BenchmarkNode(Node):
    def __init__(self):
        super().__init__('benchmark_node')
        self.detection_times = []
        self.last_image_time = None
        
        self.image_sub = self.create_subscription(
            Image, '/camera/camera/color/image_raw',
            self.image_callback, 10
        )
        
        self.detection_sub = self.create_subscription(
            Detection2DArray, '/detections',
            self.detection_callback, 10
        )
        
        self.create_timer(5.0, self.print_stats)
    
    def image_callback(self, msg):
        self.last_image_time = time.time()
    
    def detection_callback(self, msg):
        if self.last_image_time is not None:
            latency = time.time() - self.last_image_time
            self.detection_times.append(latency)
    
    def print_stats(self):
        if len(self.detection_times) > 0:
            avg_latency = sum(self.detection_times) / len(self.detection_times)
            max_latency = max(self.detection_times)
            fps = 1.0 / avg_latency if avg_latency > 0 else 0
            
            self.get_logger().info(
                f'Performance Stats:\n'
                f'  Average Latency: {avg_latency*1000:.2f}ms\n'
                f'  Max Latency: {max_latency*1000:.2f}ms\n'
                f'  Effective FPS: {fps:.2f}'
            )
            
            self.detection_times = []

def main():
    rclpy.init()
    node = BenchmarkNode()
    rclpy.spin(node)

if __name__ == '__main__':
    main()
```

运行性能测试：
```bash
python3 ~/ros2_ws/benchmark_node.py
```

---

## 性能优化

### 1. Jetson Orin Nano优化

#### 设置最大性能模式

```bash
# 查看当前模式
sudo nvpmodel -q

# 设置为最大性能模式（MODE 0）
sudo nvpmodel -m 0

# 设置风扇为最大转速
sudo jetson_clocks

# 查看功耗和频率
sudo tegrastats
```

#### TensorRT加速

修改检测器以使用TensorRT：

```python
# yolov8_detector.py 中添加
def export_to_tensorrt(self, engine_path='yolov8n.engine'):
    """导出为TensorRT引擎"""
    self.model.export(format='engine', device='0')
    
# 使用TensorRT引擎
model = YOLO('yolov8n.engine')
```

导出命令：
```bash
cd ~/ros2_ws/src/vision_detector/weights
yolo export model=yolov8n.pt format=engine device=0
```

### 2. 内存优化

在节点中添加：

```python
# detector_node.py 开头添加
import gc
import torch

# 在__init__中添加
torch.cuda.empty_cache()
gc.collect()

# 定期清理
self.create_timer(30.0, self.cleanup_memory)

def cleanup_memory(self):
    """定期清理内存"""
    torch.cuda.empty_cache()
    gc.collect()
```

### 3. 多线程处理

```python
from threading import Thread
from queue import Queue

class VisionDetectorNode(Node):
    def __init__(self):
        # ... 现有代码 ...
        
        # 添加处理队列
        self.image_queue = Queue(maxsize=2)
        
        # 启动检测线程
        self.detection_thread = Thread(target=self.detection_worker)
        self.detection_thread.daemon = True
        self.detection_thread.start()
    
    def color_callback(self, msg):
        """快速接收图像，放入队列"""
        if not self.image_queue.full():
            self.image_queue.put(msg)
    
    def detection_worker(self):
        """独立线程处理检测"""
        while True:
            msg = self.image_queue.get()
            # 执行检测...
```

### 4. 降低分辨率（如果帧率仍不够）

修改launch文件中的相机配置：

```python
'depth_module.profile': '424x240x30',  # 从640x480降低
'rgb_camera.profile': '424x240x30',
```

### 5. 使用更轻量级模型

```bash
# YOLOv8n (6.5MB) - 当前使用
# YOLOv8-tiny (更小，非官方)
# 或自定义剪枝模型
```

---

## 故障排查

### 问题1: 找不到相机

**症状**：
```
RuntimeError: No RealSense devices were found!
```

**解决方案**：
```bash
# 1. 检查USB连接
lsusb | grep Intel

# 2. 检查权限
sudo chmod 777 /dev/bus/usb/*/*

# 3. 重启udev规则
sudo udevadm control --reload-rules
sudo udevadm trigger

# 4. 重新插拔相机
```

### 问题2: CUDA Out of Memory

**症状**：
```
RuntimeError: CUDA out of memory
```

**解决方案**：
```python
# 方法1: 降低batch size（已经是1了）
# 方法2: 使用CPU推理
device: "cpu"

# 方法3: 使用更小的模型
model_path: "yolov8n.pt"

# 方法4: 清理CUDA缓存
torch.cuda.empty_cache()
```

### 问题3: 检测延迟高

**症状**：FPS < 10

**解决方案**：
```bash
# 1. 确认GPU加速
python3 -c "import torch; print(torch.cuda.is_available())"

# 2. 检查功耗模式
sudo nvpmodel -q

# 3. 启用TensorRT
# （参考性能优化章节）

# 4. 降低图像分辨率
# （修改launch文件）

# 5. 提高置信度阈值（减少检测框数量）
conf_threshold: 0.7
```

### 问题4: 深度值不准确

**症状**：距离测量偏差大

**解决方案**：
```python
# 1. 增加采样点
depth_sample_points: 48

# 2. 检查深度图对齐
# 在launch中确保：
'align_depth.enable': 'true'

# 3. 检查深度范围
# D435i有效范围：0.1m ~ 10m
# 太近或太远的物体深度值不准

# 4. 改进采样策略（在utils.py中）
# 使用网格采样替代随机采样
```

### 问题5: Topic未发布

**症状**：
```bash
ros2 topic list  # 看不到/detections
```

**解决方案**：
```bash
# 1. 检查节点是否运行
ros2 node list

# 2. 检查节点信息
ros2 node info /vision_detector

# 3. 查看日志
ros2 run vision_detector detector_node --ros-args --log-level debug

# 4. 检查依赖
pip3 list | grep ultralytics
```

### 问题6: 编译错误

**症状**：
```
CMake Error / Python ImportError
```

**解决方案**：
```bash
# 1. 清理编译缓存
cd ~/ros2_ws
rm -rf build/ install/ log/

# 2. 确保依赖完整
rosdep install --from-paths src --ignore-src -r -y

# 3. 检查Python路径
which python3
python3 --version  # 应该是3.10+

# 4. 重新source
source /opt/ros/humble/setup.bash

# 5. 重新编译
colcon build --symlink-install
```

---

## 扩展功能建议

### 1. 目标跟踪

添加SORT/DeepSORT跟踪器：

```python
from deep_sort_realtime.deepsort_tracker import DeepSort

class VisionDetectorNode(Node):
    def __init__(self):
        # ... 现有代码 ...
        self.tracker = DeepSort(max_age=30)
    
    def color_callback(self, msg):
        # ... 检测 ...
        
        # 跟踪
        tracks = self.tracker.update_tracks(detections, frame=color_image)
        
        for track in tracks:
            if track.is_confirmed():
                track_id = track.track_id
                bbox = track.to_ltrb()
                # 发布带ID的跟踪结果
```

### 2. TF广播

发布目标的TF坐标变换：

```python
from tf2_ros import TransformBroadcaster
from geometry_msgs.msg import TransformStamped

def publish_object_tf(self, detection, distance, timestamp):
    """发布目标的TF"""
    t = TransformStamped()
    t.header.stamp = timestamp
    t.header.frame_id = 'camera_link'
    t.child_frame_id = f'object_{detection["class_name"]}'
    
    # 计算3D位置
    x, y, z = pixel_to_3d_point(...)
    
    t.transform.translation.x = x
    t.transform.translation.y = y
    t.transform.translation.z = z
    
    self.tf_broadcaster.sendTransform(t)
```

### 3. 动作服务器

提供检测服务（适用于按需检测）：

```python
from vision_msgs.srv import DetectObjects

class VisionDetectorNode(Node):
    def __init__(self):
        # ... 现有代码 ...
        self.detection_service = self.create_service(
            DetectObjects,
            'detect_objects',
            self.detect_service_callback
        )
```

---

## 总结

### 完整启动流程

```bash
# 1. 打开终端1 - 启动系统
cd ~/ros2_ws
source install/setup.bash
ros2 launch vision_detector detector.launch.py
ros2 run rqt_image_view rqt_image_view

# 2. 打开终端2 - 查看结果
ros2 topic echo /detections

# 3. 打开终端3 - 可视化（可选）
ros2 run rqt_image_view rqt_image_view

```

### 关键文件清单

```
~/ros2_ws/src/
├── vision_detector/
│   ├── vision_detector/
│   │   ├── detector_node.py          ✅ 核心节点
│   │   ├── yolov8_detector.py        ✅ 检测器封装
│   │   └── utils.py                  ✅ 工具函数
│   ├── launch/
│   │   └── detector.launch.py        ✅ 启动文件
│   ├── config/
│   │   └── detector_params.yaml      ✅ 参数配置
│   └── weights/
│       └── yolov8n.pt                ✅ 模型权重
│
└── vision_msgs_custom/
    ├── msg/
    │   ├── Detection2DExtended.msg   ✅ 自定义消息
    │   └── ObjectDistance.msg        ✅ 距离消息
    └── CMakeLists.txt                ✅ 消息编译配置
```

### 下一步建议

1. **基础测试**：先用静态图像测试检测器
2. **相机测试**：单独测试RealSense功能
3. **集成测试**：启动完整系统
4. **性能调优**：根据实际FPS调整参数
5. **功能扩展**：添加跟踪、TF广播等高级功能

### 技术支持资源

- **ROS2文档**: https://docs.ros.org/en/humble/
- **YOLOv8文档**: https://docs.ultralytics.com/
- **RealSense SDK**: https://github.com/IntelRealSense/librealsense
- **Vision Messages**: https://github.com/ros-perception/vision_msgs

---

**文档版本**: v1.0  
**适用平台**: Jetson Orin Nano + ROS2 Humble  
**最后更新**: 2026年2月
