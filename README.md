# ROS2 Vision Project - YOLOv8目标检测系统

## 项目概述

本项目是一个基于ROS2的实时目标检测系统，集成了YOLOv8深度学习模型和Intel RealSense D435i深度相机，实现了高效的目标检测、距离测量和坐标转换功能。该系统适用于机器人视觉导航、物体识别、智能监控等应用场景。

## 主要功能

1. **实时目标检测**：使用YOLOv8模型进行高精度的目标检测
2. **深度信息获取**：通过RealSense D435i相机获取深度信息
3. **距离测量**：基于深度图像计算目标物体的距离
4. **坐标转换**：将2D像素坐标转换为3D相机坐标系坐标
5. **角度计算**：计算目标相对于相机的方位角和俯仰角
6. **可视化显示**：实时显示检测结果和深度信息

## 项目结构

```
ros2_vision_project/
├── docker/                          # Docker配置文件
├── ros2_ws/                         # ROS2工作空间
│   ├── src/
│   │   ├── vision_detector/         # 视觉检测功能包
│   │   │   ├── launch/              # 启动文件
│   │   │   │   ├── detector.launch.py          # 检测系统启动文件
│   │   │   │   └── detector_rviz.launch.py     # RViz可视化启动文件
│   │   │   ├── vision_detector/    # Python源代码
│   │   │   │   ├── detector_node.py           # 主检测节点
│   │   │   │   ├── yolov8_detector.py         # YOLOv8检测器封装
│   │   │   │   └── utils.py                  # 工具函数
│   │   │   ├── weights/             # 模型权重文件
│   │   │   │   └── yolov8n.pt      # YOLOv8 Nano模型
│   │   │   ├── package.xml          # 功能包描述文件
│   │   │   ├── setup.py            # Python包设置
│   │   │   └── CMakeLists.txt      # CMake构建配置
│   │   └── vision_msgs_custom/     # 自定义消息定义
│   │       ├── msg/
│   │       │   ├── Detection2DExtended.msg   # 扩展检测消息
│   │       │   └── ObjectDistance.msg        # 物体距离消息
│   │       ├── CMakeLists.txt
│   │       └── package.xml
│   └── install/                    # 构建输出目录
└── scripts/                         # 部署和构建脚本
    ├── build_and_deploy.sh         # 构建和部署脚本
    ├── setup_dev_env.sh            # 开发环境设置脚本
    └── sync_to_jetson.sh           # Jetson设备同步脚本
```

## 核心模块详解

### 1. 检测器节点 (detector_node.py)

`detector_node.py`是整个系统的核心节点，负责协调图像采集、目标检测、深度处理和结果发布。

#### 主要功能

- 订阅彩色图像、深度图像和相机信息
- 调用YOLOv8模型进行目标检测
- 计算目标距离并发布检测结果
- 生成可视化图像

#### 关键代码片段

```python
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

        # 初始化检测器
        self.detector = YOLOv8Detector(
            model_path=model_path,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            device=device
        )

        # 订阅相机数据
        self.color_sub = self.create_subscription(
            Image, camera_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # 发布检测结果
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
```

#### 图像处理流程

```python
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

    except Exception as e:
        self.get_logger().error(f'Error in color_callback: {e}')
```

### 2. YOLOv8检测器封装 (yolov8_detector.py)

该模块封装了YOLOv8检测器的核心功能，提供简洁的接口供主节点调用。

#### 主要功能

- 加载和初始化YOLOv8模型
- 执行目标检测
- 生成检测结果可视化

#### 关键代码片段

```python
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
```

### 3. 工具函数模块 (utils.py)

该模块提供了深度处理和坐标转换的核心算法。

#### 主要功能

- 目标距离计算（基于深度图像）
- 像素坐标到3D坐标转换
- 方位角和俯仰角计算

#### 关键代码片段

```python
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

### 4. 启动文件 (detector.launch.py)

该启动文件用于启动整个检测系统，包括RealSense相机和检测节点。

#### 关键代码片段

```python
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

## 检测流程详解

本项目的检测流程分为两个主要部分：YOLO模型检测和ROS2检测结果发布。下面详细说明这两个部分的工作流程和关键代码实现。

### 1. YOLO模型检测流程

YOLO模型检测由`yolov8_detector.py`模块实现，主要包含以下步骤：

#### 1.1 模型初始化

```python
class YOLOv8Detector:
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
```

#### 1.2 目标检测执行

```python
def detect(self, image: np.ndarray) -> List[dict]:
    """
    执行目标检测

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
```

### 2. ROS2检测结果发布流程

ROS2检测结果发布由`detector_node.py`模块实现，主要包含以下步骤：

#### 2.1 图像处理与检测

```python
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

    except Exception as e:
        self.get_logger().error(f'Error in color_callback: {e}')
```

#### 2.2 检测结果发布

```python
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

        detection_array.detections.append(detection_msg)

    self.detection_pub.publish(detection_array)
```

#### 2.3 可视化结果发布

```python
def visualize(self,
              image: np.ndarray,
              detections: List[dict],
              distances: Optional[List[float]] = None) -> np.ndarray:
    """
    可视化检测结果

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

### 3. 检测流程图

```
彩色图像输入
    ↓
图像格式转换
    ↓
YOLOv8模型检测
    ↓
检测结果提取
    ↓
深度信息处理
    ↓
距离计算
    ↓
ROS2消息构建
    ↓
检测结果发布
    ↓
可视化图像生成
    ↓
可视化图像发布
```

### 4. 关键数据流

#### 4.1 检测结果数据结构

```python
# YOLOv8检测结果
detection = {
    'bbox': [x1, y1, x2, y2],      # 边界框坐标
    'confidence': 0.95,              # 置信度
    'class_id': 0,                   # 类别ID
    'class_name': 'person'           # 类别名称
}
```

#### 4.2 ROS2消息结构

```python
# Detection2DArray消息
detection_array = {
    'header': {...},                 # 消息头
    'detections': [                  # 检测结果数组
        {
            'header': {...},
            'results': [
                {
                    'hypothesis': {
                        'class_id': '0',
                        'score': 0.95
                    }
                }
            ],
            'bbox': {
                'center': {
                    'position': {
                        'x': 320.0,
                        'y': 240.0
                    }
                },
                'size_x': 100.0,
                'size_y': 200.0
            }
        }
    ]
}
```

### 5. ROS2坐标发布模块

ROS2坐标发布模块负责将2D像素坐标转换为3D相机坐标系坐标，并计算目标相对于相机的方位角和俯仰角。该模块主要在`utils.py`中实现。

#### 5.1 坐标转换功能

```python
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
```

#### 5.2 角度计算功能

```python
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

#### 5.3 距离测量功能

```python
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
```

#### 5.4 坐标系统说明

项目使用以下坐标系统：

1. **像素坐标系**：
   - 原点：图像左上角
   - 单位：像素
   - 轴方向：x轴向右，y轴向下

2. **相机坐标系**：
   - 原点：相机光心
   - 单位：米
   - 轴方向：z轴向前（光轴方向），x轴向右，y轴向下

3. **角度系统**：
   - 方位角（azimuth）：水平方向角度，以z轴为0度，向右为正
   - 俯仰角（elevation）：垂直方向角度，以水平面为0度，向下为正

#### 5.5 坐标发布流程

```
像素坐标获取
    ↓
深度信息提取
    ↓
相机内参应用
    ↓
3D坐标转换
    ↓
角度计算
    ↓
ROS2消息构建
    ↓
坐标信息发布
```

#### 5.6 坐标转换示例

```python
# 假设检测到目标在像素坐标(320, 240)，深度为2.0米
u, v = 320, 240
depth = 2.0

# 相机内参（示例值）
camera_matrix = np.array([
    [615.0, 0.0, 320.0],
    [0.0, 615.0, 240.0],
    [0.0, 0.0, 1.0]
])

# 转换为3D坐标
x, y, z = pixel_to_3d_point(u, v, depth, camera_matrix)
# 结果: x≈0.0, y≈0.0, z=2.0

# 计算角度
azimuth, elevation = calculate_azimuth_elevation(x, y, z)
# 结果: azimuth≈0.0, elevation≈0.0
```

### 6. 检测结果发布模块

检测结果发布模块负责将YOLOv8模型的检测结果转换为ROS2消息格式，并发布到相应的主题上。该模块主要在`detector_node.py`中实现。

#### 6.1 发布器初始化

```python
# 发布检测结果
self.detection_pub = self.create_publisher(
    Detection2DArray, '/detections', 10)

# 发布可视化图像
if self.publish_vis:
    self.vis_pub = self.create_publisher(
        Image, '/detection_image', 10)
```

#### 6.2 检测结果发布实现

```python
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

        detection_array.detections.append(detection_msg)

    self.detection_pub.publish(detection_array)
```

#### 6.3 可视化图像发布实现

```python
# 发布可视化
if self.publish_vis:
    vis_image = self.detector.visualize(color_image, detections, distances)
    vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
    vis_msg.header = msg.header
    self.vis_pub.publish(vis_msg)
```

#### 6.4 消息格式说明

##### 6.4.1 Detection2DArray消息

```python
# Detection2DArray消息结构
detection_array = {
    'header': {
        'stamp': 时间戳,
        'frame_id': 'camera_color_optical_frame'
    },
    'detections': [
        {
            'header': {...},
            'results': [
                {
                    'hypothesis': {
                        'class_id': '0',
                        'score': 0.95
                    }
                }
            ],
            'bbox': {
                'center': {
                    'position': {
                        'x': 320.0,
                        'y': 240.0
                    }
                },
                'size_x': 100.0,
                'size_y': 200.0
            }
        }
    ]
}
```

##### 6.4.2 可视化图像消息

```python
# 可视化图像消息结构
vis_msg = {
    'header': {
        'stamp': 时间戳,
        'frame_id': 'camera_color_optical_frame'
    },
    'height': 480,
    'width': 640,
    'encoding': 'bgr8',
    'data': 图像数据
}
```

#### 6.5 发布流程图

```
检测结果获取
    ↓
消息头设置
    ↓
边界框信息提取
    ↓
检测结果信息提取
    ↓
ROS2消息构建
    ↓
消息发布到/detections主题
    ↓
可视化图像生成
    ↓
可视化图像发布到/detection_image主题
```

#### 6.6 主题订阅示例

```python
# 订阅检测结果
from vision_msgs.msg import Detection2DArray

class DetectionSubscriber(Node):
    def __init__(self):
        super().__init__('detection_subscriber')
        self.detection_sub = self.create_subscription(
            Detection2DArray,
            '/detections',
            self.detection_callback,
            10
        )

    def detection_callback(self, msg):
        for detection in msg.detections:
            bbox = detection.bbox
            center_x = bbox.center.position.x
            center_y = bbox.center.position.y
            size_x = bbox.size_x
            size_y = bbox.size_y

            for result in detection.results:
                class_id = result.hypothesis.class_id
                score = result.hypothesis.score

                self.get_logger().info(
                    f'Detected: class={class_id}, '
                    f'score={score:.2f}, '
                    f'center=({center_x:.1f}, {center_y:.1f}), '
                    f'size=({size_x:.1f}x{size_y:.1f})'
                )

# 订阅可视化图像
from sensor_msgs.msg import Image

class VisualizationSubscriber(Node):
    def __init__(self):
        super().__init__('visualization_subscriber')
        self.vis_sub = self.create_subscription(
            Image,
            '/detection_image',
            self.vis_callback,
            10
        )
        self.bridge = CvBridge()

    def vis_callback(self, msg):
        cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        cv2.imshow('Detection Visualization', cv_image)
        cv2.waitKey(1)
```

#### 6.7 性能优化建议

1. **消息发布频率控制**：
   - 根据实际需求调整发布频率
   - 可以使用节流机制限制发布频率

2. **消息大小优化**：
   - 避免发布不必要的详细信息
   - 考虑使用压缩图像格式

3. **队列深度设置**：
   - 根据处理速度调整队列深度
   - 避免消息积压导致延迟

## ROS2主题接口

### 订阅主题

| 主题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/camera/color/image_raw` | sensor_msgs/Image | 彩色图像 |
| `/camera/depth/image_rect_raw` | sensor_msgs/Image | 深度图像 |
| `/camera/color/camera_info` | sensor_msgs/CameraInfo | 相机内参信息 |

### 发布主题

| 主题名称 | 消息类型 | 描述 |
|---------|---------|------|
| `/detections` | vision_msgs/Detection2DArray | 目标检测结果 |
| `/detection_image` | sensor_msgs/Image | 可视化检测图像 |

## 检测结果消息格式

系统使用标准的`vision_msgs/Detection2DArray`消息发布检测结果，每个检测包含：

- **边界框信息**：
  - 中心点坐标 (x, y)
  - 边界框尺寸 (size_x, size_y)

- **检测结果**：
  - 类别ID (class_id)
  - 置信度 (score)
  - 类别名称 (class_name)

## 安装与使用

### 环境要求

- ROS2 Humble
- Python 3.8+
- CUDA (可选，用于GPU加速)
- Intel RealSense D435i相机

### 安装步骤

1. 克隆项目到工作空间：
```bash
cd ~/ros2_ws/src
git clone <repository_url>
```

2. 安装依赖：
```bash
cd ~/ros2_ws
rosdep install --from-paths src --ignore-src -r -y
```

3. 构建项目：
```bash
cd ~/ros2_ws
colcon build --packages-select vision_detector
```

4. 设置环境：
```bash
source install/setup.bash
```

### 运行检测系统

1. 启动检测系统（包括相机）：
```bash
ros2 launch vision_detector detector.launch.py
```

2. 启动检测系统（不启动相机）：
```bash
ros2 launch vision_detector detector.launch.py realsense:=false
```

3. 查看检测结果：
```bash
ros2 topic echo /detections
```

4. 查看可视化结果：
```bash
ros2 run image_view image_view --ros-args -r image:=/detection_image
```

## 参数配置

| 参数名 | 默认值 | 描述 |
|-------|-------|------|
| model_path | yolov8n.pt | YOLOv8模型路径 |
| conf_threshold | 0.5 | 置信度阈值 |
| iou_threshold | 0.45 | NMS的IoU阈值 |
| device | cuda | 推理设备 (cuda/cpu) |
| publish_visualization | True | 是否发布可视化图像 |
| depth_sample_points | 24 | 深度采样点数 |
| camera_topic | /camera/color/image_raw | 彩色图像话题 |
| depth_topic | /camera/depth/image_rect_raw | 深度图像话题 |
| camera_info_topic | /camera/color/camera_info | 相机信息话题 |

## 性能优化

1. **GPU加速**：使用CUDA进行GPU加速推理
2. **模型选择**：根据性能需求选择不同大小的YOLOv8模型
   - yolov8n: 最快，精度较低
   - yolov8s: 平衡速度和精度
   - yolov8m/l/x: 更高精度，速度较慢

3. **深度采样优化**：调整`depth_sample_points`参数平衡精度和速度

## 故障排除

1. **相机连接问题**：
   - 检查USB连接
   - 确认相机驱动已安装
   - 使用`realsense-viewer`测试相机

2. **检测性能问题**：
   - 降低图像分辨率
   - 使用更小的模型
   - 确认CUDA正确安装

3. **深度数据异常**：
   - 检查深度图像对齐设置
   - 调整采样点数量
   - 检查光照条件

## 扩展开发

### 添加自定义检测类别

修改`yolov8_detector.py`中的类别映射：

```python
# 在检测结果处理中添加自定义类别
custom_classes = {
    'person': 0,
    'custom_object': 80,  # 自定义类别ID
}
```

### 集成其他相机

修改`detector.launch.py`中的相机启动部分，替换为其他相机驱动。

### 添加自定义消息

在`vision_msgs_custom/msg/`目录下创建新的消息定义文件，并重新编译。

## 许可证

Apache-2.0

## 联系方式

维护者: hgzq
邮箱: HGZQ2108299415@outlook.com
