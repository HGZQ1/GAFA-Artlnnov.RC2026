# GAFA-Artlnnov.RC2026
## 广州美术学院RC2026艺创战队视觉组仓库  
本仓库用于存放广州美术学院RC2026艺创战队视觉组的相关资料和项目文件。
### 目录结构  
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
│       │   │   ├── detector_node.py  #ROS2检测节点主程序
│       │   │   ├── yolov8_detector.py # YOLOv8 检测器封装
│       │   │   └── utils.py       # 工具函数：图像处理函数（缩放裁减），检测结果可视化
│       │   ├── launch/
│       │   │   └── detection.launch.py  # ROS2 启动文件
│       │   ├── config/
│       │   │   ├── yolov8_config.yaml # YOLOv8 模型配置
│       │   │   └── camera_config.yaml # 相机参数配置
│       │   ├── resource/
│       │   ├── package.xml
│       │   └── setup.py
│       └── realsense_wrapper/      # RealSense 封装包
│       |   ├── realsense_wrapper/
│       |   │   ├── __init__.py
│       |   │   └── realsense_node.py # ROS2 RealSense 节点主程序（初始化相机，发布RBG-D话题）
│       |   ├── launch/
│       |   │   └── realsense.launch.py # ROS2 启动文件（配置相机参数，启动realsense节点）
│       |   ├── config/
│       |   │   └── camera_params.yaml # RealSense 相机参数配置
│       |   ├── package.xml
│       |   └── setup.py #安装Python包到ROS2环境
|       |
|       └──usbcreamer_wrapper/      # USB摄像头封装包(留着)
├── models/
|       └──yoloKFS/                      # YOLOv8n 模型文件夹(KFS识别)
│       |   ├── yolov8n.pt                  # 原始模型
│       |   ├── yolov8n.onnx                # ONNX 模型
│       |   └── yolov8n_fp16.trt            # TensorRT 引擎
|       └──yolohand/...                      # YOLOv8n 模型文件夹（端头识别）
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
