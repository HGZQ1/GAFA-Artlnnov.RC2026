# GAFA-Artlnnov.RC2026
## 广州美术学院RC2026艺创战队视觉组仓库  
本仓库用于存放广州美术学院RC2026艺创战队视觉组的相关资料和项目文件。

## 目录
1. [项目概述](#项目概述)
2. [项目结构](#项目结构)
3. [系统架构](#系统架构)
4. [节点功能详解](#节点功能详解)
5. [决策逻辑](#决策逻辑)
6. [数据处理流程](#数据处理流程)
7. [运行时序](#运行时序)
8. [技术实现](#技术实现)
9. [配置参数](#配置参数)

---

## 项目概述

### 1.1 项目简介
本项目是一个基于ROS2的机器人视觉感知与决策系统，主要用于机器人比赛中的目标识别、定位和自主导航任务。系统集成了深度视觉、IMU姿态感知、里程计融合、多场景决策等模块，能够实现武馆和梅林两种比赛场景下的自主导航和目标抓取。

### 1.2 核心功能
- **视觉目标检测**：基于YOLOv8的目标识别，支持模型热切换
- **3D空间定位**：结合深度相机实现目标3D坐标计算
- **多传感器融合**：IMU与编码器融合，提供精确的姿态和位姿估计
- **场景决策**：支持武馆和梅林两种比赛场景的决策逻辑
- **坡度感知**：实时检测坡度并自适应调整运动控制
- **机械臂控制**：为目标抓取提供精确的机械臂目标位置

### 1.3 硬件平台
- **视觉传感器**：Intel RealSense D435i深度相机
- **IMU传感器**：集成于D435i的六轴IMU
- **执行机构**：差速底盘+机械臂
- **计算平台**：支持CUDA的GPU加速

---

## 项目结构

### 2.1 完整目录树

```
ros2_ws/
├── src/                                    # 源代码目录
│   ├── vision_detector/                      # 视觉检测包
│   │   ├── vision_detector/
│   │   │   ├── __init__.py
│   │   │   ├── detector_node.py             # 视觉检测主节点
│   │   │   ├── yolov8_detector.py          # YOLOv8检测器封装
│   │   │   ├── model_switcher.py            # 模型热切换工具
│   │   │   └── utils.py                    # 深度处理和坐标转换工具
│   │   ├── config/                         # 配置文件目录
│   │   ├── launch/                         # 启动文件目录
│   │   │   ├── detector.launch.py          # 检测节点启动文件
│   │   │   ├── detector_rviz.launch.py     # 带RViz的启动文件
│   │   │   └── full_system.launch.py       # 完整系统启动文件
│   │   ├── rviz/                           # RViz配置
│   │   │   └── detector_view.rviz
│   │   ├── weights/                        # 模型权重目录
│   │   │   ├── best.pt                     # 武馆场景模型
│   │   │   └── yolov8n.pt                 # 通用检测模型
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── decision_processor/                  # 决策处理包
│   │   ├── decision_processor/
│   │   │   ├── __init__.py
│   │   │   ├── processor_node.py            # 决策处理主节点
│   │   │   ├── robot_decision.py            # 机器人决策状态机
│   │   │   ├── motion_planner.py           # 运动规划器
│   │   │   ├── kalman_filter.py            # 卡尔曼滤波器
│   │   │   ├── tf_manager.py               # TF坐标变换管理器
│   │   │   ├── imu_processor.py            # IMU数据处理节点
│   │   │   ├── odometry_fusion.py          # 里程计融合节点
│   │   │   ├── meilin_navigator.py        # 梅林导航节点
│   │   │   ├── target_confirmation.py      # 目标确认模块
│   │   │   └── config.py                   # 全局配置参数
│   │   ├── scenarios/                      # 场景定义目录
│   │   │   ├── base_scenario.py           # 场景基类
│   │   │   ├── scenario_wuguan.py         # 武馆场景
│   │   │   └── scenario_meilin.py         # 梅林场景
│   │   ├── package.xml
│   │   ├── setup.py
│   │   └── setup.cfg
│   │
│   ├── vision_msgs_custom/                 # 自定义消息包
│   │   ├── msg/
│   │   │   ├── Detection2DExtended.msg     # 扩展检测消息
│   │   │   └── ObjectDistance.msg          # 目标距离消息
│   │   ├── CMakeLists.txt
│   │   └── package.xml
│   │
│   └── auto_serial_bridge-main/            # 串口通信桥接包
│       ├── src/
│       │   ├── serial_controller.cpp        # 串口控制器
│       │   └── serial_node.cpp            # 串口通信节点
│       ├── config/
│       │   ├── protocol.yaml              # 通信协议配置
│       │   └── protocol-sample.yaml       # 协议配置示例
│       ├── launch/
│       │   ├── serial_bridge_by_component.launch.py
│       │   └── serial_bridge_by_node.launch.py
│       ├── mcu_output/
│       │   ├── protocol.c                 # 协议实现
│       │   ├── protocol.h                 # 协议头文件
│       │   └── PROTOCOL_DOC.md           # 协议文档
│       ├── scripts/
│       │   ├── auto_udev.sh              # 自动设备配置脚本
│       │   ├── checksum_build_matrix.py    # 校验和计算工具
│       │   └── codegen.py                # 代码生成工具
│       ├── CMakeLists.txt
│       ├── package.xml
│       └── README.md
│
├── install/                              # 安装目录
├── build/                               # 编译目录
├── log/                                 # 日志目录
├── ROS2_YOLOV8_D435i_Integration_Guide.md # 集成指南
└── 项目完整说明文档.md                     # 本文档
```

### 2.2 主要文件说明

#### 2.2.1 vision_detector包

**核心节点文件**：

- `detector_node.py` - 视觉检测主节点
  - 功能：订阅相机图像，执行目标检测，发布检测结果
  - 订阅话题：/camera/color/image_raw, /camera/depth/image_rect_raw
  - 发布话题：/detections, /vision/raw_target, /detection_image
  - 支持模型热切换和参数动态配置

- `yolov8_detector.py` - YOLOv8检测器封装
  - 功能：封装YOLOv8推理引擎，提供统一检测接口
  - 支持GPU加速推理
  - 实现检测后处理和结果格式化

- `model_switcher.py` - 模型热切换工具
  - 功能：运行时切换YOLOv8模型，无需重启节点
  - 通过ROS2参数服务实现模型切换
  - 支持模型预热和显存管理

- `utils.py` - 工具函数集合
  - 功能：深度处理、坐标转换、角度计算
  - 实现随机采样+中值滤波的深度估计算法
  - 提供像素坐标到3D坐标的转换

**配置与启动文件**：

- `launch/detector.launch.py` - 基础检测节点启动
- `launch/detector_rviz.launch.py` - 带RViz可视化的启动
- `launch/full_system.launch.py` - 完整系统启动
- `rviz/detector_view.rviz` - RViz可视化配置

**模型文件**：

- `weights/best.pt` - 武馆场景专用模型（识别矛尖、拳、掌）
- `weights/yolov8n.pt` - 通用检测模型
- `weights/kfs.pt` - 梅林场景专用模型（识别真、假KFS）

#### 2.2.2 decision_processor包

**核心节点文件**：

- `processor_node.py` - 决策处理主节点
  - 功能：整合视觉、IMU、里程计数据，执行决策逻辑
  - 管理机器人状态机和运动规划
  - 处理多场景切换
  - 订阅话题：/vision/raw_target, /imu/processed, /odom/fused
  - 发布话题：/serial/chassis_cmd, /serial/climb_cmd, /arm/target_pos

- `robot_decision.py` - 机器人决策状态机
  - 功能：实现五状态决策逻辑（SEARCHING/ALIGNING/MOVING/ARRIVED/PICKING）
  - 管理目标确认和运动规划
  - 处理状态转换逻辑

- `motion_planner.py` - 运动规划器
  - 功能：计算转向角度和前进距离
  - 实现梯形速度规划
  - 坡度自适应控制

- `kalman_filter.py` - 卡尔曼滤波器
  - 功能：2D位置跟踪和滤波
  - 实现跳变检测
  - 提供坐标变换回退方案

- `tf_manager.py` - TF坐标变换管理器
  - 功能：管理相机、底盘、机械臂之间的坐标变换
  - 广播静态TF变换
  - 提供坐标变换查询接口

- `imu_processor.py` - IMU数据处理节点
  - 功能：处理IMU原始数据，计算姿态角
  - 实现互补滤波算法
  - 坡度检测和分类

- `odometry_fusion.py` - 里程计融合节点
  - 功能：融合编码器和IMU数据
  - 计算机器人位姿和总行驶距离
  - 坡度补偿编码器误差

- `meilin_navigator.py` - 梅林导航节点
  - 功能：梅林场景路径规划和导航
  - 管理方块状态和运动模式
  - 提供速度和力矩建议

- `target_confirmation.py` - 目标确认模块
  - 功能：连续帧确认目标存在
  - 处理目标丢失逻辑

- `config.py` - 全局配置参数
  - 功能：集中管理所有可配置参数
  - 包含硬件参数、决策参数、场景参数

**场景定义文件**：

- `scenarios/base_scenario.py` - 场景基类
  - 定义场景通用接口

- `scenarios/scenario_wuguan.py` - 武馆场景
  - 实现武馆场景的目标评估逻辑
  - 处理端头识别和抓取决策

- `scenarios/scenario_meilin.py` - 梅林场景
  - 实现梅林场景的目标评估逻辑
  - 处理KFS收集和避障决策

#### 2.2.3 vision_msgs_custom包

- `msg/Detection2DExtended.msg` - 扩展检测消息
  - 扩展标准vision_msgs，添加额外字段

- `msg/ObjectDistance.msg` - 目标距离消息
  - 传递目标距离和置信度信息

#### 2.2.4 auto_serial_bridge-main包

**核心文件**：

- `src/serial_controller.cpp` - 串口控制器
  - 功能：管理串口通信
  - 实现协议解析和打包

- `src/serial_node.cpp` - 串口通信节点
  - 功能：ROS2与MCU之间的通信桥接
  - 订阅控制指令，发送到MCU
  - 接收传感器数据，发布到ROS2

- `config/protocol.yaml` - 通信协议配置
  - 定义数据包格式
  - 配置校验和算法

- `mcu_output/protocol.c/h` - 协议实现
  - MCU端的协议实现
  - 提供数据打包/解包函数

**辅助工具**：

- `scripts/auto_udev.sh` - 自动设备配置脚本
- `scripts/checksum_build_matrix.py` - 校验和计算工具
- `scripts/codegen.py` - 代码生成工具

---

## 系统架构

### 2.1 节点拓扑图

```
┌─────────────────────────────────────────────────────────────┐
│                        ROS2 系统                           │
├─────────────────────────────────────────────────────────────┤
│                                                              │
│  ┌──────────────┐      ┌──────────────┐                    │
│  │  vision_     │      │  decision_   │                    │
│  │  detector    │─────▶│  processor   │                    │
│  │              │      │              │                    │
│  └──────────────┘      └──────┬───────┘                    │
│                                │                             │
│  ┌──────────────┐      ┌──────▼───────┐                    │
│  │  imu_        │      │  meilin_     │                    │
│  │  processor   │─────▶│  navigator   │                    │
│  │              │      │              │                    │
│  └──────────────┘      └──────────────┘                    │
│                                │                             │
│  ┌──────────────┐      ┌──────▼───────┐                    │
│  │  odometry_   │─────▶│  serial_     │                    │
│  │  fusion      │      │  bridge      │                    │
│  │              │      │              │                    │
│  └──────────────┘      └──────────────┘                    │
│                                                              │
└─────────────────────────────────────────────────────────────┘
```

### 2.2 话题通信图

```
视觉数据流：
/camera/color/image_raw ──▶ vision_detector
/camera/depth/image_rect_raw ──▶ vision_detector
/vision/raw_target ──▶ decision_processor

传感器数据流：
/camera/camera/imu ──▶ imu_processor
/feedback/encoder ──▶ odometry_fusion
/imu/processed ──▶ decision_processor
/odom/fused ──▶ decision_processor, meilin_navigator

决策控制流：
/decision/state ──▶ 外部监控
/serial/chassis_cmd ──▶ serial_bridge
/serial/climb_cmd ──▶ serial_bridge
/arm/target_pos ──▶ 机械臂控制
/meilin/nav_state ──▶ decision_processor
```

---

## 节点功能详解

### 3.1 Vision Detector节点

**主要文件**：
- `vision_detector/detector_node.py` - 检测节点主程序
- `vision_detector/yolov8_detector.py` - YOLOv8检测器封装
- `vision_detector/utils.py` - 深度处理和坐标转换工具
- `vision_detector/model_switcher.py` - 模型热切换工具

**核心功能**：
1. **图像采集与处理**
   - 订阅彩色图像话题：`/camera/color/image_raw`
   - 订阅深度图像话题：`/camera/depth/image_rect_raw`
   - 订阅相机内参话题：`/camera/color/camera_info`

2. **目标检测**
   - 使用YOLOv8模型进行目标检测
   - 支持模型热切换（无需重启节点）
   - 可配置置信度阈值和NMS阈值

3. **深度估计**
   - 从深度图中提取目标距离
   - 采用随机采样+中值滤波算法提高精度
   - 支持可配置的采样点数量

4. **3D坐标计算**
   - 像素坐标转换为3D相机坐标
   - 计算方位角和俯仰角
   - 发布原始目标位置信息

5. **可视化**
   - 发布检测可视化图像到`/detection_image`话题
   - 绘制检测框、类别标签和距离信息

**关键代码模块**：

```python
# 目标检测核心函数
def detect(self, image: np.ndarray) -> List[dict]:
    results = self.model(
        image,
        conf=self.conf_threshold,
        iou=self.iou_threshold,
        verbose=False,
        device=self.device
    )[0]

    detections = []
    if results.boxes is not None:
        for i in range(len(results.boxes)):
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

**模型热切换机制**：
- 通过ROS2参数服务实现
- 支持显存管理和模型预热
- 切换过程对检测影响最小化

---

### 3.2 Decision Processor节点

**主要文件**：
- `decision_processor/processor_node.py` - 决策处理主节点
- `decision_processor/robot_decision.py` - 机器人决策状态机
- `decision_processor/motion_planner.py` - 运动规划器
- `decision_processor/kalman_filter.py` - 卡尔曼滤波器
- `decision_processor/tf_manager.py` - 坐标变换管理器
- `decision_processor/target_confirmation.py` - 目标确认模块

**核心功能**：
1. **目标处理**
   - 订阅原始目标位置：`/vision/raw_target`
   - 坐标变换：相机坐标系→底盘坐标系
   - 目标跳变检测与过滤

2. **状态机决策**
   - 五种状态：SEARCHING、ALIGNING、MOVING、ARRIVED、PICKING
   - 基于目标确认和运动规划的状态转换
   - 支持场景特定决策逻辑

3. **运动规划**
   - 两阶段运动规划：对准→移动
   - 梯形速度规划：加速→匀速→减速
   - 坡度自适应：根据坡度调整速度和力矩

4. **多场景支持**
   - 武馆场景：端头识别与抓取
   - 梅林场景：KFS收集与避障
   - 场景自动切换机制

**关键代码模块**：

```python
# 状态机核心逻辑
def _state_machine(self, confirmed: bool, plan) -> dict:
    cmd = self._stop_cmd()

    if self.state == RobotState.SEARCHING:
        if confirmed:
            self.state = RobotState.ALIGNING
        else:
            cmd['search_rotate'] = 1.0

    elif self.state == RobotState.ALIGNING:
        if not confirmed:
            self.state = RobotState.SEARCHING
            self.kalman.reset()
        elif plan.phase == 'ALIGNING':
            cmd['turn_angle'] = plan.turn_deg
            cmd['turn_wheels'] = plan.turn_wheels
        elif plan.phase in ('MOVING', 'ARRIVED'):
            self.state = RobotState.MOVING

    # ... 其他状态处理
    return cmd
```

---

### 3.3 IMU Processor节点

**主要文件**：
- `decision_processor/imu_processor.py` - IMU数据处理节点

**核心功能**：
1. **IMU数据采集**
   - 订阅加速度计数据：`/camera/camera/accel/sample`
   - 订阅陀螺仪数据：`/camera/camera/gyro/sample`
   - 订阅融合IMU数据：`/camera/camera/imu`

2. **姿态解算**
   - 互补滤波融合加速度计和陀螺仪数据
   - 计算俯仰角(pitch)、横滚角(roll)和偏航角速度(yaw_rate)
   - 坡度平滑处理，减少噪声影响

3. **坡度检测**
   - 四级坡度分类：FLAT、MILD、MODERATE、STEEP
   - 实时判断上坡/下坡/平地状态
   - 发布坡度等级和状态信息

**关键代码模块**：

```python
# 互补滤波核心算法
def _on_accel(self, msg: Imu):
    # 计算加速度计角度
    accel_pitch = math.degrees(
        math.atan2(az, math.sqrt(ay**2 + az**2)))
    accel_roll = math.degrees(
        math.atan2(ax, -ay))

    # 陀螺仪积分
    gyro_pitch_rate = math.degrees(self._gyro_y)
    gyro_roll_rate = math.degrees(self._gyro_x)

    # 互补滤波
    alpha = IMU_COMP_ALPHA
    self._pitch_filtered = (
        alpha * (self._pitch_filtered + gyro_pitch_rate * dt)
        + (1 - alpha) * accel_pitch
    )
    self._roll_filtered = (
        alpha * (self._roll_filtered + gyro_roll_rate * dt)
        + (1 - alpha) * accel_roll
    )
```

---

### 3.4 Odometry Fusion节点

**主要文件**：
- `decision_processor/odometry_fusion.py` - 里程计融合节点

**核心功能**：
1. **编码器数据处理**
   - 订阅编码器反馈：`/feedback/encoder`
   - 提取前进距离和转向角度增量
   - 坡度补偿：计算水平位移

2. **IMU辅助定位**
   - 融合IMU的yaw_rate提高转向精度
   - 使用互补滤波器融合编码器和IMU数据
   - 坡度补偿编码器测量误差

3. **位姿积分**
   - 中点法积分更新位姿
   - 角度归一化到[-π, π]
   - 发布融合后的位姿信息

**关键代码模块**：

```python
# 位姿积分核心算法
def _on_encoder(self, msg: Twist):
    delta_x = msg.linear.x
    delta_theta_enc = msg.angular.z

    # 坡度补偿
    pitch_rad = math.radians(self._pitch_deg)
    horiz_factor = math.cos(pitch_rad)
    delta_x_horiz = delta_x * horiz_factor

    # IMU辅助转向
    dt = 1.0 / ENCODER_RATE_HZ
    delta_theta_imu = math.radians(self._yaw_rate * dt)

    # 融合转向角
    delta_theta = (self._theta_alpha * delta_theta_enc
                   + (1 - self._theta_alpha) * delta_theta_imu)

    # 中点法积分
    mid_theta = self._theta + delta_theta / 2.0
    self._x += delta_x_horiz * math.cos(mid_theta)
    self._y += delta_x_horiz * math.sin(mid_theta)
    self._theta += delta_theta
```

---

### 3.5 Meilin Navigator节点

**主要文件**：
- `decision_processor/meilin_navigator.py` - 梅林导航节点

**核心功能**：
1. **路径规划**
   - 支持自定义路径设置
   - 跟踪当前方块和下一方块
   - 计算方块间高度差

2. **运动模式判断**
   - 五种运动模式：FLAT、CLIMB、DESCEND、SETTLE、DONE
   - 基于高度差和IMU坡度判断
   - 提供速度和力矩建议

3. **导航状态发布**
   - 发布当前方块、下一方块信息
   - 发布运动模式和高度差
   - 发布建议的速度和力矩系数

**关键代码模块**：

```python
# 运动模式计算
def _calc_move_mode(self) -> tuple:
    cur = self._get_current_block()
    next = self._get_next_block()

    cur_h = BLOCK_HEIGHTS.get(cur, 0.0)
    next_h = BLOCK_HEIGHTS.get(next, 0.0)
    h_diff = next_h - cur_h

    # IMU辅助判断
    imu_climbing = self._pitch_deg > SLOPE_DETECT_DEG
    imu_descend = self._pitch_deg < -SLOPE_DETECT_DEG

    if self._in_settle:
        return MoveMode.SETTLE, h_diff, 0.0, 0.0

    if h_diff > 0.03:   # 上坡
        speed = self._slope_speed_factor(abs(self._pitch_deg))
        torque = self._slope_torque_factor(abs(self._pitch_deg))
        return MoveMode.CLIMB, h_diff, speed, torque

    elif h_diff < -0.03:  # 下坡
        return MoveMode.DESCEND, h_diff, BRAKE_FACTOR_SLOPE, 0.5

    else:
        return MoveMode.FLAT, h_diff, SPEED_FACTOR_FLAT, TORQUE_FLAT
```

---

## 决策逻辑

### 4.1 状态机设计

机器人决策采用有限状态机(FSM)架构，包含五种状态：

```
┌──────────┐
│SEARCHING │◀───┐
└────┬─────┘    │
     │          │
     ▼          │
┌──────────┐    │
│ALIGNING  │    │
└────┬─────┘    │
     │          │
     ▼          │
┌──────────┐    │
│ MOVING   │────┘
└────┬─────┘
     │
     ▼
┌──────────┐
│ ARRIVED  │
└────┬─────┘
     │
     ▼
┌──────────┐
│ PICKING  │───┐
└──────────┘   │
               │
               └──▶ SEARCHING
```

**状态转换条件**：
1. **SEARCHING → ALIGNING**
   - 检测到有效目标
   - 目标连续确认帧数达到阈值

2. **ALIGNING → MOVING**
   - 目标对准角度小于阈值
   - 运动规划进入移动阶段

3. **MOVING → ARRIVED**
   - 距离目标小于停止距离
   - 运动规划进入到达阶段

4. **ARRIVED → PICKING**
   - 自动转换，开始抓取动作

5. **PICKING → SEARCHING**
   - 抓取持续时间达到设定值
   - 重置所有状态，准备下一个目标

### 4.2 场景决策逻辑

#### 武馆场景决策
```python
def evaluate_target(self, class_id, confidence, distance, base_x, base_y):
    # 只追踪指定类别的端头
    if class_id != self.target_class:
        return 'IGNORE'

    # 置信度不足忽略
    if confidence < 0.80:
        return 'IGNORE'

    # 根据距离决定动作
    if distance > 0.5:
        return 'APPROACH'
    elif distance > WEAPON_GRAB_DIST:
        return 'APPROACH'
    else:
        return 'PICK'
```

#### 梅林场景决策
```python
def evaluate_target(self, class_id, confidence, distance, base_x, base_y):
    # 假KFS：最高优先级，立即绕开
    if class_id == KFS_CLASS_FAKE_KFS:
        return 'AVOID'

    # R1 KFS：不能碰，直接忽略
    if class_id == KFS_CLASS_R1_KFS:
        return 'IGNORE'

    # R2 KFS：正常收集
    if class_id == KFS_CLASS_R2_KFS:
        if confidence < R2_CONFIDENCE_MIN:
            return 'IGNORE'
        if distance > MEILIN_STOP_DIST:
            return 'APPROACH'
        return 'PICK'

    return 'IGNORE'
```

### 4.3 运动规划逻辑

运动规划分为两个阶段：

**阶段1：对准(ALIGNING)**
- 计算目标相对机器人的角度
- 如果角度大于阈值，执行原地转向
- 转向角度 = atan2(目标Y, 目标X)

**阶段2：移动(MOVING)**
- 计算到目标的直线距离
- 减去安全停止距离
- 执行前进运动

**坡度自适应**：
- 根据坡度等级调整速度系数
- 上坡时降低速度、增大驱动力
- 下坡时启用制动系数

---

## 数据处理流程

### 5.1 视觉数据处理流程

```
1. 图像采集
   └─▶ 彩色图像 (640x480)
   └─▶ 深度图像 (640x480)

2. 目标检测
   └─▶ YOLOv8推理
   └─▶ 检测框 [x1, y1, x2, y2]
   └─▶ 类别ID和置信度

3. 深度估计
   └─▶ 检测框中心点采样
   └─▶ 随机采样+中值滤波
   └─▶ 目标距离 (米)

4. 坐标转换
   └─▶ 像素坐标 → 3D相机坐标
   └─▶ 相机坐标 → 底盘坐标 (TF变换)

5. 数据发布
   └─▶ /vision/raw_target
   └─▶ /detections
   └─▶ /detection_image
```

### 5.2 IMU数据处理流程

```
1. 数据采集
   └─▶ 加速度计 (ax, ay, az)
   └─▶ 陀螺仪 (ωx, ωy, ωz)

2. 姿态解算
   └─▶ 加速度计角度
       └─▶ pitch = atan2(az, √(ay²+az²))
       └─▶ roll = atan2(ax, -ay)

   └─▶ 陀螺仪积分
       └─▶ pitch += ωy × dt
       └─▶ roll += ωx × dt

3. 互补滤波
   └─▶ pitch = α×pitch_gyro + (1-α)×pitch_accel
   └─▶ roll = α×roll_gyro + (1-α)×roll_accel

4. 坡度判断
   └─▶ 坡度平滑 (滑动窗口)
   └─▶ 坡度等级分类
   └─▶ 上坡/下坡/平地判断

5. 数据发布
   └─▶ /imu/processed
   └─▶ /imu/slope_level
```

### 5.3 里程计融合流程

```
1. 编码器数据
   └─▶ 前进距离 Δx
   └─▶ 转向角度 Δθ

2. IMU辅助
   └─▶ 偏航角速度 ωz
   └─▶ 俯仰角 pitch

3. 坡度补偿
   └─▶ 水平位移 = Δx × cos(pitch)
   └─▶ 转向融合 = α×Δθ_enc + (1-α)×Δθ_imu

4. 位姿积分
   └─▶ x += Δx_horiz × cos(θ + Δθ/2)
   └─▶ y += Δx_horiz × sin(θ + Δθ/2)
   └─▶ θ += Δθ

5. 数据发布
   └─▶ /odom/fused
```

---

## 运行时序

### 6.1 系统启动序列

```
1. 启动ROS2核心
   └─▶ ros2 daemon start

2. 启动视觉检测节点
   └─▶ ros2 run vision_detector detector_node

3. 启动IMU处理节点
   └─▶ ros2 run decision_processor imu_processor

4. 启动里程计融合节点
   └─▶ ros2 run decision_processor odometry_fusion

5. 启动决策处理节点
   └─▶ ros2 run decision_processor processor_node

6. 启动梅林导航节点 (梅林场景)
   └─▶ ros2 run decision_processor meilin_navigator

7. 启动串口通信节点
   └─▶ ros2 run auto_serial_bridge serial_node
```

### 6.2 数据处理时序

```
视觉检测 (30Hz)
  ├─ 图像采集: 0-10ms
  ├─ YOLO推理: 10-25ms
  ├─ 深度估计: 25-28ms
  └─ 数据发布: 28-33ms

IMU处理 (100Hz)
  ├─ 数据采集: 0-2ms
  ├─ 互补滤波: 2-4ms
  └─ 数据发布: 4-5ms

里程计融合 (50Hz)
  ├─ 编码器更新: 0-1ms
  ├─ 姿态积分: 1-3ms
  └─ 数据发布: 3-5ms

决策处理 (20Hz)
  ├─ 目标接收: 0-2ms
  ├─ 坐标变换: 2-5ms
  ├─ 状态机更新: 5-8ms
  └─ 指令发布: 8-10ms
```

### 6.3 典型任务时序

**武馆抓取任务**：
```
T0: 搜索模式启动
T0-T5: 旋转搜索目标
T5: 检测到端头
T5-T7: 目标确认 (3帧)
T7-T10: 对准端头
T10-T15: 移动到端头前
T15-T18: 到达抓取位置
T18-T21: 执行抓取动作
T21: 返回搜索模式
```

**梅林导航任务**：
```
T0: 入口启动
T0-T2: 检测坡度
T2-T5: 上坡到第一方块
T5-T7: 方块稳定等待
T7-T10: 移动到下一方块
...
T30: 到达出口
```

---

## 技术实现

### 7.1 深度学习技术

**YOLOv8模型**：
- 模型架构：YOLOv8n/nano版本
- 输入尺寸：640×640
- 推理设备：CUDA GPU
- 模型格式：PyTorch .pt文件

**模型热切换**：
- 通过ROS2参数服务实现
- 显存管理和模型预热
- 切换过程对检测影响最小化

### 7.2 传感器融合技术

**互补滤波**：
- 加速度计提供长期稳定的角度估计
- 陀螺仪提供短期准确的角度变化率
- 互补系数α=0.96，平衡两者优势

**里程计融合**：
- 编码器提供位移和转向测量
- IMU提供坡度补偿和转向辅助
- 融合权重根据坡度动态调整

### 7.3 坐标变换技术

**TF坐标树**：
```
base_link (原点)
  ├─ camera_link (相机)
  └─ arm_base_link (机械臂底座)
```

**坐标变换**：
- 使用tf2库进行坐标变换
- 支持静态变换广播
- 提供回退方案以防TF查询失败

### 7.4 运动规划技术

**梯形速度规划**：
- 加速阶段：速度线性增加
- 匀速阶段：保持最大速度
- 减速阶段：速度线性减小

**坡度自适应**：
- 四级坡度分类
- 动态调整速度和力矩系数
- 下坡时启用制动模式

### 7.5 状态机技术

**有限状态机(FSM)**：
- 五种状态定义机器人行为
- 基于事件驱动的状态转换
- 支持场景特定的决策逻辑

---

## 配置参数

### 8.1 硬件参数

```python
# 底盘参数
WHEEL_DIAMETER_M = 0.096   # 车轮直径 (米)
TRACK_WIDTH_M = 0.28       # 轮距 (米)

# 相机参数
CAMERA_OFFSET_X = 0.05     # 相机X偏移 (米)
CAMERA_OFFSET_Y = 0.0      # 相机Y偏移 (米)
CAMERA_OFFSET_Z = 0.15     # 相机Z偏移 (米)

# 机械臂参数
ARM_OFFSET_X = 0.0         # 机械臂X偏移 (米)
ARM_OFFSET_Y = 0.0         # 机械臂Y偏移 (米)
ARM_OFFSET_Z = 0.05        # 机械臂Z偏移 (米)
```

### 8.2 运动控制参数

```python
# 运动控制
STOP_DISTANCE_M = 0.25     # 停止距离 (米)
ARRIVAL_THRESHOLD_M = 0.30 # 到达阈值 (米)
ALIGN_THRESHOLD_DEG = 5.0  # 对准角度容限 (度)

# 决策参数
CONF_THRESHOLD = 0.5       # YOLO置信度阈值
CONFIRM_FRAMES = 3         # 确认帧数
LOST_FRAMES = 5            # 丢失帧数
MAX_JUMP_M = 0.8          # 跳变检测阈值 (米)
```

### 8.3 坡度控制参数

```python
# 坡度检测
SLOPE_DETECT_DEG = 3.0     # 坡度检测阈值 (度)
SLOPE_LEVEL_MILD = 8.0     # 轻坡阈值 (度)
SLOPE_LEVEL_MODERATE = 15.0 # 中坡阈值 (度)
SLOPE_LEVEL_STEEP = 25.0   # 陡坡阈值 (度)

# 速度系数
SPEED_FACTOR_FLAT = 1.0    # 平地速度系数
SPEED_FACTOR_MILD = 0.7     # 轻坡速度系数
SPEED_FACTOR_MODERATE = 0.45 # 中坡速度系数
SPEED_FACTOR_STEEP = 0.25   # 陡坡速度系数

# 力矩系数
TORQUE_FACTOR_FLAT = 0.5    # 平地力矩系数
TORQUE_FACTOR_MILD = 0.7    # 轻坡力矩系数
TORQUE_FACTOR_MODERATE = 0.85 # 中坡力矩系数
TORQUE_FACTOR_STEEP = 1.0   # 陡坡力矩系数
```

### 8.4 IMU参数

```python
# 话题配置
IMU_TOPIC = '/camera/camera/imu'
IMU_ACCEL_TOPIC = '/camera/camera/accel/sample'
IMU_GYRO_TOPIC = '/camera/camera/gyro/sample'

# 滤波参数
IMU_COMP_ALPHA = 0.96      # 互补滤波系数
GRAVITY = 9.81             # 重力加速度 (m/s²)

# 编码器参数
ENCODER_RATE_HZ = 50       # 编码器频率 (Hz)
ODOM_DRIFT_FACTOR = 0.02   # 里程计漂移系数
```

---

## 总结

本系统是一个暂时完整的ROS2视觉感知与决策系统，集成了深度学习、传感器融合、运动规划等多种技术。系统采用模块化设计，各节点通过ROS2话题通信，实现了武馆和梅林两种比赛场景下的自主导航和目标抓取功能。 
后续将接入激光雷达及USB相机

系统的主要特点：
1. **模块化设计**：各功能模块独立，便于维护和扩展
2. **多传感器融合**：结合视觉、IMU、编码器等多种传感器
3. **场景自适应**：支持多种比赛场景的决策逻辑
4. **坡度感知**：实时检测坡度并自适应调整控制策略
5. **高可靠性**：采用目标确认、跳变检测等机制提高系统鲁棒性 

### 求求了，一定要过中选啊(小琴羽祈祷中...喵)

本项目遵循相应的开源许可证。

