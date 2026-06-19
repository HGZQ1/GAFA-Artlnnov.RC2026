# RC2026 ROS2 视觉感知与决策系统 v4.0 说明文档

## 项目概览

本项目为 Robocon 2026 比赛 R2 机器人的上位机系统，基于 ROS2 Humble 构建，运行在 Jetson Orin NX 上。系统分为两个独立工作空间：

| 工作空间 | 用途 | 部署环境 |
|----------|------|----------|
| `ros2_ws` | 真机代码，包含感知、决策、导航、通信 | Jetson Orin NX |
| `sim_ws` | Gazebo 仿真环境，用于离线调试 | 开发机 (x86 Ubuntu) |

坐标系采用 **game 坐标系**：原点在场地顶部中央，左半场 x<0，右半场 x>0，y 轴指向场地内部。左右半场关于 x=0 对称，右半场航点只需 x 取反。

---

# 一、ros2_ws（真机工作空间）

## 1. 代码结构

```
ros2_ws/src/
├── rc2026_bringup/          # 机器人启动与硬件配置
│   ├── launch/
│   │   ├── robot_bringup.launch.py    # 全系统启动 (传感器+视觉+决策)
│   │   ├── navigation.launch.py       # 导航启动 (FAST-LIO+EKF+WaypointNav)
│   │   └── sensor_only.launch.py      # 仅传感器 (调试用)
│   ├── config/
│   │   ├── MID360s_config.json        # Livox Mid-360S 网络配置
│   │   ├── robot_params.yaml          # 机器人物理参数
│   │   └── measurement_params.yaml    # 测量标定参数
│   ├── map/
│   │   ├── left_half.pgm/yaml         # 左半场2D地图 (备用)
│   │   ├── right_half.pgm/yaml        # 右半场2D地图 (备用)
│   │   └── field_waypoints.yaml       # 场地航点坐标
│   └── urdf/                          # (真机URDF, 无Gazebo插件)
│
├── rc2026_navigation/       # 定位与导航
│   ├── launch/
│   │   └── fastlio.launch.py          # FAST-LIO + EKF + 轮式里程计
│   ├── config/
│   │   ├── fastlio2/
│   │   │   ├── mapping.yaml           # FAST-LIO 建图参数
│   │   │   └── localization.yaml      # FAST-LIO 定位参数
│   │   └── ekf.yaml                   # robot_localization EKF 融合参数
│   └── scripts/
│       ├── save_cloud_map.py          # 保存点云地图
│       └── pcd_to_gridmap.py          # 点云转栅格
│
├── decision_processor/      # 决策处理核心
│   └── decision_processor/
│       ├── config.py                  # 全局参数 (航点/台阶高度/动作ID)
│       ├── game_controller.py         # 比赛全流程状态机
│       ├── waypoint_navigator.py      # 闭环PID路径点导航器
│       ├── processor_node.py          # 视觉伺服主节点
│       ├── robot_decision.py          # 五状态视觉决策状态机
│       ├── kalman_filter.py           # 2D卡尔曼滤波 (视觉目标平滑)
│       ├── motion_planner.py          # 运动规划 (坡度感知)
│       ├── meilin_path_planner.py     # 梅林BFS路径规划
│       ├── meilin_navigator.py        # 梅林台阶间导航
│       ├── tf_manager.py             # TF坐标变换管理
│       ├── target_confirmation.py     # 目标确认 (多帧一致性)
│       ├── imu_processor.py           # IMU数据处理/坡度检测
│       ├── odometry_fusion.py         # 编码器+IMU简易融合 (备用)
│       ├── mock_feedback_node.py      # 仿真用模拟反馈
│       └── scenarios/
│           ├── scenario_wuguan.py     # 武馆场景策略
│           └── scenario_meilin.py     # 梅林场景策略
│
├── vision_detector/         # YOLO视觉检测
│   └── vision_detector/
│       ├── detector_node.py           # ROS2检测主节点 (支持模型热切换)
│       ├── yolov8_detector.py         # YOLOv8推理封装
│       ├── model_switcher.py          # 模型切换管理
│       └── utils.py                   # 距离计算/像素转3D
│
├── cmd_vel_bridge/          # 速度指令桥接
│   └── cmd_vel_bridge/
│       ├── bridge_node.py             # /cmd_vel → /serial/chassis_cmd
│       └── wheel_odom_publisher.py    # 编码器Twist → Odometry转换
│
├── auto_serial_bridge-main/ # 串口通信框架
│   ├── config/
│   │   └── protocol.yaml             # 串口协议定义 (所有Jetson↔STM32消息)
│   └── launch/
│       └── serial_bridge_by_node.launch.py
│
└── vision_msgs_custom/      # 自定义视觉消息类型
```

## 2. 各模块功能详解

### 2.1 定位模块 — FAST-LIO + EKF 三源融合

**文件**: `rc2026_navigation/launch/fastlio.launch.py`, `config/ekf.yaml`

三个传感器数据源融合为一个高精度定位输出：

```
┌────────────────┐  ┌────────────────┐  ┌────────────────┐
│ Livox Mid-360S │  │ Livox IMU      │  │ STM32 编码器    │
│ 点云 10Hz      │  │ 角速度+加速度  │  │ 位置+速度 50Hz │
│ /livox/lidar   │  │ 200Hz          │  │ /feedback/      │
└───────┬────────┘  │ /livox/imu     │  │  wheel_odom    │
        │           └───────┬────────┘  └───────┬────────┘
        ▼                   │                   │
   ┌─────────┐              │            ┌──────▼───────┐
   │FAST-LIO2│◄─────────────┘            │wheel_odom_pub│
   │激光+IMU │                           │Twist→Odometry│
   │紧耦合   │                           └──────┬───────┘
   └────┬────┘                                  │
        │                                       │
        ▼                                       ▼
   /odom/lidar                             /odom/wheel
   (Odometry)                              (Odometry)
        │           /livox/imu                  │
        │               │                      │
        └───────────────┼──────────────────────┘
                        ▼
              ┌──────────────────┐
              │ robot_localization│
              │ EKF 卡尔曼融合   │
              │                  │
              │ odom0: 编码器    │ → 位置+速度 (高频)
              │ odom1: FAST-LIO │ → 位置+航向 (高精度)
              │ imu0:  IMU      │ → 角速度+加速度 (超高频)
              └────────┬─────────┘
                       │
              ┌────────▼─────────┐
              │ /odom (Odometry) │
              │ TF: odom→base_link│
              │ 50Hz, 融合精度   │
              └──────────────────┘
```

**各数据源的贡献**:
- **编码器**: 提供高频速度 (vx, vy, vyaw)，响应快但打滑会漂
- **FAST-LIO**: 提供绝对位置 (x, y, yaw)，精度高但频率低
- **IMU**: 提供角速度和线加速度，填补两帧之间的预测空档

### 2.2 导航模块 — WaypointNavigator PID 闭环

**文件**: `decision_processor/waypoint_navigator.py`

替代 Nav2，直接 PID 控制全向底盘导航到目标点。

**两阶段导航**:
1. **PID 闭环** (粗到位 ~5cm): 读取 TF 位姿，对角线直达目标
2. **视觉伺服** (精对齐 ~2cm, 可选): 相机识别目标，精确对齐

**PID 导航算法** (每 50ms 执行一次):
```
输入: 目标 game 坐标 (target_x, target_y, target_yaw)
      当前 game 坐标 (cur_x, cur_y, cur_yaw) ← 从 TF 读取并转换

1. 计算位置误差 (game 坐标系):
   ex = target_x - cur_x
   ey = target_y - cur_y
   dist = √(ex² + ey²)

2. 计算速度大小 (P控制 + 减速区):
   if dist > decel_distance:
     speed = clamp(kp × dist, min_speed, max_speed)
   else:
     speed = max_speed × (dist / decel_distance)  ← 线性减速

3. 计算世界坐标速度方向:
   vx_world = (ex / dist) × speed    ← 指向目标的单位向量 × 速度
   vy_world = (ey / dist) × speed

4. 世界坐标 → 机器人体坐标 (旋转变换):
   cmd.linear.x =  vx_world × cos(yaw) + vy_world × sin(yaw)
   cmd.linear.y = -vx_world × sin(yaw) + vy_world × cos(yaw)

5. 角速度控制:
   cmd.angular.z = clamp(kp_ang × yaw_error, -max_ang, max_ang)

6. 到达判定:
   if dist ≤ 0.05m and |yaw_error| ≤ 0.1rad → ARRIVED
```

**坐标变换模式**:

| 模式 | 公式 | 使用场景 |
|------|------|----------|
| `gazebo` | game_x=gz_y, game_y=-gz_x+6.0, yaw=gz_yaw-π/2 | Gazebo 仿真 |
| `fastlio` | game = R(start_yaw)×loc + (start_x, start_y) | 真机 FAST-LIO |
| `offset` | game = loc + offset | 简单偏移 |

**超时保护** (分层):
- 进度超时 (3s): 距离未减少 → 重试 (最多3次)
- 路径点超时 (30s): 总时间超限 → 放大容差判定或放弃
- 视觉伺服超时 (15s): 对齐超时 → 接受 PID 位置

### 2.3 决策模块 — GameController 全流程状态机

**文件**: `decision_processor/game_controller.py`

武馆流程:
```
WAIT_INPUT → WAIT_START → NAV_TO_WEAPON → ALIGN_WEAPON →
GRAB_WEAPON → NAV_TO_ASSEMBLY → WAIT_ASSEMBLY → RELEASE_WEAPON →
WAIT_ENTER_MERLIN → NAV_TO_MERLIN_ENTRY → SWITCH_TO_MERLIN
```

梅林流程 (子状态机):
```
M_INIT → M_ENTRY_NAV → M_ENTRY_CLIMB →
[M_ON_BLOCK → M_PICKUP_NAV → M_ALIGN_KFS → M_PICKUP_KFS →]
M_NAV_TO_TRIGGER → M_SEND_CLIMB → M_CLIMB_WAIT →
M_NAV_TO_CENTER → M_ON_BLOCK → ... →
M_EXIT_NAV → M_EXIT_DESCEND → M_DONE → STOP
```

### 2.4 视觉模块 — YOLO 检测 + 视觉伺服

**文件**: `vision_detector/detector_node.py`, `decision_processor/processor_node.py`

```
Intel D435i → /camera/color/image_raw
                    │
                    ▼
            detector_node (YOLOv8)
            模型: wuqi.pt (武馆) / kfs.pt (梅林)
                    │
                    ▼
            /vision/raw_target (PointStamped)
            frame_id = "class_id:name:confidence"
            point = (cam_x, cam_y, distance)
                    │
                    ▼
            processor_node (视觉伺服)
            ├── TF 变换: camera→base_link
            ├── 卡尔曼滤波: 平滑检测噪声
            ├── robot_decision: 5状态机
            │   SEARCHING→ALIGNING→MOVING→ARRIVED→PICKING
            └── 发布 /serial/chassis_cmd (底盘微调指令)
```

### 2.5 通信模块 — 串口协议

**文件**: `auto_serial_bridge-main/config/protocol.yaml`

| 方向 | 消息 | ID | 话题 | 内容 |
|------|------|----|------|------|
| Jetson→STM32 | CmdVel | 0x01 | /serial/chassis_cmd | vx, vy, rotation_deg |
| Jetson→STM32 | MeilinCmd | 0x04 | /serial/meilin_cmd | 台阶编号, 爬升模式 |
| Jetson→STM32 | ActionGroupCmd | 0x09 | /serial/action_group_cmd | 动作组ID |
| Jetson→STM32 | GameCmd | 0x08 | /game/cmd | 比赛指令 |
| STM32→Jetson | WheelOdom | 0x25 | /feedback/wheel_odom | 位置+速度 (EKF用) |
| STM32→Jetson | ActionGroupFeedback | 0x14 | /feedback/action_group | 动作完成状态 |
| STM32→Jetson | AssemblyStatus | 0x11 | /feedback/assembly | 组装状态 |
| STM32→Jetson | StartButton | 0x12 | /game/start_signal | 启动按钮 |
| STM32→Jetson | R1Signal | 0x13 | /game/r1_signal | R1机器人信号 |
| STM32→Jetson | GripperStatus | 0x10 | /feedback/gripper | 夹爪状态 |

### 2.6 速度桥接模块

**文件**: `cmd_vel_bridge/bridge_node.py`

```
waypoint_navigator          cmd_vel_bridge            STM32
/cmd_vel (Twist)    →    限速 + rad/s→deg转换    →   /serial/chassis_cmd
  vx m/s                  超时保护 (0.5s)            vx m/s
  vy m/s                                             vy m/s
  omega rad/s                                        rotation deg
```

## 3. 完整信息流 (传感器→底盘指令)

```
═══════════════════════════════════════════════════════════════
                       传感器层
═══════════════════════════════════════════════════════════════
  Livox Mid-360S        STM32 编码器        Intel D435i
  ├── /livox/lidar      /feedback/          /camera/color/
  └── /livox/imu         wheel_odom          image_raw
       │    │               │                    │
═══════╪════╪═══════════════╪════════════════════╪════════════
       │    │   定位层       │                    │  感知层
═══════╪════╪═══════════════╪════════════════════╪════════════
       ▼    │               ▼                    ▼
   FAST-LIO │        wheel_odom_pub       detector_node
   /odom/lidar              /odom/wheel    /vision/raw_target
       │    │               │                    │
       │    ▼               │                    │
       │  EKF 数据源2       │                    │
       │  (角速度+加速度)   │                    │
       └────┬───────────────┘                    │
            ▼                                    │
    ┌──────────────┐                             │
    │  EKF 融合    │                             │
    │  /odom       │                             │
    │  TF: odom→   │                             │
    │   base_link  │                             │
    └──────┬───────┘                             │
           │                                     │
═══════════╪═════════════════════════════════════╪════════════
           │              决策层                  │
═══════════╪═════════════════════════════════════╪════════════
           │                                     │
    static TF:                                   │
    map→odom                                     │
           │                                     │
           ▼                                     │
    TF: map→base_link                            │
           │                                     │
           ▼                                     ▼
    waypoint_navigator ◄───── game_controller ◄── processor_node
    (PID闭环导航)         NavigateToPose action   (视觉伺服)
           │                                     │
═══════════╪═════════════════════════════════════╪════════════
           │              执行层                  │
═══════════╪═════════════════════════════════════╪════════════
           ▼                                     ▼
       /cmd_vel                        /serial/chassis_cmd
           │                           (视觉微调指令)
           ▼                                     │
    cmd_vel_bridge ───────────────────────────────┘
    /serial/chassis_cmd (合并)
           │
           ▼
    auto_serial_bridge
    [0x5A,0xA5,0x01, vx, vy, rot, CRC8]
           │
           ▼
        STM32 底盘控制器
```

## 4. 滤波与控制算法

### 4.1 EKF (扩展卡尔曼滤波) — 底盘全局定位

**位置**: `rc2026_navigation/config/ekf.yaml` (robot_localization 包)

| 属性 | 值 |
|------|-----|
| 滤波对象 | 机器人自身位姿 (x, y, yaw, vx, vy, vyaw) |
| 坐标系 | odom (全局) |
| 输入 | 编码器位置+速度、FAST-LIO位置+航向、IMU角速度+加速度 |
| 输出 | /odom + TF(odom→base_link) |
| 频率 | 50Hz |
| 作用 | 融合多源定位，提供稳定、高精度、高频率的位姿估计 |

### 4.2 2D 卡尔曼滤波 — 视觉目标平滑

**位置**: `decision_processor/kalman_filter.py`

| 属性 | 值 |
|------|-----|
| 滤波对象 | 视觉检测到的目标位置 (base_link 相对坐标) |
| 坐标系 | base_link (机器人体坐标) |
| 输入 | 每帧相机检测的目标 (x, y) |
| 输出 | 平滑后的目标位置 → robot_decision |
| 频率 | 10-30Hz (跟随相机帧率) |
| 作用 | 消除视觉检测抖动，防止底盘震荡 |

附加功能: `is_valid_detection()` 跳变检测，单帧跳变 >0.8m 直接丢弃

### 4.3 PID 控制 — 底盘运动

**位置**: `decision_processor/waypoint_navigator.py`

| 属性 | 值 |
|------|-----|
| 控制对象 | 底盘速度 (vx, vy, omega) |
| 输入 | 位置误差 (ex, ey, e_yaw) 来自 TF |
| 输出 | /cmd_vel (Twist) |
| 频率 | 20Hz |
| 类型 | P 控制 + 减速区 (接近目标时线性减速) |
| 参数 | kp_linear=1.2, kp_angular=2.0, decel_distance=0.3m |

## 5. TF 树

```
                    map
                     │
              (静态 identity)
                     │
                    odom
                     │
            (EKF: robot_localization)
            (融合编码器+FAST-LIO+IMU)
                     │
                base_footprint
                     │
               (固定: z=wheel_radius)
                     │
                 base_link ──────────────────────────────────┐
                  │    │    │    │                            │
            (固定) (固定) (固定) (固定)                   (固定)
              │      │      │      │                        │
         camera_link  lidar_link  arm_base_link    wheel_fl/fr/rl/rr
              │
         (固定: -π/2旋转)
              │
       camera_optical_frame
```

## 6. 真机启动命令

```bash
# 完整导航 (FAST-LIO + EKF + WaypointNavigator)
ros2 launch rc2026_bringup navigation.launch.py

# 完整导航 + 比赛状态机
ros2 launch rc2026_bringup navigation.launch.py use_game_controller:=true

# 右半场 (x 取反)
ros2 launch rc2026_bringup navigation.launch.py start_x:=1.4

# 仅传感器 (调试)
ros2 launch rc2026_bringup sensor_only.launch.py
```

---

# 二、sim_ws（仿真工作空间）

## 1. 代码结构

```
sim_ws/src/
├── rc2026_sim/              # 仿真主包
│   ├── launch/
│   │   ├── simulation.launch.py       # 一键启动仿真
│   │   └── waypoint_nav_sim.launch.py # 仿真导航配置
│   ├── urdf/
│   │   └── rc2026_robot_sim.urdf.xacro # 仿真URDF (含Gazebo插件)
│   ├── map/                           # 地图文件 (备用)
│   └── rc2026_sim/
│       ├── __init__.py
│       └── simple_teleop.py           # 简易键盘遥控
│
└── rc2026_field/            # 比赛场地包 (开源)
    ├── worlds/
    │   ├── robocon2026.world          # 基础场地
    │   └── robocon2026_with_kfs.world # 含KFS方块场地
    ├── config/
    │   └── kfs_config.yaml            # KFS位置配置
    ├── rc2026_field/
    │   ├── kfs_manager.py             # KFS方块管理
    │   └── field_gui.py               # 场地GUI
    └── scripts/
        └── random_kfs_*.py            # 随机KFS生成
```

## 2. 仿真模块功能

### 2.1 Gazebo 插件 (URDF 内定义)

| 插件 | 功能 | 发布话题 |
|------|------|----------|
| `libgazebo_ros_planar_move.so` | 全向底盘驱动，订阅 /cmd_vel | /odom + TF(odom→base_footprint) |
| `libgazebo_ros_camera.so` | RGB相机仿真 | /camera/camera/color/image_raw |
| `libgazebo_ros_ray_sensor.so` (3D) | 360°点云 (16层) | /livox/lidar (PointCloud2) |
| `libgazebo_ros_ray_sensor.so` (2D) | 2D激光扫描 | /scan (LaserScan) |

注: `frame_upper` 的 collision 已移除，防止激光雷达射线命中自身框架。

### 2.2 坐标变换 (Gazebo ↔ Game)

Gazebo 坐标系与 game 坐标系存在轴交换：

```
Gazebo +x = Game -y      Game +x = Gazebo +y
Gazebo +y = Game +x      Game +y = Gazebo -x
Gazebo +z = Game +z      (z轴相同)

公式:
  game_x = gz_y
  game_y = -gz_x + 6.0
  game_yaw = gz_yaw - π/2

逆变换:
  gz_x = 6.0 - game_y
  gz_y = game_x
  gz_yaw = game_yaw + π/2

验证 (左半场启动点):
  Gazebo (5.6, -1.4, yaw=π) → Game (-1.4, 0.4, yaw=π/2) ✓
```

支持两种坐标模式切换测试:
- `coord_mode=gazebo`: 直接轴交换 (默认)
- `coord_mode=fastlio`: 模拟真机 FAST-LIO 模式 (机器人从 0,0,0 开始)

### 2.3 mock_feedback_node (仿真反馈模拟)

模拟真机上不存在的硬件反馈：

| 模拟功能 | 触发条件 | 延时 |
|----------|---------|------|
| 视觉对齐完成 | game_phase=ALIGN_WEAPON/ALIGN_KFS | 1.5s |
| 动作组执行完成 | 收到 /serial/action_group_cmd | 1.5s |
| 组装完成 | game_phase=WAIT_ASSEMBLY | 2.0s |
| R1进入梅林信号 | game_phase=WAIT_ENTER_MERLIN | 2.0s |
| 梅林爬升瞬移 | 收到 /serial/meilin_cmd | 即时 (Gazebo teleport) |

梅林爬升瞬移: 通过 Gazebo `/set_entity_state` 服务，将机器人瞬移到目标台阶的正确高度。

### 2.4 仿真信息流

```
═══════════════════════════════════════════════════════════════
                    Gazebo 物理仿真
═══════════════════════════════════════════════════════════════
  planar_move 插件              ray_sensor 插件
  订阅 /cmd_vel                 发布 /livox/lidar, /scan
  发布 /odom                    camera 插件
  发布 TF: odom→base_footprint  发布 /camera/.../image_raw
       │
═══════╪══════════════════════════════════════════════════════
       │              TF 层
═══════╪══════════════════════════════════════════════════════
       │
  static TF: map→odom
  (gazebo模式: identity)
  (fastlio模式: 抵消出生位姿)
       │
       ▼
  TF: map→base_link
       │
═══════╪══════════════════════════════════════════════════════
       │              导航层 (与真机共用代码)
═══════╪══════════════════════════════════════════════════════
       ▼
  waypoint_navigator        game_controller      mock_feedback
  (PID闭环, 坐标变换)      (比赛状态机)         (模拟STM32反馈)
       │                         │                    │
       ▼                         │                    │
   /cmd_vel ──────────────────────────────────────────┘
       │                                         (梅林瞬移:
       ▼                                          set_entity_state)
  Gazebo planar_move
  (机器人移动)
```

## 3. 仿真启动命令

```bash
# 纯导航测试 (手动发目标)
ros2 launch rc2026_sim simulation.launch.py

# 完整比赛流程
ros2 launch rc2026_sim simulation.launch.py use_game_controller:=true

# 测试 fastlio 坐标模式
ros2 launch rc2026_sim simulation.launch.py coord_mode:=fastlio

# 手动发送导航目标 (game 坐标)
ros2 topic pub --once /waypoint_nav/goal_pose geometry_msgs/PoseStamped \
  "{pose: {position: {x: -0.65, y: 1.55}, orientation: {w: 1.0}}}"

# 手动发送 KFS 配置 + 启动信号
ros2 topic pub --once /game/kfs_input std_msgs/String "data: 'real:5,8 fake:2,6'"
ros2 topic pub --once /game/start_signal std_msgs/UInt8 "data: 1"

# 监控
ros2 topic echo /game/phase          # 比赛阶段
ros2 topic echo /waypoint_nav/status # 导航状态
ros2 run tf2_ros tf2_echo map base_link  # 机器人位置
```

---

# 三、关键话题汇总

| 话题 | 类型 | 发布者 | 订阅者 | 用途 |
|------|------|--------|--------|------|
| `/cmd_vel` | Twist | waypoint_navigator | cmd_vel_bridge / Gazebo | 底盘速度指令 |
| `/serial/chassis_cmd` | Twist | cmd_vel_bridge / processor_node | auto_serial_bridge | STM32底盘指令 |
| `/odom` | Odometry | EKF / Gazebo | (TF发布) | 融合里程计 |
| `/odom/lidar` | Odometry | FAST-LIO | EKF | 激光里程计 |
| `/odom/wheel` | Odometry | wheel_odom_publisher | EKF | 轮式里程计 |
| `/livox/lidar` | PointCloud2 | Livox驱动 / Gazebo | FAST-LIO | 点云数据 |
| `/livox/imu` | Imu | Livox驱动 | FAST-LIO / EKF | IMU数据 |
| `/vision/raw_target` | PointStamped | detector_node | processor_node | 视觉检测目标 |
| `/game/phase` | String | game_controller | processor_node / mock_feedback | 比赛阶段 |
| `/decision/state_id` | Int8 | processor_node / mock_feedback | waypoint_navigator / game_controller | 视觉对齐状态 |
| `/waypoint_nav/status` | String | waypoint_navigator | (监控) | 导航状态 |
| `/waypoint_nav/goal_pose` | PoseStamped | (手动) | waypoint_navigator | 导航目标 |
| `/waypoint_nav/servo_phase` | String | (手动/game_controller) | waypoint_navigator | 视觉伺服阶段 |
| `/feedback/wheel_odom` | Twist | auto_serial_bridge | wheel_odom_publisher | 编码器原始数据 |
| `/feedback/action_group` | UInt8 | auto_serial_bridge / mock | game_controller | 动作完成反馈 |
| `/feedback/assembly` | UInt8 | auto_serial_bridge / mock | game_controller | 组装状态 |
| `/game/start_signal` | UInt8 | auto_serial_bridge / mock | game_controller | 启动信号 |
| `/game/r1_signal` | UInt8 | auto_serial_bridge / mock | game_controller | R1通信信号 |
| `navigate_to_pose` | Action | game_controller | waypoint_navigator | 导航目标 (Action) |
