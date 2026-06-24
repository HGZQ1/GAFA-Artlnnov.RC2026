# RC2026 整机调参手册

> 调参顺序：TF外参 → FAST-LIO建图 → EKF融合 → PID速度控制 → 决策层视觉参数
> 每一步未稳定前不要进入下一步

---

## 目录

1. [文件速查表](#1-文件速查表)
2. [TF 变换外参标定](#2-tf-变换外参标定)
3. [IMU 外参标定](#3-imu-外参标定)
4. [FAST-LIO 建图与定位](#4-fast-lio-建图与定位)
5. [EKF 融合滤波](#5-ekf-融合滤波)
6. [PID 速度控制](#6-pid-速度控制)
7. [决策层视觉参数](#7-决策层视觉参数)
8. [PID / EKF 可视化调参工具](#8-pid--ekf-可视化调参工具)
9. [整机调试顺序建议](#9-整机调试顺序建议)

---

## 1. 文件速查表

| 子系统 | 配置文件（相对 ros2_ws/src） | 作用 |
|--------|------------------------------|------|
| LiDAR TF | `rc2026_bringup/config/measurement_params.yaml` | base_link → livox_frame 静态变换 |
| 传感器安装 | `rc2026_bringup/config/robot_params.yaml` | 相机/LiDAR/机械臂安装偏移（与URDF同步）|
| URDF TF树 | `rc2026_bringup/urdf/rc2026_robot.urdf.xacro` | 所有 link/joint 的几何关系，供 robot_state_publisher |
| FAST-LIO建图 | `rc2026_navigation/config/fastlio2/mapping.yaml` | 建图模式参数，含 IMU 外参 |
| FAST-LIO定位 | `rc2026_navigation/config/fastlio2/localization.yaml` | 定位模式参数 |
| EKF融合 | `rc2026_navigation/config/ekf.yaml` | 轮式里程计 + FAST-LIO + IMU 融合 |
| PID速度 | `rc2026_bringup/config/robot_params.yaml` (`pid` 节) | 导航控制器 PID 增益 |
| Bridge限幅 | `cmd_vel_bridge/config/pid_params.yaml` | 速度硬限幅 + 源超时 |
| 决策参数 | `decision_processor/decision_processor/config.py` | 视觉对齐、航点坐标、超时保护 |

---

## 2. TF 变换外参标定

TF树链路（从上到下）：
```
map → odom → base_footprint → base_link → camera_link
                                         → lidar_link (= livox_frame)
                                         → arm_base_link
```

### 2-A 激光雷达 → base_link

**文件：** `rc2026_bringup/config/measurement_params.yaml`

```yaml
base_link2livox_frame:
  xyz: "0.0 0.0 0.55"   # 前后 侧向 高度，单位米，用卷尺实测后修改
  rpy: "0.0 0.0 0.0"    # 滚转 俯仰 偏航，弧度，LiDAR水平安装时全为0
```

**同步修改：** `robot_params.yaml` → `lidar.offset_x/y/z` 和 `lidar.roll/pitch/yaw`
**同步修改：** `rc2026_robot.urdf.xacro` → `xacro:arg name="lidar_x/y/z"` 默认值

三处必须保持一致，否则 FAST-LIO 输出的位姿和 TF 树会产生偏差。

**测量方法：**
```
卷尺从底盘中心（base_link原点，轮子接地面正中）量到 LiDAR 中心
  x: 正前方为正，向后为负
  y: 正左方为正，向右为负
  z: 从地面算（含轮子半径 0.054m）
```

---

### 2-B 相机 (D435i) → base_link

**文件（三处同步）：**

| 文件 | 字段 |
|------|------|
| `rc2026_bringup/urdf/rc2026_robot.urdf.xacro` | `xacro:arg name="camera_x/y/z"` 默认值，以及 `camera_joint` 的 `rpy` |
| `rc2026_bringup/config/robot_params.yaml` | `camera.offset_x/y/z`，`camera.roll/pitch/yaw` |
| `decision_processor/decision_processor/config.py` | `CAMERA_OFFSET_X/Y/Z`，`CAMERA_PITCH_RAD` |

**关键参数说明：**

| 参数 | 含义 | 典型值 |
|------|------|--------|
| `offset_x` | 相机相对底盘中心的前后偏移 (m) | `0.05`（前方5cm）|
| `offset_y` | 相机横向偏移 (m，正=左) | `0.0` |
| `offset_z` | 相机高度（从底盘底面/地面算）(m) | `0.45` |
| `pitch` | 相机俯仰角 (弧度，负=朝下) | `0.0`（水平）或 `-0.26`（-15°）|

> **注意：** 若相机朝下安装，`pitch` 为负值（如 `-0.262` ≈ -15°），
> 三个文件中的 `camera_joint rpy` / `CAMERA_PITCH_RAD` 必须一致。
> 否则视觉对齐计算出的距离/角度会有系统性偏差。

---

### 2-C 武器臂 / KFS臂 → base_link（占位，需现场测量）

**文件：** `rc2026_bringup/urdf/rc2026_robot.urdf.xacro`

```xml
<!-- 武器臂：joint name="weapon_arm_joint" -->
<origin xyz="0.05 0.15 0.30" rpy="0 0 0"/>   <!-- ← 现场用卷尺量后替换 -->

<!-- KFS臂：joint name="arm_base_joint" -->
<origin xyz="$(arg arm_x) $(arg arm_y) $(arg arm_z)" .../>
```

同步修改 `robot_params.yaml` → `arm.offset_x/y/z`
同步修改 `config.py` → `ARM_OFFSET_X/Y/Z`

---

## 3. IMU 外参标定

### Mid-360S 内置 IMU（与 LiDAR 同体）

**文件：** `rc2026_navigation/config/fastlio2/mapping.yaml`

```yaml
mapping:
  extrinsic_est_en: false         # false=手动填外参，true=FAST-LIO自动估计
  extrinsic_T: [0.0, 0.0, 0.0]   # IMU中心相对LiDAR中心的平移 (米)
  extrinsic_R: [1.0, 0.0, 0.0,   # IMU→LiDAR 旋转矩阵（行优先3×3）
                0.0, 1.0, 0.0,   # Mid-360S同体，通常为单位阵
                0.0, 0.0, 1.0]
```

Mid-360S 的 IMU 与 LiDAR 封装在同一外壳内，官方外参接近零，一般不需要修改。

**若使用外挂 IMU（非 Mid-360S 内置），测量方法：**
1. 将 `extrinsic_est_en: true`，让 FAST-LIO 在建图时自动估计
2. 建图完成后从日志中读取估计值，填回并改回 `false`

**EKF 中的 IMU 融合配置：**

**文件：** `rc2026_navigation/config/ekf.yaml`

```yaml
imu0: /livox/imu
imu0_config: [false, false, false,    # 位置 xyz — IMU不提供
              false, false, false,    # 姿态 roll/pitch/yaw — FAST-LIO已提供，不重复融合
              false, false, false,    # 速度
              true,  true,  true,     # 角速度 ← 保留，补偿帧间空档
              true,  true,  false]    # 线加速度 ax/ay ← 保留；az含重力不用
imu0_remove_gravitational_acceleration: true   # 融合前自动去除重力分量
```

**爬梅林时 IMU 建议调整：**
```yaml
# 坡面上 ax/ay 含有重力分量变化，可能引入噪声
# 如果爬坡时位置漂移，把 ax/ay 改为 false
imu0_config: [..., false, false, false]   # 最后三项全 false
```

---

## 4. FAST-LIO 建图与定位

### 4-A 建图模式

**文件：** `rc2026_navigation/config/fastlio2/mapping.yaml`

| 参数 | 当前值 | 含义 | 调参建议 |
|------|--------|------|----------|
| `blind` | `0.5` | 近距离滤除半径(m)，过滤底盘自身点云 | 出现地板/底盘噪点 → 增大到 0.8 |
| `point_filter_num` | `3` | 点云降采样（每N个点保留1个） | CPU不足 → 改 5；精度不够 → 改 1 |
| `det_range` | `100.0` | 有效点云范围(m)，超出丢弃 | 室内场地可缩小到 20m |
| `filter_size_surf` | `0.5` | ikd-tree体素滤波分辨率(m) | 精度不够 → 减小到 0.3；CPU高 → 增大 |
| `filter_size_map` | `0.5` | 地图维护分辨率(m) | 同上 |
| `acc_cov` | `0.1` | IMU加速度测量噪声协方差 | IMU噪声大 → 增大到 0.5 |
| `gyr_cov` | `0.1` | IMU陀螺仪测量噪声协方差 | 同上 |
| `b_acc_cov` | `0.001` | 加速度偏置随机游走 | 偏置漂移明显 → 略增大 |
| `b_gyr_cov` | `0.001` | 陀螺仪偏置随机游走 | 同上 |
| `max_iteration` | `3` | ICP/IEKF 最大迭代次数 | 精度不够 → 改 5；速度优先 → 保持 3 |

**建图步骤：**
```bash
# 1. 启动建图
ros2 launch rc2026_navigation fastlio.launch.py mode:=mapping

# 2. 手动推动/驾驶机器人绕场地一圈（不能太快，建议 < 0.5 m/s）
# 3. 等待地图收敛（rviz中点云不再跳动）
# 4. 保存地图
ros2 service call /map_save std_srvs/srv/Empty {}
# 或直接 Ctrl+C，pcd 自动保存到 ~/.ros/
```

---

### 4-B 定位模式（与建图参数差异）

**文件：** `rc2026_navigation/config/fastlio2/localization.yaml`

| 参数 | 定位值 | 与建图的差异 |
|------|--------|--------------|
| `point_filter_num` | `4` | 更激进降采样，减少实时计算量 |
| `det_range` | `30.0` | 缩小范围，只用近处特征定位 |
| `map_publish_en` | `false` | 定位不更新地图 |
| `pcd_save_en` | `false` | 定位不保存点云 |

---

## 5. EKF 融合滤波

**文件：** `rc2026_navigation/config/ekf.yaml`

EKF 融合三路来源：
- **odom0**（`/odom/wheel`）：STM32 轮式编码器里程计，高频低精度
- **odom1**（`/Odometry`）：FAST-LIO 激光里程计，低频高精度
- **imu0**（`/livox/imu`）：Mid-360S IMU，补偿两路里程计之间的空档

### 5-A 数据源配置矩阵

每行对应一个状态量，`true`=融合该量，`false`=忽略：

```yaml
# 顺序: [x, y, z, roll, pitch, yaw, vx, vy, vz, vroll, vpitch, vyaw, ax, ay, az]

odom0_config:   # 轮式里程计
  [true,  true,  false,   # x y — 用；z — 不用（2D）
   false, false, true,    # roll pitch — 不用；yaw — 用
   true,  true,  false,   # vx vy — 用；vz — 不用
   false, false, true,    # vroll vpitch — 不用；vyaw — 用
   false, false, false]   # 加速度 — 不用

odom1_config:   # FAST-LIO（位姿精度高，速度不用）
  [true,  true,  false,
   false, false, true,
   false, false, false,   # 速度全不用
   false, false, false,
   false, false, false]
```

### 5-B 过程噪声协方差（Q矩阵）

对角线15个元素，顺序同上，**值越大 = 越不相信模型预测 = 更依赖观测数据**：

```yaml
# 关键元素（当前值 → 调整建议）
x, y:       0.05   # 导航漂移大 → 增大到 0.1
yaw:        0.06   # 旋转漂移大 → 增大到 0.1~0.2
vx, vy:     0.025  # 编码器打滑 → 增大到 0.05
vyaw:       0.02   # 转速不稳定 → 增大
```

### 5-C 异常值拒绝阈值

```yaml
odom0_rejection_threshold: 2.0   # 编码器跳变超过2σ时拒绝该帧
odom1_rejection_threshold: 2.0   # FAST-LIO跳变时拒绝
```

**建议值：**
- 编码器噪声大 → 提高到 `3.0~5.0`
- FAST-LIO 偶发跳帧 → 提高 `odom1_rejection_threshold` 到 `3.0`
- 环境 LiDAR 特征少（开阔空地）→ 降低 `odom1_rejection_threshold` 到 `1.5`，减小 FAST-LIO 权重

### 5-D 初始不确定性

```yaml
initial_estimate_covariance:  # 全1e-9（极小）= 相信启动位置完全精确
```

如果每次启动机器人位置不固定，把 `x/y` 对应的 `1e-9` 改为 `0.5`：
```yaml
# 第1行第1列（x）和第2行第2列（y）
1e-9 → 0.5
```

---

## 6. PID 速度控制

### 6-A 导航 PID 增益

**文件：** `rc2026_bringup/config/robot_params.yaml`（`pid` 节）

```yaml
pid:
  linear:
    kp: 0.8     # 比例增益：误差1m时输出0.8 m/s；增大=响应更快但易超调
    ki: 0.3     # 积分增益：消除稳态误差（到达目标点附近停不住→增大ki）
    kd: 0.02    # 微分增益：抑制超调（底盘到点来回震荡→增大kd）
    max_output: 1.5    # 最大线速度限幅 (m/s)，不要超过 chassis.max_linear_vel
    deadband: 0.01     # 误差小于1cm时停止输出，防止底盘微颤
  angular:
    kp: 1.5     # 角速度比例增益
    ki: 0.2
    kd: 0.01
    max_output: 3.14   # 最大角速度限幅 (rad/s ≈ 180°/s)
    deadband: 0.02     # 误差小于0.02rad（≈1.1°）时停止输出
```

**PID 快速调参口诀：**
```
1. 先把 ki=0, kd=0，只调 kp，直到底盘能朝目标运动但会超调
2. 加 kd 抑制超调（每次增加 0.01），直到不再震荡
3. 最后加小量 ki（0.05~0.1），消除到达目标前的稳态误差
4. 如果底盘在目标点附近反复小幅抖动，增大 deadband
```

---

### 6-B Bridge 速度硬限幅

**文件：** `cmd_vel_bridge/config/pid_params.yaml`

```yaml
cmd_vel_bridge:
  ros__parameters:
    max_linear_vel: 1.5     # 所有来源的线速度上限，超过直接截断
    max_angular_vel: 3.14   # 角速度上限
    control_rate: 50.0      # 发布频率 Hz
    cmd_vel_timeout: 0.5    # 某路来源超过0.5s无更新则切换到低优先级
```

---

## 7. 决策层视觉参数

**文件：** `decision_processor/decision_processor/config.py`（快速调试区）

```python
# 视觉对齐
ALIGN_THRESHOLD_DEG  = 5.0     # 对齐容差(度)，越小越精准但越难到达
ALIGN_TURN_GAIN      = 1.0     # 转向速度增益，> 1 转更快，< 1 更慢
STOP_DISTANCE_M      = 0.50    # 停在距武器多远触发抓取(m)
FORWARD_SPEED_GAIN   = 1.0     # 前进速度增益

# 目标识别
WUGUAN_CONF_MIN      = 0.70    # YOLO置信度阈值，太高会漏检
CONFIRM_FRAMES       = 3       # 连续N帧确认目标锁定，增大=更稳但反应慢
TARGET_TIMEOUT_S     = 0.5     # 目标消失超过此时间视为丢失

# 超时保护（全在 config.py 超时区块）
KFS_PLACE_STOP_WAIT_S = 0.3   # KFS放置点底盘停稳等待时间
KFS_PLACE_CMD_DELAY_S = 0.5   # 放置动作指令发送窗口
FINE_ALIGN_TIMEOUT_S  = 15.0  # 精对齐超时兜底
DOCK_ALIGN_TIMEOUT_S  = 30.0  # 合体对齐超时兜底
MERLIN_CLIMB_WAIT_S   = 3.0   # 爬升等待时间
```

---

## 8. PID / EKF 可视化调参工具

### 8-A PlotJuggler（推荐，实时曲线）

ROS2 最好用的实时曲线工具，可以同时看多个 topic 的波形。

```bash
# 安装
sudo apt install ros-humble-plotjuggler-ros

# 启动
ros2 run plotjuggler plotjuggler
```

**PID 调参时订阅的 topic：**

| Topic | 内容 | 用途 |
|-------|------|------|
| `/cmd_vel` | 导航控制器输出的目标速度 | 看期望速度曲线 |
| `/serial/chassis_cmd` | 实际下发给底盘的速度 | 看实际执行速度 |
| `/feedback/wheel_odom` | 编码器反馈速度 | 对比期望与实际，调 kp/kd |
| `/odometry/filtered` | EKF 输出位姿 | 看融合后轨迹是否平滑 |

**操作方法：**
1. 打开 PlotJuggler → `Streaming` → `ROS2 Topic Subscriber` → Start
2. 拖拽 `/cmd_vel/linear/x` 和 `/feedback/wheel_odom/linear/z`（vx）到同一图
3. 让机器人跑直线，看两条曲线的跟随误差
4. 增大 `kp` 直到跟随误差小，若出现震荡则增大 `kd`

**EKF 调参时订阅：**

| Topic | 内容 |
|-------|------|
| `/odometry/filtered` | EKF 融合输出（主要看） |
| `/odom/wheel` | 纯编码器里程计 |
| `/Odometry` | 纯 FAST-LIO 里程计 |

三条曲线叠加看，`/odometry/filtered` 应该比两者都平滑。

---

### 8-B rqt_plot（轻量级，ROS2 自带）

```bash
# 启动
ros2 run rqt_plot rqt_plot

# 或直接 rqt
rqt
```

直接在 Topic Monitor 里输入 topic 名，支持实时折线图。比 PlotJuggler 轻但功能少。

**快速看 PID 响应：**
```bash
# 终端里对比两个值
ros2 topic echo /odometry/filtered --field pose.pose.position.x
ros2 topic echo /odom/wheel --field pose.pose.position.x
```

---

### 8-C RViz2（轨迹和位姿可视化）

```bash
ros2 run rviz2 rviz2
```

**EKF 调参必看的 Display：**

| Display 类型 | Topic | 作用 |
|-------------|-------|------|
| Odometry | `/odometry/filtered` | EKF 输出轨迹（带协方差椭圆）|
| Odometry | `/odom/wheel` | 编码器轨迹，看漂移 |
| Odometry | `/Odometry` | FAST-LIO 轨迹 |
| Path | `/path` | FAST-LIO 完整路径 |
| PointCloud2 | `/livox/lidar` | 实时点云，看有无异常 |

协方差椭圆越大 = EKF 越不确定当前位置 → 说明传感器数据噪声大或外参有误差

---

### 8-D robot_localization 内置诊断

EKF 节点自带诊断输出：

```bash
# 查看 EKF 状态
ros2 topic echo /diagnostics

# 查看 EKF 详细日志（需要 ekf.yaml 中 print_diagnostics: true）
ros2 run rqt_runtime_monitor rqt_runtime_monitor
```

---

### 8-E 快速 PID 整定命令行方法

不依赖 GUI，直接在命令行看效果：

```bash
# 让机器人前进1m，看实际走了多远
ros2 topic pub --once /cmd_vel geometry_msgs/msg/Twist \
  '{linear: {x: 0.3}, angular: {z: 0.0}}'

# 同时监控编码器反馈
ros2 topic echo /feedback/wheel_odom

# 或监控EKF位置变化
ros2 topic echo /odometry/filtered --field pose.pose.position
```

---

### 8-F pid_controller 包（可选，替代手写PID）

如果后续想用 ROS2 标准 PID 包：

```bash
sudo apt install ros-humble-pid-controller   # 提供标准 PID 节点
sudo apt install ros-humble-control-toolbox  # PID实现库
```

该包支持动态参数调整（`ros2 param set`），无需重启节点即可改 kp/ki/kd：

```bash
# 运行时调整 PID 参数（需要节点支持动态参数）
ros2 param set /your_pid_node kp 1.2
ros2 param set /your_pid_node kd 0.05
```

---

## 9. 整机调试顺序建议

```
阶段一：静态标定
  ① 用卷尺量 LiDAR 安装位置 → 修改 measurement_params.yaml
  ② 用卷尺量 D435i 安装位置 → 修改 URDF + robot_params.yaml + config.py（三处同步）
  ③ 启动 robot_state_publisher，RViz 中确认 TF 树无断链

阶段二：建图
  ④ 启动 FAST-LIO 建图模式，推动机器人绕场地一圈
  ⑤ RViz 中检查地图质量（无重影、墙面清晰）
  ⑥ 若地图漂移，检查 IMU 外参 extrinsic_T/R；若点云噪声多，减小 blind

阶段三：EKF 融合
  ⑦ 启动 EKF，PlotJuggler 同时看三路里程计
  ⑧ 让机器人走直线回原点，看 /odometry/filtered 漂移量
  ⑨ 若漂移大，调整 odom0/odom1_rejection_threshold 和 process_noise_covariance

阶段四：PID 速度控制
  ⑩ 先调线速度 PID：kp=0.5 开始，逐步增大到不震荡为止
  ⑪ 再调角速度 PID：同样从 kp=1.0 开始
  ⑫ PlotJuggler 对比 /cmd_vel 和 /feedback/wheel_odom，误差小于 5% 为合格

阶段五：决策层
  ⑬ 调 STOP_DISTANCE_M：让机器人对准武器头，看相机到武器距离，减去5cm作为停止距离
  ⑭ 调 ALIGN_THRESHOLD_DEG：视觉对齐精度要求，比赛中 5° 足够
  ⑮ 调 ALIGN_TURN_GAIN：若转速过快超调，减小到 0.5~0.7
```

---

*最后修改：2026-06-24*
