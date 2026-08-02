# RC2026 底盘测试指南

> 适用平台: Jetson Orin / Ubuntu 22.04 · ROS2 Humble  
> 串口协议: `/serial/chassis_cmd` (geometry_msgs/Twist) · 50 Hz  
> `linear.x`=vx(m/s)  `linear.y`=vy(m/s)  `angular.z`=旋转角速度(deg/s, 正=逆时针)

---

## 目录

1. [环境准备](#1-环境准备)
2. [启动串口桥](#2-启动串口桥)
3. [单条指令测试](#3-单条指令测试)
4. [连续路线测试 (chassis_test.py)](#4-连续路线测试)
5. [EKF + FAST-LIO 定位测试](#5-ekf--fast-lio-定位测试)
6. [底盘 PID 调参测试](#6-底盘-pid-调参测试)
7. [参数速查表](#7-参数速查表)

---

## 1 环境准备

```bash
# 每个终端启动前执行
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
source ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws/install/setup.bash
```

> 建议写入 `~/.bashrc`，之后直接开终端使用。

---

## 2 启动串口桥

**终端 A**（每次测试必须先启动）：

```bash
ros2 launch auto_serial_bridge serial_bridge_by_node.launch.py
```

验证通信正常：

```bash
# 确认底盘指令话题存在
ros2 topic list | grep chassis_cmd

# 查看轮式里程计（串口在线时应有输出）
ros2 topic echo /feedback/wheel_odom
```

串口设备默认 `/dev/ttyUSB0`，如需修改在 `auto_serial_bridge-main/config/protocol.yaml` 中改 `port`。

---

## 3 单条指令测试

> 以下命令在**独立终端 B**中执行，每条只发一帧，STM32 超时后自动停止（约 200ms）。  
> 如需 50 Hz 持续发送，加 `--rate 50` 参数。

### 3.1 直线前进 / 后退

```bash
# 前进 0.2 m/s（单帧确认底盘响应）
ros2 topic pub --once /serial/chassis_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"

# 后退 0.2 m/s
ros2 topic pub --once /serial/chassis_cmd geometry_msgs/msg/Twist \
  "{linear: {x: -0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

**50 Hz 持续前进**（Ctrl+C 停止）：

```bash
ros2 topic pub --rate 50 /serial/chassis_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.2, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.2 原地旋转

```bash
# 逆时针旋转，angular.z = 30.0 deg/s（正值=逆时针）
ros2 topic pub --rate 50 /serial/chassis_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 30.0}}"

# 顺时针旋转
ros2 topic pub --rate 50 /serial/chassis_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.0, z: 0.0}, angular: {x: 0.0, y: 0.0, z: -30.0}}"
```

### 3.3 横移（麦轮）

```bash
# 向左横移 0.2 m/s（linear.y 正方向 = 左）
ros2 topic pub --rate 50 /serial/chassis_cmd geometry_msgs/msg/Twist \
  "{linear: {x: 0.0, y: 0.2, z: 0.0}, angular: {x: 0.0, y: 0.0, z: 0.0}}"
```

### 3.4 走特定距离（时间换距离）

此脚本为开环控制，用**时间换距离**公式（decision_processor 航点导航读 `/odom` 有闭环，但测试脚本不走导航层）：

```
发包时长(s) = 目标距离(m) ÷ 速度(m/s)
帧数        = 发包时长 × 50
```

示例：以 0.2 m/s 前进 1 m，需发送 **5.0 秒 × 50 Hz = 250 帧**。  
推荐用第 4 节脚本精准控制，避免 `--once` 丢帧。

---

## 4 连续路线测试

`scripts/chassis_test.py` 以 50 Hz 连续发送，路段之间**零间隔**，无抖动停顿。

```bash
cd ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project
python3 scripts/chassis_test.py <模式>
```

| 模式 | 路线描述 |
|------|---------|
| `straight` | 直线前进 1 m |
| `back_forth` | 前进 1 m → 后退 1 m |
| `l_shape` | 前进 1 m → 逆时针 90° → 前进 1 m |
| `square` | 正方形闭环（4×前进 1 m + 4×旋转 90°） |
| `custom` | 自定义（编辑脚本 `ROUTES['custom']` 段落） |

**自定义路线格式**（编辑 `scripts/chassis_test.py` 中的 `ROUTES['custom']`）：

```python
'custom': [
    {'label': '前进0.5m', 'vx': 0.2,  't': 2.5},
    {'label': '左移0.3m', 'vy': 0.2,  't': 1.5},
    {'label': '旋转90°',  'az': 30.0, 't': 3.0},
],
```

字段说明：

| 字段 | 含义 | 单位 |
|------|------|------|
| `vx` | 前进速度（正=前，负=后） | m/s |
| `vy` | 横移速度（正=左，负=右） | m/s |
| `az` | 旋转角速度（正=逆时针，负=顺时针） | deg/s |
| `t` | 本段持续时间 | s |

---

## 5 EKF + FAST-LIO 定位测试

> 需要激光雷达在线。三终端并行运行。

### 5.1 启动顺序

**终端 A** — 串口桥（已启动则跳过）：

```bash
ros2 launch auto_serial_bridge serial_bridge_by_node.launch.py
```

**终端 B** — FAST-LIO（LiDAR 里程计）：

```bash
ros2 launch fast_lio mapping_mid360.launch.py
```

**终端 C** — EKF（融合轮式里程计 + FAST-LIO）：

```bash
ros2 launch rc2026_bringup ekf.launch.py
```

### 5.2 话题说明

| 话题 | 来源 | 说明 |
|------|------|------|
| `/feedback/wheel_odom` | STM32 | 轮式里程计累积位姿（Twist） |
| `/Odometry` | FAST-LIO | LiDAR 里程计（nav_msgs/Odometry） |
| `/odom` | robot_localization EKF | 融合后最优估计，供导航使用 |

### 5.3 闭环导航误差测试

`chassis_test.py` 内置闭环导航模式，自动 P 控制到达目标、停稳后采样误差，无需手动记录坐标。支持三种来源单独测试，以及三路全启的对比模式。

#### 单来源测试

```bash
cd ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project

# EKF 闭环（需启动串口桥 + EKF，不需要 FAST-LIO）
python3 scripts/chassis_test.py nav_1m_ekf        # 前进 1m
python3 scripts/chassis_test.py nav_back_ekf      # 前进 1m 后返回原点
python3 scripts/chassis_test.py nav_square_ekf    # 3m×3m 正方形闭环

# 纯 FAST-LIO 闭环（需启动串口桥 + FAST-LIO，不需要 EKF）
python3 scripts/chassis_test.py nav_1m_fastlio
python3 scripts/chassis_test.py nav_square_fastlio
```

每段完成后终端打印：

```
  到达  用时: 12.4s
  终点实测: (3.018, 0.023)
  位置误差: 2.9 cm
```

结果保存为 `scripts/nav_<路线>_<来源>_result.csv`

#### 三路全启对比模式（推荐）

同时启动串口桥 + FAST-LIO + EKF，三者全在线后运行对比模式。  
脚本用 EKF 融合结果（`/odom`）控制底盘，同帧记录 `/odom` 和 `/Odometry` 双轨迹。

**启动顺序（三个终端）：**

```bash
# 终端 A — 串口桥
ros2 launch auto_serial_bridge serial_bridge_by_node.launch.py

# 终端 B — FAST-LIO
ros2 launch fast_lio mapping_mid360.launch.py

# 终端 C — EKF（融合轮式里程计 + FAST-LIO）
ros2 launch rc2026_bringup ekf.launch.py
```

**运行对比测试：**

```bash
cd ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project

python3 scripts/chassis_test.py nav_1m_compare       # 前进 1m，双源对比
python3 scripts/chassis_test.py nav_back_compare     # 前进 1m 后返回，双源对比
python3 scripts/chassis_test.py nav_square_compare   # 3m×3m 正方形，双源对比
```

终端输出对比误差表：

```
───────────────────────────────────────────────────────────
  段                    EKF误差(cm)    FAST-LIO误差(cm)
───────────────────────────────────────────────────────────
  目标: 边1终点               2.1               3.8
  目标: 边2终点               3.4               5.1
  目标: 边3终点               2.9               4.3
  目标: 回起点                4.2               7.6
───────────────────────────────────────────────────────────
  EKF 平均误差: 3.2 cm
```

轨迹同时保存为 `scripts/nav_<路线>_compare_traj.csv`（含每帧 EKF 和 FAST-LIO 的 x/y），终端最后给出绘图命令：

```bash
python3 -c "
import pandas as pd, matplotlib.pyplot as plt
df = pd.read_csv('scripts/nav_square_compare_traj.csv')
plt.plot(df['ekf_x'], df['ekf_y'], label='EKF+FAST-LIO融合')
plt.plot(df['fl_x'],  df['fl_y'],  label='纯FAST-LIO')
plt.axis('equal'); plt.grid(True); plt.legend(); plt.show()
"
```

#### 闭环控制参数（`chassis_test.py` 顶部调整）

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `NAV_KP_LINEAR` | 0.8 | 位置误差增益（误差越大速度越快） |
| `NAV_MAX_VX` | 0.25 m/s | 最大前进速度 |
| `NAV_MAX_VY` | 0.20 m/s | 最大横移速度 |
| `NAV_ARRIVE_M` | 0.05 m | 到达判定阈值（5 cm） |
| `NAV_TIMEOUT_S` | 30.0 s | 单段超时兜底 |

### 5.4 三路对比记录表（填写实测值）

路线：`nav_square`（3m×3m 正方形，终点=起点，回到原点误差即漂移量）

| 场景 | 启动节点 | 控制来源 | 平均误差 (cm) | 终点漂移 (cm) |
|------|---------|---------|------------|------------|
| 纯 EKF（轮式里程计） | 串口桥 + EKF | `/odom` | | |
| 纯 FAST-LIO | 串口桥 + FAST-LIO | `/Odometry` | | |
| EKF + FAST-LIO 融合 | 串口桥 + FAST-LIO + EKF | `/odom` | | |

> 三个场景下 `nav_square_ekf` / `nav_square_fastlio` / `nav_square_compare` 分别运行，填写"回起点"段的误差即为最终漂移。

---

## 6 底盘 PID 调参测试

PID 测试发送**阶跃速度指令**，同步采集 `/feedback/wheel_odom` 实际速度反馈，生成 CSV 曲线以评估超调和响应时间。

### 6.1 运行测试

```bash
cd ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project
python3 scripts/chassis_test.py pid_vx   # 前进方向 vx
python3 scripts/chassis_test.py pid_vy   # 横移方向 vy
python3 scripts/chassis_test.py pid_az   # 旋转方向 az
```

脚本三阶段自动执行：

```
阶段 1: 静止基准  0.5 s
阶段 2: 阶跃输入  3.0 s（vx=0 → 0.3 m/s）
阶段 3: 归零观测  2.0 s
```

结果 CSV 保存至 `scripts/pid_<轴>_result.csv`，列：`t`(s) / `cmd`(指令) / `fb`(反馈)

### 6.2 修改测试参数

在 `scripts/chassis_test.py` 顶部 `PID_TESTS` 字典中调整：

```python
'pid_vx': {
    'target':   0.3,   # 阶跃目标值（建议 0.2~0.5 m/s）
    'hold_t':   3.0,   # 保持时长 s
    'settle_t': 2.0,   # 归零后观测时长 s
},
```

### 6.3 绘制响应曲线

```python
import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv('scripts/pid_vx_result.csv')
df.plot(x='t', y=['cmd', 'fb'], title='vx PID Step Response')
plt.xlabel('时间 (s)')
plt.ylabel('速度 (m/s)')
plt.axhline(y=0.3, color='gray', linestyle='--', label='目标值')
plt.grid(True)
plt.legend()
plt.show()
```

### 6.4 判断标准

| 指标 | 良好 | 需调整 |
|------|------|-------|
| 上升时间 | < 200 ms | > 500 ms → 增大 Kp |
| 超调量 | < 10% | > 20% → 减小 Kp 或增大 Kd |
| 稳态误差 | < 2% | > 5% → 小幅增大 Ki |
| 持续震荡 | 无 | 有 → 减小 Kp/Ki，增大 Kd |

### 6.5 典型调参流程

```
① 运行 pid_vx，保存基准曲线
② 在 STM32 固件中增大 Kp（约 +20%），重新编译烧录
③ 再次运行 pid_vx，对比曲线
④ 出现超调 → 同步增大 Kd 压制
⑤ 稳态误差偏大 → 小幅增大 Ki（过大会积分饱和）
⑥ vx / vy / az 三轴分别独立调参
```

> PID 参数在 **STM32 固件**中修改，与 ROS2 层无关。修改后重烧，再次跑脚本验证。

---

## 7 参数速查表

### 视觉对齐参数（`decision_processor/config.py`）

| 参数 | 默认值 | 说明 |
|------|-------|------|
| `ALIGN_THRESHOLD_DEG` | 5.0° | 武器头对齐角度容差 |
| `STOP_DISTANCE_M` | 0.20 m | D435i 到武器头停止距离 |
| `CAM_X_OFFSET_M` | 0.0 m | 武器头对齐 D435i 横向偏移补偿 |
| `KFS_ALIGN_THRESHOLD_DEG` | 5.0° | KFS 粗对齐角度容差 |
| `KFS_STOP_DISTANCE_M` | 0.50 m | D435i 到 KFS 停止距离 |
| `KFS_CAM_X_OFFSET_M` | 0.0 m | KFS 粗对齐 D435i 横向偏移补偿 |
| `CONFIRM_FRAMES` | 3 帧 | 目标锁定确认帧数 |
| `LOST_FRAMES` | 5 帧 | 目标丢失确认帧数 |

### 话题速查

| 话题 | 类型 | 方向 | 说明 |
|------|------|------|------|
| `/serial/chassis_cmd` | geometry_msgs/Twist | Jetson → STM32 | 底盘运动指令 |
| `/feedback/wheel_odom` | geometry_msgs/Twist | STM32 → Jetson | 轮式里程计 |
| `/game/r1_signal` | std_msgs/UInt8 | STM32 → Jetson | R1 信号(2=进入梅林) |
| `/odom` | nav_msgs/Odometry | EKF 输出 | 融合定位 |
| `/Odometry` | nav_msgs/Odometry | FAST-LIO 输出 | LiDAR 定位 |

### 修改后的生效方式

| 文件 | 生效方式 |
|------|---------|
| `decision_processor/*.py` | 重启节点（egg-link 软链，**无需 build**） |
| `scripts/chassis_test.py` | 直接运行新命令 |
| STM32 固件 PID 参数 | 编译烧录固件 |
| `protocol.yaml` | 重启串口桥节点 |
