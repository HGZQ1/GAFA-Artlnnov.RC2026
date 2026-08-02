# RC2026 真实生效调参手册

> 适用启动方式：
>
> ```bash
> cd ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project
> source ros2_ws/install/setup.bash
> python3 scripts/launch_rc2026.py
> ```
>
> 本文按这条真实启动链路写。不要只看文件注释调参，当前项目里有一些历史配置文件和注释已经不再被 `full_system.launch.py` 读取。

---

## 0. 当前真实启动链路

```text
scripts/launch_rc2026.py
  -> ros2 launch rc2026_bringup full_system.launch.py
      -> rc2026_bringup/urdf/rc2026_robot.urdf.xacro
      -> rc2026_navigation/launch/localization.launch.py
          -> livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py
          -> fast_lio/launch/mapping.launch.py config_file:=mid360.yaml
          -> rc2026_navigation/config/ekf.yaml
          -> rc2026_navigation/scripts/map_relocalizer.py
      -> decision_processor/waypoint_navigator.py
      -> decision_processor/processor_node.py
      -> decision_processor/game_controller.py
      -> cmd_vel_bridge/bridge_node.py
      -> auto_serial_bridge, only when enable_serial:=true
```

最重要的结论：

```text
当前主启动 full_system.launch.py 不读取 robot_params.yaml。
当前主启动 full_system.launch.py 不读取 measurement_params.yaml。
当前主启动 full_system.launch.py 不读取 cmd_vel_bridge/config/pid_params.yaml。
当前主启动 FAST-LIO 读取的是外部 fast_lio 包里的 mid360.yaml。
```

---

## 1. 一张表看懂改哪里才生效

| 要调的内容 | 当前真实生效文件 | 谁读取 | 生效方式 | 容易误改的文件 |
|---|---|---|---|---|
| 半场选择 | `scripts/launch_rc2026.py` 输入 `left/right` | 启动脚本传给 `full_system.launch.py` | 重启系统 | 直接改 `config.py` 的 `FIELD_SIDE`，会被 launch 环境变量覆盖 |
| KFS颜色 | `scripts/launch_rc2026.py` 自动联动 | 左半场=blue，右半场=red | 重启系统 | 手动改 `kfs_color`，现在交互脚本不再单独询问 |
| 左/右半场地图 | `rc2026_navigation/launch/localization.launch.py` | `field_side` 选择 `left_half.pcd/right_half.pcd` | 重启定位 | 只改地图文件名但没传对 `field_side` |
| PCD地图本体 | `rc2026_navigation/map/left_half.pcd` 和 `right_half.pcd` | `map_relocalizer` | 替换地图后重启定位 | FAST-LIO 临时保存的 `~/.ros/*.pcd` |
| 启动点/区域测试先验 | `rc2026_bringup/launch/full_system.launch.py` 的 `start_x/y/yaw` 默认逻辑，或启动参数覆盖 | 同时传给 ICP prior 和 `waypoint_navigator.loc_offset_*` | 重启系统 | 只改 `config.py` 航点，没改测试模式的启动先验 |
| ICP参数 | `rc2026_navigation/launch/localization.launch.py` 中 `map_relocalizer` 参数 | `map_relocalizer.py` | 重启定位节点 | 只改 `map_relocalizer.py` 默认值但 launch 已覆盖 |
| FAST-LIO参数 | `/home/hgzq/ros2_external_ws/src/fast_lio/config/mid360.yaml` | 外部 `fast_lio` 包 | 外部工作空间 build/source 后重启 | `rc2026_navigation/config/fastlio2/mapping.yaml` |
| Livox雷达IP | `/home/hgzq/ros2_external_ws/src/livox_ros_driver2/config/MID360s_config.json` | 外部 Livox launch | 外部工作空间 build/source 后重启 | `rc2026_bringup/config/MID360s_config.json` 副本 |
| EKF融合 | `rc2026_navigation/config/ekf.yaml` | `robot_localization/ekf_node` | 重启定位节点 | `robot_params.yaml` |
| 路点导航PID/限速 | `rc2026_bringup/launch/full_system.launch.py` 里的 `waypoint_nav` 参数 | `waypoint_navigator.py` | 重启 `waypoint_navigator` | `robot_params.yaml` 的 `pid` 节 |
| `/cmd_vel` 到串口限幅 | 当前是 `cmd_vel_bridge/bridge_node.py` 默认参数，或改 `full_system.launch.py` 给它传参 | `cmd_vel_bridge` | 重启 `cmd_vel_bridge` | `cmd_vel_bridge/config/pid_params.yaml` 当前主启动没加载 |
| STM32串口协议/波特率 | `auto_serial_bridge-main/config/protocol.yaml` | auto_serial_bridge 代码生成和 launch | 必须重新 build `auto_serial_bridge` | 只 source 不 build |
| 武馆/梅林/对抗区航点 | `decision_processor/decision_processor/config.py` | `game_controller.py` | build/source 后重启状态机 | `robot_params.yaml` |
| 视觉粗对齐 | `decision_processor/decision_processor/config.py` | `processor_node.py` | build/source 后重启节点 | `full_system.launch.py` 的启动默认值只管初始值，阶段切换时会使用 `config.py` |
| USB精对齐 | `config.py` 和 `fine_align_node.py` | `fine_align_node.py` | build/source 后重启节点 | 只调 D435i 的粗对齐参数 |
| ArUco合体对齐 | `full_system.launch.py` 的 `dock_align_node` 参数，部分逻辑在 `dock_align_node.py` | `dock_align_node` | 重启节点 | `robot_params.yaml` |
| TF/RViz几何 | `rc2026_bringup/urdf/rc2026_robot.urdf.xacro` | `robot_state_publisher` | build/source 后重启 | `robot_params.yaml` 当前主启动不注入 xacro |
| Python坐标换算fallback | `decision_processor/decision_processor/config.py` 的相机/机械臂 offset | `tf_manager.py` | build/source 后重启 | 只改 URDF 但不改 `config.py` |
| 下位机闭环PID | STM32固件，不在本仓库 | 下位机 | 重新烧录/参数下发 | 上位机 YAML |

---

## 2. 当前不生效或只在旧链路生效的文件

这些文件不要作为当前比赛启动的第一调参入口：

| 文件 | 当前情况 |
|---|---|
| `rc2026_bringup/config/measurement_params.yaml` | 文件自己写着已废弃，`full_system.launch.py` 没有读取它。 |
| `rc2026_bringup/config/robot_params.yaml` | `robot_bringup.launch.py` 会读取，但你当前 `launch_rc2026.py -> full_system.launch.py` 不读取。 |
| `rc2026_navigation/config/fastlio2/mapping.yaml` | 不是当前 `full_system.launch.py` 启动的 FAST-LIO 参数。当前读取外部包 `fast_lio/config/mid360.yaml`。 |
| `rc2026_navigation/config/fastlio2/localization.yaml` | 当前启动链路未使用。 |
| `cmd_vel_bridge/config/pid_params.yaml` | `cmd_vel_bridge.launch.py` 会读取，但 `full_system.launch.py` 直接启动 `bridge_node`，没有加载这个 YAML。 |
| `rc2026_bringup/config/MID360s_config.json` | 当前 Livox launch 使用外部包 share 目录里的 `MID360s_config.json`，这里是副本。 |

如果想恢复“集中在 `robot_params.yaml` 调参”的模式，需要改 `full_system.launch.py`：读取 `robot_params.yaml` 并把参数传给 xacro、`waypoint_nav`、`processor_node`、`cmd_vel_bridge`。在没改 launch 之前，改 `robot_params.yaml` 大多数不会影响你现在的启动。

---

## 3. 启动脚本和比赛参数

### 3.1 半场和KFS颜色

文件：

```text
scripts/launch_rc2026.py
```

当前逻辑：

```python
left  -> kfs_color = blue
right -> kfs_color = red
```

它会把这些 launch 参数传给 `full_system.launch.py`：

```text
field_side
test_area
kfs_color
kfs_real
kfs_fake
use_game_controller
enable_serial
debug_gui
```

影响范围：

- `field_side` 会注入环境变量 `RC2026_FIELD_SIDE`，让 `config.py` 自动镜像右半场坐标。
- `field_side` 会传给 `localization.launch.py`，决定加载 `left_half.pcd` 还是 `right_half.pcd`。
- `kfs_color` 会传给 `game_controller`，决定 USB 精对齐用蓝色还是红色滤色模式。

### 3.2 区域测试起点

文件：

```text
rc2026_bringup/launch/full_system.launch.py
```

这里的 `start_x/start_y/start_yaw` 很关键。它不是单纯“显示用起点”，而是同时影响：

```text
1. map_relocalizer.prior_offset_x/y/yaw
2. waypoint_navigator.loc_offset_x/y/yaw
```

也就是说，区域测试时机器人实际摆放位置必须和 `test_area` 对应的启动先验一致：

| test_area | 机器人应摆放到 | 默认先验 |
|---|---|---|
| `full` / `weapon` | 出发点 | `(-1.4, 0.4, 1.5708)`，右半场 x 镜像 |
| `merlin` | 梅林入口 | `(-3.0, 2.0, 1.5708)`，右半场 x 镜像 |
| `confront` | 对抗区入口 | `(-5.4, 11.6, 0.0)`，右半场 x 和 yaw 镜像 |

如果你把车放在出发点，却用 `test_area:=merlin`，ICP 先验和导航偏移都会错，表现就是启动重定位看似“校正很大”或第一段路径乱。

---

## 4. TF和硬件安装外参

### 4.1 当前主启动下，URDF TF改哪里

真实生效文件：

```text
rc2026_bringup/urdf/rc2026_robot.urdf.xacro
```

当前 `full_system.launch.py` 是这样启动 URDF 的：

```python
robot_description = ParameterValue(Command(['xacro ', urdf_file]), value_type=str)
```

它没有向 xacro 传入 `robot_params.yaml` 参数，所以 xacro 文件里的默认值就是实际 TF：

```xml
<xacro:arg name="camera_x" default="0.05"/>
<xacro:arg name="camera_y" default="0.0"/>
<xacro:arg name="camera_z" default="0.45"/>

<xacro:arg name="lidar_x" default="0.0"/>
<xacro:arg name="lidar_y" default="0.0"/>
<xacro:arg name="lidar_z" default="0.55"/>

<xacro:arg name="weapon_arm_x" default="0.05"/>
<xacro:arg name="weapon_arm_y" default="0.15"/>
<xacro:arg name="weapon_arm_z" default="0.30"/>

<xacro:arg name="kfs_arm_x" default="0.0"/>
<xacro:arg name="kfs_arm_y" default="0.0"/>
<xacro:arg name="kfs_arm_z" default="0.05"/>
```

### 4.2 视觉坐标换算还要同步 config.py

真实生效文件：

```text
decision_processor/decision_processor/config.py
```

`tf_manager.py` 会读取这些值作为相机和机械臂坐标换算 fallback：

```python
CAMERA_OFFSET_X/Y/Z
CAMERA_ROLL_RAD/PITCH_RAD/YAW_RAD
KFS_ARM_OFFSET_X/Y/Z
WEAPON_ARM_OFFSET_X/Y/Z
USB_CAMERA_OFFSET_X/Y/Z
SUCTION_OFFSET_X/Y/Z
```

现场建议：

```text
改相机/机械臂安装位置时：
1. 先改 rc2026_robot.urdf.xacro，保证 RViz/TF 树正确。
2. 再改 decision_processor/config.py，保证视觉和机械臂坐标换算一致。
3. 当前不要只改 robot_params.yaml。
```

### 4.3 LiDAR安装位置和FAST-LIO外参不是同一个东西

| 参数 | 文件 | 作用 |
|---|---|---|
| `lidar_x/y/z/roll/pitch/yaw` | `rc2026_robot.urdf.xacro` | 影响 TF/RViz/导航几何关系。 |
| `extrinsic_T/R` | `/home/hgzq/ros2_external_ws/src/fast_lio/config/mid360.yaml` | 影响 FAST-LIO 内部 LiDAR-IMU 外参。 |
| `MID360s_config.json` 里的 `extrinsic_parameter` | Livox驱动配置 | 驱动层外参字段，当前主要别和 FAST-LIO/URDF 混着乱改。 |

Mid-360 内置 IMU 和雷达同体，FAST-LIO 的 `extrinsic_T/R` 通常不用现场大改。真正要先确认的是 LiDAR 是否水平、安装高度是否和 URDF 一致。

---

## 5. Livox雷达IP

当前真实 launch：

```text
rc2026_navigation/launch/localization.launch.py
  -> livox_ros_driver2/launch_ROS2/msg_MID360s_launch.py
```

外部 launch 里写死读取：

```text
/home/hgzq/ros2_external_ws/install/livox_ros_driver2/share/livox_ros_driver2/config/MID360s_config.json
```

长期应改源码：

```text
/home/hgzq/ros2_external_ws/src/livox_ros_driver2/config/MID360s_config.json
```

关键字段：

```json
"host_ip": "192.168.1.50",
"ip": "192.168.1.153"
```

调试原则：

- `host_ip` 是 Jetson/工控机有线网卡 IP。
- `ip` 是雷达 IP。
- 两者必须在同一网段。
- 改外部源码后，重新 build/source 外部工作空间并重启系统。
- 只改 `rc2026_bringup/config/MID360s_config.json` 不会影响当前启动，除非你改 Livox launch 显式加载它。

查当前外部包路径：

```bash
ros2 pkg prefix livox_ros_driver2
```

---

## 6. FAST-LIO

### 6.1 当前真正读取的 FAST-LIO 配置

当前 `localization.launch.py` 启动：

```python
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
```

`fast_lio` 的 launch 默认 `config_path` 是外部包 share 目录：

```text
/home/hgzq/ros2_external_ws/install/fast_lio/share/fast_lio/config/mid360.yaml
```

长期应改源码：

```text
/home/hgzq/ros2_external_ws/src/fast_lio/config/mid360.yaml
```

查路径：

```bash
ros2 pkg prefix fast_lio
```

### 6.2 现场常调参数

文件：

```text
/home/hgzq/ros2_external_ws/src/fast_lio/config/mid360.yaml
```

常调项：

| 参数 | 作用 | 现象和调法 |
|---|---|---|
| `point_filter_num` | 点云抽点，每 N 个点保留 1 个 | 点云太稀/定位不稳 -> 减小；CPU高 -> 增大。 |
| `preprocess.blind` | 近距离盲区过滤 | 看到车体/地面近点干扰 -> 增大；近距离特征不够 -> 减小。 |
| `filter_size_surf` | 面特征体素滤波 | 漂/细节少 -> 减小；CPU高 -> 增大。 |
| `filter_size_map` | 地图体素滤波 | 同上。 |
| `mapping.acc_cov` | 加速度噪声 | IMU噪声大、运动时抖 -> 适当增大。 |
| `mapping.gyr_cov` | 陀螺噪声 | 旋转漂、角度跳 -> 适当增大。 |
| `mapping.b_acc_cov` | 加速度偏置随机游走 | 长时间漂移明显 -> 小幅增大。 |
| `mapping.b_gyr_cov` | 陀螺偏置随机游走 | 长时间 yaw 漂 -> 小幅增大。 |
| `mapping.extrinsic_est_en` | 是否在线估计 LiDAR-IMU 外参 | 内置IMU一般固定；不确定时可临时 true 观察。 |
| `mapping.extrinsic_T/R` | LiDAR-IMU 外参 | 安装/坐标确认后固定，不要边跑边随意改。 |

当前外部 `mid360.yaml` 里 `pcd_save.pcd_save_en: true`，它会保存建图点云。比赛定位时如果不想持续存 PCD，可以改为 `false`，但要确认你没有依赖它保存新地图。

### 6.3 当前 repo 内 fastlio2/mapping.yaml 的定位

文件：

```text
rc2026_navigation/config/fastlio2/mapping.yaml
```

当前主启动不读取它。它更像是项目内旧版/备用建图配置。除非你改 `localization.launch.py` 给 `fast_lio` 传：

```text
config_path:=.../rc2026_navigation/config/fastlio2
config_file:=mapping.yaml
```

否则改这里不会影响 `python3 scripts/launch_rc2026.py`。

---

## 7. EKF融合

真实生效文件：

```text
rc2026_navigation/config/ekf.yaml
```

启动位置：

```text
rc2026_navigation/launch/localization.launch.py
```

融合来源：

| 来源 | Topic | 来自哪里 |
|---|---|---|
| 轮式里程计 | `/odom/wheel` | `cmd_vel_bridge/wheel_odom_publisher.py` 把 `/feedback/wheel_odom` 转成 Odometry |
| FAST-LIO | `/Odometry` | `fast_lio` |
| Livox IMU | `/livox/imu` | `livox_ros_driver2` |

常调项：

| 参数 | 作用 | 调法 |
|---|---|---|
| `odom0_config` | 决定轮式里程计融合 x/y/yaw/vx/vy/vyaw 哪些量 | 更信轮速就保留更多 true；轮子打滑严重就减少位姿信任。 |
| `odom1_config` | 决定 FAST-LIO 融合哪些量 | 通常用 x/y/yaw，不用速度。 |
| `imu0_config` | 决定 IMU 融合角速度/加速度 | 爬坡时如果 ax/ay 引入重力噪声，可把最后 ax/ay 改 false。 |
| `odom0_rejection_threshold` | 轮式里程计异常值拒绝阈值 | 编码器偶发跳变被拒太多 -> 增大；脏数据进入 -> 减小。 |
| `odom1_rejection_threshold` | FAST-LIO异常值拒绝阈值 | FAST-LIO跳但仍可信 -> 增大；误匹配进入 -> 减小。 |
| `process_noise_covariance` | 过程噪声，越大越依赖观测 | 轨迹跟不上观测 -> 增大对应项；输出抖 -> 减小对应项或增大观测协方差。 |
| `initial_estimate_covariance` | 初始不确定性 | 每次摆车不准时可增大 x/y/yaw 对角值。 |

注意：

```text
ekf.yaml 当前 base_link_frame 是 base_footprint。
URDF 发布 base_footprint -> base_link 固定变换。
不要随便把 EKF 的 base_link_frame 改成 base_link，否则容易 TF 多父帧冲突。
```

检查命令：

```bash
ros2 topic echo /odom
ros2 topic echo /odom/wheel
ros2 topic echo /Odometry
ros2 topic echo /diagnostics
```

---

## 8. ICP地图重定位

### 8.1 参数在哪里改

真实生效文件：

```text
rc2026_navigation/launch/localization.launch.py
```

这里给 `map_relocalizer` 传参：

```python
'scan_accum_pts':   3000,  # 累积用于匹配的激光点云数量，最多攒3000个做配准 
'icp_max_dist':     0.5,   # ICP最大匹配距离 
'icp_min_fitness':  0.05,  # ICP最小匹配得分，低于这个就回退先验
'voxel_size':       0.1,   # 点云体素滤波大小，太大会丢特征，太小会慢
'max_correction_xy': 1.0,  # ICP最大XY修正量，超过就回退先验，单次匹配最多修正一米的位置偏移
'max_correction_yaw_deg': 30.0,  # ICP最大yaw修正量，超过就回退先验
```

`map_relocalizer.py` 里面虽然也有默认值，但当前 launch 已经显式覆盖，所以现场调参优先改 `localization.launch.py`。

### 8.2 ICP偏移是怎么算的

`map_relocalizer.py` 流程：

```text
1. 加载 left_half.pcd 或 right_half.pcd。
2. 累积 /cloud_registered 点云到 scan_accum_pts。
3. 用 prior_offset_x/y/yaw 生成 init_T。
4. 用 Open3D ICP 将当前 scan 对齐到 PCD map。
5. ICP 得到 T，取 T[0,3], T[1,3], yaw 作为新 loc_offset。
6. corr = inv(init_T) @ T，表示相对先验的修正量。
7. 如果 fitness 太低，或 corr 超过 max_correction_xy/yaw，则回退先验。
8. 在线更新 /waypoint_navigator 的 loc_offset_x/y/yaw。
```

日志里的含义：

```text
pose=(x,y,yaw)  是 ICP 算出的新 offset，也就是当前 FAST-LIO 坐标系到游戏/map坐标的变换。
corr=(dx,dy,dyaw) 是相对启动先验 prior_offset 的修正量。
```

所以看到 `pose.x=2m` 不一定说明“校正了2m”。如果你的先验本来就是右半场 `x=1.4` 或区域入口 `x=3.0`，那么要看 `corr` 是否很大。

### 8.3 常见问题和调法

| 现象 | 优先检查 | 调参 |
|---|---|---|
| 明明摆在起点，ICP显示大偏移 | `field_side` 是否选错，地图是否左右半场错，`start_x/y/yaw` 是否和实际摆车点一致 | 先别调 ICP，先修先验和地图 |
| fitness 很低 | 地图和现场不一致，点云太少，体素太大 | 增大 `scan_accum_pts`，减小 `voxel_size`，适当增大 `icp_max_dist` |
| 匹配到错误位置 | 地图重复结构太多，先验错太多，`icp_max_dist` 太大 | 减小 `icp_max_dist`，减小 `max_correction_xy/yaw`，提高 `icp_min_fitness` |
| 总是回退先验 | `corr` 超过限制或 fitness 低 | 看日志原因；必要时增大 `max_correction_xy`，但不要用它掩盖地图/半场错误 |
| 点云刚启动不稳定 | FAST-LIO还没收敛 | 增大 `TimerAction` 延时或手动触发重定位 |

调试命令：

```bash
ros2 topic echo /relocalize/status
ros2 topic pub --once /relocalize/trigger std_msgs/msg/String "{data: manual}"
ros2 param get /map_relocalizer map_file
ros2 param get /waypoint_navigator loc_offset_x
ros2 param get /waypoint_navigator loc_offset_y
ros2 param get /waypoint_navigator loc_offset_yaw
```

---

## 9. 路点导航PID和底盘速度链路

### 9.0 速度参数分层速查

项目里有好几组“最大速度”，它们不在同一层，不能互相替代：

| 参数 | 所在文件 | 限制对象 | 是否经过 `cmd_vel_bridge` | 作用边界 |
|---|---|---|---|---|
| `max_linear_speed` / `max_angular_speed` | `rc2026_bringup/launch/full_system.launch.py` 传给 `waypoint_navigator` | 路点导航发布的 `/cmd_vel` | 是 | 只限制“坐标点到坐标点”的导航速度。 |
| `max_linear_accel` / `max_angular_accel` | `full_system.launch.py` 传给 `waypoint_navigator` | `/cmd_vel` 每周期变化量 | 是 | 限制加减速斜率，不是最终速度上限。 |
| `max_linear_vel` / `max_angular_vel` | `cmd_vel_bridge/bridge_node.py` 默认值，或 launch 传参 | bridge 接收到的 `/fine_align/cmd`、`/dock_align/cmd`、`/cmd_vel` | 是 | bridge 内部最终硬限幅，不会限制绕过 bridge 直接发 `/serial/chassis_cmd` 的节点。 |
| `FORWARD_SPEED_GAIN` / `ALIGN_TURN_GAIN` | `decision_processor/config.py` | D435i 武器/KFS粗对齐速度 | 否，`processor_node` 直接发 `/serial/chassis_cmd` | 是视觉伺服增益，不是硬上限；输出过大要调小增益。 |
| `FINE_ALIGN_MAX_SPEED_MPS` | `decision_processor/config.py` | USB精对齐横移速度 | 同时可能被 `cmd_vel_bridge` 接收，也会被 `game_controller` 转发 | 精对齐自身真实速度上限，优先看它。 |
| `dock_align_node.max_linear/max_angular` | `dock_align_node.py` 默认值，或 launch 传参 | ArUco合体对齐 `/dock_align/cmd` | 是 | 只限制合体对齐节点自己的输出。 |
| `SPEED_FACTOR_*` / `BRAKE_FACTOR_SLOPE` | `decision_processor/config.py` | 梅林坡度/爬坡建议速度系数 | 取决于具体梅林指令流 | 是坡度状态下的倍率/建议值，不是底盘全局速度上限。 |
| STM32 PID/限速 | 下位机固件 | 电机实际跟随和底盘硬限制 | ROS之后 | 上位机看不到源码，需在下位机侧调。 |

对普通路点导航来说，实际速度上限约等于：

```text
实际导航线速度上限 = min(waypoint_navigator.max_linear_speed, cmd_vel_bridge.max_linear_vel)
实际导航角速度上限 = min(waypoint_navigator.max_angular_speed, cmd_vel_bridge.max_angular_vel)
```

但这只对 `/cmd_vel -> cmd_vel_bridge -> /serial/chassis_cmd` 这条链路成立。视觉粗对齐和部分状态机停车/转发命令会直接发布 `/serial/chassis_cmd`，不受 `cmd_vel_bridge` 的 `max_*_vel` 限制。

### 9.1 路点导航参数在哪里调

真实生效文件：

```text
rc2026_bringup/launch/full_system.launch.py
```

节点：

```text
decision_processor/waypoint_navigator.py
```

当前 launch 写死参数：

```python
'max_linear_speed': 0.6,    # 路点导航生成 /cmd_vel 时允许的最大线速度 (m/s)
'min_linear_speed': 0.01,   # 离目标还没进 tolerance 时的最低线速度，太小可能爬不动，太大容易冲过
'max_angular_speed': 1.0,   # 路点导航原地转 yaw 时允许的最大角速度 (rad/s)
'kp_linear': 0.50,
'kp_angular': 0.40,
'decel_distance': 0.80,     # 距离目标小于该值时开始按距离减速，不是速度上限
'max_linear_accel': 0.35,   # /cmd_vel 线速度每秒最大变化量，抑制突变 （加速度）
'max_angular_accel': 0.60,  # /cmd_vel 角速度每秒最大变化量，抑制突变
'xy_tolerance': 0.12,
'yaw_tolerance': 0.15,
'waypoint_timeout': 30.0,
'progress_timeout': 3.0,
'visual_servo_timeout': 15.0,
'control_rate': 50.0, # 底盘控制帧率
```

这组参数只决定 `waypoint_navigator` 发到 `/cmd_vel` 的速度。它不会限制 `processor_node` 或 `game_controller` 直接发布到 `/serial/chassis_cmd` 的视觉对齐/停车命令。

现场调法：

| 现象 | 调哪些 |
|---|---|
| 到点前冲/来回振荡 | 降低 `kp_linear`，降低 `max_linear_speed`，增大 `decel_distance`，降低 `max_linear_accel` |
| 原地转向过冲 | 降低 `kp_angular`，降低 `max_angular_speed`，降低 `max_angular_accel`，放宽 `yaw_tolerance` |
| 到附近但一直不算到 | 增大 `xy_tolerance` 或 `yaw_tolerance` |
| 太慢 | 小幅提高 `max_linear_speed`、`kp_linear`，但先确认下位机能跟上 |
| 被误判卡住 | 增大 `progress_timeout` 或减小 `progress_min_delta`，但当前 launch 没传 `progress_min_delta`，用的是代码默认 `0.02` |

检查运行参数：

```bash
ros2 param get /waypoint_navigator kp_linear
ros2 param get /waypoint_navigator kp_angular
ros2 param get /waypoint_navigator max_linear_speed
ros2 param get /waypoint_navigator xy_tolerance
```

### 9.2 当前已经是三阶段导航

`waypoint_navigator.py` 当前控制逻辑：

```text
阶段1：只控制 xy 到目标点附近，angular.z=0。
阶段2：xy 到位后，原地转到目标 yaw。
阶段3：如果设置了 servo_phase，则进入 ALIGN_WEAPON / ALIGN_KFS 视觉伺服。
```

所以“到点还在转 yaw”不是下位机 PID 的第一嫌疑，先看 `/cmd_vel angular.z` 和 `/serial/chassis_cmd angular.z` 是否合理。

### 9.3 `/cmd_vel` 到 `/serial/chassis_cmd`

节点：

```text
cmd_vel_bridge/bridge_node.py
```

当前 `full_system.launch.py` 直接启动：

```python
Node(package='cmd_vel_bridge', executable='bridge_node', name='cmd_vel_bridge')
```

所以它使用代码默认值：

```python
max_linear_vel = 1.5    # bridge 对输入速度做最终夹紧时的线速度上限 (m/s)
max_angular_vel = 3.14  # bridge 对输入速度做最终夹紧时的角速度上限 (rad/s)
control_rate = 50.0     # bridge 发布 /serial/chassis_cmd 的频率 (Hz)
cmd_vel_timeout = 0.5   # 某个输入源超过该时间没更新，就认为该源失效
```

如果你希望 `cmd_vel_bridge/config/pid_params.yaml` 生效，需要把 `full_system.launch.py` 里的 `bridge_node` 改成加载这个 YAML，或直接在 `full_system.launch.py` 给参数。

桥接优先级：

```text
/fine_align/cmd  >  /dock_align/cmd  >  /cmd_vel
```

这组 `max_*_vel` 只限制进入 `cmd_vel_bridge` 的三路输入。当前项目里还有节点直接发布 `/serial/chassis_cmd`：

```text
processor_node      -> /serial/chassis_cmd  # D435i 武器/KFS粗对齐
game_controller     -> /serial/chassis_cmd  # 停车、USB精对齐转发等状态机命令
cmd_vel_bridge      -> /serial/chassis_cmd  # /cmd_vel、/dock_align/cmd、/fine_align/cmd 仲裁后输出
```

因此：

```text
路点导航太快/太慢：优先调 9.1 的 waypoint_navigator 参数。
想给 /cmd_vel、/dock_align/cmd、/fine_align/cmd 做总限幅：调 bridge 的 max_*_vel。
D435i 粗对齐太猛：调 config.py 的 FORWARD_SPEED_GAIN / ALIGN_TURN_GAIN。
USB 精对齐横移太猛：调 FINE_ALIGN_MAX_SPEED_MPS。
```

输出单位：

```text
/cmd_vel angular.z: rad/s
/serial/chassis_cmd angular.z: deg/s
```

下位机协议如果最终要的是 `mdeg/s`，那是在 auto_serial_bridge 或 STM32侧再处理。当前 ROS 话题 `/serial/chassis_cmd.angular.z` 是 deg/s。

检查命令：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /serial/chassis_cmd
ros2 param get /cmd_vel_bridge max_angular_vel
```

---

## 10. 串口协议、轮式里程计和波特率

### 10.1 协议文件

真实生效文件：

```text
auto_serial_bridge-main/config/protocol.yaml
```

当前关键段：

```yaml
serial_controller:
  ros__parameters:
    port: "/dev/ttyUSB0"
    baudrate: 921600
    timeout: 0.1

config:
  baudrate: 921600
```

注意：这个包会根据 `protocol.yaml` 代码生成。改协议字段、消息ID、波特率后必须重新编译：

```bash
cd ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws
colcon build --symlink-install --packages-select auto_serial_bridge --allow-overriding auto_serial_bridge
source install/setup.bash
```

### 10.2 轮式里程计反馈

协议：

```yaml
WheelOdom:
  pub_topic: "/feedback/wheel_odom"
  ros_msg: "geometry_msgs/msg/Twist"
  linear.x  = pos_x
  linear.y  = pos_y
  angular.z = yaw
  linear.z  = vx
  angular.x = vy
  angular.y = omega
```

实时查看：

```bash
ros2 topic echo /feedback/wheel_odom
```

只看关键字段：

```bash
ros2 topic echo /feedback/wheel_odom --field linear
ros2 topic echo /feedback/wheel_odom --field angular
```

轮式里程计转标准 Odometry 的文件：

```text
cmd_vel_bridge/cmd_vel_bridge/wheel_odom_publisher.py
```

如果 EKF 轨迹跟轮式里程计差很多，先同时看：

```bash
ros2 topic echo /feedback/wheel_odom
ros2 topic echo /odom/wheel
ros2 topic echo /odom
```

---

## 11. 视觉粗对齐：武馆和KFS

真实生效文件：

```text
decision_processor/decision_processor/config.py
```

关键参数：

```python
ALIGN_THRESHOLD_DEG
STOP_DISTANCE_M
CAM_X_OFFSET_M

KFS_ALIGN_THRESHOLD_DEG
KFS_STOP_DISTANCE_M
KFS_CAM_X_OFFSET_M

ALIGN_TURN_GAIN
FORWARD_SPEED_GAIN
WUGUAN_CONF_MIN
CONFIRM_FRAMES
TARGET_TIMEOUT_S
```

`full_system.launch.py` 启动 `processor_node` 时也传了：

```python
'stop_distance_m': 0.50,
'align_threshold_deg': 5.0,
'pick_duration_s': 10.0,
'conf_threshold': 0.5,
```

但要注意：进入 `ALIGN_WEAPON` 或 `ALIGN_KFS` 阶段时，`processor_node._on_game_phase()` 会重新从 `config.py` 设置当前阶段的停止距离和角度阈值。所以现场调粗对齐，优先改 `config.py`。

速度相关区别：

```text
ALIGN_TURN_GAIN:
  视觉粗对齐角速度增益。
  processor_node 计算 angular.z = align_angle_deg * ALIGN_TURN_GAIN。
  单位在 /serial/chassis_cmd 中是 deg/s。

FORWARD_SPEED_GAIN:
  视觉粗对齐前进速度增益。
  processor_node 计算 linear.x = max(0, distance - stop_distance) * FORWARD_SPEED_GAIN。
  单位是 m/s。
```

这两个不是硬限速，而是“误差乘以增益”。它们由 `processor_node` 直接发布到 `/serial/chassis_cmd`，不经过 `cmd_vel_bridge`，所以不会被 `cmd_vel_bridge.max_linear_vel/max_angular_vel` 自动夹紧。粗对齐如果前冲或旋转太猛，优先降低这两个 gain。

现象和调法：

| 现象 | 调哪些 |
|---|---|
| 对准后仍偏左/偏右 | `CAM_X_OFFSET_M` 或 `KFS_CAM_X_OFFSET_M` |
| 转向太猛 | 降低 `ALIGN_TURN_GAIN` |
| 前进太猛 | 降低 `FORWARD_SPEED_GAIN` |
| 离武器/KFS太近才停 | 增大 `STOP_DISTANCE_M` 或 `KFS_STOP_DISTANCE_M` |
| 太远就停 | 减小停止距离 |
| 识别偶发误触发 | 增大 `CONFIRM_FRAMES` 或 `WUGUAN_CONF_MIN` |
| 经常漏检 | 降低置信度阈值，但要防误检 |

---

## 12. USB精对齐

启动参数位置：

```text
rc2026_bringup/launch/full_system.launch.py
```

当前传给 `fine_align_node`：

```python
'cam_index': 0,
'debug_gui': LaunchConfiguration('debug_gui')
```

主要调参文件：

```text
decision_processor/decision_processor/config.py
decision_processor/decision_processor/fine_align_node.py
```

`config.py`：

```python
FINE_ALIGN_MAX_SPEED_MPS      # USB精对齐横移速度真实上限，单位 m/s
FINE_ALIGN_CONFIRM_FRAMES
FINE_ALIGN_TIMEOUT_S
```

`fine_align_node.py`：

```python
DEADZONE_ERR
DEADZONE_DX
START_DECEL_ERR
START_DECEL_DX
MIN_SPEED_LIMIT
MAX_SPEED_LIMIT
TARGET_X
DEFAULT_COLOR_THRESH
DEFAULT_EDGE_SENS
DEFAULT_CLAHE_CLIP
```

速度换算关系：

```text
fine_align_node 先根据图像误差算 output_speed: 0~100 (%)
最终横移速度 linear.y = output_speed / 100 * FINE_ALIGN_MAX_SPEED_MPS
```

其中：

- `FINE_ALIGN_MAX_SPEED_MPS` 是真实底盘横移速度上限，单位 m/s。
- `MIN_SPEED_LIMIT` / `MAX_SPEED_LIMIT` 是图像算法里的百分比输出范围，不是 m/s。
- USB精对齐阶段里，`fine_align_node` 发布 `/fine_align/cmd`；`cmd_vel_bridge` 会订阅它，`game_controller` 也会在 `M_FINE_ALIGN` 中把它转发到 `/serial/chassis_cmd`。因此不要只依赖 `cmd_vel_bridge.max_linear_vel` 做精对齐限速，真正应该先调 `FINE_ALIGN_MAX_SPEED_MPS`。

现场建议：

- 颜色识别不稳定：调 `DEFAULT_COLOR_THRESH`，调光照，必要时 debug 模式用 trackbar 看掩膜。
- 快到中心还左右抖：增大 `DEADZONE_DX/DEADZONE_ERR`，降低 `FINE_ALIGN_MAX_SPEED_MPS`。
- 对齐太慢：小幅增大 `FINE_ALIGN_MAX_SPEED_MPS` 或降低 deadzone，但先确认不抖。
- 判断完成太早：增大 `FINE_ALIGN_CONFIRM_FRAMES`。
- 相机不是 `/dev/video0`：改 `full_system.launch.py` 的 `cam_index`。

调试启动时用 `debug` 模式，`launch_rc2026.py` 会把 `debug_gui:=true` 传下去。

---

## 13. ArUco合体对齐

真实生效位置：

```text
rc2026_bringup/launch/full_system.launch.py
decision_processor/decision_processor/dock_align_node.py
decision_processor/decision_processor/config.py
```

当前 launch 参数：

```python
'marker_size_m': 0.10,
'marker_ids': [0],
'target_dist_m': 0.30,
'debug_gui': LaunchConfiguration('debug_gui')
```

`dock_align_node.py` 里还有速度参数默认值：

```python
max_linear = 0.15   # ArUco合体对齐节点自身输出的最大线速度 (m/s)
max_angular = 0.40  # ArUco合体对齐节点自身输出的最大角速度 (rad/s)
```

当前 `full_system.launch.py` 没有给 `dock_align_node` 传 `max_linear/max_angular`，所以实际使用代码默认值。如果要现场可调，建议把这两个参数也加进 `full_system.launch.py` 的 `dock_align_node` 参数表。

合体对齐速度链路：

```text
dock_align_node -> /dock_align/cmd -> cmd_vel_bridge -> /serial/chassis_cmd
```

所以合体对齐最终上限是：

```text
linear 上限 = min(dock_align_node.max_linear, cmd_vel_bridge.max_linear_vel)
angular 上限 = min(dock_align_node.max_angular, cmd_vel_bridge.max_angular_vel)
```

按当前值：

```text
linear 上限 = min(0.15, 1.5) = 0.15 m/s
angular 上限 = min(0.40, 3.14) = 0.40 rad/s
```

`config.py` 里还有：

```python
DOCK_ALIGN_CONFIRM_FRAMES
DOCK_ALIGN_OK_RATIO
DOCK_ALIGN_TIMEOUT_S
```

现场常调：

| 现象 | 调哪些 |
|---|---|
| ArUco距离估计不准 | `marker_size_m` 必须和实际边长一致 |
| 合体停得太远/太近 | `target_dist_m` |
| 偶发误判完成 | 增大 `DOCK_ALIGN_CONFIRM_FRAMES` 或 `DOCK_ALIGN_OK_RATIO` |
| 长时间找不到 | 开 `debug_gui` 看图像和 marker id |

---

## 14. 航点、状态机和梅林参数

真实生效文件：

```text
decision_processor/decision_processor/config.py
decision_processor/decision_processor/game_controller.py
decision_processor/decision_processor/meilin_path_planner.py
```

### 14.1 航点坐标

改这里：

```python
WAYPOINT_START
WAYPOINT_WEAPON_RACK
WAYPOINT_ASSEMBLY
WAYPOINT_MERLIN_ENTRY
WAYPOINT_MERLIN_EXIT_GATHER
WAYPOINT_EXIT_MERLIN
WAYPOINT_CONFRONT_ENTRY
WAYPOINT_KFS_PLACE
WAYPOINT_CONFRONT_WAIT
```

右半场不需要单独写一套，`config.py` 最后会根据 `FIELD_SIDE == 'right'` 自动镜像：

```text
x -> -x
yaw -> pi - yaw
```

但注意：区域测试启动先验在 `full_system.launch.py` 里另有默认逻辑。你改了 `WAYPOINT_MERLIN_ENTRY` 后，如果还用 `test_area:=merlin`，最好同步检查 `full_system.launch.py` 里的 `start_x/y/yaw` 默认是否也要更新。

### 14.2 梅林方块和路径规划

改这里：

```python
BLOCK_CENTERS
BLOCK_HEIGHTS_MM
BLOCK_GRID
MEILIN_ENTRY_BLOCKS
MEILIN_EXIT_BLOCKS
MERLIN_DEFAULT_ENTRY
MERLIN_ENTRY_CLIMB_POINTS
MERLIN_EXIT_DESCEND_POINTS
MERLIN_TRIGGER_FROM_CENTER_M
ENTRY_CLIMB_CMD
EXIT_DESCEND_CMD
```

路径规划规则在：

```text
decision_processor/decision_processor/meilin_path_planner.py
```

当前规则：

- 假 KFS 是障碍，不通行。
- 真 KFS 可以从相邻方块拾取，不一定要走到真 KFS 方块上。
- 真 KFS 如果在入口 1/2/3，会在入口爬升前先拾取。
- 只能前进、左、右，不后退。

### 14.3 状态机时间和动作组

改这里：

```python
MATCH_DURATION_S
MATCH_TIMEOUT_S
PHASE_SWITCH_WAIT_S
MERLIN_CLIMB_WAIT_S
MERLIN_PICKUP_WAIT_S
FINE_ALIGN_TIMEOUT_S
DOCK_ALIGN_TIMEOUT_S
WEAPON_GRAB_TIMEOUT_S
KFS_PLACE_STOP_WAIT_S
KFS_PLACE_CMD_DELAY_S
```

动作组 ID：

```python
ACTION_PICKUP_WEAPON
ACTION_RELEASE_WEAPON
ACTION_PICKUP_KFS
ACTION_RELEASE_KFS
ACTION_PLACE_KFS
ACTION_LOCK_CHASSIS
ACTION_ARM_LIFT_1
ACTION_ARM_LIFT_2
```

这些 ID 必须和 STM32 里的动作组编号一致，同时也要和 `protocol.yaml` 里注释/协议理解一致。

---

## 15. 相机、YOLO模型和D435i参数

### 15.1 YOLO模型和置信度

启动参数在：

```text
rc2026_bringup/launch/full_system.launch.py
```

当前默认：

```python
model='best.pt'
conf='0.5'
device='cuda'
```

模型目录：

```text
ros2_ws/src/vision_detector/weights/
```

`launch_rc2026.py` 现在没有交互询问 `model/conf/device`，所以要临时切模型有两种方式：

```bash
ros2 launch rc2026_bringup full_system.launch.py model:=kfs.pt conf:=0.5 device:=cuda
```

或修改 `launch_rc2026.py` 增加交互项。

类别映射在：

```text
decision_processor/decision_processor/config.py
```

```python
WUGUAN_CLASS_LABELS
WUGUAN_VALID_CLASSES
MEILIN_CLASS_LABELS
MEILIN_REAL_PREFIX
MEILIN_FAKE_PREFIX
```

换模型后如果类别编号变了，必须同步改这里。

### 15.2 RealSense参数

真实生效位置：

```text
rc2026_bringup/launch/full_system.launch.py
```

当前传给 `realsense2_camera`：

```python
rgb_camera.color_profile: 1280x720x30
depth_module.depth_profile: 848x480x30
align_depth.enable: true
enable_gyro: true
enable_accel: true
gyro_fps: 200
accel_fps: 100
unite_imu_method: 2
```

如果 D435i 帧率、分辨率、深度对齐异常，改这里。

---

## 16. IMU、坡度和爬坡

视觉/爬坡用 IMU 处理参数：

```text
decision_processor/decision_processor/config.py
```

关键项：

```python
IMU_TOPIC
IMU_ACCEL_TOPIC
IMU_GYRO_TOPIC
IMU_COMP_ALPHA
GRAVITY
IMU_ACCEL_SIGN_X/Y/Z
SLOPE_DETECT_DEG
SLOPE_LEVEL_MILD/MODERATE/STEEP
SPEED_FACTOR_FLAT/MILD/MODERATE/STEEP
TORQUE_FACTOR_FLAT/MILD/MODERATE/STEEP
BRAKE_FACTOR_SLOPE
```

注意区分：

| 用途 | IMU来源 |
|---|---|
| EKF融合 | `/livox/imu`，来自 Mid-360 |
| 坡度/爬坡辅助 | `config.py` 里当前写的是 D435i IMU 相关 topic |
| 下位机底盘闭环 | STM32自己的 IMU 或编码器，不在上位机仓库内 |

如果出现“上位机姿态”和“下位机姿态”不一致，先确认它们是不是本来就用的不同 IMU。这通常不会直接导致上位机 `/cmd_vel` 振荡，但会影响 EKF、坡度判断、下位机自己的闭环。

---

## 17. 下位机PID和上位机PID怎么区分

先看两层信号：

```bash
ros2 topic echo /cmd_vel
ros2 topic echo /serial/chassis_cmd
ros2 topic echo /feedback/wheel_odom
```

判断方法：

| 现象 | 更可能是谁的问题 |
|---|---|
| `/cmd_vel` 本身已经来回正负跳 | 上位机导航/视觉控制参数问题 |
| `/cmd_vel` 平滑，但 `/serial/chassis_cmd` 被其它源抢占 | `cmd_vel_bridge` 优先级源，例如 `/fine_align/cmd` 或 `/dock_align/cmd` |
| `/serial/chassis_cmd` 合理，但车响应过冲/振荡 | STM32速度闭环 PID 或底盘动力学问题 |
| `/feedback/wheel_odom` 方向/单位错 | 下位机反馈协议、坐标系或 `wheel_odom_publisher.py` 映射问题 |
| `/odom` 跳但 `/odom/wheel` 正常 | EKF/FAST-LIO/TF 问题 |

上位机能调的是目标速度生成和限幅；下位机 PID 调的是底盘实际跟随能力。不要用上位机 PID 去掩盖下位机速度环明显过冲。

---

## 18. 修改后怎么让它生效

| 修改内容 | 推荐操作 |
|---|---|
| `scripts/launch_rc2026.py` | 不需要 build，重新运行脚本。 |
| `*.launch.py` | `colcon build --symlink-install` 后 `source install/setup.bash`，重启 launch。 |
| Python包代码，例如 `decision_processor/*.py` | `colcon build --symlink-install --packages-select decision_processor`，source 后重启节点。 |
| `rc2026_navigation/config/ekf.yaml` | 通常重启 launch 即可；稳妥起见 build/source。 |
| `rc2026_robot.urdf.xacro` | build/source 后重启 `robot_state_publisher`。 |
| `auto_serial_bridge-main/config/protocol.yaml` | 必须重新 build `auto_serial_bridge`，因为会代码生成。 |
| 外部 `fast_lio/config/mid360.yaml` | 在 `/home/hgzq/ros2_external_ws` build/source 外部工作空间，或临时改 install 目录后重启。 |
| 外部 `livox_ros_driver2/config/MID360s_config.json` | 在 `/home/hgzq/ros2_external_ws` build/source 外部工作空间，或临时改 install 目录后重启。 |
| PCD地图文件 | 替换地图后重启 `map_relocalizer` 或整套定位。 |

常用 build：

```bash
cd ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws
colcon build --symlink-install
source install/setup.bash
```

只编译决策：

```bash
colcon build --symlink-install --packages-select decision_processor
source install/setup.bash
```

只编译串口桥：

```bash
colcon build --symlink-install --packages-select auto_serial_bridge --allow-overriding auto_serial_bridge
source install/setup.bash
```

---

## 19. 现场调参推荐顺序

```text
1. 网络和雷达
   - Livox IP 能连上
   - /livox/lidar 和 /livox/imu 正常

2. TF和安装外参
   - RViz 里 base_link、lidar_link、camera_link、机械臂 link 位置正确
   - URDF 和 config.py 的相机/机械臂 offset 一致

3. FAST-LIO
   - /Odometry 不跳
   - /cloud_registered 稳定
   - 点云密度、blind、滤波参数合适

4. 地图和ICP
   - field_side 对
   - map_file 对
   - start_x/y/yaw 和实际摆车点对
   - 看 pose 和 corr，不要只看 x/y 数字大小

5. EKF
   - /odom/wheel、/Odometry、/odom 三者方向一致
   - 不跳、不慢漂

6. 路点导航
   - 先调 full_system.launch.py 里的 waypoint_nav 参数
   - 确认三阶段：先 xy，再 yaw，再视觉

7. 视觉粗对齐
   - 调 config.py 的 STOP_DISTANCE、ALIGN_THRESHOLD、CAM_X_OFFSET、增益和置信度

8. USB精对齐和动作组
   - 调 fine_align_node.py 的图像阈值和 deadzone
   - 确认动作组 ID 和 STM32 一致

9. 下位机PID
   - 上位机输出平滑后，再调 STM32 速度闭环
```

---

## 20. 快速确认当前参数是否真的加载

```bash
# 当前地图和 ICP prior
ros2 param get /map_relocalizer map_file
ros2 param get /map_relocalizer prior_offset_x
ros2 param get /map_relocalizer prior_offset_y
ros2 param get /map_relocalizer prior_offset_yaw

# 当前导航参数
ros2 param get /waypoint_navigator coord_mode
ros2 param get /waypoint_navigator loc_offset_x
ros2 param get /waypoint_navigator kp_linear
ros2 param get /waypoint_navigator kp_angular

# 当前桥接限幅
ros2 param get /cmd_vel_bridge max_linear_vel
ros2 param get /cmd_vel_bridge max_angular_vel

# 合体对齐节点自身限速
ros2 param get /dock_align_node max_linear
ros2 param get /dock_align_node max_angular

# 轮式里程计和底盘指令
ros2 topic echo /feedback/wheel_odom
ros2 topic echo /cmd_vel
ros2 topic echo /serial/chassis_cmd

# 重定位状态
ros2 topic echo /relocalize/status
```

最后更新：2026-07-03
