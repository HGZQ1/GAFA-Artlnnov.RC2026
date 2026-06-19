# RC2026 真机 TF 树说明

本文档描述 `full_system.launch.py` 启动后（已修复 `odom → camera_init` 桥接、
`base_link_frame: base_footprint` 之后）真机运行时的完整 TF 树结构、
每条变换的具体数值与发布者，以及每个坐标系节点的含义和作用。

## 1. 完整 TF 树结构

```
map
└── odom
    ├── camera_init
    │   └── body
    └── base_footprint
        └── base_link
            ├── frame_upper
            ├── camera_link
            │   └── camera_optical_frame
            ├── lidar_link
            ├── weapon_arm_base_link          ← 武器头机械臂底座 (占位, 位置待定)
            ├── arm_base_link                 ← KFS机械臂底座
            │   └── arm_deployed_link
            │       ├── usb_camera_link
            │       │   └── usb_camera_optical_frame
            │       └── suction_link
            ├── wheel_fl_link
            ├── wheel_fr_link
            ├── wheel_rl_link
            └── wheel_rr_link
```

## 2. 各条变换详情

| 父帧 → 子帧 | 类型 | 数值 | 发布者 |
|---|---|---|---|
| `map → odom` | 静态, identity | xyz=(0,0,0), rpy=(0,0,0) | `full_system.launch.py` 的 `map_to_odom_tf` (`tf2_ros static_transform_publisher`) |
| `odom → camera_init` | 静态, identity | xyz=(0,0,0), rpy=(0,0,0) | `localization.launch.py` 的 `odom_to_camera_init_tf` (`tf2_ros static_transform_publisher`) |
| `camera_init → body` | 动态，FAST-LIO 激光惯性里程计实时输出 | 随机器人运动变化 | FAST-LIO `laserMapping` 节点（`/Odometry` + TF广播） |
| `odom → base_footprint` | 动态，EKF 融合结果 | 随机器人运动变化 | `ekf_filter_node`（`ekf.yaml`: `world_frame: odom`, `base_link_frame: base_footprint`, `publish_tf: true`） |
| `base_footprint → base_link` | 固定 | xyz=(0, 0, 0.053905)，rpy=(0,0,0)（z = 轮半径 `wheel_radius`） | `robot_state_publisher`（URDF `base_footprint_joint`） |
| `base_link → frame_upper` | 固定 | xyz=(0, 0, 0.08)，rpy=(0,0,0) | URDF `frame_upper_joint` |
| `base_link → camera_link` | 固定 | xyz=(0.05, 0.0, 0.45)，rpy=(0,0,0) | URDF `camera_joint` |
| `camera_link → camera_optical_frame` | 固定 | xyz=(0,0,0)，rpy=(-π/2, 0, -π/2)（光学帧约定：Z前 X右 Y下） | URDF `camera_optical_joint` |
| `base_link → lidar_link` | 固定 | xyz=(0.0, 0.0, 0.55)，rpy=(0,0,0) | URDF `lidar_joint` |
| `base_link → weapon_arm_base_link` | 固定（**占位，位置待定，需现场测量**） | xyz=(0.05, 0.15, 0.30)，rpy=(0,0,0) | URDF `weapon_arm_joint`，武器头机械臂底座，靠近 D435i 相机前方 |
| `base_link → arm_base_link` | 固定 | xyz=(0.0, 0.0, 0.05)，rpy=(0,0,0) | URDF `arm_base_joint`（KFS机械臂底座） |
| `arm_base_link → arm_deployed_link` | 固定（占位估测，需现场标定） | xyz=(0.30, 0.0, 0.40)，rpy=(0,0,0) | URDF `arm_deployed_joint`，代表机械臂完全抬升+前伸+末端旋转朝下后(精对齐动作组完成姿态)末端横杆中心的位姿 |
| `arm_deployed_link → usb_camera_link` | 固定 | xyz=(0,0,0)，rpy=(0, π/2, 0)（绕Y轴+90°，使相机X(前)轴指向地面） | URDF `usb_camera_joint` |
| `usb_camera_link → usb_camera_optical_frame` | 固定 | xyz=(0,0,0)，rpy=(-π/2, 0, -π/2)（光学帧约定：Z前 X右 Y下） | URDF `usb_camera_optical_joint` |
| `arm_deployed_link → suction_link` | 固定（占位估测，需现场标定） | xyz=(0.03, 0.0, -0.02)，rpy=(0,0,0)（吸盘与摄像头的小偏移） | URDF `suction_joint` |
| `base_link → wheel_fl_link` | 动态（continuous旋转关节，绕Y轴） | xyz=(0.27, 0.25, 0) + 实时转角 | `robot_state_publisher` + `joint_state_publisher`（URDF `wheel_fl_joint`） |
| `base_link → wheel_fr_link` | 动态 | xyz=(0.27, -0.25, 0) + 实时转角 | 同上 |
| `base_link → wheel_rl_link` | 动态 | xyz=(-0.27, 0.25, 0) + 实时转角 | 同上 |
| `base_link → wheel_rr_link` | 动态 | xyz=(-0.27, -0.25, 0) + 实时转角 | 同上 |

## 3. 各坐标系节点的含义与作用

### `map`
- 整场比赛的**游戏坐标系**（场地绝对坐标，原点和朝向由 RC2026 场地规则定义）。
- `waypoint_navigator` 的所有航点（`WAYPOINT_*`）都以 `map` 坐标系（即游戏坐标）给出。
- 与 `odom` 之间始终是 identity 静态变换 —— 树中不直接体现"绝对定位修正"，
  绝对定位的修正是通过 `map_relocalizer` 在线修改 `waypoint_navigator` 的
  `loc_offset_x/y/yaw` 参数实现的（软修正，不改写TF）。与机器人、传感器无关。 

### `odom` EKF 的融合里程计坐标系（不是单纯的下位机里程计）
- 里程计参考系，EKF 融合结果（轮式里程计 + FAST-LIO + IMU）的世界原点，
  即"EKF 节点启动那一刻机器人所在的位置"。
- 是 `robot_localization` EKF 的 `world_frame`，EKF 发布 `odom → base_footprint`。
- 与 `map` 之间恒为 identity，因此 `odom` 系下的坐标数值上等同于"未经
  `loc_offset` 修正前的游戏坐标"。 
原点 = EKF 节点启动那一瞬间机器人所在的位置/朝向（之后固定不动）。
数值 = EKF 融合三路传感器数据后的最优估计结果 

### `camera_init`
- **FAST-LIO（激光惯性里程计SLAM）内部定义的世界坐标系/SLAM原点**，即
  FAST-LIO 节点启动那一刻机器人所在的位置/朝向，运行期间固定不变。
- 命名中的"camera"是 FAST-LIO 代码历史遗留（早期视觉惯性SLAM框架命名习惯），
  **与本机器人实际安装的 RealSense 相机无关**。
- 作用：作为 FAST-LIO 输出 `/Odometry`、`/cloud_registered` 的参考原点；
  通过 `odom → camera_init`（identity 桥接）使 EKF 能正确融合
  `/Odometry`（`frame_id=camera_init`）。

### `body`
- FAST-LIO 实时估计的**机器人当前位姿**（激光惯性里程计输出），
  对应 `/Odometry` 的 `child_frame_id`。
- 在本系统中仅作为 EKF 的一路观测输入（odom1: `/Odometry`），
  不直接驱动机器人本体的渲染/导航姿态（该姿态由 `base_footprint`/`base_link`
  分支给出），因此在树中是相对独立的一条"参考"分支。

### `base_footprint`
- URDF 根坐标系，代表机器人在**地面投影**上的位置（z=0平面），
  是 EKF 融合结果（`odom → base_footprint`）的目标帧（`ekf.yaml` 中
  `base_link_frame: base_footprint`）。
- 之所以不直接用 `base_link` 作为 EKF 输出帧：URDF 中
  `base_footprint → base_link` 已经是固定变换（z方向偏移轮半径），
  若 EKF 也发布 `odom → base_link` 会导致 `base_link` 出现两个父帧
  （TF树冲突）。改为 `base_footprint` 后链路唯一、无冲突。

### `base_link`
- 机器人**底盘主体**坐标系，是机械结构树（传感器、轮子、机械臂、上层框架）
  的公共父帧，所有车载传感器/部件的安装位置都相对 `base_link` 定义。
- `waypoint_navigator` 中 `robot_frame: base_link`，导航控制误差以
  `base_link` 在 `map`/`odom` 系中的位姿计算。

### `frame_upper`
- 底盘上层框架结构（八边形外壳上半部分）的可视化/碰撞几何坐标系，
  纯结构件，无传感器或控制意义。

### `camera_link`
- **Intel RealSense D435i 相机**在底盘上的物理安装位置/朝向坐标系
  （URDF `camera_joint` 固定关节，相对 `base_link` 偏移 (0.05, 0, 0.45)）。
- 作用：将相机图像/深度检测结果（目标在相机系下的坐标）变换到
  `base_link`/`map` 坐标系，供 `decision_processor` 视觉伺服使用。

### `camera_optical_frame`
- RealSense 驱动发布图像数据时使用的**光学坐标系**约定（Z轴朝前、
  X轴朝右、Y轴朝下），与 `camera_link`（机械安装坐标系，X前Y左Z上）
  之间相差一个固定旋转（rpy = -π/2, 0, -π/2）。
- 作用：供 `vision_detector`/深度反投影计算时与相机内参（`camera_info`）的
  坐标约定保持一致。

### `lidar_link`
- **Livox Mid-360S 激光雷达**在底盘上的物理安装位置坐标系
  （URDF `lidar_joint` 固定关节，相对 `base_link` 偏移 (0, 0, 0.55)）。
- FAST-LIO 的点云输入即在该帧采集（雷达本体坐标系）。

### `weapon_arm_base_link`
- **武器头机械臂**底座坐标系（URDF `weapon_arm_joint`，固定到 `base_link`）。
- 用途：夹取/释放武器端头（`ActionGroupCmd id=1/2`），拾取指令由上位机发出，
  下位机自行控制对应机械臂，上位机无需区分两臂。
- **当前 xyz=(0.05, 0.15, 0.30) 为占位估测值**，安装位置确定后需用卷尺测量
  相对 `base_link` 的实际偏移并更新 URDF `weapon_arm_joint` 的 `origin`。
- 未来若需要末端执行器帧（夹爪/感应器），在此节点下继续扩展子帧。

### `arm_base_link`
- **KFS机械臂**底座坐标系（URDF `arm_base_joint`，相对 `base_link`
  偏移 (0, 0, 0.05)），机械臂结构（抬升机构 + 前平移机构 + 末端横杆）以此为根。

### `arm_deployed_link`
- 代表机械臂**精对齐动作组完成后的展开姿态**：抬升到位、前平移伸出、
  末端横杆（吸盘+USB相机）旋转至朝下，此时末端横杆中心相对
  `arm_base_link` 的固定位姿。
- 由于抬升/前伸机构没有连续关节角度反馈（仅有"抬升1/抬升2/复位"等
  离散动作组信号），不建模为运动学链，而是直接给出展开后的固定位姿，
  **数值为占位估测，需结合实物测量校正**。
- 作用：作为 `usb_camera_link`/`suction_link` 的父帧，在精对齐阶段
  确定末端执行器与 `base_link`/相机像素坐标系之间的几何关系。

### `usb_camera_link` / `usb_camera_optical_frame`
- 末端横杆上的 **USB 相机**坐标系，精对齐阶段开启，用于采集KFS方块的
  图像数据进行滤色/边缘对齐。
- `usb_camera_link`：相机机械安装坐标系，相对 `arm_deployed_link`
  绕Y轴+90°，使其"前"轴（X）指向地面（镜头朝下）。
- `usb_camera_optical_frame`：图像处理使用的光学坐标系约定
  （Z沿镜头朝向/朝地面，X右，Y下），与 `camera_optical_frame` 同样的
  转换关系（rpy = -π/2, 0, -π/2）。
- **关键作用**：精对齐把图像像素偏移换算成底盘 `vx`/`vy` 修正量时，
  需要知道 `usb_camera_optical_frame` 的 X/Y 轴相对 `base_link`
  X(前)/Y(左) 轴的旋转关系，才能正确确定"图像中目标偏左 → 底盘该往左
  还是往右平移"的符号与缩放，避免硬编码导致方向错误。

### `suction_link`
- 末端横杆上的**吸盘**坐标系，相对 `arm_deployed_link` 与
  `usb_camera_link` 存在一个小的固定偏移（文档中提到"位置可能会有点
  偏移"），数值同样为占位估测，需现场标定。
- 作用：精对齐完成、下发抓取信号后，吸盘坐标系即为实际抓取作业点，
  与 USB 相机看到的目标位置之间的偏移由该 TF 给出，供后续抓取逻辑
  做坐标修正。

### `wheel_fl_link` / `wheel_fr_link` / `wheel_rl_link` / `wheel_rr_link`
- 四个万向轮的坐标系（前左/前右/后左/后右，相对 `base_link` 的
  安装偏移分别为 (±0.27, ±0.25, 0)），随轮子转动角度动态更新
  （`joint_state_publisher` 提供 `continuous` 关节角度）。
- 主要用于可视化（RViz 中轮子的旋转动画），不参与定位/导航计算。

## 4. 两条"世界坐标系"分支说明

TF树中 `odom` 下有两个并列分支：

1. **`camera_init → body`**：FAST-LIO 自身的 SLAM 定位结果（里程计精度高，
   但存在累积漂移），仅作为 EKF 的一路观测输入。
2. **`odom → base_footprint → base_link → ...`**：EKF 融合
   轮式里程计 + FAST-LIO `/Odometry` + IMU 后的最终结果，是
   `waypoint_navigator`/Nav2 实际使用的机器人位姿。

`odom → camera_init`（identity）是连接两个分支的桥梁：没有它，
EKF 在 `lookupTransform` 时会因找不到 `camera_init` 相对 `odom` 的变换而
**静默丢弃** `/Odometry` 观测，导致 FAST-LIO 的绝对定位贡献完全失效。

## 5. 离地高度核算

| 部件 | 离地高度 |
|---|---|
| `base_link`（底盘主体底面） | 0.053905 m（= 轮半径） |
| `camera_link`（相机） | 0.053905 + 0.45 ≈ 0.504 m |
| `lidar_link`（雷达） | 0.053905 + 0.55 ≈ 0.604 m | 


map：场地绝对坐标系，固定不变，绝对定位修正发生在 waypoint_navigator 参数层面（不在TF里）（坐标为实际启动点坐标0,0,0；于是就需要map系转camera_init转base_link转game这些）。
odom：EKF融合输出的"漂移参考系"，原点=EKF启动点，由轮式里程计(速度)+FAST-LIO(位置)+IMU(角速度/加速度)三路融合得到——下位机轮式里程计只是其中一个输入源，不是 odom 帧本身。（坐标为全局坐标，EKF节点启动时就定在世界坐标系不变）