# RC2026武林探秘 R2全自动机器人上位机系统🤖

## 本项目由广州美术学院湾区创新学院Artlnnov战队算法组开源和维护

  - **项目负责人**：刘卓轩（联系方式：18927743799/📫：HGZQ2108299415@outlook.com，不许闲的没事打电话给我）【R2唯一上位机】
  - **副组长**：李嘉禾

## 一、项目概览✨️

本项目为**Artlnnov战队**参加 Robocon 2026 比赛 R2 机器人的上位机系统，基于 ROS2 Humble 构建，运行在 AMD Ryzen 7 8845HS 小主机上。主要覆盖YOLO视觉处理、SLAM、导航、决策模块、全流程状态机、底盘与机械臂移动和动作、Gazebo仿真，以及与下位机之间的串口通信。

> 该系统串口通信包使用**重庆大学开源的AutoSerialBridge项目**，仅修改protocol.yaml部分；以及仿真模型环境使用**重庆大学开源的RC2026 Gazebo Classic 仿真场地功能包**，已保留相应许可证，地图右半场点云由**华南理工大学robotic战队**提供，十分感谢开源作者的贡献。🙏🙏🙏

相应的开源仓库地址如下：https://github.com/ConQU2026/rc2026_field.git

https://github.com/ConQU2026/auto_serial_bridge.git

---
**R2**是一台无人操控的**全自动机器人**，需要在比赛过程中完成：
- 武馆区域：
  - 识别武馆内相应位置的己方武器头，完成拾取；
  - 移动到组合区域与拾取完成矛杆的R1进行武器组合；
- 梅林区域：
  - 识别梅林区域内的真KFS，完成拾取；
  - 通过爬升和下降以及路径规划通过梅林台阶；
- 对抗区域：
  - 完成上坡进入对抗区域；
  - 移动到九宫格前，并放置KFS至九宫格第二层；
  - 到达合体区域与R1进行合体，并将KFS放置在九宫格最高层；
---
系统分为两个独立工作空间：`ros2_vision_project/ros2_ws` 为真机工作空间（实际运行环境，实际部署的部分），`ros2_vision_project/sim_ws` 为仿真工作空间。系统主入口为 `ros2_vision_project/scripts/launch_rc2026.py`，可交互式启动各个功能模块。


## 二、目录结构✨️

```text
GAFA-Artlnnov.RC2026/
     |-- README.md  # 项目说明文档
     |-- ros2_vision_project/ # ROS2上位机系统（详细看内置文档）
     └── LICENSE  # MIT开源协议
```

## 三、硬件使用✨️

| 硬件设备 | 用途 |
| --- | --- |
| Intel RealSense D435i | RGB-D 图像、武器头/KFS对齐、IMU 数据输入、R1R2合体检测ArUco |
| Livox Mid-360S | 点云输入、激光惯性里程计、预建点云地图定位 |
|外接USB相机| KFS精对齐|
|红外学习模块| R1R2通信|
| AMD Ryzen 7 8845HS 小主机 | 真机实际部署平台 |

## 四、技术栈✨️

| 层级 | 技术/工具 | 用途 |
| --- | --- | --- |
| 系统平台 | Ubuntu22.04 + ROS2 Humble | 上位机运行环境与节点通信框架 |
| 计算平台 | AMD Ryzen 7 8845HS 小主机/Jetson orin nano 8GB | 真机实际部署平台，YOLO 当前使用 CPU 推理 |
| 主要语言 | Python3.10、C++ | Python 用于视觉/决策/导航节点，C++ 用于串口桥底层 |
| 视觉检测 | Ultralytics YOLOv8、OpenCV、cv_bridge | 武器头/KFS 目标检测、图像处理、调试可视化 |
| RGB-D/IMU | Intel RealSense D435i、realsense2_camera | RGB-D 图像、相机内参、IMU 数据输入 |
| 激光雷达定位 | Livox Mid-360S、livox_ros_driver2、FAST-LIO2 | 点云输入、激光惯性里程计、预建点云地图定位 |
| 多源融合 | robot_localization EKF、TF2 | 融合 FAST-LIO、轮式里程计和 IMU，输出统一 TF |
| 地图重定位 | Open3D ICP | 基于左右半场 PCD 地图修正 FAST-LIO 初始偏移和漂移 |
| 导航控制 | 自研 WaypointNavigator、nav2_msgs Action 接口 | 兼容 `NavigateToPose`，实际执行 PID 路径点导航 |
| 决策系统 | GameController 状态机、梅林 BFS 路径规划 | 编排武馆、梅林、对抗区、KFS 放置与合体流程 |
| 精对齐 | USB 相机三棱检测、D435i ArUco 检测 | KFS 吸盘精对齐、R1 合体视觉对齐 |
| 下位机通信 | AutoSerialBridge、UART、CRC8 协议 | 与 STM32 交换底盘速度、动作组、反馈、启动/R1 信号 |
| 仿真 | Gazebo Classic、rc2026_field、rc2026_sim | 场地/KFS/机器人仿真、重定位与里程计漂移测试 |

## 五、核心算法模块（ros2_ws）✨️

| 类别 | 核心文件/模块 | 算法或机制 | 用途 |
| --- | --- | --- | --- |
| RGB-D 目标检测 | `vision_detector/vision_detector/detector_node.py`、`yolov8_detector.py` | YOLOv8 目标检测 + 模型热切换 | 识别武器头、真/假 KFS 等目标，并支持武馆模型与梅林 KFS 模型运行时切换 |
| 深度测距与三维反投影 | `vision_detector/vision_detector/utils.py` | 深度图采样、像素坐标转相机三维坐标 | 从检测框中心和 D435i 深度图计算目标相对相机的三维位置，发布 `/vision/raw_target` |
| 视觉伺服状态机 | `decision_processor/decision_processor/processor_node.py`、`robot_decision.py` | SEARCHING/ALIGNING/MOVING/ARRIVED/PICKING 状态机 | 在 `ALIGN_WEAPON` 和 `ALIGN_KFS` 阶段根据目标偏角与距离输出底盘微调指令 |
| 视觉目标滤波 | `decision_processor/decision_processor/kalman_filter.py` | 二维卡尔曼滤波 + 单帧跳变剔除 | 平滑视觉目标在 `base_link` 下的位置，降低检测抖动导致的底盘震荡 |
| 目标确认 | `decision_processor/decision_processor/target_confirmation.py` | 多帧确认、连续丢失判定 | 防止单帧误检直接触发对齐/抓取，提升视觉状态机稳定性 |
| 场景策略 | `decision_processor/decision_processor/scenarios/scenario_wuguan.py`、`scenario_meilin.py` | 类别、置信度、距离规则评估 | 武馆场景判断可拾取武器头；梅林场景区分 REAL/FAKE/R1 KFS 并决定拾取或忽略 |
| 路径点导航 | `decision_processor/decision_processor/waypoint_navigator.py` | 自研 PID 路径点导航 + `NavigateToPose` Action 兼容层 | 替代 Nav2 controller，读取 TF 位姿后执行 XY 到位、yaw-only 原地转向和可选视觉伺服交接 |
| 坐标变换 | `decision_processor/decision_processor/tf_manager.py`、`waypoint_navigator.py` | camera/base/arm TF 转换，game/gazebo/fastlio 坐标换算 | 统一视觉、机械臂、导航和比赛场地坐标，支持左右半场与仿真坐标转换 |
| 梅林路径规划 | `decision_processor/decision_processor/meilin_path_planner.py` | 3x4 方块有向 BFS + KFS 覆盖约束 + 假 KFS 障碍 | 根据真/假 KFS 输入规划可通行路径，生成入口、拾取点、爬升/下降触发点 |
| 比赛总控 | `decision_processor/decision_processor/game_controller.py` | 全流程有限状态机 + 子模式调度 | 编排武馆、梅林、对抗区、KFS 放置、R1 合体和超时保护，是比赛流程调度核心 |
| IMU 姿态与坡度识别 | `decision_processor/decision_processor/imu_processor.py` | 互补滤波、坡度等级分类 | 融合 D435i gyro/accel 得到 pitch/roll/yaw_rate，为爬坡退出和坡度感知控制提供依据 |
| 坡度感知运动规划 | `decision_processor/decision_processor/motion_planner.py` | 距离/角度控制、坡度速度系数、梯形速度规划 | 为视觉伺服接近目标和坡面运动提供速度规划基础 |
| KFS 精对齐 | `decision_processor/decision_processor/fine_align_node.py` | LAB 滤色、边缘检测、Hough 竖线、两棱/三棱判定、两级减速 | 使用机械臂末端 USB 相机判断 KFS 是否居中，输出横向微调速度给底盘 |
| 合体视觉对齐 | `decision_processor/decision_processor/dock_align_node.py` | ArUco 检测、solvePnP 位姿估计、三轴比例控制、滑动窗口确认 | 在对抗区识别 R1 标志，控制 R2 横向、前后和偏航完成合体前对齐 |
| 地图重定位 | `rc2026_navigation/scripts/map_relocalizer.py` | Open3D 点到面 ICP + 在线参数更新 | 将 FAST-LIO 当前点云与左/右半场 PCD 地图匹配，修正导航初始偏移和区域漂移 |
| 点云地图工具 | `rc2026_navigation/scripts/save_cloud_map.py`、`pcd_to_gridmap.py`、`generate_field_map.py` | 点云保存、PCD 栅格化、场地地图生成 | 生成和维护定位/导航所需的点云地图与二维地图资源 |
| 多源速度仲裁 | `cmd_vel_bridge/cmd_vel_bridge/bridge_node.py` | fine_align > dock_align > cmd_vel 优先级仲裁、限幅、rad/s 到 deg/s 转换 | 统一普通导航、KFS 精对齐和合体对齐的底盘速度输出，最终发布 `/serial/chassis_cmd` |
| 轮式里程计转换 | `cmd_vel_bridge/cmd_vel_bridge/wheel_odom_publisher.py` | STM32 Twist 反馈到 `nav_msgs/Odometry` 转换 | 将 `/feedback/wheel_odom` 转为 `/odom/wheel`，供 EKF 融合定位使用 |

### 使用的开源算法与库

| 开源算法/库 | 在项目中的使用位置 | 具体用途 |
| --- | --- | --- |
| **YOLOv8 / Ultralytics** | `vision_detector` | 目标检测模型，用于识别武器头、真 KFS、假 KFS 和 R1 相关目标；当前在 AMD 8845HS 小主机上使用 CPU 推理 |
| **OpenCV** | `fine_align_node.py`、`dock_align_node.py`、`triple_edge_align.py` | 图像采集、颜色空间转换、形态学处理、Canny 边缘检测、HoughLinesP 直线检测和调试显示 |
| **ArUco** | `dock_align_node.py` | 基于 OpenCV ArUco 模块检测 R1 上的标志，结合 `solvePnP` 估计标志相对相机的位姿，为合体视觉伺服提供横向、距离和偏航误差 |
| **FAST-LIO2** | `rc2026_navigation`、`fast_lio` | 激光雷达与 IMU 紧耦合激光惯性里程计，输出实时点云配准和机器人位姿 |
| **Extended Kalman Filter（EKF）** | `robot_localization` + `config/ekf.yaml` | 融合 FAST-LIO、STM32 轮式里程计和 IMU，输出 `/odom` 以及 `odom -> base_link` TF |
| **Open3D ICP** | `map_relocalizer.py` | 将实时 `/cloud_registered` 点云与左右半场 PCD 地图进行点到面 ICP 匹配，修正初始定位偏移和运行过程中的 SLAM 漂移 |
| **TF2** | `tf2_ros`、`tf_manager.py` | 管理 `map`、`odom`、`base_link`、相机、雷达和机械臂坐标系之间的变换 |
| **Nav2 NavigateToPose Action** | `waypoint_navigator.py`、`game_controller.py` | 使用 Nav2 标准 Action 消息作为导航接口；实际路径执行由项目自研 WaypointNavigator 完成，不依赖 Nav2 controller |
| **Gazebo Classic ROS** | `sim_ws/rc2026_sim`、`rc2026_field` | 仿真全向底盘、RGB 相机、3D/2D 激光雷达、场地和 KFS 模型 |
| **cv_bridge** | `vision_detector`、`dock_align_node.py` | ROS `sensor_msgs/Image` 与 OpenCV `numpy` 图像之间的转换 |
| **AutoSerialBridge** | `auto_serial_bridge-main` | 基于开源串口桥框架实现 ROS2 与 STM32 之间的协议解析、CRC8 校验、心跳和消息收发 |

### 项目自研算法

- **WaypointNavigator PID 导航**：读取 TF 位姿，在 game 坐标系下执行 XY 平移、yaw-only 原地转向和视觉伺服交接。
- **梅林有向 BFS 路径规划**：结合真/假 KFS 分布、方块高度和不可后退规则规划可行路径。
- **二维目标卡尔曼滤波与多帧确认**：对视觉目标位置进行平滑，并通过连续帧确认/丢失判定抑制误检。
- **互补滤波坡度估计**：融合 D435i 加速度计和陀螺仪，计算 pitch、roll、yaw rate 及坡度等级。
- **KFS 三棱精对齐算法**：基于 OpenCV 色彩分割、边缘和棱线检测，计算透视误差与横向居中误差。
- **比赛总控有限状态机**：编排武馆、梅林、对抗区、KFS 放置、R1 合体和超时保护等比赛阶段。

## 六、系统架构链路✨️

推荐启动链路：

```text
scripts/launch_rc2026.py
  -> rc2026_bringup/full_system.launch.py
  -> 传感器 + 定位 + 视觉 + 决策 + 导航 + 串口通信
```

真机运行主链路：

```text
传感器层
  D435i RGB-D/IMU
  Livox Mid-360S
  STM32 轮式里程计
  USB 精对齐相机
  红外学习模块

定位层
  Livox 点云/IMU -> FAST-LIO2 -> /cloud_registered
  STM32 wheel odom -> wheel_odom_publisher -> /odom/wheel
  FAST-LIO + wheel odom + IMU -> robot_localization EKF -> /odom + TF
  map_relocalizer -> Open3D ICP -> 更新 waypoint_navigator loc_offset

感知层
  D435i RGB-D -> YOLOv8 CPU 推理 -> /vision/raw_target
  USB 相机 -> fine_align_node -> /fine_align/cmd/status
  D435i ArUco -> dock_align_node -> /dock_align/cmd/status

决策层
  game_controller
    -> 发布 /game/phase 控制视觉伺服阶段
    -> 调用 navigate_to_pose 调度路径点导航
    -> 发布 /vision/switch_model 切换武馆/KFS模型
    -> 发布 /serial/action_group_cmd、/serial/meilin_cmd、/serial/confront_climb_cmd

导航与控制层
  waypoint_navigator -> /cmd_vel
  processor_node -> /serial/chassis_cmd
  fine_align_node / dock_align_node -> cmd_vel_bridge
  cmd_vel_bridge -> 多源速度优先级仲裁 -> /serial/chassis_cmd

执行层
  auto_serial_bridge -> UART -> STM32
  STM32 -> 底盘/机械臂/爬升机构
  STM32反馈 -> /feedback/*、/game/start_signal、/game/r1_signal
```

核心流程可以概括为：交互脚本收集比赛参数，`GameController` 负责全流程状态机，`WaypointNavigator` 负责全局路径点移动，`processor_node`、`fine_align_node` 和 `dock_align_node` 负责不同阶段的视觉伺服，最终所有底盘/动作指令通过串口桥发送给 STM32 执行。

## 七、系统时序✨️
![时序图](/ros2_vision_project/GAFA-Artlnnov.RC2026-main_2026-08-03T11_17_24.474Z.png)

## 八、完整的TF树✨️
![TF树](/ros2_vision_project/Screenshot%20from%202026-08-04%2018-03-35.png) 
map -> odom -> base_link 
## 九、Git克隆项目和运行✨️
系统：Ubuntu 22.04 LTS (不会装ubuntu的自己看[这个](ros2_vision_project/视觉组教程/)，或者自己退队吧，我已经不想再帮任何人装系统了，这一年给我装燃尽了)

请确保已安装 **ROS2 Humble**、**Python3.10（这个自带的）**、**C++编译器** 。

或者直接使用本项目提供的 Docker 镜像运行，配置教程见 [配置指南](/ros2_vision_project/docker/docker配置指南.md)。个人建议如果是新手，直接使用 Docker 镜像运行，避免环境配置问题。

### 接线
1. 我这里**红外学习模块**和**下位机UART通信**均用的CH340 USB转TTL模块，红外学习模块接收端的TXD接CH340的RXD，RXD接CH340的TXD，GND接GND，**VCC接VCC**。下位机UART通信的TXD接CH340的RXD，RXD接CH340的TXD，GND接GND，**VCC不接**。
2. 先接上UART通信模块，到USB主机口，再接两个红外学习模块[***因为我为了方便管理这几个接CH340的东西于是默认UART通信占的USB0口，红外学习模块分别占USB1,2口***]。
3. 然后接上USB相机，深度相机和激光雷达，雷达接LAN口，供电接下位机（这步无先后之分）。

### 安装上位机系统和启动
```bash
# 1. 加载 ROS2 Humble
source /opt/ros/humble/setup.bash

# 2. 从GitHub克隆项目到本地
cd "$HOME"  #或者自己喜欢放哪就cd到哪
git clone https://github.com/GAFA-Artlnnov/GAFA-Artlnnov.RC2026-main.git


# 3. 安装依赖
cd "$HOME/GAFA-Artlnnov.RC2026-main/ros2_vision_project" && bash docker/install_host_deps.sh

#注意：Librealsense SDK驱动、MID-360S雷达驱动、livox_ros_driver2、fast_lio2需要自行下载安装和配置(放到后面了，解压即可)

# 4. 启动系统
cd "$HOME/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws"
colcon build --symlink-install  #构建全部
#如果构建失败就单独构建失败的包，如果是串口通信包构建失败则阅读根据串口通信包的README.md进行构建

cd "$HOME/GAFA-Artlnnov.RC2026-main/ros2_vision_project"
source ros2_ws/install/setup.bash
python3 scripts/launch_rc2026.py  #启动系统交互程序

# 5.启动底盘运动（另开一个终端执行，如果没有遥控器的话）
ros2 topic pub --once /game/start_signal std_msgs/msg/UInt8 '{data: 1}'

``` 
Librealsense SDK：https://github.com/realsenseai/librealsense/blob/master/doc/installation.md 

MID-360S雷达驱动、livox_ros_driver2、fast_lio2：``` 
通过网盘分享的文件：MID-360S的开发资料.zip
链接: https://pan.baidu.com/s/17kniBEgbBarZjtuvZPW9-Q 提取码: geak  ```
**内部的livox_ros_driver2和fast_lio2的mid-360s参数文件是经过我修改的** 

**常见问题**：
| 问题 | 解决方案 |
| --- | --- |
构建失败，提示找不到某些依赖包 | 先看看该包是否存在，或者检查当下路径是否是在正确的目录（ros2_vision_project/ros2_ws）下 |
|构建失败，某个包单独构建失败|重新单独构建该包，colcon build --symlink-install --packages-select vision_detector vision_detector /colcon build --symlink-install --packages-select decision_processor decision_processor /colcon build --symlink-install --packages-select vision_msgs_custom vision_msgs_custom|
|构建失败，提示串口通信包构建失败|请阅读串口通信包的README.md，按照里面的步骤进行构建|
|构建失败，提示构建decision_processor时环境错误|```export AMENT_PREFIX_PATH=$HOME/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws/install/decision_processor:$AMENT_PREFIX_PATH && export PYTHONPATH=$HOME/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws/install/decision_processor/lib/python3.10/site-packages:$PYTHONPATH```
|如果是在docker中运行，构建失败，原因跟上面一样|```export AMENT_PREFIX_PATH=/ros2_vision_project/ros2_ws/install_docker/decision_processor:$AMENT_PREFIX_PATH  && export PYTHONPATH=/ros2_vision_project/ros2_ws/install_docker/decision_processor/lib/python3.10/site-packages:$PYTHONPATH```|
|有时colonel build会报错|可能是因为之前的构建文件残留导致的，可以尝试清理构建文件后重新构建：```rm -rf build/ install/ log/```|
|缺少依赖|直接把报错扔给AI就知道缺什么了|
### 注意：用docker运行的话，需要修改以下路径：

1. full_system.launch.py中找到WEIGHTS_DIR直接修改为：WEIGHTS_DIR = ‘/ros2_vision_project/ros2_ws/src/vision_detector/weights‘
2.  ros2_ws/src/vision_detector/vision_detector/model_switcher.py中找到WEIGHTS_DIR直接修改为：WEIGHTS_DIR = ‘/ros2_vision_project/ros2_ws/src/vision_detector/weights‘

### 一些可能要用的命令
```bash
# 查看检测视觉结果
source install/setup.bash
ros2 run rqt_image_view rqt_image_view #查看相机图像，可在界面选择查看color/image_raw、depth/image_rect_raw、detector/image_raw等图像
ros2 topic echo /detections #查看检测结果

# 手动模型切换：
ros2 run vision_detector model_switcher
#按回车，然后输入模型名字如yolov8n,best,回车确认切换（默认best.pt）切换不了可能是模型损坏或者路径错误，检查模型文件是否存在，路径是否正确，模型文件是否完整。

# 决策状态
ros2 topic echo /decision/state

# 底盘指令
ros2 topic echo /serial/chassis_cmd

# 视觉帧率
ros2 topic hz   /vision/raw_target
#==================================================================================================================

#ssh连接主机测试（一般调车用）
ssh hgzq123@192.XXX.XX.X  #后面是主机IP,用ifconfig查看当前ip地址)
docker exec -it <docker的容器名> bash  #进入容器（用docker才需要）
cd /ros2_vision_project/ros2_ws
source install/setup.bash
#===================================================================================================================

#建图：
#终端 1 — 启动 Livox Mid-360S 雷达驱动：
source /opt/ros/humble/setup.bash
source ~/livox_ros_driver2/install/setup.bash
ros2 launch livox_ros_driver2 msg_MID360s_launch.py

#终端 2 — 启动 FAST-LIO：
source /opt/ros/humble/setup.bash
source ~/FAST_LIO/install/setup.bash
ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml

#终端 3 — RViz 查看实时建图：
rviz2
#Fixed Frame 设为 camera_init
#添加 PointCloud2 显示，topic: /cloud_registered
#手持/推动机器人慢速绕场地走一圈（速度 < 1m/s，转弯尽量平滑）
#终端 4 — 建图完成后保存 PCD：
#FAST-LIO 默认将 PCD 保存到 ~/.ros/scans/ 或 PCD/ 目录。确认 pcd_save_en: true 已启用（在 mid360.yaml 中）。建图结束后 Ctrl+C 停止 FAST-LIO，它会自动保存 scans.pcd。
source /opt/ros/humble/setup.bash
python3 ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws/src/rc2026_navigation/scripts/save_cloud_map.py \
    --duration 60 \
    --output ~/FAST_LIO/PCD/scans.pcd


#===================================================================================================================

#仿真：
#终端 1 — 启动仿真包：
cd ~/GAFA-Artlnnov.RC2026-main/ros2_vision_project/sim_ws
source install/setup.bash
ros2 launch rc2026_sim simulation.launch.py use_game_controller:=true
#右半场：
ros2 launch rc2026_sim simulation.launch.py field_side:=right use_game_controller:=true

#终端 2 — 自定义真假kfs标签：
ros2 topic pub --once /game/kfs_input std_msgs/String "data: 'real:3,8 fake:1,6'"

#终端 3 — 开始启动：
ros2 topic pub --once /game/start_signal std_msgs/msg/UInt8 '{data: 1}'


#===================================================================================================================

# 1. 启动信号（替代物理按钮）
ros2 topic pub --once /game/start_signal std_msgs/msg/UInt8 '{data: 1}'

# 2. R1 组装完成信号（模拟R1通知R2武器端头已组装好）
#    game_controller 在 WAIT_ASSEMBLY 阶段等这个
ros2 topic pub --once /feedback/assembly std_msgs/msg/UInt8 '{data: 2}'
rf
# 3. R1 进入梅林信号（模拟R1通知R2可以进梅林了）
#    game_controller 在 WAIT_ENTER_MERLIN 阶段等这个
ros2 topic pub --once /game/r1_signal std_msgs/msg/UInt8 '{data: 2}'

# 4. R1 合体指令（模拟R1通知R2可以合体）
#    game_controller 在 WAIT_MERGE 阶段等这个
ros2 topic pub --once /game/r1_signal std_msgs/msg/UInt8 '{data: 3}'

```

## 十、许可✨️

本项目自研部分采用 MIT License 开源，详见 [LICENSE](./LICENSE)。

项目中的第三方组件保留各自原始许可证声明，使用、修改和分发时请同时遵守对应子项目的许可条款。

--- 

2026是我参加RC的第一年，很高兴在这个学校有志同道合的同学跟我一起完成这个比赛，同时也取到了不错的成绩，今年是我们从0到1的开始，很感谢其他学校在这一年给我们给予的帮助和指导，希望在更往后的未来我们的后继团队有机会拿到更好的成绩，如果可以...我还想打RM喵~ 

呜呜呜，写的代码太💩了，大佬们轻点喷，如果可以，请给我提issues，我会很感激的，谢谢喵~

**致Artlnnov：同心笃行，百挫弥坚；虽千万人，吾往矣🚀**