# RC2026 Docker 配置与运行指南

本文档适用于当前仓库的 Docker 环境配置，覆盖 Docker Engine 安装、CPU 镜像构建、ROS2 工作空间编译、真机硬件访问和 `sim_ws` 仿真运行。

## 1. 当前部署结论

当前真实机器人运行平台为：

| 项目 | 当前配置 |
| --- | --- |
| 宿主机 | Ubuntu 22.04 LTS，x86_64/amd64 |
| CPU | AMD Ryzen 7 8845HS 小主机 |
| YOLO 推理 | CPU，实际不使用 CUDA |
| ROS2 | ROS2 Humble |
| 当前 Docker 主线 | `Dockerfile.ros2` + Compose 服务 `yolov8-ros2-cpu` |
| 项目挂载路径 | 容器内 `/ros2_vision_project` |
| 真机工作空间 | `/ros2_vision_project/ros2_ws` |
| 仿真工作空间 | `/ros2_vision_project/sim_ws` |

因此，当前设备不需要安装 NVIDIA 驱动、CUDA、NVIDIA Container Toolkit，也不需要下载 CUDA 版 PyTorch wheel。旧文档中的 `gpu`、`cu126`、`cu128` 和 Jetson 命令不属于 AMD 8845HS 的实际运行流程。

### 1.1 宿主机和容器命令标记

本文档使用以下标记区分命令执行位置：

- **宿主机终端**：Ubuntu 系统终端，路径通常是项目所在的真实路径。
- **容器终端**：通过 `docker exec` 进入的 ROS2 容器，项目路径固定为 `/ros2_vision_project`。

除非特别说明，命令需要逐行执行。

## 2. 安装 Docker Engine 和 Docker Compose

### 2.1 检查系统架构

在**宿主机终端**执行：

```bash
uname -m
lsb_release -ds
```

AMD 8845HS 小主机应当输出 `x86_64` 和 Ubuntu 22.04。Docker 官方支持 Ubuntu 22.04 的 amd64 架构。

### 2.2 删除可能冲突的旧 Docker 包

如果之前安装过 Ubuntu 自带的旧版 Docker Compose 或 Docker Engine，建议先执行：

```bash
sudo apt remove docker.io docker-compose docker-compose-v2 docker-doc \
  podman-docker containerd runc
```

如果提示某些软件包没有安装，可以忽略。

### 2.3 添加 Docker 官方软件源

在**宿主机终端**执行：

```bash
sudo apt update
sudo apt install -y ca-certificates curl

sudo install -m 0755 -d /etc/apt/keyrings
sudo curl -fsSL https://download.docker.com/linux/ubuntu/gpg \
  -o /etc/apt/keyrings/docker.asc
sudo chmod a+r /etc/apt/keyrings/docker.asc

sudo tee /etc/apt/sources.list.d/docker.sources > /dev/null <<EOF
Types: deb
URIs: https://download.docker.com/linux/ubuntu
Suites: $(. /etc/os-release && echo "${UBUNTU_CODENAME:-$VERSION_CODENAME}")
Components: stable
Architectures: $(dpkg --print-architecture)
Signed-By: /etc/apt/keyrings/docker.asc
EOF

sudo apt update
```

### 2.4 安装 Docker、Buildx 和 Compose 插件

```bash
sudo apt install -y \
  docker-ce \
  docker-ce-cli \
  containerd.io \
  docker-buildx-plugin \
  docker-compose-plugin
```

当前项目使用 `docker compose` 子命令，不使用已经过时的独立 `docker-compose` 命令。

### 2.5 启动 Docker 服务并验证

```bash
sudo systemctl enable --now docker
sudo systemctl status docker --no-pager

sudo docker version
sudo docker compose version
sudo docker run --rm hello-world
```

看到 `Hello from Docker!` 表示 Docker Engine 基本安装成功。

### 2.6 配置当前用户免 sudo 使用 Docker

Docker 默认使用 root 权限访问 Unix socket。为了后续直接执行 `docker compose`，在**宿主机终端**执行：

```bash
sudo groupadd docker 2>/dev/null || true
sudo usermod -aG docker "$USER"
newgrp docker
```

然后验证：

```bash
docker run --rm hello-world
docker compose version
```

如果仍提示权限不足，注销当前用户并重新登录，或重启系统后再执行验证命令。

> 注意：`docker` 用户组拥有近似 root 的宿主机权限，只应将可信用户加入该组。

## 3. 宿主机硬件和图形环境

### 3.1 安装硬件检查工具

在**宿主机终端**执行：

```bash
sudo apt update
sudo apt install -y usbutils v4l-utils git

sudo usermod -aG video "$USER"
sudo usermod -aG dialout "$USER"
sudo usermod -aG plugdev "$USER"
```

加入用户组后需要重新登录，或者重启系统。

### 3.2 检查 RealSense、USB 相机和串口

```bash
lsusb
ls -l /dev/video* 2>/dev/null
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
ls -l /dev/serial/by-id/* 2>/dev/null
```

常见设备用途：

| 设备 | 用途 |
| --- | --- |
| Intel RealSense D435i | RGB-D、IMU、ArUco 合体视觉 |
| `/dev/video0` | 机械臂末端 USB 相机，当前精对齐默认使用 |
| `/dev/ttyUSB0` | STM32 或 USB 串口设备，实际编号以现场枚举结果为准 |
| `/dev/ttyUSB1` | 主红外学习模块默认串口 |
| `/dev/ttyUSB2` | 第二 KEY2 红外模块默认串口 |

实际串口编号可能随着插拔顺序变化。比赛运行时优先使用 `/dev/serial/by-id/` 下的固定设备名，并同步修改 launch 参数或串口协议配置。

### 3.3 配置 X11 图形显示

当前项目的调试模式、`rqt_image_view`、RViz2 和 OpenCV 预览需要访问宿主机 X11。

在**宿主机终端**执行：

```bash
echo "$DISPLAY"
xhost +local:docker
```

`xhost +local:docker` 只在当前图形会话内生效。停止 Docker 调试后，可以恢复限制：

```bash
xhost -local:docker
```

当前 `docker-compose.ros2.yml` 的 GPU 服务已经挂载 `/tmp/.X11-unix`，CPU 服务默认只挂载项目目录和 USB 总线。如果需要在 CPU 容器内运行 RViz2、`rqt_image_view` 或 `launch_rc2026.py` 的调试预览，需要在 CPU 服务的 `volumes` 下补充：

```yaml
volumes:
  - ../:/ros2_vision_project
  - /tmp/.X11-unix:/tmp/.X11-unix
```

修改后需要重新创建容器：

```bash
cd "$PROJECT_DIR/docker"
docker compose -f docker-compose.ros2.yml up -d --force-recreate yolov8-ros2-cpu
```

如果容器内仍找不到串口或视频设备，可以在 CPU 服务的 `volumes` 下增加：

```yaml
- /dev:/dev
```

当前 Compose 已设置 `privileged: true`，但不同 Docker/主机环境对设备可见性的表现可能不同。增加 `/dev:/dev` 后，容器会直接看到宿主机设备，权限范围也更大，只应在可信主机上使用。

## 4. 获取项目并检查 Docker 文件

如果项目已经在本机，直接进入项目目录即可。下面以当前仓库路径为例，实际路径需要按本机修改。

在**宿主机终端**执行：

```bash
export PROJECT_DIR=/home/hgzq/GAFA-Artlnnov.RC2026-main/ros2_vision_project
cd "$PROJECT_DIR"

test -f docker/Dockerfile.ros2
test -f docker/docker-compose.ros2.yml
test -f docker/run_docker_ros2.sh
test -d ros2_ws/src
test -d sim_ws/src
```

如果是从 Git 仓库获取项目：

```bash
cd /home/hgzq
git clone https://github.com/HGZQ1/GAFA-Artlnnov.RC2026.git
export PROJECT_DIR=/home/hgzq/GAFA-Artlnnov.RC2026-main/ros2_vision_project
cd "$PROJECT_DIR"
```

## 5. 当前 CPU 镜像说明

`docker/Dockerfile.ros2` 是当前 AMD x86_64 主机的主线镜像配置，主要包含：

- Ubuntu 22.04 / ROS2 Humble Desktop Full；
- Python3、colcon、rosdep、OpenCV；
- RealSense ROS2 驱动和 librealsense SDK；
- Ultralytics YOLOv8 及 `docker/requirements.txt` 中的 Python 依赖；
- RViz2、rqt、TF2、`vision_msgs`、`cv_bridge` 等 ROS2 组件。

这个 Dockerfile 本身没有单独写 GPU / CUDA / CUDA wheel 安装步骤；`torch` 若进入容器，来源通常是 Python 依赖解析结果，而不是显式的 CUDA 配置。验证时应当关注 CPU 是否可用，而不是 CUDA：

```bash
python3 -c "import torch; print(torch.__version__); print('CUDA available:', torch.cuda.is_available()); print('device: cpu')"
```

在 AMD 小主机上 `CUDA available: False` 是预期结果。

### 5.1 当前镜像没有自动包含的外部依赖

完整真机 `full_system.launch.py` 还会查找以下外部 ROS2 包：

- `livox_ros_driver2`
- `fast_lio`
- `robot_localization`
- `serial_driver`

其中 `livox_ros_driver2` 和 `fast_lio` 不在当前仓库的 `ros2_ws/src` 中，当前 Compose 也没有自动挂载外部工作空间。因此：

- 仅构建并进入 CPU 容器，不等于完整真机系统已经具备运行条件；
- 视觉节点和项目内部包可以先编译、单独测试；
- 启动包含 FAST-LIO 的完整系统前，必须让容器内的 ROS2 环境能够找到 `livox_ros_driver2` 和 `fast_lio`；
- `scripts/setup_env.sh` 使用了宿主机绝对路径，不能直接在当前容器内 source。

推荐做法是将 Livox 和 FAST-LIO 源码放入一个外部工作空间，在容器内重新编译，并将该工作空间作为卷挂载到容器。不要直接复用带有宿主机绝对路径的 `install` 目录，除非宿主机和容器路径完全一致。

容器内的目标环境应类似：

```text
/ros2_external_ws/
├── src/
│   ├── livox_ros_driver2/
│   └── FAST-LIO/
└── install/
```

外部工作空间配置完成后，在容器内按顺序 source：

```bash
source /opt/ros/humble/setup.bash
source /ros2_external_ws/install/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash
```

检查包是否可见：

```bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
ros2 pkg prefix robot_localization
ros2 pkg prefix serial_driver
```

如果某个包提示找不到，先不要启动 `full_system.launch.py`，应先完成对应依赖的安装、编译或 Compose 卷挂载。

## 6. 构建 CPU Docker 镜像

### 6.1 使用项目脚本构建

在**宿主机终端**执行：

```bash
cd "$PROJECT_DIR/docker"
chmod +x run_docker_ros2.sh
./run_docker_ros2.sh build cpu
```

脚本实际构建 Compose 服务：

```text
yolov8-ros2-cpu
```

镜像标签为：

```text
yolov8-d435i:ros2-cpu-latest
```

### 6.2 直接使用 Docker Compose 构建

如果不使用脚本，也可以执行：

```bash
cd "$PROJECT_DIR/docker"
docker compose -f docker-compose.ros2.yml build yolov8-ros2-cpu
```

检查镜像：

```bash
docker image ls | grep yolov8-d435i
```

首次构建会下载 ROS2 基础镜像、Ubuntu 软件包和 Python 依赖，耗时取决于网络速度和磁盘性能。

### 6.3 CPU 构建不需要的旧步骤

AMD CPU 部署不要执行以下旧流程：

```bash
pip download torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126
pip download torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128
./run_docker_ros2.sh build gpu
```

GPU Dockerfile 还要求项目根目录存在 `docker/docker_deps/librealsense-2.56.4` 源码目录和本地 CUDA wheel；当前 CPU 部署不需要准备这些文件。

## 7. 启动和进入 CPU 容器

### 7.1 启动容器

在**宿主机终端**执行：

```bash
cd "$PROJECT_DIR/docker"
export DISPLAY="${DISPLAY:-:0}"
xhost +local:docker

./run_docker_ros2.sh start cpu
```

检查容器状态：

```bash
docker ps --filter name=yolov8-ros2-cpu
docker compose -f docker-compose.ros2.yml ps
```

### 7.2 进入容器

在**宿主机终端**执行：

```bash
cd "$PROJECT_DIR/docker"
./run_docker_ros2.sh exec cpu
```

也可以直接执行：

```bash
docker exec -it yolov8-ros2-cpu bash
```

### 7.3 容器内初始化 ROS2 环境

进入容器后执行：

```bash
cd /ros2_vision_project
source /opt/ros/humble/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

如果已经编译过真机工作空间：

```bash
source /ros2_vision_project/ros2_ws/install/setup.bash
```

如果已经编译过仿真工作空间：

```bash
source /ros2_vision_project/sim_ws/install/setup.bash
```

如果有外部工作空间，必须在项目工作空间之前 source：

```bash
[ -f /ros2_external_ws/install/setup.bash ] && \
  source /ros2_external_ws/install/setup.bash
```

## 8. 容器内基础检查

在**容器终端**执行：

```bash
cd /ros2_vision_project

python3 -c "import torch; print('torch:', torch.__version__); print('cuda:', torch.cuda.is_available())"
python3 -c "import cv2; print('opencv:', cv2.__version__)"
python3 -c "import ultralytics; print('ultralytics:', ultralytics.__version__)"

ros2 pkg prefix realsense2_camera
ros2 pkg prefix cv_bridge
ros2 pkg prefix rviz2
```

检查项目挂载和模型文件：

```bash
test -d /ros2_vision_project/ros2_ws/src
ls -lh /ros2_vision_project/ros2_ws/src/vision_detector/weights
ls -lh /ros2_vision_project/ros2_ws/src/rc2026_navigation/map
```

检查硬件是否进入容器：

```bash
lsusb
ls -l /dev/video* 2>/dev/null
ls -l /dev/ttyUSB* /dev/ttyACM* 2>/dev/null
```

如果宿主机可以看到设备、容器看不到，优先检查 Compose 中的 `/dev` 挂载和 `privileged: true`。

### 8.1 容器内补齐项目依赖

当前 CPU 镜像已经包含 ROS2 Humble、RealSense、OpenCV、PyTorch CPU 和大部分基础工具，但项目真机和仿真工作空间还需要补齐一批 ROS2 包和 Python 包。先在**宿主机终端**打开 root 容器终端：

```bash
docker exec -u root -it yolov8-ros2-cpu bash
```

然后在这个 **root 容器终端**执行：

```bash
apt update
apt install -y \
  ros-humble-robot-localization \
  ros-humble-nav2-bringup \
  ros-humble-nav2-msgs \
  ros-humble-nav2-amcl \
  ros-humble-nav2-map-server \
  ros-humble-nav2-lifecycle-manager \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-msgs \
  ros-humble-serial-driver \
  python3-open3d \
  python3-serial \
  python3-scipy \
  python3-pip

python3 -m pip install --upgrade ttkbootstrap
```

如果后续删除并重建容器，这些手动安装的包也会丢失；长期稳定方案是把这批依赖同步写入 `Dockerfile.ros2`。

建议再执行一次当前仓库的依赖解析：

```bash
source /opt/ros/humble/setup.bash
cd /ros2_vision_project
rosdep install --from-paths ros2_ws/src sim_ws/src --ignore-src -r -y
```

如果你已经把外部 `livox_ros_driver2` 和 `fast_lio` 放进了 `/ros2_external_ws/src`，再执行：

```bash
source /opt/ros/humble/setup.bash
rosdep install --from-paths /ros2_external_ws/src --ignore-src -r -y
```

这一步会把 `ros2_ws` 和 `sim_ws` 的系统依赖补齐，避免后面 `colcon build` 卡在缺包上。

## 9. 编译 `ros2_ws` 真机工作空间

### 9.1 编译项目内部包

在**容器终端**执行：

```bash
source /opt/ros/humble/setup.bash
cd /ros2_vision_project/ros2_ws

colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

编译完成后检查：

```bash
ros2 pkg prefix rc2026_bringup
ros2 pkg prefix vision_detector
ros2 pkg prefix decision_processor
ros2 pkg prefix rc2026_navigation
ros2 pkg prefix cmd_vel_bridge
ros2 pkg prefix auto_serial_bridge
```

### 9.2 关于 `scripts/build_robot.sh`

该脚本的作用是编译 `ros2_ws`，但当前脚本注释中仍保留了 Jetson 部署描述。Docker 容器内可以执行：

```bash
cd /ros2_vision_project
chmod +x scripts/build_robot.sh
scripts/build_robot.sh
source ros2_ws/install/setup.bash
```

如果需要使用外部 Livox/FAST-LIO 工作空间，仍需先 source 外部工作空间，再 source 项目 `ros2_ws`。

### 9.3 不要在容器内直接使用宿主机版 `setup_env.sh`

当前 `scripts/setup_env.sh` 中包含宿主机绝对路径，例如：

```text
/home/hgzq/ros2_external_ws/install
/home/hgzq/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws/install
```

这些路径在容器中不一定存在。容器内请手动使用下面的顺序：

```bash
source /opt/ros/humble/setup.bash
[ -f /ros2_external_ws/install/setup.bash ] && \
  source /ros2_external_ws/install/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp
```

## 10. CPU 视觉节点测试

完整真机 launch 默认仍声明 `device:=cuda`，虽然检测器代码会在 CUDA 不可用时回退 CPU，但手动启动时必须显式传入 `device:=cpu`。

### 10.1 RealSense 驱动检查

在**容器终端**执行：

```bash
rs-enumerate-devices
```

如果命令不存在或没有检测到设备，检查：

```bash
lsusb | grep -i intel
ls -l /dev/bus/usb
```

### 10.2 单独启动视觉检测

先 source 项目工作空间：

```bash
source /opt/ros/humble/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash
```

启动检测节点：

```bash
ros2 launch vision_detector detector.launch.py \
  model_path:=/ros2_vision_project/ros2_ws/src/vision_detector/weights/best.pt \
  device:=cpu
```

另开一个容器终端，进入同一个容器后执行：

```bash
source /opt/ros/humble/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash

ros2 topic list
ros2 topic echo /detections
ros2 run rqt_image_view rqt_image_view
```

如果没有 X11 socket 挂载，ROS2 节点仍可能运行，但 `rqt_image_view`、RViz2 和 OpenCV GUI 无法显示。

### 10.3 检查 CPU 推理

启动检测节点后观察日志中的模型设备信息。也可以执行：

```bash
python3 - <<'PY'
import torch
print("torch version:", torch.__version__)
print("cuda available:", torch.cuda.is_available())
print("selected device: cpu")
PY
```

不要用以下 CUDA 测试作为 AMD 小主机的验收标准：

```bash
torch.ones(100, 100).cuda()
```

在当前部署中该命令失败是正常的，因为项目实际使用 CPU 推理。

## 11. 真机全系统启动

### 11.1 模型路径兼容处理

当前 `rc2026_bringup/launch/full_system.launch.py` 和 `vision_detector/model_switcher.py` 仍按宿主机路径拼接模型目录：

```text
~/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws/src/vision_detector/weights
```

在当前 CPU 镜像中，容器用户通常是 `developer`，项目实际挂载在 `/ros2_vision_project`。如果直接运行全系统前出现模型文件不存在，可以在**容器终端**建立兼容软链接：

```bash
mkdir -p /home/developer/GAFA-Artlnnov.RC2026-main
ln -sfn /ros2_vision_project \
  /home/developer/GAFA-Artlnnov.RC2026-main/ros2_vision_project
```

确认模型路径：

```bash
test -f /home/developer/GAFA-Artlnnov.RC2026-main/ros2_vision_project/ros2_ws/src/vision_detector/weights/best.pt
```

### 11.2 推荐交互式启动

确认以下条件后再启动：

1. `ros2_ws` 已在容器内编译。
2. `livox_ros_driver2`、`fast_lio`、`robot_localization` 和 `serial_driver` 可被 `ros2 pkg prefix` 找到。
3. RealSense、Livox、USB 相机和串口设备已经进入容器。
4. 使用调试 GUI 时，X11 socket 已挂载并执行了 `xhost +local:docker`。

在**容器终端**执行：

```bash
cd /ros2_vision_project
source /opt/ros/humble/setup.bash
[ -f /ros2_external_ws/install/setup.bash ] && \
  source /ros2_external_ws/install/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
export RMW_IMPLEMENTATION=rmw_fastrtps_cpp

python3 scripts/launch_rc2026.py
```

脚本会交互询问：

- 比赛模式或调试模式；
- `full`、`chongwu`、`jiugong` 或局部测试区域；
- 左半场或右半场；
- 真/假 KFS 台阶编号；
- 是否启用 STM32 串口；
- 红外模块串口；
- 第二 KEY2 红外模块串口。

AMD CPU 部署不需要在交互脚本之外设置 CUDA 环境变量。当前 `launch_rc2026.py` 不会显式向 `full_system.launch.py` 追加 `device:=cpu`，因此底层 launch 会先使用默认 `device:=cuda`，随后 `YOLOv8Detector` 在检测到 CUDA 不可用时自动回退到 CPU。若希望启动日志更明确，可以手动启动时追加：

```text
device:=cpu
```

### 11.3 直接启动全系统

不使用交互脚本时，可以手动执行：

```bash
source /opt/ros/humble/setup.bash
[ -f /ros2_external_ws/install/setup.bash ] && \
  source /ros2_external_ws/install/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash

ros2 launch rc2026_bringup full_system.launch.py \
  field_side:=left \
  test_area:=full \
  kfs_real:='5' \
  kfs_fake:='2 11' \
  kfs_color:=blue \
  use_game_controller:=true \
  enable_serial:=true \
  enable_ir_r1_signal:=true \
  enable_ir_key2_signal:=true \
  match_timeout_s:=250.0 \
  device:=cpu
```

调试时建议先关闭串口和比赛状态机：

```bash
ros2 launch rc2026_bringup full_system.launch.py \
  field_side:=left \
  test_area:=weapon_align \
  use_game_controller:=false \
  enable_serial:=false \
  device:=cpu
```

注意：即使关闭串口，当前 `full_system.launch.py` 仍会启动定位链路，因此仍需要 `livox_ros_driver2` 和 `fast_lio`。如果只想验证相机和 YOLO，请使用第 10 节的 `vision_detector detector.launch.py`。

### 11.4 真机运行监控

在另一个容器终端中执行：

```bash
docker exec -it yolov8-ros2-cpu bash
source /opt/ros/humble/setup.bash
[ -f /ros2_external_ws/install/setup.bash ] && \
  source /ros2_external_ws/install/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash

ros2 topic list
ros2 node list
ros2 topic echo /game/phase
ros2 topic echo /waypoint_nav/status
ros2 topic echo /decision/state
ros2 topic echo /relocalize/status
ros2 run tf2_ros tf2_echo map base_link
```

## 12. 编译和运行 `sim_ws`

当前仿真工作空间实际目录为：

```text
/ros2_vision_project/sim_ws
```

当前 `sim_ws/src` 已包含：

- `rc2026_sim`：仿真机器人、Gazebo 启动、里程计漂移和重定位测试；
- `rc2026_field`：Robocon 2026 场地、KFS 模型、地图和场地 GUI。

### 12.1 编译仿真工作空间

在**容器终端**执行：

```bash
source /opt/ros/humble/setup.bash
cd /ros2_vision_project/ros2_ws
source install/setup.bash

cd /ros2_vision_project/sim_ws
colcon build --symlink-install \
  --cmake-args -DCMAKE_BUILD_TYPE=Release

source install/setup.bash
```

检查仿真包：

```bash
ros2 pkg prefix rc2026_sim
ros2 pkg prefix rc2026_field
ros2 pkg prefix gazebo_ros
```

当前仓库已经把 `rc2026_field` 放入 `sim_ws/src`，不需要再从外部包查找地图场地。

### 12.2 完整 Gazebo 场地仿真

```bash
source /opt/ros/humble/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash
source /ros2_vision_project/sim_ws/install/setup.bash

ros2 launch rc2026_sim simulation.launch.py
```

右半场：

```bash
ros2 launch rc2026_sim simulation.launch.py field_side:=right
```

启动仿真状态机：

```bash
ros2 launch rc2026_sim simulation.launch.py \
  field_side:=left \
  use_game_controller:=true
```

启用仿真视觉链路时，建议显式指定 CPU。当前 `simulation.launch.py` 的视觉节点默认参数仍写有 `device:=cuda`，在 AMD 主机上应确认代码回退到 CPU；如需要强制 CPU，优先修改仿真 launch 中视觉节点的 `device` 参数为 `cpu`，或单独启动视觉节点进行验证。

### 12.3 出生点误差和重定位测试

```bash
ros2 launch rc2026_sim simulation.launch.py \
  spawn_offset_x:=0.3 \
  spawn_offset_y:=-0.2 \
  spawn_offset_yaw:=0.05 \
  enable_reloc:=true \
  reloc_delay:=5.0
```

该模式用 Gazebo 真值模拟开局重定位修正，不等同于真机的 FAST-LIO + Open3D ICP。

### 12.4 里程计累积漂移测试

```bash
ros2 launch rc2026_sim simulation.launch.py \
  enable_odom_drift:=true \
  drift_linear:=0.05 \
  drift_angular:=0.05
```

### 12.5 空白平地测试

```bash
ros2 launch rc2026_sim test_flatground.launch.py
```

### 12.6 场地 GUI

`rc2026_field` 的 GUI 控制器依赖 `ttkbootstrap`。在容器内首次使用时执行：

```bash
python3 -m pip install ttkbootstrap
source /ros2_vision_project/ros2_ws/install/setup.bash
source /ros2_vision_project/sim_ws/install/setup.bash

ros2 launch rc2026_field rc2026_field_sim_with_controller.launch.py
```

场地 GUI 只负责场地和 KFS 实体管理，不仿真 R2 机械臂的真实物理动作。

### 12.7 关于 `scripts/build_sim.sh`

当前仓库的实际仿真目录是 `sim_ws`，但 `scripts/build_sim.sh` 中仍保留了 `simulation_ws` 路径。Docker 内编译时应按本节命令手动执行，避免脚本找不到：

```text
/ros2_vision_project/simulation_ws
```

## 13. 容器生命周期和日志

在**宿主机终端**执行：

```bash
cd "$PROJECT_DIR/docker"

# 查看状态
docker compose -f docker-compose.ros2.yml ps

# 查看 CPU 容器日志
docker compose -f docker-compose.ros2.yml logs -f yolov8-ros2-cpu

# 停止 CPU 容器
docker compose -f docker-compose.ros2.yml stop yolov8-ros2-cpu

# 停止并删除容器，不删除项目目录
docker compose -f docker-compose.ros2.yml down

# 重新启动
docker compose -f docker-compose.ros2.yml up -d yolov8-ros2-cpu
```

`run_docker_ros2.sh` 的部分旧命令仍调用独立的 `docker-compose` 命令。如果系统只安装了 Compose 插件而没有独立命令，`stop`、`logs` 或 `clean` 失败时，直接使用上面的 `docker compose` 写法。

不要随意执行以下清理命令：

```bash
docker system prune -a
docker compose down -v
```

这些命令可能删除其他项目的镜像、容器、卷或缓存。

## 14. 常见问题

### 14.1 `permission denied while trying to connect to the Docker daemon`

重新加载用户组：

```bash
newgrp docker
docker ps
```

仍然失败时注销并重新登录。

### 14.2 `docker-compose: command not found`

当前项目使用 Compose 插件，命令应写成：

```bash
docker compose version
docker compose -f docker-compose.ros2.yml ps
```

不要依赖旧的 `docker-compose` 独立命令。

### 14.3 容器内没有 `/dev/video0`

依次检查：

```bash
# 宿主机
ls -l /dev/video*

# 容器
ls -l /dev/video*
```

如果只有宿主机可见，在 CPU Compose 服务的 `volumes` 中补充：

```yaml
- /dev:/dev
```

同时确认相机没有被宿主机其他程序独占。

### 14.4 容器内没有 `/dev/ttyUSB0`

检查宿主机实际设备名：

```bash
ls -l /dev/ttyUSB* /dev/ttyACM* /dev/serial/by-id/*
```

检查容器是否挂载 `/dev`，并确认 launch 参数使用了正确串口。串口编号不是固定不变的，不能仅凭历史记录假设一定是 `/dev/ttyUSB0`。

### 14.5 `PackageNotFoundError: livox_ros_driver2` 或 `fast_lio`

这是当前 Docker 配置的外部依赖缺失，不是 YOLO CPU 问题。先检查：

```bash
ros2 pkg prefix livox_ros_driver2
ros2 pkg prefix fast_lio
```

必须将外部源码/工作空间挂载到容器，并在 source 项目 `ros2_ws` 之前 source 外部工作空间：

```bash
source /opt/ros/humble/setup.bash
source /ros2_external_ws/install/setup.bash
source /ros2_vision_project/ros2_ws/install/setup.bash
```

### 14.6 模型文件不存在

检查：

```bash
ls -lh /ros2_vision_project/ros2_ws/src/vision_detector/weights
```

单独视觉节点建议使用绝对路径：

```bash
model_path:=/ros2_vision_project/ros2_ws/src/vision_detector/weights/best.pt
```

全系统 launch 如果仍按宿主机路径查找模型，执行第 11.1 节的兼容软链接命令。

### 14.7 出现 CUDA 不可用警告

AMD 8845HS 实际使用 CPU，以下日志属于预期现象：

```text
CUDA requested but not available, falling back to CPU
```

启动命令中显式增加：

```text
device:=cpu
```

不要为当前小主机安装 NVIDIA Container Toolkit，也不要切换到 GPU Compose 服务。

### 14.8 RViz、rqt 或 OpenCV 窗口无法显示

检查宿主机：

```bash
echo "$DISPLAY"
xhost +local:docker
```

检查 CPU Compose 服务是否挂载：

```yaml
- /tmp/.X11-unix:/tmp/.X11-unix
```

然后重新创建容器：

```bash
docker compose -f docker-compose.ros2.yml up -d --force-recreate yolov8-ros2-cpu
```

### 14.9 Livox 能否在 Docker 中使用

当前 CPU Compose 使用：

```yaml
network_mode: host
```

因此容器和宿主机共享网络命名空间，Livox UDP 通信可以按宿主机网络配置工作。但仍需要：

1. 宿主机网卡与 Mid-360S 的 IP 配置正确；
2. 容器内能找到 `livox_ros_driver2`；
3. `MID360s_config.json` 中的设备地址与现场一致；
4. 防火墙没有阻断 Livox 通信。

## 15. NVIDIA GPU 和 Jetson 说明

以下配置仅作为仓库中保留的备用方案，不是当前 AMD 8845HS 真机运行方案。

### 15.1 NVIDIA x86_64

需要同时满足：

- 宿主机安装匹配版本的 NVIDIA 驱动；
- 安装 NVIDIA Container Toolkit；
- 准备 `docker/docker_deps/librealsense-2.56.4` 源码目录；
- 准备 `docker/docker_deps/whl/40` 或 `docker/docker_deps/whl/50` 中的 CUDA PyTorch wheel；
- 使用 `yolov8-ros2-gpu` 服务；
- 使用 `Dockerfile.ros2.gpu`。

该路径不应在 AMD CPU 小主机上执行。

### 15.2 Jetson Orin Nano

Jetson 路径使用 `Dockerfile.ros2.jetson` 或 `Dockerfile.ros2-1.jetson`，依赖 JetPack/L4T 环境，只能在 Jetson 设备上构建和运行。它与 AMD 8845HS 的 CPU 镜像不是同一套运行环境。

## 16. 推荐验收顺序

建议按以下顺序验收，不要一开始就启动完整比赛系统：

1. 宿主机 `docker run --rm hello-world` 成功。
2. `docker compose version` 成功。
3. CPU 镜像构建成功。
4. 容器内能看到 `/ros2_vision_project`。
5. 容器内 `torch.cuda.is_available()` 为 `False`，且 Python 可导入 `cv2`、`ultralytics`。
6. 容器内能看到 RealSense、USB 相机和串口设备。
7. `ros2_ws` 编译成功。
8. 单独启动 `vision_detector`，确认 CPU YOLO 和 RealSense 话题。
9. 配置并 source 外部 Livox/FAST-LIO 工作空间。
10. 最后启动 `full_system.launch.py` 或 `scripts/launch_rc2026.py`。
11. 单独编译并启动 `sim_ws`，验证 Gazebo、场地包、重定位和里程计漂移测试。

当前 Docker 主线可以概括为：

```text
Ubuntu 22.04 宿主机
  -> Docker Engine + Compose plugin
  -> Dockerfile.ros2
  -> yolov8-ros2-cpu
  -> /ros2_vision_project
  -> ros2_ws 真机包 + sim_ws 仿真包
  -> AMD CPU YOLO 推理
```
