# 注意带 # 号的是注释，不用输入，但是指令前后的注释也不需要刻意删除
## docker 配置  
```bash

# 转到docker的配置目录（文件夹） 
# 先打开到 ros2_vision_project 这个，项目的总文件夹
# 在 ros2_vision_project 文件夹右键，选择“在终端中打开”，届时会启动一个黑色页面


# 然后在此界面输入 （一行一行输入）
cd /docker  # 定位到docker配置文件夹

# 下载需要预先下载的安装包
mkdir -p docker_deps/whl/50
mkdir -p docker_deps/whl/40

# 下载相机驱动 下载完后解压
cd docker_deps
wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.56.2.tar.gz -O librealsense-2.56.2.tar.gz
wget https://github.com/IntelRealSense/librealsense/archive/refs/tags/v2.56.4.tar.gz -O librealsense-2.56.4.tar.gz

# 下载pytorch 
pip download torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu126 -d ./whl/40  # 40/30 系列显卡执行

pip download torch torchvision torchaudio --index-url https://download.pytorch.org/whl/cu128 -d ./whl/50  # 50 系列显卡执行

cd ..  # 退回 docker 文件夹
chmod +x run_docker_ros2.sh  # 给脚本执行权限

# 构建镜像 （二选一）
./run_docker_ros2.sh build gpu  # 40/30系列显卡执行这个

GPU_SERIES=50 ./run_docker_ros2.sh build gpu  # 50系列显卡执行这个

```
##

## 启动docker （需要完整阅读）

由于我已经在 ros2_vision_project 文件夹下配置了 .devcontainer 文件夹及其内部的自动指令
所以 vsc 可以自动识别打开容器，但是需要先下载 Dev Containers 插件，
然后我们只需要 打开vsc 在导航栏选择 文件 > 新建窗口（可选，保证是新窗口即可） > 打开文件夹，> 选择 ros2_vision_projec 文件夹，vsc就会自动识别，你需要在 页面右下角的提示处 选择 在容器中打开 页面自动刷新后就进入容器了， 可以简单认为现在是进入了一个虚拟机的系统里，而ros2_vision_projec 文件夹是通过 挂载 从你的宿主电脑文件系统挂进来的，所以你可以在vsc的容器中编辑这个文件夹的文件，运行指令，同时也可以直接在你的电脑上，点击图标进行编辑。

##

## 基础配置 
```bash
# 在宿主机终端输入，允许容器显示画面到主机（注意，这是危险指令，涉及到高级权限，不可随便更改）
xhost +local:docker

# 在 容器内部，及vsc的终端中 和宿主机的终端中分别输入，确保数字一致
echo $DISPLAY

# 不一致时，在容器终端输入，“x”为宿主机输出结果
export DISPLAY=:x
export LIBGL_ALWAYS_SOFTWARE=1  # 可选

# 测试代码路径配置
export PYTHONPATH=$PYTHONPATH:/ros2_vision_project/ros2_ws/src/vision_detector

# 构建ros2
cd ~/ros2_ws  # 总之在此文件夹执行以下命令
colcon build --symlink-install

# Source工作空间
source install/setup.bash
```
##

## 基础测试
```bash

# 测显卡 正常则输出 tensor(...) 
python3 -c "import torch; print(f'GPU型号: {torch.cuda.get_device_name(0)}'); print(f'计算测试: {torch.ones(100, 100).cuda() @ torch.ones(100, 100).cuda()}')"

# 测yolo
# 1.确定文件
# 打开 /ros2_vision_project/ros2_ws/test_detector.py文件
# 确保第 1o 行为存在的图片，建议直接改为
test_image = cv2.imread('/ros2_vision_project/test2.jpg')

# 2,运行测试，容器内控制台直接输入
usr/bin/python3 /ros2_vision_project/ros2_ws/test_detector.py

# 测USB
# 在宿主机终端 (作用：将当前用户加入视频和热插拔设备组。执行完通常需要重启电脑生效。
# 这是确保 RealSense 相机能被系统读取的基础。)
sudo usermod -aG video $USER
sudo usermod -aG plugdev $USER

# 宿主机终端检查 USB 总线层面的连接
lsusb | grep -i Intel  # 没输出，说明线没插好或接口有问题

# 检查相机是否被识别 宿主机 / 容器内 均执行
ls /dev/video*  # 有输出 /dev/video0, video1... 说明硬件连通了

# 终极666魔鬼权限（慎用）（一次性，关机失效）
# 宿主机终端（插上相机之后）执行以下两条指令
sudo chmod 666 /dev/video*  # 给所有视频设备（/dev/video）开放读写权限
sudo chmod -R 666 /dev/bus/usb/  # 给整个 USB 总线开放读写权限（针对 RealSense 特别重要）


# 测相机 （三个终端）
# 1. 打开容器终端1 - 启动系统
cd ~/ros2_ws
source install/setup.bash
ros2 launch vision_detector detector.launch.py
ros2 run rqt_image_view rqt_image_view

# 2. 打开终端2 - 查看结果
ros2 topic echo /detections

# 3. 打开终端3 - 可视化（可选）
ros2 run rqt_image_view rqt_image_view

```
##