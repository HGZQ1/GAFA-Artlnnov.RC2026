#!/bin/bash

# YOLOv8-D435i + ROS2 Docker启动脚本

set -e

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
BLUE='\033[0;34m'
NC='\033[0m'

print_info() {
    echo -e "${BLUE}[INFO]${NC} $1"
}

print_success() {
    echo -e "${GREEN}[SUCCESS]${NC} $1"
}

print_warning() {
    echo -e "${YELLOW}[WARNING]${NC} $1"
}

print_error() {
    echo -e "${RED}[ERROR]${NC} $1"
}

# 检查Docker
check_docker() {
    if ! command -v docker &> /dev/null; then
        print_error "Docker未安装"
        exit 1
    fi
    print_success "Docker已安装"
}

# 检查Docker Compose
check_docker_compose() {
    if ! command -v docker-compose &> /dev/null && ! docker compose version &> /dev/null; then
        print_error "Docker Compose未安装"
        exit 1
    fi
    print_success "Docker Compose已安装"
}

# 检查架构
check_architecture() {
    ARCH=$(uname -m)
    print_info "系统架构: $ARCH"
    
    if [ -f /etc/nv_tegra_release ]; then
        IS_JETSON=true
        print_info "检测到Jetson设备"
    else
        IS_JETSON=false
    fi
}

# 允许X11转发
setup_x11() {
    print_info "配置X11显示..."
    xhost +local:docker &> /dev/null || true
    export DISPLAY=${DISPLAY:-:0}
    print_success "X11显示已配置: $DISPLAY"
}

# 构建镜像
build_image() {
    local mode=$1
    
    print_info "构建ROS2镜像: $mode"
    
    case "$mode" in
        cpu)
            docker-compose -f docker-compose.ros2.yml build yolov8-ros2-cpu
            ;;
        gpu)
            docker-compose -f docker-compose.ros2.yml build yolov8-ros2-gpu
            ;;
        jetson)
            if [ "$IS_JETSON" != true ]; then
                print_error "Jetson镜像仅能在Jetson设备上构建"
                exit 1
            fi
            docker-compose -f docker-compose.ros2.yml build yolov8-ros2-jetson
            ;;
        all)
            if [ "$IS_JETSON" = true ]; then
                docker-compose -f docker-compose.ros2.yml build yolov8-ros2-jetson
            else
                docker-compose -f docker-compose.ros2.yml build
            fi
            ;;
        *)
            print_error "未知模式: $mode"
            exit 1
            ;;
    esac
    
    print_success "镜像构建完成"
}

# 启动容器
start_container() {
    local mode=$1
    
    setup_x11
    
    print_info "启动ROS2容器: $mode"
    
    case "$mode" in
        cpu)
            docker-compose -f docker-compose.ros2.yml up -d yolov8-ros2-cpu
            CONTAINER_NAME="yolov8-ros2-cpu"
            ;;
        gpu)
            docker-compose -f docker-compose.ros2.yml up -d yolov8-ros2-gpu
            CONTAINER_NAME="yolov8-ros2-gpu"
            ;;
        jetson)
            docker-compose -f docker-compose.ros2.yml up -d yolov8-ros2-jetson
            CONTAINER_NAME="yolov8-ros2-jetson"
            ;;
        *)
            print_error "未知模式: $mode"
            exit 1
            ;;
    esac
    
    print_success "容器已启动: $CONTAINER_NAME"
    print_info "进入容器: $0 exec $mode"
}

# 停止容器
stop_container() {
    local mode=$1
    
    if [ "$mode" == "all" ]; then
        docker-compose -f docker-compose.ros2.yml stop
    else
        docker-compose -f docker-compose.ros2.yml stop yolov8-ros2-$mode
    fi
    
    print_success "容器已停止"
}

# 进入容器
exec_container() {
    local mode=$1
    
    print_info "进入ROS2容器: $mode"
    docker exec -it yolov8-ros2-$mode bash
}

# 运行ROS2节点
run_ros2_node() {
    local mode=$1
    
    print_info "启动ROS2检测节点..."
    docker exec -it yolov8-ros2-$mode bash -c "cd ~/ros2_ws && source install/setup.bash && ros2 run vision_detector detector_node"
}

# 启动ROS2 launch文件
launch_ros2() {
    local mode=$1
    local launch_file=${2:-detector.launch.py}
    
    print_info "启动ROS2 launch文件: $launch_file"
    docker exec -it yolov8-ros2-$mode bash -c "cd ~/ros2_ws && source install/setup.bash && ros2 launch vision_detector $launch_file"
}

# 构建ROS2工作空间
build_ros2_ws() {
    local mode=$1
    
    print_info "构建ROS2工作空间..."
    docker exec -it yolov8-ros2-$mode bash -c "cd ~/ros2_ws && colcon build --symlink-install"
    print_success "ROS2工作空间构建完成"
}

# 查看ROS2话题
list_topics() {
    local mode=$1
    
    print_info "ROS2话题列表:"
    docker exec -it yolov8-ros2-$mode bash -c "source /opt/ros/humble/setup.bash && ros2 topic list"
}

# 查看ROS2节点
list_nodes() {
    local mode=$1
    
    print_info "ROS2节点列表:"
    docker exec -it yolov8-ros2-$mode bash -c "source /opt/ros/humble/setup.bash && ros2 node list"
}

# 启动RViz2
launch_rviz() {
    local mode=$1
    
    setup_x11
    print_info "启动RViz2..."
    docker exec -it yolov8-ros2-$mode bash -c "source /opt/ros/humble/setup.bash && rviz2"
}

# 查看日志
show_logs() {
    local mode=$1
    
    if [ -z "$mode" ] || [ "$mode" == "all" ]; then
        docker-compose -f docker-compose.ros2.yml logs -f
    else
        docker-compose -f docker-compose.ros2.yml logs -f yolov8-ros2-$mode
    fi
}

# 清理
clean() {
    print_warning "这将删除所有ROS2容器和镜像"
    read -p "确定要继续吗? (y/N): " -n 1 -r
    echo
    
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        print_info "清理容器..."
        docker-compose -f docker-compose.ros2.yml down -v
        
        print_info "删除镜像..."
        docker rmi yolov8-d435i:ros2-cpu-latest yolov8-d435i:ros2-gpu-latest yolov8-d435i:ros2-jetson-latest 2>/dev/null || true
        
        print_success "清理完成"
    fi
}

# 显示帮助
show_help() {
    cat << EOF
YOLOv8-D435i + ROS2 Docker管理脚本

用法: $0 <命令> <模式> [选项]

模式:
    cpu                CPU版本
    gpu                GPU版本（需要NVIDIA GPU）
    jetson             Jetson版本（仅Jetson设备）

命令:
    build <mode>           构建镜像
    start <mode>           启动容器
    stop <mode|all>        停止容器
    exec <mode>            进入容器
    logs [mode|all]        查看日志
    
    # ROS2专用命令
    build-ws <mode>        构建ROS2工作空间
    run-node <mode>        运行检测节点
    launch <mode> [file]   启动launch文件
    topics <mode>          列出ROS2话题
    nodes <mode>           列出ROS2节点
    rviz <mode>            启动RViz2
    
    clean                  清理容器和镜像
    help                   显示此帮助

示例:
    # 基础使用
    $0 build gpu           # 构建GPU版本镜像
    $0 start gpu           # 启动GPU容器
    $0 exec gpu            # 进入容器
    
    # ROS2开发
    $0 build-ws gpu        # 构建ROS2工作空间
    $0 run-node gpu        # 运行检测节点
    $0 launch gpu          # 启动launch文件
    $0 topics gpu          # 查看话题列表
    $0 rviz gpu            # 启动RViz2
    
    # Jetson
    $0 build jetson        # 构建Jetson镜像
    $0 start jetson        # 启动Jetson容器

注意:
    - GPU模式需要安装NVIDIA Container Toolkit
    - Jetson模式需要在Jetson设备上运行
    - ROS2工作空间位于 ./ros2_ws
    - 首次运行需要先构建镜像
EOF
}

# 主函数
main() {
    check_docker
    check_docker_compose
    check_architecture
    
    case "$1" in
        build)
            build_image ${2:-cpu}
            ;;
        start)
            if [ -z "$2" ]; then
                print_error "请指定模式: cpu, gpu, 或 jetson"
                exit 1
            fi
            start_container $2
            ;;
        stop)
            stop_container ${2:-all}
            ;;
        exec)
            if [ -z "$2" ]; then
                print_error "请指定模式: cpu, gpu, 或 jetson"
                exit 1
            fi
            exec_container $2
            ;;
        build-ws)
            if [ -z "$2" ]; then
                print_error "请指定模式"
                exit 1
            fi
            build_ros2_ws $2
            ;;
        run-node)
            if [ -z "$2" ]; then
                print_error "请指定模式"
                exit 1
            fi
            run_ros2_node $2
            ;;
        launch)
            if [ -z "$2" ]; then
                print_error "请指定模式"
                exit 1
            fi
            launch_ros2 $2 ${3:-detector.launch.py}
            ;;
        topics)
            if [ -z "$2" ]; then
                print_error "请指定模式"
                exit 1
            fi
            list_topics $2
            ;;
        nodes)
            if [ -z "$2" ]; then
                print_error "请指定模式"
                exit 1
            fi
            list_nodes $2
            ;;
        rviz)
            if [ -z "$2" ]; then
                print_error "请指定模式"
                exit 1
            fi
            launch_rviz $2
            ;;
        logs)
            show_logs ${2:-all}
            ;;
        clean)
            clean
            ;;
        help|--help|-h)
            show_help
            ;;
        *)
            print_error "未知命令: $1"
            echo ""
            show_help
            exit 1
            ;;
    esac
}

main "$@"
