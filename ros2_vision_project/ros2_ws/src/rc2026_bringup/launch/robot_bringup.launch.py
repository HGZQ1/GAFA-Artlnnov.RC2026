#!/usr/bin/env python3
"""
robot_bringup.launch.py
完整系统启动：URDF + RealSense + YOLOv8 + 决策 + IMU + 里程计 + 串口桥
替代原 full_system.launch.py，统一管理所有节点启动顺序
"""
import os
import yaml
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, GroupAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PathJoinSubstitution, Command,
)
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory


WEIGHTS_DIR = os.path.join(
    os.path.expanduser('~'),
    'GAFA-Artlnnov.RC2026-main', 'ros2_vision_project',
    'ros2_ws', 'src', 'vision_detector', 'weights',
)


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc2026_bringup')
    params_file = os.path.join(bringup_dir, 'config', 'robot_params.yaml')
    urdf_file   = os.path.join(bringup_dir, 'urdf', 'rc2026_robot.urdf.xacro')

    # ── 启动参数 ──
    model_arg = DeclareLaunchArgument(
        'model', default_value='best.pt',
        description='YOLO模型文件名或绝对路径')
    conf_arg = DeclareLaunchArgument(
        'conf', default_value='0.5',
        description='置信度阈值')
    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda',
        description='推理设备 (cuda/cpu)')
    enable_nav_arg = DeclareLaunchArgument(
        'enable_nav', default_value='false',
        description='是否启动导航栈')
    enable_serial_arg = DeclareLaunchArgument(
        'enable_serial', default_value='true',
        description='是否启动串口桥')

    model_name = LaunchConfiguration('model')
    conf       = LaunchConfiguration('conf')
    device     = LaunchConfiguration('device')

    # ── 从 robot_params.yaml 读取所有硬件安装参数 ──
    with open(params_file, 'r') as f:
        hw = yaml.safe_load(f)

    cam     = hw['camera']
    lid     = hw['lidar']
    w_arm   = hw['weapon_arm']
    kfs     = hw['kfs_arm']
    dep     = kfs['deployed']
    usb_cam = kfs['usb_camera']
    suc     = kfs['suction']

    def _a(k, v):
        return f' {k}:={v}'

    xacro_args = (
        # D435i 相机
        _a('camera_x',      cam['offset_x'])  +
        _a('camera_y',      cam['offset_y'])  +
        _a('camera_z',      cam['offset_z'])  +
        _a('camera_roll',   cam['roll'])      +
        _a('camera_pitch',  cam['pitch'])     +
        _a('camera_yaw',    cam['yaw'])       +
        # Livox LiDAR
        _a('lidar_x',       lid['offset_x'])  +
        _a('lidar_y',       lid['offset_y'])  +
        _a('lidar_z',       lid['offset_z'])  +
        _a('lidar_roll',    lid['roll'])      +
        _a('lidar_pitch',   lid['pitch'])     +
        _a('lidar_yaw',     lid['yaw'])       +
        # 武器头机械臂
        _a('weapon_arm_x',      w_arm['offset_x'])  +
        _a('weapon_arm_y',      w_arm['offset_y'])  +
        _a('weapon_arm_z',      w_arm['offset_z'])  +
        _a('weapon_arm_roll',   w_arm['roll'])      +
        _a('weapon_arm_pitch',  w_arm['pitch'])     +
        _a('weapon_arm_yaw',    w_arm['yaw'])       +
        # KFS机械臂底座
        _a('kfs_arm_x',     kfs['offset_x'])  +
        _a('kfs_arm_y',     kfs['offset_y'])  +
        _a('kfs_arm_z',     kfs['offset_z'])  +
        _a('kfs_arm_roll',  kfs['roll'])      +
        _a('kfs_arm_pitch', kfs['pitch'])     +
        _a('kfs_arm_yaw',   kfs['yaw'])       +
        # KFS机械臂展开末端
        _a('kfs_deployed_x', dep['offset_x'])  +
        _a('kfs_deployed_y', dep['offset_y'])  +
        _a('kfs_deployed_z', dep['offset_z'])  +
        # USB相机
        _a('usb_cam_x',     usb_cam['offset_x'])  +
        _a('usb_cam_y',     usb_cam['offset_y'])  +
        _a('usb_cam_z',     usb_cam['offset_z'])  +
        _a('usb_cam_pitch', usb_cam['pitch'])     +
        # 吸盘
        _a('suction_x', suc['offset_x'])  +
        _a('suction_y', suc['offset_y'])  +
        _a('suction_z', suc['offset_z'])
    )

    # ── URDF: robot_state_publisher ──
    robot_description = Command([f'xacro {urdf_file}{xacro_args}'])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    # ── RealSense 相机 ──
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color':              'true',
            'enable_depth':              'true',
            'enable_infra1':             'false',
            'enable_infra2':             'false',
            'align_depth.enable':        'true',
            'rgb_camera.color_profile':  '1280x720x30',
            'depth_module.depth_profile':'848x480x30',
            'enable_gyro':               'true',
            'enable_accel':              'true',
            'gyro_fps':                  '200',
            'accel_fps':                 '100',
            'unite_imu_method':          '2',
        }.items(),
    )

    # ── YOLOv8 检测 (延迟3秒) ──
    detector_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='vision_detector',
            executable='detector_node',
            name='vision_detector',
            output='screen',
            parameters=[{
                'model_path': PathJoinSubstitution([WEIGHTS_DIR, model_name]),
                'conf_threshold':        conf,
                'iou_threshold':         0.45,
                'device':                device,
                'publish_visualization': True,
                'depth_sample_points':   24,
                'camera_topic':    '/camera/camera/color/image_raw',
                'depth_topic':     '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
            }],
            remappings=[
                ('/camera/color/image_raw',
                 '/camera/camera/color/image_raw'),
                ('/camera/depth/image_rect_raw',
                 '/camera/camera/aligned_depth_to_color/image_raw'),
                ('/camera/color/camera_info',
                 '/camera/camera/color/camera_info'),
            ],
        )],
    )

    # ── IMU 处理 (延迟4秒) ──
    imu_node = TimerAction(
        period=4.0,
        actions=[Node(
            package='decision_processor',
            executable='imu_processor',
            name='imu_processor',
            output='screen',
        )],
    )

    # ── 里程计融合 (延迟4.5秒) ──
    odom_node = TimerAction(
        period=4.5,
        actions=[Node(
            package='decision_processor',
            executable='odometry_fusion',
            name='odometry_fusion',
            output='screen',
        )],
    )

    # ── 梅林导航 (延迟5秒) ──
    meilin_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='decision_processor',
            executable='meilin_navigator',
            name='meilin_navigator',
            output='screen',
        )],
    )

    # ── 决策处理 (延迟5秒) ──
    decision_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='decision_processor',
            executable='processor_node',
            name='decision_processor_node',
            output='screen',
            parameters=[params_file],
        )],
    )

    # ── 串口桥 (条件启动) ──
    serial_node = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_serial')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('auto_serial_bridge'),
                        'launch', 'serial_bridge_by_node.launch.py')
                ),
            ),
        ],
    )

    # ── 导航栈 (条件启动: FAST-LIO + EKF + Nav2 + cmd_vel_bridge) ──
    nav_stack = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_nav')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('rc2026_navigation'),
                        'launch', 'fastlio.launch.py')
                ),
            ),
            TimerAction(
                period=3.0,
                actions=[
                    IncludeLaunchDescription(
                        PythonLaunchDescriptionSource(
                            os.path.join(
                                get_package_share_directory('rc2026_navigation'),
                                'launch', 'nav2.launch.py')
                        ),
                    ),
                ],
            ),
            Node(
                package='cmd_vel_bridge',
                executable='bridge_node',
                name='cmd_vel_bridge',
                output='screen',
            ),
        ],
    )

    return LaunchDescription([
        model_arg, conf_arg, device_arg,
        enable_nav_arg, enable_serial_arg,
        robot_state_pub,
        joint_state_pub,
        realsense_launch,
        detector_node,
        imu_node,
        odom_node,
        meilin_node,
        decision_node,
        serial_node,
        nav_stack,
    ])
