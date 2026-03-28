#!/usr/bin/env python3
"""
full_system.launch.py
一键启动完整系统：RealSense相机 + YOLOv8检测 + 决策处理

切换模型示例：
  ros2 launch vision_detector full_system.launch.py
  ros2 launch vision_detector full_system.launch.py model:=best.pt
  ros2 launch vision_detector full_system.launch.py model:=armor.pt
  ros2 launch vision_detector full_system.launch.py model:=/绝对路径/custom.pt
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


# 模型文件夹路径（所有模型统一放这里）
WEIGHTS_DIR = os.path.join(
    os.path.expanduser('~'),
    'GAFA-Artlnnov.RC2026-main',
    'ros2_vision_project',
    'ros2_ws',
    'src',
    'vision_detector',
    'weights'
)


def generate_launch_description():

    # ── 模型参数：支持文件名或绝对路径 ──
    model_arg = DeclareLaunchArgument(
        'model',
        default_value='best.pt',        # 只写文件名，自动在 weights/ 下查找
        description=(
            '模型文件名（放在 weights/ 目录下）或绝对路径\n'
            '  文件名示例: best.pt / armor.pt / punch.pt\n'
            '  绝对路径示例: /home/hgzq/custom.pt'
        )
    )
    model_name = LaunchConfiguration('model')

    # ── 其他可调参数 ──
    conf_arg = DeclareLaunchArgument(
        'conf', default_value='0.5',
        description='置信度阈值 (0.0~1.0)')

    device_arg = DeclareLaunchArgument(
        'device', default_value='cuda',
        description='推理设备 (cuda / cpu)')

    conf   = LaunchConfiguration('conf')
    device = LaunchConfiguration('device')

    # ── 节点1：RealSense 相机 ──
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py'
            )
        ]),
        launch_arguments={
            'enable_color':               'true',
            'enable_depth':               'true',
            'enable_infra1':              'false',
            'enable_infra2':              'false',
            'align_depth.enable':         'true',
            'rgb_camera.color_profile':   '1280x720x30',
            'depth_module.depth_profile': '848x480x30',
        }.items()
    )

    # ── 节点2：YOLOv8 视觉检测（延迟3秒）──
    detector_node = TimerAction(
        period=3.0,
        actions=[
            Node(
                package='vision_detector',
                executable='detector_node',
                name='vision_detector',
                output='screen',
                parameters=[{
                    # 支持两种格式：
                    # 1. 只写文件名 → 自动拼接 weights/ 目录
                    # 2. 绝对路径   → 直接使用
                    'model_path': PathJoinSubstitution([
                        WEIGHTS_DIR, model_name
                    ]),
                    'conf_threshold':        conf,
                    'iou_threshold':         0.45,
                    'device':                device,
                    'publish_visualization': True,
                    'depth_sample_points':   24,
                    'camera_topic':          '/camera/camera/color/image_raw',
                    'depth_topic':           '/camera/camera/aligned_depth_to_color/image_raw',
                    'camera_info_topic':     '/camera/camera/color/camera_info',
                }],
                remappings=[
                    ('/camera/color/image_raw',
                     '/camera/camera/color/image_raw'),
                    ('/camera/depth/image_rect_raw',
                     '/camera/camera/aligned_depth_to_color/image_raw'),
                    ('/camera/color/camera_info',
                     '/camera/camera/color/camera_info'),
                ]
            )
        ]
    )

    # ── 节点3：决策处理（延迟5秒）──
    decision_node = TimerAction(
        period=5.0,
        actions=[
            Node(
                package='decision_processor',
                executable='processor_node',
                name='decision_processor_node',
                output='screen',
                parameters=[{
                    'wheel_diameter_m':    0.096,
                    'track_width_m':       0.28,
                    'stop_distance_m':     0.20,
                    'align_threshold_deg': 5.0,
                    'pick_duration_s':     3.0,
                    'conf_threshold':      conf,
                }]
            )
        ]
    )

    return LaunchDescription([
        model_arg,
        conf_arg,
        device_arg,
        realsense_launch,
        detector_node,
        decision_node,
    ])
