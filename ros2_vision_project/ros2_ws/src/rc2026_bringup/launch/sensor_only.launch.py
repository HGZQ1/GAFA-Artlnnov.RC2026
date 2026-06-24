#!/usr/bin/env python3
"""
sensor_only.launch.py
仅启动传感器节点用于调试：URDF + RealSense + 激光雷达
不启动决策/导航/串口
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import Command
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.parameter_descriptions import ParameterValue

def generate_launch_description():
    bringup_dir = get_package_share_directory('rc2026_bringup')
    urdf_file   = os.path.join(bringup_dir, 'urdf', 'rc2026_robot.urdf.xacro')

    robot_description = Command(['xacro ', urdf_file])

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': ParameterValue(robot_description, value_type=str)}],
        output='screen',
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        output='screen',
    )

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')
        ),
        launch_arguments={
            'enable_color':              'true',
            'enable_depth':              'true',
            'align_depth.enable':        'true',
            'rgb_camera.color_profile':  '1280x720x30',
            'depth_module.depth_profile':'848x480x30',
            'enable_gyro':               'true',
            'enable_accel':              'true',
            'unite_imu_method':          '2',
        }.items(),
    )

    rviz_config = os.path.join(bringup_dir, 'rviz', 'navigation.rviz')

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        arguments=['-d', rviz_config],
        output='screen',
    )

    return LaunchDescription([
        robot_state_pub,
        joint_state_pub,
        realsense_launch,
        rviz_node,
    ])
