#!/usr/bin/env python3
"""
fastlio.launch.py
一键启动: Livox Mid-360S 驱动 + FAST-LIO2 + EKF融合 + 轮式里程计

等价于手动分别执行:
  ros2 launch livox_ros_driver2 msg_MID360s_launch.py
  ros2 launch fast_lio mapping.launch.py config_file:=mid360.yaml
  + EKF + wheel_odom
"""
import os
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory('rc2026_navigation')
    livox_dir = get_package_share_directory('livox_ros_driver2')
    fastlio_dir = get_package_share_directory('fast_lio')

    # ── Livox Mid-360S 驱动 (和 msg_MID360s_launch.py 完全一致) ──
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_dir, 'launch_ROS2', 'msg_MID360s_launch.py')),
    )

    # ── FAST-LIO2 (和 mapping.launch.py config_file:=mid360.yaml 一致) ──
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_dir, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': 'false',
        }.items(),
    )

    # ── 轮式里程计发布 ──
    wheel_odom_node = Node(
        package='cmd_vel_bridge',
        executable='wheel_odom_publisher',
        name='wheel_odom_publisher',
        output='screen',
        parameters=[{
            'odom_frame': 'odom',
            'base_frame': 'base_link',
            'publish_tf': False,
        }],
    )

    # ── EKF 融合定位 ──
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[
            os.path.join(nav_dir, 'config', 'ekf.yaml'),
        ],
        remappings=[
            ('/odometry/filtered', '/odom'),
        ],
    )

    return LaunchDescription([
        livox_launch,
        fastlio_launch,
        wheel_odom_node,
        ekf_node,
    ])
