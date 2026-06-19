#!/usr/bin/env python3
"""
带RViz可视化的启动文件
"""

from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    # 包含主检测启动文件
    detector_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare('vision_detector'),
                'launch',
                'detector.launch.py'
            ])
        ])
    )
    
    # RViz节点
    rviz_config = PathJoinSubstitution([
        FindPackageShare('vision_detector'),
        'rviz',
        'detector_view.rviz'
    ])
    
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        output='screen'
    )
    
    return LaunchDescription([
        detector_launch,
        rviz_node,
    ])