#!/usr/bin/env python3
"""
cmd_vel_bridge.launch.py
启动 cmd_vel → 串口协议桥接节点
"""
import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    pkg_dir = get_package_share_directory('cmd_vel_bridge')
    params_file = os.path.join(pkg_dir, 'config', 'pid_params.yaml')

    bridge_node = Node(
        package='cmd_vel_bridge',
        executable='bridge_node',
        name='cmd_vel_bridge',
        output='screen',
        parameters=[params_file],
    )

    return LaunchDescription([bridge_node])
