#!/usr/bin/env python3
"""
waypoint_nav_sim.launch.py
仿真环境下的路径点导航 (替代 Nav2)

支持两种坐标模式:
  coord_mode=gazebo  (默认) 直接从 Gazebo 坐标变换到 game 坐标
  coord_mode=fastlio  模拟真机 FAST-LIO: 机器人从 (0,0,0) 开始

使用:
  ros2 launch rc2026_sim waypoint_nav_sim.launch.py
  ros2 launch rc2026_sim waypoint_nav_sim.launch.py coord_mode:=fastlio
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import LaunchConfigurationEquals
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    coord_mode_arg = DeclareLaunchArgument(
        'coord_mode', default_value='gazebo',
        description='坐标模式: gazebo / fastlio')

    # ── gazebo 模式: map→odom = identity ──
    tf_gazebo = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
        condition=LaunchConfigurationEquals('coord_mode', 'gazebo'),
    )

    # ── fastlio 模式: map→odom 抵消出生位姿 ──
    # 出生 (5.6, -1.4, yaw=π) → 逆变换使 map 帧中机器人从 (0,0,0) 开始
    tf_fastlio = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['5.6', '-1.4', '0', '3.14159', '0', '0', 'map', 'odom'],
        condition=LaunchConfigurationEquals('coord_mode', 'fastlio'),
    )

    # ── WaypointNavigator ──
    waypoint_nav = Node(
        package='decision_processor',
        executable='waypoint_nav',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'coord_mode': LaunchConfiguration('coord_mode'),
            'loc_offset_x': -1.4,
            'loc_offset_y': 0.4,
            'loc_offset_yaw': 1.5708,
            'map_frame': 'map',
            'robot_frame': 'base_link',
            'max_linear_speed': 1.0,
            'min_linear_speed': 0.05,
            'max_angular_speed': 2.0,
            'kp_linear': 1.2,
            'kp_angular': 2.0,
            'decel_distance': 0.30,
            'xy_tolerance': 0.05,
            'yaw_tolerance': 0.10,
            'waypoint_timeout': 30.0,
            'progress_timeout': 3.0,
            'visual_servo_timeout': 15.0,
            'control_rate': 20.0,
        }],
    )

    return LaunchDescription([
        coord_mode_arg,
        tf_gazebo,
        tf_fastlio,
        waypoint_nav,
    ])
