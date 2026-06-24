#!/usr/bin/env python3
"""
navigation.launch.py
真机导航启动: FAST-LIO + EKF融合 + WaypointNavigator + GameController

定位链路:
  Livox Mid-360S → FAST-LIO → /odom/lidar ─┐
  STM32 编码器 → wheel_odom → /odom/wheel ──┤→ EKF → TF(odom→base_link)
                                            │
  map→odom TF (静态, 起点已知) ──────────────┘→ TF(map→base_link)
                                                    │
  waypoint_navigator 读取 TF → PID闭环导航 → /cmd_vel

使用:
  ros2 launch rc2026_bringup navigation.launch.py
  ros2 launch rc2026_bringup navigation.launch.py use_game_controller:=true
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory('rc2026_navigation')
    bringup_dir = get_package_share_directory('rc2026_bringup')

    # ── Launch Arguments ──
    use_gc_arg = DeclareLaunchArgument(
        'use_game_controller', default_value='false',
        description='启动 game_controller 比赛状态机')

    start_x_arg = DeclareLaunchArgument(
        'start_x', default_value='-1.4',
        description='启动点 game 坐标 x')
    start_y_arg = DeclareLaunchArgument(
        'start_y', default_value='0.4',
        description='启动点 game 坐标 y')
    start_yaw_arg = DeclareLaunchArgument(
        'start_yaw', default_value='1.5708',
        description='启动点 game 坐标 yaw (rad)')

    # ── FAST-LIO + EKF 融合定位 ──
    fastlio_ekf = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'fastlio.launch.py')),
        launch_arguments={
            'mode': 'mapping',
            'enable_ekf': 'true',
        }.items(),
    )

    # ── 静态 TF: map → odom (起点已知, FAST-LIO从0开始) ──
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── WaypointNavigator (fastlio 模式) ──
    waypoint_nav = Node(
        package='decision_processor',
        executable='waypoint_nav',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'coord_mode': 'fastlio',
            'loc_offset_x': LaunchConfiguration('start_x'),
            'loc_offset_y': LaunchConfiguration('start_y'),
            'loc_offset_yaw': LaunchConfiguration('start_yaw'),
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

    # ── cmd_vel → STM32 桥接 ──
    cmd_vel_bridge = Node(
        package='cmd_vel_bridge',
        executable='bridge_node',
        name='cmd_vel_bridge',
        output='screen',
    )

    # ── Game Controller (可选) ──
    game_controller = Node(
        package='decision_processor',
        executable='game_controller',
        name='game_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_game_controller')),
    )

    return LaunchDescription([
        use_gc_arg,
        start_x_arg,
        start_y_arg,
        start_yaw_arg,
        fastlio_ekf,
        map_to_odom_tf,
        waypoint_nav,
        cmd_vel_bridge,
        game_controller,
    ])
