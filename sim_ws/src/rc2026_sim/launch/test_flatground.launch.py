#!/usr/bin/env python3
"""
test_flatground.launch.py
空白平地测试: 验证 fastlio 模式的定位误差

测试方法:
  1. 正常启动, 机器人在 (0,0), 系统认为 game=(0,0):
     ros2 launch rc2026_sim test_flatground.launch.py
     发送目标: ros2 topic pub --once /waypoint_nav/goal_pose ...
     → 机器人应该精确到达

  2. 模拟"放偏了": 改出生点但不改 offset:
     ros2 launch rc2026_sim test_flatground.launch.py spawn_x:=0.3 spawn_y:=0.1
     → 系统仍然认为机器人在 game (0,0), 但实际偏了 (0.3, 0.1)
     → 导航目标会全部偏移 0.3m 和 0.1m

坐标: game = TF 直接读数 + offset, 无轴交换, 最简单的情况
"""
import os

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    sim_pkg = get_package_share_directory('rc2026_sim')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file = os.path.join(sim_pkg, 'urdf', 'rc2026_robot_sim.urdf.xacro')
    world_file = os.path.join(sim_pkg, 'worlds', 'empty_ground.world')

    # ── 出生点参数 (可调, 模拟"放偏了") ──
    spawn_x_arg = DeclareLaunchArgument('spawn_x', default_value='0.0')
    spawn_y_arg = DeclareLaunchArgument('spawn_y', default_value='0.0')
    spawn_yaw_arg = DeclareLaunchArgument('spawn_yaw', default_value='0.0')

    # ── Gazebo 空白平地 ──
    gazebo = Node(
        package='gazebo_ros',
        executable='gzserver',
        arguments=[world_file, '-s', 'libgazebo_ros_factory.so',
                   '-s', 'libgazebo_ros_init.so',
                   '-s', 'libgazebo_ros_state.so'],
        output='screen',
    )
    gazebo_client = Node(
        package='gazebo_ros',
        executable='gzclient',
        output='screen',
    )

    # ── Robot ──
    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{
            'robot_description': robot_description,
            'use_sim_time': True,
        }],
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
        parameters=[{'use_sim_time': True}],
    )

    spawn_robot = Node(
        package='gazebo_ros',
        executable='spawn_entity.py',
        output='screen',
        arguments=[
            '-topic', 'robot_description',
            '-entity', 'test_robot',
            '-x', LaunchConfiguration('spawn_x'),
            '-y', LaunchConfiguration('spawn_y'),
            '-z', '0.0',
            '-Y', LaunchConfiguration('spawn_yaw'),
        ],
    )

    # ── TF: map → odom (identity, 不做任何变换) ──
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ── WaypointNavigator (offset 模式, 无坐标变换) ──
    # game = TF读数 + offset, offset 全部为 0
    # 所以 game 坐标 = Gazebo 坐标, 最直观
    waypoint_nav = Node(
        package='decision_processor',
        executable='waypoint_nav',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'coord_mode': 'offset',
            'loc_offset_x': 0.0,
            'loc_offset_y': 0.0,
            'loc_offset_yaw': 0.0,
            'map_frame': 'map',
            'robot_frame': 'base_link',
            'max_linear_speed': 0.5,
            'kp_linear': 1.0,
            'kp_angular': 1.5,
            'decel_distance': 0.3,
            'xy_tolerance': 0.05,
            'yaw_tolerance': 0.10,
            'waypoint_timeout': 30.0,
            'progress_timeout': 5.0,
            'control_rate': 20.0,
        }],
    )

    return LaunchDescription([
        spawn_x_arg,
        spawn_y_arg,
        spawn_yaw_arg,
        gazebo,
        gazebo_client,
        robot_state_pub,
        joint_state_pub,
        spawn_robot,
        map_to_odom,
        waypoint_nav,
    ])
