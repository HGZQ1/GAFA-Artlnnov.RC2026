#!/usr/bin/env python3
"""
nav2.launch.py
启动 Nav2 导航栈（不含 AMCL，使用 FAST-LIO 提供定位）

TF 链路:
  map → odom: 静态 (identity)
  odom → camera_init: 静态 (identity)
  camera_init → body: FAST-LIO 动态发布
  body → base_link: 静态 (identity)
"""
import os
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    nav_dir = get_package_share_directory('rc2026_navigation')
    bringup_dir = get_package_share_directory('rc2026_bringup')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')

    params_file = os.path.join(nav_dir, 'config', 'nav2', 'planner.yaml')

    map_arg = DeclareLaunchArgument(
        'map', default_value=os.path.join(bringup_dir, 'map', 'arena.yaml'),
        description='地图文件路径 (.yaml)')

    # 机器人在地图上的初始位置（FAST-LIO开机时的物理位置）
    start_x_arg = DeclareLaunchArgument('start_x', default_value='1.4',
        description='起始X坐标 (m)')
    start_y_arg = DeclareLaunchArgument('start_y', default_value='11.6',
        description='起始Y坐标 (m)')
    start_yaw_arg = DeclareLaunchArgument('start_yaw', default_value='-1.5708',
        description='起始朝向 (rad)')

    # ── TF 桥接: 构建 Nav2 所需的 map→odom→...→base_link 链路 ──
    # FAST-LIO 发布: camera_init → body
    # 我们补充: map→odom, odom→camera_init, body→base_link (均为 identity)
    # 机器人开机位置在地图中的坐标 (x, y, z, yaw)
    # 根据实际放置位置修改，或通过 launch argument 传入
    start_x = LaunchConfiguration('start_x')
    start_y = LaunchConfiguration('start_y')
    start_yaw = LaunchConfiguration('start_yaw')

    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['--x', start_x, '--y', start_y, '--z', '0',
                   '--yaw', start_yaw, '--pitch', '0', '--roll', '0',
                   '--frame-id', 'map', '--child-frame-id', 'odom'],
    )

    odom_to_camera_init_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_camera_init_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
    )

    body_to_base_link_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='body_to_base_link_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'body', 'base_link'],
    )

    # ── PointCloud2 → LaserScan 转换 (Nav2 costmap 需要) ──
    pointcloud_to_laserscan = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        name='pointcloud_to_laserscan',
        remappings=[
            ('cloud_in', '/livox/cloud_registered'),
            ('scan', '/scan'),
        ],
        parameters=[{
            'target_frame': 'base_link',
            'transform_tolerance': 0.1,
            'min_height': 0.1,
            'max_height': 1.5,
            'angle_min': -3.14159,
            'angle_max': 3.14159,
            'angle_increment': 0.00872,   # ~0.5 degree
            'scan_time': 0.1,
            'range_min': 0.3,
            'range_max': 40.0,
            'use_inf': True,
            'inf_epsilon': 1.0,
        }],
    )

    # ── Map Server (独立启动，不经过 AMCL) ──
    map_server = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'yaml_filename': LaunchConfiguration('map'),
        }],
    )

    map_server_lifecycle = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_map',
        output='screen',
        parameters=[{
            'use_sim_time': False,
            'autostart': True,
            'node_names': ['map_server'],
        }],
    )

    # ── Nav2 Navigation (不含 localization/AMCL) ──
    nav2_navigation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
        ),
        launch_arguments={
            'use_sim_time': 'false',
            'params_file': params_file,
            'autostart': 'true',
        }.items(),
    )

    return LaunchDescription([
        map_arg,
        start_x_arg,
        start_y_arg,
        start_yaw_arg,
        map_to_odom_tf,
        odom_to_camera_init_tf,
        body_to_base_link_tf,
        pointcloud_to_laserscan,
        map_server,
        map_server_lifecycle,
        nav2_navigation,
    ])
