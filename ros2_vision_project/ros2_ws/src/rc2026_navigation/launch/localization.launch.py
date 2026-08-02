#!/usr/bin/env python3
"""
localization.launch.py
基于预建点云地图的启动重定位 + FAST-LIO2 + EKF

与 fastlio.launch.py 的区别:
  - 额外启动 map_relocalizer 节点
  - 根据 field_side 自动选择对应地图文件并设置先验偏移
  - ICP 匹配完成后自动校正 waypoint_navigator 的初始位姿

TF 链路:
  map → odom:          静态 (identity, 由 full_system.launch.py 提供)
  odom → camera_init:  静态 (identity, 本文件提供, 使 EKF 能将
                        FAST-LIO 的 /Odometry(camera_init帧) 转换到
                        odom 世界坐标系参与融合)
  camera_init → body:  FAST-LIO 动态发布
  odom → base_link:    EKF 融合结果动态发布 (publish_tf=true)

启动命令:
  # 右半场
  ros2 launch rc2026_navigation localization.launch.py field_side:=right

  # 左半场
  ros2 launch rc2026_navigation localization.launch.py field_side:=left

  # 调试: 不启动重定位 (仅 FAST-LIO)
  ros2 launch rc2026_navigation localization.launch.py field_side:=right use_reloc:=false

  # 区域测试: 自定义 ICP 先验偏移 (机器人实际摆放位置对应的游戏坐标)
  # 一般由 full_system.launch.py 根据 test_area 自动传入, 无需手动指定
  ros2 launch rc2026_navigation localization.launch.py field_side:=right \\
      prior_offset_x:=3.0 prior_offset_y:=2.0 prior_offset_yaw:=1.5708
"""
import math
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription, LogInfo, TimerAction,
)
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import (
    LaunchConfiguration, PythonExpression, PathJoinSubstitution,
)
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# ── 先验偏移 (右半场, 关于 x=0 的镜像即得左半场) ──────────────
RIGHT_OFFSET_X   =  1.4
RIGHT_OFFSET_Y   =  0.4
RIGHT_OFFSET_YAW =  math.pi / 2.0   # 1.5708

LEFT_OFFSET_X    = -1.4
LEFT_OFFSET_Y    =  0.4
LEFT_OFFSET_YAW  =  math.pi / 2.0


def generate_launch_description():
    nav_dir    = get_package_share_directory('rc2026_navigation')
    livox_dir  = get_package_share_directory('livox_ros_driver2')
    fastlio_dir = get_package_share_directory('fast_lio')

    map_dir = os.path.join(nav_dir, 'map')

    # ── 参数 ──────────────────────────────────────────────────
    field_side_arg = DeclareLaunchArgument(
        'field_side', default_value='right',
        description="场地半场: 'right' 或 'left'")

    use_reloc_arg = DeclareLaunchArgument(
        'use_reloc', default_value='true',
        description='是否启动地图重定位节点')

    # ── 动态选择地图文件 ───────────────────────────────────────
    map_file = PythonExpression([
        "'" + os.path.join(map_dir, 'right_half.pcd') + "'"
        " if '", LaunchConfiguration('field_side'), "' == 'right'"
        " else '" + os.path.join(map_dir, 'left_half.pcd') + "'",
    ])

    # ── ICP先验偏移 (机器人实际摆放位置对应的游戏坐标) ──────────
    # 默认值随 field_side 取整场比赛起点; 区域测试时由
    # full_system.launch.py 根据 test_area 传入对应区域入口坐标
    prior_x_default = PythonExpression([
        f"'{RIGHT_OFFSET_X}' if '",
        LaunchConfiguration('field_side'),
        f"' == 'right' else '{LEFT_OFFSET_X}'",
    ])
    prior_y_default = PythonExpression([
        f"'{RIGHT_OFFSET_Y}' if '",
        LaunchConfiguration('field_side'),
        f"' == 'right' else '{LEFT_OFFSET_Y}'",
    ])
    prior_yaw_default = PythonExpression([
        f"'{RIGHT_OFFSET_YAW}' if '",
        LaunchConfiguration('field_side'),
        f"' == 'right' else '{LEFT_OFFSET_YAW}'",
    ])
    prior_x_arg = DeclareLaunchArgument('prior_offset_x', default_value=prior_x_default)
    prior_y_arg = DeclareLaunchArgument('prior_offset_y', default_value=prior_y_default)
    prior_yaw_arg = DeclareLaunchArgument('prior_offset_yaw', default_value=prior_yaw_default)

    # ════════════════════════════════════════
    #   1. Livox 驱动
    # ════════════════════════════════════════
    livox_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(livox_dir, 'launch_ROS2', 'msg_MID360s_launch.py')),
    )

    # ════════════════════════════════════════
    #   2. FAST-LIO2 (建图 / 里程计)
    # ════════════════════════════════════════
    fastlio_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(fastlio_dir, 'launch', 'mapping.launch.py')),
        launch_arguments={
            'config_file': 'mid360.yaml',
            'rviz': 'false',
        }.items(),
    )

    # ════════════════════════════════════════
    #   2.5 TF桥接: odom → camera_init (identity)
    #       使 EKF 能将 FAST-LIO 的 /Odometry (frame_id=camera_init)
    #       转换到 odom 世界坐标系参与融合, 否则该测量会因 TF
    #       查找失败被 robot_localization 静默丢弃
    # ════════════════════════════════════════
    odom_to_camera_init_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='odom_to_camera_init_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'odom', 'camera_init'],
    )

    # ════════════════════════════════════════
    #   3. 轮式里程计 + EKF
    # ════════════════════════════════════════
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

    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[os.path.join(nav_dir, 'config', 'ekf.yaml')],
        remappings=[('/odometry/filtered', '/odom')],
    )

    # ════════════════════════════════════════
    #   4. 地图重定位节点 (延迟5s等FAST-LIO就绪)
    #      完成后自动更新 waypoint_navigator 的 loc_offset
    # ════════════════════════════════════════
    reloc_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='rc2026_navigation',
            executable='map_relocalizer',
            name='map_relocalizer',
            output='screen',
            condition=IfCondition(LaunchConfiguration('use_reloc')),
            parameters=[{
                'map_file':         map_file,
                'prior_offset_x':   ParameterValue(LaunchConfiguration('prior_offset_x'),   value_type=float),
                'prior_offset_y':   ParameterValue(LaunchConfiguration('prior_offset_y'),   value_type=float),
                'prior_offset_yaw': ParameterValue(LaunchConfiguration('prior_offset_yaw'), value_type=float),
                'scan_accum_pts':   3000,
                'icp_max_dist':     0.5,
                'icp_min_fitness':  0.05,
                'voxel_size':       0.1,
                'max_correction_xy': 1.0,
                'max_correction_yaw_deg': 30.0,
            }],
        )],
    )

    return LaunchDescription([
        field_side_arg,
        use_reloc_arg,
        prior_x_arg, prior_y_arg, prior_yaw_arg,
        LogInfo(msg=[
            '[localization] field_side=', LaunchConfiguration('field_side'),
            ' map_file=', map_file,
            ' prior_offset=(',
            LaunchConfiguration('prior_offset_x'), ', ',
            LaunchConfiguration('prior_offset_y'), ', ',
            LaunchConfiguration('prior_offset_yaw'), ')',
        ]),
        livox_launch,
        fastlio_launch,
        odom_to_camera_init_tf,
        wheel_odom_node,
        ekf_node,
        reloc_node,
    ])
