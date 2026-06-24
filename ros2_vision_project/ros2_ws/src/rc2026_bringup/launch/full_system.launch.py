#!/usr/bin/env python3
"""
full_system.launch.py  (v4.1)
真机全系统启动 (除串口通信外全部启动)

始终启动: URDF + RealSense + YOLO检测 + 视觉伺服 + IMU +
         FAST-LIO + EKF + WaypointNav + cmd_vel_bridge

可选:
  use_game_controller  比赛状态机
  enable_serial        串口通信 (auto_serial_bridge, 连接STM32)
  field_side           left(默认) / right
  test_area            区域单独测试: full(默认,完整比赛) / weapon(武馆) /
                       merlin(梅林) / confront(对抗区)
                       非full时自动: 1) 将 game_controller 启动阶段直接跳到
                       该区域入口对应阶段(跳过前置流程和终端KFS输入);
                       2) 将 start_x/y/yaw (= 该区域入口点的游戏坐标) 同时
                       作为 waypoint_navigator 的 loc_offset 与 FAST-LIO地图
                       重定位的ICP先验偏移 —— 测试前需把机器人实际摆放在
                       对应区域入口点 (见下方"区域测试"用法)

使用:
  ros2 launch rc2026_bringup full_system.launch.py                              # 左半场
  ros2 launch rc2026_bringup full_system.launch.py field_side:=right            # 右半场
  ros2 launch rc2026_bringup full_system.launch.py use_game_controller:=true
  ros2 launch rc2026_bringup full_system.launch.py enable_serial:=true use_game_controller:=true

区域测试 (机器人需实际摆放在对应区域入口点, 朝向见 config.py 中 WAYPOINT_*):
  # 武馆区域测试 (机器人摆放在 WAYPOINT_START)
  ros2 launch rc2026_bringup full_system.launch.py test_area:=weapon enable_serial:=true

  # 梅林区域测试 (机器人摆放在 WAYPOINT_MERLIN_ENTRY)
  ros2 launch rc2026_bringup full_system.launch.py test_area:=merlin enable_serial:=true \\
      kfs_real:='5 8' kfs_fake:='2 11' kfs_color:=blue

  # 对抗区区域测试 (机器人摆放在 WAYPOINT_CONFRONT_ENTRY)
  ros2 launch rc2026_bringup full_system.launch.py test_area:=confront enable_serial:=true

  # 右半场同理加 field_side:=right
"""
import math
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    TimerAction, GroupAction, SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


WEIGHTS_DIR = os.path.join(
    os.path.expanduser('~'),
    'GAFA-Artlnnov.RC2026-main', 'ros2_vision_project',
    'ros2_ws', 'src', 'vision_detector', 'weights',
)

START_GAME_X   = -1.4
START_GAME_Y   =  0.4
START_GAME_YAW =  math.pi / 2.0


def generate_launch_description():
    bringup_dir = get_package_share_directory('rc2026_bringup')
    nav_dir = get_package_share_directory('rc2026_navigation')
    urdf_file = os.path.join(bringup_dir, 'urdf', 'rc2026_robot.urdf.xacro')

    # ── 参数 ──
    model_arg = DeclareLaunchArgument('model', default_value='best.pt')
    conf_arg = DeclareLaunchArgument('conf', default_value='0.5')
    device_arg = DeclareLaunchArgument('device', default_value='cuda')
    field_side_arg = DeclareLaunchArgument(
        'field_side', default_value='left',
        description="场地半场: 'left'(默认) 或 'right'")
    test_area_arg = DeclareLaunchArgument(
        'test_area', default_value='full',
        description="区域单独测试: full(默认)/weapon/merlin/confront")
    kfs_real_arg = DeclareLaunchArgument(
        'kfs_real', default_value='5',
        description='区域测试用: 真KFS台阶编号 (空格/逗号分隔)')
    kfs_fake_arg = DeclareLaunchArgument(
        'kfs_fake', default_value='8',
        description='区域测试用: 假KFS台阶编号 (空格/逗号分隔)')
    kfs_color_arg = DeclareLaunchArgument(
        'kfs_color', default_value='blue',
        description='区域测试用: KFS颜色 (blue/red)')

    # test_area != full 时默认自动启动比赛状态机
    use_gc_default = PythonExpression(
        ["'false' if '", LaunchConfiguration('test_area'), "' == 'full' else 'true'"])
    use_gc_arg = DeclareLaunchArgument(
        'use_game_controller', default_value=use_gc_default,
        description='启动比赛状态机')
    enable_serial_arg = DeclareLaunchArgument(
        'enable_serial', default_value='false',
        description='启动串口通信 (连接 STM32)')

    # start_x/y/yaw 默认值随 field_side + test_area 自动切换为对应
    # 区域入口点的游戏坐标 (full/weapon=WAYPOINT_START, merlin=WAYPOINT_MERLIN_ENTRY,
    # confront=WAYPOINT_CONFRONT_ENTRY); 可手动覆盖
    start_x_default = PythonExpression([
        f"({START_GAME_X} if '", LaunchConfiguration('field_side'), f"' == 'left' else {-START_GAME_X})"
        " if '", LaunchConfiguration('test_area'), "' in ('full', 'weapon') else "
        "(-3.0 if '", LaunchConfiguration('field_side'), "' == 'left' else 3.0)"
        " if '", LaunchConfiguration('test_area'), "' == 'merlin' else "
        "(-5.4 if '", LaunchConfiguration('field_side'), "' == 'left' else 5.4)",
    ])
    start_y_default = PythonExpression([
        f"{START_GAME_Y} if '", LaunchConfiguration('test_area'), "' in ('full', 'weapon') else "
        "2.0 if '", LaunchConfiguration('test_area'), "' == 'merlin' else 11.6",
    ])
    start_yaw_default = PythonExpression([
        f"{START_GAME_YAW} if '", LaunchConfiguration('test_area'), "' in ('full', 'weapon', 'merlin') else "
        "(0.0 if '", LaunchConfiguration('field_side'), "' == 'left' else 3.14159265358979)",
    ])
    start_x_arg   = DeclareLaunchArgument('start_x',   default_value=start_x_default)
    start_y_arg   = DeclareLaunchArgument('start_y',   default_value=start_y_default)
    start_yaw_arg = DeclareLaunchArgument('start_yaw', default_value=start_yaw_default)

    # 将半场选择注入环境变量, 供 config.py 在节点启动时读取
    set_field_side_env = SetEnvironmentVariable(
        name='RC2026_FIELD_SIDE',
        value=LaunchConfiguration('field_side'))

    # ══════════════════════════════════════
    #   1. URDF 机器人模型
    # ══════════════════════════════════════

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file]), value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        parameters=[{'robot_description': robot_description}],
        output='screen',
    )

    joint_state_pub = Node(
        package='joint_state_publisher',
        executable='joint_state_publisher',
    )

    # ══════════════════════════════════════
    #   2. RealSense D435i 相机
    # ══════════════════════════════════════

    realsense = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch', 'rs_launch.py')),
        launch_arguments={
            'enable_color': 'true',
            'enable_depth': 'true',
            'enable_infra1': 'false',
            'enable_infra2': 'false',
            'align_depth.enable': 'true',
            'rgb_camera.color_profile': '1280x720x30',
            'depth_module.depth_profile': '848x480x30',
            'enable_gyro': 'true',
            'enable_accel': 'true',
            'gyro_fps': '200',
            'accel_fps': '100',
            'unite_imu_method': '2',
        }.items(),
    )

    # ══════════════════════════════════════
    #   3. YOLO 视觉检测 (延迟3s等相机就绪)
    # ══════════════════════════════════════

    detector_node = TimerAction(
        period=3.0,
        actions=[Node(
            package='vision_detector',
            executable='detector_node',
            name='vision_detector',
            output='screen',
            parameters=[{
                'model_path': PathJoinSubstitution([
                    WEIGHTS_DIR, LaunchConfiguration('model')]),
                'conf_threshold': LaunchConfiguration('conf'),
                'device': LaunchConfiguration('device'),
                'publish_visualization': True,
                'depth_sample_points': 24,
                'camera_topic': '/camera/camera/color/image_raw',
                'depth_topic': '/camera/camera/aligned_depth_to_color/image_raw',
                'camera_info_topic': '/camera/camera/color/camera_info',
            }],
        )],
    )

    # ══════════════════════════════════════
    #   4. 视觉伺服 + IMU处理 (延迟4s)
    # ══════════════════════════════════════

    processor_node = TimerAction(
        period=4.0,
        actions=[Node(
            package='decision_processor',
            executable='processor_node',
            name='decision_processor_node',
            output='screen',
            parameters=[{
                'wheel_diameter_m': 0.10781,
                'track_width_m': 0.50,
                'stop_distance_m': 0.20,
                'align_threshold_deg': 5.0,
                'pick_duration_s': 10.0,
                'conf_threshold': 0.5,
            }],
        )],
    )

    imu_node = TimerAction(
        period=4.0,
        actions=[Node(
            package='decision_processor',
            executable='imu_processor',
            name='imu_processor',
            output='screen',
        )],
    )

    # ══════════════════════════════════════
    #   5. 定位: FAST-LIO + EKF + 地图重定位
    #      localization.launch.py 会在 FAST-LIO 启动 5s 后自动
    #      对预建地图做一次 ICP 校正初始位姿偏移
    # ══════════════════════════════════════

    lidar_stack = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav_dir, 'launch', 'localization.launch.py')),
        launch_arguments={
            'field_side': LaunchConfiguration('field_side'),
            'use_reloc':  'true',
            'prior_offset_x':   LaunchConfiguration('start_x'),
            'prior_offset_y':   LaunchConfiguration('start_y'),
            'prior_offset_yaw': LaunchConfiguration('start_yaw'),
        }.items(),
    )

    # map → odom
    map_to_odom_tf = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom'],
    )

    # ══════════════════════════════════════
    #   6. 路径点导航
    # ══════════════════════════════════════

    waypoint_nav = Node(
        package='decision_processor',
        executable='waypoint_nav',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'coord_mode': 'fastlio',
            'loc_offset_x':   ParameterValue(LaunchConfiguration('start_x'),   value_type=float),
            'loc_offset_y':   ParameterValue(LaunchConfiguration('start_y'),   value_type=float),
            'loc_offset_yaw': ParameterValue(LaunchConfiguration('start_yaw'), value_type=float),
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
            'control_rate': 50.0,
        }],
    )

    # ══════════════════════════════════════
    #   7. 串口通信 + cmd_vel桥接 (可选)
    # ══════════════════════════════════════

    serial_stack = GroupAction(
        condition=IfCondition(LaunchConfiguration('enable_serial')),
        actions=[
            IncludeLaunchDescription(
                PythonLaunchDescriptionSource(
                    os.path.join(
                        get_package_share_directory('auto_serial_bridge'),
                        'launch', 'serial_bridge_by_node.launch.py')),
            ),
            Node(
                package='cmd_vel_bridge',
                executable='bridge_node',
                name='cmd_vel_bridge',
                output='screen',
            ),
        ],
    )

    # ══════════════════════════════════════
    #   8. 合体对齐节点 (延迟5s等相机就绪)
    # ══════════════════════════════════════

    dock_align_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='decision_processor',
            executable='dock_align_node',
            name='dock_align_node',
            output='screen',
            parameters=[{
                'marker_size_m': 0.10,   # ★真机用卷尺测量后修改
                'marker_ids':    [0],
                'target_dist_m': 0.30,
                'debug_gui':     False,
            }],
        )],
    )

    # ══════════════════════════════════════
    #   9. 比赛状态机 (可选)
    # ══════════════════════════════════════

    game_controller = Node(
        package='decision_processor',
        executable='game_controller',
        name='game_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_game_controller')),
        parameters=[{
            'test_area': LaunchConfiguration('test_area'),
            'kfs_real':  LaunchConfiguration('kfs_real'),
            'kfs_fake':  LaunchConfiguration('kfs_fake'),
            'kfs_color': LaunchConfiguration('kfs_color'),
        }],
    )

    return LaunchDescription([
        model_arg, conf_arg, device_arg,
        use_gc_arg, enable_serial_arg,
        field_side_arg,
        test_area_arg, kfs_real_arg, kfs_fake_arg, kfs_color_arg,
        start_x_arg, start_y_arg, start_yaw_arg,
        set_field_side_env,   # 必须在所有节点之前注入环境变量
        # 基础
        robot_state_pub,
        joint_state_pub,
        realsense,
        # 视觉
        detector_node,
        processor_node,
        imu_node,
        # 定位
        lidar_stack,
        map_to_odom_tf,
        # 导航
        waypoint_nav,
        # 速度桥接 (始终启动, /cmd_vel → /serial/chassis_cmd)
        Node(
            package='cmd_vel_bridge',
            executable='bridge_node',
            name='cmd_vel_bridge',
            output='screen',
        ),
        # 通信
        serial_stack,
        # 合体对齐
        dock_align_node,
        # 状态机
        game_controller,
    ])
