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
                       weapon_align(跳过导航直接测武器头夹取) /
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

  # 武器头夹取状态机测试 (机器人已摆放在 WAYPOINT_WEAPON_RACK)
  ros2 launch rc2026_bringup full_system.launch.py test_area:=weapon_align enable_serial:=true

  # 梅林区域测试 (机器人摆放在 WAYPOINT_MERLIN_ENTRY)
  ros2 launch rc2026_bringup full_system.launch.py test_area:=merlin enable_serial:=true \\
      kfs_real:='5 8' kfs_fake:='2 11' kfs_color:=blue

  # 对抗区区域测试 (机器人摆放在 WAYPOINT_EXIT_MERLIN, 即爬坡开始点)
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
WEAPON_RACK_LEFT_X   = -0.85
WEAPON_RACK_LEFT_Y   =  0.97
WEAPON_RACK_LEFT_YAW = -0.087
WEAPON_RACK_RIGHT_X   = 0.85
WEAPON_RACK_RIGHT_Y   = 1.25
WEAPON_RACK_RIGHT_YAW = math.pi


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
        description="运行区域/模式: full(默认)/weapon/weapon_align/merlin/confront/chongwu/jiugong")
    kfs_real_arg = DeclareLaunchArgument(
        'kfs_real', default_value='5',
        description='区域测试用: 真KFS台阶编号 (空格/逗号分隔)')
    kfs_fake_arg = DeclareLaunchArgument(
        'kfs_fake', default_value='8',
        description='区域测试用: 假KFS台阶编号 (空格/逗号分隔)')
    kfs_color_arg = DeclareLaunchArgument(
        'kfs_color', default_value='blue',
        description='区域测试用: KFS颜色 (blue/red)')

    use_gc_arg = DeclareLaunchArgument(
        'use_game_controller', default_value='true',
        description='启动比赛状态机，默认启动以订阅 /game/start_signal')
    enable_serial_arg = DeclareLaunchArgument(
        'enable_serial', default_value='false',
        description='启动串口通信 (连接 STM32)')
    enable_ir_r1_arg = DeclareLaunchArgument(
        'enable_ir_r1_signal', default_value='false',
        description='启动红外学习模块接收R1信号，并发布到 /game/r1_signal')
    ir_port_arg = DeclareLaunchArgument(
        'ir_port', default_value='/dev/ttyUSB1',
        description='红外学习模块USB-TTL串口，建议使用 /dev/serial/by-id/... 固定设备名')
    ir_baud_arg = DeclareLaunchArgument(
        'ir_baud', default_value='115200',
        description='红外学习模块串口波特率')
    ir_max_score_arg = DeclareLaunchArgument(
        'ir_max_score', default_value='1.0',
        description='红外特征序列最大编辑距离，越大越容易识别但更容易误判')
    ir_min_gap_arg = DeclareLaunchArgument(
        'ir_min_gap', default_value='1.0',
        description='红外最佳/次佳特征距离分差，越大越保守')
    enable_ir_key2_arg = DeclareLaunchArgument(
        'enable_ir_key2_signal', default_value='false',
        description='启动第二红外学习模块，专门接收KEY2=进入梅林信号')
    ir_key2_port_arg = DeclareLaunchArgument(
        'ir_key2_port', default_value='/dev/ttyUSB2',
        description='第二红外学习模块USB-TTL串口，默认 /dev/ttyUSB2')
    ir_key2_baud_arg = DeclareLaunchArgument(
        'ir_key2_baud', default_value='115200',
        description='第二红外学习模块串口波特率')
    ir_key2_preferred_margin_arg = DeclareLaunchArgument(
        'ir_key2_preferred_margin', default_value='1.0',
        description='第二红外模块KEY2优先余量，越大越偏向KEY2')
    debug_gui_arg = DeclareLaunchArgument(
        'debug_gui', default_value='false',
        description='调试模式：相机节点显示OpenCV预览窗口')
    match_timeout_default = PythonExpression([
        "180.0 if '", LaunchConfiguration('test_area'),
        "' in ('chongwu', 'jiugong') else 250.0",
    ])
    match_timeout_arg = DeclareLaunchArgument(
        'match_timeout_s', default_value=match_timeout_default,
        description='比赛/子模式超时停止时间 (s)')

    # start_x/y/yaw 默认值随 field_side + test_area 自动切换为对应
    # 区域入口点的游戏坐标 (full/weapon=WAYPOINT_START,
    # weapon_align=WAYPOINT_WEAPON_RACK, merlin=WAYPOINT_MERLIN_ENTRY,
    # jiugong/confront=WAYPOINT_EXIT_MERLIN); 可手动覆盖
    start_x_default = PythonExpression([
        f"({START_GAME_X} if '", LaunchConfiguration('field_side'), f"' == 'left' else {-START_GAME_X})"
        " if '", LaunchConfiguration('test_area'), "' in ('full', 'weapon', 'chongwu') else "
        f"({WEAPON_RACK_LEFT_X} if '", LaunchConfiguration('field_side'), f"' == 'left' else {WEAPON_RACK_RIGHT_X})"
        " if '", LaunchConfiguration('test_area'), "' == 'weapon_align' else "
        "(-3.0 if '", LaunchConfiguration('field_side'), "' == 'left' else 3.0)"
        " if '", LaunchConfiguration('test_area'), "' == 'merlin' else "
        "(-5.4 if '", LaunchConfiguration('field_side'), "' == 'left' else 5.4)"
        " if '", LaunchConfiguration('test_area'), "' == 'jiugong' else "
        "(-5.4 if '", LaunchConfiguration('field_side'), "' == 'left' else 5.4)",
    ])
    start_y_default = PythonExpression([
        f"{START_GAME_Y} if '", LaunchConfiguration('test_area'), "' in ('full', 'weapon', 'chongwu') else "
        f"({WEAPON_RACK_LEFT_Y} if '", LaunchConfiguration('field_side'), f"' == 'left' else {WEAPON_RACK_RIGHT_Y})"
        " if '", LaunchConfiguration('test_area'), "' == 'weapon_align' else "
        "2.0 if '", LaunchConfiguration('test_area'), "' == 'merlin' else "
        "8.4",
    ])
    start_yaw_default = PythonExpression([
        f"{START_GAME_YAW} if '", LaunchConfiguration('test_area'), "' in ('full', 'weapon', 'chongwu', 'merlin', 'jiugong', 'confront') else "
        f"({WEAPON_RACK_LEFT_YAW} if '", LaunchConfiguration('field_side'), f"' == 'left' else {WEAPON_RACK_RIGHT_YAW})"
        " if '", LaunchConfiguration('test_area'), "' == 'weapon_align' else "
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
                'raw_target_priority_classes': 'W_punch',
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
                'wheel_diameter_m': 0.10781,   # 轮子直径 (m)
                'track_width_m': 0.50,  # 轮距 (m)
                'stop_distance_m': 0.50,   # 停止距离 (m)
                'align_threshold_deg': 5.0,  # 对齐阈值 (度)
                'pick_duration_s': 10.0,    # 拾取持续时间 (s)
                'conf_threshold': 0.5,     # 置信度阈值
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
            'max_linear_speed': 0.6,  # 原0.25
            'min_linear_speed': 0.01,
            'max_angular_speed': 1.0,
            'kp_linear': 0.50,
            'kp_angular': 0.40,
            'decel_distance': 0.80,
            'max_linear_accel': 0.35,
            'max_angular_accel': 0.60,
            'xy_tolerance': 0.12,
            'yaw_tolerance': 0.15,
            'waypoint_timeout': 30.0,
            'progress_timeout': 3.0,    # 进度超时时间 (秒)
            'visual_servo_timeout': 15.0,   # 视觉伺服超时时间 (秒)
            'control_rate': 50.0,   # 控制频率 (Hz)
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
        ],
    )

    ir_r1_signal_node = Node(
        package='decision_processor',
        executable='ir_key_receiver',
        name='ir_key_receiver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ir_r1_signal')),
        parameters=[{
            'module_name': 'main',
            'port': LaunchConfiguration('ir_port'),
            'baud': ParameterValue(LaunchConfiguration('ir_baud'), value_type=int),
            'max_score': ParameterValue(LaunchConfiguration('ir_max_score'), value_type=float),
            'min_gap': ParameterValue(LaunchConfiguration('ir_min_gap'), value_type=float),
            # 默认: KEY1=启动按钮, KEY2=进入梅林, KEY3=合体后释放KFS指令
            'key1_start_value': 1,
            'key2_signal': 2,
            'key3_signal': 3,
        }],
    )

    ir_key2_signal_node = Node(
        package='decision_processor',
        executable='ir_key_receiver',
        name='ir_key2_receiver',
        output='screen',
        condition=IfCondition(LaunchConfiguration('enable_ir_key2_signal')),
        parameters=[{
            'module_name': 'key2',
            'port': LaunchConfiguration('ir_key2_port'),
            'baud': ParameterValue(LaunchConfiguration('ir_key2_baud'), value_type=int),
            'max_score': ParameterValue(LaunchConfiguration('ir_max_score'), value_type=float),
            'min_gap': ParameterValue(LaunchConfiguration('ir_min_gap'), value_type=float),
            'preferred_key': 'KEY2',
            'preferred_margin': ParameterValue(
                LaunchConfiguration('ir_key2_preferred_margin'), value_type=float),
            # 第二红外模块只负责KEY2=进入梅林，避免重复触发启动/释放KFS。
            'key1_start_value': -1,
            'key2_signal': 2,
            'key3_signal': -1,
        }],
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
                'debug_gui':     LaunchConfiguration('debug_gui'),
            }],
        )],
    )

    # ══════════════════════════════════════
    #   9. 精对齐节点 (延迟5s等相机就绪)
    # ══════════════════════════════════════

    fine_align_node = TimerAction(
        period=5.0,
        actions=[Node(
            package='decision_processor',
            executable='fine_align_node',
            name='fine_align_node',
            output='screen',
            parameters=[{
                'cam_index': 0,
                'debug_gui': LaunchConfiguration('debug_gui'),
            }],
        )],
    )

    # ══════════════════════════════════════
    #   10. 比赛状态机 (可选)
    # ══════════════════════════════════════

    game_controller = Node(
        package='decision_processor',
        executable='game_controller',
        name='game_controller',
        output='screen',
        condition=IfCondition(LaunchConfiguration('use_game_controller')),
        parameters=[{
            'test_area': ParameterValue(LaunchConfiguration('test_area'), value_type=str),
            'kfs_real':  ParameterValue(LaunchConfiguration('kfs_real'),  value_type=str),
            'kfs_fake':  ParameterValue(LaunchConfiguration('kfs_fake'),  value_type=str),
            'kfs_color': ParameterValue(LaunchConfiguration('kfs_color'), value_type=str),
            'match_timeout_s': ParameterValue(LaunchConfiguration('match_timeout_s'), value_type=float),
        }],
    )

    return LaunchDescription([
        model_arg, conf_arg, device_arg,
        field_side_arg,
        test_area_arg, kfs_real_arg, kfs_fake_arg, kfs_color_arg,
        use_gc_arg, enable_serial_arg, debug_gui_arg, match_timeout_arg,
        enable_ir_r1_arg, ir_port_arg, ir_baud_arg,
        ir_max_score_arg, ir_min_gap_arg,
        enable_ir_key2_arg, ir_key2_port_arg, ir_key2_baud_arg,
        ir_key2_preferred_margin_arg,
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
        ir_r1_signal_node,
        ir_key2_signal_node,
        # 合体对齐
        dock_align_node,
        # 精对齐 (USB相机)
        fine_align_node,
        # 状态机
        game_controller,
    ])
