#!/usr/bin/env python3
"""
simulation.launch.py  (v2.0)
一键启动完整仿真: Gazebo + 导航 + 视觉 + 决策 + 状态机

使用:
  # 左半场 (默认)
  ros2 launch rc2026_sim simulation.launch.py
  ros2 launch rc2026_sim simulation.launch.py use_game_controller:=true

  # 右半场
  ros2 launch rc2026_sim simulation.launch.py field_side:=right
  ros2 launch rc2026_sim simulation.launch.py field_side:=right use_game_controller:=true

  # 附加选项
  ros2 launch rc2026_sim simulation.launch.py field_side:=right use_vision:=true
  ros2 launch rc2026_sim simulation.launch.py field_side:=right use_game_controller:=true use_vision:=true

  # 测试: 自定义假定出生点 (留空则按 field_side 使用默认值)
  #   assumed_spawn_x/y/yaw: 自定义假定出生点 (game 坐标系, 单位米/弧度), map→odom TF /
  #   loc_offset / sim_relocalizer 的"假定值"均基于此重新计算
  ros2 launch rc2026_sim simulation.launch.py assumed_spawn_x:=1.4 assumed_spawn_y:=0.4 assumed_spawn_yaw:=1.5708

  # 测试: 启动点放置误差 + 重定位
  #   spawn_offset_x/y/yaw: 实际出生点相对假定出生点的偏移量 (模拟人工放置误差)
  #   enable_reloc: 是否启动 sim_relocalizer 在 reloc_delay 秒后用 Gazebo 真值修正 map→odom
  ros2 launch rc2026_sim simulation.launch.py spawn_offset_x:=0.3 spawn_offset_y:=-0.2 \
       enable_reloc:=true reloc_delay:=5.0

  # 测试: 里程计累积漂移 (导航对累积误差的鲁棒性)
  #   enable_odom_drift: 启动后由 odom_drift_injector 替代 planar_move 发布 odom TF,
  #                       按行驶距离/转角比例累积误差
  ros2 launch rc2026_sim simulation.launch.py enable_odom_drift:=true drift_linear:=0.05 drift_angular:=0.05

坐标系: Gazebo↔game  game_x=gz_y, game_y=6-gz_x
  左半场出生: Gazebo(5.6, -1.4, π)  →  game(-1.4, 0.4, π/2)
  右半场出生: Gazebo(5.6,  1.4, π)  →  game( 1.4, 0.4, π/2)
"""
import math
import os

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument, IncludeLaunchDescription,
    AppendEnvironmentVariable, SetEnvironmentVariable,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, Command, PythonExpression
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue
from ament_index_python.packages import get_package_share_directory


# ══════════════════════════════════════════════
#   双半场默认出生坐标 (Gazebo 坐标系)
#   坐标换算: gz_x = 6.0 - game_y,  gz_y = game_x
#   game_x = gz_y,  game_y = 6 - gz_x,  game_yaw = gz_yaw - π/2
#   可通过 assumed_spawn_x/y/yaw 参数覆盖, 自定义假定出生点
# ══════════════════════════════════════════════
_SPAWN_YAW = math.pi       # 两侧 Gazebo yaw 相同

# 左半场: game(-1.4, 0.4) → Gazebo(5.6, -1.4, π)
_LEFT_SPAWN_X,  _LEFT_SPAWN_Y  = 5.6, -1.4

# 右半场: game(1.4, 0.4) → Gazebo(5.6, 1.4, π)
_RIGHT_SPAWN_X, _RIGHT_SPAWN_Y = 5.6,  1.4


def generate_launch_description():
    sim_pkg       = get_package_share_directory('rc2026_sim')
    field_pkg     = get_package_share_directory('rc2026_field')
    gazebo_ros_pkg = get_package_share_directory('gazebo_ros')

    urdf_file  = os.path.join(sim_pkg,   'urdf',   'rc2026_robot_sim.urdf.xacro')
    world_file = os.path.join(field_pkg, 'worlds', 'robocon2026_with_kfs.world')

    # ── 参数声明 ──────────────────────────────────────────────
    field_side_arg = DeclareLaunchArgument(
        'field_side', default_value='left',
        description="场地半场: 'left'(默认) 或 'right'")
    use_gc_arg = DeclareLaunchArgument(
        'use_game_controller', default_value='false',
        description='启动 game_controller 比赛状态机')
    use_vision_arg = DeclareLaunchArgument(
        'use_vision', default_value='false',
        description='启动视觉检测 + 视觉伺服')

    # ── 测试用参数 ──
    assumed_spawn_x_arg = DeclareLaunchArgument(
        'assumed_spawn_x', default_value='',
        description='自定义假定出生点 game x坐标 (留空则按 field_side 使用默认值)')
    assumed_spawn_y_arg = DeclareLaunchArgument(
        'assumed_spawn_y', default_value='',
        description='自定义假定出生点 game y坐标 (留空则按 field_side 使用默认值)')
    assumed_spawn_yaw_arg = DeclareLaunchArgument(
        'assumed_spawn_yaw', default_value='',
        description='自定义假定出生点 game yaw (弧度, 留空则按 field_side 使用默认值)')
    spawn_offset_x_arg = DeclareLaunchArgument(
        'spawn_offset_x', default_value='0.0',
        description='出生点放置误差 (Gazebo x方向, 米), 模拟人工放置偏差')
    spawn_offset_y_arg = DeclareLaunchArgument(
        'spawn_offset_y', default_value='0.0',
        description='出生点放置误差 (Gazebo y方向, 米)')
    spawn_offset_yaw_arg = DeclareLaunchArgument(
        'spawn_offset_yaw', default_value='0.0',
        description='出生点放置误差 (Gazebo yaw, 弧度)')
    enable_reloc_arg = DeclareLaunchArgument(
        'enable_reloc', default_value='false',
        description='启动 sim_relocalizer, 模拟开局重定位修正放置误差')
    reloc_delay_arg = DeclareLaunchArgument(
        'reloc_delay', default_value='5.0',
        description='sim_relocalizer 在启动多少秒后用 Gazebo 真值修正 map→odom')
    enable_odom_drift_arg = DeclareLaunchArgument(
        'enable_odom_drift', default_value='false',
        description='启动 odom_drift_injector, 模拟里程计累积漂移')
    drift_linear_arg = DeclareLaunchArgument(
        'drift_linear', default_value='0.03',
        description='里程计线性漂移比例 (每米额外误差)')
    drift_angular_arg = DeclareLaunchArgument(
        'drift_angular', default_value='0.03',
        description='里程计角度漂移比例 (每弧度额外误差)')

    fs = LaunchConfiguration('field_side')

    # ── 注入 RC2026_FIELD_SIDE 环境变量 (供 config.py 读取) ──
    set_field_env = SetEnvironmentVariable('RC2026_FIELD_SIDE', fs)

    # ── 半场相关参数 (PythonExpression 在运行时计算) ──────────
    # 假定出生坐标 (Gazebo) -- map→odom TF 与 sim_relocalizer 的"假定值"均基于此
    # 若 assumed_spawn_x/y/yaw (game坐标) 非空, 则换算为 Gazebo 坐标作为自定义出生点;
    # 否则按 field_side 取默认值 (Gazebo 坐标)
    _custom_gx   = LaunchConfiguration('assumed_spawn_x')   # 自定义出生点 game x
    _custom_gy   = LaunchConfiguration('assumed_spawn_y')   # 自定义出生点 game y
    _custom_gyaw = LaunchConfiguration('assumed_spawn_yaw')  # 自定义出生点 game yaw

    _default_ax   = PythonExpression([f"'{_LEFT_SPAWN_X}' if '", fs, f"' == 'left' else '{_RIGHT_SPAWN_X}'"])
    _default_ay   = PythonExpression([f"'{_LEFT_SPAWN_Y}' if '", fs, f"' == 'left' else '{_RIGHT_SPAWN_Y}'"])
    _default_ayaw = str(_SPAWN_YAW)   # 两侧相同

    # game → Gazebo: gz_x = 6 - game_y, gz_y = game_x, gz_yaw = game_yaw + π/2
    # (空字符串时给出占位值 '0', 仅在 _custom_gx == '' 时该分支不会被选用)
    _custom_ax = PythonExpression([
        "str(6 - float('", _custom_gy, "')) if '", _custom_gy, "' != '' else '0'"])
    _custom_ay = PythonExpression([
        "str(float('", _custom_gx, "')) if '", _custom_gx, "' != '' else '0'"])
    _custom_ayaw = PythonExpression([
        "str(float('", _custom_gyaw, "') + pi / 2) if '", _custom_gyaw, "' != '' else '0'"])

    assumed_spawn_x = PythonExpression([
        "'", _custom_ax, "' if '", _custom_gx, "' != '' else '", _default_ax, "'"])
    assumed_spawn_y = PythonExpression([
        "'", _custom_ay, "' if '", _custom_gx, "' != '' else '", _default_ay, "'"])
    assumed_spawn_yaw = PythonExpression([
        "'", _custom_ayaw, "' if '", _custom_gx, "' != '' else '", _default_ayaw, "'"])

    # 实际出生坐标 = 假定出生坐标 + 放置误差 (spawn_offset_*)
    spawn_x = PythonExpression([
        'str(float(', assumed_spawn_x, ') + float(', LaunchConfiguration('spawn_offset_x'), '))'])
    spawn_y = PythonExpression([
        'str(float(', assumed_spawn_y, ') + float(', LaunchConfiguration('spawn_offset_y'), '))'])
    spawn_yaw = PythonExpression([
        'str(float(', assumed_spawn_yaw, ') + float(', LaunchConfiguration('spawn_offset_yaw'), '))'])

    # map→odom 静态 TF (基于假定出生点, 不感知放置误差 -- 模拟"开局前我们以为自己在这里")
    # tf = (-cos(yaw)*x - sin(yaw)*y, sin(yaw)*x - cos(yaw)*y, -yaw)
    tf_x = PythonExpression([
        'str(-cos(', assumed_spawn_yaw, ') * ', assumed_spawn_x,
        ' - sin(', assumed_spawn_yaw, ') * ', assumed_spawn_y, ')'])
    tf_y = PythonExpression([
        'str(sin(', assumed_spawn_yaw, ') * ', assumed_spawn_x,
        ' - cos(', assumed_spawn_yaw, ') * ', assumed_spawn_y, ')'])
    tf_yaw = PythonExpression(['str(-(', assumed_spawn_yaw, '))'])

    # waypoint_navigator loc_offset: map 原点 (=假定出生点) 对应的 game 坐标
    # game_x = gz_y, game_y = 6 - gz_x, game_yaw = gz_yaw - π/2
    off_x = ParameterValue(assumed_spawn_y, value_type=float)
    off_y = ParameterValue(
        PythonExpression(['str(6 - ', assumed_spawn_x, ')']), value_type=float)
    off_yaw = ParameterValue(
        PythonExpression(['str(', assumed_spawn_yaw, ' - pi / 2)']), value_type=float)

    # ══════════════════════════════════════
    #   Gazebo 仿真环境
    # ══════════════════════════════════════

    set_model_path = AppendEnvironmentVariable(
        name='GAZEBO_MODEL_PATH',
        value=os.path.join(field_pkg, 'models'))

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros_pkg, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world_file}.items(),
    )

    # ══════════════════════════════════════
    #   机器人模型
    # ══════════════════════════════════════

    # 启用 odom_drift_injector 时, 关闭 planar_move 自带的 odom→base_footprint TF 发布,
    # 改由 odom_drift_injector 接管 (在 planar_move 真值基础上叠加累积漂移)
    publish_odom_tf = PythonExpression([
        "'false' if '", LaunchConfiguration('enable_odom_drift'), "' == 'true' else 'true'"])

    robot_description = ParameterValue(
        Command(['xacro ', urdf_file, ' publish_odom_tf:=', publish_odom_tf]), value_type=str)

    robot_state_pub = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description, 'use_sim_time': True}],
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
            '-entity', 'rc2026_robot',
            '-x', spawn_x,
            '-y', spawn_y,
            '-z', '0.0',
            '-Y', spawn_yaw,
        ],
    )

    # ══════════════════════════════════════
    #   定位 + 导航
    # ══════════════════════════════════════

    # 关闭重定位时: 直接发布静态 map→odom TF (基于假定出生点, 无放置误差修正)
    map_to_odom = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='map_to_odom_tf',
        arguments=[tf_x, tf_y, '0', tf_yaw, '0', '0', 'map', 'odom'],
        condition=UnlessCondition(LaunchConfiguration('enable_reloc')),
    )

    # 启用重定位时: 用 sim_relocalizer 模拟"开局先按假定出生点定位,
    # reloc_delay 秒后用 Gazebo 真值修正放置误差"
    sim_relocalizer = Node(
        package='rc2026_sim',
        executable='sim_relocalizer',
        name='sim_relocalizer',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'assumed_x':   ParameterValue(assumed_spawn_x,   value_type=float),
            'assumed_y':   ParameterValue(assumed_spawn_y,   value_type=float),
            'assumed_yaw': ParameterValue(assumed_spawn_yaw, value_type=float),
            'entity_name': 'rc2026_robot',
            'reloc_delay': ParameterValue(LaunchConfiguration('reloc_delay'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('enable_reloc')),
    )

    # 里程计累积漂移注入 (与 publish_odom_tf:=false 配套使用)
    odom_drift_injector = Node(
        package='rc2026_sim',
        executable='odom_drift_injector',
        name='odom_drift_injector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'drift_linear':  ParameterValue(LaunchConfiguration('drift_linear'),  value_type=float),
            'drift_angular': ParameterValue(LaunchConfiguration('drift_angular'), value_type=float),
        }],
        condition=IfCondition(LaunchConfiguration('enable_odom_drift')),
    )

    waypoint_nav = Node(
        package='decision_processor',
        executable='waypoint_nav',
        name='waypoint_navigator',
        output='screen',
        parameters=[{
            'use_sim_time':      True,
            'coord_mode':        'fastlio',
            'loc_offset_x':      off_x,
            'loc_offset_y':      off_y,
            'loc_offset_yaw':    off_yaw,
            'map_frame':         'map',
            'robot_frame':       'base_link',
            'max_linear_speed':  1.0,
            'min_linear_speed':  0.05,
            'max_angular_speed': 2.0,
            'kp_linear':         1.2,
            'kp_angular':        2.0,
            'decel_distance':    0.30,
            'xy_tolerance':      0.05,
            'yaw_tolerance':     0.10,
            'waypoint_timeout':  30.0,
            'progress_timeout':  3.0,
            'visual_servo_timeout': 15.0,
            'control_rate':      20.0,
        }],
    )

    # ══════════════════════════════════════
    #   视觉系统 (可选)
    # ══════════════════════════════════════

    detector_node = Node(
        package='vision_detector',
        executable='detector_node',
        name='vision_detector',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'model_path': 'best.pt',
            'conf_threshold': 0.5,
            'device': 'cuda',
            'publish_visualization': True,
            'camera_topic': '/camera/camera/color/image_raw',
            'depth_topic': '',
            'camera_info_topic': '/camera/camera/color/camera_info',
        }],
        condition=IfCondition(LaunchConfiguration('use_vision')),
    )

    processor_node = Node(
        package='decision_processor',
        executable='processor_node',
        name='decision_processor_node',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'stop_distance_m': 0.20,
            'align_threshold_deg': 5.0,
            'pick_duration_s': 10.0,
            'conf_threshold': 0.5,
        }],
        condition=IfCondition(LaunchConfiguration('use_vision')),
    )

    # ══════════════════════════════════════
    #   比赛状态机 + Mock反馈 (可选)
    # ══════════════════════════════════════

    game_controller = Node(
        package='decision_processor',
        executable='game_controller',
        name='game_controller',
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(LaunchConfiguration('use_game_controller')),
    )

    mock_feedback = Node(
        package='decision_processor',
        executable='mock_feedback',
        name='mock_feedback',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'mock_nav':   False,
            'auto_start': False,
        }],
        condition=IfCondition(LaunchConfiguration('use_game_controller')),
    )

    return LaunchDescription([
        field_side_arg,
        use_gc_arg,
        use_vision_arg,
        assumed_spawn_x_arg,
        assumed_spawn_y_arg,
        assumed_spawn_yaw_arg,
        spawn_offset_x_arg,
        spawn_offset_y_arg,
        spawn_offset_yaw_arg,
        enable_reloc_arg,
        reloc_delay_arg,
        enable_odom_drift_arg,
        drift_linear_arg,
        drift_angular_arg,
        set_field_env,
        set_model_path,
        gazebo,
        robot_state_pub,
        joint_state_pub,
        spawn_robot,
        map_to_odom,
        sim_relocalizer,
        odom_drift_injector,
        waypoint_nav,
        detector_node,
        processor_node,
        game_controller,
        mock_feedback,
    ])
