"""
config.py
所有需要按实际硬件调整的参数集中在这里
"""
import math
import os as _os

# ══════════════════════════════════════
#   场地半场选择
#   'left'  = 左半场 (默认, x < 0)
#   'right' = 右半场 (x > 0, 关于 x=0 镜像)
#
#   切换方法:
#     1. 直接改这行: FIELD_SIDE = 'right'
#     2. 启动时传参: field_side:=right (由 launch 文件注入环境变量)
# ══════════════════════════════════════
FIELD_SIDE = _os.environ.get('RC2026_FIELD_SIDE', 'left')

# ══════════════════════════════════════
#   TF 坐标系名称定义
# ══════════════════════════════════════
FRAME_BASE_LINK   = 'base_link'
FRAME_CAMERA      = 'camera_link'
FRAME_ARM_BASE    = 'arm_base_link'

# ══════════════════════════════════════
#   相机安装参数
#   权威来源: rc2026_bringup/config/robot_params.yaml → camera
# ══════════════════════════════════════
CAMERA_OFFSET_X   =  0.05
CAMERA_OFFSET_Y   =  0.0
CAMERA_OFFSET_Z   =  0.45         # 更新: 相机安装在框架上方
CAMERA_ROLL_RAD   = 0.0
CAMERA_PITCH_RAD  = 0.0
CAMERA_YAW_RAD    = 0.0

# ══════════════════════════════════════
#   机械臂底座安装参数
# ══════════════════════════════════════
ARM_OFFSET_X      =  0.0
ARM_OFFSET_Y      =  0.0
ARM_OFFSET_Z      =  0.05
ARM_ROLL_RAD      =  0.0
ARM_PITCH_RAD     =  0.0
ARM_YAW_RAD       =  0.0

# 机械臂相对相机的水平偏移（米，正=相机左侧）
ARM_LATERAL_FROM_CAMERA = ARM_OFFSET_Y - CAMERA_OFFSET_Y

# ══════════════════════════════════════
#   底盘物理参数
#   权威来源: rc2026_bringup/config/robot_params.yaml
#   此处保留默认值供单元测试和独立运行使用
#   正式运行时由 launch 文件注入 robot_params.yaml 中的值
# ══════════════════════════════════════
WHEEL_DIAMETER_M  = 0.10781        # 万向轮直径 (107.81mm)
TRACK_WIDTH_M     = 0.50           # 左右轮距 (万向轮取wheel_base_y)

# ══════════════════════════════════════
#   运动控制参数
# ══════════════════════════════════════
STOP_DISTANCE_M     = 0.50
ARRIVAL_THRESHOLD_M = 0.55
ALIGN_THRESHOLD_DEG = 5.0

# ══════════════════════════════════════
#   决策参数
# ══════════════════════════════════════
PICK_DURATION_S     = 10.0
ARRIVAL_SETTLE_S    = 1.0
CONF_THRESHOLD      = 0.5
CONFIRM_FRAMES      = 3
LOST_FRAMES         = 5
MAX_JUMP_M          = 0.8

# ══════════════════════════════════════════════════════════════════
#   武馆模型标签映射（best.pt / wuqi.pt）
#   key = YOLO class_id, value = 标签名
#   修改模型后只需更新此表
# ══════════════════════════════════════════════════════════════════
WUGUAN_CLASS_LABELS = {
    0: 'W_punch',    # 拳
    1: 'W_palm',     # 掌
    2: 'W_spear',    # 矛尖
}

# 武馆允许拾取的类别（类别ID列表）
WUGUAN_VALID_CLASSES = [0, 1, 2]

# 武馆置信度阈值
WUGUAN_CONF_MIN = 0.70

# 武馆需要拾取的武器总数
WUGUAN_TOTAL_WEAPONS = 999    # 999=不限制；比赛改为3

# ══════════════════════════════════════════════════════════════════
#   梅林模型标签映射（kfs.pt）
#   key = YOLO class_id, value = 标签名
#   模型类别变动时只需更新此表，代码无需修改
# ══════════════════════════════════════════════════════════════════
MEILIN_CLASS_LABELS = {
    0:  'FAKE_1',   1:  'FAKE_2',   2:  'FAKE_3',   3:  'FAKE_4',
    4:  'FAKE_5',   5:  'FAKE_6',   6:  'FAKE_7',   7:  'FAKE_8',
    8:  'FAKE_9',   9:  'FAKE_10',  10: 'FAKE_11',  11: 'FAKE_12',
    12: 'FAKE_13',  13: 'FAKE_14',  14: 'FAKE_15',
    15: 'REAL_1',   16: 'REAL_2',   17: 'REAL_3',   18: 'REAL_4',
    19: 'REAL_5',   20: 'REAL_6',   21: 'REAL_7',   22: 'REAL_8',
    23: 'REAL_9',   24: 'REAL_10',  25: 'REAL_11',  26: 'REAL_12',
    27: 'REAL_13',  28: 'REAL_14',  29: 'REAL_15',
}

# 梅林标签前缀（通过前缀判断真假，不依赖固定ID）
MEILIN_REAL_PREFIX = 'REAL_'
MEILIN_FAKE_PREFIX = 'FAKE_'

# R1 方块标签列表（目前模型无此类别，后续添加R1标签后扩展此列表）
# 命名例：'R1_1', 'R1_2'；在 scenario_meilin 中被视为不可触碰
MEILIN_R1_LABELS = []    # 模型加入R1后填入，如 ['R1_1', 'R1_2', 'R1_3']

# 梅林置信度阈值（比武馆严格，宁漏不误触）
MEILIN_CONF_MIN = 0.50

# 梅林收集目标数（比赛规则：4个真KFS随机放，最多能拾4个）
MEILIN_TOTAL_TARGETS = 2

# 梅林到达拾取距离（米）
MEILIN_STOP_DIST = 0.12

# ══════════════════════════════════════════════════════════════════
#   梅林方块中心坐标 (场地全局坐标, 来源: field_waypoints.yaml)
#   修改此处即可统一调整梅林所有方块坐标
# ══════════════════════════════════════════════════════════════════
BLOCK_CENTERS = {
    1:  (-1.8, 3.8),  2:  (-3.0, 3.8),  3:  (-4.2, 3.8),
    4:  (-1.8, 5.0),  5:  (-3.0, 5.0),  6:  (-4.2, 5.0),
    7:  (-1.8, 6.2),  8:  (-3.0, 6.2),  9:  (-4.2, 6.2),
    10: (-1.8, 7.4),  11: (-3.0, 7.4),  12: (-4.2, 7.4),
}

# 方块平台绝对高度 (mm, 来源: field_waypoints.yaml)
BLOCK_HEIGHTS_MM = {
    1:  400,  2:  200,  3:  400,
    4:  600,  5:  400,  6:  200,
    7:  400,  8:  600,  9:  400,
    10: 200,  11: 400,  12: 200,
}

# 方块网格位置 (row, col)  row 0=入口侧  col 0=左侧
BLOCK_GRID = {
    1:  (0, 0),  2:  (0, 1),  3:  (0, 2),
    4:  (1, 0),  5:  (1, 1),  6:  (1, 2),
    7:  (2, 0),  8:  (2, 1),  9:  (2, 2),
    10: (3, 0),  11: (3, 1),  12: (3, 2),
}

BLOCK_SIZE_M      = 1.2   # 方块边长 (判定区域)
BLOCK_HALF_SIZE_M = 0.6

# ══════════════════════════════════════════════════════════════════
#   梅林路径规划
# ══════════════════════════════════════════════════════════════════
MEILIN_ENTRY_BLOCKS = [1, 2, 3]
MEILIN_EXIT_BLOCKS  = [10, 11, 12]
MERLIN_DEFAULT_ENTRY = 2

# 入口爬升点 (从场地地面到入口方块前的导航目标)
MERLIN_ENTRY_CLIMB_POINTS = {
    1: {'x': -1.8, 'y': 2.8, 'yaw': 1.5708},
    2: {'x': -3.0, 'y': 2.8, 'yaw': 1.5708},
    3: {'x': -4.2, 'y': 2.8, 'yaw': 1.5708},
}

# 出口下降点 (从出口方块下来后的落地位置)
MERLIN_EXIT_DESCEND_POINTS = {
    10: {'x': -1.8, 'y': 8.4, 'yaw': 1.5708},
    11: {'x': -3.0, 'y': 8.4, 'yaw': 1.5708},
    12: {'x': -4.2, 'y': 8.4, 'yaw': 1.5708},
}

# 爬升/下降触发距离: 距中心0.2m = 距边缘0.4m
MERLIN_TRIGGER_FROM_CENTER_M = 0.2

# 爬升/下降等待时间 (秒, 等待下位机完成动作)
MERLIN_CLIMB_WAIT_S  = 3.0
MERLIN_PICKUP_WAIT_S = 5.0

# KFS拾取夹爪动作编号
GRIPPER_KFS_PICKUP = 3

# 夹爪状态反馈 (/feedback/gripper, GripperStatus.status)
GRIPPER_STATUS_IDLE      = 0   # 空载
GRIPPER_STATUS_GRABBED   = 1   # 已抓取
GRIPPER_STATUS_ASSEMBLED = 2   # 已完成组装
GRIPPER_STATUS_ERROR     = 3   # 错误

# ══════════════════════════════════════════════════════════════════
#   合体对齐 (/dock_align/*, 仅对抗区合体流程使用)
#   R2 主动视觉伺服对齐 R1, 检测 R1 上的 ArUco 标志做三自由度闭环控制
# ══════════════════════════════════════════════════════════════════

# /dock_align/enable (UInt8)
DOCK_ALIGN_DISABLE = 0
DOCK_ALIGN_ENABLE  = 1

# /dock_align/status (UInt8)
DOCK_ALIGN_STATUS_SEARCHING = 0   # 启动中, 搜索标志
DOCK_ALIGN_STATUS_ALIGNING  = 1   # 检测到标志, 伺服对齐中
DOCK_ALIGN_STATUS_DONE      = 2   # 滑动窗口内合格帧比例 ≥ OK_RATIO → 完成
DOCK_ALIGN_STATUS_FAILED    = 3   # 超时或持续无目标

DOCK_ALIGN_TIMEOUT_S      = 30.0  # 最大允许对齐时间 (s)
DOCK_ALIGN_CONFIRM_FRAMES = 15    # 滑动窗口帧数 N
DOCK_ALIGN_OK_RATIO       = 0.8   # 窗口内合格帧占比阈值 (允许 20% 抖动帧)

# ══════════════════════════════════════════════════════════════════
#   机械臂末端USB相机精对齐 (/fine_align/*, 仅梅林KFS拾取流程使用)
# ══════════════════════════════════════════════════════════════════
# 己方KFS颜色 (比赛前手动输入, 决定精对齐的滤色模式)
KFS_COLOR_BLUE = 0
KFS_COLOR_RED  = 1

# /fine_align/enable (UInt8): 0=关闭(释放USB相机) 1=启用-蓝色KFS 2=启用-红色KFS
FINE_ALIGN_DISABLE    = 0
FINE_ALIGN_ENABLE_BLUE = 1
FINE_ALIGN_ENABLE_RED  = 2

# /fine_align/status (UInt8): 0=对齐中 1=已居中完成 2=未检测到目标
FINE_ALIGN_STATUS_ALIGNING = 0
FINE_ALIGN_STATUS_DONE     = 1
FINE_ALIGN_STATUS_NO_TARGET = 2

# 精对齐超时兜底 (秒), 超时则放弃精对齐直接进入拾取
FINE_ALIGN_TIMEOUT_S = 15.0

# 精对齐底盘横向微调最大速度 (m/s), 对应triple_edge_align输出speed=100%时
FINE_ALIGN_MAX_SPEED_MPS = 0.05

# 精对齐"已居中"需连续确认的帧数 (防抖, 避免单帧抖动提前结束)
FINE_ALIGN_CONFIRM_FRAMES = 5

# ══════════════════════════════════════
#   梅林方块高度图（兼容老代码 meilin_navigator）
# ══════════════════════════════════════
BLOCK_HEIGHTS = {
    0:  0.00,
    1:  0.40,  2:  0.20,  3:  0.40,
    4:  0.60,  5:  0.40,  6:  0.20,
    7:  0.40,  8:  0.60,  9:  0.40,
    10: 0.20, 11: 0.40,  12: 0.20,
    99: 0.00,
}

BLOCK_TOP_SIZE     = 0.355
KFS_SIZE           = 0.350
BLOCK_SPACING      = 0.60

# ══════════════════════════════════════
#   IMU / 坡度 / 爬坡控制参数
# ══════════════════════════════════════
IMU_TOPIC         = '/camera/camera/imu'
IMU_ACCEL_TOPIC   = '/camera/camera/accel/sample'
IMU_GYRO_TOPIC    = '/camera/camera/gyro/sample'

IMU_COMP_ALPHA    = 0.96
GRAVITY           = 9.81
IMU_ACCEL_SIGN_X  = 1
IMU_ACCEL_SIGN_Y  = 1
IMU_ACCEL_SIGN_Z  = 1

SLOPE_DETECT_DEG       = 3.0
SLOPE_LEVEL_MILD       = 8.0
SLOPE_LEVEL_MODERATE   = 15.0
SLOPE_LEVEL_STEEP      = 25.0

SPEED_FACTOR_FLAT      = 1.0
SPEED_FACTOR_MILD      = 0.7
SPEED_FACTOR_MODERATE  = 0.45
SPEED_FACTOR_STEEP     = 0.25

TORQUE_FACTOR_FLAT     = 0.5
TORQUE_FACTOR_MILD     = 0.7
TORQUE_FACTOR_MODERATE = 0.85
TORQUE_FACTOR_STEEP    = 1.0

BRAKE_FACTOR_SLOPE     = 0.6

ENCODER_RATE_HZ        = 50
ODOM_DRIFT_FACTOR      = 0.02

# ══════════════════════════════════════
#   梅林方块导航参数（里程计辅助）
# ══════════════════════════════════════
BLOCK_CENTER_SPACING   = 0.60
BLOCK_ARRIVAL_ODOM_THR = 0.45
BLOCK_SETTLE_TIME      = 0.5
MEILIN_ENTRY_DIST      = 0.30

BLOCK_ODOM_DISTANCES = {
    'entry': 0.00,
    1:  0.60,  2:  0.60,  3:  0.60,
    4:  1.20,  5:  1.20,  6:  1.20,
    7:  1.80,  8:  1.80,  9:  1.80,
    10: 2.40, 11: 2.40,  12: 2.40,
    'exit': 3.00,
}

# ══════════════════════════════════════════════════════════════════
#   比赛航点坐标（左半场, game 坐标系）
#   坐标系: 原点 = 旧坐标系 (0, 12), new_x = -old_x, new_y = 12 - old_y
#   左右半场关于 x=0 对称, 右半场航点只需 x 取反
# ══════════════════════════════════════════════════════════════════
WAYPOINT_START         = {'x': -1.4,  'y': 0.4,   'yaw':  1.5708}   # 面向y正
WAYPOINT_WEAPON_RACK   = {'x': -0.65, 'y': 1.55,  'yaw':  0.0}      # 待标定
WAYPOINT_ASSEMBLY      = {'x':  -0.55, 'y': 0.4,   'yaw':  0.0}      # 待标定
WAYPOINT_MERLIN_ENTRY  = {'x':  -3.0,  'y': 2.0,   'yaw': 1.5708}

# ── 对抗区航点 (左半场, game坐标) ──
WAYPOINT_MERLIN_EXIT_GATHER = {'x': -3.0,  'y': 8.4,   'yaw':  1.5708}  # 梅林出口集合点
WAYPOINT_EXIT_MERLIN        = {'x': -5.4,  'y': 8.4,   'yaw':  1.5708}  # 出梅林点(左移2.4m)
WAYPOINT_CONFRONT_ENTRY     = {'x': -5.4,  'y': 11.6,  'yaw':  0.0}     # 对抗区入口(前进3.2m)
WAYPOINT_KFS_PLACE          = {'x': -0.56, 'y': 10.75, 'yaw':  0.0}     # KFS放置点
WAYPOINT_CONFRONT_WAIT      = {'x': -3.0,  'y': 11.6,  'yaw':  0.0}     # 等待合体点

# ══════════════════════════════════════════════════════════════════
#   比赛时间参数
# ══════════════════════════════════════════════════════════════════
MATCH_DURATION_S     = 240.0    # 比赛4分钟
MATCH_TIMEOUT_S      = 250.0    # 4分10秒后停止
PHASE_SWITCH_WAIT_S  = 1.0      # 阶段切换等待时间

# ══════════════════════════════════════════════════════════════════
#   动作组指令 (Jetson → STM32, /serial/action_group_cmd)
#   每个 ID 对应下位机一套完整动作序列
# ══════════════════════════════════════════════════════════════════
ACTION_NONE           = 0
ACTION_PICKUP_WEAPON  = 1   # 拾取武器端头
ACTION_RELEASE_WEAPON = 2   # 释放武器端头
ACTION_PICKUP_KFS     = 3   # 拾取KFS
ACTION_RELEASE_KFS    = 4   # 释放KFS
ACTION_PLACE_KFS      = 5   # 放置KFS (底盘抬升40cm + 放置动作组)
ACTION_LOCK_CHASSIS   = 6   # 底盘锁死 (到达组装点/合体点后通知下位机)
ACTION_ARM_LIFT_1     = 7   # 机械臂抬升1 (抓取比当前台阶高的KFS: 抬升+前伸+摄像头/吸盘转向下)
ACTION_ARM_LIFT_2     = 8   # 机械臂抬升2 (抓取比当前台阶低的KFS: 同上, 抬升幅度不同)

# ══════════════════════════════════════════════════════════════════
#   动作组反馈 (STM32 → Jetson, /feedback/action_group)
# ══════════════════════════════════════════════════════════════════
ACTION_STATUS_IDLE     = 0
ACTION_STATUS_RUNNING  = 1
ACTION_STATUS_DONE     = 2
ACTION_STATUS_FAILED   = 3

# ══════════════════════════════════════════════════════════════════
#   组装信号常量
# ══════════════════════════════════════════════════════════════════
ASSEMBLY_STATUS_NONE        = 0
ASSEMBLY_STATUS_IN_PROGRESS = 1
ASSEMBLY_STATUS_DONE        = 2
ASSEMBLY_STATUS_FAILED      = 3

# ══════════════════════════════════════════════════════════════════
#   R1 机器人通信信号
# ══════════════════════════════════════════════════════════════════
R1_SIGNAL_NONE          = 0
R1_SIGNAL_ASSEMBLY_DONE = 1
R1_SIGNAL_ENTER_MERLIN  = 2
R1_SIGNAL_MERGE         = 3   # 合体指令

# ══════════════════════════════════════════════════════════════════
#   比赛指令常量
# ══════════════════════════════════════════════════════════════════
GAME_CMD_NONE  = 0
GAME_CMD_RESET = 1

# ══════════════════════════════════════════════════════════════════
#   梅林爬升/下降指令（发送给下位机）
# ══════════════════════════════════════════════════════════════════
CLIMB_1    = 1    # 爬升20cm：2号方块入口 / 梅林内相邻高方块
CLIMB_2    = 2    # 爬升40cm：1,3号方块入口
DESCEND_1  = 3    # 下降20cm：10,12号出口 / 梅林内相邻矮方块
DESCEND_2  = 4    # 下降40cm：11号方块出口

CLIMB_TRIGGER_DIST_M = 0.40   # 距台阶边缘40cm时触发爬升/下降

ENTRY_CLIMB_CMD = {1: CLIMB_2, 2: CLIMB_1, 3: CLIMB_2}
EXIT_DESCEND_CMD = {10: DESCEND_1, 11: DESCEND_2, 12: DESCEND_1}

# ══════════════════════════════════════
#   兼容性参数（老代码引用，保留）
# ══════════════════════════════════════
WEAPON_RACK_POS    = (1.5, 0.0, 0.1)
WEAPON_GRAB_DIST   = 0.05
WUQI_CLASS_MAOJIAN = 2   # 矛尖→W_spear
WUQI_CLASS_QUAN    = 0   # 拳→W_punch
WUQI_CLASS_ZHANG   = 1   # 掌→W_palm
ASSEMBLY_WAIT_POS  = (0.8, 0.0, 0.0)
KFS_CLASS_R2_KFS   = 0
KFS_CLASS_FAKE_KFS = 1
KFS_CLASS_R1_KFS   = 2
R2_CONFIDENCE_MIN  = MEILIN_CONF_MIN

# ══════════════════════════════════════════════════════════════════
#   右半场坐标自动镜像 (在所有左半场常量定义完毕后执行)
#
#   单点航点 (WAYPOINT_*): x → -x, yaw → π - yaw (归一化到 (-π, π])
#
#   梅林方块 (BLOCK_CENTERS / MERLIN_ENTRY_CLIMB_POINTS /
#             MERLIN_EXIT_DESCEND_POINTS): 标签编号 1~12 顺序不变,
#   按 BLOCK_GRID 同行的列对称位置 (col → 2-col) 取该位置的左半场坐标
#   取反作为本编号的右半场坐标。
#   例: 左半场1号(x=-1.8,y=3.8, col0) → 右半场1号取同行col2(3号,x=-4.2)
#       取反 → (x=4.2, y=3.8)
#   BLOCK_GRID / 路径规划邻接关系完全不变
# ══════════════════════════════════════════════════════════════════
if FIELD_SIDE == 'right':
    def _mw(wp: dict) -> dict:
        """镜像单个航点: x 取反, yaw 关于 x=0 对称."""
        ny = math.pi - wp['yaw']
        if ny >  math.pi: ny -= 2 * math.pi
        if ny <= -math.pi: ny += 2 * math.pi
        return {'x': -wp['x'], 'y': wp['y'], 'yaw': round(ny, 6)}

    # ── 武馆 / 组装 / 梅林入口 ──
    WAYPOINT_START              = _mw(WAYPOINT_START)
    WAYPOINT_WEAPON_RACK        = _mw(WAYPOINT_WEAPON_RACK)
    WAYPOINT_ASSEMBLY           = _mw(WAYPOINT_ASSEMBLY)
    WAYPOINT_MERLIN_ENTRY       = _mw(WAYPOINT_MERLIN_ENTRY)

    # ── 对抗区 ──
    WAYPOINT_MERLIN_EXIT_GATHER = _mw(WAYPOINT_MERLIN_EXIT_GATHER)
    WAYPOINT_EXIT_MERLIN        = _mw(WAYPOINT_EXIT_MERLIN)
    WAYPOINT_CONFRONT_ENTRY     = _mw(WAYPOINT_CONFRONT_ENTRY)
    WAYPOINT_KFS_PLACE          = _mw(WAYPOINT_KFS_PLACE)
    WAYPOINT_CONFRONT_WAIT      = _mw(WAYPOINT_CONFRONT_WAIT)

    # ── 梅林方块: 标签不变, 按同行对称列取值后取反 ──
    _grid_to_id = {v: k for k, v in BLOCK_GRID.items()}

    def _mirror_block_id(bid: int) -> int:
        row, col = BLOCK_GRID[bid]
        return _grid_to_id[(row, 2 - col)]

    BLOCK_CENTERS = {
        k: (-BLOCK_CENTERS[_mirror_block_id(k)][0], v[1])
        for k, v in BLOCK_CENTERS.items()
    }

    MERLIN_ENTRY_CLIMB_POINTS = {
        k: _mw(MERLIN_ENTRY_CLIMB_POINTS[_mirror_block_id(k)])
        for k in MERLIN_ENTRY_CLIMB_POINTS
    }
    MERLIN_EXIT_DESCEND_POINTS = {
        k: _mw(MERLIN_EXIT_DESCEND_POINTS[_mirror_block_id(k)])
        for k in MERLIN_EXIT_DESCEND_POINTS
    }

    del _mw, _grid_to_id, _mirror_block_id