"""
config.py
所有需要按实际硬件调整的参数集中在这里
"""
import math

# ══════════════════════════════════════
#   TF 坐标系名称定义
# ══════════════════════════════════════
FRAME_BASE_LINK   = 'base_link'
FRAME_CAMERA      = 'camera_link'
FRAME_ARM_BASE    = 'arm_base_link'

# ══════════════════════════════════════
#   相机安装参数
# ══════════════════════════════════════
CAMERA_OFFSET_X   =  0.05
CAMERA_OFFSET_Y   =  0.0
CAMERA_OFFSET_Z   =  0.15
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
# ══════════════════════════════════════
WHEEL_DIAMETER_M  = 0.096
TRACK_WIDTH_M     = 0.28

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
MEILIN_TOTAL_TARGETS = 4

# 梅林到达拾取距离（米）
MEILIN_STOP_DIST = 0.12

# ══════════════════════════════════════════════════════════════════
#   梅林路径规划
# ══════════════════════════════════════════════════════════════════
# 三条预定义直线路径（方块编号1-12）
MEILIN_PATH_LEFT   = [1, 4, 7, 10]    # 左路
MEILIN_PATH_MID    = [2, 5, 8, 11]    # 中路（默认）
MEILIN_PATH_RIGHT  = [3, 6, 9, 12]    # 右路

MEILIN_DEFAULT_PATH = 'MID'  # 可选 'LEFT' / 'MID' / 'RIGHT'

# 绕道时原地旋转角度（度）
MEILIN_DETOUR_TURN_DEG = 90.0

# 入口方块（规则4.4.15）
MEILIN_ENTRY_BLOCKS = [1, 2, 3]

# 出口方块（规则4.4.19）
MEILIN_EXIT_BLOCKS  = [10, 11, 12]

# ══════════════════════════════════════
#   梅林方块高度图（按图册附录3）
# ══════════════════════════════════════
BLOCK_HEIGHTS = {
    0:  0.00,   # entry（伪方块，入口地面）
    1:  0.00,  2:  0.10,  3:  0.00,
    4:  0.20,  5:  0.30,  6:  0.20,
    7:  0.10,  8:  0.20,  9:  0.10,
    10: 0.00, 11: 0.10,  12: 0.00,
    99: 0.00,   # exit
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