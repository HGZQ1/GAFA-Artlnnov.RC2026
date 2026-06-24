"""
config.py
所有需要按实际硬件调整的参数集中在这里
"""
import math
import os as _os

# ╔══════════════════════════════════════════════════════════════════╗
#   ★ 快速调试区 — 最常调整的参数集中在此处 ★
#
#   视觉对齐:
#     ALIGN_THRESHOLD_DEG  — 对齐容差，越小越精准但越难到达
#     ALIGN_TURN_GAIN      — 转向速度增益，>1 转更快，<1 转更慢
#     CONFIRM_FRAMES       — 目标确认帧数，越大越稳但反应越慢
#     WUGUAN_CONF_MIN      — YOLO置信度阈值，太高会漏检
#
#   接近控制:
#     STOP_DISTANCE_M      — 停在距武器多远触发抓取 (m)
#     FORWARD_SPEED_GAIN   — 前进速度增益，>1 接近更快，<1 更慢
#
#   相机标定 (需与 URDF camera_x/y/z/rpy 保持一致):
#     CAMERA_OFFSET_*      — 相机相对 base_link 的安装位置 (m)
#     CAMERA_PITCH_RAD     — 相机俯仰角 (正=朝上, 负=朝下)
# ╚══════════════════════════════════════════════════════════════════╝
ALIGN_THRESHOLD_DEG  = 5.0     # 对齐角度容差 (度)
ALIGN_TURN_GAIN      = 1.0     # 转向速度增益 (1.0=直接用偏角, 0.5=减半)
STOP_DISTANCE_M      = 0.50    # 武馆停止距离 (m, 相机到武器)
FORWARD_SPEED_GAIN   = 1.0     # 前进速度增益
WUGUAN_CONF_MIN      = 0.70    # 武馆 YOLO 置信度最低阈值
WUGUAN_TOTAL_WEAPONS = 999     # 武馆拾取总数 (999=不限, 比赛改3)
CONFIRM_FRAMES       = 3       # 连续N帧确认目标锁定
TARGET_TIMEOUT_S     = 0.5     # 目标消失超过此时间视为丢失 (s)

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
FRAME_BASE_LINK    = 'base_link'
FRAME_CAMERA       = 'camera_link'          # D435i 主视觉相机
FRAME_WEAPON_ARM   = 'weapon_arm_base_link' # 武器头机械臂底座
FRAME_KFS_ARM      = 'arm_base_link'        # KFS机械臂底座
FRAME_KFS_DEPLOYED = 'arm_deployed_link'    # KFS机械臂展开末端
FRAME_USB_CAMERA   = 'usb_camera_link'      # USB精对齐相机（KFS臂末端）
FRAME_SUCTION      = 'suction_link'         # 吸盘（KFS臂末端）
FRAME_ARM_BASE     = FRAME_KFS_ARM          # 兼容旧代码 (tf_manager.py)

# ══════════════════════════════════════════════════════════════════
#   ★ 硬件安装参数 ★
#   权威来源: robot_params.yaml（唯一修改入口）
#   此处的值由 launch 文件自动注入 URDF xacro，同时供 Python
#   坐标变换（tf_manager.py fallback）和调试使用
#   修改后需同步: robot_params.yaml + 此处（两处保持一致）
# ══════════════════════════════════════════════════════════════════

# ── D435i 主视觉相机（武器头对齐用）──────────────────────────────
CAMERA_OFFSET_X   =  0.05    # 相对 base_link 前方偏移 (m)
CAMERA_OFFSET_Y   =  0.0     # 横向偏移 (m，正=左)
CAMERA_OFFSET_Z   =  0.45    # 高度 (m，从底盘底面算)
CAMERA_ROLL_RAD   =  0.0     # 滚转角 (rad)
CAMERA_PITCH_RAD  =  0.0     # 俯仰角 (rad，朝下为负，如-0.262≈-15°)
CAMERA_YAW_RAD    =  0.0     # 偏航角 (rad)

# ── 武器头机械臂底座 ──────────────────────────────────────────────
# ★ 待现场测量后修改，同步更新 robot_params.yaml ★
WEAPON_ARM_OFFSET_X   =  0.05   # 相对 base_link 偏移 (m)
WEAPON_ARM_OFFSET_Y   =  0.15
WEAPON_ARM_OFFSET_Z   =  0.30
WEAPON_ARM_ROLL_RAD   =  0.0
WEAPON_ARM_PITCH_RAD  =  0.0
WEAPON_ARM_YAW_RAD    =  0.0

# ── KFS机械臂底座 ─────────────────────────────────────────────────
# ★ 待现场测量后修改，同步更新 robot_params.yaml ★
KFS_ARM_OFFSET_X   =  0.0    # 相对 base_link 偏移 (m)
KFS_ARM_OFFSET_Y   =  0.0
KFS_ARM_OFFSET_Z   =  0.05
KFS_ARM_ROLL_RAD   =  0.0
KFS_ARM_PITCH_RAD  =  0.0
KFS_ARM_YAW_RAD    =  0.0

# ── KFS机械臂展开末端位置（相对 arm_base_link）────────────────────
# ★ 待实测机械臂展开尺寸后修改，同步更新 robot_params.yaml ★
KFS_DEPLOYED_OFFSET_X =  0.30   # 前伸距离 (m)
KFS_DEPLOYED_OFFSET_Y =  0.0
KFS_DEPLOYED_OFFSET_Z =  0.40   # 抬升高度 (m)

# ── USB相机（KFS臂末端，精对齐用，镜头朝下）──────────────────────
USB_CAMERA_OFFSET_X   =  0.0     # 相对 arm_deployed_link 偏移 (m)
USB_CAMERA_OFFSET_Y   =  0.0
USB_CAMERA_OFFSET_Z   =  0.0
USB_CAMERA_PITCH_RAD  =  1.5708  # π/2 rad，朝下安装

# ── 吸盘（KFS臂末端）────────────────────────────────────────────
# ★ 待现场测量吸盘相对USB相机的实际偏移后修改 ★
SUCTION_OFFSET_X   =  0.03   # 相对 arm_deployed_link 偏移 (m)
SUCTION_OFFSET_Y   =  0.0
SUCTION_OFFSET_Z   = -0.02   # 负值=低于末端横杆

# ── 兼容别名（tf_manager.py 使用 ARM_OFFSET_*，保留勿删）────────
ARM_OFFSET_X   = KFS_ARM_OFFSET_X
ARM_OFFSET_Y   = KFS_ARM_OFFSET_Y
ARM_OFFSET_Z   = KFS_ARM_OFFSET_Z
ARM_ROLL_RAD   = KFS_ARM_ROLL_RAD
ARM_PITCH_RAD  = KFS_ARM_PITCH_RAD
ARM_YAW_RAD    = KFS_ARM_YAW_RAD

# KFS臂底座相对相机的水平偏移（精对齐横向误差计算用）
ARM_LATERAL_FROM_CAMERA = KFS_ARM_OFFSET_Y - CAMERA_OFFSET_Y

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
ARRIVAL_THRESHOLD_M = 0.55    # planner 内部到达判定距离 (m)

# ══════════════════════════════════════
#   决策参数
# ══════════════════════════════════════
PICK_DURATION_S     = 10.0    # 拾取动作最长等待时间 (s)
ARRIVAL_SETTLE_S    = 1.0     # 到达后稳定等待时间 (s)
LOST_FRAMES         = 5       # 连续N帧未检测到视为目标丢失
MAX_JUMP_M          = 0.8     # 检测跳变过滤阈值 (m)

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
MERLIN_DEFAULT_ENTRY = 2  #默认入口方块

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
DOCK_ALIGN_DISABLE = 0   # 禁用对齐
DOCK_ALIGN_ENABLE  = 1   # 启用对齐

# /dock_align/status (UInt8)
DOCK_ALIGN_STATUS_SEARCHING = 0   # 启动中, 搜索标志
DOCK_ALIGN_STATUS_ALIGNING  = 1   # 检测到标志, 伺服对齐中
DOCK_ALIGN_STATUS_DONE      = 2   # 滑动窗口内合格帧比例 ≥ OK_RATIO → 完成
DOCK_ALIGN_STATUS_FAILED    = 3   # 超时或持续无目标

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

# 精对齐底盘横向微调最大速度 (m/s), 对应triple_edge_align输出speed=100%时
FINE_ALIGN_MAX_SPEED_MPS = 0.05

# 精对齐"已居中"需连续确认的帧数 (防抖, 避免单帧抖动提前结束)
FINE_ALIGN_CONFIRM_FRAMES = 5

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
BLOCK_CENTER_SPACING   = 0.60  # 方块中心间距
BLOCK_ARRIVAL_ODOM_THR = 0.45  # 方块到达里程计阈值
BLOCK_SETTLE_TIME      = 0.5  # 方块稳定时间
MEILIN_ENTRY_DIST      = 0.30  # 梅林入口距离（确定了爬升动作组会使机体前进多少再调）

BLOCK_ODOM_DISTANCES = {         
    'entry': 0.00,
    1:  0.60,  2:  0.60,  3:  0.60,
    4:  1.20,  5:  1.20,  6:  1.20,
    7:  1.80,  8:  1.80,  9:  1.80,
    10: 2.40, 11: 2.40,  12: 2.40,
    'exit': 3.00,
}
# 方块里程计距离表
# ══════════════════════════════════════════════════════════════════
#   ★ 比赛航点坐标（左半场，game 坐标系）★
#
#   坐标系说明:
#     原点  = 场地参考点（旧坐标系 x=0, y=12 处）
#     x 轴  = 水平横向，左半场 x < 0，右半场 x > 0
#     y 轴  = 纵深方向，出发区 y ≈ 0，终点区 y ≈ 12
#     yaw   = 绕z轴朝向角（弧度），π/2 = 朝y轴正方向，0 = 朝x轴正方向
#
#   半场规则:
#     左半场 (FIELD_SIDE='left') : 以下坐标直接使用，x均为负值
#     右半场 (FIELD_SIDE='right'): 文件末尾自动对 x 取反（x → -x）
#                                  yaw 自动镜像（π - yaw）
#     ★ 右半场只需修改本节中的数值，镜像由代码自动完成 ★
#
#   完整路径 (左半场):
#     出发区(-1.4, 0.4) ──前进1.15m──▶ 武器架(-0.65, 1.55)
#          └─ 视觉粗对齐 → 夹取武器端头
#     武器架 ──移动──▶ 组装点(-0.55, 0.4)
#          └─ 等待R1组装 → 松爪 → 等待"进入梅林"信号
#     组装点 ──转向后前进──▶ 梅林入口(-3.0, 2.0)
#          └─ 进入梅林决策子系统（穿越方块，拾取KFS）
#     梅林出口 ──集合──▶ 出口集合点(-3.0, 8.4)
#          └─ 向左移动2.4m ──▶ 出梅林点(-5.4, 8.4)
#          └─ 向前移动3.2m ──▶ 对抗区入口(-5.4, 11.6)
#          └─ 移动到KFS放置点(-0.56, 10.75) → 放置KFS
#          └─ 移动到等待合体点(-3.0, 11.6) → 等待R1合体指令
# ══════════════════════════════════════════════════════════════════

# ── 武馆区域 ──────────────────────────────────────────────────────
WAYPOINT_START = {'x': -1.4, 'y': 0.4, 'yaw': 1.5708}
# 左半场出发点，朝向y轴正方向（面向场地纵深）
# 路径：先向前（y+）行走1.15m，再向右（x+）行走0.75m，到达武器架

WAYPOINT_WEAPON_RACK = {'x': -0.65, 'y': 1.55, 'yaw': 0.0}
# 武器端头架前导航目标点，到达后激活视觉对齐（ALIGN_WEAPON阶段）
# D435i识别武器头做旋转粗对齐，对齐完成后前进到 STOP_DISTANCE_M 处停止
# ★ 待现场标定：用卷尺测量武器架正前方场地坐标后修改

WAYPOINT_ASSEMBLY = {'x': -0.55, 'y': 0.4, 'yaw': 0.0}
# 武器端头组装等待点（R2与R1机器人组装对接位置）
# 到达后发送底盘锁死（ACTION_LOCK_CHASSIS），等R1"组装完成"信号后松爪
# ★ 待现场标定：与R1实际对接位置测量后修改

# ── 梅林区域 ──────────────────────────────────────────────────────
WAYPOINT_MERLIN_ENTRY = {'x': -3.0, 'y': 2.0, 'yaw': 1.5708}
# 梅林入口导航目标，朝向y轴正方向（正面对着梅林入口方向）
# 到达后切换梅林决策系统，机器人在此处原地等待1s后执行爬升

# ── 对抗区域 ──────────────────────────────────────────────────────
WAYPOINT_MERLIN_EXIT_GATHER = {'x': -3.0, 'y': 8.4, 'yaw': 1.5708}
# 梅林出口集合点，从10/11/12号任意出口方块下来后先汇集至此
# 朝向y轴正方向，为向左横移至出梅林点做准备

WAYPOINT_EXIT_MERLIN = {'x': -5.4, 'y': 8.4, 'yaw': 1.5708}
# 出梅林点，从集合点向左（x减小方向）横移2.4m到达
# 朝向y轴正方向，为向前纵向进入对抗区做准备

WAYPOINT_CONFRONT_ENTRY = {'x': -5.4, 'y': 11.6, 'yaw': 0.0}
# 对抗区入口关键点，从出梅林点向前（y+）纵移3.2m到达
# 朝向x轴正方向（即朝向KFS放置点方向）

WAYPOINT_KFS_PLACE = {'x': -0.56, 'y': 10.75, 'yaw': 0.0}
# KFS放置点，底盘抬升40cm后执行放置动作组（ACTION_PLACE_KFS）
# ★ 待现场标定：与放置台实际位置对齐后修改

WAYPOINT_CONFRONT_WAIT = {'x': -3.0, 'y': 11.6, 'yaw': 0.0}
# 合体等待点，KFS放置完成后移动至此等待来自R1的合体指令
# 朝向x轴正方向，合体完成后底盘锁死不动

# ══════════════════════════════════════════════════════════════════
#   ★ 超时保护参数（所有时间单位: 秒）★
#
#   调试提示:
#     - 遇到某阶段卡住或跳过过快 → 先改本节对应的超时参数
#     - 减小超时值加快测试节奏，加大超时值给动作更多时间完成
# ══════════════════════════════════════════════════════════════════

# ── 比赛全局时间 ──────────────────────────────────────────────────
MATCH_DURATION_S     = 240.0   # 比赛总时长 4分钟 (240s)
MATCH_TIMEOUT_S      = 250.0   # 超过此时间后强制停机兜底 (4分10秒)
PHASE_SWITCH_WAIT_S  = 1.0     # 阶段切换后稳定等待时间，避免连续跳阶段 (s)

# ── 梅林区域等待时间 ─────────────────────────────────────────────
MERLIN_CLIMB_WAIT_S  = 3.0     # 发送爬升指令后等待下位机动作完成的最长时间 (s)
MERLIN_PICKUP_WAIT_S = 5.0     # 梅林内拾取KFS后等待确认的最长时间 (s)

# ── 视觉对齐超时 ─────────────────────────────────────────────────
FINE_ALIGN_TIMEOUT_S   = 15.0  # 精对齐（USB相机）超时兜底，超时则放弃精对齐直接进拾取 (s)
DOCK_ALIGN_TIMEOUT_S   = 30.0  # 合体对齐（ArUco深度相机）超时兜底 (s)

# ── KFS放置控制（game_controller._tick_place_kfs）──────────────
KFS_PLACE_STOP_WAIT_S = 0.3    # 到达KFS放置点后底盘停止稳定等待时间，避免平台晃动 (s)
KFS_PLACE_CMD_DELAY_S = 0.5    # 放置KFS动作组指令的发送窗口时长（窗口内重发确保到达） (s)

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
R1_SIGNAL_NONE          = 0   # 无信号
R1_SIGNAL_ASSEMBLY_DONE = 1   # 组装完成信号
R1_SIGNAL_ENTER_MERLIN  = 2   # 进入梅林信号
R1_SIGNAL_MERGE         = 3   # 合体指令

# ══════════════════════════════════════════════════════════════════
#   比赛指令常量
# ════════════════════════════════════╗
GAME_CMD_NONE  = 0  # 无指令
GAME_CMD_RESET = 1  # 复位指令

# ══════════════════════════════════════════════════════════════════
#   梅林爬升/下降指令（发送给下位机）
# ══════════════════════════════════════════════════════════════════
CLIMB_1    = 1    # 爬升20cm：2号方块入口 / 梅林内相邻高方块
CLIMB_2    = 2    # 爬升40cm：1,3号方块入口
DESCEND_1  = 3    # 下降20cm：10,12号出口 / 梅林内相邻矮方块
DESCEND_2  = 4    # 下降40cm：11号方块出口

CLIMB_TRIGGER_DIST_M = 0.40   # 距台阶边缘40cm时触发爬升/下降

ENTRY_CLIMB_CMD = {1: CLIMB_2, 2: CLIMB_1, 3: CLIMB_2}     # 梅林入口爬升指令
EXIT_DESCEND_CMD = {10: DESCEND_1, 11: DESCEND_2, 12: DESCEND_1}     # 梅林出口下降指令

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