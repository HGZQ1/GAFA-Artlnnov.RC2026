"""
game_controller.py
比赛全流程状态机 -- 武馆 + 梅林 导航决策联调

武馆流程:
  WAIT_INPUT -> WAIT_START -> NAV_TO_WEAPON -> ALIGN_WEAPON ->
  GRAB_WEAPON -> NAV_TO_ASSEMBLY -> WAIT_ASSEMBLY -> RELEASE_WEAPON ->
  WAIT_ENTER_MERLIN -> NAV_TO_MERLIN_ENTRY -> SWITCH_TO_MERLIN

梅林流程 (子状态机):
  M_INIT -> M_ENTRY_NAV -> M_ENTRY_CLIMB ->
  [M_ON_BLOCK -> M_PICKUP_NAV -> M_ALIGN_KFS -> M_ARM_LIFT -> M_FINE_ALIGN
   -> M_PICKUP_KFS ->]
  M_NAV_TO_TRIGGER -> M_SEND_CLIMB -> M_CLIMB_WAIT ->
  M_NAV_TO_CENTER -> M_ON_BLOCK -> ... ->
  M_EXIT_NAV -> M_EXIT_DESCEND -> M_DONE -> STOP

KFS拾取精对齐子流程:
  M_ALIGN_KFS  : D435I粗对齐(五状态机) -> decision_state_id==ARRIVED
  M_ARM_LIFT   : 根据KFS与当前方块高度差发送机械臂抬升信号(ACTION_ARM_LIFT_1/2),
                 等待下位机动作组完成反馈(关闭D435I视觉伺服, 抬升+前伸+摄像头/吸盘转向下)
  M_FINE_ALIGN : 启动机械臂末端USB相机精对齐(/fine_align/enable),
                 持续转发底盘横向微调指令(/fine_align/cmd -> /serial/chassis_cmd),
                 直到/fine_align/status==DONE 或超时(FINE_ALIGN_TIMEOUT_S)
  M_PICKUP_KFS : 关闭USB相机, 发送ACTION_PICKUP_KFS(吸盘拾取), 等待完成反馈
"""

import math
import os
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_ros

from std_msgs.msg import String, UInt8, Int8, Float32MultiArray
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from .robot_decision import RobotState
from . import meilin_path_planner as mpp
from .config import (
    FIELD_SIDE,
    WAYPOINT_START,
    WAYPOINT_WEAPON_RACK, WAYPOINT_ASSEMBLY, WAYPOINT_MERLIN_ENTRY,
    WAYPOINT_MERLIN_EXIT_GATHER, WAYPOINT_EXIT_MERLIN,
    WAYPOINT_CHONGWU_FINISH, WAYPOINT_JIUGONG_FINISH,
    WAYPOINT_CONFRONT_ENTRY, WAYPOINT_KFS_PLACE, WAYPOINT_CONFRONT_WAIT,
    MATCH_TIMEOUT_S, PHASE_SWITCH_WAIT_S, PRESTART_RELOCALIZE_TIMEOUT_S,
    ACTION_PICKUP_WEAPON, ACTION_RELEASE_WEAPON,
    ACTION_PICKUP_KFS, ACTION_RELEASE_KFS, ACTION_PLACE_KFS, ACTION_LOCK_CHASSIS, ACTION_MERGE,
    ACTION_STATUS_DONE,
    ACTION_ARM_LIFT_1, ACTION_ARM_LIFT_2,
    R1_SIGNAL_ENTER_MERLIN, R1_SIGNAL_PLACE_KFS,
    GAME_CMD_RESET,
    BLOCK_CENTERS, BLOCK_HEIGHTS,
    MERLIN_DEFAULT_ENTRY,
    MERLIN_ENTRY_CLIMB_POINTS, MERLIN_EXIT_DESCEND_POINTS,
    MERLIN_CLIMB_WAIT_S, MERLIN_PICKUP_WAIT_S,
    ENTRY_CLIMB_CMD, EXIT_DESCEND_CMD,
    DESCEND_1, DESCEND_2,
    CONFRONT_CLIMB_CMD, CONFRONT_CLIMB_EXIT_CMD,
    CONFRONT_CLIMB_FORWARD_SPEED_MPS,
    CONFRONT_CLIMB_STOP_DIST_M,
    CONFRONT_CLIMB_PITCH_DEG, CONFRONT_CLIMB_EXIT_DELTA_DEG,
    CONFRONT_CLIMB_EXIT_STABLE_S,
    CONFRONT_CLIMB_TIMEOUT_S, CONFRONT_CLIMB_STOP_S,
    CONFRONT_RELOCALIZE_TURN_DEG,
    CONFRONT_RELOCALIZE_TURN_MAX_DEG_S, CONFRONT_RELOCALIZE_TURN_MIN_DEG_S,
    CONFRONT_RELOCALIZE_TURN_KP, CONFRONT_RELOCALIZE_TURN_YAW_TOL_DEG,
    CONFRONT_RELOCALIZE_TURN_TIMEOUT_S,
    CONFRONT_NAV_MAX_LINEAR_SPEED_MPS, CONFRONT_NAV_MAX_ANGULAR_SPEED_RADPS,
    KFS_COLOR_BLUE, KFS_COLOR_RED,
    FINE_ALIGN_DISABLE, FINE_ALIGN_ENABLE_BLUE, FINE_ALIGN_ENABLE_RED,
    FINE_ALIGN_STATUS_ALIGNING, FINE_ALIGN_STATUS_DONE,
    DOCK_ALIGN_DISABLE, DOCK_ALIGN_ENABLE,
    DOCK_ALIGN_STATUS_SEARCHING, DOCK_ALIGN_STATUS_DONE, DOCK_ALIGN_STATUS_FAILED,
    DOCK_ALIGN_TIMEOUT_S,
    FINE_ALIGN_TIMEOUT_S,
    KFS_PLACE_STOP_WAIT_S, KFS_PLACE_CMD_DELAY_S, KFS_PLACE_ACTION_TIMEOUT_S,
    WEAPON_GRAB_TIMEOUT_S, RELEASE_WEAPON_WAIT_S,
)


# ═══════════════════════════════════════
#   Phase / Step 常量
# ═══════════════════════════════════════

class GamePhase:
    WAIT_INPUT          = 'WAIT_INPUT'
    WAIT_START          = 'WAIT_START'
    NAV_TO_WEAPON       = 'NAV_TO_WEAPON'
    ALIGN_WEAPON        = 'ALIGN_WEAPON'
    GRAB_WEAPON         = 'GRAB_WEAPON'
    NAV_TO_ASSEMBLY     = 'NAV_TO_ASSEMBLY'
    WAIT_ASSEMBLY       = 'WAIT_ASSEMBLY'
    RELEASE_WEAPON      = 'RELEASE_WEAPON'
    WAIT_ENTER_MERLIN   = 'WAIT_ENTER_MERLIN'
    NAV_TO_MERLIN_ENTRY = 'NAV_TO_MERLIN_ENTRY'
    SWITCH_TO_MERLIN    = 'SWITCH_TO_MERLIN'
    MERLIN_PHASE        = 'MERLIN_PHASE'
    # 对抗区
    NAV_TO_MERLIN_EXIT     = 'NAV_TO_MERLIN_EXIT'
    NAV_TO_EXIT_MERLIN     = 'NAV_TO_EXIT_MERLIN'
    NAV_TO_CONFRONT_ENTRY  = 'NAV_TO_CONFRONT_ENTRY'
    CONFRONT_CLIMB         = 'CONFRONT_CLIMB'
    CONFRONT_RELOCALIZE_TURN = 'CONFRONT_RELOCALIZE_TURN'
    NAV_TO_KFS_PLACE       = 'NAV_TO_KFS_PLACE'
    PLACE_KFS              = 'PLACE_KFS'
    NAV_TO_CONFRONT_WAIT   = 'NAV_TO_CONFRONT_WAIT'
    WAIT_MERGE             = 'WAIT_MERGE'
    MERGE_WITH_R1          = 'MERGE_WITH_R1'
    NAV_TO_FINISH          = 'NAV_TO_FINISH'
    STOP                   = 'STOP'


class MerlinStep:
    INIT           = 'M_INIT'
    ENTRY_NAV      = 'M_ENTRY_NAV'
    ENTRY_CLIMB    = 'M_ENTRY_CLIMB'
    ON_BLOCK       = 'M_ON_BLOCK'
    PICKUP_NAV     = 'M_PICKUP_NAV'
    ALIGN_KFS      = 'M_ALIGN_KFS'
    ARM_LIFT       = 'M_ARM_LIFT'
    FINE_ALIGN     = 'M_FINE_ALIGN'
    PICKUP_KFS     = 'M_PICKUP_KFS'
    NAV_TO_TRIGGER = 'M_NAV_TO_TRIGGER'
    DESCEND_TURN   = 'M_DESCEND_TURN'
    SEND_CLIMB     = 'M_SEND_CLIMB'
    CLIMB_WAIT     = 'M_CLIMB_WAIT'
    NAV_TO_CENTER  = 'M_NAV_TO_CENTER'
    EXIT_NAV       = 'M_EXIT_NAV'
    EXIT_DESCEND_TURN = 'M_EXIT_DESCEND_TURN'
    EXIT_DESCEND   = 'M_EXIT_DESCEND'
    DONE           = 'M_DONE'


GRAB_WEAPON_SETTLE_S = 0.2


class GameController(Node):

    def __init__(self):
        super().__init__('game_controller')

        # ── Game State ──
        self._phase = GamePhase.WAIT_INPUT
        self._phase_enter_time = time.time()
        self._match_start_time = None
        self._run_mode = 'full'
        self._match_timeout_s = MATCH_TIMEOUT_S
        self._prestart_reloc_sent = False
        self._prestart_reloc_done = False
        self._prestart_reloc_label = ''
        self._prestart_reloc_time = 0.0
        self._prestart_reloc_wait_logged = False
        self._prestart_reloc_timeout_logged = False

        # Pre-match input
        self._kfs_real_blocks = []
        self._kfs_fake_blocks = []
        self._kfs_color = KFS_COLOR_BLUE   # 己方KFS颜色, 决定精对齐滤色模式
        self._input_complete = False
        self._start_signal_received = False

        # Nav2 state
        self._nav_active    = False
        self._nav_done      = False
        self._nav_succeeded = False
        self._nav_goal_handle = None

        # Feedback
        self._action_status     = 0
        self._r1_signal         = 0
        self._decision_state_id = 0
        self._grab_action_sent  = False
        self._imu_pitch_deg     = 0.0
        self._imu_last_time     = 0.0
        self._confront_climb_base_pitch = 0.0
        self._confront_climb_peak_delta = 0.0
        self._confront_climb_exit_candidate_time = 0.0
        self._confront_reloc_turn_sent = False
        self._confront_reloc_turn_target_yaw = 0.0
        self._r1_place_signal_received = False
        self._place_kfs_sent = False
        self._place_kfs_sent_time = 0.0
        self._place_kfs_timeout_log_time = 0.0
        self._place_wait_logged = False
        self._dock_align_sent = False
        self._dock_align_status = DOCK_ALIGN_STATUS_SEARCHING
        self._dock_align_retry_time = 0.0
        self._merge_action_sent = False
        self._merge_action_done = False
        self._merge_wait_release_logged = False
        self._release_kfs_sent = False

        # ── Merlin State ──
        self._merlin_step       = MerlinStep.INIT
        self._merlin_step_time  = 0.0
        self._merlin_path       = []
        self._merlin_path_idx   = 0
        self._merlin_pickup     = {}   # {kfs_block: pickup_from_block}
        self._merlin_picked     = set()
        self._merlin_kfs_target = 0
        self._merlin_climb_sent = False
        self._merlin_pre_entry_pickups = []
        self._merlin_pre_entry_idx = 0
        self._merlin_pre_entry_active = False

        # 对抗区入口上坡状态
        self._confront_climb_sent = False
        self._confront_climb_exit_sent = False
        self._confront_climb_saw_slope = False
        self._confront_climb_stop_start = 0.0

        # M_ARM_LIFT / M_FINE_ALIGN 步骤标记与精对齐反馈
        self._arm_lift_sent     = False
        self._fine_align_sent   = False
        self._fine_align_status = FINE_ALIGN_STATUS_ALIGNING
        self._fine_align_cmd    = Twist()

        # 底盘锁死指令一次性发送标记 (随 phase 切换重置)
        self._lock_sent = False

        # ── TF (for position monitoring) ──
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Nav2 Action Client ──
        self._action_cb = MutuallyExclusiveCallbackGroup()
        self._nav_client = ActionClient(
            self, NavigateToPose, 'navigate_to_pose',
            callback_group=self._action_cb)


        # ── Publishers ──
        self._phase_pub      = self.create_publisher(String, '/game/phase', 10)
        self._action_pub     = self.create_publisher(UInt8,  '/serial/action_group_cmd', 10)
        self._chassis_pub    = self.create_publisher(Twist,  '/serial/chassis_cmd', 10)
        self._game_cmd_pub   = self.create_publisher(UInt8,  '/game/cmd', 10)
        self._kfs_config_pub = self.create_publisher(String, '/game/kfs_config', 10)
        self._meilin_cmd_pub = self.create_publisher(Twist,  '/serial/meilin_cmd', 10)
        self._confront_climb_pub = self.create_publisher(UInt8, '/serial/confront_climb_cmd', 10)
        self._fine_align_enable_pub = self.create_publisher(UInt8, '/fine_align/enable', 10)
        self._dock_align_enable_pub = self.create_publisher(UInt8, '/dock_align/enable', 10)
        self._relocalize_pub = self.create_publisher(String, '/relocalize/trigger', 10)
        self._model_switch_pub = self.create_publisher(String, '/vision/switch_model', 10)
        self._waypoint_speed_limit_pub = self.create_publisher(Twist, '/waypoint_nav/speed_limit', 10)
        self._waypoint_yaw_turn_dir_pub = self.create_publisher(
            Int8, '/waypoint_nav/yaw_turn_direction', 10)
        self._last_waypoint_yaw_turn_dir = 0

        # ── Subscribers ──
        self.create_subscription(UInt8,  '/feedback/action_group', self._on_action_fb, 10)
        self.create_subscription(UInt8,  '/game/start_signal',   self._on_start_signal, 10)
        self.create_subscription(String, '/game/kfs_input',      self._on_kfs_input, 10)
        self.create_subscription(UInt8,  '/game/r1_signal',      self._on_r1_signal, 10)
        self.create_subscription(Int8,   '/decision/state_id',   self._on_decision_state, 10)
        self.create_subscription(Twist,  '/cmd_vel',             self._on_cmd_vel, 10)
        self.create_subscription(String, '/waypoint_nav/status', self._on_nav_status_str, 10)
        self.create_subscription(Twist,  '/fine_align/cmd',       self._on_fine_align_cmd, 10)
        self.create_subscription(UInt8,  '/fine_align/status',    self._on_fine_align_status, 10)
        self.create_subscription(Float32MultiArray, '/imu/processed', self._on_imu_processed, 10)
        self.create_subscription(String, '/relocalize/status', self._on_relocalize_status, 10)
        self.create_subscription(UInt8, '/dock_align/status', self._on_dock_align_status, 10)

        self._nav_status_str = 'IDLE'

        # ── Main loop 10Hz ──
        self.create_timer(0.1, self._tick)

        # ── 状态摘要日志 2Hz ──
        self._last_cmd_vel = Twist()
        self.create_timer(0.5, self._print_status)

        # ── 区域单独测试 ──
        self.declare_parameter('test_area', '')
        self.declare_parameter('kfs_real', '5')
        self.declare_parameter('kfs_fake', '8')
        self.declare_parameter('kfs_color', 'blue')
        self.declare_parameter('match_timeout_s', MATCH_TIMEOUT_S)
        self._match_timeout_s = float(self.get_parameter('match_timeout_s').value)
        self._setup_test_area()

        # ── Terminal input ──
        if sys.stdin.isatty():
            threading.Thread(target=self._terminal_input_loop, daemon=True).start()
        else:
            self.get_logger().info(
                '非交互终端, 使用话题: /game/kfs_input, /game/start_signal')

        self.get_logger().info('比赛控制器已启动, 等待输入KFS标签...')

    # ════════════════════════════════════════
    #   区域单独测试: 跳过前置流程, 直接进入指定区域
    # ════════════════════════════════════════

    def _setup_test_area(self):
        """根据 test_area 参数, 跳过KFS输入/启动信号等前置流程,
        直接将状态机切到对应区域的起始阶段, 并自动配置KFS信息.

        机器人重定位由 launch 文件的 start_x/y/yaw (=该区域入口点游戏坐标)
        统一驱动 (waypoint_navigator.loc_offset_* 与
        map_relocalizer.prior_offset_*), 此处仅负责状态机跳转.
        """
        area = str(self.get_parameter('test_area').value).strip().lower()
        self._run_mode = area if area in (
            'full', 'chongwu', 'jiugong', 'weapon', 'weapon_align', 'merlin', 'confront'
        ) else 'full'

        if area in ('full', 'chongwu'):
            self._load_kfs_config_from_params()

        if area in ('chongwu', 'jiugong'):
            if area == 'jiugong':
                # 九宫藏宝从对抗区开始，不需要输入/规划梅林KFS。
                self._input_complete = True
            self.get_logger().info(
                f'===== 比赛子模式: {area} | 限时 {self._match_timeout_s:.0f}s =====')
            if area == 'chongwu':
                self.get_logger().info('流程: 武馆 -> 梅林 -> 完成停止点')
            else:
                self.get_logger().info('流程: 对抗区 -> 放置KFS -> 完成停止点/合体')
            return

        if area not in ('weapon', 'weapon_align', 'merlin', 'confront'):
            return

        self._kfs_real_blocks = self._parse_block_labels(
            str(self.get_parameter('kfs_real').value))
        self._kfs_fake_blocks = self._parse_block_labels(
            str(self.get_parameter('kfs_fake').value))
        color_str = str(self.get_parameter('kfs_color').value).strip().lower()
        self._kfs_color = KFS_COLOR_RED if color_str.startswith('r') else KFS_COLOR_BLUE

        self._input_complete = True
        self._start_signal_received = True
        self._match_start_time = time.time()
        self._reset_nav()

        now = time.time()
        self._phase_enter_time = now
        self._merlin_step_time = now

        if area == 'weapon':
            self._phase = GamePhase.NAV_TO_WEAPON
            entry_wp = WAYPOINT_START
        elif area == 'weapon_align':
            self._phase = GamePhase.ALIGN_WEAPON
            entry_wp = WAYPOINT_WEAPON_RACK
        elif area == 'merlin':
            self._phase = GamePhase.MERLIN_PHASE
            self._merlin_step = MerlinStep.INIT
            entry_wp = WAYPOINT_MERLIN_ENTRY
        else:  # confront
            self._phase = GamePhase.CONFRONT_CLIMB
            entry_wp = WAYPOINT_EXIT_MERLIN

        color_name = '红' if self._kfs_color == KFS_COLOR_RED else '蓝'
        self.get_logger().info(
            f'===== 区域测试模式: {area} =====')
        self.get_logger().info(
            f'真KFS={self._kfs_real_blocks} 假KFS={self._kfs_fake_blocks} '
            f'颜色={color_name}, 起始阶段={self._phase}')
        self.get_logger().info(
            f'请确认机器人已摆放在区域入口点 (重定位目标): '
            f"x={entry_wp['x']:.3f}, y={entry_wp['y']:.3f}, "
            f"yaw={entry_wp['yaw']:.4f}")

    def _load_kfs_config_from_params(self):
        self._kfs_real_blocks = self._parse_block_labels(
            str(self.get_parameter('kfs_real').value))
        self._kfs_fake_blocks = self._parse_block_labels(
            str(self.get_parameter('kfs_fake').value))
        color_str = str(self.get_parameter('kfs_color').value).strip().lower()
        self._kfs_color = KFS_COLOR_RED if color_str.startswith('r') else KFS_COLOR_BLUE
        if self._kfs_real_blocks:
            self._input_complete = True
            color_name = '红' if self._kfs_color == KFS_COLOR_RED else '蓝'
            self.get_logger().info(
                f'KFS参数输入: 真={self._kfs_real_blocks} 假={self._kfs_fake_blocks} '
                f'颜色={color_name}')

    # ════════════════════════════════════════
    #   Phase / Merlin Step Management
    # ════════════════════════════════════════

    def _set_phase(self, p: str):
        old = self._phase
        self._phase = p
        self._phase_enter_time = time.time()
        self._lock_sent = False
        if p == GamePhase.WAIT_START:
            self._prestart_reloc_sent = False
            self._prestart_reloc_done = False
            self._prestart_reloc_label = ''
            self._prestart_reloc_time = 0.0
            self._prestart_reloc_wait_logged = False
            self._prestart_reloc_timeout_logged = False
        if p == GamePhase.GRAB_WEAPON:
            self._grab_action_sent = False
        if p == GamePhase.PLACE_KFS:
            self._place_kfs_sent = False
            self._place_kfs_sent_time = 0.0
            self._place_kfs_timeout_log_time = 0.0
            self._place_wait_logged = False
        if p == GamePhase.WAIT_MERGE:
            self._dock_align_sent = False
            self._dock_align_status = DOCK_ALIGN_STATUS_SEARCHING
            self._dock_align_retry_time = 0.0
        if p == GamePhase.MERGE_WITH_R1:
            self._merge_action_sent = False
            self._merge_action_done = False
            self._merge_wait_release_logged = False
            self._release_kfs_sent = False
            self._r1_place_signal_received = False
        if p == GamePhase.CONFRONT_CLIMB:
            self._confront_climb_sent = False
            self._confront_climb_exit_sent = False
            self._confront_climb_saw_slope = False
            self._confront_climb_stop_start = 0.0
            self._confront_climb_base_pitch = self._imu_pitch_deg
            self._confront_climb_peak_delta = 0.0
            self._confront_climb_exit_candidate_time = 0.0
        if p == GamePhase.CONFRONT_RELOCALIZE_TURN:
            self._confront_reloc_turn_sent = False
            self._confront_reloc_turn_target_yaw = 0.0

        if p == GamePhase.NAV_TO_ASSEMBLY:
            direction = 1 if FIELD_SIDE == 'left' else -1
            self._set_waypoint_yaw_turn_direction(
                direction,
                '去武器头组装点: 左半场逆时针/右半场顺时针')
        elif old == GamePhase.NAV_TO_ASSEMBLY:
            self._set_waypoint_yaw_turn_direction(0, '离开武器头组装点导航')

        confront_limited = {
            GamePhase.NAV_TO_CONFRONT_ENTRY,
            GamePhase.CONFRONT_RELOCALIZE_TURN,
            GamePhase.NAV_TO_KFS_PLACE,
            GamePhase.PLACE_KFS,
            GamePhase.NAV_TO_CONFRONT_WAIT,
            GamePhase.WAIT_MERGE,
            GamePhase.MERGE_WITH_R1,
        }
        if self._run_mode in ('full', 'jiugong'):
            confront_limited.add(GamePhase.NAV_TO_FINISH)
        if p in confront_limited:
            self._set_confront_nav_speed_limit()
        elif old in confront_limited and p != GamePhase.CONFRONT_CLIMB:
            self._reset_nav_speed_limit()
        self.get_logger().info(f'[{old} -> {p}]')

    def _phase_elapsed(self) -> float:
        return time.time() - self._phase_enter_time

    def _match_elapsed(self) -> float:
        return time.time() - self._match_start_time if self._match_start_time else 0.0

    def _set_merlin_step(self, s: str):
        old = self._merlin_step
        self._merlin_step = s
        self._merlin_step_time = time.time()
        self._merlin_climb_sent = False
        self._arm_lift_sent = False
        self._fine_align_sent = False
        self.get_logger().info(f'  梅林 [{old} -> {s}]')

    def _merlin_step_elapsed(self) -> float:
        return time.time() - self._merlin_step_time

    def _trigger_relocalize(self, area_label: str):
        """到达区域入口点时触发一次地图重定位, 校正本区域内积累的SLAM漂移."""
        msg = String()
        msg.data = area_label
        self._relocalize_pub.publish(msg)
        self.get_logger().info(f'触发区域入口重定位: {area_label}')

    def _prestart_relocalize_label(self) -> str:
        return 'confront' if self._run_mode == 'jiugong' else 'weapon'

    def _trigger_prestart_relocalize_once(self):
        if self._prestart_reloc_sent:
            return
        label = self._prestart_relocalize_label()
        self._prestart_reloc_label = label
        self._prestart_reloc_sent = True
        self._prestart_reloc_done = False
        self._prestart_reloc_time = time.time()
        self._trigger_relocalize(label)
        if self._run_mode == 'jiugong':
            self.get_logger().info(
                '启动前重定位: 九宫藏宝使用 WAYPOINT_EXIT_MERLIN 作为先验起点')
        else:
            self.get_logger().info('启动前重定位: 使用比赛起点/武馆起点作为先验起点')

    def _prestart_relocalize_ready(self) -> bool:
        if self._prestart_reloc_done:
            return True
        elapsed = time.time() - self._prestart_reloc_time
        if elapsed >= PRESTART_RELOCALIZE_TIMEOUT_S:
            if not self._prestart_reloc_timeout_logged:
                self.get_logger().warn(
                    f'启动前重定位等待超时({elapsed:.1f}s), 继续响应启动信号')
                self._prestart_reloc_timeout_logged = True
            return True
        return False

    # ════════════════════════════════════════
    #   Terminal Input
    # ════════════════════════════════════════

    def _terminal_input_loop(self):
        try:
            print('\n========== RC2026 比赛控制器 ==========')
            self._read_kfs_from_terminal()
            self._wait_start_from_terminal()
        except (EOFError, KeyboardInterrupt):
            pass

    def _read_kfs_from_terminal(self):
        print('请输入真KFS所在台阶标签 (空格分隔, 1-12)')
        print('例: 5 8')
        while rclpy.ok() and not self._input_complete:
            try:
                line = input('真KFS> ').strip()
            except EOFError:
                return
            if not line:
                continue
            real = self._parse_block_labels(line)
            if not real:
                print('无效输入, 请输入1-12的数字')
                continue
            self._kfs_real_blocks = real
            print('请输入假KFS所在台阶标签 (空格分隔, 无则直接回车)')
            try:
                line = input('假KFS> ').strip()
            except EOFError:
                return
            self._kfs_fake_blocks = self._parse_block_labels(line) if line else []

            print('请输入己方KFS颜色 (b=蓝/r=红, 默认蓝, 用于机械臂精对齐滤色)')
            try:
                line = input('颜色> ').strip().lower()
            except EOFError:
                return
            self._kfs_color = KFS_COLOR_RED if line.startswith('r') else KFS_COLOR_BLUE

            self._input_complete = True
            color_name = '红' if self._kfs_color == KFS_COLOR_RED else '蓝'
            print(f'KFS配置: 真={self._kfs_real_blocks}  假={self._kfs_fake_blocks}  颜色={color_name}')

    def _wait_start_from_terminal(self):
        print('输入 q 启动比赛 (或按下启动按钮)')
        while rclpy.ok() and not self._start_signal_received:
            try:
                line = input('> ').strip()
            except EOFError:
                return
            if line.lower() == 'q':
                self._start_signal_received = True
                print('===== 比赛开始! =====')

    @staticmethod
    def _parse_block_labels(text: str) -> list:
        labels = []
        for tok in text.replace(',', ' ').split():
            try:
                n = int(tok)
                if 1 <= n <= 12:
                    labels.append(n)
            except ValueError:
                pass
        return labels

    # ════════════════════════════════════════
    #   Topic Callbacks
    # ════════════════════════════════════════

    def _on_action_fb(self, msg):     self._action_status = msg.data
    def _on_r1_signal(self, msg):
        self._r1_signal = msg.data
        if msg.data == R1_SIGNAL_ENTER_MERLIN:
            self.get_logger().info('收到 R1 进入梅林指令: data=2')
        elif msg.data == R1_SIGNAL_PLACE_KFS:
            merge_complete = (
                self._phase == GamePhase.MERGE_WITH_R1
                and (
                    self._merge_action_done
                    or (self._merge_action_sent and self._action_status == ACTION_STATUS_DONE)
                )
            )
            if merge_complete:
                self._merge_action_done = True
                self._r1_place_signal_received = True
                self.get_logger().info('收到 R1 合体后释放KFS指令: 将发送 ACTION_RELEASE_KFS=4')
            else:
                self.get_logger().info(
                    f'收到 R1 data=3, 但当前阶段={self._phase}, '
                    '该信号只在合体动作组完成后用于释放KFS')
    def _on_decision_state(self, msg): self._decision_state_id = msg.data
    def _on_cmd_vel(self, msg):      self._last_cmd_vel = msg
    def _on_nav_status_str(self, msg): self._nav_status_str = msg.data
    def _on_fine_align_cmd(self, msg):    self._fine_align_cmd = msg
    def _on_fine_align_status(self, msg): self._fine_align_status = msg.data
    def _on_dock_align_status(self, msg): self._dock_align_status = msg.data

    def _on_imu_processed(self, msg):
        if len(msg.data) >= 1:
            self._imu_pitch_deg = float(msg.data[0])
            self._imu_last_time = time.time()

    def _on_relocalize_status(self, msg):
        if not self._prestart_reloc_sent or self._prestart_reloc_done:
            return
        label = self._prestart_reloc_label
        if msg.data.startswith(f'{label}:') or msg.data.startswith('startup:'):
            self._prestart_reloc_done = True
            self.get_logger().info(f'启动前重定位完成: {msg.data}')

    def _on_start_signal(self, msg):
        if msg.data == 1 and not self._start_signal_received:
            self._start_signal_received = True
            self.get_logger().info('启动信号已接收: /game/start_signal data=1')

    def _on_kfs_input(self, msg):
        text = msg.data.strip()
        for seg in text.split():
            if seg.startswith('real:'):
                self._kfs_real_blocks = self._parse_block_labels(seg[5:])
            elif seg.startswith('fake:'):
                self._kfs_fake_blocks = self._parse_block_labels(seg[5:])
            elif seg.startswith('color:'):
                self._kfs_color = (KFS_COLOR_RED if seg[6:].strip().lower().startswith('r')
                                    else KFS_COLOR_BLUE)
        if self._kfs_real_blocks:
            self._input_complete = True
            color_name = '红' if self._kfs_color == KFS_COLOR_RED else '蓝'
            self.get_logger().info(
                f'KFS话题输入: 真={self._kfs_real_blocks} 假={self._kfs_fake_blocks} '
                f'颜色={color_name}')

    # ════════════════════════════════════════
    #   Nav2 Action Client
    # ════════════════════════════════════════

    def _try_send_nav(self, x, y, yaw) -> bool:
        if self._nav_active:
            return True
        if not self._nav_client.server_is_ready():
            return False
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.header.stamp = self.get_clock().now().to_msg()
        goal.pose.pose.position.x = float(x)
        goal.pose.pose.position.y = float(y)
        goal.pose.pose.orientation.z = math.sin(float(yaw) / 2.0)
        goal.pose.pose.orientation.w = math.cos(float(yaw) / 2.0)
        future = self._nav_client.send_goal_async(goal)
        future.add_done_callback(self._on_nav_goal_response)
        self._nav_active = True
        self.get_logger().info(f'Nav2 目标: ({x:.2f}, {y:.2f}, yaw={yaw:.3f})')
        return True

    def _on_nav_goal_response(self, future):
        handle = future.result()
        if not handle.accepted:
            self.get_logger().warn('Nav2 目标被拒绝')
            self._nav_active = False
            self._nav_done = True
            self._nav_succeeded = False
            return
        self._nav_goal_handle = handle
        handle.get_result_async().add_done_callback(self._on_nav_result)

    def _on_nav_result(self, future):
        self._nav_active = False
        self._nav_done = True
        self._nav_succeeded = (future.result().status == GoalStatus.STATUS_SUCCEEDED)

    def _cancel_nav(self):
        if self._nav_goal_handle is not None:
            self._nav_goal_handle.cancel_goal_async()
            self._nav_goal_handle = None
        self._nav_active = False

    def _reset_nav(self):
        self._nav_active = False
        self._nav_done = False
        self._nav_succeeded = False
        self._nav_goal_handle = None

    # ════════════════════════════════════════
    #   Commands
    # ════════════════════════════════════════

    def _send_action(self, action_id):
        if action_id == ACTION_PICKUP_KFS and self._phase != GamePhase.MERLIN_PHASE:
            self.get_logger().error(
                f'阻止异常动作组: ACTION_PICKUP_KFS=3 只能在 MERLIN_PHASE 发送, '
                f'当前阶段={self._phase}')
            return
        msg = UInt8(); msg.data = action_id
        self._action_pub.publish(msg)
        self._action_status = 0
        self.get_logger().info(f'发送动作组 action={action_id}')

    def _send_chassis_stop(self):
        self._chassis_pub.publish(Twist())

    def _send_confront_climb_drive(self):
        cmd = Twist()
        cmd.linear.x = float(CONFRONT_CLIMB_FORWARD_SPEED_MPS)
        cmd.linear.y = 0.0
        cmd.angular.z = 0.0
        self._chassis_pub.publish(cmd)

    def _set_confront_nav_speed_limit(self):
        msg = Twist()
        msg.linear.x = float(CONFRONT_NAV_MAX_LINEAR_SPEED_MPS)
        msg.angular.z = float(CONFRONT_NAV_MAX_ANGULAR_SPEED_RADPS)
        self._waypoint_speed_limit_pub.publish(msg)
        self.get_logger().info(
            '对抗区普通导航限速: '
            f'linear<={CONFRONT_NAV_MAX_LINEAR_SPEED_MPS:.2f}m/s, '
            f'angular<={CONFRONT_NAV_MAX_ANGULAR_SPEED_RADPS:.2f}rad/s')

    def _reset_nav_speed_limit(self):
        self._waypoint_speed_limit_pub.publish(Twist())
        self.get_logger().info('导航限速恢复默认')

    def _set_waypoint_yaw_turn_direction(self, direction: int, reason: str = ''):
        direction = 1 if direction > 0 else -1 if direction < 0 else 0
        if direction == self._last_waypoint_yaw_turn_dir:
            return
        msg = Int8()
        msg.data = direction
        self._waypoint_yaw_turn_dir_pub.publish(msg)
        self._last_waypoint_yaw_turn_dir = direction
        label = '逆时针' if direction > 0 else '顺时针' if direction < 0 else '最短路径'
        suffix = f' ({reason})' if reason else ''
        self.get_logger().info(f'导航yaw-only转向方向: {label}{suffix}')

    def _send_lock_chassis_once(self):
        """到达组装/合体点后, 一次性通知下位机底盘锁死."""
        if not self._lock_sent:
            self._send_action(ACTION_LOCK_CHASSIS)
            self._lock_sent = True
            self.get_logger().info('已通知下位机: 底盘锁死')

    def _send_game_cmd(self, cmd):
        msg = UInt8(); msg.data = cmd
        self._game_cmd_pub.publish(msg)

    def _send_meilin_climb(self, climb_mode, next_block=0, height_diff=0.0):
        msg = Twist()
        msg.linear.x  = float(next_block)
        msg.linear.y  = float(climb_mode)
        msg.angular.x = float(height_diff)
        self._meilin_cmd_pub.publish(msg)

    def _send_confront_climb(self, mode):
        msg = UInt8()
        msg.data = int(mode)
        self._confront_climb_pub.publish(msg)

    def _send_dock_align_enable(self, enable):
        msg = UInt8()
        msg.data = int(enable)
        self._dock_align_enable_pub.publish(msg)

    def _publish_kfs_config(self):
        msg = String()
        r = ','.join(str(b) for b in self._kfs_real_blocks)
        f = ','.join(str(b) for b in self._kfs_fake_blocks)
        msg.data = f'real:{r} fake:{f}'
        self._kfs_config_pub.publish(msg)

    # ════════════════════════════════════════
    #   TF Position Helper
    # ════════════════════════════════════════

    def _get_robot_xy(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            return t.transform.translation.x, t.transform.translation.y
        except Exception:
            return None, None

    def _get_robot_yaw(self):
        try:
            t = self._tf_buffer.lookup_transform('map', 'base_link', rclpy.time.Time())
            q = t.transform.rotation
            import math
            return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                              1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        except Exception:
            return None

    # ════════════════════════════════════════
    #   状态摘要日志 (2Hz)
    # ════════════════════════════════════════

    def _print_status(self):
        if self._phase in (GamePhase.WAIT_INPUT, GamePhase.STOP):
            return

        # 位置
        rx, ry = self._get_robot_xy()
        ryaw = self._get_robot_yaw()
        if rx is not None:
            pos_str = f'map({rx:+.2f},{ry:+.2f},yaw={math.degrees(ryaw or 0):+.1f})'
        else:
            pos_str = 'map(?,?)'

        # 阶段
        phase = self._phase
        if phase == GamePhase.MERLIN_PHASE:
            phase = f'MERLIN/{self._merlin_step}'

        # cmd_vel
        cv = self._last_cmd_vel
        vel_str = f'vx={cv.linear.x:+.2f} vy={cv.linear.y:+.2f} w={cv.angular.z:+.2f}'

        # 比赛时间
        elapsed = f'{self._match_elapsed():.0f}s' if self._match_start_time else '--'

        # 反馈状态
        fb_parts = []
        if self._action_status:    fb_parts.append(f'act={self._action_status}')
        if self._r1_signal:        fb_parts.append(f'r1={self._r1_signal}')
        if self._decision_state_id: fb_parts.append(f'vis={self._decision_state_id}')
        if self._merlin_step == MerlinStep.FINE_ALIGN:
            fb_parts.append(f'falign={self._fine_align_status}')
        if self._phase == GamePhase.WAIT_MERGE:
            fb_parts.append(f'dock={self._dock_align_status}')
        if self._phase == GamePhase.CONFRONT_CLIMB:
            climb_delta = abs(self._imu_pitch_deg - self._confront_climb_base_pitch)
            fb_parts.append(
                f'pitch={self._imu_pitch_deg:+.1f}deg d={climb_delta:.1f}deg')
        fb_str = ' '.join(fb_parts) if fb_parts else '-'

        self.get_logger().info(
            f'[{phase}] {pos_str} | nav={self._nav_status_str} | '
            f'{vel_str} | fb: {fb_str} | t={elapsed}')

    # ════════════════════════════════════════
    #   Main State Machine Tick (10Hz)
    # ════════════════════════════════════════

    def _tick(self):
        if (self._match_start_time is not None
                and self._phase != GamePhase.STOP
                and self._match_elapsed() >= self._match_timeout_s):
            self._on_timeout()
            return

        phase = self._phase

        if   phase == GamePhase.WAIT_INPUT:          self._tick_wait_input()
        elif phase == GamePhase.WAIT_START:           self._tick_wait_start()
        elif phase == GamePhase.NAV_TO_WEAPON:        self._tick_nav_to(WAYPOINT_WEAPON_RACK, GamePhase.ALIGN_WEAPON)
        elif phase == GamePhase.ALIGN_WEAPON:         self._tick_align_weapon()
        elif phase == GamePhase.GRAB_WEAPON:          self._tick_grab_weapon()
        elif phase == GamePhase.NAV_TO_ASSEMBLY:      self._tick_nav_to(WAYPOINT_ASSEMBLY, GamePhase.WAIT_ASSEMBLY)
        elif phase == GamePhase.WAIT_ASSEMBLY:        self._tick_wait_assembly()
        elif phase == GamePhase.RELEASE_WEAPON:       self._tick_release_weapon()
        elif phase == GamePhase.WAIT_ENTER_MERLIN:    self._tick_wait_enter_merlin()
        elif phase == GamePhase.NAV_TO_MERLIN_ENTRY:  self._tick_nav_to(WAYPOINT_MERLIN_ENTRY, GamePhase.SWITCH_TO_MERLIN)
        elif phase == GamePhase.SWITCH_TO_MERLIN:     self._tick_switch_to_merlin()
        elif phase == GamePhase.MERLIN_PHASE:         self._tick_merlin()
        # 对抗区
        elif phase == GamePhase.NAV_TO_MERLIN_EXIT:    self._tick_nav_to(WAYPOINT_MERLIN_EXIT_GATHER, GamePhase.NAV_TO_EXIT_MERLIN)
        elif phase == GamePhase.NAV_TO_EXIT_MERLIN:    self._tick_nav_to(WAYPOINT_EXIT_MERLIN, GamePhase.CONFRONT_CLIMB)
        elif phase == GamePhase.NAV_TO_CONFRONT_ENTRY: self._tick_confront_entry()
        elif phase == GamePhase.CONFRONT_CLIMB:        self._tick_confront_climb()
        elif phase == GamePhase.CONFRONT_RELOCALIZE_TURN: self._tick_confront_relocalize_turn()
        elif phase == GamePhase.NAV_TO_KFS_PLACE:      self._tick_nav_to(WAYPOINT_KFS_PLACE, GamePhase.PLACE_KFS)
        elif phase == GamePhase.PLACE_KFS:             self._tick_place_kfs()
        elif phase == GamePhase.NAV_TO_CONFRONT_WAIT:  self._tick_nav_to(WAYPOINT_CONFRONT_WAIT, GamePhase.WAIT_MERGE)
        elif phase == GamePhase.WAIT_MERGE:            self._tick_wait_merge()
        elif phase == GamePhase.MERGE_WITH_R1:         self._tick_merge_with_r1()
        elif phase == GamePhase.NAV_TO_FINISH:         self._tick_nav_to_finish()
        elif phase == GamePhase.STOP:                  self._send_chassis_stop()

        # 发布有效阶段 (梅林视觉对齐时覆盖为 ALIGN_KFS)
        pub_phase = self._phase
        if (self._phase == GamePhase.MERLIN_PHASE
                and self._merlin_step == MerlinStep.ALIGN_KFS):
            pub_phase = 'ALIGN_KFS'
        msg = String(); msg.data = pub_phase
        self._phase_pub.publish(msg)

    # ── 武馆各阶段 tick ──

    def _tick_wait_input(self):
        if self._input_complete:
            self._publish_kfs_config()
            self._set_phase(GamePhase.WAIT_START)

    def _tick_wait_start(self):
        self._trigger_prestart_relocalize_once()
        if self._start_signal_received:
            if not self._prestart_relocalize_ready():
                if not self._prestart_reloc_wait_logged:
                    self.get_logger().info('已收到启动信号, 等待启动前重定位完成...')
                    self._prestart_reloc_wait_logged = True
                return
            self._match_start_time = time.time()
            self._reset_nav()
            if self._run_mode == 'jiugong':
                self._set_phase(GamePhase.CONFRONT_CLIMB)
            else:
                self._set_phase(GamePhase.NAV_TO_WEAPON)

    def _tick_nav_to(self, wp, next_phase):
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(wp['x'], wp['y'], wp['yaw'])
        elif self._nav_done:
            self._reset_nav()
            if next_phase == GamePhase.SWITCH_TO_MERLIN:
                self._trigger_relocalize('merlin')
            self._set_phase(next_phase)

    def _tick_align_weapon(self):
        if self._decision_state_id == RobotState.ARRIVED.value:
            self._send_chassis_stop()
            self._set_phase(GamePhase.GRAB_WEAPON)

    def _tick_grab_weapon(self):
        self._send_chassis_stop()
        if not self._grab_action_sent:
            if self._phase_elapsed() < GRAB_WEAPON_SETTLE_S:
                return
            self._send_chassis_stop()
            self._send_action(ACTION_PICKUP_WEAPON)
            self._grab_action_sent = True
            self.get_logger().info('GRAB_WEAPON: 已停车并退出视觉对齐, 发送拾取武器动作')
            return

        if self._action_status == ACTION_STATUS_DONE:
            self._reset_nav()
            self._send_chassis_stop()
            if self._run_mode == 'weapon_align':
                self.get_logger().info('weapon_align测试: 拾取动作完成，停止在GRAB_WEAPON后')
                self._set_phase(GamePhase.STOP)
            else:
                self._set_phase(GamePhase.NAV_TO_ASSEMBLY)
        elif self._phase_elapsed() >= WEAPON_GRAB_TIMEOUT_S:
            self.get_logger().warn(
                f'抓取武器超时 ({WEAPON_GRAB_TIMEOUT_S}s)，'
                f'{"测试停止" if self._run_mode == "weapon_align" else "强制跳转至组装点"}')
            self._reset_nav()
            self._send_chassis_stop()
            if self._run_mode == 'weapon_align':
                self._set_phase(GamePhase.STOP)
            else:
                self._set_phase(GamePhase.NAV_TO_ASSEMBLY)

    def _tick_wait_assembly(self):
        self._send_lock_chassis_once()
        self._send_chassis_stop()
        if self._r1_signal == R1_SIGNAL_ENTER_MERLIN:
            # "进入梅林"信号同时视为组装完成，提前触发模型切换并行加载 kfs.pt
            self._switch_vision_model('kfs.pt')
            self._send_chassis_stop()
            self._send_action(ACTION_RELEASE_WEAPON)
            self._set_phase(GamePhase.RELEASE_WEAPON)

    def _tick_release_weapon(self):
        self._send_chassis_stop()
        if self._phase_elapsed() >= RELEASE_WEAPON_WAIT_S:
            self._reset_nav()
            self._send_chassis_stop()
            self._set_phase(GamePhase.NAV_TO_MERLIN_ENTRY)

    def _tick_wait_enter_merlin(self):
        self._send_chassis_stop()
        if self._r1_signal == R1_SIGNAL_ENTER_MERLIN:
            self._switch_vision_model('kfs.pt')
            self._reset_nav()
            self._send_chassis_stop()
            self._set_phase(GamePhase.NAV_TO_MERLIN_ENTRY)

    # ════════════════════════════════════════
    #   YOLO 模型热切换
    # ════════════════════════════════════════

    _WEIGHTS_DIR = os.path.join(
        os.path.expanduser('~'),
        'GAFA-Artlnnov.RC2026-main', 'ros2_vision_project',
        'ros2_ws', 'src', 'vision_detector', 'weights',
    )

    def _switch_vision_model(self, model_name: str):
        """发布 /vision/switch_model 话题触发模型切换，detector_node 后台加载。"""
        if not model_name.endswith('.pt'):
            model_name += '.pt'
        model_path = os.path.join(self._WEIGHTS_DIR, model_name)
        if not os.path.exists(model_path):
            self.get_logger().error(
                f'[模型切换] 文件不存在: {model_path}')
            return
        msg = String()
        msg.data = model_path
        self._model_switch_pub.publish(msg)
        self.get_logger().info(f'[模型切换] 已发布切换请求 → {model_name}')

    def _tick_switch_to_merlin(self):
        self._send_chassis_stop()
        if self._phase_elapsed() >= PHASE_SWITCH_WAIT_S:
            self._merlin_step = MerlinStep.INIT
            self._set_phase(GamePhase.MERLIN_PHASE)

    # ════════════════════════════════════════
    #   梅林子状态机
    # ════════════════════════════════════════

    def _tick_merlin(self):
        s = self._merlin_step

        if   s == MerlinStep.INIT:           self._m_init()
        elif s == MerlinStep.ENTRY_NAV:      self._m_entry_nav()
        elif s == MerlinStep.ENTRY_CLIMB:    self._m_entry_climb()
        elif s == MerlinStep.ON_BLOCK:       self._m_on_block()
        elif s == MerlinStep.PICKUP_NAV:     self._m_pickup_nav()
        elif s == MerlinStep.ALIGN_KFS:      self._m_align_kfs()
        elif s == MerlinStep.ARM_LIFT:       self._m_arm_lift()
        elif s == MerlinStep.FINE_ALIGN:     self._m_fine_align()
        elif s == MerlinStep.PICKUP_KFS:     self._m_pickup_kfs()
        elif s == MerlinStep.NAV_TO_TRIGGER: self._m_nav_to_trigger()
        elif s == MerlinStep.DESCEND_TURN:   self._m_descend_turn()
        elif s == MerlinStep.SEND_CLIMB:     self._m_send_climb()
        elif s == MerlinStep.CLIMB_WAIT:     self._m_climb_wait()
        elif s == MerlinStep.NAV_TO_CENTER:  self._m_nav_to_center()
        elif s == MerlinStep.EXIT_NAV:       self._m_exit_nav()
        elif s == MerlinStep.EXIT_DESCEND_TURN: self._m_exit_descend_turn()
        elif s == MerlinStep.EXIT_DESCEND:   self._m_exit_descend()
        elif s == MerlinStep.DONE:           self._m_done()

    @staticmethod
    def _normalize_yaw(yaw: float) -> float:
        while yaw > math.pi:
            yaw -= 2.0 * math.pi
        while yaw <= -math.pi:
            yaw += 2.0 * math.pi
        return yaw

    @staticmethod
    def _is_descend_cmd(cmd: int) -> bool:
        return int(cmd) in (DESCEND_1, DESCEND_2)

    def _rear_facing_yaw(self, target_yaw: float) -> float:
        """返回让车尾朝向目标方向的车体yaw。"""
        return self._normalize_yaw(target_yaw + math.pi)

    # ── M_INIT: 规划路径 ──

    def _m_init(self):
        # 无论通过何种路径进入梅林（全流程/test_area=merlin），都在此触发模型切换
        # 全流程下 _tick_wait_enter_merlin 已提前触发，此处是兜底
        self._switch_vision_model('kfs.pt')

        pre_entry_pickups = mpp.get_pre_entry_pickups(self._kfs_real_blocks)
        if pre_entry_pickups:
            # 若入口KFS不止一个，先依次在场外入口点拾取，最后从最后一个入口进入梅林。
            # 单入口KFS是比赛常见情况，也是4.0.md新增规则的核心场景。
            entry = pre_entry_pickups[-1]
            if len(pre_entry_pickups) > 1:
                self.get_logger().warn(
                    f'检测到多个入口KFS {pre_entry_pickups}, 将依次入口前拾取, '
                    f'最后从方块{entry}进入梅林')
        else:
            entry = MERLIN_DEFAULT_ENTRY

        path = mpp.plan_path(entry, self._kfs_real_blocks, self._kfs_fake_blocks)
        if not path:
            self.get_logger().error('梅林路径规划失败! 无可行路径')
            self._set_phase(GamePhase.STOP)
            return
        self._merlin_path = path
        self._merlin_path_idx = 0
        in_merlin_kfs = [
            block for block in self._kfs_real_blocks
            if block not in pre_entry_pickups
        ]
        self._merlin_pickup = mpp.get_pickup_info(path, in_merlin_kfs)
        self._merlin_picked = set()
        self._merlin_pre_entry_pickups = pre_entry_pickups
        self._merlin_pre_entry_idx = 0
        self._merlin_pre_entry_active = False
        self.get_logger().info(f'梅林路径: {path}')
        self.get_logger().info(f'KFS拾取: {self._merlin_pickup}')
        if self._merlin_pre_entry_pickups:
            self.get_logger().info(
                f'入口前拾取KFS: {self._merlin_pre_entry_pickups} '
                f'(先对齐拾取, 再入口爬升)')
        self._reset_nav()
        self._set_merlin_step(MerlinStep.ENTRY_NAV)

    # ── M_ENTRY_NAV: 导航到入口爬升点 ──

    def _m_entry_nav(self):
        pre_entry_kfs = self._next_pre_entry_pickup()
        entry = pre_entry_kfs if pre_entry_kfs is not None else self._merlin_path[0]
        wp = MERLIN_ENTRY_CLIMB_POINTS[entry]
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(wp['x'], wp['y'], wp['yaw'])
        elif self._nav_done:
            self._reset_nav()
            self._send_chassis_stop()
            if pre_entry_kfs is not None:
                self._merlin_kfs_target = pre_entry_kfs
                self._merlin_pre_entry_active = True
                self._decision_state_id = 0
                self.get_logger().info(
                    f'入口前拾取: 已到达方块{pre_entry_kfs}入口点, '
                    f'开始KFS对齐/拾取')
                self._set_merlin_step(MerlinStep.ALIGN_KFS)
            else:
                self._set_merlin_step(MerlinStep.ENTRY_CLIMB)

    # ── M_ENTRY_CLIMB: 发送入口爬升指令, 等待完成 ──

    def _m_entry_climb(self):
        if not self._merlin_climb_sent:
            entry = self._merlin_path[0]
            cmd = ENTRY_CLIMB_CMD[entry]
            self._send_meilin_climb(cmd, next_block=entry)
            self._merlin_climb_sent = True
            self.get_logger().info(f'入口爬升: 方块{entry}, 指令={cmd}')
        if self._merlin_step_elapsed() >= MERLIN_CLIMB_WAIT_S:
            self._merlin_path_idx = 0
            self._set_merlin_step(MerlinStep.ON_BLOCK)

    # ── M_ON_BLOCK: 到达方块, 判断拾取/下一步 ──

    def _m_on_block(self):
        idx = self._merlin_path_idx
        cur = self._merlin_path[idx]
        self.get_logger().info(f'当前方块: {cur} (path[{idx}])')

        # 检查是否需要拾取KFS
        kfs = self._merlin_need_pickup(cur)
        if kfs is not None:
            self._merlin_kfs_target = kfs
            self._reset_nav()
            self._gripper_status = 0
            self._set_merlin_step(MerlinStep.PICKUP_NAV)
            return

        # 检查是否为最后一个方块 (出口)
        if idx + 1 >= len(self._merlin_path):
            self._reset_nav()
            self._set_merlin_step(MerlinStep.EXIT_NAV)
            return

        # 前往下一方块的触发点
        self._reset_nav()
        self._set_merlin_step(MerlinStep.NAV_TO_TRIGGER)

    def _merlin_need_pickup(self, block) -> int:
        """如果当前方块需要执行KFS拾取, 返回KFS方块标签, 否则 None."""
        for kfs, pickup_from in self._merlin_pickup.items():
            if pickup_from == block and kfs not in self._merlin_picked:
                return kfs
        return None

    def _next_pre_entry_pickup(self):
        """返回下一个需要在入口爬升前拾取的KFS标签；没有则返回None."""
        while self._merlin_pre_entry_idx < len(self._merlin_pre_entry_pickups):
            kfs = self._merlin_pre_entry_pickups[self._merlin_pre_entry_idx]
            if kfs not in self._merlin_picked:
                return kfs
            self._merlin_pre_entry_idx += 1
        return None

    # ── M_PICKUP_NAV: 导航到拾取/触发点 (面向KFS方块) ──

    def _m_pickup_nav(self):
        cur = self._merlin_path[self._merlin_path_idx]
        kfs = self._merlin_kfs_target
        px, py, yaw = mpp.compute_trigger_point(cur, kfs)
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(px, py, yaw)
        elif self._nav_done:
            self._reset_nav()
            self._send_chassis_stop()
            self._decision_state_id = 0
            self._set_merlin_step(MerlinStep.ALIGN_KFS)

    # ── M_ALIGN_KFS: D435I粗对齐 (processor_node 执行) ──

    def _m_align_kfs(self):
        if self._decision_state_id == RobotState.ARRIVED.value:
            self._send_chassis_stop()
            self._set_merlin_step(MerlinStep.ARM_LIFT)
            self.get_logger().info(f'KFS {self._merlin_kfs_target} 粗对齐完成, 发送机械臂抬升信号')

    # ── M_ARM_LIFT: 机械臂抬升+前伸 (下位机动作组, 完成后关闭D435I视觉伺服) ──

    def _m_arm_lift(self):
        if not self._arm_lift_sent:
            kfs = self._merlin_kfs_target
            if self._merlin_pre_entry_active:
                cur = 'entry_ground'
                cur_label = '入口地面'
                h_cur = 0.0
            else:
                cur = self._merlin_path[self._merlin_path_idx]
                cur_label = f'方块{cur}'
                h_cur = BLOCK_HEIGHTS.get(cur, 0.0)
            h_kfs = BLOCK_HEIGHTS.get(kfs, 0.0)
            # 抬升1: 抓取比当前台阶高的物体; 抬升2: 抓取比当前台阶低的物体
            action = ACTION_ARM_LIFT_1 if h_kfs > h_cur else ACTION_ARM_LIFT_2
            self._send_action(action)
            self._arm_lift_sent = True
            self.get_logger().info(
                f'机械臂抬升: {cur_label}(h={h_cur:.0f}mm) -> KFS{kfs}(h={h_kfs:.0f}mm) '
                f'=> action={action}')

        if self._action_status == ACTION_STATUS_DONE:
            self._set_merlin_step(MerlinStep.FINE_ALIGN)

    # ── M_FINE_ALIGN: USB相机精对齐 (转发底盘微调指令直到居中) ──

    def _m_fine_align(self):
        if not self._fine_align_sent:
            enable = (FINE_ALIGN_ENABLE_RED if self._kfs_color == KFS_COLOR_RED
                      else FINE_ALIGN_ENABLE_BLUE)
            msg = UInt8(); msg.data = enable
            self._fine_align_enable_pub.publish(msg)
            self._fine_align_sent = True
            self._fine_align_status = FINE_ALIGN_STATUS_ALIGNING
            self.get_logger().info(
                f'精对齐启动: KFS{self._merlin_kfs_target} '
                f'颜色={"红" if self._kfs_color == KFS_COLOR_RED else "蓝"}')

        timeout = self._merlin_step_elapsed() >= FINE_ALIGN_TIMEOUT_S
        if self._fine_align_status == FINE_ALIGN_STATUS_DONE or timeout:
            if timeout:
                self.get_logger().warn(f'精对齐超时({FINE_ALIGN_TIMEOUT_S:.0f}s), 直接拾取')
            else:
                self.get_logger().info(f'KFS {self._merlin_kfs_target} 精对齐完成, 发送拾取指令')
            self._send_chassis_stop()
            off = UInt8(); off.data = FINE_ALIGN_DISABLE
            self._fine_align_enable_pub.publish(off)
            self._send_action(ACTION_PICKUP_KFS)
            self._set_merlin_step(MerlinStep.PICKUP_KFS)
            return

        # 居中前持续把精对齐微调指令转发到底盘
        self._chassis_pub.publish(self._fine_align_cmd)

    # ── M_PICKUP_KFS: 等待拾取反馈 ──

    def _m_pickup_kfs(self):
        if self._action_status == ACTION_STATUS_DONE:
            self.get_logger().info(f'KFS {self._merlin_kfs_target} 拾取成功')
            self._finish_kfs_pickup()
        elif self._merlin_step_elapsed() >= MERLIN_PICKUP_WAIT_S:
            self.get_logger().warn(f'KFS {self._merlin_kfs_target} 拾取超时, 跳过')
            self._finish_kfs_pickup()

    def _finish_kfs_pickup(self):
        """统一处理KFS拾取完成/超时后的状态跳转."""
        self._merlin_picked.add(self._merlin_kfs_target)

        if self._merlin_pre_entry_active:
            finished = self._merlin_kfs_target
            self._merlin_pre_entry_active = False
            self._merlin_pre_entry_idx += 1
            self._reset_nav()

            if self._next_pre_entry_pickup() is not None:
                self._set_merlin_step(MerlinStep.ENTRY_NAV)
            elif finished == self._merlin_path[0]:
                self._set_merlin_step(MerlinStep.ENTRY_CLIMB)
            else:
                self._set_merlin_step(MerlinStep.ENTRY_NAV)
            return

        # 拾取后已在触发点位置, 判断是否需要爬升到KFS方块
        idx = self._merlin_path_idx
        if idx + 1 < len(self._merlin_path) and self._merlin_path[idx + 1] == self._merlin_kfs_target:
            self._reset_nav()
            cmd = mpp.get_transition_climb(
                self._merlin_path[idx],
                self._merlin_path[idx + 1])
            next_step = (MerlinStep.DESCEND_TURN if self._is_descend_cmd(cmd)
                         else MerlinStep.SEND_CLIMB)
            self._set_merlin_step(next_step)
        else:
            self._reset_nav()
            self._set_merlin_step(MerlinStep.NAV_TO_TRIGGER)

    # ── M_NAV_TO_TRIGGER: 导航到爬升/下降触发点 ──

    def _m_nav_to_trigger(self):
        idx = self._merlin_path_idx
        cur = self._merlin_path[idx]
        nxt = self._merlin_path[idx + 1]
        px, py, yaw = mpp.compute_trigger_point(cur, nxt)
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(px, py, yaw)
        elif self._nav_done:
            self._reset_nav()
            cmd = mpp.get_transition_climb(cur, nxt)
            next_step = (MerlinStep.DESCEND_TURN if self._is_descend_cmd(cmd)
                         else MerlinStep.SEND_CLIMB)
            self._set_merlin_step(next_step)

    # ── M_DESCEND_TURN: 下降前原地转向, 让车尾朝向目标方块 ──

    def _m_descend_turn(self):
        idx = self._merlin_path_idx
        cur = self._merlin_path[idx]
        nxt = self._merlin_path[idx + 1]
        px, py, target_yaw = mpp.compute_trigger_point(cur, nxt)
        turn_yaw = self._rear_facing_yaw(target_yaw)

        if not self._nav_active and not self._nav_done:
            self.get_logger().info(
                f'下降前转向: 方块{cur}->{nxt}, 车尾朝向目标方块, '
                f'yaw={math.degrees(turn_yaw):+.1f}°')
            self._try_send_nav(px, py, turn_yaw)
        elif self._nav_done:
            self._reset_nav()
            self._set_merlin_step(MerlinStep.SEND_CLIMB)

    # ── M_SEND_CLIMB: 发送爬升/下降指令 ──

    def _m_send_climb(self):
        idx = self._merlin_path_idx
        cur = self._merlin_path[idx]
        nxt = self._merlin_path[idx + 1]
        cmd = mpp.get_transition_climb(cur, nxt)
        if cmd != 0:
            h_diff = mpp.get_transition_height_diff_mm(cur, nxt)
            self._send_meilin_climb(cmd, next_block=nxt, height_diff=h_diff)
            self.get_logger().info(
                f'方块{cur}->{nxt}: 爬升指令={cmd}, 高度差={h_diff:+.0f}mm')
        self._set_merlin_step(MerlinStep.CLIMB_WAIT)

    # ── M_CLIMB_WAIT: 等待爬升完成 ──

    def _m_climb_wait(self):
        if self._merlin_step_elapsed() >= MERLIN_CLIMB_WAIT_S:
            self._reset_nav()
            self._set_merlin_step(MerlinStep.NAV_TO_CENTER)

    # ── M_NAV_TO_CENTER: 导航到下一方块中心 ──

    def _m_nav_to_center(self):
        nxt = self._merlin_path[self._merlin_path_idx + 1]
        cx, cy = BLOCK_CENTERS[nxt]
        yaw = -1.5708  # 默认朝向出口方向 (y 递减)
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(cx, cy, yaw)
        elif self._nav_done:
            self._merlin_path_idx += 1
            self._reset_nav()
            self._set_merlin_step(MerlinStep.ON_BLOCK)

    # ── M_EXIT_NAV: 导航到出口下降点 ──

    def _m_exit_nav(self):
        exit_block = self._merlin_path[-1]
        cx, cy = BLOCK_CENTERS[exit_block]
        yaw = -1.5708
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(cx, cy, yaw)
        elif self._nav_done:
            self._reset_nav()
            self._set_merlin_step(MerlinStep.EXIT_DESCEND_TURN)

    # ── M_EXIT_DESCEND_TURN: 出口下降前转向, 让车尾朝向落地点 ──

    def _m_exit_descend_turn(self):
        exit_block = self._merlin_path[-1]
        cx, cy = BLOCK_CENTERS[exit_block]
        wp = MERLIN_EXIT_DESCEND_POINTS[exit_block]
        target_yaw = math.atan2(wp['y'] - cy, wp['x'] - cx)
        turn_yaw = self._rear_facing_yaw(target_yaw)

        if not self._nav_active and not self._nav_done:
            self.get_logger().info(
                f'出口下降前转向: 方块{exit_block}, 车尾朝向落地点, '
                f'yaw={math.degrees(turn_yaw):+.1f}°')
            self._try_send_nav(cx, cy, turn_yaw)
        elif self._nav_done:
            self._reset_nav()
            self._set_merlin_step(MerlinStep.EXIT_DESCEND)

    # ── M_EXIT_DESCEND: 发送出口下降指令 ──

    def _m_exit_descend(self):
        if not self._merlin_climb_sent:
            exit_block = self._merlin_path[-1]
            cmd = EXIT_DESCEND_CMD.get(exit_block, 3)
            self._send_meilin_climb(cmd, next_block=0)
            self._merlin_climb_sent = True
            self.get_logger().info(f'出口下降: 方块{exit_block}, 指令={cmd}')
        if self._merlin_step_elapsed() >= MERLIN_CLIMB_WAIT_S:
            self._set_merlin_step(MerlinStep.DONE)

    # ── M_DONE ──

    def _m_done(self):
        self.get_logger().info(
            f'梅林穿越完成, 已拾取KFS: {self._merlin_picked}')
        self._switch_vision_model('best.pt')
        self._reset_nav()
        if self._run_mode == 'chongwu':
            self._set_phase(GamePhase.NAV_TO_FINISH)
        else:
            self._set_phase(GamePhase.NAV_TO_MERLIN_EXIT)

    def _finish_waypoint(self):
        return (WAYPOINT_JIUGONG_FINISH if self._run_mode == 'jiugong'
                else WAYPOINT_CHONGWU_FINISH)

    def _finish_next_phase(self):
        return GamePhase.WAIT_MERGE if self._run_mode == 'jiugong' else GamePhase.STOP

    def _tick_nav_to_finish(self):
        wp = self._finish_waypoint()
        self._tick_nav_to(wp, self._finish_next_phase())

    # ════════════════════════════════════════
    #   对抗区
    # ════════════════════════════════════════

    def _tick_confront_entry(self):
        """导航到对抗区入口。该点已在坡顶, 到达后直接进入KFS放置导航。"""
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(
                WAYPOINT_CONFRONT_ENTRY['x'],
                WAYPOINT_CONFRONT_ENTRY['y'],
                WAYPOINT_CONFRONT_ENTRY['yaw'])
        elif self._nav_done:
            self._reset_nav()
            self._send_chassis_stop()
            self._set_phase(GamePhase.CONFRONT_RELOCALIZE_TURN)

    def _tick_confront_climb(self):
        """对抗区入口上坡: 下位机接管爬坡PID, 上位机用D435i pitch判定退出."""
        now = time.time()
        pitch_delta = abs(self._imu_pitch_deg - self._confront_climb_base_pitch)
        self._confront_climb_peak_delta = max(
            self._confront_climb_peak_delta,
            pitch_delta)
        pitch_return_delta = self._confront_climb_peak_delta - pitch_delta

        if not self._confront_climb_sent:
            self._send_chassis_stop()
            self._send_confront_climb(CONFRONT_CLIMB_CMD)
            if CONFRONT_CLIMB_FORWARD_SPEED_MPS > 0.0:
                self._send_confront_climb_drive()
            self._confront_climb_sent = True
            self.get_logger().info(
                '对抗区入口上坡: 已发送上坡指令 '
                f'mode={CONFRONT_CLIMB_CMD}, '
                f'前进速度={CONFRONT_CLIMB_FORWARD_SPEED_MPS:.2f}m/s, '
                f"停止点=({WAYPOINT_CONFRONT_ENTRY['x']:.2f},{WAYPOINT_CONFRONT_ENTRY['y']:.2f}), "
                f'半径<={CONFRONT_CLIMB_STOP_DIST_M:.2f}m, '
                f'基准pitch={self._confront_climb_base_pitch:+.1f}deg, '
                f'上坡触发变化>={CONFRONT_CLIMB_PITCH_DEG:.1f}deg, '
                f'退出回落变化>={CONFRONT_CLIMB_EXIT_DELTA_DEG:.1f}deg, '
                f'稳定={CONFRONT_CLIMB_EXIT_STABLE_S:.1f}s, '
                f'超时={CONFRONT_CLIMB_TIMEOUT_S:.1f}s')
            if self._imu_last_time <= 0.0 or now - self._imu_last_time > 1.0:
                self.get_logger().warn(
                    '对抗区入口上坡: /imu/processed 近1秒无数据, '
                    '将依靠超时退出爬坡')
            return

        if pitch_delta >= CONFRONT_CLIMB_PITCH_DEG:
            self._confront_climb_saw_slope = True

        rx, ry = self._get_robot_xy()
        reached_stop_point = False
        if rx is not None:
            stop_dist = math.hypot(
                rx - WAYPOINT_CONFRONT_ENTRY['x'],
                ry - WAYPOINT_CONFRONT_ENTRY['y'])
            reached_stop_point = stop_dist <= CONFRONT_CLIMB_STOP_DIST_M

        returned_after_slope = (
            self._confront_climb_saw_slope
            and pitch_return_delta >= CONFRONT_CLIMB_EXIT_DELTA_DEG
        )
        if returned_after_slope:
            if self._confront_climb_exit_candidate_time <= 0.0:
                self._confront_climb_exit_candidate_time = now
        else:
            self._confront_climb_exit_candidate_time = 0.0
        level_after_slope = (
            self._confront_climb_exit_candidate_time > 0.0
            and now - self._confront_climb_exit_candidate_time >= CONFRONT_CLIMB_EXIT_STABLE_S
        )
        timeout = self._phase_elapsed() >= CONFRONT_CLIMB_TIMEOUT_S

        if (not self._confront_climb_exit_sent
                and (reached_stop_point or level_after_slope or timeout)):
            self._send_confront_climb(CONFRONT_CLIMB_EXIT_CMD)
            self._send_chassis_stop()
            self._confront_climb_exit_sent = True
            self._confront_climb_stop_start = now
            if reached_stop_point:
                reason = '到达停止爬坡点'
            elif level_after_slope:
                reason = (
                    f'pitch从坡段峰值回落{pitch_return_delta:.1f}deg'
                    f'并稳定{CONFRONT_CLIMB_EXIT_STABLE_S:.1f}s')
            else:
                reason = '爬坡超时'
            self.get_logger().info(
                f'对抗区入口上坡退出: {reason}, '
                f'pitch={self._imu_pitch_deg:+.1f}deg, '
                f'base={self._confront_climb_base_pitch:+.1f}deg, '
                f'delta={pitch_delta:.1f}deg, peak={self._confront_climb_peak_delta:.1f}deg; '
                f'已发送退出爬坡 mode={CONFRONT_CLIMB_EXIT_CMD}, '
                f'开始{CONFRONT_CLIMB_STOP_S:.1f}s停车稳定')
            return

        if self._confront_climb_exit_sent:
            self._send_chassis_stop()
            if now - self._confront_climb_stop_start >= CONFRONT_CLIMB_STOP_S:
                self._reset_nav()
                self._set_phase(GamePhase.CONFRONT_RELOCALIZE_TURN)
            return

        if CONFRONT_CLIMB_FORWARD_SPEED_MPS > 0.0:
            self._send_confront_climb_drive()

    def _tick_confront_relocalize_turn(self):
        """对抗区入口重定位前先转向九宫格方向，提高ICP特征匹配质量."""
        ryaw = self._get_robot_yaw()
        if ryaw is None:
            self._send_chassis_stop()
            self.get_logger().warn(
                '对抗区入口重定位前转向: 当前yaw不可用, 直接触发confront重定位')
            self._trigger_relocalize('confront')
            self._set_phase(GamePhase.NAV_TO_KFS_PLACE)
            return

        if not self._confront_reloc_turn_sent:
            turn_sign = -1.0 if FIELD_SIDE == 'left' else 1.0
            turn_rad = math.radians(CONFRONT_RELOCALIZE_TURN_DEG) * turn_sign
            target_yaw = self._normalize_yaw(ryaw + turn_rad)
            self._confront_reloc_turn_target_yaw = target_yaw
            self._confront_reloc_turn_sent = True
            direction = '顺时针' if turn_sign < 0.0 else '逆时针'
            self.get_logger().info(
                f'对抗区入口重定位前纯原地转向: 半场={FIELD_SIDE}, {direction}'
                f'{CONFRONT_RELOCALIZE_TURN_DEG:.0f}deg, '
                f'当前yaw={math.degrees(ryaw):+.1f}deg, '
                f'目标yaw={math.degrees(target_yaw):+.1f}deg; '
                'linear=0, 转向完成后再触发confront重定位')

        yaw_err = self._normalize_yaw(self._confront_reloc_turn_target_yaw - ryaw)
        yaw_err_deg = math.degrees(yaw_err)
        done = abs(yaw_err_deg) <= CONFRONT_RELOCALIZE_TURN_YAW_TOL_DEG
        timeout = self._phase_elapsed() >= CONFRONT_RELOCALIZE_TURN_TIMEOUT_S

        if done or timeout:
            self._send_chassis_stop()
            reason = '到达目标角度' if done else '转向超时'
            self.get_logger().info(
                f'对抗区入口重定位前转向完成: {reason}, '
                f'当前yaw={math.degrees(ryaw):+.1f}deg, '
                f'目标yaw={math.degrees(self._confront_reloc_turn_target_yaw):+.1f}deg, '
                f'误差={yaw_err_deg:+.1f}deg, '
                '开始触发confront重定位')
            self._trigger_relocalize('confront')
            self._set_phase(GamePhase.NAV_TO_KFS_PLACE)
            return

        w_deg_s = CONFRONT_RELOCALIZE_TURN_KP * yaw_err_deg
        w_deg_s = max(-CONFRONT_RELOCALIZE_TURN_MAX_DEG_S,
                      min(CONFRONT_RELOCALIZE_TURN_MAX_DEG_S, w_deg_s))
        if abs(w_deg_s) < CONFRONT_RELOCALIZE_TURN_MIN_DEG_S:
            w_deg_s = math.copysign(CONFRONT_RELOCALIZE_TURN_MIN_DEG_S, yaw_err_deg)

        cmd = Twist()
        cmd.linear.x = 0.0
        cmd.linear.y = 0.0
        cmd.angular.z = w_deg_s
        self._chassis_pub.publish(cmd)

    def _advance_after_place_kfs(self):
        self._reset_nav()
        if self._run_mode == 'jiugong':
            self._set_phase(GamePhase.NAV_TO_FINISH)
        else:
            self._set_phase(GamePhase.NAV_TO_CONFRONT_WAIT)

    def _tick_place_kfs(self):
        """到达放置点后, R2自主发送放置KFS动作组指令."""
        if self._phase_elapsed() < KFS_PLACE_STOP_WAIT_S:
            self._send_chassis_stop()
            return

        self._send_chassis_stop()
        place_elapsed_s = self._phase_elapsed()
        place_timeout = place_elapsed_s >= KFS_PLACE_ACTION_TIMEOUT_S

        if not self._place_kfs_sent:
            self._send_action(ACTION_PLACE_KFS)
            self._place_kfs_sent = True
            self._place_kfs_sent_time = time.time()
            self._place_kfs_timeout_log_time = self._place_kfs_sent_time
            self.get_logger().info(
                '到达KFS放置点, R2自主发送放置KFS动作组 ACTION_PLACE_KFS=5 '
                '(抬升40cm + 放置), '
                f'超时保护={KFS_PLACE_ACTION_TIMEOUT_S:.1f}s')
            return

        if self._action_status == ACTION_STATUS_DONE:
            self.get_logger().info('KFS放置完成')
            self._advance_after_place_kfs()
            return

        if place_timeout:
            self.get_logger().warn(
                f'PLACE_KFS阶段超时 ({KFS_PLACE_ACTION_TIMEOUT_S:.1f}s), '
                '未收到放置动作组完成反馈, 继续执行后续合体等待流程')
            self._advance_after_place_kfs()
            return

        action_wait_s = time.time() - self._place_kfs_sent_time
        if time.time() - self._place_kfs_timeout_log_time >= 5.0:
            self._place_kfs_timeout_log_time = time.time()
            self.get_logger().info(
                f'等待KFS放置动作组完成: action_wait={action_wait_s:.1f}s, '
                f'place_elapsed={place_elapsed_s:.1f}/{KFS_PLACE_ACTION_TIMEOUT_S:.1f}s, '
                f'action_status={self._action_status}')

    def _tick_wait_merge(self):
        """在对抗区等待点, 启用D435i ArUco合体对齐, 对齐完成后进入合体动作."""
        if not self._dock_align_sent:
            self._send_chassis_stop()
            self._dock_align_status = DOCK_ALIGN_STATUS_SEARCHING
            self._send_dock_align_enable(DOCK_ALIGN_ENABLE)
            self._dock_align_sent = True
            self._dock_align_retry_time = time.time()
            self.get_logger().info('WAIT_MERGE: 已启用D435i ArUco合体对齐')
            return

        if self._dock_align_status == DOCK_ALIGN_STATUS_DONE:
            self._send_dock_align_enable(DOCK_ALIGN_DISABLE)
            self._send_chassis_stop()
            self.get_logger().info('Aruco合体对齐完成, 准备发送合体动作组')
            self._set_phase(GamePhase.MERGE_WITH_R1)
            return

        failed = self._dock_align_status == DOCK_ALIGN_STATUS_FAILED
        timeout = (time.time() - self._dock_align_retry_time) >= DOCK_ALIGN_TIMEOUT_S
        if failed or timeout:
            self._send_dock_align_enable(DOCK_ALIGN_DISABLE)
            self._send_chassis_stop()
            self._dock_align_sent = False
            self._dock_align_status = DOCK_ALIGN_STATUS_SEARCHING
            reason = 'dock_align状态失败' if failed else '等待超时'
            self.get_logger().warn(f'WAIT_MERGE: {reason}, 重新搜索ArUco')

    def _tick_merge_with_r1(self):
        """合体: 先发送合体动作组, 完成后等待R1指令释放KFS."""
        self._send_chassis_stop()
        if not self._merge_action_sent:
            self._send_action(ACTION_MERGE)
            self._merge_action_sent = True
            self.get_logger().info(f'发送合体动作组 action={ACTION_MERGE}')
            return

        if not self._merge_action_done:
            if self._action_status != ACTION_STATUS_DONE:
                return
            self._merge_action_done = True
            self._action_status = 0
            self.get_logger().info('合体动作组完成')
            return

        if not self._r1_place_signal_received:
            if not self._merge_wait_release_logged:
                self.get_logger().info('等待 R1 合体后释放KFS指令(data=3)')
                self._merge_wait_release_logged = True
            return

        if not self._release_kfs_sent:
            self._send_action(ACTION_RELEASE_KFS)
            self._release_kfs_sent = True
            self.get_logger().info(f'收到R1释放KFS指令, 发送 ACTION_RELEASE_KFS={ACTION_RELEASE_KFS}')
            return

        if self._action_status == ACTION_STATUS_DONE:
            self.get_logger().info('合体后释放KFS完成')
            self._set_phase(GamePhase.STOP)

    # ════════════════════════════════════════
    #   Timeout
    # ════════════════════════════════════════

    def _on_timeout(self):
        self.get_logger().warn(
            f'比赛超时 ({self._match_elapsed():.1f}s >= {self._match_timeout_s:.1f}s)')
        self._cancel_nav()
        self._send_chassis_stop()
        if self._run_mode in ('chongwu', 'jiugong'):
            self.get_logger().warn('子模式限时到达: 仅停车停止, 不发送复位指令')
        else:
            self._send_game_cmd(GAME_CMD_RESET)
        self._set_phase(GamePhase.STOP)


def main(args=None):
    rclpy.init(args=args)
    node = GameController()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
