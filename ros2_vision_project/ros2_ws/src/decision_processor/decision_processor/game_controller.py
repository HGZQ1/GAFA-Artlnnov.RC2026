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
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionClient
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import tf2_ros

from std_msgs.msg import String, UInt8, Int8
from geometry_msgs.msg import Twist, PoseStamped
from nav2_msgs.action import NavigateToPose
from action_msgs.msg import GoalStatus

from .robot_decision import RobotState
from . import meilin_path_planner as mpp
from .config import (
    WAYPOINT_START,
    WAYPOINT_WEAPON_RACK, WAYPOINT_ASSEMBLY, WAYPOINT_MERLIN_ENTRY,
    WAYPOINT_MERLIN_EXIT_GATHER, WAYPOINT_EXIT_MERLIN,
    WAYPOINT_CONFRONT_ENTRY, WAYPOINT_KFS_PLACE, WAYPOINT_CONFRONT_WAIT,
    MATCH_TIMEOUT_S, PHASE_SWITCH_WAIT_S,
    ACTION_PICKUP_WEAPON, ACTION_RELEASE_WEAPON,
    ACTION_PICKUP_KFS, ACTION_PLACE_KFS, ACTION_LOCK_CHASSIS, ACTION_STATUS_DONE,
    ACTION_ARM_LIFT_1, ACTION_ARM_LIFT_2,
    ASSEMBLY_STATUS_DONE,
    R1_SIGNAL_ENTER_MERLIN, R1_SIGNAL_MERGE,
    GAME_CMD_RESET,
    BLOCK_CENTERS, BLOCK_HEIGHTS,
    MERLIN_DEFAULT_ENTRY,
    MERLIN_ENTRY_CLIMB_POINTS, MERLIN_EXIT_DESCEND_POINTS,
    MERLIN_CLIMB_WAIT_S, MERLIN_PICKUP_WAIT_S,
    ENTRY_CLIMB_CMD, EXIT_DESCEND_CMD,
    KFS_COLOR_BLUE, KFS_COLOR_RED,
    FINE_ALIGN_DISABLE, FINE_ALIGN_ENABLE_BLUE, FINE_ALIGN_ENABLE_RED,
    FINE_ALIGN_STATUS_ALIGNING, FINE_ALIGN_STATUS_DONE,
    FINE_ALIGN_TIMEOUT_S,
    KFS_PLACE_STOP_WAIT_S, KFS_PLACE_CMD_DELAY_S,
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
    NAV_TO_KFS_PLACE       = 'NAV_TO_KFS_PLACE'
    PLACE_KFS              = 'PLACE_KFS'
    NAV_TO_CONFRONT_WAIT   = 'NAV_TO_CONFRONT_WAIT'
    WAIT_MERGE             = 'WAIT_MERGE'
    MERGE_WITH_R1          = 'MERGE_WITH_R1'
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
    SEND_CLIMB     = 'M_SEND_CLIMB'
    CLIMB_WAIT     = 'M_CLIMB_WAIT'
    NAV_TO_CENTER  = 'M_NAV_TO_CENTER'
    EXIT_NAV       = 'M_EXIT_NAV'
    EXIT_DESCEND   = 'M_EXIT_DESCEND'
    DONE           = 'M_DONE'


class GameController(Node):

    def __init__(self):
        super().__init__('game_controller')

        # ── Game State ──
        self._phase = GamePhase.WAIT_INPUT
        self._phase_enter_time = time.time()
        self._match_start_time = None

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
        self._assembly_status   = 0
        self._r1_signal         = 0
        self._decision_state_id = 0

        # ── Merlin State ──
        self._merlin_step       = MerlinStep.INIT
        self._merlin_step_time  = 0.0
        self._merlin_path       = []
        self._merlin_path_idx   = 0
        self._merlin_pickup     = {}   # {kfs_block: pickup_from_block}
        self._merlin_picked     = set()
        self._merlin_kfs_target = 0
        self._merlin_climb_sent = False

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
        self._fine_align_enable_pub = self.create_publisher(UInt8, '/fine_align/enable', 10)
        self._relocalize_pub = self.create_publisher(String, '/relocalize/trigger', 10)

        # ── Subscribers ──
        self.create_subscription(UInt8,  '/feedback/action_group', self._on_action_fb, 10)
        self.create_subscription(UInt8,  '/feedback/assembly',     self._on_assembly, 10)
        self.create_subscription(UInt8,  '/game/start_signal',   self._on_start_signal, 10)
        self.create_subscription(String, '/game/kfs_input',      self._on_kfs_input, 10)
        self.create_subscription(UInt8,  '/game/r1_signal',      self._on_r1_signal, 10)
        self.create_subscription(Int8,   '/decision/state_id',   self._on_decision_state, 10)
        self.create_subscription(Twist,  '/cmd_vel',             self._on_cmd_vel, 10)
        self.create_subscription(String, '/waypoint_nav/status', self._on_nav_status_str, 10)
        self.create_subscription(Twist,  '/fine_align/cmd',       self._on_fine_align_cmd, 10)
        self.create_subscription(UInt8,  '/fine_align/status',    self._on_fine_align_status, 10)

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
        if area not in ('weapon', 'merlin', 'confront'):
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
        elif area == 'merlin':
            self._phase = GamePhase.MERLIN_PHASE
            self._merlin_step = MerlinStep.INIT
            entry_wp = WAYPOINT_MERLIN_ENTRY
        else:  # confront
            self._phase = GamePhase.NAV_TO_CONFRONT_ENTRY
            entry_wp = WAYPOINT_CONFRONT_ENTRY

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

    # ════════════════════════════════════════
    #   Phase / Merlin Step Management
    # ════════════════════════════════════════

    def _set_phase(self, p: str):
        old = self._phase
        self._phase = p
        self._phase_enter_time = time.time()
        self._lock_sent = False
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
    def _on_assembly(self, msg):     self._assembly_status = msg.data
    def _on_r1_signal(self, msg):    self._r1_signal = msg.data
    def _on_decision_state(self, msg): self._decision_state_id = msg.data
    def _on_cmd_vel(self, msg):      self._last_cmd_vel = msg
    def _on_nav_status_str(self, msg): self._nav_status_str = msg.data
    def _on_fine_align_cmd(self, msg):    self._fine_align_cmd = msg
    def _on_fine_align_status(self, msg): self._fine_align_status = msg.data

    def _on_start_signal(self, msg):
        if msg.data == 1 and not self._start_signal_received:
            self._start_signal_received = True
            self.get_logger().info('启动信号已接收 (下位机)')

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
        msg = UInt8(); msg.data = action_id
        self._action_pub.publish(msg)
        self._action_status = 0

    def _send_chassis_stop(self):
        self._chassis_pub.publish(Twist())

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
        if self._assembly_status:  fb_parts.append(f'asm={self._assembly_status}')
        if self._r1_signal:        fb_parts.append(f'r1={self._r1_signal}')
        if self._decision_state_id: fb_parts.append(f'vis={self._decision_state_id}')
        if self._merlin_step == MerlinStep.FINE_ALIGN:
            fb_parts.append(f'falign={self._fine_align_status}')
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
                and self._match_elapsed() >= MATCH_TIMEOUT_S):
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
        elif phase == GamePhase.NAV_TO_EXIT_MERLIN:    self._tick_nav_to(WAYPOINT_EXIT_MERLIN, GamePhase.NAV_TO_CONFRONT_ENTRY)
        elif phase == GamePhase.NAV_TO_CONFRONT_ENTRY: self._tick_confront_entry()
        elif phase == GamePhase.NAV_TO_KFS_PLACE:      self._tick_nav_to(WAYPOINT_KFS_PLACE, GamePhase.PLACE_KFS)
        elif phase == GamePhase.PLACE_KFS:             self._tick_place_kfs()
        elif phase == GamePhase.NAV_TO_CONFRONT_WAIT:  self._tick_nav_to(WAYPOINT_CONFRONT_WAIT, GamePhase.WAIT_MERGE)
        elif phase == GamePhase.WAIT_MERGE:            self._tick_wait_merge()
        elif phase == GamePhase.MERGE_WITH_R1:         self._tick_merge_with_r1()
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
        if self._start_signal_received:
            self._match_start_time = time.time()
            self._reset_nav()
            self._trigger_relocalize('weapon')
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
            self._send_action(ACTION_PICKUP_WEAPON)
            self._set_phase(GamePhase.GRAB_WEAPON)

    def _tick_grab_weapon(self):
        if self._action_status == ACTION_STATUS_DONE:
            self._reset_nav()
            self._set_phase(GamePhase.NAV_TO_ASSEMBLY)

    def _tick_wait_assembly(self):
        self._send_lock_chassis_once()
        if self._assembly_status == ASSEMBLY_STATUS_DONE:
            self._send_action(ACTION_RELEASE_WEAPON)
            self._set_phase(GamePhase.RELEASE_WEAPON)

    def _tick_release_weapon(self):
        if self._phase_elapsed() >= PHASE_SWITCH_WAIT_S:
            self._set_phase(GamePhase.WAIT_ENTER_MERLIN)

    def _tick_wait_enter_merlin(self):
        if self._r1_signal == R1_SIGNAL_ENTER_MERLIN:
            self._reset_nav()
            self._set_phase(GamePhase.NAV_TO_MERLIN_ENTRY)

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
        elif s == MerlinStep.SEND_CLIMB:     self._m_send_climb()
        elif s == MerlinStep.CLIMB_WAIT:     self._m_climb_wait()
        elif s == MerlinStep.NAV_TO_CENTER:  self._m_nav_to_center()
        elif s == MerlinStep.EXIT_NAV:       self._m_exit_nav()
        elif s == MerlinStep.EXIT_DESCEND:   self._m_exit_descend()
        elif s == MerlinStep.DONE:           self._m_done()

    # ── M_INIT: 规划路径 ──

    def _m_init(self):
        entry = MERLIN_DEFAULT_ENTRY
        path = mpp.plan_path(entry, self._kfs_real_blocks, self._kfs_fake_blocks)
        if not path:
            self.get_logger().error('梅林路径规划失败! 无可行路径')
            self._set_phase(GamePhase.STOP)
            return
        self._merlin_path = path
        self._merlin_path_idx = 0
        self._merlin_pickup = mpp.get_pickup_info(path, self._kfs_real_blocks)
        self._merlin_picked = set()
        self.get_logger().info(f'梅林路径: {path}')
        self.get_logger().info(f'KFS拾取: {self._merlin_pickup}')
        self._reset_nav()
        self._set_merlin_step(MerlinStep.ENTRY_NAV)

    # ── M_ENTRY_NAV: 导航到入口爬升点 ──

    def _m_entry_nav(self):
        entry = self._merlin_path[0]
        wp = MERLIN_ENTRY_CLIMB_POINTS[entry]
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(wp['x'], wp['y'], wp['yaw'])
        elif self._nav_done:
            self._reset_nav()
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
            cur = self._merlin_path[self._merlin_path_idx]
            kfs = self._merlin_kfs_target
            h_cur = BLOCK_HEIGHTS.get(cur, 0.0)
            h_kfs = BLOCK_HEIGHTS.get(kfs, 0.0)
            # 抬升1: 抓取比当前台阶高的物体; 抬升2: 抓取比当前台阶低的物体
            action = ACTION_ARM_LIFT_1 if h_kfs > h_cur else ACTION_ARM_LIFT_2
            self._send_action(action)
            self._arm_lift_sent = True
            self.get_logger().info(
                f'机械臂抬升: 方块{cur}(h={h_cur:.2f}m) -> KFS{kfs}(h={h_kfs:.2f}m) '
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
            self._merlin_picked.add(self._merlin_kfs_target)
            self.get_logger().info(f'KFS {self._merlin_kfs_target} 拾取成功')
            # 拾取后已在触发点位置, 判断是否需要爬升到KFS方块
            idx = self._merlin_path_idx
            if idx + 1 < len(self._merlin_path) and self._merlin_path[idx + 1] == self._merlin_kfs_target:
                self._reset_nav()
                self._set_merlin_step(MerlinStep.SEND_CLIMB)
            else:
                self._reset_nav()
                self._set_merlin_step(MerlinStep.NAV_TO_TRIGGER)
        elif self._merlin_step_elapsed() >= MERLIN_PICKUP_WAIT_S:
            self.get_logger().warn(f'KFS {self._merlin_kfs_target} 拾取超时, 跳过')
            self._merlin_picked.add(self._merlin_kfs_target)
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
            self._set_merlin_step(MerlinStep.SEND_CLIMB)

    # ── M_SEND_CLIMB: 发送爬升/下降指令 ──

    def _m_send_climb(self):
        idx = self._merlin_path_idx
        cur = self._merlin_path[idx]
        nxt = self._merlin_path[idx + 1]
        cmd = mpp.get_transition_climb(cur, nxt)
        if cmd != 0:
            h_diff = (BLOCK_CENTERS.get(nxt, (0, 0))[1] -
                      BLOCK_CENTERS.get(cur, (0, 0))[1])
            self._send_meilin_climb(cmd, next_block=nxt, height_diff=h_diff)
            self.get_logger().info(f'方块{cur}->{nxt}: 爬升指令={cmd}')
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
        self._reset_nav()
        self._set_phase(GamePhase.NAV_TO_MERLIN_EXIT)

    # ════════════════════════════════════════
    #   对抗区
    # ════════════════════════════════════════

    def _tick_confront_entry(self):
        """导航到对抗区入口, 到达后不停车直接切到放置点导航."""
        if not self._nav_active and not self._nav_done:
            self._try_send_nav(
                WAYPOINT_CONFRONT_ENTRY['x'],
                WAYPOINT_CONFRONT_ENTRY['y'],
                WAYPOINT_CONFRONT_ENTRY['yaw'])
        elif self._nav_done:
            self._reset_nav()
            self._trigger_relocalize('confront')
            self._try_send_nav(
                WAYPOINT_KFS_PLACE['x'],
                WAYPOINT_KFS_PLACE['y'],
                WAYPOINT_KFS_PLACE['yaw'])
            self._set_phase(GamePhase.NAV_TO_KFS_PLACE)

    def _tick_place_kfs(self):
        """到达放置点后, 发送放置 KFS 动作组指令."""
        if self._phase_elapsed() < KFS_PLACE_STOP_WAIT_S:
            self._send_chassis_stop()
            return
        if self._action_status != ACTION_STATUS_DONE:
            if self._phase_elapsed() < KFS_PLACE_CMD_DELAY_S:
                self._send_action(ACTION_PLACE_KFS)
                self.get_logger().info('发送放置KFS指令 (抬升40cm + 放置)')
        else:
            self.get_logger().info('KFS放置完成')
            self._reset_nav()
            self._set_phase(GamePhase.NAV_TO_CONFRONT_WAIT)

    def _tick_wait_merge(self):
        """在对抗区等待点, 等待 R1 发送合体指令."""
        self._send_chassis_stop()
        if self._r1_signal == R1_SIGNAL_MERGE:
            self.get_logger().info('收到 R1 合体指令')
            self._set_phase(GamePhase.MERGE_WITH_R1)

    def _tick_merge_with_r1(self):
        """合体: 通知下位机底盘锁死, 等待下位机完成."""
        self._send_chassis_stop()
        self._send_lock_chassis_once()
        # 合体动作由下位机控制, 上位机只需锁死底盘
        # 预留: 后续加入 R2 对齐 R1 的导航逻辑
        self._set_phase(GamePhase.STOP)

    # ════════════════════════════════════════
    #   Timeout
    # ════════════════════════════════════════

    def _on_timeout(self):
        self.get_logger().warn(
            f'比赛超时 ({self._match_elapsed():.1f}s), 复位')
        self._cancel_nav()
        self._send_chassis_stop()
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
