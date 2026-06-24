"""
mock_feedback_node.py
模拟下位机 / R1 / 视觉反馈, 用于离线测试 game_controller 状态机

模拟的信号:
  - Nav2 action server (NavigateToPose): 接受目标, 延时后返回成功
  - /feedback/action_group: 动作组执行反馈
  - /feedback/assembly:     组装完成反馈
  - /game/r1_signal:        R1 机器人信号
  - /game/start_signal:     启动按钮
  - /game/kfs_input:        KFS配置 (可选, 也可用终端输入)
  - /decision/state_id:     视觉对齐状态 (模拟 processor_node)
  - 梅林爬升/下降: 通过 Gazebo 瞬移机器人到目标台阶高度

启动方式:
  ros2 run decision_processor mock_feedback
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, GoalResponse, CancelResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
from rclpy.executors import MultiThreadedExecutor

from std_msgs.msg import String, UInt8, Int8
from geometry_msgs.msg import Twist
from nav2_msgs.action import NavigateToPose

from .config import (
    ACTION_PICKUP_WEAPON, ACTION_RELEASE_WEAPON, ACTION_PICKUP_KFS,
    ACTION_LOCK_CHASSIS, ACTION_ARM_LIFT_1, ACTION_ARM_LIFT_2,
    ACTION_STATUS_DONE,
    ASSEMBLY_STATUS_DONE,
    R1_SIGNAL_ENTER_MERLIN,
    BLOCK_CENTERS,
    BLOCK_HEIGHTS,
    FINE_ALIGN_DISABLE,
    FINE_ALIGN_STATUS_DONE,
)

try:
    from gazebo_msgs.srv import SetEntityState
    from gazebo_msgs.msg import EntityState
    _HAS_GAZEBO_MSGS = True
except ImportError:
    _HAS_GAZEBO_MSGS = False


NAV_DELAY_S        = 2.0
ACTION_DELAY_S     = 1.5
ALIGN_DELAY_S      = 1.5
ASSEMBLY_DELAY_S   = 2.0
R1_DELAY_S         = 2.0
AUTO_START_DELAY_S = 1.0


class MockFeedbackNode(Node):

    def __init__(self):
        super().__init__('mock_feedback')

        self._current_phase = ''
        self._prev_phase = ''

        # ── Publishers (模拟反馈) ──
        self._action_fb_pub   = self.create_publisher(UInt8, '/feedback/action_group', 10)
        self._assembly_fb_pub = self.create_publisher(UInt8, '/feedback/assembly', 10)
        self._r1_signal_pub   = self.create_publisher(UInt8, '/game/r1_signal', 10)
        self._start_sig_pub   = self.create_publisher(UInt8, '/game/start_signal', 10)
        self._kfs_input_pub   = self.create_publisher(String, '/game/kfs_input', 10)
        self._state_id_pub    = self.create_publisher(Int8, '/decision/state_id', 10)
        self._fine_align_status_pub = self.create_publisher(UInt8, '/fine_align/status', 10)

        # ── Subscribers (监控指令) ──
        self.create_subscription(String, '/game/phase', self._on_phase, 10)
        self.create_subscription(UInt8, '/serial/action_group_cmd', self._on_action_cmd, 10)
        self.create_subscription(UInt8, '/game/cmd', self._on_game_cmd, 10)
        self.create_subscription(UInt8, '/fine_align/enable', self._on_fine_align_enable, 10)

        # ── Mock Nav2 Action Server (当有真实导航器时可禁用) ──
        self.declare_parameter('mock_nav', True)
        self._nav_server = None
        if self.get_parameter('mock_nav').value:
            self._action_cb = MutuallyExclusiveCallbackGroup()
            self._nav_server = ActionServer(
                self,
                NavigateToPose,
                'navigate_to_pose',
                execute_callback=self._nav_execute,
                goal_callback=self._nav_goal_cb,
                cancel_callback=self._nav_cancel_cb,
                callback_group=self._action_cb,
            )

        # ── 梅林爬升瞬移 (Gazebo) ──
        self.declare_parameter('sim_teleport', True)
        self.declare_parameter('robot_name', 'rc2026_robot')
        self.declare_parameter('gz_origin_y', 6.0)      # game_y = -gz_x + origin_y

        self._sim_teleport = self.get_parameter('sim_teleport').value
        self._robot_name = self.get_parameter('robot_name').value
        self._gz_origin_y = self.get_parameter('gz_origin_y').value
        self._gz_client = None

        if self._sim_teleport and _HAS_GAZEBO_MSGS:
            self._gz_client = self.create_client(
                SetEntityState, '/set_entity_state')
            self.create_subscription(
                Twist, '/serial/meilin_cmd', self._on_meilin_cmd, 10)
            self.get_logger().info('梅林爬升瞬移已启用 (Gazebo teleport)')

        # ── Phase reaction timer (5Hz) ──
        self.create_timer(0.2, self._phase_tick)

        # ── Auto-start via topic (optional) ──
        self.declare_parameter('auto_start', True)
        self.declare_parameter('kfs_real', '5')
        self.declare_parameter('kfs_fake', '8')
        self.declare_parameter('kfs_color', 'blue')

        if self.get_parameter('auto_start').value:
            self.create_timer(AUTO_START_DELAY_S, self._auto_start, callback_group=None)
            self._auto_started = False

        self.get_logger().info('Mock反馈节点已启动')
        self.get_logger().info(f'  Nav延时={NAV_DELAY_S}s  动作组延时={ACTION_DELAY_S}s')

    # ════════════════════════════════════════
    #   Auto Start
    # ════════════════════════════════════════

    def _auto_start(self):
        if getattr(self, '_auto_started', False):
            return
        self._auto_started = True

        real = self.get_parameter('kfs_real').value
        fake = self.get_parameter('kfs_fake').value
        color = self.get_parameter('kfs_color').value
        kfs_msg = String()
        kfs_msg.data = f'real:{real} fake:{fake} color:{color}'
        self._kfs_input_pub.publish(kfs_msg)
        self.get_logger().info(f'发送KFS配置: {kfs_msg.data}')

        start_msg = UInt8()
        start_msg.data = 1
        self._start_sig_pub.publish(start_msg)
        self.get_logger().info('发送启动信号')

    # ════════════════════════════════════════
    #   Phase Monitor
    # ════════════════════════════════════════

    def _on_phase(self, msg):
        self._current_phase = msg.data

    def _phase_tick(self):
        phase = self._current_phase
        prev = self._prev_phase
        if not phase or phase == prev:
            return
        self._prev_phase = phase

        if phase == 'ALIGN_WEAPON':
            threading.Thread(
                target=self._delayed_state_id,
                args=(ALIGN_DELAY_S,),
                daemon=True,
            ).start()

        elif phase == 'WAIT_ASSEMBLY':
            threading.Thread(
                target=self._delayed_assembly,
                daemon=True,
            ).start()

        elif phase == 'WAIT_ENTER_MERLIN':
            threading.Thread(
                target=self._delayed_r1_signal,
                daemon=True,
            ).start()

        elif phase == 'ALIGN_KFS':
            threading.Thread(
                target=self._delayed_state_id,
                args=(ALIGN_DELAY_S,),
                daemon=True,
            ).start()

        elif phase == 'WAIT_MERGE':
            threading.Thread(
                target=self._delayed_r1_merge,
                daemon=True,
            ).start()

    def _delayed_state_id(self, delay):
        time.sleep(delay)
        msg = Int8()
        msg.data = 4  # RobotState.ARRIVED.value
        self._state_id_pub.publish(msg)
        self.get_logger().info('模拟视觉对齐完成 (state_id=ARRIVED)')

    def _delayed_assembly(self):
        time.sleep(ASSEMBLY_DELAY_S)
        msg = UInt8()
        msg.data = ASSEMBLY_STATUS_DONE
        self._assembly_fb_pub.publish(msg)
        self.get_logger().info('模拟组装完成')

    def _delayed_r1_signal(self):
        time.sleep(R1_DELAY_S)
        msg = UInt8()
        msg.data = R1_SIGNAL_ENTER_MERLIN
        self._r1_signal_pub.publish(msg)
        self.get_logger().info('模拟R1进入梅林信号')

    def _delayed_r1_merge(self):
        time.sleep(R1_DELAY_S)
        msg = UInt8()
        msg.data = 3  # R1_SIGNAL_MERGE
        self._r1_signal_pub.publish(msg)
        self.get_logger().info('模拟R1合体指令')

    # ════════════════════════════════════════
    #   Action Group Command → Feedback
    # ════════════════════════════════════════

    ACTION_NAMES = {
        ACTION_PICKUP_WEAPON:  '拾取武器端头',
        ACTION_RELEASE_WEAPON: '释放武器端头',
        ACTION_PICKUP_KFS:     '拾取KFS',
        ACTION_LOCK_CHASSIS:   '底盘锁死',
        ACTION_ARM_LIFT_1:     '机械臂抬升1',
        ACTION_ARM_LIFT_2:     '机械臂抬升2',
    }

    def _on_action_cmd(self, msg):
        action_id = msg.data
        name = self.ACTION_NAMES.get(action_id, f'未知({action_id})')
        self.get_logger().info(f'收到动作组指令: {name}, {ACTION_DELAY_S}s后返回完成')
        threading.Thread(
            target=self._delayed_action_done,
            daemon=True,
        ).start()

    def _delayed_action_done(self):
        time.sleep(ACTION_DELAY_S)
        msg = UInt8()
        msg.data = ACTION_STATUS_DONE
        self._action_fb_pub.publish(msg)
        self.get_logger().info('模拟动作组执行完成')

    # ════════════════════════════════════════
    #   Fine-Align Mock (无USB相机时模拟精对齐完成)
    # ════════════════════════════════════════

    def _on_fine_align_enable(self, msg):
        if msg.data == FINE_ALIGN_DISABLE:
            return
        self.get_logger().info(f'收到精对齐启用指令: {msg.data}, {ALIGN_DELAY_S}s后返回对齐完成')
        threading.Thread(
            target=self._delayed_fine_align_done,
            daemon=True,
        ).start()

    def _delayed_fine_align_done(self):
        time.sleep(ALIGN_DELAY_S)
        msg = UInt8()
        msg.data = FINE_ALIGN_STATUS_DONE
        self._fine_align_status_pub.publish(msg)
        self.get_logger().info('模拟精对齐完成')

    # ════════════════════════════════════════
    #   Game Command Monitor
    # ════════════════════════════════════════

    def _on_game_cmd(self, msg):
        self.get_logger().info(f'收到比赛指令: {msg.data}')

    # ════════════════════════════════════════
    #   Mock Nav2 Action Server
    # ════════════════════════════════════════

    def _nav_goal_cb(self, goal_request):
        pos = goal_request.pose.pose.position
        self.get_logger().info(
            f'Nav2 目标接收: ({pos.x:.2f}, {pos.y:.2f})')
        return GoalResponse.ACCEPT

    def _nav_cancel_cb(self, goal_handle):
        self.get_logger().info('Nav2 目标取消')
        return CancelResponse.ACCEPT

    def _nav_execute(self, goal_handle):
        pos = goal_handle.request.pose.pose.position
        self.get_logger().info(
            f'Nav2 导航中: ({pos.x:.2f}, {pos.y:.2f}), 等待{NAV_DELAY_S}s...')
        time.sleep(NAV_DELAY_S)

        if goal_handle.is_cancel_requested:
            goal_handle.canceled()
            self.get_logger().info('Nav2 已取消')
            return NavigateToPose.Result()

        goal_handle.succeed()
        self.get_logger().info(f'Nav2 到达: ({pos.x:.2f}, {pos.y:.2f})')
        return NavigateToPose.Result()


    # ════════════════════════════════════════
    #   梅林爬升瞬移 (Gazebo)
    #
    #   /serial/meilin_cmd (Twist):
    #     linear.x  = next_block (目标台阶编号)
    #     linear.y  = climb_mode (1=爬升, 2=爬升, 3=下降, 4=下降)
    #
    #   BLOCK_CENTERS 为 game 坐标, 转换为 Gazebo:
    #     gz_x = origin_y - game_y
    #     gz_y = game_x
    #     gz_z = block_height
    # ════════════════════════════════════════

    def _on_meilin_cmd(self, msg: Twist):
        next_block = int(msg.linear.x)
        climb_mode = int(msg.linear.y)
        if next_block <= 0 or climb_mode == 0:
            return

        height = BLOCK_HEIGHTS.get(next_block, 0.0)
        center = BLOCK_CENTERS.get(next_block)
        if center is None:
            self.get_logger().warn(f'未知台阶编号: {next_block}')
            return

        game_x, game_y = center
        gz_x = self._gz_origin_y - game_y
        gz_y = game_x

        self.get_logger().info(
            f'梅林瞬移: 台阶{next_block} game({game_x:.1f},{game_y:.1f}) → '
            f'Gazebo({gz_x:.2f}, {gz_y:.2f}, z={height:.2f}m)')

        self._teleport_robot(gz_x, gz_y, height)

    def _teleport_robot(self, gz_x, gz_y, gz_z):
        if self._gz_client is None or not _HAS_GAZEBO_MSGS:
            return

        if not self._gz_client.service_is_ready():
            self.get_logger().warn('Gazebo set_entity_state 服务不可用')
            return

        req = SetEntityState.Request()
        state = EntityState()
        state.name = self._robot_name
        state.pose.position.x = gz_x
        state.pose.position.y = gz_y
        state.pose.position.z = gz_z
        state.pose.orientation.w = 1.0
        state.reference_frame = 'world'
        req.state = state

        future = self._gz_client.call_async(req)
        future.add_done_callback(
            lambda f: self._on_teleport_result(f, gz_x, gz_y, gz_z))

    def _on_teleport_result(self, future, gz_x, gz_y, gz_z):
        try:
            result = future.result()
            if result and result.success:
                self.get_logger().info(
                    f'瞬移成功: Gazebo({gz_x:.2f}, {gz_y:.2f}, {gz_z:.2f})')
            else:
                self.get_logger().warn('瞬移失败: 服务返回失败')
        except Exception as e:
            self.get_logger().warn(f'瞬移异常: {e}')


def main(args=None):
    rclpy.init(args=args)
    node = MockFeedbackNode()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
