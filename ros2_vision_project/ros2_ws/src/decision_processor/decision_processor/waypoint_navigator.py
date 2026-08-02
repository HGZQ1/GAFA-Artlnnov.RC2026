"""
waypoint_navigator.py
闭环路径点导航器 — 替代 Nav2, 直接 PID 控制全向底盘

三阶段导航:
  1. XY 闭环导航: 只控制平移到目标点附近, 不强制转 yaw
  2. 原地转向: XY 到位后只控制 angular.z 转到目标 yaw
  3. 视觉伺服对齐: 自动切入 processor_node 视觉对齐 (精对齐, ~2cm)
     可选, 通过 /waypoint_nav/servo_phase 话题激活

话题接口:
  订阅  /waypoint_nav/goal_pose    PoseStamped  设置导航目标 (game坐标)
  订阅  /waypoint_nav/cancel       String       取消当前导航
  订阅  /waypoint_nav/servo_phase  String       设置视觉伺服阶段名
         (发送 "ALIGN_WEAPON" / "ALIGN_KFS" 激活, 空字符串禁用)
  订阅  /waypoint_nav/speed_limit  Twist        linear.x=max线速度, angular.z=max角速度; <=0恢复默认
  订阅  /waypoint_nav/yaw_turn_direction Int8   yaw-only转向方向: 1=逆时针, -1=顺时针, 0=最短路径
  订阅  /decision/state_id         Int8         processor_node 状态反馈
  发布  /cmd_vel                   Twist        底盘速度指令
  发布  /waypoint_nav/status       String       IDLE/NAV/VISUAL_SERVO/ARRIVED/TIMEOUT/STUCK
  发布  /game/phase                String       通知 processor_node 进入视觉伺服

Action:
  navigate_to_pose  nav2_msgs/NavigateToPose  兼容 Nav2 action 接口

坐标变换 (coord_mode 参数选择):
  'offset' 模式: game = loc + offset                  (FAST-LIO / 简单场景)
  'gazebo' 模式: game_x = gz_y + offset_x  (-2.8)     (Gazebo 仿真专用)
                 game_y = -gz_x + offset_y  (6.0)
                 game_yaw = gz_yaw - π/2
"""

import math
import time
import threading

import rclpy
from rclpy.node import Node
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor

import tf2_ros
from geometry_msgs.msg import Twist, PoseStamped
from std_msgs.msg import String, Int8
from nav2_msgs.action import NavigateToPose

# processor_node 的 RobotState.ARRIVED = 4
_VISUAL_SERVO_ARRIVED = 4


class WaypointNavigator(Node):

    def __init__(self):
        super().__init__('waypoint_navigator')

        # ── 坐标变换参数 ──
        self.declare_parameter('coord_mode', 'gazebo')   # 'gazebo' / 'fastlio' / 'offset'
        self.declare_parameter('loc_offset_x', -1.4)     # fastlio: 启动点 game_x
        self.declare_parameter('loc_offset_y', 0.4)      # fastlio: 启动点 game_y
        self.declare_parameter('loc_offset_yaw', 1.5708) # fastlio: 启动点 game_yaw
        # gazebo 模式硬编码: game_x=gz_y, game_y=-gz_x+6.0, yaw=gz_yaw-π/2
        self.declare_parameter('gz_origin_y', 6.0)       # gazebo 模式专用
        self.declare_parameter('map_frame', 'map')
        self.declare_parameter('robot_frame', 'base_link')

        # ── 运动控制参数 ──
        self.declare_parameter('max_linear_speed', 1.0)
        self.declare_parameter('min_linear_speed', 0.05)
        self.declare_parameter('max_angular_speed', 2.0)
        self.declare_parameter('kp_linear', 0.4)   #原1.2
        self.declare_parameter('kp_angular', 2.0)
        self.declare_parameter('decel_distance', 0.30)
        self.declare_parameter('max_linear_accel', 0.40)
        self.declare_parameter('max_angular_accel', 0.80)

        # ── 到达判定 ──
        self.declare_parameter('xy_tolerance', 0.05)
        self.declare_parameter('yaw_tolerance', 0.10)

        # ── 超时参数 ──
        self.declare_parameter('waypoint_timeout', 30.0)
        self.declare_parameter('progress_timeout', 3.0)
        self.declare_parameter('progress_min_delta', 0.02)
        self.declare_parameter('timeout_tolerance_factor', 2.0)
        self.declare_parameter('visual_servo_timeout', 15.0)

        # ── 控制频率 ──
        self.declare_parameter('control_rate', 20.0)

        # 读取参数
        self._coord_mode = self.get_parameter('coord_mode').value
        self._offset_x = self.get_parameter('loc_offset_x').value
        self._offset_y = self.get_parameter('loc_offset_y').value
        self._offset_yaw = self.get_parameter('loc_offset_yaw').value
        self._gz_origin_y = self.get_parameter('gz_origin_y').value
        self._map_frame = self.get_parameter('map_frame').value
        self._robot_frame = self.get_parameter('robot_frame').value

        self._max_lin = self.get_parameter('max_linear_speed').value
        self._min_lin = self.get_parameter('min_linear_speed').value
        self._max_ang = self.get_parameter('max_angular_speed').value
        self._default_max_lin = self._max_lin
        self._default_max_ang = self._max_ang
        self._kp_lin = self.get_parameter('kp_linear').value
        self._kp_ang = self.get_parameter('kp_angular').value
        self._decel_dist = self.get_parameter('decel_distance').value
        self._max_lin_accel = self.get_parameter('max_linear_accel').value
        self._max_ang_accel = self.get_parameter('max_angular_accel').value

        self._xy_tol = self.get_parameter('xy_tolerance').value
        self._yaw_tol = self.get_parameter('yaw_tolerance').value

        self._wp_timeout = self.get_parameter('waypoint_timeout').value
        self._prog_timeout = self.get_parameter('progress_timeout').value
        self._prog_delta = self.get_parameter('progress_min_delta').value
        self._relax_factor = self.get_parameter('timeout_tolerance_factor').value
        self._servo_timeout = self.get_parameter('visual_servo_timeout').value

        rate = self.get_parameter('control_rate').value

        # ── 导航状态 ──
        self._lock = threading.Lock()
        self._target_x = 0.0
        self._target_y = 0.0
        self._target_yaw = 0.0
        self._has_target = False
        self._nav_status = 'IDLE'      # IDLE / NAV / VISUAL_SERVO / ARRIVED / TIMEOUT / STUCK

        self._nav_start_time = 0.0
        self._last_progress_time = 0.0
        self._best_dist = float('inf')
        self._stuck_recovery_count = 0
        self._last_cmd = Twist()
        self._yaw_only_latched = False
        self._yaw_only_logged = False
        self._yaw_turn_direction = 0

        # ── 视觉伺服状态 ──
        self._servo_phase = ''         # 空 = 不使用视觉伺服
        self._servo_start_time = 0.0
        self._decision_state_id = 0    # 来自 processor_node

        # ── TF ──
        self._tf_buffer = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer, self)

        # ── Action Server (兼容 Nav2 NavigateToPose) ──
        self._action_cb_group = ReentrantCallbackGroup()
        self._action_server = ActionServer(
            self, NavigateToPose, 'navigate_to_pose',
            execute_callback=self._execute_navigate,
            goal_callback=self._on_goal_request,
            cancel_callback=self._on_cancel_request,
            callback_group=self._action_cb_group)

        # ── Topic 接口 ──
        self.create_subscription(
            PoseStamped, '/waypoint_nav/goal_pose',
            self._on_goal_topic, 10)
        self.create_subscription(
            String, '/waypoint_nav/cancel',
            self._on_cancel_topic, 10)
        self.create_subscription(
            String, '/waypoint_nav/servo_phase',
            self._on_servo_phase, 10)
        self.create_subscription(
            Twist, '/waypoint_nav/speed_limit',
            self._on_speed_limit, 10)
        self.create_subscription(
            Int8, '/waypoint_nav/yaw_turn_direction',
            self._on_yaw_turn_direction, 10)
        self.create_subscription(
            Int8, '/decision/state_id',
            self._on_decision_state, 10)

        # ── Publishers ──
        self._cmd_pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self._status_pub = self.create_publisher(String, '/waypoint_nav/status', 10)
        self._phase_pub = self.create_publisher(String, '/game/phase', 10)

        # ── 控制循环 ──
        self._dt = 1.0 / rate
        self.create_timer(self._dt, self._control_tick)

        self.get_logger().info(
            f'WaypointNavigator | coord_mode={self._coord_mode} '
            f'offset=({self._offset_x:.1f}, {self._offset_y:.1f}) '
            f'max_speed={self._max_lin} tol={self._xy_tol}m')

    # ═══════════════════════════════════
    #   坐标变换
    #
    #   'gazebo' 模式 (Gazebo仿真):
    #     game_x   = gz_y
    #     game_y   = -gz_x + gz_origin_y    (6.0)
    #     game_yaw = gz_yaw - π/2
    #
    #   'fastlio' 模式 (真机 / 仿真测试):
    #     FAST-LIO 初始 (0,0,yaw=0), 起点 game 坐标 = (offset_x, offset_y, offset_yaw)
    #     game = R(offset_yaw) × loc + (offset_x, offset_y)
    #
    #   'offset' 模式 (简单偏移):
    #     game = loc + offset
    # ═══════════════════════════════════

    def _loc_to_game(self, lx, ly, lyaw):
        if self._coord_mode == 'gazebo':
            gx = ly
            gy = -lx + self._gz_origin_y
            gyaw = _normalize(lyaw - math.pi / 2.0)
            return gx, gy, gyaw

        if self._coord_mode == 'fastlio':
            cos_a = math.cos(self._offset_yaw)
            sin_a = math.sin(self._offset_yaw)
            gx = cos_a * lx - sin_a * ly + self._offset_x
            gy = sin_a * lx + cos_a * ly + self._offset_y
            gyaw = _normalize(lyaw + self._offset_yaw)
            return gx, gy, gyaw

        return (lx + self._offset_x,
                ly + self._offset_y,
                _normalize(lyaw + self._offset_yaw))

    def _game_to_loc(self, gx, gy, gyaw):
        if self._coord_mode == 'gazebo':
            lx = self._gz_origin_y - gy
            ly = gx
            lyaw = _normalize(gyaw + math.pi / 2.0)
            return lx, ly, lyaw

        if self._coord_mode == 'fastlio':
            dx = gx - self._offset_x
            dy = gy - self._offset_y
            cos_a = math.cos(-self._offset_yaw)
            sin_a = math.sin(-self._offset_yaw)
            lx = cos_a * dx - sin_a * dy
            ly = sin_a * dx + cos_a * dy
            lyaw = _normalize(gyaw - self._offset_yaw)
            return lx, ly, lyaw

        return (gx - self._offset_x,
                gy - self._offset_y,
                _normalize(gyaw - self._offset_yaw))

    # ═══════════════════════════════════
    #   获取当前位姿 (game坐标)
    # ═══════════════════════════════════

    def _get_pose(self):
        try:
            t = self._tf_buffer.lookup_transform(
                self._map_frame, self._robot_frame, rclpy.time.Time())
            lx = t.transform.translation.x
            ly = t.transform.translation.y
            q = t.transform.rotation
            lyaw = math.atan2(
                2.0 * (q.w * q.z + q.x * q.y),
                1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            return self._loc_to_game(lx, ly, lyaw)
        except Exception:
            return None, None, None

    # ═══════════════════════════════════
    #   设置 / 取消目标
    # ═══════════════════════════════════

    def _set_target(self, x, y, yaw):
        with self._lock:
            self._target_x = x
            self._target_y = y
            self._target_yaw = yaw
            self._has_target = True
            self._nav_status = 'NAV'
            self._nav_start_time = time.time()
            self._last_progress_time = time.time()
            self._best_dist = float('inf')
            self._stuck_recovery_count = 0
            self._decision_state_id = 0
            self._last_cmd = Twist()
            self._yaw_only_latched = False
            self._yaw_only_logged = False
        self.get_logger().info(
            f'Target: ({x:.3f}, {y:.3f}, yaw={math.degrees(yaw):.1f}deg)'
            f'{" + visual_servo=" + self._servo_phase if self._servo_phase else ""}')

    def _cancel(self):
        with self._lock:
            self._has_target = False
            self._nav_status = 'IDLE'
            self._yaw_only_latched = False
            self._yaw_only_logged = False
        self._publish_stop()
        self._publish_phase('IDLE')
        self.get_logger().info('Navigation cancelled')

    # ═══════════════════════════════════
    #   Topic 回调
    # ═══════════════════════════════════

    def _on_goal_topic(self, msg: PoseStamped):
        x = msg.pose.position.x
        y = msg.pose.position.y
        yaw = _quat_to_yaw(msg.pose.orientation)
        self._set_target(x, y, yaw)

    def _on_cancel_topic(self, msg: String):
        self._cancel()

    def _on_servo_phase(self, msg: String):
        phase = msg.data.strip()
        self._servo_phase = phase
        if phase:
            self.get_logger().info(f'Visual servo phase set: {phase}')
        else:
            self.get_logger().info('Visual servo disabled')

    def _on_speed_limit(self, msg: Twist):
        max_lin = float(msg.linear.x)
        max_ang = float(msg.angular.z)

        if max_lin <= 0.0 and max_ang <= 0.0:
            self._max_lin = self._default_max_lin
            self._max_ang = self._default_max_ang
            self.get_logger().info(
                f'Speed limit reset: max_linear={self._max_lin:.2f}m/s, '
                f'max_angular={self._max_ang:.2f}rad/s')
            return

        if max_lin > 0.0:
            self._max_lin = max_lin
        if max_ang > 0.0:
            self._max_ang = max_ang
        self.get_logger().info(
            f'Speed limit updated: max_linear={self._max_lin:.2f}m/s, '
            f'max_angular={self._max_ang:.2f}rad/s')

    def _on_yaw_turn_direction(self, msg: Int8):
        direction = 1 if msg.data > 0 else -1 if msg.data < 0 else 0
        if direction == self._yaw_turn_direction:
            return
        self._yaw_turn_direction = direction
        label = '逆时针' if direction > 0 else '顺时针' if direction < 0 else '最短路径'
        self.get_logger().info(f'Yaw-only turn direction set: {label}')

    def _on_decision_state(self, msg: Int8):
        self._decision_state_id = msg.data

    def _apply_yaw_turn_direction(self, yaw_error: float) -> float:
        if self._yaw_turn_direction > 0 and yaw_error < 0.0:
            return yaw_error + 2.0 * math.pi
        if self._yaw_turn_direction < 0 and yaw_error > 0.0:
            return yaw_error - 2.0 * math.pi
        return yaw_error

    # ═══════════════════════════════════
    #   Action Server 回调
    # ═══════════════════════════════════

    def _on_goal_request(self, goal_request):
        return GoalResponse.ACCEPT

    def _on_cancel_request(self, goal_handle):
        self._cancel()
        return CancelResponse.ACCEPT

    def _execute_navigate(self, goal_handle):
        pose = goal_handle.request.pose.pose
        x = pose.position.x
        y = pose.position.y
        yaw = _quat_to_yaw(pose.orientation)
        self._set_target(x, y, yaw)

        while rclpy.ok():
            if goal_handle.is_cancel_requested:
                self._cancel()
                goal_handle.canceled()
                return NavigateToPose.Result()

            with self._lock:
                status = self._nav_status

            if status == 'ARRIVED':
                goal_handle.succeed()
                return NavigateToPose.Result()
            elif status in ('TIMEOUT', 'STUCK'):
                goal_handle.abort()
                return NavigateToPose.Result()
            elif status == 'IDLE':
                goal_handle.abort()
                return NavigateToPose.Result()

            feedback = NavigateToPose.Feedback()
            gx, gy, gyaw = self._get_pose()
            if gx is not None:
                feedback.current_pose.header.frame_id = 'game'
                feedback.current_pose.header.stamp = self.get_clock().now().to_msg()
                feedback.current_pose.pose.position.x = gx
                feedback.current_pose.pose.position.y = gy
                feedback.current_pose.pose.orientation.z = math.sin(gyaw / 2.0)
                feedback.current_pose.pose.orientation.w = math.cos(gyaw / 2.0)
            goal_handle.publish_feedback(feedback)

            time.sleep(0.1)

        goal_handle.abort()
        return NavigateToPose.Result()

    # ═══════════════════════════════════
    #   核心控制循环 (20Hz)
    # ═══════════════════════════════════

    def _control_tick(self):
        with self._lock:
            has_target = self._has_target
            target_x = self._target_x
            target_y = self._target_y
            target_yaw = self._target_yaw
            status = self._nav_status

        # 发布状态
        smsg = String()
        smsg.data = status
        self._status_pub.publish(smsg)

        if not has_target:
            return

        # 视觉伺服阶段: 由 processor_node 接管底盘控制
        if status == 'VISUAL_SERVO':
            self._tick_visual_servo()
            return

        # PID 导航阶段
        self._tick_pid_nav(target_x, target_y, target_yaw)

    # ─────────────────────────────────
    #   PID 导航 tick
    # ─────────────────────────────────

    def _tick_pid_nav(self, target_x, target_y, target_yaw):
        cur_x, cur_y, cur_yaw = self._get_pose()
        if cur_x is None:
            return

        ex = target_x - cur_x
        ey = target_y - cur_y
        dist = math.hypot(ex, ey)
        e_yaw_short = _normalize(target_yaw - cur_yaw)
        e_yaw = e_yaw_short
        now = time.time()

        # ── 超时检测 ──
        elapsed = now - self._nav_start_time
        tol = self._xy_tol
        pos_ok = dist <= tol
        yaw_ok = abs(e_yaw_short) <= self._yaw_tol

        if pos_ok and not self._yaw_only_latched:
            self._yaw_only_latched = True
            self._yaw_only_logged = False

        if self._yaw_only_latched:
            if yaw_ok:
                self.get_logger().info(
                    f'PID arrived after yaw-only ({cur_x:.3f}, {cur_y:.3f}), '
                    f'err: {dist:.3f}m yaw={math.degrees(e_yaw_short):.1f}deg')
                self._try_enter_visual_servo_or_finish()
                return

            if elapsed > self._wp_timeout:
                self.get_logger().warn(
                    f'Yaw-only timeout! dist={dist:.3f}m '
                    f'yaw_err={math.degrees(e_yaw_short):.1f}deg')
                self._finish('TIMEOUT')
                return

            if not self._yaw_only_logged:
                direction_label = ''
                if self._yaw_turn_direction != 0:
                    direction_label = (
                        ', forced='
                        + ('CCW' if self._yaw_turn_direction > 0 else 'CW'))
                self.get_logger().info(
                    f'XY reached ({dist:.3f}m), latch yaw-only; '
                    f'yaw_err={math.degrees(e_yaw_short):.1f}deg'
                    f'{direction_label}')
                self._yaw_only_logged = True

            cmd = Twist()
            self._last_cmd.linear.x = 0.0
            self._last_cmd.linear.y = 0.0
            cmd.linear.x = 0.0
            cmd.linear.y = 0.0
            e_yaw = self._apply_yaw_turn_direction(e_yaw_short)
            cmd.angular.z = max(-self._max_ang,
                                min(self._max_ang, self._kp_ang * e_yaw))
            self._publish_limited_cmd(cmd)
            self._last_progress_time = now
            return

        # 路径点超时
        if elapsed > self._wp_timeout:
            relaxed_tol = tol * self._relax_factor
            if dist <= relaxed_tol and yaw_ok:
                self.get_logger().info(
                    f'Timeout but within relaxed tol ({dist:.3f}m) -> trying visual servo')
                self._try_enter_visual_servo_or_finish()
                return
            else:
                self.get_logger().warn(
                    f'Waypoint timeout! dist={dist:.3f}m '
                    f'yaw_err={math.degrees(e_yaw):.1f}deg')
                self._finish('TIMEOUT')
                return

        # 进度超时
        if dist < self._best_dist - self._prog_delta:
            self._best_dist = dist
            self._last_progress_time = now
            self._stuck_recovery_count = 0
        elif now - self._last_progress_time > self._prog_timeout:
            self._stuck_recovery_count += 1
            if self._stuck_recovery_count >= 3:
                if dist <= tol * self._relax_factor and yaw_ok:
                    self.get_logger().info('Stuck but close -> trying visual servo')
                    self._try_enter_visual_servo_or_finish()
                    return
                self.get_logger().warn(
                    f'Stuck 3x, dist={dist:.3f}m '
                    f'yaw_err={math.degrees(e_yaw):.1f}deg -> STUCK')
                self._finish('STUCK')
                return
            self.get_logger().info(
                f'No progress {self._prog_timeout}s (dist={dist:.3f}m), '
                f'retry {self._stuck_recovery_count}/3')
            self._last_progress_time = now
            self._best_dist = dist

        # ── 到达判定 → 进入视觉伺服或直接完成 ──
        if pos_ok and yaw_ok:
            self.get_logger().info(
                f'PID arrived ({cur_x:.3f}, {cur_y:.3f}), '
                f'err: {dist:.3f}m yaw={math.degrees(e_yaw):.1f}deg')
            self._try_enter_visual_servo_or_finish()
            return

        # ── 计算速度指令 ──
        cmd = Twist()

        if dist > tol:
            speed = self._kp_lin * dist
            if dist < self._decel_dist:
                speed = self._max_lin * (dist / self._decel_dist)
            speed = max(self._min_lin, min(self._max_lin, speed))

            vx_world = (ex / dist) * speed
            vy_world = (ey / dist) * speed

            cos_yaw = math.cos(cur_yaw)
            sin_yaw = math.sin(cur_yaw)
            cmd.linear.x = vx_world * cos_yaw + vy_world * sin_yaw
            cmd.linear.y = -vx_world * sin_yaw + vy_world * cos_yaw
            cmd.angular.z = 0.0

        self._publish_limited_cmd(cmd)

    # ─────────────────────────────────
    #   视觉伺服交接
    # ─────────────────────────────────

    def _try_enter_visual_servo_or_finish(self):
        """PID 到达后: 有视觉伺服配置则进入, 否则直接 ARRIVED."""
        if self._servo_phase:
            self._enter_visual_servo()
        else:
            self._finish('ARRIVED')

    def _enter_visual_servo(self):
        """进入视觉伺服阶段: 停车, 通知 processor_node, 等待对齐."""
        self._publish_stop()
        with self._lock:
            self._nav_status = 'VISUAL_SERVO'
        self._servo_start_time = time.time()
        self._decision_state_id = 0

        self._publish_phase(self._servo_phase)
        self.get_logger().info(
            f'Entering visual servo: {self._servo_phase} '
            f'(timeout={self._servo_timeout}s)')

    def _tick_visual_servo(self):
        """视觉伺服阶段 tick: 等待 processor_node 报告对齐完成."""
        elapsed = time.time() - self._servo_start_time

        # processor_node 报告 ARRIVED → 对齐完成
        if self._decision_state_id == _VISUAL_SERVO_ARRIVED:
            self.get_logger().info(
                f'Visual servo ALIGNED ({elapsed:.1f}s)')
            self._publish_phase('IDLE')
            self._finish('ARRIVED')
            return

        # 视觉伺服超时 → 仍然算到达 (粗定位已完成)
        if elapsed > self._servo_timeout:
            self.get_logger().warn(
                f'Visual servo timeout ({self._servo_timeout}s), '
                f'accepting PID position as ARRIVED')
            self._publish_phase('IDLE')
            self._finish('ARRIVED')
            return

    # ─────────────────────────────────
    #   工具方法
    # ─────────────────────────────────

    def _finish(self, status):
        with self._lock:
            self._nav_status = status
            self._has_target = False
            self._yaw_only_latched = False
            self._yaw_only_logged = False
        self._publish_stop()

    def _publish_stop(self):
        self._last_cmd = Twist()
        self._cmd_pub.publish(Twist())

    @staticmethod
    def _slew(target, current, max_delta):
        if target > current + max_delta:
            return current + max_delta
        if target < current - max_delta:
            return current - max_delta
        return target

    def _publish_limited_cmd(self, cmd: Twist):
        limited = Twist()
        lin_delta = self._max_lin_accel * self._dt
        ang_delta = self._max_ang_accel * self._dt

        limited.linear.x = self._slew(cmd.linear.x, self._last_cmd.linear.x, lin_delta)
        limited.linear.y = self._slew(cmd.linear.y, self._last_cmd.linear.y, lin_delta)
        limited.angular.z = self._slew(cmd.angular.z, self._last_cmd.angular.z, ang_delta)

        self._last_cmd = limited
        self._cmd_pub.publish(limited)

    def _publish_phase(self, phase):
        msg = String()
        msg.data = phase
        self._phase_pub.publish(msg)

    # ═══════════════════════════════════
    #   公共方法 (供外部节点直接调用)
    # ═══════════════════════════════════

    def navigate_to(self, x, y, yaw, servo_phase=''):
        """设置导航目标, 可选视觉伺服阶段."""
        self._servo_phase = servo_phase
        self._set_target(x, y, yaw)

    def get_status(self):
        with self._lock:
            return self._nav_status

    def get_game_pose(self):
        return self._get_pose()


# ═══════════════════════════════════
#   工具函数
# ═══════════════════════════════════

def _normalize(angle):
    while angle > math.pi:
        angle -= 2.0 * math.pi
    while angle < -math.pi:
        angle += 2.0 * math.pi
    return angle


def _quat_to_yaw(q):
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main(args=None):
    rclpy.init(args=args)
    node = WaypointNavigator()
    executor = MultiThreadedExecutor()
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        node._publish_stop()
        node.destroy_node()
        rclpy.shutdown()
