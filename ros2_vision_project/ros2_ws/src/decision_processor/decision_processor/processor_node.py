"""
processor_node.py
决策处理 ROS2 主节点
"""
import time
import rclpy
from rclpy.node import Node
import numpy as np
import math
import os

from geometry_msgs.msg import PointStamped, Twist, Vector3
from std_msgs.msg       import String, Int8, Float32MultiArray, UInt8
from sensor_msgs.msg    import CameraInfo

from .robot_decision import RobotDecision, RobotState
from .tf_manager     import TFManager
from .kalman_filter  import is_valid_detection
from .scenarios.scenario_wuguan  import ScenarioWuguan
from .scenarios.scenario_meilin  import ScenarioMeilin
from .imu_processor  import SlopeLevel
from .config import (
    WUGUAN_TOTAL_WEAPONS,
    WUGUAN_CLASS_LABELS as WUGUAN_CLASS_ABBR,
    MEILIN_CLASS_LABELS as MEILIN_CLASS_ABBR,
)

MODEL_SCENARIO_MAP = {
    'wuqi.pt':  ScenarioWuguan,
    'best.pt':  ScenarioWuguan,
    'kfs.pt':   ScenarioMeilin,
}



class ProcessorNode(Node):

    def __init__(self):
        super().__init__('decision_processor_node')

        self.declare_parameter('wheel_diameter_m',     0.096)
        self.declare_parameter('track_width_m',       0.28)
        self.declare_parameter('stop_distance_m',     0.20)
        self.declare_parameter('align_threshold_deg', 5.0)
        self.declare_parameter('pick_duration_s',     10.0)
        self.declare_parameter('conf_threshold',      0.5)

        self.tf_manager = TFManager(self)

        self.decision = RobotDecision(
            wheel_diameter_m    = self.get_parameter('wheel_diameter_m').value,
            track_width_m       = self.get_parameter('track_width_m').value,
            stop_distance_m     = self.get_parameter('stop_distance_m').value,
            align_threshold_deg = self.get_parameter('align_threshold_deg').value,
            pick_duration_s     = self.get_parameter('pick_duration_s').value,
            conf_threshold      = self.get_parameter('conf_threshold').value,
        )

        self.current_scenario = ScenarioWuguan()
        self.current_model    = 'best.pt'

        self._pitch_deg    = 0.0
        self._roll_deg     = 0.0
        self._yaw_rate     = 0.0
        self._is_climbing  = 0.0
        self._slope_level  = SlopeLevel.FLAT

        self._odom_x     = 0.0
        self._odom_y     = 0.0
        self._odom_theta = 0.0
        self._odom_dist  = 0.0

        self._nav_move_mode     = 0
        self._nav_height_diff   = 0.0
        self._nav_speed_factor  = 1.0
        self._nav_torque_factor = 0.5
        self._nav_dist_to_next  = 0.0
        self._nav_current_block = 0
        self._nav_next_block    = 0

        # 当前帧目标信息（用于统一输出）
        self._cur_class_id   = -1
        self._cur_class_name = ''
        self._cur_confidence = 0.0

        self.camera_matrix = np.array([
            [913.461, 0.0, 650.967],
            [0.0, 913.483, 381.094],
            [0.0, 0.0,         1.0]
        ], dtype=float)

        self.get_logger().info('相机内参已加载（真实值）')
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._on_camera_info,
            10)
        self.create_subscription(CameraInfo,
            '/camera/camera/color/camera_info', self._on_camera_info, 10)

        self.create_subscription(PointStamped,
            '/vision/raw_target', self._on_raw_target, 10)
        self.create_subscription(String,
            '/vision/current_model', self._on_model_changed, 10)
        self.create_subscription(Float32MultiArray,
            '/imu/processed', self._on_imu, 10)
        self.create_subscription(Float32MultiArray,
            '/odom/fused', self._on_odom, 10)
        self.create_subscription(Float32MultiArray,
            '/meilin/nav_state', self._on_nav_state, 10)

        self.chassis_pub    = self.create_publisher(Twist,    '/serial/chassis_cmd', 10)
        self.climb_pub      = self.create_publisher(Twist,    '/serial/climb_cmd', 10)
        self.slope_info_pub = self.create_publisher(Vector3,  '/serial/slope_info', 10)
        self.meilin_cmd_pub = self.create_publisher(Twist,    '/serial/meilin_cmd', 10)
        self.arm_pub        = self.create_publisher(PointStamped, '/arm/target_pos', 10)
        self.state_pub      = self.create_publisher(String,   '/decision/state', 10)
        self.scene_pub      = self.create_publisher(String,   '/decision/scene', 10)

        self._latest_cmd  = Twist()
        self._last_seen_t = time.time()
        self.TARGET_TIMEOUT = 0.5

        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info(
            f'decision_processor started | '
            f'scene: {type(self.current_scenario).__name__} | '
            f'stop_dist: {self.decision.stop_dist:.2f}m | '
            f'pick_timeout: {self.decision.pick_duration:.0f}s | '
            f'align_thr: {self.decision.align_thr_deg:.1f}°')

    # ── 传感器回调 ──

    def _on_camera_info(self, msg: CameraInfo):
        self.camera_matrix = np.array(msg.k, dtype=float).reshape(3, 3)

    def _on_imu(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            self._pitch_deg   = msg.data[0]
            self._roll_deg    = msg.data[1]
            self._yaw_rate    = msg.data[2]
            self._is_climbing = msg.data[3]
            self._slope_level = int(msg.data[4])
            self.decision.planner.update_slope(self._pitch_deg, self._slope_level)
            self._publish_slope_info()

    def _on_odom(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            self._odom_x     = msg.data[0]
            self._odom_y     = msg.data[1]
            self._odom_theta = msg.data[2]
            self._odom_dist  = msg.data[4]

    def _on_nav_state(self, msg: Float32MultiArray):
        if len(msg.data) >= 7:
            self._nav_current_block = int(msg.data[0])
            self._nav_next_block    = int(msg.data[1])
            self._nav_move_mode     = int(msg.data[2])
            self._nav_height_diff   = msg.data[3]
            self._nav_dist_to_next  = msg.data[4]
            self._nav_speed_factor  = msg.data[5]
            self._nav_torque_factor = msg.data[6]
            if isinstance(self.current_scenario, ScenarioMeilin):
                self._publish_meilin_cmd()

    # ── 模型联动 ──

    def _on_model_changed(self, msg: String):
        model_name = os.path.basename(msg.data.strip())
        if model_name == self.current_model:
            return
        scenario_cls = MODEL_SCENARIO_MAP.get(model_name)
        if scenario_cls is None:
            self.get_logger().warn(f'unknown model: {model_name}')
            return
        old = type(self.current_scenario).__name__
        self.current_scenario = scenario_cls()
        self.current_model    = model_name
        self.decision.reset_all()

        # ═══ 新增：重置梅林绕道状态 ═══
        self._meilin_current_path = 'MID'
        self._meilin_detour_done  = False
        self.get_logger().info(f'scene switch: {old} -> {type(self.current_scenario).__name__}')
        scene_msg = String()
        scene_msg.data = type(self.current_scenario).__name__
        self.scene_pub.publish(scene_msg)

    # ── 类别缩写 ──

    def _class_abbr(self, class_id: int) -> str:
        if isinstance(self.current_scenario, ScenarioMeilin):
            return MEILIN_CLASS_ABBR.get(class_id, f'unk_{class_id}')
        return WUGUAN_CLASS_ABBR.get(class_id, f'unk_{class_id}')

    # ── 目标处理 ──

    def _on_raw_target(self, msg: PointStamped):
        try:
            parts      = msg.header.frame_id.split(':')
            class_id   = int(parts[0])   if len(parts) > 0 else 0
            class_name = parts[1]        if len(parts) > 1 else ''
            confidence = float(parts[2]) if len(parts) > 2 else 0.0
        except Exception:
            confidence, class_id, class_name = 0.0, 0, ''

        cam_x = float(msg.point.x)
        cam_y = float(msg.point.y)
        dist  = float(msg.point.z)
        if dist <= 0:
            return

        result = self.tf_manager.transform_camera_to_base(
            cam_x, cam_y, dist, msg.header.stamp)
        if result is None:
            return
        base_x, base_y, base_z = result

        if abs(self._pitch_deg) > 3.0:
            dist_h = dist * math.cos(math.radians(self._pitch_deg))
        else:
            dist_h = dist

        action = self.current_scenario.evaluate_target(
            class_id, confidence, dist_h, base_x, base_y)

        if action == 'AVOID':
            # 梅林场景：触发绕道
            if isinstance(self.current_scenario, ScenarioMeilin):
                danger = getattr(self.current_scenario, 'last_danger_block', None)
                self._trigger_meilin_detour(danger, class_name, dist_h)
            else:
                self.decision._reset_tracking()
                self.decision.state = RobotState.SEARCHING
                self.get_logger().warn(
                    f'!! AVOID | cls: {self._class_abbr(class_id)} | '
                    f'dist: {dist_h:.2f}m')
            return
        if action == 'IGNORE':
            return

        self._last_seen_t    = time.time()
        self._cur_class_id   = class_id
        self._cur_class_name = class_name
        self._cur_confidence = confidence

        new_pos = (base_x, base_y)
        if not is_valid_detection(new_pos, self.decision._last_pos):
            return
        self.decision._last_pos = new_pos

        rx, ry = self.decision.kalman.update(base_x, base_y)

        arm_result = self.tf_manager.transform_base_to_arm(base_x, base_y, base_z)
        arm_distance = math.sqrt(sum(a**2 for a in arm_result)) if arm_result else dist_h


        # ═══ 对齐角度：直接从相机水平偏移计算，不经过TF ═══
        # cam_x > 0 = 物体在相机画面右侧 → 机器人需右转 → 角度为负
        # cam_x < 0 = 物体在相机画面左侧 → 机器人需左转 → 角度为正
        # cam_x ≈ 0 = 物体在画面中心     → 已对齐        → 角度≈0
        align_angle = math.degrees(math.atan2(-cam_x, dist))  #cam_x 是物体相对相机光心的水平偏移

        cmd_dict = self.decision.update(
            detected=True, base_x=rx, base_y=ry,
            distance=dist_h, align_angle=align_angle,
            arm_distance=arm_distance)

        chassis_twist = self._build_chassis_cmd(cmd_dict)
        self._latest_cmd = chassis_twist

        plan = self.decision.planner.plan(rx, ry)
        self._publish_climb_cmd(plan)

        if self.decision.state == RobotState.PICKING:
            self._publish_arm_target(base_x, base_y, base_z, msg.header.stamp)

        # ═══ 统一输出 ═══
        self._print_status_line()
        self._print_event()

    # ──────────────────────────────────────────────
    #   统一输出格式
    # ──────────────────────────────────────────────

    def _print_status_line(self):
        """
        单行输出所有关键状态，格式：
        [STATE] cls: W_punch | conf: 0.87 | dist: 0.65m | ang: +5.2° | pitch: -1.3° | yaw_r: +0.5°/s | picked: 1/3 | scene: Wuguan
        """
        d = self.decision
        is_meilin = isinstance(self.current_scenario, ScenarioMeilin)
        scene_tag = 'Meilin' if is_meilin else 'Wuguan'
        cls_tag   = self._class_abbr(self._cur_class_id)

        # 基础行
        line = (
            f'[{d.state_name:9s}] '
            f'cls: {cls_tag:8s} | '
            f'conf: {self._cur_confidence:.2f} | '
            f'dist: {d.target_distance:.2f}m | '
            f'ang: {d.align_angle_deg:+6.1f}\u00b0 | '
            f'pitch: {self._pitch_deg:+5.1f}\u00b0 | '
            f'yaw_r: {self._yaw_rate:+6.1f}\u00b0/s | '
            f'picked: {d.pick_count}/{WUGUAN_TOTAL_WEAPONS} | '
            f'scene: {scene_tag}'
        )

        # 状态特有附加信息
        if d.state == RobotState.ARRIVED:
            line += f' | settle: {d.settle_remain:.1f}s'
        elif d.state == RobotState.PICKING:
            line += (
                f' | pick_t: {d.pick_elapsed:.1f}/{d.pick_duration:.0f}s'
                f' | arm_d: {d.arm_target_dist:.3f}m'
            )
        elif d.state == RobotState.MOVING:
            remain = max(0, d.target_distance - d.stop_dist)
            line += f' | remain: {remain:.2f}m'

        self.get_logger().info(line)

        # 也发布到话题供外部监控
        state_msg = String()
        state_msg.data = line
        self.state_pub.publish(state_msg)

    def _print_event(self):
        """事件信号：仅在状态切换时输出一次"""
        ev = self.decision.event
        if not ev:
            return

        d = self.decision
        cls_tag = self._class_abbr(self._cur_class_id)

        if ev == 'ALL_DONE':
            self.get_logger().info(
                f'===== ALL {WUGUAN_TOTAL_WEAPONS} WEAPONS PICKED ===== '
                f'total: {d.pick_count}')
        elif ev == 'PICK_DONE':
            self.get_logger().info(
                f'>> PICK_DONE | cls: {cls_tag} | '
                f'count: {d.pick_count}/{WUGUAN_TOTAL_WEAPONS}')
        elif ev == 'PICK_START':
            self.get_logger().info(
                f'>> PICK_START | cls: {cls_tag} | '
                f'arm_dist: {d.arm_target_dist:.3f}m')
        elif ev == 'TARGET_LOCKED':
            self.get_logger().info(
                f'>> TARGET_LOCKED | cls: {cls_tag} | '
                f'dist: {d.target_distance:.2f}m | '
                f'ang: {d.align_angle_deg:+.1f}\u00b0')
        elif ev == 'ALIGNED':
            self.get_logger().info(
                f'>> ALIGNED | ang: {d.align_angle_deg:+.1f}\u00b0 -> MOVING')
        elif ev == 'ARRIVED':
            self.get_logger().info(
                f'>> ARRIVED | dist: {d.target_distance:.2f}m <= '
                f'{d.stop_dist:.2f}m | settling...')
        elif ev == 'TARGET_LOST':
            self.get_logger().info('>> TARGET_LOST -> SEARCHING')
        elif ev == 'REALIGN':
            self.get_logger().info(
                f'>> REALIGN | ang: {d.align_angle_deg:+.1f}\u00b0 too large')

    # ── 指令构建 ──

    def _build_chassis_cmd(self, cmd_dict: dict) -> Twist:
        msg = Twist()
        msg.linear.x  = float(cmd_dict.get('forward_dist',  0.0))
        msg.linear.y  = float(cmd_dict.get('drive_wheels',  0.0))
        msg.linear.z  = float(cmd_dict.get('turn_wheels',   0.0))
        msg.angular.x = float(cmd_dict.get('pickup_action', 0.0))
        msg.angular.y = float(cmd_dict.get('search_rotate', 0.0))
        msg.angular.z = float(cmd_dict.get('turn_angle',    0.0))
        return msg

    def _publish_climb_cmd(self, plan):
        is_meilin = isinstance(self.current_scenario, ScenarioMeilin)
        speed  = min(plan.speed_factor, self._nav_speed_factor) if is_meilin else plan.speed_factor
        torque = self._nav_torque_factor if (is_meilin and self._nav_speed_factor < plan.speed_factor) else plan.torque_factor
        msg = Twist()
        msg.linear.x  = float(plan.climb_mode)
        msg.linear.y  = float(self._pitch_deg)
        msg.linear.z  = float(speed)
        msg.angular.x = float(torque)
        msg.angular.y = float(self._nav_move_mode if is_meilin else 0)
        msg.angular.z = float(self._nav_height_diff if is_meilin else 0)
        self.climb_pub.publish(msg)

    def _publish_slope_info(self):
        msg = Vector3()
        msg.x = float(self._pitch_deg)
        msg.y = float(self._odom_dist)
        msg.z = float(self._nav_height_diff)
        self.slope_info_pub.publish(msg)

    def _publish_meilin_cmd(self):
        msg = Twist()
        msg.linear.x  = float(self._nav_next_block)
        msg.linear.y  = float(self._nav_move_mode)
        msg.linear.z  = float(self._pitch_deg)
        msg.angular.x = float(self._nav_height_diff)
        msg.angular.y = 0.0
        self.meilin_cmd_pub.publish(msg)

    def _publish_arm_target(self, base_x, base_y, base_z, stamp):
        arm = self.tf_manager.transform_base_to_arm(base_x, base_y, base_z)
        if arm is None:
            return
        arm_msg = PointStamped()
        arm_msg.header.stamp    = stamp
        arm_msg.header.frame_id = 'arm_base_link'
        arm_msg.point.x, arm_msg.point.y, arm_msg.point.z = arm
        self.arm_pub.publish(arm_msg)

    # ── 超时定时器 ──

    def _timer_cb(self):
        now = time.time()
        if (now - self._last_seen_t) > self.TARGET_TIMEOUT:
            cmd_dict = self.decision.update(
                detected=False, base_x=0.0, base_y=0.0,
                distance=0.0, align_angle=0.0,
                arm_distance=0.0)

            if self.decision.state == RobotState.SEARCHING:
                self._latest_cmd = self._build_chassis_cmd(cmd_dict)
            elif self.decision.state in (RobotState.PICKING, RobotState.ARRIVED):
                pass  # 不打断
            else:
                self._latest_cmd = self._build_chassis_cmd(cmd_dict)

            # 超时时也输出事件（如 TARGET_LOST）
            if self.decision.event:
                self._print_event()

        self.chassis_pub.publish(self._latest_cmd)

    # ──────────────────────────────────────────────
    #   梅林绕道逻辑
    # ──────────────────────────────────────────────

    def _trigger_meilin_detour(self, danger, class_name, distance):
        """
        触发梅林绕道：发送原地旋转90度指令，切换到备用路径
        danger: (block_num, label) 或 None
        """
        from .config import (
            MEILIN_PATH_LEFT, MEILIN_PATH_MID, MEILIN_PATH_RIGHT,
            MEILIN_DETOUR_TURN_DEG,
        )

        if not hasattr(self, '_meilin_current_path'):
            self._meilin_current_path = 'MID'
            self._meilin_detour_done = False

        # 已经绕过道就不再重复触发
        if self._meilin_detour_done:
            self.get_logger().warn(
                f'!! DANGER detected again on detour path: {class_name} | '
                f'stopping robot !!')
            self.decision._reset_tracking()
            self.decision.state = RobotState.SEARCHING
            return

        old_path = self._meilin_current_path
        block_num = danger[0] if danger else None

        # 决定向左还是向右绕
        # 简化策略：中路→左路 优先；左路→右路；右路→左路
        if old_path == 'MID':
            new_path = 'LEFT'
            turn_direction = +1   # 左转为正
        elif old_path == 'LEFT':
            new_path = 'RIGHT'
            turn_direction = -1
        else:  # RIGHT
            new_path = 'LEFT'
            turn_direction = +1

        self._meilin_current_path = new_path
        self._meilin_detour_done  = True

        # 发送原地旋转90度指令
        turn_cmd = Twist()
        turn_cmd.angular.z = float(turn_direction * MEILIN_DETOUR_TURN_DEG)
        # turn_wheels 由下位机计算，这里只传角度
        self.chassis_pub.publish(turn_cmd)

        self.get_logger().warn(
            f'>> MEILIN DETOUR: {old_path} -> {new_path} | '
            f'danger={class_name} (block {block_num}) | '
            f'turn {turn_direction * MEILIN_DETOUR_TURN_DEG:+.0f}°')

        # 重置状态机，继续搜索新路径上的真方块
        self.decision._reset_tracking()
        self.decision.state = RobotState.SEARCHING


def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()