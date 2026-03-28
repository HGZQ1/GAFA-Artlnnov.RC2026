"""
processor_node.py
决策处理 ROS2 主节点
"""
import rclpy
from rclpy.node import Node
import numpy as np
import math

from geometry_msgs.msg import PointStamped, Twist
from std_msgs.msg       import String
from sensor_msgs.msg    import CameraInfo

from .robot_decision import RobotDecision


class ProcessorNode(Node):

    def __init__(self):
        super().__init__('decision_processor_node')

        # ── 参数声明 ──
        self.declare_parameter('wheel_diameter_m',    0.096)
        self.declare_parameter('track_width_m',       0.28)
        self.declare_parameter('stop_distance_m',     0.20)
        self.declare_parameter('align_threshold_deg', 5.0)
        self.declare_parameter('pick_duration_s',     3.0)
        self.declare_parameter('conf_threshold',      0.5)

        # ── 初始化决策模块 ──
        self.decision = RobotDecision(
            wheel_diameter_m    = self.get_parameter('wheel_diameter_m').value,
            track_width_m       = self.get_parameter('track_width_m').value,
            stop_distance_m     = self.get_parameter('stop_distance_m').value,
            align_threshold_deg = self.get_parameter('align_threshold_deg').value,
            pick_duration_s     = self.get_parameter('pick_duration_s').value,
            conf_threshold      = self.get_parameter('conf_threshold').value,
        )

        # ── 相机内参 ──
        # D435i 真实内参（从 ros2 topic echo /camera/camera/color/camera_info 获取）
        self.camera_matrix = np.array([
            [913.461, 0.0,     650.967],
            [0.0,     913.483, 381.094],
            [0.0,     0.0,     1.0    ]
        ], dtype=float)
        self.get_logger().info('相机内参已加载（真实值）')
        self.create_subscription(
            CameraInfo,
            '/camera/camera/color/camera_info',
            self._on_camera_info,
            10)

        # ── 订阅原始目标话题 ──
        self.create_subscription(
            PointStamped,
            '/vision/raw_target',
            self._on_raw_target,
            10)

        # ── 发布底盘指令 ──
        self.chassis_pub = self.create_publisher(
            Twist, '/serial/chassis_cmd', 10)

        # ── 发布决策状态（调试用）──
        self.state_pub = self.create_publisher(
            String, '/decision/state', 10)

        # 最新指令缓存（定时器持续发布）
        self._latest_cmd   = Twist()
        self._last_seen_t  = 0.0       # 上次看到目标的时间
        self.TARGET_TIMEOUT = 0.5      # 超过 0.5s 无目标才算丢失

        # ── 定时器：10Hz 发布指令 ──
        self.create_timer(0.1, self._timer_cb)

        self.get_logger().info('decision_processor_node 已启动')

    # ──────────────────────────────────────────────
    def _on_camera_info(self, msg: CameraInfo):
        if self.camera_matrix is None:
            k = msg.k
            self.camera_matrix = np.array(k, dtype=float).reshape(3, 3)
            self.get_logger().info('已获取相机内参矩阵')

    # ──────────────────────────────────────────────
    def _on_raw_target(self, msg: PointStamped):
        """收到视觉目标数据，送入决策模块"""
        if self.camera_matrix is None:
            return

        # 解析置信度
        try:
            parts      = msg.header.frame_id.split(':')
            confidence = float(parts[2]) if len(parts) > 2 else 0.0
            class_id   = int(parts[0])   if len(parts) > 0 else 0
            class_name = parts[1]        if len(parts) > 1 else ''
        except Exception:
            confidence = 0.0
            class_id   = 0
            class_name = ''

        dist = float(msg.point.z)

        if confidence < self.decision.conf_thr or dist <= 0:
            return

        # 记录最近看到目标的时间
        import time
        self._last_seen_t = time.time()

        # 构造 detections / distances 格式送入 step()
        # bbox 用点坐标还原（仅用于 pixel_to_3d_point，已在 detector 算好）
        # 这里直接用已算好的相机坐标，绕过 bbox 计算
        cam_x = float(msg.point.x)
        cam_y = float(msg.point.y)

        # 直接调用内部各层（绕过 step() 中重复的 bbox 计算）
        from .kalman_filter  import camera_to_robot, is_valid_detection

        robot_x, robot_y = camera_to_robot(cam_x, cam_y, dist)

        # 跳变检测
        new_pos = (robot_x, robot_y)
        if not is_valid_detection(new_pos, self.decision._last_pos):
            self.get_logger().debug('跳变检测：本帧丢弃')
            return

        self.decision._last_pos = new_pos

        # 卡尔曼滤波
        rx, ry = self.decision.kalman.update(robot_x, robot_y)

        # 连续帧确认
        confirmed = self.decision.confirmation.update(True)

        # 运动规划
        plan = self.decision.planner.plan(rx, ry)

        # 状态机
        cmd_dict = self.decision._state_machine(confirmed, plan)

        # 转换并缓存指令
        self._latest_cmd = self._dict_to_twist(cmd_dict)

        # 发布调试状态
        turn  = cmd_dict.get('turn_angle',   0.0)
        fwd   = cmd_dict.get('drive_wheels', 0.0)
        pick  = int(cmd_dict.get('pickup_action', 0))
        state_msg = String()
        state_msg.data = (
            f"state={self.decision.state_name} | "
            f"dist={dist:.2f}m | "
            f"turn={turn:.1f}° | "
            f"wheels={fwd:.2f}圈 | "
            f"pickup={pick}"
        )
        self.state_pub.publish(state_msg)
        self.get_logger().info(state_msg.data)

    # ──────────────────────────────────────────────
    def _timer_cb(self):
        """
        10Hz 定时发布最新指令
        仅在目标超时丢失时更新为搜索指令
        不再调用状态机（避免干扰状态转移）
        """
        import time
        now = time.time()

        # 目标超时：更新确认器，切换回搜索状态
        if (now - self._last_seen_t) > self.TARGET_TIMEOUT:
            was_confirmed = self.decision.confirmation.update(False)
            if not was_confirmed and self.decision.state_name != 'SEARCHING':
                # 目标真正丢失，重置状态机
                from .robot_decision import RobotState
                self.decision.state      = RobotState.SEARCHING
                self.decision.kalman.reset()
                self.decision._last_pos  = None
                self.get_logger().info('目标丢失，回到 SEARCHING')

            # 发搜索旋转指令
            search_cmd = Twist()
            search_cmd.angular.y = 1.0
            self._latest_cmd = search_cmd

        # 持续发布当前最新指令
        self.chassis_pub.publish(self._latest_cmd)

    # ──────────────────────────────────────────────
    def _dict_to_twist(self, cmd: dict) -> Twist:
        msg = Twist()
        msg.angular.z = float(cmd.get('turn_angle',    0.0))
        msg.linear.z  = float(cmd.get('turn_wheels',   0.0))
        msg.linear.x  = float(cmd.get('forward_dist',  0.0))
        msg.linear.y  = float(cmd.get('drive_wheels',  0.0))
        msg.angular.x = float(cmd.get('pickup_action', 0.0))
        msg.angular.y = float(cmd.get('search_rotate', 0.0))
        return msg


def main(args=None):
    rclpy.init(args=args)
    node = ProcessorNode()
    rclpy.spin(node)
    rclpy.shutdown()


if __name__ == '__main__':
    main()