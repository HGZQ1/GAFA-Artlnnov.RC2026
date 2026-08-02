"""
bridge_node.py
多源速度指令汇聚 + 优先级仲裁 + 单位转换 → /serial/chassis_cmd

优先级 (高→低):
  1. /fine_align/cmd  — USB相机精对齐 (角速度 rad/s)
  2. /dock_align/cmd  — ArUco深度相机对齐 (角速度 rad/s)
  3. /cmd_vel         — 路点导航 (角速度 rad/s)

超时: 某源超过 cmd_vel_timeout 秒没发新消息则视为不活跃, 降级到下一优先级.
/cmd_vel 输入:
  linear.x/y = m/s
  angular.z  = rad/s

/serial/chassis_cmd 输出:
  linear.x/y = m/s
  angular.z  = deg/s
"""
import math
import time

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('max_linear_vel',  1.5)
        self.declare_parameter('max_angular_vel', 3.14)
        self.declare_parameter('control_rate',    50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.max_lin         = self.get_parameter('max_linear_vel').value
        self.max_ang         = self.get_parameter('max_angular_vel').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value
        rate                 = self.get_parameter('control_rate').value
        self._dt             = 1.0 / rate

        # 每个优先级源: (vx, vy, omega_rad_s, last_recv_time)
        _zero = (0.0, 0.0, 0.0, 0.0)
        self._src_fine  = _zero   # priority 1: /fine_align/cmd
        self._src_dock  = _zero   # priority 2: /dock_align/cmd
        self._src_nav   = _zero   # priority 3: /cmd_vel
        self._need_stop = False   # 有源刚切空时发一次停车包

        self.create_subscription(Twist, '/fine_align/cmd', self._on_fine,  10)
        self.create_subscription(Twist, '/dock_align/cmd', self._on_dock,  10)
        self.create_subscription(Twist, '/cmd_vel',        self._on_nav,   10)
        self.chassis_pub = self.create_publisher(Twist, '/serial/chassis_cmd', 10)

        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'cmd_vel_bridge started | '
            f'max_lin: {self.max_lin} | max_ang: {self.max_ang} | '
            f'rate: {rate}Hz | '
            f'sources: fine_align > dock_align > cmd_vel')

    def _clamp(self, v, limit):
        return max(-limit, min(limit, v))

    def _store(self, msg: Twist):
        return (
            self._clamp(msg.linear.x,  self.max_lin),
            self._clamp(msg.linear.y,  self.max_lin),
            self._clamp(msg.angular.z, self.max_ang),
            time.monotonic(),
        )

    def _on_fine(self, msg: Twist):
        self._src_fine = self._store(msg)

    def _on_dock(self, msg: Twist):
        self._src_dock = self._store(msg)

    def _on_nav(self, msg: Twist):
        self._src_nav = self._store(msg)

    def _active(self, src):
        return (time.monotonic() - src[3]) < self.cmd_vel_timeout

    def _control_loop(self):
        # 按优先级选取活跃源
        if self._active(self._src_fine):
            vx, vy, omega, _ = self._src_fine
        elif self._active(self._src_dock):
            vx, vy, omega, _ = self._src_dock
        elif self._active(self._src_nav):
            vx, vy, omega, _ = self._src_nav
        else:
            # 无活跃源时不发布，避免覆盖 processor_node / game_controller 的直发指令
            if self._need_stop:
                cmd = Twist()
                self.chassis_pub.publish(cmd)
                self._need_stop = False
            return

        self._need_stop = True   # 本次有命令，下次切空时发一次停车包

        cmd = Twist()
        cmd.linear.x = vx
        cmd.linear.y = vy
        cmd.angular.z = omega * (180.0 / math.pi)
        self.chassis_pub.publish(cmd)


def main(args=None):
    rclpy.init(args=args)
    node = CmdVelBridge()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
