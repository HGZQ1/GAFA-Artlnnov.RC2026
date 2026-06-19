"""
bridge_node.py
Nav2 /cmd_vel → 限速 + rad/s→deg转换 + 超时保护 → /serial/chassis_cmd

Nav2 输出: [vx m/s, vy m/s, omega rad/s]
STM32 期望: [vx m/s, vy m/s, 旋转角度 deg]  (正=逆时针 负=顺时针)

转换: rotation_deg = omega_rad_s × dt × (180/π)
"""
import math

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class CmdVelBridge(Node):

    def __init__(self):
        super().__init__('cmd_vel_bridge')

        self.declare_parameter('max_linear_vel', 1.5)
        self.declare_parameter('max_angular_vel', 3.14)
        self.declare_parameter('control_rate', 50.0)
        self.declare_parameter('cmd_vel_timeout', 0.5)

        self.max_lin = self.get_parameter('max_linear_vel').value
        self.max_ang = self.get_parameter('max_angular_vel').value
        self.cmd_vel_timeout = self.get_parameter('cmd_vel_timeout').value

        rate = self.get_parameter('control_rate').value
        self._dt = 1.0 / rate

        self._target_vx = 0.0
        self._target_vy = 0.0
        self._target_omega = 0.0
        self._last_cmd_time = self.get_clock().now()

        self.create_subscription(Twist, '/cmd_vel', self._on_cmd_vel, 10)
        self.chassis_pub = self.create_publisher(Twist, '/serial/chassis_cmd', 10)

        self.create_timer(self._dt, self._control_loop)

        self.get_logger().info(
            f'cmd_vel_bridge started | '
            f'max_lin: {self.max_lin} | max_ang: {self.max_ang} | '
            f'rate: {rate}Hz | '
            f'omega(rad/s) → rotation(deg/cycle)')

    def _on_cmd_vel(self, msg: Twist):
        self._target_vx = max(-self.max_lin, min(self.max_lin, msg.linear.x))
        self._target_vy = max(-self.max_lin, min(self.max_lin, msg.linear.y))
        self._target_omega = max(-self.max_ang, min(self.max_ang, msg.angular.z))
        self._last_cmd_time = self.get_clock().now()

    def _control_loop(self):
        dt = (self.get_clock().now() - self._last_cmd_time).nanoseconds / 1e9
        if dt > self.cmd_vel_timeout:
            self._target_vx = 0.0
            self._target_vy = 0.0
            self._target_omega = 0.0

        rotation_deg = self._target_omega * self._dt * (180.0 / math.pi)

        cmd = Twist()
        cmd.linear.x = self._target_vx
        cmd.linear.y = self._target_vy
        cmd.angular.z = rotation_deg
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
