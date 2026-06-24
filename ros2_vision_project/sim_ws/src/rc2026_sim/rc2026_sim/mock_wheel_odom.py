"""
mock_wheel_odom.py
模拟下位机轮式里程计, 以 50Hz 发布 /feedback/wheel_odom (Twist)
用于真机测试时没接串口的情况, 让 EKF 三源齐全能启动

数据全零 = 模拟机器人静止 (编码器没转)
FAST-LIO 仍能靠雷达提供真实位移, EKF 会以 FAST-LIO 为主
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class MockWheelOdom(Node):

    def __init__(self):
        super().__init__('mock_wheel_odom')
        self._pub = self.create_publisher(Twist, '/feedback/wheel_odom', 10)
        self.create_timer(0.02, self._tick)  # 50Hz
        self.get_logger().info('Mock wheel odom started (50Hz, all zeros)')

    def _tick(self):
        self._pub.publish(Twist())


def main(args=None):
    rclpy.init(args=args)
    node = MockWheelOdom()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
