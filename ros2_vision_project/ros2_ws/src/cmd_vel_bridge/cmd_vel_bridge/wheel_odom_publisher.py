"""
wheel_odom_publisher.py
将下位机轮式里程计数据转换为标准 nav_msgs/Odometry 格式
供 AMCL / Nav2 使用

数据来源:
  /feedback/wheel_odom (Twist) — STM32累积位姿 + 速度

发布:
  /odom/wheel (nav_msgs/Odometry) — 标准里程计消息
  odom → base_link TF (可选)
"""
import math
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from tf2_ros import TransformBroadcaster


class WheelOdomPublisher(Node):

    def __init__(self):
        super().__init__('wheel_odom_publisher')

        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_link')
        self.declare_parameter('publish_tf', False)

        self.odom_frame = self.get_parameter('odom_frame').value
        self.base_frame = self.get_parameter('base_frame').value
        self.publish_tf = self.get_parameter('publish_tf').value

        self._pos_x = 0.0
        self._pos_y = 0.0
        self._yaw = 0.0
        self._vx = 0.0
        self._vy = 0.0
        self._omega = 0.0

        self.odom_pub = self.create_publisher(Odometry, '/odom/wheel', 50)

        if self.publish_tf:
            self.tf_broadcaster = TransformBroadcaster(self)

        self.create_subscription(
            Twist, '/feedback/wheel_odom', self._on_wheel_odom, 10)

        self.get_logger().info(
            f'wheel_odom_publisher started | '
            f'publish_tf: {self.publish_tf}')

    def _on_wheel_odom(self, msg: Twist):
        self._pos_x = msg.linear.x
        self._pos_y = msg.linear.y
        self._yaw = msg.angular.z
        self._vx = msg.linear.z
        self._vy = msg.angular.x
        self._omega = msg.angular.y
        self._publish_odom()

    def _publish_odom(self):
        now = self.get_clock().now().to_msg()

        odom = Odometry()
        odom.header.stamp = now
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self._pos_x
        odom.pose.pose.position.y = self._pos_y
        odom.pose.pose.position.z = 0.0
        odom.pose.pose.orientation = self._yaw_to_quaternion(self._yaw)

        odom.pose.covariance = [
            0.01, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.01, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  1e6,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  1e6, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 1e6, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.05,
        ]

        odom.twist.twist.linear.x = self._vx
        odom.twist.twist.linear.y = self._vy
        odom.twist.twist.angular.z = self._omega

        odom.twist.covariance = [
            0.01, 0.0,  0.0,  0.0, 0.0, 0.0,
            0.0,  0.01, 0.0,  0.0, 0.0, 0.0,
            0.0,  0.0,  1e6,  0.0, 0.0, 0.0,
            0.0,  0.0,  0.0,  1e6, 0.0, 0.0,
            0.0,  0.0,  0.0,  0.0, 1e6, 0.0,
            0.0,  0.0,  0.0,  0.0, 0.0, 0.02,
        ]

        self.odom_pub.publish(odom)

        if self.publish_tf:
            t = TransformStamped()
            t.header.stamp = now
            t.header.frame_id = self.odom_frame
            t.child_frame_id = self.base_frame
            t.transform.translation.x = self._pos_x
            t.transform.translation.y = self._pos_y
            t.transform.translation.z = 0.0
            t.transform.rotation = odom.pose.pose.orientation
            self.tf_broadcaster.sendTransform(t)

    @staticmethod
    def _yaw_to_quaternion(yaw: float) -> Quaternion:
        q = Quaternion()
        q.x = 0.0
        q.y = 0.0
        q.z = math.sin(yaw / 2.0)
        q.w = math.cos(yaw / 2.0)
        return q


def main(args=None):
    rclpy.init(args=args)
    node = WheelOdomPublisher()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
