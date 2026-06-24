"""
odom_drift_injector.py
仿真用累积里程计漂移注入节点

用途:
  真实轮式里程计存在打滑/累积误差, 但 Gazebo 的 planar_move 插件直接
  按物理引擎真值发布 odom→base_footprint TF (零误差)。
  本节点订阅 planar_move 发布的真值 /odom, 按行驶距离/转角累积比例误差,
  重新发布一份"带漂移"的 odom→base_footprint TF, 用于测试 waypoint_nav
  在存在累积定位误差时的导航/校正表现。

使用前提:
  urdf 需以 publish_odom_tf:=false 启动 (避免和 planar_move 的 TF 冲突)
"""
import math

import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from std_msgs.msg import Float64
import tf2_ros
from geometry_msgs.msg import TransformStamped


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_from_yaw(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (qz, qw)


class OdomDriftInjector(Node):

    def __init__(self):
        super().__init__('odom_drift_injector')

        self.declare_parameter('drift_linear', 0.03)   # 每米行驶额外漂移比例
        self.declare_parameter('drift_angular', 0.03)  # 每弧度转动额外漂移比例
        self.declare_parameter('odom_topic', '/odom')
        self.declare_parameter('odom_frame', 'odom')
        self.declare_parameter('base_frame', 'base_footprint')

        self._drift_lin = float(self.get_parameter('drift_linear').value)
        self._drift_ang = float(self.get_parameter('drift_angular').value)
        self._odom_frame = self.get_parameter('odom_frame').value
        self._base_frame = self.get_parameter('base_frame').value
        odom_topic = self.get_parameter('odom_topic').value

        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._err_pub = self.create_publisher(Float64, '/odom_drift/error_m', 10)

        self._initialized = False
        self._last_time = None
        self._last_true_x = 0.0
        self._last_true_y = 0.0
        self._last_true_yaw = 0.0
        self._drift_x = 0.0
        self._drift_y = 0.0
        self._drift_yaw = 0.0

        self.create_subscription(Odometry, odom_topic, self._on_odom, 10)
        self.get_logger().info(
            f'里程计漂移注入已启动: drift_linear={self._drift_lin}, '
            f'drift_angular={self._drift_ang} (订阅 {odom_topic})')

    def _on_odom(self, msg: Odometry):
        t = self.get_clock().now()
        true_x = msg.pose.pose.position.x
        true_y = msg.pose.pose.position.y
        true_yaw = _yaw_from_quat(msg.pose.pose.orientation)

        if not self._initialized:
            self._drift_x, self._drift_y, self._drift_yaw = true_x, true_y, true_yaw
            self._last_true_x, self._last_true_y, self._last_true_yaw = true_x, true_y, true_yaw
            self._last_time = t
            self._initialized = True
            self._publish_tf(t)
            return

        # 真值增量 (世界坐标系下的位移 + 转角)
        dx = true_x - self._last_true_x
        dy = true_y - self._last_true_y
        dyaw = true_yaw - self._last_true_yaw
        if dyaw > math.pi:  dyaw -= 2 * math.pi
        if dyaw < -math.pi: dyaw += 2 * math.pi

        dist = math.hypot(dx, dy)

        # 按行驶距离/转角累积比例误差 (额外乘上 (1+drift) 倍)
        self._drift_x   += dx   * (1.0 + self._drift_lin)
        self._drift_y   += dy   * (1.0 + self._drift_lin)
        self._drift_yaw += dyaw * (1.0 + self._drift_ang)

        self._last_true_x, self._last_true_y, self._last_true_yaw = true_x, true_y, true_yaw
        self._last_time = t

        self._publish_tf(t)

        err = math.hypot(self._drift_x - true_x, self._drift_y - true_y)
        err_msg = Float64(); err_msg.data = err
        self._err_pub.publish(err_msg)

    def _publish_tf(self, stamp):
        tf = TransformStamped()
        tf.header.stamp = stamp.to_msg()
        tf.header.frame_id = self._odom_frame
        tf.child_frame_id  = self._base_frame
        tf.transform.translation.x = self._drift_x
        tf.transform.translation.y = self._drift_y
        tf.transform.translation.z = 0.0
        qz, qw = _quat_from_yaw(self._drift_yaw)
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)


def main(args=None):
    rclpy.init(args=args)
    node = OdomDriftInjector()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
