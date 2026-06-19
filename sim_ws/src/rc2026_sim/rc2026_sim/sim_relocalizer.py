"""
sim_relocalizer.py
仿真用启动点重定位模拟节点

用途:
  比赛时机器人不一定能精确放置在假定的出生点坐标上 (存在放置误差)。
  真机使用 map_relocalizer.py (FAST-LIO + ICP) 在开局后用激光点云
  与预建地图匹配, 修正 map→odom TF。
  仿真没有 FAST-LIO/Livox, 本节点用 Gazebo 的真值位姿模拟"重定位"过程:

  1. 启动后立即按"假定出生点" (assumed_x/y/yaw) 发布 map→odom TF
     (模拟"开局前我们以为自己在这里")
  2. reloc_delay 秒后, 调用 /gazebo/get_entity_state 获取机器人在 Gazebo
     世界坐标系下的真实位姿, 重新计算并发布修正后的 map→odom TF
     (模拟"ICP 重定位完成, 修正坐标系偏差")
"""
import math

import rclpy
from rclpy.node import Node
from gazebo_msgs.srv import GetEntityState
import tf2_ros
from geometry_msgs.msg import TransformStamped, PoseWithCovarianceStamped


def _yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _quat_from_yaw(yaw: float):
    return math.sin(yaw / 2.0), math.cos(yaw / 2.0)  # (qz, qw)


def _compute_map_to_odom_tf(sx, sy, syaw):
    c = math.cos(syaw)
    s = math.sin(syaw)
    return -c * sx - s * sy, s * sx - c * sy, -syaw


class SimRelocalizer(Node):

    def __init__(self):
        super().__init__('sim_relocalizer')

        self.declare_parameter('assumed_x', 0.0)
        self.declare_parameter('assumed_y', 0.0)
        self.declare_parameter('assumed_yaw', 0.0)
        self.declare_parameter('entity_name', 'rc2026_robot')
        self.declare_parameter('reloc_delay', 5.0)

        ax = float(self.get_parameter('assumed_x').value)
        ay = float(self.get_parameter('assumed_y').value)
        ayaw = float(self.get_parameter('assumed_yaw').value)
        self._entity_name = self.get_parameter('entity_name').value
        self._reloc_delay = float(self.get_parameter('reloc_delay').value)

        self._tx, self._ty, self._tyaw = _compute_map_to_odom_tf(ax, ay, ayaw)
        self._tf_broadcaster = tf2_ros.TransformBroadcaster(self)
        self._initialpose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)

        self._cli = self.create_client(GetEntityState, '/gazebo/get_entity_state')

        # 20Hz 持续广播 map→odom TF (开局先用假定出生点)
        self.create_timer(0.05, self._publish_tf)
        # reloc_delay 秒后做一次性"重定位"修正
        self._reloc_timer = self.create_timer(self._reloc_delay, self._do_relocalize)

        self.get_logger().info(
            f'sim_relocalizer 已启动: 假定出生点=({ax:.2f},{ay:.2f},{ayaw:.3f}), '
            f'{self._reloc_delay}s 后将用 Gazebo 真值修正 map→odom')

    def _publish_tf(self):
        tf = TransformStamped()
        tf.header.stamp = self.get_clock().now().to_msg()
        tf.header.frame_id = 'map'
        tf.child_frame_id  = 'odom'
        tf.transform.translation.x = self._tx
        tf.transform.translation.y = self._ty
        tf.transform.translation.z = 0.0
        qz, qw = _quat_from_yaw(self._tyaw)
        tf.transform.rotation.z = qz
        tf.transform.rotation.w = qw
        self._tf_broadcaster.sendTransform(tf)

    def _do_relocalize(self):
        self._reloc_timer.cancel()
        if not self._cli.wait_for_service(timeout_sec=2.0):
            self.get_logger().warn('/gazebo/get_entity_state 服务不可用, 跳过重定位修正')
            return
        req = GetEntityState.Request()
        req.name = self._entity_name
        req.reference_frame = 'world'
        future = self._cli.call_async(req)
        future.add_done_callback(self._on_entity_state)

    def _on_entity_state(self, future):
        try:
            res = future.result()
        except Exception as e:
            self.get_logger().warn(f'获取 Gazebo 真值位姿失败: {e}')
            return
        if not res.success:
            self.get_logger().warn(f'实体 {self._entity_name} 未找到, 跳过重定位修正')
            return

        p = res.state.pose.position
        true_yaw = _yaw_from_quat(res.state.pose.orientation)
        new_tx, new_ty, new_tyaw = _compute_map_to_odom_tf(p.x, p.y, true_yaw)

        old = (self._tx, self._ty, self._tyaw)
        self._tx, self._ty, self._tyaw = new_tx, new_ty, new_tyaw
        self.get_logger().info(
            f'重定位完成: Gazebo真值=({p.x:.3f},{p.y:.3f},{true_yaw:.3f})  '
            f'map→odom 修正: {old[0]:.3f},{old[1]:.3f},{old[2]:.3f} -> '
            f'{new_tx:.3f},{new_ty:.3f},{new_tyaw:.3f}')

        self._publish_initialpose()

    def _publish_initialpose(self):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp = self.get_clock().now().to_msg()
        # odom 原点在 map 系下的位姿即为修正量本身 (机器人当前回到 odom 原点)
        msg.pose.pose.position.x = self._tx
        msg.pose.pose.position.y = self._ty
        qz, qw = _quat_from_yaw(self._tyaw)
        msg.pose.pose.orientation.z = qz
        msg.pose.pose.orientation.w = qw
        self._initialpose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    node = SimRelocalizer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
