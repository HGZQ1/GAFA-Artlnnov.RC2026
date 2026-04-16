"""
odometry_fusion.py
IMU + 编码器 融合里程计

融合策略（互补型 EKF 简化版）：
  - 编码器提供：Δx（前进），Δθ（转向），频率50Hz
  - IMU提供：pitch（坡度），yaw_rate（辅助转向），频率10Hz
  - 融合输出：机器人相对启动点的位姿 (x, y, θ)
              以及当前坡度角

坐标系：
  以比赛开始时 R2 启动位置为原点
  x = 前方, y = 左方, θ = 朝向角（前方为0，左转为正）

发布话题：
  /odom/fused   Float32MultiArray
    data[0] = x          (米)
    data[1] = y          (米)
    data[2] = theta      (弧度)
    data[3] = pitch_deg  (当前坡度角，度)
    data[4] = dist_total (总行驶距离，米)
"""
import math
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, Int8
from geometry_msgs.msg import Twist

from .config import (
    ENCODER_RATE_HZ,
    ODOM_DRIFT_FACTOR,
    IMU_COMP_ALPHA,
)


class OdometryFusion(Node):

    def __init__(self):
        super().__init__('odometry_fusion')

        # 机器人位姿状态
        self._x     = 0.0   # 前方位移（米）
        self._y     = 0.0   # 左方位移（米）
        self._theta = 0.0   # 朝向角（弧度，左转为正）
        self._dist  = 0.0   # 总行驶距离（米）

        # IMU 状态缓存
        self._pitch_deg  = 0.0
        self._yaw_rate   = 0.0   # 度/秒

        # 融合权重（编码器朝向 vs IMU朝向）
        # 编码器在平地可靠，IMU在爬坡转弯时更准
        self._theta_alpha = 0.7   # 0.7权重给编码器，0.3给IMU

        # 订阅编码器反馈（来自STM32）
        self.create_subscription(
            Twist, '/feedback/encoder',
            self._on_encoder, 10)

        # 订阅处理后的IMU数据
        self.create_subscription(
            Float32MultiArray, '/imu/processed',
            self._on_imu, 10)

        # 发布融合里程计
        self.odom_pub = self.create_publisher(
            Float32MultiArray, '/odom/fused', 10)

        # 重置服务（比赛开始时调用）
        self.create_subscription(
            Int8, '/odom/reset',
            lambda msg: self._reset(), 10)

        self.get_logger().info('里程计融合节点已启动')

    def _on_imu(self, msg: Float32MultiArray):
        """更新IMU状态缓存"""
        if len(msg.data) >= 5:
            self._pitch_deg = msg.data[0]
            self._yaw_rate  = msg.data[2]   # 度/秒

    def _on_encoder(self, msg: Twist):
        """
        收到STM32编码器数据，更新融合里程计

        STM32 发布格式（Twist）：
          linear.x  = delta_x（本周期前进距离，米）
          linear.y  = delta_y（侧向，差速底盘通常为0）
          angular.z = delta_theta（本周期转向角，弧度）
        """
        delta_x     = msg.linear.x
        delta_y     = msg.linear.y
        delta_theta_enc = msg.angular.z    # 编码器计算的转向角增量

        # 坡度补偿：编码器测量的是沿轮子运动方向的距离
        # 在坡面上，实际水平位移 = 编码器距离 × cos(坡度角)
        pitch_rad = math.radians(self._pitch_deg)
        horiz_factor = math.cos(pitch_rad)
        delta_x_horiz = delta_x * horiz_factor

        # IMU辅助转向（如果IMU的yaw_rate更可靠）
        # dt 约为 1/ENCODER_RATE_HZ 秒
        dt = 1.0 / ENCODER_RATE_HZ
        delta_theta_imu = math.radians(self._yaw_rate * dt)

        # 融合转向角
        delta_theta = (self._theta_alpha * delta_theta_enc
                       + (1 - self._theta_alpha) * delta_theta_imu)

        # 积分更新位姿（中点法，比前向欧拉精度更高）
        mid_theta = self._theta + delta_theta / 2.0
        self._x     += delta_x_horiz * math.cos(mid_theta)
        self._y     += delta_x_horiz * math.sin(mid_theta)
        self._theta += delta_theta
        self._dist  += abs(delta_x)

        # 角度归一化到 [-π, π]
        self._theta = math.atan2(
            math.sin(self._theta),
            math.cos(self._theta))

        # 发布融合结果
        odom_msg = Float32MultiArray()
        odom_msg.data = [
            float(self._x),
            float(self._y),
            float(self._theta),
            float(self._pitch_deg),
            float(self._dist),
        ]
        self.odom_pub.publish(odom_msg)

    def _reset(self):
        """重置里程计到原点（比赛开始时调用）"""
        self._x     = 0.0
        self._y     = 0.0
        self._theta = 0.0
        self._dist  = 0.0
        self.get_logger().info('里程计已重置到原点')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(OdometryFusion())
    rclpy.shutdown()