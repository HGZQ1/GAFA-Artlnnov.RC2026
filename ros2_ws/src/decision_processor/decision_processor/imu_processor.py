"""
imu_processor.py
RealSense D435i IMU 数据处理节点

功能：
  1. 订阅 IMU 加速度计和陀螺仪原始数据
  2. 互补滤波（Complementary Filter）融合 → pitch/roll/yaw_rate
  3. 发布处理后的 IMU 状态：坡度角、横滚角、转向角速度
  4. 发布坡度等级（平地/轻坡/中坡/陡坡）

互补滤波原理：
  pitch_filtered = α × (pitch_prev + gyro_y × dt)
                 + (1-α) × atan2(accel_x, accel_z)

  α 接近1时更信陀螺仪（短期准确），
  α 接近0时更信加速度计（长期稳定）
  典型值 α=0.96，dt=0.01s(100Hz)
"""



import math
import time
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu
from std_msgs.msg    import Float32MultiArray, Int8

from .config import (
    IMU_TOPIC, IMU_ACCEL_TOPIC, IMU_GYRO_TOPIC,
    IMU_COMP_ALPHA, GRAVITY,
    IMU_ACCEL_SIGN_X, IMU_ACCEL_SIGN_Y, IMU_ACCEL_SIGN_Z,
    SLOPE_DETECT_DEG, SLOPE_LEVEL_MILD,
    SLOPE_LEVEL_MODERATE, SLOPE_LEVEL_STEEP,
)

from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

# 定义与 RealSense 匹配的 QoS
IMU_QOS = QoSProfile(
    reliability=ReliabilityPolicy.BEST_EFFORT,
    history=HistoryPolicy.KEEP_LAST,
    depth=10
)

class SlopeLevel:
    """坡度等级常量"""
    FLAT     = 0   # 平地
    MILD     = 1   # 轻坡（< SLOPE_LEVEL_MILD°）
    MODERATE = 2   # 中坡（< SLOPE_LEVEL_MODERATE°）
    STEEP    = 3   # 陡坡（>= SLOPE_LEVEL_STEEP°）

    @staticmethod
    def from_angle(pitch_deg: float) -> int:
        abs_deg = abs(pitch_deg)
        if abs_deg < SLOPE_DETECT_DEG:
            return SlopeLevel.FLAT
        elif abs_deg < SLOPE_LEVEL_MILD:
            return SlopeLevel.MILD
        elif abs_deg < SLOPE_LEVEL_MODERATE:
            return SlopeLevel.MODERATE
        else:
            return SlopeLevel.STEEP

    @staticmethod
    def name(level: int) -> str:
        return {0:'FLAT', 1:'MILD', 2:'MODERATE', 3:'STEEP'}.get(level, '?')


class IMUProcessor(Node):
    """
    IMU 处理节点

    发布话题：
      /imu/processed  Float32MultiArray
        data[0] = pitch_deg  俯仰角（度，向上为正，向下为负）
        data[1] = roll_deg   横滚角（度，左倾为正）
        data[2] = yaw_rate   偏航角速度（度/秒，左转为正）
        data[3] = is_climbing  1.0=正在上坡, -1.0=正在下坡, 0.0=平地
        data[4] = slope_level  0~3的坡度等级

      /imu/slope_level  Int8
        0=平地, 1=轻坡, 2=中坡, 3=陡坡
    """

    def __init__(self):
        super().__init__('imu_processor')

        # 互补滤波状态
        self._pitch_filtered = 0.0   # 俯仰角（度）
        self._roll_filtered  = 0.0   # 横滚角（度）
        self._last_time      = None

        # 陀螺仪缓存
        self._gyro_x = 0.0   # 绕x轴角速度（rad/s）
        self._gyro_y = 0.0   # 绕y轴角速度（rad/s，对应pitch）
        self._gyro_z = 0.0   # 绕z轴角速度（rad/s，对应yaw）

        # 坡度平滑窗口（防止一帧噪声触发爬坡模式切换）
        self._slope_buf    = [0.0] * 10
        self._slope_idx    = 0
        self._slope_level  = SlopeLevel.FLAT

        # 订阅陀螺仪（高频，200Hz）
        self.create_subscription(
            Imu, IMU_GYRO_TOPIC, self._on_gyro, IMU_QOS)

        # 订阅加速度计（100Hz），在这里做融合
        self.create_subscription(
            Imu, IMU_ACCEL_TOPIC, self._on_accel, IMU_QOS)

        # 也尝试订阅融合后的 /imu（如果 RealSense 驱动已经融合）
        self.create_subscription(
            Imu, IMU_TOPIC, self._on_imu_combined, IMU_QOS)

        # 发布处理后的 IMU 状态
        self.processed_pub = self.create_publisher(
            Float32MultiArray, '/imu/processed', 10)
        self.slope_pub = self.create_publisher(
            Int8, '/imu/slope_level', 10)

        # 定时发布（10Hz，避免下游处理过快）
        self.create_timer(0.1, self._publish)

        self.get_logger().info(
            'IMU处理节点已启动\n'
            f'  互补滤波系数 α={IMU_COMP_ALPHA}\n'
            f'  坡度检测阈值={SLOPE_DETECT_DEG}°')

    def _on_gyro(self, msg: Imu):
        """接收陀螺仪数据，缓存角速度"""
        # D435i 陀螺仪坐标系：x右,y下,z后
        # 映射到机器人坐标系：pitch对应陀螺仪y轴
        self._gyro_x = msg.angular_velocity.x
        self._gyro_y = msg.angular_velocity.y   # pitch角速度
        self._gyro_z = msg.angular_velocity.z   # yaw角速度

    def _on_accel(self, msg: Imu):
        """
        接收加速度计数据，执行互补滤波
        这里做主要的融合计算
        """
        now = time.time()
        if self._last_time is None:
            self._last_time = now
            return
        dt = now - self._last_time
        self._last_time = now

        # 防止dt异常（首帧或长时间无数据）
        if dt > 0.5 or dt <= 0:
            dt = 0.01

        # 读取加速度（m/s²），应用安装方向修正
        ax = msg.linear_acceleration.x * IMU_ACCEL_SIGN_X
        ay = msg.linear_acceleration.y * IMU_ACCEL_SIGN_Y
        az = msg.linear_acceleration.z * IMU_ACCEL_SIGN_Z

        # ── 加速度计计算静态角度（机器人静止或匀速时准确）──
        # pitch：绕y轴倾斜，前倾为正
        # 公式：atan2(ax, sqrt(ay² + az²))
        accel_pitch = math.degrees(
            math.atan2(az, math.sqrt(ay**2 + az**2)))

        # roll：绕x轴倾斜，左倾为正
        accel_roll = math.degrees(
            math.atan2(ax, -ay))

        # ── 陀螺仪积分（短期准确，长期漂移）──
        # 陀螺仪y轴角速度对应pitch的变化率
        gyro_pitch_rate = math.degrees(self._gyro_y)  # rad/s → deg/s
        gyro_roll_rate  = math.degrees(self._gyro_x)

        # ── 互补滤波 ──
        alpha = IMU_COMP_ALPHA
        self._pitch_filtered = (
            alpha * (self._pitch_filtered + gyro_pitch_rate * dt)
            + (1 - alpha) * accel_pitch
        )
        self._roll_filtered = (
            alpha * (self._roll_filtered + gyro_roll_rate * dt)
            + (1 - alpha) * accel_roll
        )

        # ── 坡度平滑（滑动窗口均值）──
        self._slope_buf[self._slope_idx] = self._pitch_filtered
        self._slope_idx = (self._slope_idx + 1) % len(self._slope_buf)
        smooth_pitch = sum(self._slope_buf) / len(self._slope_buf)

        # ── 坡度等级判断 ──
        self._slope_level = SlopeLevel.from_angle(smooth_pitch)

    def _on_imu_combined(self, msg: Imu):
        """
        如果 RealSense 驱动发布了融合后的 /camera/camera/imu
        直接使用四元数计算角度（更准确）
        """
        qx = msg.orientation.x
        qy = msg.orientation.y
        qz = msg.orientation.z
        qw = msg.orientation.w

        # 四元数全为零说明未启用融合，跳过
        if abs(qw) < 1e-6 and abs(qx) < 1e-6:
            return

        # 四元数 → 欧拉角
        # pitch（绕y轴）
        sinp = 2.0 * (qw * qy - qz * qx)
        sinp = max(-1.0, min(1.0, sinp))
        pitch_rad = math.asin(sinp)

        # roll（绕x轴）
        sinr_cosp = 2.0 * (qw * qx + qy * qz)
        cosr_cosp = 1.0 - 2.0 * (qx * qx + qy * qy)
        roll_rad  = math.atan2(sinr_cosp, cosr_cosp)

        # 直接更新（四元数比互补滤波更准确）
        self._pitch_filtered = math.degrees(pitch_rad)
        self._roll_filtered  = math.degrees(roll_rad)
        self._slope_level    = SlopeLevel.from_angle(self._pitch_filtered)

    def _publish(self):
        """定时发布处理结果"""
        yaw_rate_deg = math.degrees(self._gyro_z)

        # 判断上坡/下坡/平地
        if self._pitch_filtered > SLOPE_DETECT_DEG:
            is_climbing = 1.0   # 上坡
        elif self._pitch_filtered < -SLOPE_DETECT_DEG:
            is_climbing = -1.0  # 下坡
        else:
            is_climbing = 0.0   # 平地

        # 发布处理后的IMU状态
        imu_msg = Float32MultiArray()
        imu_msg.data = [
            float(self._pitch_filtered),   # [0] pitch_deg 俯仰角
            float(self._roll_filtered),    # [1] roll_deg  翻转角
            float(yaw_rate_deg),           # [2] yaw_rate deg/s 航向角（转头）maybe
            float(is_climbing),            # [3] 上坡1/下坡-1/平地0
            float(self._slope_level),      # [4] 0~3坡度等级
        ]
        self.processed_pub.publish(imu_msg)

        # 发布坡度等级（供 processor_node 直接判断）
        level_msg = Int8()
        level_msg.data = self._slope_level
        self.slope_pub.publish(level_msg)


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(IMUProcessor())
    rclpy.shutdown()