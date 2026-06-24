"""
tf_manager.py
TF 坐标变换管理器

职责：
  1. 广播静态变换：camera_link → base_link
  2. 广播静态变换：arm_base_link → base_link
  3. 提供坐标变换查询接口：将相机坐标转换为底盘坐标

使用方式：
  在 processor_node.py 的 __init__ 里实例化 TFManager
  在 _on_raw_target 里调用 transform_camera_to_base()
"""
import math
import rclpy
from rclpy.node import Node
import numpy as np

from geometry_msgs.msg import TransformStamped, PointStamped
import tf2_ros
import tf2_geometry_msgs   # 必须导入才能注册 PointStamped 的变换支持

from .config import (
    FRAME_BASE_LINK, FRAME_CAMERA, FRAME_ARM_BASE,
    CAMERA_OFFSET_X, CAMERA_OFFSET_Y, CAMERA_OFFSET_Z,
    CAMERA_ROLL_RAD, CAMERA_PITCH_RAD, CAMERA_YAW_RAD,
    ARM_OFFSET_X, ARM_OFFSET_Y, ARM_OFFSET_Z,
    ARM_ROLL_RAD, ARM_PITCH_RAD, ARM_YAW_RAD,
)


def euler_to_quaternion(roll: float, pitch: float,
                        yaw: float) -> tuple:
    """
    欧拉角（弧度）转四元数
    返回 (x, y, z, w)
    """
    cr = math.cos(roll  / 2)
    sr = math.sin(roll  / 2)
    cp = math.cos(pitch / 2)
    sp = math.sin(pitch / 2)
    cy = math.cos(yaw   / 2)
    sy = math.sin(yaw   / 2)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy
    return x, y, z, w


class TFManager:
    """
    TF 坐标变换管理器
    实例化时自动广播所有静态变换
    """

    def __init__(self, node: Node):
        self._node = node

        # 静态变换广播器（只广播一次，持续有效）
        self._static_broadcaster = tf2_ros.StaticTransformBroadcaster(node)

        # TF 缓冲区和监听器（用于查询变换）
        self._tf_buffer   = tf2_ros.Buffer()
        self._tf_listener = tf2_ros.TransformListener(
            self._tf_buffer, node)

        # 广播所有静态变换
        self._broadcast_static_transforms()

        node.get_logger().info(
            'TFManager 初始化完成\n'
            f'  camera_link → base_link: '
            f'offset=({CAMERA_OFFSET_X:.3f}, '
            f'{CAMERA_OFFSET_Y:.3f}, {CAMERA_OFFSET_Z:.3f})\n'
            f'  arm_base_link → base_link: '
            f'offset=({ARM_OFFSET_X:.3f}, '
            f'{ARM_OFFSET_Y:.3f}, {ARM_OFFSET_Z:.3f})')

    def _broadcast_static_transforms(self):
        """广播所有静态变换"""
        transforms = []

        # ── 变换1：camera_link → base_link ──
        t_cam = TransformStamped()
        t_cam.header.stamp            = self._node.get_clock().now().to_msg()
        t_cam.header.frame_id         = FRAME_BASE_LINK
        t_cam.child_frame_id          = FRAME_CAMERA
        t_cam.transform.translation.x = CAMERA_OFFSET_X
        t_cam.transform.translation.y = CAMERA_OFFSET_Y
        t_cam.transform.translation.z = CAMERA_OFFSET_Z
        qx, qy, qz, qw = euler_to_quaternion(
            CAMERA_ROLL_RAD, CAMERA_PITCH_RAD, CAMERA_YAW_RAD)
        t_cam.transform.rotation.x = qx
        t_cam.transform.rotation.y = qy
        t_cam.transform.rotation.z = qz
        t_cam.transform.rotation.w = qw
        transforms.append(t_cam)

        # ── 变换2：arm_base_link → base_link ──
        t_arm = TransformStamped()
        t_arm.header.stamp            = self._node.get_clock().now().to_msg()
        t_arm.header.frame_id         = FRAME_BASE_LINK
        t_arm.child_frame_id          = FRAME_ARM_BASE
        t_arm.transform.translation.x = ARM_OFFSET_X
        t_arm.transform.translation.y = ARM_OFFSET_Y
        t_arm.transform.translation.z = ARM_OFFSET_Z
        qx, qy, qz, qw = euler_to_quaternion(
            ARM_ROLL_RAD, ARM_PITCH_RAD, ARM_YAW_RAD)
        t_arm.transform.rotation.x = qx
        t_arm.transform.rotation.y = qy
        t_arm.transform.rotation.z = qz
        t_arm.transform.rotation.w = qw
        transforms.append(t_arm)

        self._static_broadcaster.sendTransform(transforms)

    def transform_camera_to_base(self,
                                  cam_x: float,
                                  cam_y: float,
                                  cam_z: float,
                                  stamp=None) -> tuple | None:
        """
        将相机坐标系中的点变换到底盘坐标系（base_link）

        Args:
            cam_x, cam_y, cam_z: 相机坐标系下的3D坐标（米）
            stamp: 时间戳，None时使用最新变换
        Returns:
            (base_x, base_y, base_z) 底盘坐标系坐标，失败返回 None
        """
        try:
            # 构造相机坐标系中的点
            point_in_camera = PointStamped()
            if stamp is not None:
                point_in_camera.header.stamp = stamp
            else:
                point_in_camera.header.stamp = \
                    self._node.get_clock().now().to_msg()
            point_in_camera.header.frame_id = FRAME_CAMERA
            point_in_camera.point.x = cam_x
            point_in_camera.point.y = cam_y
            point_in_camera.point.z = cam_z

            # 查询变换（超时0.1秒）
            point_in_base = self._tf_buffer.transform(
                point_in_camera,
                FRAME_BASE_LINK,
                timeout=rclpy.duration.Duration(seconds=0.1))

            return (point_in_base.point.x,
                    point_in_base.point.y,
                    point_in_base.point.z)

        except Exception as e:
            self._node.get_logger().debug(
                f'TF变换失败（使用回退方案）：{e}')
            return self._fallback_transform(cam_x, cam_y, cam_z)

    def transform_base_to_arm(self,
                               base_x: float,
                               base_y: float,
                               base_z: float) -> tuple | None:
        """
        将底盘坐标系中的点变换到机械臂底座坐标系（arm_base_link）
        用于计算机械臂抓取时的目标位置

        Returns:
            (arm_x, arm_y, arm_z) 机械臂底座坐标系坐标，失败返回 None
        """
        try:
            point_in_base = PointStamped()
            point_in_base.header.stamp    = \
                self._node.get_clock().now().to_msg()
            point_in_base.header.frame_id = FRAME_BASE_LINK
            point_in_base.point.x = base_x
            point_in_base.point.y = base_y
            point_in_base.point.z = base_z

            point_in_arm = self._tf_buffer.transform(
                point_in_base,
                FRAME_ARM_BASE,
                timeout=rclpy.duration.Duration(seconds=0.1))

            return (point_in_arm.point.x,
                    point_in_arm.point.y,
                    point_in_arm.point.z)

        except Exception as e:
            self._node.get_logger().debug(f'TF变换(base→arm)失败：{e}')
            return None

    def _fallback_transform(self, cam_x: float,
                             cam_y: float,
                             cam_z: float) -> tuple:
        """
        TF查询失败时的回退方案
        使用 config.py 中的参数直接计算（不依赖TF树）
        """
        # 旋转变换（简化：只处理常见的水平安装情况）
        pitch = CAMERA_PITCH_RAD
        yaw   = CAMERA_YAW_RAD

        # 相机坐标系（z前，x右，y下）→ 底盘坐标系（x前，y左，z上）
        # 先做相机内部轴变换
        robot_x_cam =  cam_z   # 相机z前方 → 底盘x前方
        robot_y_cam = -cam_x   # 相机x右方 → 底盘y左方（取反）
        robot_z_cam = -cam_y   # 相机y下方 → 底盘z上方（取反）

        # 俯仰角补偿（相机向下倾斜时）
        import math
        horiz = robot_x_cam * math.cos(pitch) + robot_z_cam * math.sin(pitch)
        vert  = -robot_x_cam * math.sin(pitch) + robot_z_cam * math.cos(pitch)

        # 偏航角补偿
        base_x = horiz * math.cos(yaw) - robot_y_cam * math.sin(yaw)
        base_y = horiz * math.sin(yaw) + robot_y_cam * math.cos(yaw)
        base_z = vert

        # 加上安装位置偏移
        base_x += CAMERA_OFFSET_X
        base_y += CAMERA_OFFSET_Y
        base_z += CAMERA_OFFSET_Z

        return base_x, base_y, base_z