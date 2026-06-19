#!/usr/bin/env python3
"""
dock_align_node.py
合体对齐节点 — R2 主动视觉伺服对齐 R1 (对抗区合体流程)

算法流程:
  1. 收到 /dock_align/enable=1 后开始检测 R1 上的 ArUco 标志
  2. 利用 solvePnP 获取标志 6-DOF 位姿, 提取三个误差量:
       tx        横向偏移 (m)       → linear.y  (底盘横移)
       tz−target 距离误差 (m)       → linear.x  (底盘前后)
       yaw_error 偏航角误差 (rad)   → angular.z (底盘转向)
  3. 三路比例控制同时输出, 三路误差均进入死区连续 N 帧 → status=DONE
  4. game_controller 收到 DONE 后发送合体动作组指令,
     并向本节点发送 enable=0 关闭对齐

话题接口:
  /dock_align/enable  (std_msgs/UInt8, sub)   0=关闭  1=启动
  /dock_align/cmd     (geometry_msgs/Twist, pub) 底盘三轴速度指令
  /dock_align/status  (std_msgs/UInt8, pub)
      0=搜索中  1=伺服中  2=完成(DONE)  3=失败(超时)

可调参数 (见下方 declare_parameter, 可通过 launch 文件覆盖):
  aruco_dict_id      ArUco 字典 ID (默认 DICT_4X4_50)
  marker_ids         R1 上的标志 ID 列表 (默认 [0])
  marker_size_m      标志实际边长 m (★需现场用卷尺测量后填入)
  target_dist_m      期望停靠距离 m (默认 0.30)
  kp_lateral         横向比例增益
  kp_distance        前后比例增益
  kp_yaw             偏航比例增益
  max_linear         最大线速度 m/s
  max_angular        最大角速度 rad/s
  deadzone_lateral_m 横向死区 m
  deadzone_dist_m    距离死区 m
  deadzone_yaw_rad   偏航死区 rad
  debug_gui          True=显示调试画面 (调参用)

参考实现:
  https://github.com/GSNCodes/ArUCo-Markers-Pose-Estimation-Generation-Python
  https://github.com/AIRLab-POLIMI/ros2-aruco-pose-estimation
"""
import math
import time
from collections import deque

import cv2
import numpy as np

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge

from .config import (
    DOCK_ALIGN_DISABLE, DOCK_ALIGN_ENABLE,
    DOCK_ALIGN_STATUS_SEARCHING, DOCK_ALIGN_STATUS_ALIGNING,
    DOCK_ALIGN_STATUS_DONE, DOCK_ALIGN_STATUS_FAILED,
    DOCK_ALIGN_CONFIRM_FRAMES, DOCK_ALIGN_OK_RATIO,
)


class DockAlignNode(Node):

    def __init__(self):
        super().__init__('dock_align_node')

        # ── ROS2 可调参数 ─────────────────────────────────────
        self.declare_parameter('aruco_dict_id',      cv2.aruco.DICT_4X4_50)
        self.declare_parameter('marker_ids',         [0])
        self.declare_parameter('marker_size_m',      0.10)   # ★现场测量后修改
        self.declare_parameter('target_dist_m',      0.30)
        self.declare_parameter('kp_lateral',         0.8)
        self.declare_parameter('kp_distance',        0.5)
        self.declare_parameter('kp_yaw',             1.2)
        self.declare_parameter('max_linear',         0.15)
        self.declare_parameter('max_angular',        0.40)
        self.declare_parameter('deadzone_lateral_m', 0.015)  # 1.5 cm
        self.declare_parameter('deadzone_dist_m',    0.025)  # 2.5 cm
        self.declare_parameter('deadzone_yaw_rad',   0.052)  # ≈3°
        self.declare_parameter('debug_gui',          False)

        self._load_params()

        # ── 内部状态 ──────────────────────────────────────────
        self._enabled         = False
        self._camera_matrix   = None   # 由 /camera_info 填充
        self._dist_coeffs     = None
        self._t_start         = 0.0
        self._ok_window       = deque(maxlen=DOCK_ALIGN_CONFIRM_FRAMES)  # 滑动窗口
        self._no_target_count = 0
        self._status          = DOCK_ALIGN_STATUS_SEARCHING
        self._bridge          = CvBridge()
        self._img_sub         = None

        # ── ArUco 检测器 (兼容新旧 API) ───────────────────────
        aruco_dict   = cv2.aruco.getPredefinedDictionary(self._dict_id)
        aruco_params = cv2.aruco.DetectorParameters()
        try:
            self._detector    = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
            self._use_new_api = True
        except AttributeError:
            self._detector       = None
            self._aruco_dict_obj = aruco_dict
            self._aruco_params   = aruco_params
            self._use_new_api    = False

        # ── 话题 ──────────────────────────────────────────────
        self._cmd_pub    = self.create_publisher(Twist, '/dock_align/cmd',    10)
        self._status_pub = self.create_publisher(UInt8, '/dock_align/status', 10)

        self.create_subscription(UInt8,      '/dock_align/enable',                 self._on_enable,      10)
        self.create_subscription(CameraInfo, '/camera/camera/color/camera_info',   self._on_camera_info,  1)

        # 30 Hz 状态发布 + 超时检测
        self.create_timer(1.0 / 30.0, self._timer_cb)

        self.get_logger().info('合体对齐节点已启动, 等待 /dock_align/enable=1')

    # ─────────────────────────────────────────────────────────
    #   参数加载
    # ─────────────────────────────────────────────────────────

    def _load_params(self):
        self._dict_id     = self.get_parameter('aruco_dict_id').value
        self._marker_ids  = set(self.get_parameter('marker_ids').value)
        self._marker_size = self.get_parameter('marker_size_m').value
        self._target_dist = self.get_parameter('target_dist_m').value
        self._kp_lat      = self.get_parameter('kp_lateral').value
        self._kp_dist     = self.get_parameter('kp_distance').value
        self._kp_yaw      = self.get_parameter('kp_yaw').value
        self._max_lin     = self.get_parameter('max_linear').value
        self._max_ang     = self.get_parameter('max_angular').value
        self._dz_lat      = self.get_parameter('deadzone_lateral_m').value
        self._dz_dist     = self.get_parameter('deadzone_dist_m').value
        self._dz_yaw      = self.get_parameter('deadzone_yaw_rad').value
        self._debug_gui   = bool(self.get_parameter('debug_gui').value)
        self._obj_pts     = self._make_obj_pts(self._marker_size)

    @staticmethod
    def _make_obj_pts(size: float) -> np.ndarray:
        h = size / 2.0
        return np.array([
            [-h,  h, 0.0],   # 左上
            [ h,  h, 0.0],   # 右上
            [ h, -h, 0.0],   # 右下
            [-h, -h, 0.0],   # 左下
        ], dtype=np.float64)

    # ─────────────────────────────────────────────────────────
    #   相机内参 (仅需订阅一次)
    # ─────────────────────────────────────────────────────────

    def _on_camera_info(self, msg: CameraInfo):
        if self._camera_matrix is None:
            self._camera_matrix = np.array(msg.k, dtype=np.float64).reshape(3, 3)
            self._dist_coeffs   = np.array(msg.d, dtype=np.float64)
            self.get_logger().info('相机内参已加载')

    # ─────────────────────────────────────────────────────────
    #   启用 / 禁用
    # ─────────────────────────────────────────────────────────

    def _on_enable(self, msg: UInt8):
        if msg.data == DOCK_ALIGN_DISABLE:
            self._stop()
            return
        if not self._enabled:
            self._enabled         = True
            self._t_start         = time.time()
            self._ok_window.clear()
            self._no_target_count = 0
            self._status          = DOCK_ALIGN_STATUS_SEARCHING
            if self._img_sub is None:
                self._img_sub = self.create_subscription(
                    Image, '/camera/camera/color/image_raw', self._on_image, 10)
            self.get_logger().info('合体对齐启动, 搜索 ArUco 标志 ...')

    def _stop(self):
        self._enabled = False
        if self._img_sub is not None:
            self.destroy_subscription(self._img_sub)
            self._img_sub = None
        self._ok_window.clear()
        self._cmd_pub.publish(Twist())   # 停止底盘
        if self._debug_gui:
            cv2.destroyAllWindows()
        self.get_logger().info('合体对齐已关闭')

    # ─────────────────────────────────────────────────────────
    #   图像回调
    # ─────────────────────────────────────────────────────────

    def _on_image(self, msg: Image):
        if not self._enabled or self._camera_matrix is None:
            return
        if self._status == DOCK_ALIGN_STATUS_DONE:
            self._cmd_pub.publish(Twist())
            return

        frame = self._bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        gray  = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        self._process(gray, frame if self._debug_gui else None)

    # ─────────────────────────────────────────────────────────
    #   核心：ArUco 检测 → solvePnP → 三轴控制
    # ─────────────────────────────────────────────────────────

    def _process(self, gray: np.ndarray, dbg_frame=None):
        # 1. 检测所有 ArUco 标志
        if self._use_new_api:
            corners, ids, _ = self._detector.detectMarkers(gray)
        else:
            corners, ids, _ = cv2.aruco.detectMarkers(
                gray, self._aruco_dict_obj, parameters=self._aruco_params)

        # 2. 从目标 ID 中选取画面最大（最近）的标志
        best = self._select_best_marker(corners, ids)

        if best is None:
            self._no_target_count += 1
            self._cmd_pub.publish(Twist())   # 无标志时停车
            if dbg_frame is not None:
                cv2.putText(dbg_frame, 'SEARCHING...', (10, 40),
                            cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
                cv2.imshow('Dock Align', dbg_frame)
                cv2.waitKey(1)
            return

        self._no_target_count = 0
        img_pts = best.reshape(4, 1, 2).astype(np.float64)

        # 3. solvePnP → tvec(横向tx, 竖向ty, 前向tz) + rvec
        ok, rvec, tvec = cv2.solvePnP(
            self._obj_pts, img_pts,
            self._camera_matrix, self._dist_coeffs,
        )
        if not ok:
            self._cmd_pub.publish(Twist())
            return

        tx = float(tvec[0])   # 相机系横向偏移: 正=标志在相机右侧
        tz = float(tvec[2])   # 相机系前向距离 (m)

        # 4. 从旋转矩阵提取偏航误差
        #    marker_z_in_cam: 标志法向量在相机坐标系中的方向
        #    正对时 ≈ [0, 0, -1]；法向偏右时 [0] > 0 → R2 需左转
        R_mat, _ = cv2.Rodrigues(rvec)
        marker_z = R_mat[:, 2]
        yaw_err  = math.atan2(marker_z[0], -marker_z[2])

        dist_err = tz - self._target_dist   # >0 = 未到位需前进

        # 5. 死区判断
        lat_ok  = abs(tx)       < self._dz_lat
        dist_ok = abs(dist_err) < self._dz_dist
        yaw_ok  = abs(yaw_err)  < self._dz_yaw
        all_ok  = lat_ok and dist_ok and yaw_ok

        # 滑动窗口：记录本帧是否合格，允许抖动帧
        self._ok_window.append(1 if all_ok else 0)
        if not all_ok:
            self._status = DOCK_ALIGN_STATUS_ALIGNING

        # 6. 计算三轴速度指令
        cmd = Twist()
        if not all_ok:
            if not (lat_ok and dist_ok):
                # 横向: tx>0 标志偏右 → 向右移动 → linear.y<0 (ROS +y=左)
                cmd.linear.y = float(
                    np.clip(-self._kp_lat * tx, -self._max_lin, self._max_lin))
                # 前后: dist_err>0 未到位 → 前进
                cmd.linear.x = float(
                    np.clip(self._kp_dist * dist_err, -self._max_lin, self._max_lin))
            # 偏航始终修正: yaw_err>0 → 左转 → angular.z>0
            cmd.angular.z = float(
                np.clip(self._kp_yaw * yaw_err, -self._max_ang, self._max_ang))

        self._cmd_pub.publish(cmd)

        if dbg_frame is not None:
            self._draw_debug(dbg_frame, tx, tz, yaw_err, dist_err,
                             lat_ok, dist_ok, yaw_ok, cmd)

    # ─────────────────────────────────────────────────────────
    #   定时器：状态发布 + 超时 + DONE 确认
    # ─────────────────────────────────────────────────────────

    def _timer_cb(self):
        if not self._enabled:
            return

        elapsed = time.time() - self._t_start

        # 滑动窗口达到 N 帧且合格帧占比 ≥ OK_RATIO → 完成
        win_full = len(self._ok_window) == DOCK_ALIGN_CONFIRM_FRAMES
        ok_ratio = sum(self._ok_window) / DOCK_ALIGN_CONFIRM_FRAMES if win_full else 0.0
        if (win_full and ok_ratio >= DOCK_ALIGN_OK_RATIO
                and self._status != DOCK_ALIGN_STATUS_DONE):
            self._status = DOCK_ALIGN_STATUS_DONE
            self._cmd_pub.publish(Twist())
            self.get_logger().info(
                f'合体对齐完成 ✓ (窗口合格率={ok_ratio:.0%}), '
                '等待 game_controller 发送 enable=0')

        # 持续无目标 → 回到搜索状态
        if (self._no_target_count > 30
                and self._status == DOCK_ALIGN_STATUS_ALIGNING):
            self._status = DOCK_ALIGN_STATUS_SEARCHING

        msg = UInt8()
        msg.data = self._status
        self._status_pub.publish(msg)

    # ─────────────────────────────────────────────────────────
    #   辅助：选取最大（最近）的目标标志
    # ─────────────────────────────────────────────────────────

    def _select_best_marker(self, corners, ids):
        if ids is None or len(ids) == 0:
            return None
        best, best_area = None, -1.0
        for i, mid in enumerate(ids.flatten()):
            if int(mid) not in self._marker_ids:
                continue
            area = float(cv2.contourArea(corners[i][0]))
            if area > best_area:
                best_area, best = area, corners[i][0]
        return best

    # ─────────────────────────────────────────────────────────
    #   调试画面
    # ─────────────────────────────────────────────────────────

    def _draw_debug(self, frame, tx, tz, yaw_err, dist_err,
                    lat_ok, dist_ok, yaw_ok, cmd):
        def ok_color(flag):
            return (0, 220, 0) if flag else (0, 130, 255)

        lines = [
            (f'tx={tx:+.3f} m   {"OK" if lat_ok  else "ERR"}',  ok_color(lat_ok)),
            (f'tz={tz:.3f} m  err={dist_err:+.3f}  {"OK" if dist_ok else "ERR"}', ok_color(dist_ok)),
            (f'yaw={math.degrees(yaw_err):+.1f} deg  {"OK" if yaw_ok  else "ERR"}',ok_color(yaw_ok)),
            (f'vx={cmd.linear.x:+.3f}  vy={cmd.linear.y:+.3f}  wz={cmd.angular.z:+.3f}',
             (255, 255, 255)),
            (f'win={sum(self._ok_window)}/{DOCK_ALIGN_CONFIRM_FRAMES}'
             f'({100*sum(self._ok_window)//max(len(self._ok_window),1)}%)  '
             f'{"★DONE!" if self._status == DOCK_ALIGN_STATUS_DONE else "ALIGNING"}',
             (0, 220, 0) if self._status == DOCK_ALIGN_STATUS_DONE else (0, 165, 255)),
        ]
        for i, (text, color) in enumerate(lines):
            cv2.putText(frame, text, (10, 35 + i * 32),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.75, color, 2)
        cv2.imshow('Dock Align', frame)
        cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = DockAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._stop()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
