#!/usr/bin/env python3
"""
map_relocalizer.py
对预建点云地图做 ICP 重定位, 自动校正 waypoint_navigator 的 loc_offset 参数

流程:
  1. 加载 PCD 地图 (right_half.pcd 或 left_half.pcd)
  2. 启动后自动执行一次重定位 (等待 FAST-LIO 输出 /cloud_registered)
  3. 之后常驻运行, 可通过 /relocalize/trigger (std_msgs/String) 触发新一轮重定位
     —— 用于真机跑全程时, 在每次进入新区域(武馆/梅林/对抗区)的入口点重新校正
        loc_offset, 消除该区域内积累的 FAST-LIO SLAM 漂移
  4. 每轮: 累积若干帧 /cloud_registered, 做 ICP 匹配, 用结果修正
     waypoint_navigator 的 loc_offset_x/y/yaw, 并通过 /relocalize/status 报告结果

注意:
  - prior_offset_x/y/yaw (= FAST-LIO 启动点的游戏坐标) 在整个运行期间保持不变,
    每轮 ICP 都基于这组常量重新计算 "当前 FAST-LIO 坐标系 -> 游戏坐标" 的偏移,
    因此可以独立吸收掉 SLAM 漂移, 不需要也不应该用上一轮的修正结果作为输入

使用前提:
  - FAST-LIO 已启动并持续输出 /cloud_registered
  - Open3D 已安装 (pip install open3d)
"""

import math
import time
import numpy as np
import open3d as o3d

import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import PointCloud2
import sensor_msgs_py.point_cloud2 as pc2
from geometry_msgs.msg import PoseWithCovarianceStamped
from rcl_interfaces.msg import Parameter, ParameterType
from rcl_interfaces.srv import SetParameters


class MapRelocalizer(Node):

    def __init__(self):
        super().__init__('map_relocalizer')

        # ── 参数声明 ──────────────────────────────────────────
        self.declare_parameter('map_file', '')
        self.declare_parameter('prior_offset_x',   1.4)     # 游戏坐标: FAST-LIO 原点 x
        self.declare_parameter('prior_offset_y',   0.4)     # 游戏坐标: FAST-LIO 原点 y
        self.declare_parameter('prior_offset_yaw', 1.5708)  # 游戏坐标: FAST-LIO 初始 yaw
        self.declare_parameter('scan_accum_pts',   3000)    # 累积到此点数后触发 ICP
        self.declare_parameter('icp_max_dist',     0.5)     # ICP 最大对应点距离 (m)
        self.declare_parameter('icp_min_fitness',  0.05)    # 低于此值视为匹配失败
        self.declare_parameter('voxel_size',       0.1)     # 降采样体素大小 (m)

        self._map_file  = self.get_parameter('map_file').value
        self._off_x     = self.get_parameter('prior_offset_x').value
        self._off_y     = self.get_parameter('prior_offset_y').value
        self._off_yaw   = self.get_parameter('prior_offset_yaw').value
        self._tgt_pts   = self.get_parameter('scan_accum_pts').value
        self._icp_dist  = self.get_parameter('icp_max_dist').value
        self._icp_fit   = self.get_parameter('icp_min_fitness').value
        self._voxel     = self.get_parameter('voxel_size').value

        if not self._map_file:
            self.get_logger().error('map_file 参数为空, 退出')
            raise RuntimeError('map_file not set')

        # ── 加载地图 ──────────────────────────────────────────
        self.get_logger().info(f'加载地图: {self._map_file}')
        raw = o3d.io.read_point_cloud(self._map_file)
        if len(raw.points) == 0:
            self.get_logger().error('地图加载失败或点云为空')
            raise RuntimeError('empty map')
        self._map = raw.voxel_down_sample(self._voxel)
        self._map.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))
        self.get_logger().info(
            f'地图就绪: {len(raw.points)} → {len(self._map.points)} 点(下采样后)')

        self._pts_buf    = []
        self._running    = False
        self._pass_label = 'startup'
        self._t_start    = time.time()
        self._sub        = None

        # ── 可视化发布 / 状态上报 ───────────────────────────────
        self._pose_pub = self.create_publisher(
            PoseWithCovarianceStamped, '/initialpose', 10)
        self._status_pub = self.create_publisher(String, '/relocalize/status', 10)

        # ── 参数服务客户端 (复用) ────────────────────────────────
        self._set_param_client = self.create_client(
            SetParameters, '/waypoint_navigator/set_parameters')

        # ── 重定位触发话题 ──────────────────────────────────────
        # game_controller 在每次进入新区域入口点时发布该话题 (data=区域名)
        self.create_subscription(String, '/relocalize/trigger', self._on_trigger, 10)

        self.get_logger().info(
            '地图重定位节点已就绪 (启动时自动执行一次, '
            '之后可通过 /relocalize/trigger 触发新一轮重定位)')

        # ── 启动时自动执行一次 ──────────────────────────────────
        self._start_pass('startup')

    # ─────────────────────────────────────────────────────────
    #   开始一轮重定位
    # ─────────────────────────────────────────────────────────

    def _start_pass(self, label: str):
        if self._running:
            self.get_logger().warn(f'重定位进行中, 忽略触发: {label}')
            return
        self._running    = True
        self._pts_buf    = []
        self._pass_label = label
        self._t_start    = time.time()
        self._sub = self.create_subscription(
            PointCloud2, '/cloud_registered', self._on_cloud, 10)
        self.get_logger().info(
            f'[{label}] 开始重定位, 等待 /cloud_registered ... '
            f'(需累积 {self._tgt_pts} 点)')

    def _on_trigger(self, msg: String):
        label = msg.data.strip() if msg.data.strip() else 'manual'
        self._start_pass(label)

    # ─────────────────────────────────────────────────────────
    #   Cloud callback
    # ─────────────────────────────────────────────────────────

    def _on_cloud(self, msg: PointCloud2):
        if not self._running:
            return
        pts = list(pc2.read_points(msg, field_names=('x', 'y', 'z'), skip_nans=True))
        if pts:
            self._pts_buf.extend(pts)
        if len(self._pts_buf) >= self._tgt_pts:
            self._run_icp()

    # ─────────────────────────────────────────────────────────
    #   ICP 主逻辑
    # ─────────────────────────────────────────────────────────

    def _run_icp(self):
        if self._sub:
            self.destroy_subscription(self._sub)
            self._sub = None

        arr = np.array(self._pts_buf, dtype=np.float32)
        scan = o3d.geometry.PointCloud()
        scan.points = o3d.utility.Vector3dVector(arr[:, :3])
        scan = scan.voxel_down_sample(self._voxel)
        scan.estimate_normals(
            search_param=o3d.geometry.KDTreeSearchParamHybrid(radius=0.5, max_nn=30))

        label = self._pass_label
        self.get_logger().info(
            f'[{label}] ICP 开始: scan={len(scan.points)} pt | map={len(self._map.points)} pt')

        result = o3d.pipelines.registration.registration_icp(
            scan, self._map,
            max_correspondence_distance=self._icp_dist,
            init=np.eye(4),
            estimation_method=o3d.pipelines.registration.TransformationEstimationPointToPlane(),
            criteria=o3d.pipelines.registration.ICPConvergenceCriteria(max_iteration=100),
        )

        T   = result.transformation
        dx  = T[0, 3]   # FAST-LIO 当前坐标系原点在 MAP 坐标系中的 x 偏移
        dy  = T[1, 3]   # FAST-LIO 当前坐标系原点在 MAP 坐标系中的 y 偏移
        dyaw = math.atan2(T[1, 0], T[0, 0])

        self.get_logger().info(
            f'[{label}] ICP: fitness={result.fitness:.4f}  RMSE={result.inlier_rmse:.4f}m  '
            f'offset=({dx:+.3f}, {dy:+.3f}, {math.degrees(dyaw):+.1f}°)')

        ok = result.fitness >= self._icp_fit
        if not ok:
            self.get_logger().warn(
                f'[{label}] ICP fitness={result.fitness:.4f} < 阈值 {self._icp_fit}, '
                f'使用先验偏移量 (不校正)')
            dx = dy = dyaw = 0.0

        # ── 将 MAP 帧偏移转换为游戏坐标修正 ──────────────────
        # prior_offset_x/y/yaw 始终保持启动时的常量 (FAST-LIO坐标系原点=游戏坐标),
        # 每轮独立地用该常量 + 本轮ICP结果, 重新算出当前应使用的 loc_offset,
        # 从而吸收掉本轮重定位之前积累的 SLAM 漂移
        c, s = math.cos(self._off_yaw), math.sin(self._off_yaw)
        new_x   = self._off_x   + c * dx - s * dy
        new_y   = self._off_y   + s * dx + c * dy
        new_yaw = self._off_yaw + dyaw

        status = '✓ 校正成功' if ok else '⚠ 使用先验'
        self.get_logger().info(
            f'[{label}] [{status}] 新偏移: x={new_x:.4f}  y={new_y:.4f}  '
            f'yaw={math.degrees(new_yaw):.2f}°  '
            f'(耗时 {time.time()-self._t_start:.1f}s)')

        self._update_nav_params(new_x, new_y, new_yaw)
        self._publish_initialpose(dx, dy, dyaw)

        status_msg = String()
        status_msg.data = f'{label}:{"ok" if ok else "fallback"}'
        self._status_pub.publish(status_msg)

        self._running = False

    # ─────────────────────────────────────────────────────────
    #   更新 waypoint_navigator 参数
    # ─────────────────────────────────────────────────────────

    def _update_nav_params(self, x: float, y: float, yaw: float):
        if not self._set_param_client.wait_for_service(timeout_sec=5.0):
            self.get_logger().warn(
                'waypoint_navigator 参数服务未就绪, 跳过在线更新\n'
                f'  请手动启动时传入: loc_offset_x:={x:.4f} '
                f'loc_offset_y:={y:.4f} loc_offset_yaw:={yaw:.6f}')
            return

        def _p(name, val):
            p = Parameter()
            p.name = name
            p.value.type = ParameterType.PARAMETER_DOUBLE
            p.value.double_value = float(val)
            return p

        req = SetParameters.Request()
        req.parameters = [
            _p('loc_offset_x',   x),
            _p('loc_offset_y',   y),
            _p('loc_offset_yaw', yaw),
        ]
        future = self._set_param_client.call_async(req)
        rclpy.spin_until_future_complete(self, future, timeout_sec=5.0)
        if future.result() and all(r.successful for r in future.result().results):
            self.get_logger().info('waypoint_navigator loc_offset 已在线更新 ✓')
        else:
            self.get_logger().warn('参数更新失败 (可能已超时)')

    # ─────────────────────────────────────────────────────────
    #   发布 initialpose (供 RViz 可视化)
    # ─────────────────────────────────────────────────────────

    def _publish_initialpose(self, dx: float, dy: float, dyaw: float):
        msg = PoseWithCovarianceStamped()
        msg.header.frame_id = 'map'
        msg.header.stamp    = self.get_clock().now().to_msg()
        msg.pose.pose.position.x    = dx
        msg.pose.pose.position.y    = dy
        msg.pose.pose.orientation.z = math.sin(dyaw / 2.0)
        msg.pose.pose.orientation.w = math.cos(dyaw / 2.0)
        self._pose_pub.publish(msg)


def main(args=None):
    rclpy.init(args=args)
    try:
        node = MapRelocalizer()
        rclpy.spin(node)
    except (RuntimeError, KeyboardInterrupt):
        pass
    finally:
        rclpy.shutdown()


if __name__ == '__main__':
    main()
