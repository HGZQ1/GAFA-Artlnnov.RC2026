"""
meilin_navigator.py
梅林方块位置追踪与导航状态机

职责：
  1. 根据融合里程计（/odom/fused）判断当前在哪个方块
  2. 根据路径规划（meilin_map.py）给出下一个目标方块
  3. 结合 IMU 坡度判断是否需要切换爬坡模式
  4. 发布导航状态（当前方块、下一方块、运动模式）

运动模式（MoveMode）：
  FLAT     平地行走（块间高度差 < 阈值）
  CLIMB    爬坡（前方方块更高）
  DESCEND  下坡（前方方块更低）
  SETTLE   稳定等待（刚到达方块顶部，等待稳定）
  DONE     已完成梅林路径
"""
import math
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32MultiArray, String, Int8

from .config import (
    BLOCK_HEIGHTS, BLOCK_CENTER_SPACING,
    BLOCK_ARRIVAL_ODOM_THR, BLOCK_SETTLE_TIME,
    MEILIN_ENTRY_DIST, BLOCK_ODOM_DISTANCES,
    SLOPE_DETECT_DEG,
)


class MoveMode:
    FLAT    = 'FLAT'
    CLIMB   = 'CLIMB'
    DESCEND = 'DESCEND'
    SETTLE  = 'SETTLE'
    DONE    = 'DONE'


class MeilinNavigator(Node):
    """
    梅林导航节点

    发布话题：
      /meilin/nav_state  Float32MultiArray
        data[0] = current_block   当前方块号（0=entry, 99=exit）
        data[1] = next_block      下一目标方块号
        data[2] = move_mode       0=FLAT,1=CLIMB,2=DESCEND,3=SETTLE,4=DONE
        data[3] = height_diff     当前→下一方块的高度差（米，正=上坡）
        data[4] = dist_to_next    到下一方块中心的估计距离（米）
        data[5] = speed_factor    建议速度系数（0~1）
        data[6] = torque_factor   建议驱动力系数（0~1）
    """

    MOVE_MODE_ID = {
        MoveMode.FLAT:    0,
        MoveMode.CLIMB:   1,
        MoveMode.DESCEND: 2,
        MoveMode.SETTLE:  3,
        MoveMode.DONE:    4,
    }

    def __init__(self):
        super().__init__('meilin_navigator')

        # 当前路径（由外部设置，默认中线路径）
        self._path     = ['entry', 2, 5, 8, 11, 'exit']
        self._path_idx = 0   # 当前在路径中的索引

        # 里程计状态
        self._odom_x     = 0.0
        self._odom_y     = 0.0
        self._odom_dist  = 0.0
        self._pitch_deg  = 0.0

        # 稳定等待计时
        self._settle_start = None
        self._in_settle    = False

        # 运动模式
        self._move_mode = MoveMode.FLAT

        # 订阅融合里程计
        self.create_subscription(
            Float32MultiArray, '/odom/fused',
            self._on_odom, 10)

        # 订阅路径指令（由 processor_node 在梅林阶段发布）
        self.create_subscription(
            String, '/meilin/set_path',
            self._on_set_path, 10)

        # 发布导航状态
        self.nav_pub = self.create_publisher(
            Float32MultiArray, '/meilin/nav_state', 10)

        # 定时发布（20Hz）
        self.create_timer(0.05, self._update_and_publish)

        # 重置订阅
        self.create_subscription(
            Int8, '/meilin/reset',
            lambda msg: self._reset(), 10)

        self.get_logger().info('梅林导航节点已启动')

    def _on_odom(self, msg: Float32MultiArray):
        if len(msg.data) >= 5:
            self._odom_x    = msg.data[0]
            self._odom_y    = msg.data[1]
            self._odom_dist = msg.data[4]
            self._pitch_deg = msg.data[3]

    def _on_set_path(self, msg: String):
        """
        接收路径字符串，格式如 "entry,2,5,8,11,exit"
        """
        parts = msg.data.strip().split(',')
        path  = []
        for p in parts:
            p = p.strip()
            if p in ('entry', 'exit'):
                path.append(p)
            else:
                try:
                    path.append(int(p))
                except ValueError:
                    pass
        if len(path) >= 2:
            self._path     = path
            self._path_idx = 0
            self.get_logger().info(
                f'梅林路径已设置：{path}')

    def _get_current_block(self):
        """返回当前方块标识"""
        if self._path_idx < len(self._path):
            return self._path[self._path_idx]
        return 'exit'

    def _get_next_block(self):
        """返回下一个目标方块标识"""
        next_idx = self._path_idx + 1
        if next_idx < len(self._path):
            return self._path[next_idx]
        return 'exit'

    def _check_block_arrival(self) -> bool:
        """
        根据里程计判断是否已到达下一个方块
        使用累计行驶距离对比方块间距
        """
        next_block = self._get_next_block()
        expected_dist = BLOCK_ODOM_DISTANCES.get(next_block, 0)
        return self._odom_dist >= expected_dist - 0.05

    def _calc_move_mode(self) -> tuple:
        """
        计算当前应该使用的运动模式和建议速度

        Returns:
            (move_mode_str, height_diff, speed_factor, torque_factor)
        """
        cur  = self._get_current_block()
        next = self._get_next_block()

        if next == 'exit' or self._path_idx >= len(self._path) - 1:
            return MoveMode.DONE, 0.0, 0.0, 0.0

        cur_h  = BLOCK_HEIGHTS.get(cur,  0.0)
        next_h = BLOCK_HEIGHTS.get(next, 0.0)
        h_diff = next_h - cur_h

        # 用IMU实时坡度辅助判断（比仅靠高度表更可靠）
        imu_climbing = self._pitch_deg > SLOPE_DETECT_DEG
        imu_descend  = self._pitch_deg < -SLOPE_DETECT_DEG

        if self._in_settle:
            return MoveMode.SETTLE, h_diff, 0.0, 0.0

        if h_diff > 0.03:   # 上坡（高度差 > 3cm）
            speed  = self._slope_speed_factor(abs(self._pitch_deg))
            torque = self._slope_torque_factor(abs(self._pitch_deg))
            return MoveMode.CLIMB, h_diff, speed, torque

        elif h_diff < -0.03:  # 下坡
            from .config import BRAKE_FACTOR_SLOPE
            speed  = BRAKE_FACTOR_SLOPE
            torque = 0.5
            return MoveMode.DESCEND, h_diff, speed, torque

        else:
            from .config import SPEED_FACTOR_FLAT, TORQUE_FACTOR_FLAT
            return MoveMode.FLAT, h_diff, SPEED_FACTOR_FLAT, TORQUE_FACTOR_FLAT

    def _slope_speed_factor(self, slope_deg: float) -> float:
        from .config import (SLOPE_LEVEL_MILD, SLOPE_LEVEL_MODERATE,
                             SPEED_FACTOR_MILD, SPEED_FACTOR_MODERATE,
                             SPEED_FACTOR_STEEP)
        if slope_deg < SLOPE_LEVEL_MILD:
            return SPEED_FACTOR_MILD
        elif slope_deg < SLOPE_LEVEL_MODERATE:
            return SPEED_FACTOR_MODERATE
        else:
            return SPEED_FACTOR_STEEP

    def _slope_torque_factor(self, slope_deg: float) -> float:
        from .config import (SLOPE_LEVEL_MILD, SLOPE_LEVEL_MODERATE,
                             TORQUE_FACTOR_MILD, TORQUE_FACTOR_MODERATE,
                             TORQUE_FACTOR_STEEP)
        if slope_deg < SLOPE_LEVEL_MILD:
            return TORQUE_FACTOR_MILD
        elif slope_deg < SLOPE_LEVEL_MODERATE:
            return TORQUE_FACTOR_MODERATE
        else:
            return TORQUE_FACTOR_STEEP

    def _update_and_publish(self):
        """主循环：更新方块状态并发布"""
        # 检查是否到达下一个方块
        if (not self._in_settle and
                self._path_idx < len(self._path) - 1 and
                self._check_block_arrival()):

            self._path_idx += 1
            self._in_settle    = True
            self._settle_start = time.time()
            self.get_logger().info(
                f'到达方块：{self._get_current_block()}')

        # 检查稳定等待是否结束
        if (self._in_settle and
                self._settle_start is not None and
                time.time() - self._settle_start > BLOCK_SETTLE_TIME):
            self._in_settle = False
            self.get_logger().info(
                f'方块稳定，前往：{self._get_next_block()}')

        # 计算运动模式
        mode, h_diff, speed, torque = self._calc_move_mode()
        self._move_mode = mode

        # 估算到下一方块的距离
        next_block    = self._get_next_block()
        expected_dist = BLOCK_ODOM_DISTANCES.get(next_block, 0)
        dist_to_next  = max(0.0, expected_dist - self._odom_dist)

        # 发布导航状态
        nav_msg = Float32MultiArray()
        nav_msg.data = [
            float(self._path_idx),                         # [0] 路径索引
            float(self._path.index(self._get_next_block())
                  if self._get_next_block() in self._path
                  else 99),                                 # [1] 下一方块索引
            float(self.MOVE_MODE_ID.get(mode, 0)),         # [2] 运动模式
            float(h_diff),                                 # [3] 高度差
            float(dist_to_next),                           # [4] 到下一方块距离
            float(speed),                                  # [5] 速度系数
            float(torque),                                 # [6] 驱动力系数
        ]
        self.nav_pub.publish(nav_msg)

    def _reset(self):
        self._path_idx  = 0
        self._in_settle = False
        self.get_logger().info('梅林导航已重置')


def main(args=None):
    rclpy.init(args=args)
    rclpy.spin(MeilinNavigator())
    rclpy.shutdown()