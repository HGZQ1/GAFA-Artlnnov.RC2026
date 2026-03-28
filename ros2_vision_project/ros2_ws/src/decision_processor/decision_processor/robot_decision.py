"""
robot_decision.py
整合所有算法的主决策类，每帧调用 step() 返回底盘指令
"""
import time
from enum import Enum, auto

import numpy as np


from .target_confirmation import TargetConfirmation, select_best_target
from .kalman_filter        import KalmanFilter2D, camera_to_robot, is_valid_detection
from .motion_planner       import MotionPlanner, TrapezoidPlanner

# 需要从 vision_detector 的 utils.py 导入坐标转换函数
# 如果打包方式不同，可以直接复制函数到本文件
def pixel_to_3d_point(u, v, depth, camera_matrix):
    """像素坐标 + 深度 → 相机坐标系3D坐标"""
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    return (u - cx) * depth / fx, (v - cy) * depth / fy, depth
# try:
    
# except ImportError:
#     def pixel_to_3d_point(u, v, depth, camera_matrix):
#         fx = camera_matrix[0, 0]
#         fy = camera_matrix[1, 1]
#         cx = camera_matrix[0, 2]
#         cy = camera_matrix[1, 2]
#         z = depth
#         x = (u - cx) * z / fx
#         y = (v - cy) * z / fy
#         return x, y, z


class RobotState(Enum):
    SEARCHING = auto()
    ALIGNING  = auto()
    MOVING    = auto()
    ARRIVED   = auto()
    PICKING   = auto()


class RobotDecision:
    """
    整合感知、处理、规划三层的主决策类
    每帧调用 step()，返回发给 auto_serial_bridge 的指令字典
    """

    def __init__(self,
                 wheel_diameter_m:    float = 0.096,
                 track_width_m:       float = 0.28,
                 stop_distance_m:     float = 0.20,
                 align_threshold_deg: float = 5.0,
                 pick_duration_s:     float = 3.0,
                 conf_threshold:      float = 0.5):
        """
        Args:
            wheel_diameter_m:    车轮直径（米），按实际填写
            track_width_m:       轮距（米），按实际填写
            stop_distance_m:     距目标多远时停止前进
            align_threshold_deg: 对准角度容限（度）
            pick_duration_s:     拾取动作持续时间（秒）
            conf_threshold:      YOLO 置信度阈值
        """
        self.state = RobotState.SEARCHING

        # 初始化各层模块
        self.confirmation = TargetConfirmation(
            confirm_frames=3, lost_frames=5)
        self.kalman       = KalmanFilter2D(dt=0.05)
        self.planner      = MotionPlanner(
            wheel_diameter_m    = wheel_diameter_m,
            track_width_m       = track_width_m,
            stop_distance_m     = stop_distance_m,
            align_threshold_deg = align_threshold_deg)
        self.trap         = TrapezoidPlanner()

        self.conf_thr      = conf_threshold
        self.pick_duration = pick_duration_s

        self._last_pos     = None      # 上一帧目标位置，用于跳变检测
        self._pick_start   = 0.0       # 拾取开始时间戳

    def step(self, detections: list,
             distances: list,
             camera_matrix: np.ndarray) -> dict:
        """
        每帧调用一次，输入当前检测结果，输出底盘指令

        Args:
            detections:    YOLOv8 检测结果列表（detector_node 输出）
            distances:     各目标距离列表（米）
            camera_matrix: 相机内参矩阵 (3x3 numpy array)
        Returns:
            cmd: 指令字典，直接映射到 protocol.yaml 的字段
                 {turn_angle, turn_wheels, forward_dist,
                  drive_wheels, pickup_action, search_rotate}
        """
        # ── 感知层：选最优目标，连续帧确认 ──
        best      = select_best_target(detections, distances,
                                       self.conf_thr)
        confirmed = self.confirmation.update(best is not None)

        # ── 数据处理层：滤波 + 坐标转换 ──
        if confirmed and best is not None:
            det, dist = best
            x1, y1, x2, y2 = det['bbox']
            u = int((x1 + x2) / 2)
            v = int((y1 + y2) / 2)

            # 像素坐标 → 相机3D坐标
            cam_x, cam_y, _ = pixel_to_3d_point(
                u, v, dist, camera_matrix)

            # 相机坐标 → 机器人坐标
            robot_x, robot_y = camera_to_robot(cam_x, cam_y, dist)

            # 跳变检测
            new_pos = (robot_x, robot_y)
            if not is_valid_detection(new_pos, self._last_pos):
                return self._stop_cmd()   # 误检，本帧跳过

            self._last_pos = new_pos

            # 卡尔曼滤波平滑
            rx, ry = self.kalman.update(robot_x, robot_y)
        else:
            rx, ry = 0.0, 0.0
            if not confirmed:
                self._last_pos = None
                self.kalman.reset()

        # ── 规划层：计算运动指令 ──
        plan = self.planner.plan(rx, ry) if confirmed else None

        # ── 决策层：状态机 ──
        return self._state_machine(confirmed, plan)

    def _state_machine(self, confirmed: bool,
                       plan) -> dict:
        now = time.time()
        cmd = self._stop_cmd()

        if self.state == RobotState.SEARCHING:
            if confirmed:
                self.state = RobotState.ALIGNING
            else:
                cmd['search_rotate'] = 1.0   # 告知STM32原地旋转搜索

        elif self.state == RobotState.ALIGNING:
            if not confirmed:
                self.state = RobotState.SEARCHING
                self.kalman.reset()
            elif plan.phase == 'ALIGNING':
                cmd['turn_angle']   = plan.turn_deg
                cmd['turn_wheels']  = plan.turn_wheels
            elif plan.phase in ('MOVING', 'ARRIVED'):
                self.state = RobotState.MOVING

        elif self.state == RobotState.MOVING:
            if not confirmed:
                self.state = RobotState.SEARCHING
                self.kalman.reset()
            elif plan.phase == 'ARRIVED':
                self.state = RobotState.ARRIVED
            elif plan.phase == 'ALIGNING':
                # 前进途中偏转，退回对准
                self.state = RobotState.ALIGNING
            else:
                cmd['forward_dist']  = plan.forward_m
                cmd['drive_wheels']  = plan.drive_wheels

        elif self.state == RobotState.ARRIVED:
            # 到达后立即触发拾取
            self._pick_start = now
            self.state = RobotState.PICKING

        elif self.state == RobotState.PICKING:
            cmd['pickup_action'] = 1.0
            if (now - self._pick_start) >= self.pick_duration:
                # 拾取完成，重置寻找下一个目标
                self.state = RobotState.SEARCHING
                self.kalman.reset()
                self.confirmation.reset()
                self._last_pos = None

        return cmd

    def _stop_cmd(self) -> dict:
        """返回全停指令"""
        return {
            'turn_angle':    0.0,
            'turn_wheels':   0.0,
            'forward_dist':  0.0,
            'drive_wheels':  0.0,
            'pickup_action': 0.0,
            'search_rotate': 0.0,
        }

    @property
    def state_name(self) -> str:
        return self.state.name