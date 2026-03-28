"""
motion_planner.py
两阶段运动规划 + 梯形速度规划
所有硬件参数从 config.py 读取，调参只需改 config.py
"""
import math
from .config import (
    WHEEL_DIAMETER_M,
    TRACK_WIDTH_M,
    STOP_DISTANCE_M,
    ARRIVAL_THRESHOLD_M,
    ALIGN_THRESHOLD_DEG,
)


class MotionPlan:
    """单次规划结果"""
    def __init__(self):
        self.phase        = 'STOP'
        self.turn_deg     = 0.0
        self.turn_wheels  = 0.0
        self.forward_m    = 0.0
        self.drive_wheels = 0.0


class MotionPlanner:
    """
    根据目标在机器人坐标系中的位置，计算底盘运动指令
    两阶段策略：先原地对准，再直线前进

    所有参数从 config.py 读取，无需在此修改
    """

    def __init__(self,
                 wheel_diameter_m:    float = WHEEL_DIAMETER_M,
                 track_width_m:       float = TRACK_WIDTH_M,
                 stop_distance_m:     float = STOP_DISTANCE_M,
                 arrival_threshold_m: float = ARRIVAL_THRESHOLD_M,
                 align_threshold_deg: float = ALIGN_THRESHOLD_DEG):
        """
        参数默认值全部来自 config.py
        也可以在实例化时手动覆盖，例如：
            MotionPlanner(wheel_diameter_m=0.10)
        """
        self.wheel_circ  = math.pi * wheel_diameter_m
        self.track_width = track_width_m
        self.stop_dist   = stop_distance_m
        self.arrival_thr = arrival_threshold_m
        self.align_thr   = align_threshold_deg

        # 打印当前使用的参数，方便调试时确认
        print(f"[MotionPlanner] 轮径={wheel_diameter_m*100:.1f}cm  "
              f"轮距={track_width_m*100:.1f}cm  "
              f"停止距离={stop_distance_m*100:.0f}cm  "
              f"对准阈值={align_threshold_deg:.1f}°")

    def plan(self, robot_x: float,
             robot_y: float) -> MotionPlan:
        """
        输入目标在机器人坐标系的 (x前, y左) 坐标
        输出本帧运动规划

        Args:
            robot_x: 目标前方距离（米），由 camera_to_robot() 转换得到
            robot_y: 目标左方偏移（米），左正右负
        Returns:
            MotionPlan 对象
        """
        plan = MotionPlan()
        dist = math.sqrt(robot_x ** 2 + robot_y ** 2)

        # ── 已到达目标 ──
        if dist <= self.arrival_thr:
            plan.phase = 'ARRIVED'
            return plan

        # ── 计算转向角 ──
        # atan2(y左, x前)：目标偏左=正值(左转)，偏右=负值(右转)
        turn_rad = math.atan2(robot_y, robot_x)
        turn_deg = math.degrees(turn_rad)

        # 原地差速转向时单侧轮子的圈数
        # 弧长 = 转角(rad) × 轮距/2，圈数 = 弧长 / 轮子周长
        turn_arc    = abs(turn_rad) * self.track_width / 2.0
        turn_wheels = turn_arc / self.wheel_circ

        if abs(turn_deg) > self.align_thr:
            # ── 阶段1：对准目标（角度偏差超过阈值）──
            plan.phase       = 'ALIGNING'
            plan.turn_deg    = turn_deg
            plan.turn_wheels = round(turn_wheels, 4)
        else:
            # ── 阶段2：直线前进 ──
            fwd_dist          = max(0.0, dist - self.stop_dist)
            # ── 修复：fwd_dist为0说明已到达停止距离，直接返回ARRIVED ──
            if fwd_dist <= 0.001:
                plan.phase = 'ARRIVED'
            else:
                plan.phase        = 'MOVING'
                plan.forward_m    = round(fwd_dist, 4)
                plan.drive_wheels = round(fwd_dist / self.wheel_circ, 4)

        return plan

    def turns_for_rotation(self, angle_deg: float) -> float:
        """
        纯转向时，单侧轮子需要转的圈数（供外部调用）
        正值=左转，负值=右转
        """
        arc = abs(math.radians(angle_deg)) * self.track_width / 2.0
        return round(arc / self.wheel_circ, 4)


class TrapezoidPlanner:
    """
    梯形速度规划：加速 → 匀速 → 减速
    输出 0~1 的速度系数，STM32 乘以最大速度得到目标速度
    """

    def __init__(self,
                 accel_dist_m:      float = 0.15,
                 decel_dist_m:      float = 0.20,
                 min_speed_factor:  float = 0.15):
        self.accel_dist = accel_dist_m
        self.decel_dist = decel_dist_m
        self.min_speed  = min_speed_factor

    def get_speed_factor(self, traveled_m: float,
                         total_m: float) -> float:
        """
        Args:
            traveled_m: 已行进距离（米）
            total_m:    总行进距离（米）
        Returns:
            0.0~1.0 速度系数
        """
        if total_m <= 0:
            return 0.0

        remaining = total_m - traveled_m

        if traveled_m < self.accel_dist:
            factor = traveled_m / self.accel_dist      # 加速段
        elif remaining < self.decel_dist:
            factor = remaining / self.decel_dist       # 减速段
        else:
            factor = 1.0                               # 匀速段

        return max(self.min_speed, min(1.0, factor))