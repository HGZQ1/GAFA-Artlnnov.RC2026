"""
motion_planner.py
两阶段运动规划 + 梯形速度规划 + 坡度感知
"""
import math
from .config import (
    WHEEL_DIAMETER_M, TRACK_WIDTH_M,
    STOP_DISTANCE_M, ARRIVAL_THRESHOLD_M, ALIGN_THRESHOLD_DEG,
    SLOPE_DETECT_DEG, SLOPE_LEVEL_MILD, SLOPE_LEVEL_MODERATE,
    SPEED_FACTOR_FLAT, SPEED_FACTOR_MILD,
    SPEED_FACTOR_MODERATE, SPEED_FACTOR_STEEP,
    TORQUE_FACTOR_FLAT, TORQUE_FACTOR_MILD,
    TORQUE_FACTOR_MODERATE, TORQUE_FACTOR_STEEP,
    BRAKE_FACTOR_SLOPE,
)


class MotionPlan:
    def __init__(self):
        self.phase        = 'STOP'
        self.turn_deg     = 0.0
        self.turn_wheels  = 0.0
        self.forward_m    = 0.0
        self.drive_wheels = 0.0
        # 3D 扩展字段（新增）
        self.speed_factor  = 1.0   # 速度系数 0~1
        self.torque_factor = 0.5   # 驱动力系数 0~1
        self.climb_mode    = 0     # 0=平地 1=上坡 2=下坡
        self.slope_deg     = 0.0   # 当前坡度角


class MotionPlanner:

    def __init__(self,
                 wheel_diameter_m:    float = WHEEL_DIAMETER_M,
                 track_width_m:       float = TRACK_WIDTH_M,
                 stop_distance_m:     float = STOP_DISTANCE_M,
                 arrival_threshold_m: float = ARRIVAL_THRESHOLD_M,
                 align_threshold_deg: float = ALIGN_THRESHOLD_DEG):

        self.wheel_circ  = math.pi * wheel_diameter_m
        self.track_width = track_width_m
        self.stop_dist   = stop_distance_m
        self.arrival_thr = arrival_threshold_m
        self.align_thr   = align_threshold_deg

        # 当前坡度信息（由 processor_node 更新）
        self._current_pitch = 0.0
        self._slope_level   = 0

        print(f"[MotionPlanner] 轮径={wheel_diameter_m*100:.1f}cm  "
              f"轮距={track_width_m*100:.1f}cm  "
              f"停止距离={stop_distance_m*100:.0f}cm")

    def update_slope(self, pitch_deg: float, slope_level: int):
        """
        由 processor_node 每帧调用，更新坡度状态
        pitch_deg > 0 = 上坡, < 0 = 下坡
        slope_level = 0~3 (FLAT/MILD/MODERATE/STEEP)
        """
        self._current_pitch = pitch_deg
        self._slope_level   = slope_level

    def plan(self, robot_x: float, robot_y: float) -> MotionPlan:
        plan = MotionPlan()
        dist = math.sqrt(robot_x ** 2 + robot_y ** 2)

        if dist <= self.arrival_thr:
            plan.phase = 'ARRIVED'
            return plan

        turn_rad    = math.atan2(robot_y, robot_x)
        turn_deg    = math.degrees(turn_rad)
        turn_arc    = abs(turn_rad) * self.track_width / 2.0
        turn_wheels = turn_arc / self.wheel_circ

        if abs(turn_deg) > self.align_thr:
            plan.phase       = 'ALIGNING'
            plan.turn_deg    = turn_deg
            plan.turn_wheels = round(turn_wheels, 4)
        else:
            fwd_dist = max(0.0, dist - self.stop_dist)
            if fwd_dist <= 0.001:
                plan.phase = 'ARRIVED'
            else:
                plan.phase        = 'MOVING'
                plan.forward_m    = round(fwd_dist, 4)
                plan.drive_wheels = round(fwd_dist / self.wheel_circ, 4)

        # ── 坡度感知：填充3D扩展字段 ──
        plan.slope_deg = self._current_pitch

        pitch_abs = abs(self._current_pitch)
        if pitch_abs < SLOPE_DETECT_DEG:
            # 平地模式
            plan.climb_mode    = 0
            plan.speed_factor  = SPEED_FACTOR_FLAT
            plan.torque_factor = TORQUE_FACTOR_FLAT

        elif self._current_pitch > 0:
            # 上坡模式：降速增力
            plan.climb_mode = 1
            if pitch_abs < SLOPE_LEVEL_MILD:
                plan.speed_factor  = SPEED_FACTOR_MILD
                plan.torque_factor = TORQUE_FACTOR_MILD
            elif pitch_abs < SLOPE_LEVEL_MODERATE:
                plan.speed_factor  = SPEED_FACTOR_MODERATE
                plan.torque_factor = TORQUE_FACTOR_MODERATE
            else:
                plan.speed_factor  = SPEED_FACTOR_STEEP
                plan.torque_factor = TORQUE_FACTOR_STEEP

        else:
            # 下坡模式：制动减速
            plan.climb_mode    = 2
            plan.speed_factor  = BRAKE_FACTOR_SLOPE
            plan.torque_factor = 0.4

        return plan

    def turns_for_rotation(self, angle_deg: float) -> float:
        arc = abs(math.radians(angle_deg)) * self.track_width / 2.0
        return round(arc / self.wheel_circ, 4)
        


class TrapezoidPlanner:

    def __init__(self, accel_dist_m=0.15, decel_dist_m=0.20,
                 min_speed_factor=0.15):
        self.accel_dist = accel_dist_m
        self.decel_dist = decel_dist_m
        self.min_speed  = min_speed_factor

    def get_speed_factor(self, traveled_m, total_m):
        if total_m <= 0:
            return 0.0
        remaining = total_m - traveled_m
        if traveled_m < self.accel_dist:
            factor = traveled_m / self.accel_dist
        elif remaining < self.decel_dist:
            factor = remaining / self.decel_dist
        else:
            factor = 1.0
        return max(self.min_speed, min(1.0, factor))