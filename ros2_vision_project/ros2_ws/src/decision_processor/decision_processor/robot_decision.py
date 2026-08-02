"""
robot_decision.py
五状态决策状态机（以距离为核心判据）
"""
import time
import math
from enum import Enum, auto

from .target_confirmation import TargetConfirmation
from .kalman_filter       import KalmanFilter2D
from .motion_planner      import MotionPlanner, TrapezoidPlanner
from .config import (
    STOP_DISTANCE_M,
    ARRIVAL_SETTLE_S,
    PICK_DURATION_S,
    ALIGN_THRESHOLD_DEG,
    ALIGN_TURN_GAIN,
    FORWARD_SPEED_GAIN,
    WEAPON_APPROACH_MAX_SPEED_MPS,
    WEAPON_SERVO_LEVEL_WAIT_S,
    WUGUAN_TOTAL_WEAPONS,
    CONFIRM_FRAMES,
    LOST_FRAMES,
)


class RobotState(Enum):
    SEARCHING = auto()
    ALIGNING  = auto()
    MOVING    = auto()
    ARRIVED   = auto()
    PICKING   = auto()
    SERVO_WAIT = auto()


class RobotDecision:

    def __init__(self,
                 wheel_diameter_m:    float = 0.096,
                 track_width_m:       float = 0.28,
                 stop_distance_m:     float = STOP_DISTANCE_M,
                 align_threshold_deg: float = ALIGN_THRESHOLD_DEG,
                 pick_duration_s:     float = PICK_DURATION_S,
                 conf_threshold:      float = 0.5):

        self.state = RobotState.SEARCHING

        self.confirmation = TargetConfirmation(confirm_frames=CONFIRM_FRAMES, lost_frames=LOST_FRAMES)
        self.kalman       = KalmanFilter2D(dt=0.05)
        self.planner      = MotionPlanner(
            wheel_diameter_m    = wheel_diameter_m,
            track_width_m       = track_width_m,
            stop_distance_m     = stop_distance_m,
            align_threshold_deg = align_threshold_deg,
        )
        self.trap = TrapezoidPlanner()

        self.conf_thr      = conf_threshold
        self.stop_dist     = stop_distance_m
        self.align_thr_deg = align_threshold_deg
        self.pick_duration = pick_duration_s
        self.settle_time   = ARRIVAL_SETTLE_S
        self.weapon_approach_max_speed = WEAPON_APPROACH_MAX_SPEED_MPS

        self._last_pos       = None
        self._arrival_start  = 0.0
        self._pick_start     = 0.0
        self._approach_remaining_m = 0.0
        self._approach_last_t = 0.0
        self._approach_last_speed = 0.0

        self.pick_count      = 0
        self.all_picked      = False
        self._all_done_fired = False

        self.pause_at_arrived = False
        self.enable_weapon_servo = False

        self.align_angle_deg = 0.0
        self.target_distance = 0.0
        self.arm_target_dist = 0.0
        self.pick_elapsed    = 0.0
        self.settle_remain   = 0.0
        self.servo_wait_remain = 0.0
        self.forward_speed_cmd = 0.0
        self.weapon_speed_limited = False

        self._servo_wait_start = 0.0

        # 下位机夹爪反馈：收到"已抓取"后可提前结束PICKING计时
        self._pick_feedback_done = False
        self.pick_done_by_feedback = False

        self.event = ''

    def update(self, detected: bool,
               base_x: float, base_y: float,
               distance: float,
               align_angle: float,
               arm_distance: float = 0.0) -> dict:
        """
        Args:
            detected:    本帧是否检测到目标
            base_x/y:    底盘坐标系下的目标位置（用于运动规划）
            distance:    相机到目标直线距离（用于到达判定）
            align_angle: 目标相对相机中轴线的水平偏角（度）
                         正=物体在左侧，负=物体在右侧，0=正中
                         由 processor_node 直接从相机水平偏移计算
            arm_distance: 目标到机械臂底座的距离
        """
        now = time.time()
        cmd = self._stop_cmd()
        self.event = ''
        self.forward_speed_cmd = 0.0
        self.weapon_speed_limited = False

        confirmed = self.confirmation.update(detected)

        # 只在本帧实际看到目标时更新显示值
        if detected and distance > 0:
            self.align_angle_deg = align_angle
            self.target_distance = distance

        # ── SEARCHING ──
        if self.state == RobotState.SEARCHING:
            if self.all_picked:
                if not self._all_done_fired:
                    self._all_done_fired = True
                    self.event = 'ALL_DONE'
                return cmd
            if confirmed:
                self.state = RobotState.ALIGNING
                self.event = 'TARGET_LOCKED'
            else:
                cmd['search_rotate'] = 1.0

        # ── ALIGNING ──
        elif self.state == RobotState.ALIGNING:
            if not confirmed:
                self._reset_tracking()
                self.state = RobotState.SEARCHING
                self.event = 'TARGET_LOST'
            elif not detected:
                # 确认器没超时但本帧没看到，等待
                pass
            else:
                # 本帧看到了，用实时角度判断
                if abs(align_angle) > self.align_thr_deg:
                    # 将相机测量的对齐角按比例作为角速度命令(deg/s)，避免
                    # planner.plan() 内置的到达检查
                    # 在近距离时错误返回 turn_deg=0（plan 会因 dist≤arrival_thr 提前返回）
                    cmd['turn_angle'] = align_angle * ALIGN_TURN_GAIN
                else:
                    if self.enable_weapon_servo:
                        self._start_weapon_approach(now, distance)
                        self._servo_wait_start = now
                        self.servo_wait_remain = WEAPON_SERVO_LEVEL_WAIT_S
                        self.state = RobotState.SERVO_WAIT
                        self.event = 'WEAPON_SERVO_LEVEL'
                    else:
                        self._start_weapon_approach(now, distance)
                        self.state = RobotState.MOVING
                        self.event = 'ALIGNED'

        # ── SERVO_WAIT ──
        elif self.state == RobotState.SERVO_WAIT:
            elapsed = now - self._servo_wait_start
            self.servo_wait_remain = max(0.0, WEAPON_SERVO_LEVEL_WAIT_S - elapsed)
            if elapsed >= WEAPON_SERVO_LEVEL_WAIT_S:
                self._approach_last_t = now
                self._approach_last_speed = 0.0
                self.state = RobotState.MOVING
                self.event = 'ALIGNED'

        # ── MOVING ──
        elif self.state == RobotState.MOVING:
            if self.enable_weapon_servo:
                self._update_weapon_approach(now, detected, distance)
                if self._approach_remaining_m <= 0.01:
                    self._arrival_start = now
                    self._approach_last_speed = 0.0
                    self.state = RobotState.ARRIVED
                    self.event = 'ARRIVED'
                else:
                    speed = self._approach_remaining_m * FORWARD_SPEED_GAIN
                    if (self.weapon_approach_max_speed > 0.0
                            and speed > self.weapon_approach_max_speed):
                        speed = self.weapon_approach_max_speed
                        self.weapon_speed_limited = True
                    self.forward_speed_cmd = speed
                    self._approach_last_speed = speed
                    cmd['forward_dist'] = speed
            elif not confirmed:
                self._reset_tracking()
                self.state = RobotState.SEARCHING
                self.event = 'TARGET_LOST'
            elif not detected:
                pass
            else:
                # if abs(align_angle) > self.align_thr_deg * 2:
                #     self.state = RobotState.ALIGNING
                #     self.event = 'REALIGN'
                if distance <= self.stop_dist:
                    self._arrival_start = now
                    self.state = RobotState.ARRIVED
                    self.event = 'ARRIVED'
                else:
                    # 直接用相机深度计算前进速度，绕过 planner 的对齐检查干扰
                    fwd = max(0.0, distance - self.stop_dist)
                    speed = fwd * FORWARD_SPEED_GAIN
                    if (self.enable_weapon_servo
                            and self.weapon_approach_max_speed > 0.0
                            and speed > self.weapon_approach_max_speed):
                        speed = self.weapon_approach_max_speed
                        self.weapon_speed_limited = True
                    self.forward_speed_cmd = speed
                    cmd['forward_dist'] = speed

        # ── ARRIVED ──
        elif self.state == RobotState.ARRIVED:
            elapsed = now - self._arrival_start
            self.settle_remain = max(0, self.settle_time - elapsed)
            if not self.pause_at_arrived and elapsed >= self.settle_time:
                self._pick_start = now
                self.arm_target_dist = arm_distance
                self._pick_feedback_done = False
                self.state = RobotState.PICKING
                self.event = 'PICK_START'

        # ── PICKING ──
        elif self.state == RobotState.PICKING:
            cmd['pickup_action'] = 1.0
            cmd['arm_distance']  = self.arm_target_dist
            self.pick_elapsed = now - self._pick_start

            # 退出条件：超时(兜底) 或 下位机夹爪反馈"已抓取"(提前结束)
            if self.pick_elapsed >= self.pick_duration or self._pick_feedback_done:
                self.pick_done_by_feedback = self._pick_feedback_done
                self.pick_count += 1
                self.event = 'PICK_DONE'
                if self.pick_count >= WUGUAN_TOTAL_WEAPONS:
                    self.all_picked = True
                self._pick_feedback_done = False
                self._reset_tracking()
                self.state = RobotState.SEARCHING

        return cmd

    def _stop_cmd(self) -> dict:
        return {
            'turn_angle': 0.0, 'turn_wheels': 0.0,
            'forward_dist': 0.0, 'drive_wheels': 0.0,
            'pickup_action': 0.0, 'search_rotate': 0.0,
            'arm_distance': 0.0,
        }

    def notify_gripper_grabbed(self):
        """收到下位机夹爪反馈"已抓取"，PICKING状态下一次update()时提前结束计时"""
        self._pick_feedback_done = True

    def _reset_tracking(self):
        self.kalman.reset()
        self.confirmation.reset()
        self._last_pos = None
        self._reset_weapon_approach()

    def _start_weapon_approach(self, now: float, distance: float):
        """Lock the forward-only weapon approach distance after ALIGNING."""
        self._approach_remaining_m = max(0.0, distance - self.stop_dist)
        self._approach_last_t = now
        self._approach_last_speed = 0.0

    def _update_weapon_approach(self, now: float, detected: bool, distance: float):
        if self._approach_last_t <= 0.0:
            seed_distance = distance if distance > 0.0 else self.target_distance
            self._start_weapon_approach(now, seed_distance)

        dt = max(0.0, now - self._approach_last_t)
        self._approach_last_t = now
        self._approach_remaining_m = max(
            0.0,
            self._approach_remaining_m - self._approach_last_speed * dt)

        if detected and distance > 0.0:
            live_remaining = max(0.0, distance - self.stop_dist)
            self._approach_remaining_m = min(
                self._approach_remaining_m,
                live_remaining)

        self.target_distance = self.stop_dist + self._approach_remaining_m

    def _reset_weapon_approach(self):
        self._approach_remaining_m = 0.0
        self._approach_last_t = 0.0
        self._approach_last_speed = 0.0

    def reset_all(self):
        self._reset_tracking()
        self.state = RobotState.SEARCHING
        self.pick_count = 0
        self.all_picked = False
        self._all_done_fired = False
        self.servo_wait_remain = 0.0
        self._servo_wait_start = 0.0
        self.forward_speed_cmd = 0.0
        self.weapon_speed_limited = False

    @property
    def state_name(self) -> str:
        return self.state.name
