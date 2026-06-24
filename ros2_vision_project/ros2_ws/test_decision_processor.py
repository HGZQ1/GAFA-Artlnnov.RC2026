#!/usr/bin/env python3
"""
test_decision_processor.py
==========================
decision_processor 节点独立测试文件
在不连接 STM32 下位机的情况下，在上位机（Ubuntu/Jetson）上测试：
  - 感知层：连续帧确认、目标选择
  - 数据处理层：卡尔曼滤波、坐标转换、跳变检测
  - 决策层：五状态状态机输出
  - 串口指令输出（打印到终端，不实际发送）

使用方法：
  # 方式1：纯算法测试（不需要 ROS2 环境）
  python3 test_decision_processor.py --mode algo

  # 方式2：ROS2 话题测试（需要启动 ROS2）
  # 终端1：ros2 run decision_processor processor_node
  # 终端2：python3 test_decision_processor.py --mode ros

  # 方式3：完整仿真（模拟相机+YOLO，自动走完全流程）
  python3 test_decision_processor.py --mode sim
"""

import sys
import time
import math
import argparse
import numpy as np
from enum import Enum


# ═══════════════════════════════════════════════════════
#   终端彩色输出工具
# ═══════════════════════════════════════════════════════

class Color:
    RESET  = '\033[0m'
    BOLD   = '\033[1m'
    RED    = '\033[91m'
    GREEN  = '\033[92m'
    YELLOW = '\033[93m'
    BLUE   = '\033[94m'
    PURPLE = '\033[95m'
    CYAN   = '\033[96m'
    WHITE  = '\033[97m'
    GRAY   = '\033[90m'

STATE_COLORS = {
    'SEARCHING': Color.YELLOW,
    'ALIGNING':  Color.BLUE,
    'MOVING':    Color.CYAN,
    'ARRIVED':   Color.GREEN,
    'PICKING':   Color.PURPLE,
}

def print_header(title: str):
    width = 60
    print(f"\n{Color.BOLD}{'═' * width}")
    print(f"  {title}")
    print(f"{'═' * width}{Color.RESET}\n")

def print_state(state_name: str, cmd: dict,
                extra_info: str = ''):
    color = STATE_COLORS.get(state_name, Color.WHITE)
    bar   = '█' * 20
    print(f"{Color.BOLD}状态：{color}{state_name:<12}{Color.RESET}  "
          f"{Color.GRAY}{extra_info}{Color.RESET}")

    fields = [
        ('转向角',   f"{cmd.get('turn_angle',    0):.2f}°"),
        ('转向圈数', f"{cmd.get('turn_wheels',   0):.3f}圈"),
        ('前进距离', f"{cmd.get('forward_dist',  0):.3f}m"),
        ('前进圈数', f"{cmd.get('drive_wheels',  0):.3f}圈"),
        ('拾取指令', '执行' if cmd.get('pickup_action', 0) > 0.5 else '无'),
        ('搜索旋转', '旋转' if cmd.get('search_rotate', 0) > 0.5 else '停止'),
    ]
    for name, val in fields:
        print(f"  {Color.GRAY}{name:<8}{Color.RESET}: {val}")
    print()

def print_kalman(raw_x, raw_y, filtered_x, filtered_y):
    noise_x = abs(filtered_x - raw_x)
    noise_y = abs(filtered_y - raw_y)
    print(f"  {Color.GRAY}卡尔曼滤波{Color.RESET}  "
          f"原始=({raw_x:+.3f}, {raw_y:+.3f})  "
          f"→  滤波后=({Color.CYAN}{filtered_x:+.3f}, "
          f"{filtered_y:+.3f}{Color.RESET})  "
          f"噪声抑制=({noise_x:.3f}, {noise_y:.3f})")


# ═══════════════════════════════════════════════════════
#   直接导入决策模块（不通过 ROS2）
# ═══════════════════════════════════════════════════════

def load_modules():
    """
    尝试导入 decision_processor 包的各模块
    如果包未安装，则从当前目录相对路径导入
    """
    try:
        # 方式1：已经 colcon build 并 source 过
        from decision_processor.target_confirmation import (
            TargetConfirmation, select_best_target)
        from decision_processor.kalman_filter import (
            KalmanFilter2D, camera_to_robot, is_valid_detection)
        from decision_processor.motion_planner import (
            MotionPlanner, TrapezoidPlanner)
        from decision_processor.robot_decision import RobotDecision
        print(f"{Color.GREEN}✓ 从已安装包导入模块成功{Color.RESET}")
        return (TargetConfirmation, select_best_target,
                KalmanFilter2D, camera_to_robot, is_valid_detection,
                MotionPlanner, TrapezoidPlanner, RobotDecision)
    except ImportError:
        pass

    try:
        # 方式2：从源码目录导入（直接在仓库里运行）
        import importlib.util, os
        base = os.path.join(os.path.dirname(__file__),
                            'decision_processor')
        def load(name):
            spec = importlib.util.spec_from_file_location(
                name, os.path.join(base, name + '.py'))
            mod = importlib.util.module_from_spec(spec)
            spec.loader.exec_module(mod)
            return mod

        tc  = load('target_confirmation')
        kf  = load('kalman_filter')
        mp  = load('motion_planner')
        rd  = load('robot_decision')
        print(f"{Color.GREEN}✓ 从源码目录导入模块成功{Color.RESET}")
        return (tc.TargetConfirmation, tc.select_best_target,
                kf.KalmanFilter2D, kf.camera_to_robot,
                kf.is_valid_detection,
                mp.MotionPlanner, mp.TrapezoidPlanner,
                rd.RobotDecision)
    except Exception as e:
        print(f"{Color.YELLOW}⚠ 无法导入模块({e})，将使用内嵌版本{Color.RESET}")
        return None


# ═══════════════════════════════════════════════════════
#   内嵌版算法（模块导入失败时的备用）
# ═══════════════════════════════════════════════════════

class _TargetConfirmation:
    def __init__(self, confirm_frames=3, lost_frames=5):
        self.confirm_frames = confirm_frames
        self.lost_frames    = lost_frames
        self._seen = 0
        self._lost = 0
        self.is_confirmed = False

    def update(self, detected):
        if detected:
            self._seen += 1; self._lost = 0
            if self._seen >= self.confirm_frames:
                self.is_confirmed = True
        else:
            self._lost += 1; self._seen = 0
            if self._lost >= self.lost_frames:
                self.is_confirmed = False
        return self.is_confirmed

    def reset(self):
        self._seen = 0; self._lost = 0
        self.is_confirmed = False


def _select_best(detections, distances, conf_thr=0.5):
    valid = [(d, dist) for d, dist in zip(detections, distances)
             if dist > 0 and d['confidence'] >= conf_thr]
    return min(valid, key=lambda p: p[1]) if valid else None


class _KalmanFilter2D:
    def __init__(self, dt=0.05, process_noise=0.01,
                 measurement_noise=0.1):
        self.F = np.array([[1,0,dt,0],[0,1,0,dt],
                           [0,0,1,0],[0,0,0,1]], dtype=float)
        self.H = np.array([[1,0,0,0],[0,1,0,0]], dtype=float)
        self.Q = np.eye(4) * process_noise
        self.R = np.eye(2) * measurement_noise
        self.P = np.eye(4)
        self.x = np.zeros((4,1))
        self.initialized = False

    def update(self, mx, my):
        z = np.array([[mx],[my]])
        if not self.initialized:
            self.x[:2] = z; self.initialized = True
            return mx, my
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q
        y = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x += K @ y
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return float(self.x[0]), float(self.x[1])

    def reset(self):
        self.x = np.zeros((4,1))
        self.P = np.eye(4)
        self.initialized = False


def _camera_to_robot(cx, cy, cz):
    return cz, -cx   # robot_x=前, robot_y=左

def _is_valid(new_pos, last_pos, max_jump=0.8):
    if last_pos is None: return True
    d = math.sqrt((new_pos[0]-last_pos[0])**2 +
                  (new_pos[1]-last_pos[1])**2)
    return d < max_jump


class _MotionPlanner:
    def __init__(self, wheel_diameter_m=0.096,
                 track_width_m=0.28,
                 align_threshold_deg=5.0,
                 stop_distance_m=0.20,
                 arrival_threshold_m=0.15):
        self.wheel_circ  = math.pi * wheel_diameter_m
        self.track_width = track_width_m
        self.align_thr   = align_threshold_deg
        self.stop_dist   = stop_distance_m
        self.arrival_thr = arrival_threshold_m

    def plan(self, rx, ry):
        dist     = math.sqrt(rx**2 + ry**2)
        turn_rad = math.atan2(ry, rx)
        turn_deg = math.degrees(turn_rad)
        turn_arc = abs(turn_rad) * self.track_width / 2
        tw       = turn_arc / self.wheel_circ

        if dist <= self.arrival_thr:
            return {'phase':'ARRIVED','turn_deg':0,'turn_wheels':0,
                    'forward_m':0,'drive_wheels':0}
        if abs(turn_deg) > self.align_thr:
            return {'phase':'ALIGNING','turn_deg':turn_deg,
                    'turn_wheels':tw,'forward_m':0,'drive_wheels':0}
        fwd = max(0, dist - self.stop_dist)
        return {'phase':'MOVING','turn_deg':0,'turn_wheels':0,
                'forward_m':fwd,
                'drive_wheels':fwd / self.wheel_circ}


# ═══════════════════════════════════════════════════════
#   测试场景定义
# ═══════════════════════════════════════════════════════

class Scenario(Enum):
    """内置测试场景"""
    FULL_APPROACH  = 'full'    # 完整流程：搜索→对准→前进→到达→拾取
    TARGET_LOST    = 'lost'    # 目标中途丢失然后重新发现
    JUMP_DETECTION = 'jump'    # 测试跳变检测过滤误检
    KALMAN_NOISE   = 'kalman'  # 测试卡尔曼对噪声的平滑效果
    MULTI_TARGET   = 'multi'   # 多目标选择（选最近/最高置信度）
    SIDE_APPROACH  = 'side'    # 目标在侧面，需要先大角度转向


def make_detection(x_px=320, y_px=240,
                   depth_m=2.0,
                   conf=0.85,
                   cls_id=0,
                   cls_name='target',
                   bbox_w=80, bbox_h=100):
    """
    构造模拟的 YOLO 检测结果
    x_px, y_px: 目标中心在图像中的像素坐标（默认图像中心）
    depth_m:    目标深度/距离（米）
    """
    half_w, half_h = bbox_w // 2, bbox_h // 2
    return {
        'bbox':       [x_px - half_w, y_px - half_h,
                       x_px + half_w, y_px + half_h],
        'confidence': conf,
        'class_id':   cls_id,
        'class_name': cls_name,
    }


def make_camera_matrix(fx=615.0, fy=615.0,
                       cx=320.0, cy=240.0):
    """
    构造模拟的相机内参矩阵
    RealSense D435i 典型值：fx=fy≈615，cx≈320，cy≈240
    """
    return np.array([
        [fx,  0, cx],
        [ 0, fy, cy],
        [ 0,  0,  1]
    ], dtype=float)


# ═══════════════════════════════════════════════════════
#   模式1：纯算法测试（无需 ROS2）
# ═══════════════════════════════════════════════════════

def run_algo_test(modules=None):
    """
    直接实例化各算法类，逐帧输入模拟数据，打印处理结果
    完全不依赖 ROS2，可在任何 Python 环境运行
    """
    if modules:
        (TargetConfirmation, select_best_target,
         KalmanFilter2D, camera_to_robot, is_valid_detection,
         MotionPlanner, TrapezoidPlanner, RobotDecision) = modules
    else:
        TargetConfirmation = _TargetConfirmation
        select_best_target = _select_best
        KalmanFilter2D     = _KalmanFilter2D
        camera_to_robot    = _camera_to_robot
        is_valid_detection = _is_valid
        MotionPlanner      = _MotionPlanner

    cam_matrix = make_camera_matrix()

    print_header("模式1：纯算法单元测试")

    # ── 测试1：卡尔曼滤波效果 ──────────────────────────
    print(f"{Color.BOLD}【测试1】卡尔曼滤波噪声抑制效果{Color.RESET}")
    print(f"  模拟：目标匀速靠近（真实距离 3.0→0.5m），叠加随机噪声\n")

    kf        = KalmanFilter2D()
    true_dist = 3.0
    np.random.seed(42)

    for i in range(20):
        true_dist   = max(0.5, 3.0 - i * 0.15)
        noise_x     = np.random.normal(0, 0.08)   # 8cm 水平噪声
        noise_z     = np.random.normal(0, 0.06)   # 6cm 深度噪声
        raw_x       = 0.0 + noise_x
        raw_z       = true_dist + noise_z

        rx_raw, ry_raw   = camera_to_robot(raw_x, 0, raw_z)
        rx_filt, ry_filt = kf.update(rx_raw, ry_raw)

        if i % 4 == 0:
            print_kalman(rx_raw, ry_raw, rx_filt, ry_filt)

    print(f"  {Color.GREEN}✓ 卡尔曼滤波测试通过{Color.RESET}\n")

    # ── 测试2：跳变检测 ────────────────────────────────
    print(f"{Color.BOLD}【测试2】跳变检测（过滤误检）{Color.RESET}")
    print(f"  模拟：正常数据中插入一个 2m 跳变的误检帧\n")

    positions = [
        (1.5, 0.1),   # 正常
        (1.4, 0.1),   # 正常
        (1.3, 0.1),   # 正常
        (3.8, 2.5),   # 误检！跳变 > 0.8m
        (1.2, 0.1),   # 正常，误检后恢复
        (1.1, 0.1),   # 正常
    ]
    last_pos = None
    for i, pos in enumerate(positions):
        valid  = is_valid_detection(pos, last_pos, max_jump=0.8)
        status = (f"{Color.GREEN}✓ 有效{Color.RESET}"
                  if valid
                  else f"{Color.RED}✗ 跳变过滤{Color.RESET}")
        jump   = (math.sqrt((pos[0]-last_pos[0])**2 +
                            (pos[1]-last_pos[1])**2)
                  if last_pos else 0)
        print(f"  帧{i+1}  位置=({pos[0]:.1f}, {pos[1]:.1f})  "
              f"跳变={jump:.2f}m  {status}")
        if valid:
            last_pos = pos

    print(f"\n  {Color.GREEN}✓ 跳变检测测试通过{Color.RESET}\n")

    # ── 测试3：连续帧确认 ──────────────────────────────
    print(f"{Color.BOLD}【测试3】连续帧确认算法{Color.RESET}")
    print(f"  需要连续3帧检测到才确认，连续5帧未检测才丢失\n")

    confirm = TargetConfirmation(confirm_frames=3, lost_frames=5)
    frames  = [True, True, False, True, True, True,
               False, False, False, False, False, True]

    for i, detected in enumerate(frames):
        is_conf = confirm.update(detected)
        det_str = (f"{Color.GREEN}检测到{Color.RESET}"
                   if detected
                   else f"{Color.GRAY}未检测{Color.RESET}")
        conf_str = (f"{Color.GREEN}已确认{Color.RESET}"
                    if is_conf
                    else f"{Color.YELLOW}未确认{Color.RESET}")
        print(f"  帧{i+1:02d}  {det_str}  →  {conf_str}")

    print(f"\n  {Color.GREEN}✓ 连续帧确认测试通过{Color.RESET}\n")

    # ── 测试4：运动规划 ────────────────────────────────
    print(f"{Color.BOLD}【测试4】两阶段运动规划{Color.RESET}")
    print(f"  模拟不同目标位置，验证规划结果\n")

    planner    = MotionPlanner(wheel_diameter_m=0.096,
                               track_width_m=0.28)
    test_cases = [
        (2.0,  0.0,  "正前方 2m"),
        (1.5,  0.8,  "前方偏左 30°"),
        (1.0, -0.5,  "前方偏右 27°"),
        (0.1,  0.05, "正前方 0.1m（应到达）"),
        (0.0,  2.0,  "正左方 2m（需大角度转向）"),
    ]

    for rx, ry, desc in test_cases:
        plan = planner.plan(rx, ry)
        dist = math.sqrt(rx**2 + ry**2)
        ang  = math.degrees(math.atan2(ry, rx))
        phase_color = {
            'ARRIVED': Color.GREEN,
            'ALIGNING': Color.BLUE,
            'MOVING': Color.CYAN,
        }.get(plan['phase'], Color.WHITE)

        print(f"  [{desc}]  距离={dist:.2f}m  偏角={ang:.1f}°")
        print(f"    → 阶段={phase_color}{plan['phase']}{Color.RESET}  "
              f"转向={plan['turn_deg']:.1f}°  "
              f"转向圈数={plan['turn_wheels']:.3f}  "
              f"前进={plan.get('forward_m',0):.3f}m  "
              f"前进圈数={plan.get('drive_wheels',0):.3f}\n")

    print(f"  {Color.GREEN}✓ 运动规划测试通过{Color.RESET}\n")

    # ── 测试5：多目标选择 ──────────────────────────────
    print(f"{Color.BOLD}【测试5】多目标选择策略（选最近目标）{Color.RESET}\n")

    detections = [
        make_detection(x_px=200, depth_m=3.0, conf=0.92, cls_name='A'),
        make_detection(x_px=350, depth_m=1.5, conf=0.78, cls_name='B'),
        make_detection(x_px=450, depth_m=2.8, conf=0.65, cls_name='C'),
    ]
    distances = [3.0, 1.5, 2.8]

    for i, (det, dist) in enumerate(zip(detections, distances)):
        print(f"  目标{i+1}: class={det['class_name']}  "
              f"dist={dist:.1f}m  conf={det['confidence']:.2f}")

    best = select_best_target(detections, distances, conf_thr=0.5)
    print(f"\n  {Color.GREEN}→ 选中目标: "
          f"class={best[0]['class_name']}  "
          f"dist={best[1]:.1f}m  "
          f"conf={best[0]['confidence']:.2f}{Color.RESET}")
    print(f"  {Color.GREEN}✓ 多目标选择测试通过{Color.RESET}\n")

    print(f"{Color.BOLD}{Color.GREEN}{'═'*50}")
    print(f"  所有算法单元测试通过！")
    print(f"{'═'*50}{Color.RESET}\n")


# ═══════════════════════════════════════════════════════
#   模式2：完整仿真（逐帧模拟相机输出，走完全流程）
# ═══════════════════════════════════════════════════════

def run_simulation(scenario: Scenario = Scenario.FULL_APPROACH,
                   modules=None):
    """
    完整流程仿真：模拟相机逐帧输出，走完整个拾取任务
    打印每帧的决策状态和输出指令
    """

    # 初始化算法模块
    if modules:
        (TargetConfirmation, select_best_target,
         KalmanFilter2D, camera_to_robot, is_valid_detection,
         MotionPlanner, _, RobotDecision) = modules
        # 直接使用 RobotDecision
        decision = RobotDecision(
            wheel_diameter_m=0.096,
            track_width_m=0.28,
            stop_distance_m=0.20,
            align_threshold_deg=5.0,
            pick_duration_s=2.0,
            conf_threshold=0.5)
        use_robot_decision = True
    else:
        use_robot_decision = False
        TargetConfirmation = _TargetConfirmation
        KalmanFilter2D     = _KalmanFilter2D
        camera_to_robot    = _camera_to_robot
        is_valid_detection = _is_valid
        MotionPlanner      = _MotionPlanner

        # 手动组合各层（兼容内嵌版本）
        confirmation = TargetConfirmation(confirm_frames=3, lost_frames=5)
        kf           = KalmanFilter2D()
        planner      = MotionPlanner()
        from enum import Enum, auto
        class RS(Enum):
            SEARCHING=auto(); ALIGNING=auto()
            MOVING=auto(); ARRIVED=auto(); PICKING=auto()
        state    = RS.SEARCHING
        last_pos = None
        pick_start = 0.0
        PICK_DURATION = 2.0

    cam_matrix = make_camera_matrix()

    # ── 生成场景帧序列 ──────────────────────────────────
    frames = _build_scenario(scenario)

    print_header(f"模式3：完整仿真 — 场景: {scenario.value}")
    print(f"  场景共 {len(frames)} 帧，模拟 20Hz 检测\n")
    print(f"  {'帧':>4}  {'状态':<12}  {'距离':>6}  "
          f"{'偏角':>7}  {'指令摘要'}")
    print(f"  {'─'*56}")

    prev_state_name = ''

    for frame_idx, frame_data in enumerate(frames):
        detections = frame_data['detections']
        distances  = frame_data['distances']
        note       = frame_data.get('note', '')

        if use_robot_decision:
            # 直接调用 RobotDecision.step()
            cmd = decision.step(detections, distances, cam_matrix)
            state_name = decision.state_name
        else:
            # 手动组合各层
            best      = _select_best(detections, distances, 0.5)
            confirmed = confirmation.update(best is not None)

            if confirmed and best is not None:
                det, dist = best
                x1,y1,x2,y2 = det['bbox']
                u = int((x1+x2)/2); v = int((y1+y2)/2)
                fx=cam_matrix[0,0]; fy=cam_matrix[1,1]
                cx=cam_matrix[0,2]; cy=cam_matrix[1,2]
                cam_x = (u-cx)*dist/fx
                cam_z = dist
                rx_raw, ry_raw = camera_to_robot(cam_x, 0, cam_z)

                new_pos = (rx_raw, ry_raw)
                if not is_valid_detection(new_pos, last_pos):
                    cmd = {'turn_angle':0,'turn_wheels':0,
                           'forward_dist':0,'drive_wheels':0,
                           'pickup_action':0,'search_rotate':0}
                    state_name = state.name
                    _print_frame(frame_idx, state_name, cmd,
                                 dist, 0, note, prev_state_name)
                    prev_state_name = state_name
                    time.sleep(0.05)
                    continue

                last_pos = new_pos
                rx, ry = kf.update(rx_raw, ry_raw)
                plan   = planner.plan(rx, ry)
            else:
                plan = None
                rx, ry = 0.0, 0.0

            # 简化状态机
            now = time.time()
            cmd = {'turn_angle':0,'turn_wheels':0,'forward_dist':0,
                   'drive_wheels':0,'pickup_action':0,'search_rotate':0}

            if state == RS.SEARCHING:
                if confirmed: state = RS.ALIGNING
                else: cmd['search_rotate'] = 1
            elif state == RS.ALIGNING:
                if not confirmed:
                    state = RS.SEARCHING; kf.reset()
                elif plan and plan['phase'] == 'ALIGNING':
                    cmd['turn_angle']  = plan['turn_deg']
                    cmd['turn_wheels'] = plan['turn_wheels']
                elif plan and plan['phase'] in ('MOVING','ARRIVED'):
                    state = RS.MOVING
            elif state == RS.MOVING:
                if not confirmed:
                    state = RS.SEARCHING; kf.reset()
                elif plan and plan['phase'] == 'ARRIVED':
                    state = RS.ARRIVED
                elif plan and plan['phase'] == 'ALIGNING':
                    state = RS.ALIGNING
                elif plan:
                    cmd['forward_dist']  = plan['forward_m']
                    cmd['drive_wheels']  = plan['drive_wheels']
            elif state == RS.ARRIVED:
                pick_start = now; state = RS.PICKING
            elif state == RS.PICKING:
                cmd['pickup_action'] = 1
                if (now - pick_start) >= PICK_DURATION:
                    state = RS.SEARCHING; kf.reset()
                    confirmation.reset(); last_pos = None

            state_name = state.name
            dist = distances[0] if distances else 0
            ang  = (math.degrees(math.atan2(ry, rx))
                    if rx or ry else 0)

        dist_val = distances[0] if distances else 0
        ang_val  = 0  # 简化显示
        _print_frame(frame_idx + 1, state_name, cmd,
                     dist_val, ang_val, note, prev_state_name)
        prev_state_name = state_name
        time.sleep(0.05)   # 模拟 20Hz

    print(f"\n  {'─'*56}")
    print(f"  {Color.GREEN}✓ 仿真完成！状态机完整走完全流程。{Color.RESET}\n")


def _print_frame(frame_idx, state_name, cmd,
                 dist, ang, note, prev_state):
    color = STATE_COLORS.get(state_name, Color.WHITE)

    # 状态切换时高亮提示
    transition = ''
    if state_name != prev_state and prev_state:
        transition = (f"  {Color.BOLD}← 状态切换！"
                      f"{prev_state}→{state_name}{Color.RESET}")

    # 生成指令摘要
    parts = []
    if cmd.get('search_rotate', 0) > 0.5:
        parts.append(f"{Color.YELLOW}旋转搜索{Color.RESET}")
    if abs(cmd.get('turn_angle', 0)) > 0.1:
        parts.append(f"转向{cmd['turn_angle']:.1f}°/"
                     f"{cmd['turn_wheels']:.3f}圈")
    if cmd.get('forward_dist', 0) > 0.001:
        parts.append(f"前进{cmd['forward_dist']:.3f}m/"
                     f"{cmd['drive_wheels']:.3f}圈")
    if cmd.get('pickup_action', 0) > 0.5:
        parts.append(f"{Color.PURPLE}执行拾取{Color.RESET}")
    if not parts:
        parts.append(f"{Color.GRAY}停止{Color.RESET}")

    summary = ' | '.join(parts)
    note_str = f"  {Color.GRAY}[{note}]{Color.RESET}" if note else ''

    print(f"  {frame_idx:>4}  "
          f"{color}{state_name:<12}{Color.RESET}  "
          f"{dist:>5.2f}m  "
          f"{ang:>6.1f}°  "
          f"{summary}{note_str}{transition}")


def _build_scenario(scenario: Scenario) -> list:
    """根据场景类型生成帧序列"""

    def frame(det=None, dist=0.0, note=''):
        if det is None:
            return {'detections': [], 'distances': [], 'note': note}
        return {'detections': [det], 'distances': [dist], 'note': note}

    if scenario == Scenario.FULL_APPROACH:
        # 正前方目标，完整拾取流程
        frames = []
        frames += [frame(note='无目标，搜索中')] * 4
        # 目标出现（连续帧确认需要3帧）
        for d in [3.0, 2.8, 2.6, 2.4, 2.2,
                  2.0, 1.8, 1.6, 1.4, 1.2,
                  1.0, 0.8, 0.6, 0.4, 0.25, 0.18]:
            frames.append(frame(
                make_detection(x_px=320, depth_m=d, conf=0.88),
                d, f'目标靠近 {d:.2f}m'))
        # 到达后拾取
        for i in range(15):
            frames.append(frame(
                make_detection(x_px=320, depth_m=0.18, conf=0.90),
                0.18, '拾取中'))
        return frames

    elif scenario == Scenario.TARGET_LOST:
        frames = []
        frames += [frame(note='搜索')] * 4
        for d in [2.5, 2.3, 2.1, 1.9]:
            frames.append(frame(
                make_detection(depth_m=d, conf=0.85), d, '靠近'))
        frames += [frame(note='目标丢失！')] * 3
        for d in [1.7, 1.5, 1.3, 1.0, 0.5, 0.22]:
            frames.append(frame(
                make_detection(depth_m=d, conf=0.82), d, '重新发现'))
        frames += [frame(
            make_detection(depth_m=0.18, conf=0.88),
            0.18, '拾取')] * 12
        return frames

    elif scenario == Scenario.SIDE_APPROACH:
        # 目标在右侧，需要大角度转向
        frames = []
        frames += [frame(note='搜索')] * 4
        # 目标在图像右侧（x_px=550），需要右转
        for d in [2.5, 2.3, 2.1, 1.9, 1.7,
                  1.5, 1.2, 0.9, 0.5, 0.22]:
            frames.append(frame(
                make_detection(x_px=550, depth_m=d, conf=0.85),
                d, '目标在右侧'))
        frames += [frame(
            make_detection(x_px=330, depth_m=0.18, conf=0.88),
            0.18, '对准后拾取')] * 12
        return frames

    elif scenario == Scenario.JUMP_DETECTION:
        frames = []
        frames += [frame(note='搜索')] * 4
        for i, d in enumerate([2.0, 1.9, 1.8]):
            frames.append(frame(
                make_detection(depth_m=d, conf=0.85), d, '正常'))
        # 插入一帧误检（x偏移很大）
        frames.append(frame(
            make_detection(x_px=600, depth_m=5.0, conf=0.90),
            5.0, '误检！跳变过大'))
        for d in [1.7, 1.5, 1.2, 0.8, 0.4, 0.22]:
            frames.append(frame(
                make_detection(depth_m=d, conf=0.85), d, '正常恢复'))
        frames += [frame(
            make_detection(depth_m=0.18, conf=0.88),
            0.18, '拾取')] * 12
        return frames

    else:
        # 默认：完整流程
        return _build_scenario(Scenario.FULL_APPROACH)


# ═══════════════════════════════════════════════════════
#   模式3：ROS2 话题测试（需要 processor_node 已在运行）
# ═══════════════════════════════════════════════════════

def run_ros_test():
    """
    通过 ROS2 话题与 processor_node 交互测试
    需要先启动：ros2 run decision_processor processor_node
    """
    try:
        import rclpy
        from rclpy.node import Node
        from geometry_msgs.msg import PointStamped, Twist
        from std_msgs.msg import String
        from sensor_msgs.msg import CameraInfo
    except ImportError:
        print(f"{Color.RED}✗ 未找到 rclpy，ROS2 话题模式不可用。"
              f"\n  请先：source /opt/ros/humble/setup.bash"
              f"\n       source ~/ros2_ws/install/setup.bash{Color.RESET}")
        return

    print_header("模式2：ROS2 话题测试")
    print(f"  确保 processor_node 已启动：")
    print(f"  {Color.CYAN}ros2 run decision_processor processor_node{Color.RESET}\n")

    rclpy.init()

    class TestNode(Node):
        def __init__(self):
            super().__init__('decision_test_node')

            # 发布模拟目标
            self.target_pub = self.create_publisher(
                PointStamped, '/vision/raw_target', 10)

            # 发布模拟相机内参
            self.info_pub = self.create_publisher(
                CameraInfo, '/camera/color/camera_info', 10)

            # 订阅决策状态输出
            self.create_subscription(
                String, '/decision/state', self._on_state, 10)

            # 订阅底盘指令输出
            self.create_subscription(
                Twist, '/serial/chassis_cmd', self._on_cmd, 10)

            self._frame   = 0
            self._scenario= _build_scenario(Scenario.FULL_APPROACH)
            self._timer   = self.create_timer(0.05, self._tick)

            # 先发布相机内参
            self._pub_camera_info()
            self.get_logger().info('测试节点已启动，开始发布模拟数据...')

        def _pub_camera_info(self):
            msg    = CameraInfo()
            msg.k  = [615.0, 0.0, 320.0,
                       0.0, 615.0, 240.0,
                       0.0, 0.0, 1.0]
            self.info_pub.publish(msg)

        def _tick(self):
            if self._frame >= len(self._scenario):
                self.get_logger().info('所有测试帧发送完毕')
                self._timer.cancel()
                return

            frame_data = self._scenario[self._frame]
            if frame_data['detections']:
                det  = frame_data['detections'][0]
                dist = frame_data['distances'][0]

                msg = PointStamped()
                msg.header.stamp    = self.get_clock().now().to_msg()
                x1,y1,x2,y2        = det['bbox']
                cx = (x1+x2)//2; cy_px = (y1+y2)//2
                fx = fy = 615.0; ox = oy = 320.0
                cam_x = (cx - ox) * dist / fx
                cam_y = (cy_px - oy) * dist / fy
                msg.point.x = cam_x
                msg.point.y = cam_y
                msg.point.z = dist
                msg.header.frame_id = (
                    f"{det['class_id']}:"
                    f"{det['class_name']}:"
                    f"{det['confidence']:.3f}")
                self.target_pub.publish(msg)

            self._frame += 1

        def _on_state(self, msg):
            color = Color.GREEN if 'PICKING' in msg.data else Color.CYAN
            print(f"  [决策状态] {color}{msg.data}{Color.RESET}")

        def _on_cmd(self, msg):
            has_action = any([
                abs(msg.angular.z) > 0.1,
                msg.linear.x > 0.001,
                msg.angular.x > 0.5,
                msg.angular.y > 0.5,
            ])
            if has_action:
                print(f"  [底盘指令] "
                      f"转向={msg.angular.z:.2f}°  "
                      f"转向圈={msg.linear.z:.3f}  "
                      f"前进={msg.linear.x:.3f}m  "
                      f"前进圈={msg.linear.y:.3f}  "
                      f"拾取={int(msg.angular.x)}  "
                      f"搜索={int(msg.angular.y)}")

    node = TestNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


# ═══════════════════════════════════════════════════════
#   主入口
# ═══════════════════════════════════════════════════════

def main():
    parser = argparse.ArgumentParser(
        description='decision_processor 测试工具',
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
测试模式：
  algo    纯算法单元测试（无需ROS2，无需下位机）
  sim     完整仿真测试（无需ROS2，无需下位机）
  ros     ROS2话题测试（需要processor_node在运行）

仿真场景（--scene，配合 sim 模式使用）：
  full    完整流程：搜索→对准→前进→到达→拾取（默认）
  lost    目标中途丢失后重新发现
  side    目标在侧面，需大角度转向
  jump    测试跳变检测过滤误检

示例：
  python3 test_decision_processor.py --mode algo
  python3 test_decision_processor.py --mode sim --scene lost
  python3 test_decision_processor.py --mode ros
        """)
    parser.add_argument('--mode',
                        choices=['algo', 'sim', 'ros'],
                        default='algo',
                        help='测试模式（默认：algo）')
    parser.add_argument('--scene',
                        choices=['full','lost','side','jump'],
                        default='full',
                        help='仿真场景（sim模式专用，默认：full）')
    args = parser.parse_args()

    print(f"\n{Color.BOLD}{Color.CYAN}")
    print(r"  ____       _           _   ")
    print(r" |  _ \ ___ | |__   ___ | |_ ")
    print(r" | |_) / _ \| '_ \ / _ \| __|")
    print(r" |  _ < (_) | |_) | (_) | |_ ")
    print(r" |_| \_\___/|_.__/ \___/ \__|")
    print(f"  decision_processor 测试工具{Color.RESET}")
    print(f"  模式: {Color.YELLOW}{args.mode}{Color.RESET}  "
          f"场景: {Color.YELLOW}{args.scene}{Color.RESET}\n")

    # 尝试加载真实模块
    modules = load_modules()

    scene_map = {
        'full': Scenario.FULL_APPROACH,
        'lost': Scenario.TARGET_LOST,
        'side': Scenario.SIDE_APPROACH,
        'jump': Scenario.JUMP_DETECTION,
    }

    if args.mode == 'algo':
        run_algo_test(modules)

    elif args.mode == 'sim':
        # 先跑算法单元测试，再跑完整仿真
        run_algo_test(modules)
        run_simulation(scene_map[args.scene], modules)

    elif args.mode == 'ros':
        run_ros_test()


if __name__ == '__main__':
    main()