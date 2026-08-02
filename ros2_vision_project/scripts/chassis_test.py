#!/usr/bin/env python3
"""
chassis_test.py — 底盘运动与PID测试工具
50Hz 连续发送指令，路段之间无停顿，PID模式自动记录响应曲线，
闭环导航模式读取 EKF/FAST-LIO 位姿反馈，自动到达目标并记录误差。

用法:
    source ros2_ws/install/setup.bash
    python3 scripts/chassis_test.py [模式]

开环路线模式（时间换距离）:
    straight      直线前进 1m
    back_forth    来回 1m往返
    l_shape       L形路线
    square        正方形闭环
    custom        自定义路线（编辑脚本底部 ROUTES['custom']）

PID 阶跃响应模式（记录 wheel_odom 反馈到 CSV）:
    pid_vx        前进方向阶跃响应
    pid_vy        横移方向阶跃响应
    pid_az        旋转方向阶跃响应

闭环导航模式（读 EKF 或 FAST-LIO 位姿，P控制到达目标，量化误差）:
    nav_1m_ekf         EKF 闭环：前进 1m
    nav_1m_fastlio     FAST-LIO 闭环：前进 1m
    nav_back_ekf       EKF 闭环：前进 1m 后返回原点
    nav_back_fastlio   FAST-LIO 闭环：前进 1m 后返回原点
    nav_square_ekf     EKF 闭环：1m×1m 正方形闭环
    nav_square_fastlio FAST-LIO 闭环：1m×1m 正方形闭环
    nav_1m_compare     对比模式：EKF(+FAST-LIO融合)控制，同步记录 /odom 和 /Odometry
    nav_back_compare   对比模式：前进1m后返回
    nav_square_compare 对比模式：1m×1m 正方形
"""

import csv
import math
import os
import sys
import time
import threading

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry

RATE_HZ    = 50
SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))

# ── 闭环导航 P 控制参数 ──────────────────────────────────────────────
NAV_KP_LINEAR    = 0.8    # 位置误差增益 (误差1m → 指令0.8m/s)
NAV_MAX_VX       = 0.25   # 最大前进速度 m/s
NAV_MAX_VY       = 0.20   # 最大横移速度 m/s
NAV_ARRIVE_M     = 0.05   # 到达判定距离 m（5cm）
NAV_TIMEOUT_S    = 30.0   # 单段最长等待时间 s

# ══════════════════════════════════════════════════════
#   开环路线（每段: vx/vy/az/t/label）
# ══════════════════════════════════════════════════════

ROUTES = {

    'straight': [
        {'label': '前进1m', 'vx': 0.2, 't': 5.0},
    ],

    'back_forth': [
        {'label': '前进1m', 'vx':  0.2, 't': 5.0},
        {'label': '后退1m', 'vx': -0.2, 't': 5.0},
    ],

    'l_shape': [
        {'label': '前进1m',      'vx': 0.2,  't': 5.0},
        {'label': '转90°逆时针', 'az': 30.0, 't': 3.0},
        {'label': '前进1m',      'vx': 0.2,  't': 5.0},
    ],

    'square': [
        {'label': '第1边前进1m', 'vx': 0.2,  't': 5.0},
        {'label': '转90°',       'az': 30.0, 't': 3.0},
        {'label': '第2边前进1m', 'vx': 0.2,  't': 5.0},
        {'label': '转90°',       'az': 30.0, 't': 3.0},
        {'label': '第3边前进1m', 'vx': 0.2,  't': 5.0},
        {'label': '转90°',       'az': 30.0, 't': 3.0},
        {'label': '第4边前进1m', 'vx': 0.2,  't': 5.0},
        {'label': '转90°',       'az': 30.0, 't': 3.0},
    ],

    'custom': [
        {'label': '前进0.5m', 'vx': 0.2, 't': 2.5},
        {'label': '左移0.3m', 'vy': 0.2, 't': 1.5},
        {'label': '前进0.5m', 'vx': 0.2, 't': 2.5},
    ],
}

# ══════════════════════════════════════════════════════
#   PID 阶跃测试参数
# ══════════════════════════════════════════════════════

PID_TESTS = {
    'pid_vx': {
        'label':    '前进方向(vx)阶跃响应',
        'field':    'vx',
        'target':    0.3,
        'hold_t':    3.0,
        'settle_t':  2.0,
        'fb_field': 'linear.x',
    },
    'pid_vy': {
        'label':    '横移方向(vy)阶跃响应',
        'field':    'vy',
        'target':    0.2,
        'hold_t':    3.0,
        'settle_t':  2.0,
        'fb_field': 'linear.y',
    },
    'pid_az': {
        'label':    '旋转方向(az)阶跃响应',
        'field':    'az',
        'target':   30.0,
        'hold_t':    3.0,
        'settle_t':  2.0,
        'fb_field': 'angular.z',
    },
}

# ══════════════════════════════════════════════════════
#   闭环导航航点（相对起点的世界坐标 m）
# ══════════════════════════════════════════════════════

NAV_ROUTES = {
    'nav_1m': [
        {'x': 1.0, 'y': 0.0, 'label': '目标: 前进1m'},
    ],
    'nav_back': [
        {'x': 1.0, 'y': 0.0, 'label': '目标: 前进1m'},
        {'x': 0.0, 'y': 0.0, 'label': '目标: 返回原点'},
    ],
    'nav_square': [
        {'x': 3.0, 'y': 0.0, 'label': '目标: 边1终点'},
        {'x': 3.0, 'y': 3.0, 'label': '目标: 边2终点'},
        {'x': 0.0, 'y': 3.0, 'label': '目标: 边3终点'},
        {'x': 0.0, 'y': 0.0, 'label': '目标: 回起点'},
    ],
}


def _quat_to_yaw(q) -> float:
    """四元数转航向角 (rad)。"""
    return math.atan2(
        2.0 * (q.w * q.z + q.x * q.y),
        1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    )


class ChassisTest(Node):

    def __init__(self):
        super().__init__('chassis_test')
        self._pub = self.create_publisher(Twist, '/serial/chassis_cmd', 10)

        self._latest_odom     = None   # wheel odom (Twist)
        self._latest_ekf      = None   # EKF (nav_msgs/Odometry)
        self._latest_fastlio  = None   # FAST-LIO (nav_msgs/Odometry)

        self.create_subscription(Twist,    '/feedback/wheel_odom', self._on_wheel_odom,  10)
        self.create_subscription(Odometry, '/odom',                self._on_ekf,         10)
        self.create_subscription(Odometry, '/Odometry',            self._on_fastlio,      10)

    def _on_wheel_odom(self, msg): self._latest_odom    = msg
    def _on_ekf(self,       msg): self._latest_ekf     = msg
    def _on_fastlio(self,   msg): self._latest_fastlio = msg

    # ──────────────────────────────────────────────────
    #   开环路线
    # ──────────────────────────────────────────────────

    def run_route(self, route: list):
        dt    = 1.0 / RATE_HZ
        total = sum(seg['t'] for seg in route)
        print(f'\n[开环] {len(route)} 段路线，总时长 {total:.1f}s，{RATE_HZ}Hz')

        for i, seg in enumerate(route):
            vx    = seg.get('vx', 0.0)
            vy    = seg.get('vy', 0.0)
            az    = seg.get('az', 0.0)
            t     = seg.get('t',  1.0)
            label = seg.get('label', f'段{i+1}')

            print(f'  [{i+1}/{len(route)}] {label}  '
                  f'vx={vx:.2f} vy={vy:.2f} az={az:.1f}  {t:.1f}s')

            msg = Twist()
            msg.linear.x  = float(vx)
            msg.linear.y  = float(vy)
            msg.angular.z = float(az)

            for _ in range(int(t * RATE_HZ)):
                self._pub.publish(msg)
                time.sleep(dt)

        self._stop()
        print('[开环] 路线完成')

    # ──────────────────────────────────────────────────
    #   PID 阶跃响应
    # ──────────────────────────────────────────────────

    def run_pid_test(self, cfg: dict):
        dt       = 1.0 / RATE_HZ
        field    = cfg['field']
        target   = cfg['target']
        hold_t   = cfg['hold_t']
        settle_t = cfg['settle_t']
        fb_field = cfg['fb_field']
        label    = cfg['label']

        csv_path = os.path.join(SCRIPT_DIR, f'pid_{field}_result.csv')
        records  = []

        print(f'\n[PID测试] {label}')
        print(f'  目标值: {target}  保持: {hold_t}s  跌落后观测: {settle_t}s')
        print('  等待5秒让轮式里程计就绪...')
        time.sleep(5.0)

        def _get_fb():
            if self._latest_odom is None:
                return None
            parts = fb_field.split('.')
            obj = self._latest_odom
            for p in parts:
                obj = getattr(obj, p, None)
                if obj is None:
                    return None
            return float(obj)

        t_start = time.time()

        print('  [1/3] 静止基准...')
        for _ in range(int(0.5 * RATE_HZ)):
            self._pub.publish(Twist())
            records.append({'t': time.time() - t_start, 'cmd': 0.0, 'fb': _get_fb()})
            time.sleep(dt)

        print(f'  [2/3] 阶跃至 {target}...')
        cmd_msg = Twist()
        if   field == 'vx': cmd_msg.linear.x  = float(target)
        elif field == 'vy': cmd_msg.linear.y  = float(target)
        elif field == 'az': cmd_msg.angular.z = float(target)

        for _ in range(int(hold_t * RATE_HZ)):
            self._pub.publish(cmd_msg)
            records.append({'t': time.time() - t_start, 'cmd': target, 'fb': _get_fb()})
            time.sleep(dt)

        print(f'  [3/3] 归零，观测 {settle_t}s...')
        for _ in range(int(settle_t * RATE_HZ)):
            self._pub.publish(Twist())
            records.append({'t': time.time() - t_start, 'cmd': 0.0, 'fb': _get_fb()})
            time.sleep(dt)

        self._stop()

        with open(csv_path, 'w', newline='') as f:
            csv.DictWriter(f, fieldnames=['t', 'cmd', 'fb']).writeheader()
            csv.DictWriter(f, fieldnames=['t', 'cmd', 'fb']).writerows(records)

        fb_vals = [r['fb'] for r in records if r['fb'] is not None and r['cmd'] > 0]
        if fb_vals:
            peak = max(abs(v) for v in fb_vals)
            overshoot = (peak - abs(target)) / abs(target) * 100 if target != 0 else 0
            print(f'\n  峰值反馈: {peak:.4f}')
            print(f'  超调量:   {overshoot:.1f}%')
        else:
            print('\n  未收到 /feedback/wheel_odom，检查串口是否在线')

        print(f'  CSV: {csv_path}')

    # ──────────────────────────────────────────────────
    #   闭环导航
    # ──────────────────────────────────────────────────

    def run_nav(self, route_name: str, source: str):
        """
        闭环导航：P 控制跟踪航点序列，记录每段起点/终点误差。

        source: 'ekf'     → 读 /odom (nav_msgs/Odometry)
                'fastlio' → 读 /Odometry (nav_msgs/Odometry)
        """
        topic  = '/odom' if source == 'ekf' else '/Odometry'
        route  = NAV_ROUTES[route_name]
        dt     = 1.0 / RATE_HZ

        print(f'\n[闭环导航] 来源={source}({topic})  路线={route_name}  {len(route)} 段')
        print(f'  P增益={NAV_KP_LINEAR}  最大速度 vx={NAV_MAX_VX} vy={NAV_MAX_VY}  到达阈值={NAV_ARRIVE_M}m')
        print('  等待定位话题就绪（最多10s）...')

        deadline = time.time() + 10.0
        while self._get_odom(source) is None:
            if time.time() > deadline:
                print(f'  [错误] {topic} 无数据，请确认定位节点已启动')
                return
            time.sleep(0.1)

        # 记录全局起点（用于坐标偏移）
        x0, y0, _ = self._get_odom(source)
        print(f'  全局起点: ({x0:.3f}, {y0:.3f})')

        results = []

        for i, wp in enumerate(route):
            # 目标在世界坐标系 = 起点 + 航点偏移
            gx = x0 + wp['x']
            gy = y0 + wp['y']
            label = wp['label']

            cx, cy, _ = self._get_odom(source)
            print(f'\n  [{i+1}/{len(route)}] {label}')
            print(f'    当前: ({cx:.3f}, {cy:.3f})  目标: ({gx:.3f}, {gy:.3f})')

            seg_start_x, seg_start_y = cx, cy
            t_seg = time.time()
            arrived = False

            while time.time() - t_seg < NAV_TIMEOUT_S:
                pose = self._get_odom(source)
                if pose is None:
                    time.sleep(dt)
                    continue

                cx, cy, cyaw = pose
                dx = gx - cx
                dy = gy - cy
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < NAV_ARRIVE_M:
                    arrived = True
                    break

                # 世界坐标误差 → 机器人坐标系速度
                speed = min(NAV_KP_LINEAR * dist, 1.0)  # 归一化
                # 世界系方向向量
                dx_n = dx / dist
                dy_n = dy / dist
                # 旋转到机器人坐标系
                cos_y = math.cos(cyaw)
                sin_y = math.sin(cyaw)
                vx_r =  dx_n * cos_y + dy_n * sin_y
                vy_r = -dx_n * sin_y + dy_n * cos_y

                msg = Twist()
                msg.linear.x = float(max(-NAV_MAX_VX, min(NAV_MAX_VX, vx_r * NAV_MAX_VX / max(abs(vx_r), 1e-6) * speed)))
                msg.linear.y = float(max(-NAV_MAX_VY, min(NAV_MAX_VY, vy_r * NAV_MAX_VY / max(abs(vy_r), 1e-6) * speed)))
                # 简单限速：保持速度比例
                msg.linear.x = float(max(-NAV_MAX_VX, min(NAV_MAX_VX, vx_r * NAV_KP_LINEAR * dist)))
                msg.linear.y = float(max(-NAV_MAX_VY, min(NAV_MAX_VY, vy_r * NAV_KP_LINEAR * dist)))
                self._pub.publish(msg)
                time.sleep(dt)

            self._stop()
            time.sleep(0.3)  # 停稳再采样终点

            ex, ey, _ = self._get_odom(source)
            pos_err = math.sqrt((ex - gx) ** 2 + (ey - gy) ** 2)
            elapsed = time.time() - t_seg

            status = '到达' if arrived else '超时'
            print(f'    {status}  用时: {elapsed:.1f}s')
            print(f'    终点实测: ({ex:.3f}, {ey:.3f})')
            print(f'    位置误差: {pos_err*100:.1f} cm')

            results.append({
                'segment':  label,
                'target_x': gx, 'target_y': gy,
                'end_x':    ex, 'end_y':    ey,
                'error_m':  pos_err,
                'elapsed_s': elapsed,
                'status':   status,
            })

        # 打印汇总表
        print(f'\n{"─"*55}')
        print(f'  {"段":20s} {"误差(cm)":>10s}  {"状态":>6s}')
        print(f'{"─"*55}')
        for r in results:
            print(f'  {r["segment"]:20s} {r["error_m"]*100:>10.1f}  {r["status"]:>6s}')
        avg = sum(r['error_m'] for r in results) / len(results)
        print(f'{"─"*55}')
        print(f'  平均位置误差: {avg*100:.1f} cm')

        # 保存 CSV
        csv_path = os.path.join(SCRIPT_DIR, f'nav_{route_name}_{source}_result.csv')
        with open(csv_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=list(results[0].keys()))
            writer.writeheader()
            writer.writerows(results)
        print(f'  结果已保存: {csv_path}')

    # ──────────────────────────────────────────────────
    #   对比模式：EKF 控制，同步记录 /odom 与 /Odometry
    # ──────────────────────────────────────────────────

    def run_nav_compare(self, route_name: str):
        """
        使用 EKF(/odom) 做闭环控制，同时记录 /odom 和 /Odometry 的轨迹，
        方便对比 EKF融合 vs 纯FAST-LIO 的定位精度。
        需要 EKF 和 FAST-LIO 同时在线。
        """
        route = NAV_ROUTES[route_name]
        dt    = 1.0 / RATE_HZ

        print(f'\n[对比模式] 路线={route_name}  控制源=/odom(EKF+FAST-LIO融合)')
        print('  需要同时启动: FAST-LIO + EKF + 串口桥')
        print('  等待 /odom 和 /Odometry 就绪（最多15s）...')

        deadline = time.time() + 15.0
        while True:
            ekf_ok = self._get_odom('ekf') is not None
            fl_ok  = self._get_odom('fastlio') is not None
            if ekf_ok and fl_ok:
                break
            if time.time() > deadline:
                missing = []
                if not ekf_ok: missing.append('/odom (EKF)')
                if not fl_ok:  missing.append('/Odometry (FAST-LIO)')
                print(f'  [错误] 以下话题无数据: {", ".join(missing)}')
                print('  仅 EKF 在线可用 nav_xxx_ekf；仅 FAST-LIO 可用 nav_xxx_fastlio')
                return
            time.sleep(0.2)

        x0_e, y0_e, _ = self._get_odom('ekf')
        x0_f, y0_f, _ = self._get_odom('fastlio')
        print(f'  EKF 起点:     ({x0_e:.3f}, {y0_e:.3f})')
        print(f'  FAST-LIO 起点: ({x0_f:.3f}, {y0_f:.3f})')

        # 轨迹记录（每帧）
        traj = []
        results = []

        for i, wp in enumerate(route):
            gx_e = x0_e + wp['x']
            gy_e = y0_e + wp['y']
            label = wp['label']
            print(f'\n  [{i+1}/{len(route)}] {label}  目标(EKF坐标系): ({gx_e:.3f}, {gy_e:.3f})')

            t_seg = time.time()
            arrived = False

            while time.time() - t_seg < NAV_TIMEOUT_S:
                ekf_pose = self._get_odom('ekf')
                fl_pose  = self._get_odom('fastlio')
                if ekf_pose is None:
                    time.sleep(dt)
                    continue

                cx, cy, cyaw = ekf_pose

                # 记录双轨迹
                traj.append({
                    't':     round(time.time() - t_seg, 3),
                    'seg':   i + 1,
                    'ekf_x': cx,
                    'ekf_y': cy,
                    'fl_x':  fl_pose[0] if fl_pose else None,
                    'fl_y':  fl_pose[1] if fl_pose else None,
                })

                dx = gx_e - cx
                dy = gy_e - cy
                dist = math.sqrt(dx * dx + dy * dy)

                if dist < NAV_ARRIVE_M:
                    arrived = True
                    break

                cos_y = math.cos(cyaw)
                sin_y = math.sin(cyaw)
                dx_n  = dx / dist
                dy_n  = dy / dist
                vx_r  =  dx_n * cos_y + dy_n * sin_y
                vy_r  = -dx_n * sin_y + dy_n * cos_y

                msg = Twist()
                msg.linear.x = float(max(-NAV_MAX_VX, min(NAV_MAX_VX, vx_r * NAV_KP_LINEAR * dist)))
                msg.linear.y = float(max(-NAV_MAX_VY, min(NAV_MAX_VY, vy_r * NAV_KP_LINEAR * dist)))
                self._pub.publish(msg)
                time.sleep(dt)

            self._stop()
            time.sleep(0.3)

            ekf_end = self._get_odom('ekf')
            fl_end  = self._get_odom('fastlio')

            ex_e, ey_e = ekf_end[0], ekf_end[1]
            err_ekf  = math.sqrt((ex_e - gx_e)**2 + (ey_e - gy_e)**2)

            if fl_end:
                # FAST-LIO 误差相对其自身起点偏移
                gx_f = x0_f + wp['x']
                gy_f = y0_f + wp['y']
                err_fl = math.sqrt((fl_end[0] - gx_f)**2 + (fl_end[1] - gy_f)**2)
            else:
                err_fl = None

            elapsed = time.time() - t_seg
            status  = '到达' if arrived else '超时'
            print(f'    {status}  用时: {elapsed:.1f}s')
            print(f'    EKF误差:      {err_ekf*100:.1f} cm')
            if err_fl is not None:
                print(f'    FAST-LIO误差: {err_fl*100:.1f} cm')
            else:
                print(f'    FAST-LIO误差: 无数据')

            results.append({
                'segment':    label,
                'ekf_err_cm': round(err_ekf * 100, 2),
                'fl_err_cm':  round(err_fl * 100, 2) if err_fl is not None else 'N/A',
                'elapsed_s':  round(elapsed, 1),
                'status':     status,
            })

        # 汇总
        print(f'\n{"─"*60}')
        print(f'  {"段":20s} {"EKF误差(cm)":>12s}  {"FAST-LIO误差(cm)":>16s}')
        print(f'{"─"*60}')
        for r in results:
            print(f'  {r["segment"]:20s} {str(r["ekf_err_cm"]):>12s}  {str(r["fl_err_cm"]):>16s}')
        ekf_errs = [r['ekf_err_cm'] for r in results if isinstance(r['ekf_err_cm'], float)]
        if ekf_errs:
            print(f'{"─"*60}')
            print(f'  EKF 平均误差: {sum(ekf_errs)/len(ekf_errs):.1f} cm')

        # 保存轨迹 CSV（可用于绘图）
        traj_path = os.path.join(SCRIPT_DIR, f'nav_{route_name}_compare_traj.csv')
        with open(traj_path, 'w', newline='', encoding='utf-8') as f:
            writer = csv.DictWriter(f, fieldnames=['t','seg','ekf_x','ekf_y','fl_x','fl_y'])
            writer.writeheader()
            writer.writerows(traj)
        print(f'  轨迹 CSV: {traj_path}')
        print()
        print('  绘图命令:')
        print(f'    python3 -c "import pandas as pd, matplotlib.pyplot as plt; '
              f'df=pd.read_csv(\'{traj_path}\'); '
              f'df.plot(x=\'ekf_x\',y=\'ekf_y\',label=\'EKF\'); '
              f'df.plot(x=\'fl_x\',y=\'fl_y\',label=\'FAST-LIO\'); '
              f'plt.axis(\'equal\'); plt.grid(True); plt.show()"')

    def _get_odom(self, source: str):
        """返回 (x, y, yaw_rad) 或 None。"""
        msg = self._latest_ekf if source == 'ekf' else self._latest_fastlio
        if msg is None:
            return None
        p = msg.pose.pose.position
        q = msg.pose.pose.orientation
        return p.x, p.y, _quat_to_yaw(q)

    def _stop(self):
        self._pub.publish(Twist())


# ══════════════════════════════════════════════════════
#   入口
# ══════════════════════════════════════════════════════

def _parse_nav_mode(mode: str):
    """
    解析闭环模式字符串，例如 'nav_square_ekf' → ('nav_square', 'ekf')
    结尾必须是 _ekf 或 _fastlio。
    """
    for src in ('ekf', 'fastlio', 'compare'):
        suffix = f'_{src}'
        if mode.endswith(suffix):
            route = mode[:-len(suffix)]
            if route in NAV_ROUTES:
                return route, src
    return None, None


def main():
    mode = sys.argv[1] if len(sys.argv) > 1 else 'straight'

    rclpy.init()
    node = ChassisTest()

    spin_thread = threading.Thread(target=rclpy.spin, args=(node,), daemon=True)
    spin_thread.start()

    try:
        route_key, src = _parse_nav_mode(mode)
        if route_key is not None:
            if src == 'compare':
                node.run_nav_compare(route_key)
            else:
                node.run_nav(route_key, src)
        elif mode in PID_TESTS:
            node.run_pid_test(PID_TESTS[mode])
        elif mode in ROUTES:
            node.run_route(ROUTES[mode])
        else:
            print(f'未知模式: {mode}\n')
            print('开环路线:  ' + ', '.join(ROUTES.keys()))
            print('PID测试:   ' + ', '.join(PID_TESTS.keys()))
            print('闭环导航:  ' + ', '.join(
                f'nav_{r}_{s}' for r in NAV_ROUTES for s in ('ekf', 'fastlio', 'compare')))
            sys.exit(1)

    except KeyboardInterrupt:
        node._stop()
        print('\n已中断，发送停止指令')
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
