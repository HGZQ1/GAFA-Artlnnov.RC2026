"""
fine_align_node.py
机械臂末端USB相机精对齐节点

D435I完成粗对齐(五状态机ARRIVED)后, 机械臂抬升+前伸, 末端USB相机朝下,
本节点基于"三棱检测"算法(滤色->边缘检测->棱数判定->透视/居中两级减速),
持续输出底盘横向微调指令, 直到吸盘正对KFS方块中心。

核心图像算法与 triple_edge_align.py 保持一致(滤色/边缘/棱数判定/减速曲线
均未改动), 仅将"调参GUI + 显示"包装为ROS2节点：

  /fine_align/enable (std_msgs/UInt8, sub)
      0=关闭(释放USB相机, 停止微调)
      1=启用-蓝色KFS (对应原脚本 Mode=0, LAB-B通道)
      2=启用-红色KFS (对应原脚本 Mode=1, LAB-A通道)

  /fine_align/cmd (geometry_msgs/Twist, pub)
      linear.y = 底盘横向微调速度(m/s)。
      正值="MOVE LEFT"(对应原脚本status文字), 负值="MOVE RIGHT"。
      若现场测试方向反了, 只需翻转 FINE_ALIGN_MAX_SPEED_MPS 的符号或本文件中
      direction 的取值, 不影响图像算法部分。
      linear.x 始终为0 (算法仅解出左右居中误差, 无前后修正量)。

  /fine_align/status (std_msgs/UInt8, pub)
      0=对齐中(ALIGNING)  1=已连续N帧居中, 可拾取(DONE)  2=未检测到KFS色块(NO_TARGET)
"""
import os
import cv2
import numpy as np
from collections import deque

import rclpy
from rclpy.node import Node
from std_msgs.msg import UInt8
from geometry_msgs.msg import Twist

from .config import (
    KFS_COLOR_BLUE, KFS_COLOR_RED,
    FINE_ALIGN_DISABLE, FINE_ALIGN_ENABLE_BLUE, FINE_ALIGN_ENABLE_RED,
    FINE_ALIGN_STATUS_ALIGNING, FINE_ALIGN_STATUS_DONE, FINE_ALIGN_STATUS_NO_TARGET,
    FINE_ALIGN_MAX_SPEED_MPS, FINE_ALIGN_CONFIRM_FRAMES,
)

# ═══════════════════════════════════════════════════════
#   核心控制参数 (线性减速区间) -- 与 triple_edge_align.py 完全一致
# ═══════════════════════════════════════════════════════
DEADZONE_ERR      = 15
DEADZONE_DX       = 20
START_DECEL_ERR   = 120
START_DECEL_DX    = 200
MIN_SPEED_LIMIT   = 15
MAX_SPEED_LIMIT   = 100

HISTORY_SIZE = 10
TARGET_X     = 320

# 图像处理参数默认值 (= triple_edge_align.py trackbar默认值)
DEFAULT_COLOR_THRESH = 115
DEFAULT_EDGE_SENS    = 35
DEFAULT_CLAHE_CLIP   = 4


class FineAlignNode(Node):

    def __init__(self):
        super().__init__('fine_align_node')

        self.declare_parameter('cam_index', 2)
        self.declare_parameter('debug_gui', False)

        self._cam_index = self.get_parameter('cam_index').value
        self._debug_gui = bool(self.get_parameter('debug_gui').value)

        self._cap     = None
        self._enabled = False
        self._mode    = KFS_COLOR_BLUE   # 0=蓝(LAB-B通道) 1=红(LAB-A通道), 对应原脚本 'Mode: B/R'

        self._error_history  = deque(maxlen=HISTORY_SIZE)
        self._dx_history     = deque(maxlen=HISTORY_SIZE)
        self._mask_history   = deque(maxlen=3)
        self._centered_count  = 0
        self._no_target_count = 0

        self._clahe = cv2.createCLAHE(clipLimit=DEFAULT_CLAHE_CLIP, tileGridSize=(8, 8))

        self._cmd_pub    = self.create_publisher(Twist, '/fine_align/cmd', 10)
        self._status_pub = self.create_publisher(UInt8, '/fine_align/status', 10)
        self.create_subscription(UInt8, '/fine_align/enable', self._on_enable, 10)

        if self._debug_gui:
            self._init_debug_gui()

        self.create_timer(1.0 / 30.0, self._timer_cb)
        self.get_logger().info('精对齐节点已启动, 等待 /fine_align/enable')

    # ════════════════════════════════════════
    #   相机开关 (由 game_controller 在 M_FINE_ALIGN 步骤控制)
    # ════════════════════════════════════════

    def _on_enable(self, msg: UInt8):
        if msg.data == FINE_ALIGN_DISABLE:
            self._close_camera()
            return

        self._mode = KFS_COLOR_BLUE if msg.data == FINE_ALIGN_ENABLE_BLUE else KFS_COLOR_RED
        if not self._enabled:
            self._open_camera()
        self._reset_histories()
        mode_name = '蓝(LAB-B)' if self._mode == KFS_COLOR_BLUE else '红(LAB-A)'
        self.get_logger().info(f'精对齐启用: mode={mode_name}')

    def _open_camera(self):
        idx = self._cam_index

        # --- 1. 自动相机硬件参数配置 (与 triple_edge_align.py 一致) ---
        init_cmd = (
            f"v4l2-ctl -d /dev/video{idx} -c "
            f"auto_exposure=1,"
            f"exposure_time_absolute=100,"
            f"white_balance_automatic=0,"
            f"white_balance_temperature=5500,"
            f"hue=0,"
            f"gain=100,"
            f"saturation=70"
        )
        os.system(init_cmd)

        # --- 2. 初始化摄像头 ---
        self._cap = cv2.VideoCapture(idx)
        self._cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
        self._cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
        self._cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
        self._cap.set(cv2.CAP_PROP_FPS, 120)
        self._enabled = True
        self.get_logger().info(f'USB相机已打开: /dev/video{idx}')

    def _close_camera(self):
        if self._cap is not None:
            self._cap.release()
            self._cap = None
        self._enabled = False
        self._reset_histories()
        self._cmd_pub.publish(Twist())   # 停止底盘微调
        if self._debug_gui:
            cv2.destroyAllWindows()
        self.get_logger().info('USB相机已关闭')

    def _reset_histories(self):
        self._error_history.clear()
        self._dx_history.clear()
        self._mask_history.clear()
        self._centered_count  = 0
        self._no_target_count = 0

    # ════════════════════════════════════════
    #   调试GUI (debug_gui:=true 时启用trackbar调参与画面显示)
    # ════════════════════════════════════════

    def _init_debug_gui(self):
        def nothing(x): pass
        cv2.namedWindow('Config')
        cv2.createTrackbar('Color_Thresh', 'Config', DEFAULT_COLOR_THRESH, 255, nothing)
        cv2.createTrackbar('Edge_Sens', 'Config', DEFAULT_EDGE_SENS, 100, nothing)
        cv2.createTrackbar('CLAHE', 'Config', DEFAULT_CLAHE_CLIP, 10, nothing)

    def _get_tuning_params(self):
        if self._debug_gui:
            return (cv2.getTrackbarPos('Color_Thresh', 'Config'),
                    cv2.getTrackbarPos('Edge_Sens', 'Config'),
                    cv2.getTrackbarPos('CLAHE', 'Config'))
        return DEFAULT_COLOR_THRESH, DEFAULT_EDGE_SENS, DEFAULT_CLAHE_CLIP

    # ════════════════════════════════════════
    #   主循环 (30Hz, 仅 enabled 时处理)
    # ════════════════════════════════════════

    def _timer_cb(self):
        if not self._enabled or self._cap is None:
            return

        ret, frame = self._cap.read()
        if not ret:
            return

        # --- 4. 图像预处理 ---
        roi = frame[50:450, 50:590].copy()
        lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
        l, a, b_chan = cv2.split(lab)

        c_thresh, e_sens, c_clip = self._get_tuning_params()

        if c_clip > 0:
            self._clahe.setClipLimit(c_clip)
            l_enhanced = self._clahe.apply(l)
        else:
            l_enhanced = l

        if self._mode == KFS_COLOR_BLUE:
            color_mask = cv2.inRange(b_chan, 0, c_thresh)
        else:
            color_mask = cv2.inRange(a, c_thresh, 255)

        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, np.ones((7, 7), np.uint8))
        color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, np.ones((5, 5), np.uint8))

        self._mask_history.append(color_mask)
        if len(self._mask_history) == 3:
            smoothed_mask = cv2.bitwise_and(self._mask_history[0], self._mask_history[1])
            color_mask = cv2.bitwise_and(smoothed_mask, self._mask_history[2])

        final_edges = np.zeros_like(color_mask)

        # --- 5. 空间约束：只在最大色块范围内找线 ---
        contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)

        vertical_x = []
        cube_rect = None
        x_min, x_max = 0, 0
        y_b, h_b = 0, 0

        if contours:
            max_c = max(contours, key=cv2.contourArea)
            if cv2.contourArea(max_c) > 3000:
                x_b, y_b, w_b, h_b = cv2.boundingRect(max_c)

                margin_x = int(w_b * 0.15)
                margin_y = int(h_b * 0.05)

                x_min = max(0, x_b - margin_x)
                x_max = min(roi.shape[1], x_b + w_b + margin_x)
                y_min = max(0, y_b - margin_y)
                y_max = min(roi.shape[0], y_b + h_b + margin_y)

                cube_rect = (x_min, y_min, x_max - x_min, y_max - y_min)

                l_edges = cv2.Canny(l_enhanced, e_sens, e_sens * 3)

                mask_roi = np.zeros_like(color_mask)
                cv2.rectangle(mask_roi, (x_min, y_min), (x_max, y_max), 255, -1)
                final_edges = cv2.bitwise_and(l_edges, mask_roi)

                lines = cv2.HoughLinesP(final_edges, 1, np.pi / 180, 40,
                                         minLineLength=h_b * 0.4, maxLineGap=40)

                if lines is not None:
                    for line in lines:
                        x1, y1, x2, y2 = line[0]
                        angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)

                        if 86 < angle < 94:
                            if x_min <= (x1 + x2) / 2 <= x_max:
                                vertical_x.append((x1 + x2) // 2)

        # --- 6. 核心：自适应确定"面数"与"棱数" ---
        detected_state = None
        xl, xm, xr = None, None, None

        if len(vertical_x) > 0 and cube_rect is not None:
            vertical_x.sort()
            raw_edges = [vertical_x[0]]
            for i in range(1, len(vertical_x)):
                if vertical_x[i] - raw_edges[-1] > 25:
                    raw_edges.append(vertical_x[i])

            total_w = x_max - x_min
            left_candidates = []
            mid_candidates = []
            right_candidates = []

            for x_val in raw_edges:
                relative_x = x_val - x_min
                if 0 <= relative_x < 0.25 * total_w:
                    left_candidates.append(x_val)
                elif 0.25 * total_w <= relative_x < 0.75 * total_w:
                    mid_candidates.append(x_val)
                elif 0.75 * total_w <= relative_x <= total_w:
                    right_candidates.append(x_val)

            # A. 三区都有线 且 中棱两侧亮度差足够大 -> 真正的面交界(3棱)
            if left_candidates and mid_candidates and right_candidates:
                best_xm = None
                max_brightness_diff = -1
                xl_cand = left_candidates[-1]
                xr_cand = right_candidates[0]

                for xm_cand in mid_candidates:
                    if 15 < xm_cand < l_enhanced.shape[1] - 15:
                        mean_left = np.mean(l_enhanced[y_b:y_b + h_b, xm_cand - 15: xm_cand - 3])
                        mean_right = np.mean(l_enhanced[y_b:y_b + h_b, xm_cand + 3: xm_cand + 15])
                        diff = abs(mean_left - mean_right)

                        if diff > max_brightness_diff:
                            max_brightness_diff = diff
                            best_xm = xm_cand

                if best_xm is not None and max_brightness_diff > 15:
                    xl, xm, xr = xl_cand, best_xm, xr_cand
                    detected_state = "3_EDGES"

            # B. 不满足3棱, 但能找到稳定的左右两条物理边缘 -> 只看到正面(2棱, 已对准)
            if detected_state is None and left_candidates and right_candidates:
                xl = left_candidates[-1]
                xr = right_candidates[0]
                detected_state = "2_EDGES"

        # --- 7. 逻辑计算 ---
        raw_error = None
        raw_dx = None

        if detected_state == "3_EDGES":
            wl = xm - xl
            wr = xr - xm
            raw_error = wl - wr                          # 透视偏差(两面宽度差)
            raw_dx = (xl + xr) / 2 - (TARGET_X - 50)     # 整体居中偏移

            if self._debug_gui:
                cx_large = (xl + xm) // 2 if wl >= wr else (xm + xr) // 2
                cx_small = (xm + xr) // 2 if wl >= wr else (xl + xm) // 2
                cv2.line(roi, (xl, 0), (xl, 400), (100, 100, 100), 2)
                cv2.line(roi, (xm, 0), (xm, 400), (100, 100, 100), 2)
                cv2.line(roi, (xr, 0), (xr, 400), (100, 100, 100), 2)
                cv2.line(roi, (cx_large, 0), (cx_large, 400), (255, 255, 0), 3)
                cv2.line(roi, (cx_small, 0), (cx_small, 400), (0, 255, 0), 3)

        elif detected_state == "2_EDGES":
            cx_large = (xl + xr) // 2
            raw_error = 0     # 只有一个面, 透视偏差归零
            raw_dx = cx_large - (TARGET_X - 50)

            if self._debug_gui:
                cv2.line(roi, (xl, 0), (xl, 400), (100, 100, 100), 2)
                cv2.line(roi, (xr, 0), (xr, 400), (100, 100, 100), 2)
                cv2.line(roi, (cx_large, 0), (cx_large, 400), (255, 255, 0), 3)

        if raw_error is not None and raw_dx is not None:
            self._error_history.append(raw_error)
            self._dx_history.append(raw_dx)
            self._no_target_count = 0
        else:
            self._no_target_count += 1

        # --- 8. 10帧平均值 + 两级减速 -> 底盘横向微调指令 ---
        cmd = Twist()
        status = FINE_ALIGN_STATUS_ALIGNING
        avg_error, avg_dx, output_speed, direction_text = 0.0, 0.0, 0, 'HOLD'

        if len(self._error_history) == HISTORY_SIZE:
            avg_error = sum(self._error_history) / HISTORY_SIZE
            avg_dx    = sum(self._dx_history) / HISTORY_SIZE

            direction = 0   # +1 = "MOVE LEFT" -1 = "MOVE RIGHT" 0 = 已居中

            if abs(avg_error) > DEADZONE_ERR:
                direction = 1 if avg_error > 0 else -1
                direction_text = '<- MOVE LEFT (Align Perspective)' if direction > 0 \
                    else 'MOVE RIGHT (Align Perspective) ->'
                error_val = abs(avg_error)
                if error_val >= START_DECEL_ERR:
                    output_speed = MAX_SPEED_LIMIT
                else:
                    ratio = (error_val - DEADZONE_ERR) / (START_DECEL_ERR - DEADZONE_ERR)
                    output_speed = MIN_SPEED_LIMIT + ratio * (MAX_SPEED_LIMIT - MIN_SPEED_LIMIT)

            elif abs(avg_dx) > DEADZONE_DX:
                direction = -1 if avg_dx > 0 else 1
                direction_text = 'MOVE RIGHT (Align Crosshair) ->' if avg_dx > 0 \
                    else '<- MOVE LEFT (Align Crosshair)'
                dx_val = abs(avg_dx)
                if dx_val >= START_DECEL_DX:
                    output_speed = MAX_SPEED_LIMIT
                else:
                    ratio = (dx_val - DEADZONE_DX) / (START_DECEL_DX - DEADZONE_DX)
                    output_speed = MIN_SPEED_LIMIT + ratio * (MAX_SPEED_LIMIT - MIN_SPEED_LIMIT)
            else:
                direction_text = 'CENTERED OK (FIRE!)'

            output_speed = int(np.clip(output_speed, 0, MAX_SPEED_LIMIT))

            if direction == 0:
                self._centered_count += 1
                if self._centered_count >= FINE_ALIGN_CONFIRM_FRAMES:
                    status = FINE_ALIGN_STATUS_DONE
            else:
                self._centered_count = 0
                # direction>0("MOVE LEFT")-> linear.y为正 (REP-103: +y朝左)
                cmd.linear.y = direction * (output_speed / 100.0) * FINE_ALIGN_MAX_SPEED_MPS

        elif self._no_target_count > HISTORY_SIZE:
            status = FINE_ALIGN_STATUS_NO_TARGET

        if status != FINE_ALIGN_STATUS_DONE:
            self._cmd_pub.publish(cmd)
        else:
            self._cmd_pub.publish(Twist())   # 已居中, 停止微调

        status_msg = UInt8()
        status_msg.data = status
        self._status_pub.publish(status_msg)

        # --- 9. 调试显示 ---
        if self._debug_gui:
            cv2.putText(roi, f"State: {detected_state}", (10, 30), 1, 1.2, (255, 100, 255), 2)
            cv2.putText(roi, f"Avg Err: {int(avg_error)}", (10, 60), 1, 1.2, (0, 255, 255), 2)
            cv2.putText(roi, f"Avg DX: {int(avg_dx)}", (10, 90), 1, 1.2, (255, 0, 255), 2)
            cv2.putText(roi, f"Speed Rate: {output_speed}%", (10, 125), 1, 1.2, (0, 255, 255), 2)
            cv2.putText(roi, direction_text, (10, 165), 1, 1.4,
                        (0, 255, 0) if 'OK' in direction_text else (0, 165, 255), 2)
            if cube_rect:
                cv2.rectangle(roi, (cube_rect[0], cube_rect[1]),
                               (cube_rect[0] + cube_rect[2], cube_rect[1] + cube_rect[3]), (0, 255, 0), 2)
            cv2.imshow('Fine Align', roi)
            cv2.imshow('Edges', final_edges)
            cv2.waitKey(1)


def main(args=None):
    rclpy.init(args=args)
    node = FineAlignNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node._close_camera()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
