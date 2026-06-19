#!/usr/bin/env python3
"""
test_dock_align.py — 独立测试脚本（不依赖 ROS2）

直接打开相机，检测 ArUco 标志，运行 solvePnP，显示三轴控制输出。
用于验证检测效果和 vx/vy/angular 方向是否正确。

使用方法:
    python3 test_dock_align.py
    python3 test_dock_align.py --cam 2 --marker-size 0.10 --target-dist 0.30
"""
import argparse
import math
import cv2
import numpy as np

# ── 参数（可通过命令行覆盖）────────────────────────────────────
parser = argparse.ArgumentParser()
parser.add_argument('--cam',         type=int,   default=2,    help='相机设备号 (默认 2 = USB 相机)')
parser.add_argument('--marker-size', type=float, default=0.10, help='ArUco 标志实际边长 m (默认 0.10)')
parser.add_argument('--target-dist', type=float, default=0.30, help='期望停靠距离 m (默认 0.30)')
parser.add_argument('--dict',        type=int,   default=cv2.aruco.DICT_4X4_50, help='ArUco 字典 ID')
parser.add_argument('--marker-ids',  type=int,   nargs='+', default=[0], help='目标标志 ID 列表')
# 控制增益
parser.add_argument('--kp-lat',  type=float, default=0.8,  help='横向增益')
parser.add_argument('--kp-dist', type=float, default=0.5,  help='前后增益')
parser.add_argument('--kp-yaw',  type=float, default=1.2,  help='偏航增益')
parser.add_argument('--max-lin', type=float, default=0.15, help='最大线速度 m/s')
parser.add_argument('--max-ang', type=float, default=0.40, help='最大角速度 rad/s')
# 死区
parser.add_argument('--dz-lat',  type=float, default=0.015, help='横向死区 m')
parser.add_argument('--dz-dist', type=float, default=0.025, help='距离死区 m')
parser.add_argument('--dz-yaw',  type=float, default=0.052, help='偏航死区 rad ≈3°')
args = parser.parse_args()

MARKER_IDS  = set(args.marker_ids)
MARKER_SIZE = args.marker_size
TARGET_DIST = args.target_dist
KP_LAT      = args.kp_lat
KP_DIST     = args.kp_dist
KP_YAW      = args.kp_yaw
MAX_LIN     = args.max_lin
MAX_ANG     = args.max_ang
DZ_LAT      = args.dz_lat
DZ_DIST     = args.dz_dist
DZ_YAW      = args.dz_yaw

# ── 相机内参（从 D435i 标定，或使用默认值）──────────────────────
# 如果有 camera_info 话题，替换这里的值
# 否则脚本会用 OpenCV 自动估算（精度低但可以验证方向）
USE_ESTIMATED_K = True  # 设为 False 时使用下面的标定值

CALIB_K = np.array([
    [615.0,   0.0, 320.0],
    [  0.0, 615.0, 240.0],
    [  0.0,   0.0,   1.0],
], dtype=np.float64)
CALIB_D = np.zeros(5, dtype=np.float64)

# ── ArUco 检测器 ────────────────────────────────────────────────
aruco_dict   = cv2.aruco.getPredefinedDictionary(args.dict)
aruco_params = cv2.aruco.DetectorParameters()
try:
    detector    = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
    USE_NEW_API = True
except AttributeError:
    USE_NEW_API = False

# ── 标志角点世界坐标（以标志中心为原点）────────────────────────
h = MARKER_SIZE / 2.0
OBJ_PTS = np.array([
    [-h,  h, 0.0],
    [ h,  h, 0.0],
    [ h, -h, 0.0],
    [-h, -h, 0.0],
], dtype=np.float64)


def select_best(corners, ids):
    if ids is None or len(ids) == 0:
        return None
    best, best_area = None, -1.0
    for i, mid in enumerate(ids.flatten()):
        if int(mid) not in MARKER_IDS:
            continue
        area = float(cv2.contourArea(corners[i][0]))
        if area > best_area:
            best_area, best = area, corners[i][0]
    return best


def ok_color(flag):
    return (0, 220, 0) if flag else (0, 100, 255)


# ── 打开相机 ────────────────────────────────────────────────────
cap = cv2.VideoCapture(args.cam)
if not cap.isOpened():
    print(f'[错误] 无法打开相机 /dev/video{args.cam}')
    exit(1)

cap.set(cv2.CAP_PROP_FRAME_WIDTH,  640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
print(f'相机 /dev/video{args.cam} 已打开')
print(f'目标 MarkerIDs={list(MARKER_IDS)}, size={MARKER_SIZE}m, target_dist={TARGET_DIST}m')
print('按 Q 退出')

# 用实际帧分辨率估算内参（仅 USE_ESTIMATED_K=True 时有效）
ret, frame0 = cap.read()
if ret and USE_ESTIMATED_K:
    h0, w0 = frame0.shape[:2]
    fx = fy = max(w0, h0) * 1.0
    K = np.array([[fx, 0, w0/2], [0, fy, h0/2], [0, 0, 1]], dtype=np.float64)
    D = np.zeros(5, dtype=np.float64)
    print(f'[估算内参] K={K[0,0]:.0f}  (精度有限，建议用 camera_info 标定值)')
else:
    K, D = CALIB_K, CALIB_D

confirm_frames = 10
centered_count = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)

    # ArUco 检测
    if USE_NEW_API:
        corners, ids, _ = detector.detectMarkers(gray)
    else:
        corners, ids, _ = cv2.aruco.detectMarkers(gray, aruco_dict, parameters=aruco_params)

    best = select_best(corners, ids)

    if best is None:
        centered_count = 0
        cv2.putText(frame, 'SEARCHING...', (10, 40),
                    cv2.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
        cv2.putText(frame, f'MarkerIDs={list(MARKER_IDS)}', (10, 80),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.6, (200, 200, 200), 1)
        cv2.imshow('Dock Align Test', frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        continue

    # solvePnP
    img_pts = best.reshape(4, 1, 2).astype(np.float64)
    ok, rvec, tvec = cv2.solvePnP(OBJ_PTS, img_pts, K, D)
    if not ok:
        continue

    tx = float(tvec[0])
    tz = float(tvec[2])

    R_mat, _ = cv2.Rodrigues(rvec)
    marker_z = R_mat[:, 2]
    yaw_err  = math.atan2(marker_z[0], -marker_z[2])

    dist_err = tz - TARGET_DIST

    lat_ok  = abs(tx)       < DZ_LAT
    dist_ok = abs(dist_err) < DZ_DIST
    yaw_ok  = abs(yaw_err)  < DZ_YAW
    all_ok  = lat_ok and dist_ok and yaw_ok

    if all_ok:
        centered_count += 1
    else:
        centered_count = 0

    # 计算控制输出
    vx = vy = wz = 0.0
    if not all_ok:
        if not (lat_ok and dist_ok):
            vy = float(np.clip(-KP_LAT  * tx,       -MAX_LIN, MAX_LIN))
            vx = float(np.clip( KP_DIST * dist_err, -MAX_LIN, MAX_LIN))
        wz = float(np.clip(KP_YAW * yaw_err, -MAX_ANG, MAX_ANG))

    done = centered_count >= confirm_frames

    # 画标志轮廓和轴
    cv2.aruco.drawDetectedMarkers(frame, corners, ids)
    try:
        cv2.drawFrameAxes(frame, K, D, rvec, tvec, MARKER_SIZE * 0.5)
    except Exception:
        pass

    # 叠加文字
    info_lines = [
        (f'tx={tx:+.3f}m  {"OK" if lat_ok  else "ERR"}',           ok_color(lat_ok)),
        (f'tz={tz:.3f}m  err={dist_err:+.3f}  {"OK" if dist_ok else "ERR"}', ok_color(dist_ok)),
        (f'yaw={math.degrees(yaw_err):+.1f}deg  {"OK" if yaw_ok  else "ERR"}', ok_color(yaw_ok)),
        (f'vx={vx:+.3f}  vy={vy:+.3f}  wz={wz:+.3f}', (255, 255, 255)),
        (f'cnt={centered_count}/{confirm_frames}  {"★ DONE!" if done else "ALIGNING"}',
         (0, 220, 0) if done else (0, 165, 255)),
    ]
    for i, (text, color) in enumerate(info_lines):
        cv2.putText(frame, text, (10, 35 + i * 32),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.72, color, 2)

    # 终端也打印一行
    print(f'\rtx={tx:+.3f} tz={tz:.3f} yaw={math.degrees(yaw_err):+.1f}° | '
          f'vx={vx:+.3f} vy={vy:+.3f} wz={wz:+.3f} | '
          f'cnt={centered_count}{"  ★DONE" if done else ""}      ', end='')

    cv2.imshow('Dock Align Test', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
print()
