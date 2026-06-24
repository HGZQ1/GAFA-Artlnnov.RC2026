import cv2
import numpy as np
import time
import os
from collections import deque

# --- 1. 自动相机硬件参数配置 ---
CAM_INDEX = 2
print(f"正在一键配置 /dev/video{CAM_INDEX} 的最佳参数...")
init_cmd = (
    f"v4l2-ctl -d /dev/video{CAM_INDEX} -c "
    f"auto_exposure=1,"                  
    f"exposure_time_absolute=100,"       
    f"white_balance_automatic=0,"        
    f"white_balance_temperature=5500,"   
    f"hue=0,"                            
    f"gain=100,"                         
    f"saturation=70"                     
)
os.system(init_cmd)
print("相机硬件参数配置锁定成功！")

def nothing(x): pass

# --- 2. 初始化摄像头 ---
cap = cv2.VideoCapture(CAM_INDEX) 
cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)
cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter_fourcc(*'MJPG'))
cap.set(cv2.CAP_PROP_FPS, 120)

cv2.namedWindow('Config')
cv2.createTrackbar('Mode: B/R', 'Config', 0, 1, nothing)
cv2.createTrackbar('Color_Thresh', 'Config', 115, 255, nothing) 
cv2.createTrackbar('Edge_Sens', 'Config', 35, 100, nothing)    
cv2.createTrackbar('CLAHE', 'Config', 4, 10, nothing)

# =================================================================
# 3. 核心控制参数配置 (线性减速区间)
# =================================================================
DEADZONE_ERR = 15       
DEADZONE_DX = 20        
START_DECEL_ERR = 120   
START_DECEL_DX = 200    
MIN_SPEED_LIMIT = 15    
MAX_SPEED_LIMIT = 100   

history_size = 10
error_history = deque(maxlen=history_size)
dx_history = deque(maxlen=history_size)
mask_history = deque(maxlen=3) 

TARGET_X = 320 
prev_time = 0
clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8,8))

while True:
    ret, frame = cap.read()
    if not ret: break

    # --- 4. 图像预处理 ---
    roi = frame[50:450, 50:590].copy()
    lab = cv2.cvtColor(roi, cv2.COLOR_BGR2LAB)
    l, a, b_chan = cv2.split(lab)

    mode = cv2.getTrackbarPos('Mode: B/R', 'Config')
    c_thresh = cv2.getTrackbarPos('Color_Thresh', 'Config')
    e_sens = cv2.getTrackbarPos('Edge_Sens', 'Config')
    c_clip = cv2.getTrackbarPos('CLAHE', 'Config')

    if c_clip > 0:
        clahe.setClipLimit(c_clip)
        l_enhanced = clahe.apply(l)
    else:
        l_enhanced = l

    if mode == 0: color_mask = cv2.inRange(b_chan, 0, c_thresh)
    else: color_mask = cv2.inRange(a, c_thresh, 255)
    
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_CLOSE, np.ones((7,7), np.uint8))
    color_mask = cv2.morphologyEx(color_mask, cv2.MORPH_OPEN, np.ones((5,5), np.uint8))

    mask_history.append(color_mask)
    if len(mask_history) == 3:
        smoothed_mask = cv2.bitwise_and(mask_history[0], mask_history[1])
        color_mask = cv2.bitwise_and(smoothed_mask, mask_history[2])

    final_edges = np.zeros_like(color_mask)

    # --- 5. 空间约束：只在最大色块范围内找线 ---
    contours, _ = cv2.findContours(color_mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
    
    vertical_x = []
    cube_rect = None
    x_min, x_max = 0, 0 

    if contours:
        max_c = max(contours, key=cv2.contourArea)
        if cv2.contourArea(max_c) > 3000: 
            x_b, y_b, w_b, h_b = cv2.boundingRect(max_c)
            
            # 动态计算 15% 的横向扩张余量
            margin_x = int(w_b * 0.15) 
            margin_y = int(h_b * 0.05) 
            
            x_min = max(0, x_b - margin_x)
            x_max = min(roi.shape[1], x_b + w_b + margin_x)
            y_min = max(0, y_b - margin_y)
            y_max = min(roi.shape[0], y_b + h_b + margin_y)
            
            cube_rect = (x_min, y_min, x_max - x_min, y_max - y_min)
            
            l_edges = cv2.Canny(l_enhanced, e_sens, e_sens*3)
            
            # 使用宽裕限幅区生成 ROI 掩码
            mask_roi = np.zeros_like(color_mask)
            cv2.rectangle(mask_roi, (x_min, y_min), (x_max, y_max), 255, -1)
            final_edges = cv2.bitwise_and(l_edges, mask_roi)

            # 提取垂直线
            lines = cv2.HoughLinesP(final_edges, 1, np.pi/180, 40, minLineLength=h_b*0.4, maxLineGap=40)
            
            if lines is not None:
                for line in lines:
                    x1, y1, x2, y2 = line[0]
                    angle = np.abs(np.arctan2(y2 - y1, x2 - x1) * 180 / np.pi)
                    
                    if 86 < angle < 94:
                        if x_min <= (x1+x2)/2 <= x_max:
                            vertical_x.append((x1 + x2) // 2)

    # --- 6. 核心：自适应确定“面数”与“棱数” ---
    detected_state = None  # 状态标志： "2_EDGES" (只看到正面) 或 "3_EDGES" (看到两个面)
    xl, xm, xr = None, None, None

    if len(vertical_x) > 0 and cube_rect is not None:
        vertical_x.sort()
        raw_edges = [vertical_x[0]]
        for i in range(1, len(vertical_x)):
            if vertical_x[i] - raw_edges[-1] > 25: 
                raw_edges.append(vertical_x[i])
        
        # 分区过滤机制
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

        # 核心决策逻辑：
        # A. 如果三个区域都有线，且中棱两边的亮度差足够大 (说明是真正的面交界，而不是图案)
        if left_candidates and mid_candidates and right_candidates:
            best_xm = None
            max_brightness_diff = -1
            xl_cand = left_candidates[-1]
            xr_cand = right_candidates[0]
            
            for xm_cand in mid_candidates:
                if 15 < xm_cand < l_enhanced.shape[1] - 15:
                    mean_left = np.mean(l_enhanced[y_b:y_b+h_b, xm_cand-15 : xm_cand-3])
                    mean_right = np.mean(l_enhanced[y_b:y_b+h_b, xm_cand+3 : xm_cand+15])
                    diff = abs(mean_left - mean_right)
                    
                    if diff > max_brightness_diff:
                        max_brightness_diff = diff
                        best_xm = xm_cand
            
            # 亮度阶跃阈值设为 15，大于 15 确认为 3 棱状态
            if best_xm is not None and max_brightness_diff > 15:
                xl = xl_cand
                xm = best_xm
                xr = xr_cand
                detected_state = "3_EDGES"

        # B. 如果不满足 3 棱（没有亮度阶跃），但能找到稳定的左右两个物理边缘线
        if detected_state is None and left_candidates and right_candidates:
            xl = left_candidates[-1]
            xr = right_candidates[0]
            detected_state = "2_EDGES"

    # --- 7. 逻辑计算与平滑滤波 ---
    raw_error = None
    raw_dx = None

    if detected_state == "3_EDGES":
        # 状态 A：看到两个面。绘制灰线，并分别计算大面（青）和小面（绿）中线
        wl = xm - xl  
        wr = xr - xm  
        
        if wl >= wr:
            cx_large = (xl + xm) // 2
            cx_small = (xm + xr) // 2
        else:
            cx_large = (xm + xr) // 2
            cx_small = (xl + xm) // 2
            
        raw_error = wl - wr  # 有透视偏差
        raw_dx = (xl + xr) / 2 - (TARGET_X - 50) 

        # 绘制三条物理棱
        cv2.line(roi, (xl, 0), (xl, 400), (100, 100, 100), 2)
        cv2.line(roi, (xm, 0), (xm, 400), (100, 100, 100), 2)
        cv2.line(roi, (xr, 0), (xr, 400), (100, 100, 100), 2)
        
        # 绘制大面中线（青色）
        cv2.line(roi, (cx_large, 0), (cx_large, 400), (255, 255, 0), 3)
        # 绘制小面中线（绿色）
        cv2.line(roi, (cx_small, 0), (cx_small, 400), (0, 255, 0), 3)

    elif detected_state == "2_EDGES":
        # 状态 B：对准了，只看到一个面。绿色中线消失，仅绘制 1 条青色正面中线
        cx_large = (xl + xr) // 2
        
        raw_error = 0 # 既然只有一个面，透视偏差直接归零！
        raw_dx = cx_large - (TARGET_X - 50)

        # 绘制两条物理边缘线
        cv2.line(roi, (xl, 0), (xl, 400), (100, 100, 100), 2)
        cv2.line(roi, (xr, 0), (xr, 400), (100, 100, 100), 2)
        
        # 仅绘制大面中线（青色）
        cv2.line(roi, (cx_large, 0), (cx_large, 400), (255, 255, 0), 3)

    # 喂给平滑历史队列
    if raw_error is not None and raw_dx is not None:
        error_history.append(raw_error)
        dx_history.append(raw_dx)

    # --- 8. 计算10帧平均值与线性减速 ---
    if len(error_history) == history_size:
        avg_error = sum(error_history) / history_size
        avg_dx = sum(dx_history) / history_size

        status = "HOLD"
        output_speed = 0

        # 两阶段平滑减速
        if abs(avg_error) > DEADZONE_ERR:
            if avg_error > 0: status = "<- MOVE LEFT (Align Perspective)"
            else: status = "MOVE RIGHT (Align Perspective) ->"
            error_val = abs(avg_error)
            if error_val >= START_DECEL_ERR: output_speed = MAX_SPEED_LIMIT
            else:
                ratio = (error_val - DEADZONE_ERR) / (START_DECEL_ERR - DEADZONE_ERR)
                output_speed = MIN_SPEED_LIMIT + ratio * (MAX_SPEED_LIMIT - MIN_SPEED_LIMIT)

        elif abs(avg_dx) > DEADZONE_DX:
            if avg_dx > 0: status = "MOVE RIGHT (Align Crosshair) ->"
            else: status = "<- MOVE LEFT (Align Crosshair)"
            dx_val = abs(avg_dx)
            if dx_val >= START_DECEL_DX: output_speed = MAX_SPEED_LIMIT
            else:
                ratio = (dx_val - DEADZONE_DX) / (START_DECEL_DX - DEADZONE_DX)
                output_speed = MIN_SPEED_LIMIT + ratio * (MAX_SPEED_LIMIT - MIN_SPEED_LIMIT)
        else:
            status = "🏆 CENTERED OK (FIRE!)"
            output_speed = 0

        output_speed = int(np.clip(output_speed, 0, MAX_SPEED_LIMIT))

        # 画面显示
        cv2.putText(roi, f"State: {detected_state}", (10, 30), 1, 1.2, (255, 100, 255), 2)
        cv2.putText(roi, f"Avg Err: {int(avg_error)}", (10, 60), 1, 1.2, (0, 255, 255), 2)
        cv2.putText(roi, f"Avg DX: {int(avg_dx)}", (10, 90), 1, 1.2, (255, 0, 255), 2)
        speed_color = (0, 255, 255) if output_speed > 20 else (0, 255, 0)
        cv2.putText(roi, f"Speed Rate: {int(output_speed)}%", (10, 125), 1, 1.2, speed_color, 2)
        cv2.putText(roi, status, (10, 165), 1, 1.4, (0, 255, 0) if "OK" in status else (0, 165, 255), 2)
        
        if cube_rect:
            cv2.rectangle(roi, (cube_rect[0], cube_rect[1]), 
                          (cube_rect[0]+cube_rect[2], cube_rect[1]+cube_rect[3]), (0, 255, 0), 2)
    else:
        cv2.putText(roi, f"Sampling... {len(error_history)}/10", (10, 30), 1, 1.2, (0, 0, 255), 2)

    # --- 9. 显示 ---
    fps = 1 / (time.time() - prev_time + 1e-6)
    prev_time = time.time()
    cv2.putText(roi, f"FPS: {int(fps)}", (10, 380), 1, 1, (255,255,255), 1)

    cv2.imshow('Triple Edge Detection (Smoothed)', roi)
    cv2.imshow('Constrained Edges', final_edges)

    if cv2.waitKey(1) & 0xFF == ord('q'): break

cap.release()
cv2.destroyAllWindows()