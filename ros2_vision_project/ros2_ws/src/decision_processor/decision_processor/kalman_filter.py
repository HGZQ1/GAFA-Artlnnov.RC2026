"""
kalman_filter.py
2D 卡尔曼滤波器，追踪目标在机器人坐标系中的位置
状态向量：[x, y, vx, vy]（位置 + 速度）
"""
import numpy as np

from .config import (CAMERA_OFFSET_X, CAMERA_OFFSET_Y,
                     CAMERA_PITCH_DEG, CAMERA_YAW_DEG)



class KalmanFilter2D:
    """
    卡尔曼滤波器
    追踪目标在机器人坐标系的 (x前, y左) 坐标
    使用匀速运动模型（适合静止或缓慢移动的目标）
    """

    def __init__(self, dt: float = 0.05,
                 process_noise: float = 0.01,
                 measurement_noise: float = 0.1):
        """
        Args:
            dt:                每帧时间间隔(秒)，20Hz 对应 0.05
            process_noise:     过程噪声，越大越相信运动模型
            measurement_noise: 观测噪声，越大越不信传感器读数
        """
        # 状态转移矩阵（匀速运动模型）
        # x(t+1) = x(t) + vx*dt
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1,  0, dt],
            [0, 0,  1, 0],
            [0, 0,  0, 1]
        ], dtype=float)

        # 观测矩阵（只能观测到位置，观测不到速度）
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)

        # 过程噪声协方差矩阵
        self.Q = np.eye(4) * process_noise

        # 观测噪声协方差矩阵
        self.R = np.eye(2) * measurement_noise

        # 初始状态协方差（不确定性较大）
        self.P = np.eye(4) * 1.0

        # 状态向量 [x, y, vx, vy]
        self.x = np.zeros((4, 1))

        self.initialized = False

    def update(self, measured_x: float,
               measured_y: float) -> tuple[float, float]:
        """
        输入一次观测，返回滤波后的位置估计

        Args:
            measured_x: 观测到的 x 坐标（机器人坐标系，前方）
            measured_y: 观测到的 y 坐标（机器人坐标系，左方）
        Returns:
            (filtered_x, filtered_y)
        """
        z = np.array([[measured_x], [measured_y]])

        # 第一次调用时直接初始化
        if not self.initialized:
            self.x[0, 0] = measured_x
            self.x[1, 0] = measured_y
            self.initialized = True
            return measured_x, measured_y

        # ── 预测步骤 ──
        # 根据运动模型预测下一时刻状态
        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        # ── 更新步骤 ──
        # 融合预测值和观测值
        y_err = z - self.H @ self.x                    # 预测误差
        S     = self.H @ self.P @ self.H.T + self.R    # 创新协方差
        K     = self.P @ self.H.T @ np.linalg.inv(S)  # 卡尔曼增益
        self.x = self.x + K @ y_err
        self.P = (np.eye(4) - K @ self.H) @ self.P

        return float(self.x[0, 0]), float(self.x[1, 0])

    def reset(self):
        """目标丢失后重置滤波器"""
        self.x           = np.zeros((4, 1))
        self.P           = np.eye(4) * 1.0
        self.initialized = False
    
    
def camera_to_robot(cam_x, cam_y, cam_z):
    """
    相机坐标系 → 机器人底盘坐标系（水平面）
    
    假设 D435i 安装在机器人正前方，镜头水平朝前：
      cam_z（相机深度/前方距离） → robot_x（机器人前进方向）
      cam_x（相机右方）          → robot_y 取反（机器人左方为正）

    Args:
        cam_x, cam_y, cam_z: 相机坐标系下的 3D 坐标（米）
    Returns:
        (robot_x, robot_y): 机器人坐标系水平面坐标（米）
    """
    robot_x =  cam_z   # 前方距离
    robot_y = -cam_x   # 左右偏移（取反）
    return robot_x, robot_y

def is_valid_detection(new_pos: tuple,
                       last_pos: tuple | None,
                       max_jump_m: float = 0.8) -> bool:
    """
    跳变检测：如果目标位置在一帧内移动超过 max_jump_m 米
    认为是误检，丢弃本帧数据

    Args:
        new_pos:    本帧目标位置 (x, y)
        last_pos:   上一帧目标位置 (x, y) 或 None（第一帧）
        max_jump_m: 最大允许跳变距离（米），建议 0.5~1.0
    Returns:
        True  = 数据有效，可以使用
        False = 跳变过大，本帧丢弃
    """
    if last_pos is None:
        return True
    dist = ((new_pos[0] - last_pos[0]) ** 2 +
            (new_pos[1] - last_pos[1]) ** 2) ** 0.5
    return dist < max_jump_m


    