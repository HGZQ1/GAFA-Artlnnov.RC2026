"""
kalman_filter.py
2D 卡尔曼滤波器，追踪目标在机器人坐标系中的位置
坐标变换改为通过 TFManager 进行，不再手动计算
"""
import numpy as np
from .config import MAX_JUMP_M


class KalmanFilter2D:
    """
    卡尔曼滤波器
    追踪目标在底盘坐标系（base_link）的 (x前, y左) 坐标
    """

    def __init__(self, dt: float = 0.05,
                 process_noise: float = 0.01,
                 measurement_noise: float = 0.1):
        self.F = np.array([
            [1, 0, dt, 0],
            [0, 1,  0, dt],
            [0, 0,  1, 0],
            [0, 0,  0, 1]
        ], dtype=float)

        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=float)

        self.Q = np.eye(4) * process_noise
        self.R = np.eye(2) * measurement_noise
        self.P = np.eye(4) * 1.0
        self.x = np.zeros((4, 1))
        self.initialized = False

    def update(self, measured_x: float,
               measured_y: float) -> tuple:
        z = np.array([[measured_x], [measured_y]])

        if not self.initialized:
            self.x[0, 0] = measured_x
            self.x[1, 0] = measured_y
            self.initialized = True
            return measured_x, measured_y

        self.x = self.F @ self.x
        self.P = self.F @ self.P @ self.F.T + self.Q

        y_err = z - self.H @ self.x
        S     = self.H @ self.P @ self.H.T + self.R
        K     = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ y_err
        self.P = (np.eye(4) - K @ self.H) @ self.P

        return float(self.x[0, 0]), float(self.x[1, 0])

    def reset(self):
        self.x           = np.zeros((4, 1))
        self.P           = np.eye(4) * 1.0
        self.initialized = False


def is_valid_detection(new_pos: tuple,
                       last_pos,
                       max_jump_m: float = MAX_JUMP_M) -> bool:
    """
    跳变检测：位置在一帧内跳变超过阈值则丢弃
    """
    if last_pos is None:
        return True
    dist = ((new_pos[0] - last_pos[0]) ** 2 +
            (new_pos[1] - last_pos[1]) ** 2) ** 0.5
    return dist < max_jump_m


# 保留此函数作为 TF 不可用时的最终回退
def camera_to_robot_direct(cam_x: float,
                            cam_y: float,
                            cam_z: float) -> tuple:
    """
    直接坐标转换（不经过TF，仅用于TFManager完全失败时的兜底）
    相机坐标系(z前, x右, y下) → 底盘坐标系(x前, y左)
    """
    robot_x =  cam_z
    robot_y = -cam_x
    return robot_x, robot_y