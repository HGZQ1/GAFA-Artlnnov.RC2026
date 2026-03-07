#!/usr/bin/env python3
"""
深度处理和坐标转换工具函数
"""

import numpy as np
import random
from typing import Tuple, Optional


def get_object_distance(bbox: list, 
                       depth_image: np.ndarray,
                       sample_points: int = 24) -> float:
    """
    使用随机采样+中值滤波获取目标距离
    
    Args:
        bbox: [x1, y1, x2, y2] 边界框
        depth_image: 深度图像 (单位: 毫米)
        sample_points: 采样点数量
        
    Returns:
        距离（米），如果无效则返回0.0
    """
    x1, y1, x2, y2 = map(int, bbox)
    
    # 计算中心点
    cx = (x1 + x2) // 2
    cy = (y1 + y2) // 2
    
    # 计算采样范围
    width = abs(x2 - x1)
    height = abs(y2 - y1)
    min_dim = min(width, height)
    sample_range = min_dim // 4
    
    # 随机采样
    distance_list = []
    for _ in range(sample_points):
        offset_x = random.randint(-sample_range, sample_range)
        offset_y = random.randint(-sample_range, sample_range)
        
        sample_x = np.clip(cx + offset_x, 0, depth_image.shape[1] - 1)
        sample_y = np.clip(cy + offset_y, 0, depth_image.shape[0] - 1)
        
        depth_value = depth_image[sample_y, sample_x]
        
        if depth_value > 0:  # 有效深度值
            distance_list.append(depth_value)
    
    if len(distance_list) == 0:
        return 0.0
    
    # 中值滤波
    distance_array = np.array(distance_list)
    distance_array = np.sort(distance_array)
    
    # 取中间50%的数据
    start_idx = len(distance_array) // 4
    end_idx = start_idx + len(distance_array) // 2
    filtered_distances = distance_array[start_idx:end_idx]
    
    # 返回平均值（转换为米）
    return float(np.mean(filtered_distances)) / 1000.0 if len(filtered_distances) > 0 else 0.0


def pixel_to_3d_point(u: int, v: int, 
                      depth: float,
                      camera_matrix: np.ndarray) -> Tuple[float, float, float]:
    """
    将像素坐标和深度转换为3D坐标（相机坐标系）
    
    Args:
        u, v: 像素坐标
        depth: 深度值（米）
        camera_matrix: 3x3相机内参矩阵
        
    Returns:
        (x, y, z) 3D坐标（米）
    """
    fx = camera_matrix[0, 0]
    fy = camera_matrix[1, 1]
    cx = camera_matrix[0, 2]
    cy = camera_matrix[1, 2]
    
    z = depth
    x = (u - cx) * z / fx
    y = (v - cy) * z / fy
    
    return x, y, z


def calculate_azimuth_elevation(x: float, y: float, z: float) -> Tuple[float, float]:
    """
    计算方位角和俯仰角
    
    Args:
        x, y, z: 3D坐标（相机坐标系）
        
    Returns:
        (azimuth, elevation) 方位角和俯仰角（弧度）
    """
    # 方位角（水平角度）
    azimuth = np.arctan2(x, z)
    
    # 俯仰角（垂直角度）
    horizontal_dist = np.sqrt(x**2 + z**2)
    elevation = np.arctan2(-y, horizontal_dist)
    
    return azimuth, elevation