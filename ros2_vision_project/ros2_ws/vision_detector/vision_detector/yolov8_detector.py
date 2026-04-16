#!/usr/bin/env python3
"""
YOLOv8检测器封装类
"""

import numpy as np
from ultralytics import YOLO
import cv2
from typing import List, Tuple, Optional


class YOLOv8Detector:
    """YOLOv8检测器封装"""
    
    # def model_path_turn(self) -> str:
    #     """切换模型权重
        
    #     Returns:
    #         str: 新的模型路径
    #     """
    #     print("模型权重，当前权重路径:", self.model_path)
        
    #     # 模型名称到路径的映射
    #     model_mapping = {
    #         "kfs": "kfs.pt",
    #         "wuqi": "wuqi.pt",
    #         "yolov8n": "yolov8n.pt"
    #     }
        
    #     # 显示可用模型
    #     print("可用模型:", ", ".join(model_mapping.keys()))
        
    #     # 获取用户输入
    #     model = input("请输入新的模型名称: ").strip()
        
    #     # 检查输入是否有效
    #     if model in model_mapping:
    #         new_model_path = model_mapping[model]
    #         print(f"已切换到 {new_model_path}")
    #         return new_model_path
    #     else:
    #         print(f"错误: 未知的模型名称 '{model}'，使用默认模型 yolov8n.pt")
    #         return "yolov8n.pt"

    def __init__(self, 
                 model_path: str = 'yolov8n.pt',#Optional[str] = None,
                 conf_threshold: float = 0.5,
                 iou_threshold: float = 0.45,
                 device: str = 'cuda'):
        """
        初始化YOLOv8检测器
        
        Args:
            model_path: 模型权重路径
            conf_threshold: 置信度阈值
            iou_threshold: NMS的IoU阈值
            device: 推理设备 ('cuda' or 'cpu')
        """

        # if model_path is None:
        #     model_path = "yolov8n.pt" # 默认模型权重路径
        # else:
        #     self.model_path = model_path

        #self.model_path = model_path
        self.model = YOLO(model_path)
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        
        # 预热模型
        dummy_img = np.zeros((640, 640, 3), dtype=np.uint8)
        _ = self.model(dummy_img, verbose=False, device=self.device)

    
        
    def detect(self, image: np.ndarray) -> List[dict]:
        if self.model is None:
            return []
        """
        执行目标检测
        
        Args:
            image: 输入图像 (BGR格式)
            
        Returns:
            检测结果列表，每个元素包含:
                - bbox: [x1, y1, x2, y2]
                - confidence: float
                - class_id: int
                - class_name: str
        """
        results = self.model(
            image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            verbose=False,
            device=self.device
        )[0]
        
        detections = []
        
        if results.boxes is not None and len(results.boxes) > 0:
            boxes = results.boxes
            
            for i in range(len(boxes)):
                bbox = boxes.xyxy[i].cpu().numpy()
                conf = float(boxes.conf[i].cpu().numpy())
                cls_id = int(boxes.cls[i].cpu().numpy())
                cls_name = results.names[cls_id]
                
                detections.append({
                    'bbox': bbox.tolist(),
                    'confidence': conf,
                    'class_id': cls_id,
                    'class_name': cls_name
                })
        
        return detections
    
    def visualize(self, 
                  image: np.ndarray, 
                  detections: List[dict],
                  distances: Optional[List[float]] = None) -> np.ndarray:
        """
        可视化检测结果
        
        Args:
            image: 原始图像
            detections: 检测结果
            distances: 可选的距离信息
            
        Returns:
            绘制了检测框的图像
        """
        vis_img = image.copy()
        
        for i, det in enumerate(detections):
            x1, y1, x2, y2 = map(int, det['bbox'])
            conf = det['confidence']
            cls_name = det['class_name']
            
            # 绘制边界框
            cv2.rectangle(vis_img, (x1, y1), (x2, y2), (0, 255, 0), 2)
            
            # 准备标签
            if distances is not None and i < len(distances):
                label = f"{cls_name} {conf:.2f} {distances[i]:.2f}m"
            else:
                label = f"{cls_name} {conf:.2f}"
            
            # 绘制标签背景
            (label_w, label_h), _ = cv2.getTextSize(
                label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 2
            )
            cv2.rectangle(vis_img, (x1, y1 - label_h - 10), 
                         (x1 + label_w, y1), (0, 255, 0), -1)
            
            # 绘制标签文字
            cv2.putText(vis_img, label, (x1, y1 - 5),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 0), 2)
        
        return vis_img