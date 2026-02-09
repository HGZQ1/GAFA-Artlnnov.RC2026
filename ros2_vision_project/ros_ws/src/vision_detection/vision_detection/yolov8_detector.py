import cv2
import numpy as np
from ultralytics import YOLO
import torch
from typing import List, Tuple


class YOLOv8Detector:
    """YOLOv8 目标检测器封装"""
    
    def __init__(
        self,
        model_path: str = 'models/yolov8n.pt',
        conf_threshold: float = 0.25,
        iou_threshold: float = 0.45,
        device: str = 'cuda',
        use_tensorrt: bool = False
    ):
        """
        初始化 YOLOv8 检测器
        
        Args:
            model_path: 模型路径
            conf_threshold: 置信度阈值
            iou_threshold: NMS IOU 阈值
            device: 设备 ('cuda' or 'cpu')
            use_tensorrt: 是否使用 TensorRT
        """
        self.conf_threshold = conf_threshold
        self.iou_threshold = iou_threshold
        self.device = device
        
        # 加载模型
        if use_tensorrt and model_path.endswith('.trt'):
            # TensorRT 引擎
            from ultralytics.engine.exporter import TensorRTPredictor
            self.model = YOLO(model_path, task='detect')
        else:
            # PyTorch 或 ONNX
            self.model = YOLO(model_path)
        
        # 预热
        self._warmup()
    
    def _warmup(self, size=(640, 640)):
        """模型预热"""
        dummy_img = np.zeros((size[1], size[0], 3), dtype=np.uint8)
        for _ in range(3):
            _ = self.model(dummy_img, verbose=False)
    
    def detect(
        self, 
        image: np.ndarray,
        classes: List[int] = None
    ) -> List[Tuple]:
        """
        执行目标检测
        
        Args:
            image: BGR 图像
            classes: 要检测的类别列表
        
        Returns:
            检测结果列表 [(x1, y1, x2, y2, conf, cls), ...]
        """
        # 推理
        results = self.model(
            image,
            conf=self.conf_threshold,
            iou=self.iou_threshold,
            classes=classes,
            verbose=False
        )
        
        # 解析结果
        detections = []
        for result in results:
            boxes = result.boxes
            for box in boxes:
                x1, y1, x2, y2 = box.xyxy[0].cpu().numpy()
                conf = float(box.conf[0])
                cls = int(box.cls[0])
                
                detections.append((x1, y1, x2, y2, conf, cls))
        
        return detections
    
    def get_class_name(self, class_id: int) -> str:
        """获取类别名称"""
        return self.model.names[class_id]


# ============================================
# 测试代码
# ============================================
if __name__ == "__main__":
    detector = YOLOv8Detector(
        model_path='models/yolov8s.pt',
        device='cuda'
    )
    
    # 测试图像
    img = cv2.imread('test.jpg')
    detections = detector.detect(img)
    
    # 可视化
    for x1, y1, x2, y2, conf, cls in detections:
        cv2.rectangle(img, (int(x1), int(y1)), (int(x2), int(y2)), (0, 255, 0), 2)
        label = f"{detector.get_class_name(cls)}: {conf:.2f}"
        cv2.putText(img, label, (int(x1), int(y1)-10), 
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)
    
    cv2.imshow('Detections', img)
    cv2.waitKey(0)