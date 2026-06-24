#!/usr/bin/env python3
import cv2
import numpy as np
from vision_detector.yolov8_detector import YOLOv8Detector

# 初始化检测器
detector = YOLOv8Detector('yolov8n.pt')

# 读取测试图像
test_image = cv2.imread('/home/hgzq/ros2_ws/test1.jpg')

# 执行检测
detections = detector.detect(test_image)

# 可视化
vis_image = detector.visualize(test_image, detections)

cv2.imshow('Detection', vis_image)
cv2.waitKey(0)
cv2.destroyAllWindows()

print(f"Detected {len(detections)} objects:")
for det in detections:
    print(f"  {det['class_name']}: {det['confidence']:.2f}")