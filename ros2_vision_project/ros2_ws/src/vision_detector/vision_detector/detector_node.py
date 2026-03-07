# def main():
#     print('Hi from vision_detector.')


# if __name__ == '__main__':
#     main()

#!/usr/bin/env python3
"""
YOLOv8-D435i ROS2检测节点
"""

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D, Pose2D
from geometry_msgs.msg import Point
from std_msgs.msg import Header
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import Optional

from .yolov8_detector import YOLOv8Detector
from .utils import get_object_distance, pixel_to_3d_point, calculate_azimuth_elevation


class VisionDetectorNode(Node):
    """视觉检测ROS2节点"""
    
    def __init__(self):
        super().__init__('vision_detector_node')
        
        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', 'yolov8n.pt'),
                ('conf_threshold', 0.5),
                ('iou_threshold', 0.45),
                ('device', 'cuda'),
                ('publish_visualization', True),
                ('depth_sample_points', 24),
                ('camera_topic', '/camera/color/image_raw'),
                ('depth_topic', '/camera/depth/image_rect_raw'),
                ('camera_info_topic', '/camera/color/camera_info'),
            ]
        )
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('conf_threshold').value
        iou_threshold = self.get_parameter('iou_threshold').value
        device = self.get_parameter('device').value
        self.publish_vis = self.get_parameter('publish_visualization').value
        self.depth_sample_points = self.get_parameter('depth_sample_points').value
        
        # 初始化检测器
        self.get_logger().info(f'Initializing YOLOv8 detector with model: {model_path}')
        self.detector = YOLOv8Detector(
            model_path=model_path,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            device=device
        )
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 相机内参（延迟初始化）
        self.camera_matrix: Optional[np.ndarray] = None
        
        # 缓存深度图像
        self.latest_depth_image: Optional[np.ndarray] = None
        
        # 订阅器
        camera_topic = self.get_parameter('camera_topic').value
        depth_topic = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value
        
        self.color_sub = self.create_subscription(
            Image,
            camera_topic,
            self.color_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            depth_topic,
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            camera_info_topic,
            self.camera_info_callback,
            10
        )
        
        # 发布器
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        if self.publish_vis:
            self.vis_pub = self.create_publisher(
                Image,
                '/detection_image',
                10
            )
        
        # 统计信息
        self.frame_count = 0
        self.detection_count = 0
        
        # 定时器：打印统计信息
        self.create_timer(5.0, self.print_stats)
        
        self.get_logger().info('Vision Detector Node initialized successfully')
    
    def camera_info_callback(self, msg: CameraInfo):
        """接收相机内参"""
        if self.camera_matrix is None:
            K = np.array(msg.k).reshape(3, 3)
            self.camera_matrix = K
            self.get_logger().info(f'Received camera intrinsics:\n{K}')
    
    def depth_callback(self, msg: Image):
        """接收深度图像"""
        try:
            # 转换为numpy数组（单位：毫米）
            depth_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
            self.latest_depth_image = depth_image
        except Exception as e:
            self.get_logger().error(f'Failed to convert depth image: {e}')
    
    def color_callback(self, msg: Image):
        """接收彩色图像并执行检测"""
        try:
            # 转换图像
            color_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 执行检测
            detections = self.detector.detect(color_image)
            
            # 计算距离
            distances = []
            if self.latest_depth_image is not None:
                for det in detections:
                    dist = get_object_distance(
                        det['bbox'],
                        self.latest_depth_image,
                        self.depth_sample_points
                    )
                    distances.append(dist)
            
            # 发布检测结果
            self.publish_detections(msg.header, detections, distances)
            
            # 发布可视化
            if self.publish_vis:
                vis_image = self.detector.visualize(color_image, detections, distances)
                vis_msg = self.bridge.cv2_to_imgmsg(vis_image, encoding='bgr8')
                vis_msg.header = msg.header
                self.vis_pub.publish(vis_msg)
            
            # 更新统计
            self.frame_count += 1
            self.detection_count += len(detections)
            
        except Exception as e:
            self.get_logger().error(f'Error in color_callback: {e}')
    
    def publish_detections(self, 
                          header: Header,
                          detections: list,
                          distances: list):
        """发布检测结果"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        for i, det in enumerate(detections):
            detection_msg = Detection2D()
            detection_msg.header = header
            
            # 边界框
            bbox = BoundingBox2D()
            x1, y1, x2, y2 = det['bbox']
            bbox.center = Pose2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            detection_msg.bbox = bbox
            
            # 检测结果
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score = det['confidence']
            detection_msg.results.append(hypothesis)
            
            # 源ID（可选）
            #detection_msg.source_img = header
            
            detection_array.detections.append(detection_msg)
        
        self.detection_pub.publish(detection_array)
    
    def print_stats(self):
        """打印统计信息"""
        if self.frame_count > 0:
            avg_detections = self.detection_count / self.frame_count
            self.get_logger().info(
                f'Stats - Frames: {self.frame_count}, '
                f'Total Detections: {self.detection_count}, '
                f'Avg: {avg_detections:.2f} det/frame'
            )


def main(args=None):
    rclpy.init(args=args)
    node = VisionDetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()