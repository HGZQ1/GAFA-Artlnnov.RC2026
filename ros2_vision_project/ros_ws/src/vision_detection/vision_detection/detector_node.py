import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2DArray, Detection2D, ObjectHypothesisWithPose
from cv_bridge import CvBridge
import numpy as np
import cv2
from .yolov8_detector import YOLOv8Detector


class DetectorNode(Node):
    """YOLOv8 ROS2 检测节点"""
    
    def __init__(self):
        super().__init__('yolov8_detector')
        
        # 声明参数
        self.declare_parameter('model_path', '/ros2_ws/models/yolov8s.pt')
        self.declare_parameter('conf_threshold', 0.25)
        self.declare_parameter('iou_threshold', 0.45)
        self.declare_parameter('device', 'cuda')
        self.declare_parameter('use_tensorrt', False)
        self.declare_parameter('publish_viz', True)
        
        # 获取参数
        model_path = self.get_parameter('model_path').value
        conf_threshold = self.get_parameter('conf_threshold').value
        iou_threshold = self.get_parameter('iou_threshold').value
        device = self.get_parameter('device').value
        use_tensorrt = self.get_parameter('use_tensorrt').value
        self.publish_viz = self.get_parameter('publish_viz').value
        
        # 初始化检测器
        self.get_logger().info(f'Loading model: {model_path}')
        self.detector = YOLOv8Detector(
            model_path=model_path,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            device=device,
            use_tensorrt=use_tensorrt
        )
        self.get_logger().info('Model loaded successfully')
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 订阅相机话题
        self.image_sub = self.create_subscription(
            Image,
            '/camera/color/image_raw',
            self.image_callback,
            10
        )
        
        self.depth_sub = self.create_subscription(
            Image,
            '/camera/aligned_depth_to_color/image_raw',
            self.depth_callback,
            10
        )
        
        self.camera_info_sub = self.create_subscription(
            CameraInfo,
            '/camera/color/camera_info',
            self.camera_info_callback,
            10
        )
        
        # 发布检测结果
        self.detection_pub = self.create_publisher(
            Detection2DArray,
            '/detections',
            10
        )
        
        # 发布可视化图像
        if self.publish_viz:
            self.viz_pub = self.create_publisher(
                Image,
                '/detections/visualization',
                10
            )
        
        # 缓存
        self.latest_depth = None
        self.camera_info = None
        
        self.get_logger().info('Detector node initialized')
    
    def depth_callback(self, msg):
        """深度图回调"""
        try:
            self.latest_depth = self.bridge.imgmsg_to_cv2(msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'Error converting depth image: {e}')
    
    def camera_info_callback(self, msg):
        """相机信息回调"""
        self.camera_info = msg
    
    def image_callback(self, msg):
        """图像回调和检测"""
        try:
            # 转换图像
            cv_image = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
            
            # 执行检测
            detections = self.detector.detect(cv_image)
            
            # 发布检测结果
            self.publish_detections(detections, msg.header)
            
            # 发布可视化
            if self.publish_viz:
                viz_image = self.visualize_detections(cv_image, detections)
                viz_msg = self.bridge.cv2_to_imgmsg(viz_image, encoding='bgr8')
                viz_msg.header = msg.header
                self.viz_pub.publish(viz_msg)
        
        except Exception as e:
            self.get_logger().error(f'Error in image callback: {e}')
    
    def publish_detections(self, detections, header):
        """发布检测结果为 ROS2 消息"""
        detection_array = Detection2DArray()
        detection_array.header = header
        
        for x1, y1, x2, y2, conf, cls in detections:
            detection = Detection2D()
            
            # 边界框
            detection.bbox.center.position.x = (x1 + x2) / 2.0
            detection.bbox.center.position.y = (y1 + y2) / 2.0
            detection.bbox.size_x = x2 - x1
            detection.bbox.size_y = y2 - y1
            
            # 类别和置信度
            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(cls)
            hypothesis.hypothesis.score = conf
            detection.results.append(hypothesis)
            
            # 如果有深度信息，计算距离
            if self.latest_depth is not None:
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                if 0 <= cy < self.latest_depth.shape[0] and 0 <= cx < self.latest_depth.shape[1]:
                    depth = self.latest_depth[cy, cx] / 1000.0  # mm to m
                    detection.bbox.center.position.z = float(depth)
            
            detection_array.detections.append(detection)
        
        self.detection_pub.publish(detection_array)
    
    def visualize_detections(self, image, detections):
        """可视化检测结果"""
        viz_image = image.copy()
        
        for x1, y1, x2, y2, conf, cls in detections:
            # 绘制边界框
            color = (0, 255, 0)
            cv2.rectangle(viz_image, (int(x1), int(y1)), (int(x2), int(y2)), color, 2)
            
            # 标签
            class_name = self.detector.get_class_name(cls)
            label = f'{class_name}: {conf:.2f}'
            
            # 如果有深度信息
            if self.latest_depth is not None:
                cx = int((x1 + x2) / 2)
                cy = int((y1 + y2) / 2)
                if 0 <= cy < self.latest_depth.shape[0] and 0 <= cx < self.latest_depth.shape[1]:
                    depth = self.latest_depth[cy, cx] / 1000.0
                    label += f' | {depth:.2f}m'
            
            # 绘制文本
            cv2.putText(viz_image, label, (int(x1), int(y1) - 10),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)
        
        return viz_image


def main(args=None):
    rclpy.init(args=args)
    node = DetectorNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()