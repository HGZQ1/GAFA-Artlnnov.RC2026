#!/usr/bin/env python3
"""
YOLOv8-D435i ROS2检测节点（支持模型热切换）
"""

import gc
import os
import torch
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from vision_msgs.msg import Detection2D, Detection2DArray, ObjectHypothesisWithPose
from vision_msgs.msg import BoundingBox2D, Pose2D
from geometry_msgs.msg import Point, PointStamped
from std_msgs.msg import Header
from rcl_interfaces.msg import SetParametersResult
from cv_bridge import CvBridge
import numpy as np
import cv2
from typing import Optional

from .yolov8_detector import YOLOv8Detector
from .utils import get_object_distance, pixel_to_3d_point, calculate_azimuth_elevation

# 模型权重目录
WEIGHTS_DIR = os.path.join(
    os.path.dirname(os.path.dirname(os.path.abspath(__file__))),
    'weights')


class VisionDetectorNode(Node):
    """视觉检测ROS2节点"""

    def __init__(self):
        super().__init__('vision_detector_node')

        # 声明参数
        self.declare_parameters(
            namespace='',
            parameters=[
                ('model_path', 'best.pt'),
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
        model_path        = self.get_parameter('model_path').value
        conf_threshold    = self.get_parameter('conf_threshold').value
        iou_threshold     = self.get_parameter('iou_threshold').value
        device            = self.get_parameter('device').value
        self.publish_vis  = self.get_parameter('publish_visualization').value
        self.depth_sample_points = self.get_parameter('depth_sample_points').value

        # 模型路径处理（支持文件名或绝对路径）
        model_path = self._resolve_model_path(model_path)

        # 初始化检测器
        self.get_logger().info(f'加载模型：{model_path}')
        self.detector = YOLOv8Detector(
            model_path=model_path,
            conf_threshold=conf_threshold,
            iou_threshold=iou_threshold,
            device=device
        )

        # CV Bridge
        self.bridge = CvBridge()

        # 相机内参
        self.camera_matrix: Optional[np.ndarray] = None

        # 深度图缓存
        self.latest_depth_image: Optional[np.ndarray] = None

        # 模型切换锁（切换期间暂停检测）
        self._model_lock = False

        # 订阅器
        camera_topic      = self.get_parameter('camera_topic').value
        depth_topic       = self.get_parameter('depth_topic').value
        camera_info_topic = self.get_parameter('camera_info_topic').value

        self.color_sub = self.create_subscription(
            Image, camera_topic, self.color_callback, 10)
        self.depth_sub = self.create_subscription(
            Image, depth_topic, self.depth_callback, 10)
        self.camera_info_sub = self.create_subscription(
            CameraInfo, camera_info_topic, self.camera_info_callback, 10)

        # 发布器
        self.detection_pub = self.create_publisher(
            Detection2DArray, '/detections', 10)
        self.raw_target_pub = self.create_publisher(
            PointStamped, '/vision/raw_target', 10)

        if self.publish_vis:
            self.vis_pub = self.create_publisher(
                Image, '/detection_image', 10)

        # 统计
        self.frame_count     = 0
        self.detection_count = 0
        self.create_timer(5.0, self.print_stats)

        # 注册参数变更回调（model_switcher 通过此接口热切换）
        self.add_on_set_parameters_callback(self._on_param_change)

        self.get_logger().info(
            'Vision Detector Node 启动成功\n'
            f'  当前模型：{os.path.basename(model_path)}\n'
            f'  模型目录：{WEIGHTS_DIR}')

    # ──────────────────────────────────────────────────────
    #   辅助方法
    # ──────────────────────────────────────────────────────

    def _resolve_model_path(self, model_path: str) -> str:
        """将文件名解析为完整路径，绝对路径直接返回"""
        if os.path.isabs(model_path):
            return model_path
        full = os.path.join(WEIGHTS_DIR, model_path)
        if os.path.exists(full):
            return full
        # 兜底：原样返回，由 YOLO 自行处理
        return model_path

    # ──────────────────────────────────────────────────────
    #   参数变更回调（model_switcher 触发）
    # ──────────────────────────────────────────────────────

    def _on_param_change(self, params):
        for param in params:
            if param.name == 'model_path' and param.value:
                self.get_logger().info(
                    f'收到模型切换请求：{param.value}')
                self._switch_model(param.value)
        return SetParametersResult(successful=True)

    def _switch_model(self, new_model_path: str):
        """热切换模型：释放旧模型显存 → 加载新模型 → 预热"""
        import time
        new_model_path = self._resolve_model_path(new_model_path)

        if not os.path.exists(new_model_path):
            self.get_logger().error(f'模型文件不存在：{new_model_path}')
            return

        self.get_logger().info(f'开始切换模型：{new_model_path}')

        # 设置锁，等待正在执行的 color_callback 完成（最多等1秒）
        self._model_lock = True
        time.sleep(0.1)   # 给当前帧处理留出完成时间

        try:
            # 1. 先把 model 替换为占位对象，防止 None 被调用
            class _DummyModel:
                def __call__(self, *a, **kw):
                    return []
            self.detector.model = _DummyModel()

            # 2. 释放旧模型显存
            gc.collect()
            if torch.cuda.is_available():
                torch.cuda.empty_cache()
                used_mb = torch.cuda.memory_allocated() / 1024 ** 2
                self.get_logger().info(
                    f'显存已释放，当前占用：{used_mb:.1f} MB')

            # 3. 加载新模型
            from ultralytics import YOLO
            new_model = YOLO(new_model_path)

            # 4. 预热
            dummy = np.zeros((640, 640, 3), dtype=np.uint8)
            new_model(dummy, verbose=False, device=self.detector.device)

            # 5. 原子替换
            self.detector.model = new_model

            self.get_logger().info(
                f'模型切换成功：{os.path.basename(new_model_path)}')

        except Exception as e:
            self.get_logger().error(f'模型切换失败：{e}')
        finally:
            self._model_lock = False
    # ──────────────────────────────────────────────────────
    #   相机回调
    # ──────────────────────────────────────────────────────

    def camera_info_callback(self, msg: CameraInfo):
        if self.camera_matrix is None:
            self.camera_matrix = np.array(msg.k).reshape(3, 3)
            self.get_logger().info(
                f'相机内参已接收：\n{self.camera_matrix}')

    def depth_callback(self, msg: Image):
        try:
            self.latest_depth_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='passthrough')
        except Exception as e:
            self.get_logger().error(f'深度图转换失败：{e}')

    def color_callback(self, msg: Image):
        # 模型切换期间跳过检测
        if self._model_lock:
            return
        try:
            color_image = self.bridge.imgmsg_to_cv2(
                msg, desired_encoding='bgr8')
            
            if self.detector.model is None:
                return

            detections = self.detector.detect(color_image)

            distances = []
            if self.latest_depth_image is not None:
                for det in detections:
                    dist = get_object_distance(
                        det['bbox'],
                        self.latest_depth_image,
                        self.depth_sample_points)
                    distances.append(dist)

            self.publish_detections(msg.header, detections, distances)
            self._publish_raw_target(detections, distances, msg.header)

            if self.publish_vis:
                vis_image = self.detector.visualize(
                    color_image, detections, distances)
                vis_msg = self.bridge.cv2_to_imgmsg(
                    vis_image, encoding='bgr8')
                vis_msg.header = msg.header
                self.vis_pub.publish(vis_msg)

            self.frame_count     += 1
            self.detection_count += len(detections)

        except Exception as e:
            self.get_logger().error(f'color_callback 异常：{e}')

    # ──────────────────────────────────────────────────────
    #   发布方法
    # ──────────────────────────────────────────────────────

    def publish_detections(self, header: Header,
                           detections: list, distances: list):
        detection_array = Detection2DArray()
        detection_array.header = header

        for det in detections:
            detection_msg = Detection2D()
            detection_msg.header = header

            bbox = BoundingBox2D()
            x1, y1, x2, y2 = det['bbox']
            bbox.center = Pose2D()
            bbox.center.position.x = (x1 + x2) / 2.0
            bbox.center.position.y = (y1 + y2) / 2.0
            bbox.size_x = x2 - x1
            bbox.size_y = y2 - y1
            detection_msg.bbox = bbox

            hypothesis = ObjectHypothesisWithPose()
            hypothesis.hypothesis.class_id = str(det['class_id'])
            hypothesis.hypothesis.score    = det['confidence']
            detection_msg.results.append(hypothesis)

            detection_array.detections.append(detection_msg)

        self.detection_pub.publish(detection_array)

    def _publish_raw_target(self, detections, distances, header):
        if not detections or not distances:
            return
        if self.camera_matrix is None:
            return

        best_idx  = max(range(len(detections)),
                        key=lambda i: detections[i]['confidence'])
        best_det  = detections[best_idx]
        best_dist = distances[best_idx]

        if best_dist <= 0.0:
            return

        x1, y1, x2, y2 = best_det['bbox']
        u = int((x1 + x2) / 2)
        v = int((y1 + y2) / 2)

        px, py, _ = pixel_to_3d_point(
            u, v, best_dist, self.camera_matrix)

        msg = PointStamped()
        msg.header         = header
        msg.point.x        = float(px)
        msg.point.y        = float(py)
        msg.point.z        = float(best_dist)
        msg.header.frame_id = (
            f"{best_det['class_id']}:"
            f"{best_det['class_name']}:"
            f"{best_det['confidence']:.3f}"
        )
        self.raw_target_pub.publish(msg)

    def print_stats(self):
        if self.frame_count > 0:
            avg = self.detection_count / self.frame_count
            self.get_logger().info(
                f'Stats - Frames: {self.frame_count}  '
                f'Detections: {self.detection_count}  '
                f'Avg: {avg:.2f}/frame')


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
