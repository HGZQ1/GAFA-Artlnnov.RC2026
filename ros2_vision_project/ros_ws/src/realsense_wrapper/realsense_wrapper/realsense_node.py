import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image, CameraInfo
from cv_bridge import CvBridge
import pyrealsense2 as rs
import numpy as np


class RealSenseNode(Node):
    """RealSense 相机 ROS2 节点"""
    
    def __init__(self):
        super().__init__('realsense_camera')
        
        # 参数
        self.declare_parameter('width', 640)
        self.declare_parameter('height', 480)
        self.declare_parameter('fps', 30)
        self.declare_parameter('align_depth', True)
        
        width = self.get_parameter('width').value
        height = self.get_parameter('height').value
        fps = self.get_parameter('fps').value
        align_depth = self.get_parameter('align_depth').value
        
        # 初始化 RealSense
        self.pipeline = rs.pipeline()
        config = rs.config()
        
        config.enable_stream(rs.stream.color, width, height, rs.format.bgr8, fps)
        config.enable_stream(rs.stream.depth, width, height, rs.format.z16, fps)
        
        # 启动 pipeline
        profile = self.pipeline.start(config)
        
        # 对齐
        if align_depth:
            self.align = rs.align(rs.stream.color)
        else:
            self.align = None
        
        # 获取相机内参
        color_profile = profile.get_stream(rs.stream.color).as_video_stream_profile()
        self.intrinsics = color_profile.get_intrinsics()
        
        # CV Bridge
        self.bridge = CvBridge()
        
        # 发布器
        self.color_pub = self.create_publisher(Image, '/camera/color/image_raw', 10)
        self.depth_pub = self.create_publisher(Image, '/camera/aligned_depth_to_color/image_raw', 10)
        self.camera_info_pub = self.create_publisher(CameraInfo, '/camera/color/camera_info', 10)
        
        # 定时器
        self.timer = self.create_timer(1.0 / fps, self.timer_callback)
        
        self.get_logger().info('RealSense camera node initialized')
    
    def timer_callback(self):
        """定时发布图像"""
        try:
            # 等待帧
            frames = self.pipeline.wait_for_frames(timeout_ms=1000)
            
            # 对齐
            if self.align:
                frames = self.align.process(frames)
            
            # 获取彩色和深度
            color_frame = frames.get_color_frame()
            depth_frame = frames.get_depth_frame()
            
            if not color_frame or not depth_frame:
                return
            
            # 转换为 numpy
            color_image = np.asanyarray(color_frame.get_data())
            depth_image = np.asanyarray(depth_frame.get_data())
            
            # 时间戳
            timestamp = self.get_clock().now().to_msg()
            
            # 发布彩色图
            color_msg = self.bridge.cv2_to_imgmsg(color_image, encoding='bgr8')
            color_msg.header.stamp = timestamp
            color_msg.header.frame_id = 'camera_color_optical_frame'
            self.color_pub.publish(color_msg)
            
            # 发布深度图
            depth_msg = self.bridge.cv2_to_imgmsg(depth_image, encoding='passthrough')
            depth_msg.header.stamp = timestamp
            depth_msg.header.frame_id = 'camera_depth_optical_frame'
            self.depth_pub.publish(depth_msg)
            
            # 发布相机信息
            camera_info = self.create_camera_info_msg(timestamp)
            self.camera_info_pub.publish(camera_info)
        
        except Exception as e:
            self.get_logger().error(f'Error in timer callback: {e}')
    
    def create_camera_info_msg(self, timestamp):
        """创建相机信息消息"""
        camera_info = CameraInfo()
        camera_info.header.stamp = timestamp
        camera_info.header.frame_id = 'camera_color_optical_frame'
        
        camera_info.width = self.intrinsics.width
        camera_info.height = self.intrinsics.height
        
        camera_info.k = [
            self.intrinsics.fx, 0.0, self.intrinsics.ppx,
            0.0, self.intrinsics.fy, self.intrinsics.ppy,
            0.0, 0.0, 1.0
        ]
        
        camera_info.d = list(self.intrinsics.coeffs)
        camera_info.distortion_model = 'plumb_bob'
        
        return camera_info
    
    def destroy_node(self):
        """停止相机"""
        self.pipeline.stop()
        super().destroy_node()


def main(args=None):
    rclpy.init(args=args)
    node = RealSenseNode()
    
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()