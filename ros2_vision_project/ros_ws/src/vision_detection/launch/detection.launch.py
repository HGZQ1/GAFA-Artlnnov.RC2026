from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    # 参数
    model_path_arg = DeclareLaunchArgument(
        'model_path',
        default_value='/ros2_ws/models/yolov8s.pt',
        description='Path to YOLOv8 model'
    )
    
    use_tensorrt_arg = DeclareLaunchArgument(
        'use_tensorrt',
        default_value='false',
        description='Use TensorRT engine'
    )
    
    # RealSense Launch
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            os.path.join(
                get_package_share_directory('realsense_wrapper'),
                'launch',
                'realsense.launch.py'
            )
        ])
    )
    
    # 检测节点
    detector_node = Node(
        package='vision_detection',
        executable='detector_node',
        name='yolov8_detector',
        output='screen',
        parameters=[{
            'model_path': LaunchConfiguration('model_path'),
            'conf_threshold': 0.25,
            'iou_threshold': 0.45,
            'device': 'cuda',
            'use_tensorrt': LaunchConfiguration('use_tensorrt'),
            'publish_viz': True
        }]
    )
    
    return LaunchDescription([
        model_path_arg,
        use_tensorrt_arg,
        realsense_launch,
        detector_node
    ])