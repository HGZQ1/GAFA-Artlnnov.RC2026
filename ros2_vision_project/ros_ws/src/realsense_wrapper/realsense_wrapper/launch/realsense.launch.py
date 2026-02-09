from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    realsense_node = Node(
        package='realsense_wrapper',
        executable='realsense_node',
        name='realsense_camera',
        output='screen',
        parameters=[{
            'width': 640,
            'height': 480,
            'fps': 30,
            'align_depth': True
        }]
    )
    
    return LaunchDescription([realsense_node])