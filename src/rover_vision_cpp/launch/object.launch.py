import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    cameras_node = Node(
        package='rover_vision_cpp',
        executable='cameras_node',
        name='camera_pub_node'
    )

    object_node = Node(
        package='rover_vision_cpp',
        executable='object_detection_node',
        name='yolo_detector_node',
    )

    return LaunchDescription([
        cameras_node,
        object_node
    ])
