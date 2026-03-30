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

    aruco_params_yaml = os.path.join(
        get_package_share_directory('rover_vision_cpp'),
        'config',
        'aruco_params.yaml'
    )

    aruco_node = Node(
        package='rover_vision_cpp',
        executable='aruco_node',
        name='marker_qr_detection',
        parameters=[aruco_params_yaml]
    )

    return LaunchDescription([
        cameras_node,
        aruco_node
    ])
