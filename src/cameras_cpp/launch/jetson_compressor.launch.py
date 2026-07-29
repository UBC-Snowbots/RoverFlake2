import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
from ..config.cameras import CAMERAS

def generate_launch_description():
    nodes = []

    for camera_name, camera_params_file in CAMERAS:
        camera_params_path = os.path.join(
            get_package_share_directory('cameras_cpp'),
            'config', camera_params_file
        )
        camera_node = create_camera_node(camera_name, camera_params_path)
        nodes.append(camera_node)

    return LaunchDescription(nodes)

def create_camera_node(camera_name, camera_params):
    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name=camera_name,
        output='screen',
        parameters=[camera_params],
        remappings=[
            ('/image_raw', f'/vehicle_1/{camera_name}/image_raw'),
            ('/image_raw/ffmpeg', f'/vehicle_1/{camera_name}/image_raw/ffmpeg'),
        ],
    )