import json
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy
def generate_launch_description():
    nodes = []

    cameras_config_path = os.path.join(
        get_package_share_directory('cameras_cpp'),
        'config', 'cameras.json'
    )
    with open(cameras_config_path, 'r') as f:
        cameras_json = json.load(f)

    cameras = cameras_json.get('cameras', [])

    for camera in cameras:
        camera_name = camera['name']
        camera_params_file = camera['params_file']
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
        parameters=[
            camera_params, 
            {'image_raw.ffmpeg.encoder': 'h264_nvmpi'},
        ],
        remappings=[
            ('/image_raw', f'/{camera_name}/image_raw'),
            ('/image_raw/ffmpeg', f'/{camera_name}/image_raw/ffmpeg'),
        ],
    )