import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    broadcastor_nodes = []

    cameras_config_path = os.path.join(
        get_package_share_directory('cameras_cpp'),
        'config', 'cameras.json'
    )

    with open(cameras_config_path, 'r') as f:
        cameras_json = json.load(f)

    cameras = cameras_json.get('cameras', [])

    for camera in cameras:
        camera_name = camera['name']
        broadcastor_node = Node(
            package='image_transport',
            executable='republish',
            name=f'{camera_name}_decoder_node',
            remappings=[
                ('in/ffmpeg', f'/{camera_name}/image_raw/ffmpeg'),
                ('out', f'/{camera_name}/image_raw_decoded'),
            ],
            parameters=[
                {'in.ffmpeg.decoders.hvec': 'hvec'},
                {'in.ffmpeg.decoders.h264': 'h264'},
            ],
            output='screen',
            arguments=['ffmpeg', 'raw']
        )
        broadcastor_nodes.append(broadcastor_node)

    return LaunchDescription(broadcastor_nodes)