from launch import LaunchDescription
from launch_ros.actions import Node

from ..config.cameras import CAMERAS

def generate_launch_description():
    broadcastor_nodes = []
    for camera_name, _ in CAMERAS:
        broadcastor_node = Node(
            package='image_transport',
            executable='republish',
            name=f'{camera_name}_decoder_node',
            remappings=[
                ('in/ffmpeg', f'/vehicle_1/{camera_name}/image_raw/ffmpeg'),
                ('out', f'/vehicle_1/{camera_name}/image_raw_decoded'),
            ],
            parameters=[
                {'in.ffmpeg.decoders.h264': 'h264'},
            ],
            output='screen',
            arguments=['ffmpeg', 'raw']
        )
        broadcastor_nodes.append(broadcastor_node)

    return LaunchDescription(broadcastor_nodes)