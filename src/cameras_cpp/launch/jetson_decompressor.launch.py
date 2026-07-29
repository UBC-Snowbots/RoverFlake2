from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    broadcast_decoder_0 = Node(
        package='image_transport',
        executable='republish',
        name='test_decoder_node',
        remappings=[
            ('in/ffmpeg', '/vehicle_1/rear_feed/ffmpeg'),
            ('out', '/vehicle_1/rear_feed/image_raw_decoded'),
        ],
        parameters=[
            {'in.ffmpeg.decoders.h264': 'h264'},
        ],
        output='screen',
        arguments=['ffmpeg', 'raw']
    )

    return LaunchDescription([
        broadcast_decoder_0,
    ])