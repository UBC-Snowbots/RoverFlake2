from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    broadcast_decoder_0 = Node(
        package='image_transport',
        executable='republish',
        name='broadcast_decoder_0',
        remappings=[
            ('in/ffmpeg', '/broadcast/cam_0/h265'),
            ('out/image_raw', '/decoded/cam_0/image_raw'),
        ],
        parameters=[
            {'ffmpeg_image_transport.map.libx264': 'h264'},
        ],
        arguments=['ffmpeg', 'in:=/broadcast/cam_0/h265', 'raw', 'out:=/decoded/cam_0/image_raw']
    )
    
    broadcast_decoder_1 = Node(
        package='image_transport',
        executable='republish',
        name='broadcast_decoder_1',
        remappings=[
            ('in/ffmpeg', '/broadcast/cam_1/h265'),
            ('out/image_raw', '/decoded/cam_1/image_raw'),
        ],
        parameters=[
            {'ffmpeg_image_transport.map.libx264': 'h264'},
        ],
        arguments=['ffmpeg', 'in:=/broadcast/cam_1/h265', 'raw', 'out:=/decoded/cam_1/image_raw']
    )
    
    broadcast_decoder_2 = Node(
        package='image_transport',
        executable='republish',
        name='broadcast_decoder_2',
        remappings=[
            ('in/ffmpeg', '/broadcast/cam_2/h265'),
            ('out/image_raw', '/decoded/cam_2/image_raw'),
        ],
        parameters=[
            {'ffmpeg_image_transport.map.libx264': 'h264'},
        ],
        arguments=['ffmpeg', 'in:=/broadcast/cam_2/h265', 'raw', 'out:=/decoded/cam_2/image_raw']
    )
    
    broadcast_decoder_3 = Node(
        package='image_transport',
        executable='republish',
        name='broadcast_decoder_3',
        remappings=[
            ('in/ffmpeg', '/broadcast/cam_3/h265'),
            ('out/image_raw', '/decoded/cam_3/image_raw'),
        ],
        parameters=[
            {'ffmpeg_image_transport.map.libx264': 'h264'},
        ],
        arguments=['ffmpeg', 'in:=/broadcast/cam_3/h265', 'raw', 'out:=/decoded/cam_3/image_raw']
    )

    return LaunchDescription([
        broadcast_decoder_0,
        broadcast_decoder_1,
        broadcast_decoder_2,
        broadcast_decoder_3
    ])