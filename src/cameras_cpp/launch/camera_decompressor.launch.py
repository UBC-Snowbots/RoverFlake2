import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    rear_ffmpeg_subscriber_node = Node(
        package='image_transport',
        executable='republish',
        name='ffmpeg_subscriber',
        remappings=[
            ('in/ffmpeg', '/vehicle_1/rear_feed/image_raw/h265'),
            ('out/image_raw', '/cam_1/image_decoded'),
        ],
        # Assuming the encoded data is using H.264, we'll set the subscriber to decode using the appropriate codec.
        # If you used a different codec or have specific decoding needs, you might need to adjust the parameters accordingly.
        parameters=[
            {'ffmpeg_image_transport.map.libx264': 'h264'},  # Map the libx265 encoder to use the h264 decoder
        ],
        arguments=['ffmpeg', 'in:=/vehicle_1/rear_feed/h265', 'raw', 'out:=/cam_1/image_decoded']
    )
    main_ffmpeg_subscriber_node = Node(
        package='image_transport',
        executable='republish',
        name='ffmpeg_subscriber',
        remappings=[
            ('in/ffmpeg', '/vehicle_1/main_feed/image_raw/h265'),
            ('out/image_raw', '/cam_2/image_decoded'),
        ],
        # Assuming the encoded data is using H.264, we'll set the subscriber to decode using the appropriate codec.
        # If you used a different codec or have specific decoding needs, you might need to adjust the parameters accordingly.
        parameters=[
            {'ffmpeg_image_transport.map.libx264': 'h264'},  # Map the libx265 encoder to use the h264 decoder
        ],
        arguments=['ffmpeg', 'in:=/vehicle_1/main_feed/image_raw/h265', 'raw', 'out:=/cam_2/image_decoded']
    )

    return LaunchDescription([
        rear_ffmpeg_subscriber_node,
        main_ffmpeg_subscriber_node
    ])