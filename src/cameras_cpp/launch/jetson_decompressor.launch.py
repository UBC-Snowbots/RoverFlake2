import time
import rclpy
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

# Suffix published by jetson_compressor.launch.py for every camera, e.g. /realsense415/image_raw/ffmpeg
FFMPEG_TOPIC_SUFFIX = '/image_raw/ffmpeg'


def create_broadcaster_node(camera_name):
    return Node(
        package='image_transport',
        executable='republish',
        name=f'{camera_name}_decoder_node',
        remappings=[
            ('in/ffmpeg', f'/{camera_name}/image_raw/ffmpeg'),
            ('out', f'/{camera_name}/image_raw_decoded'),
        ],
        parameters=[
            {'in.ffmpeg.decoders.hevc': 'hevc'},
            {'in.ffmpeg.decoders.h264': 'h264'},
        ],
        output='screen',
        arguments=['ffmpeg', 'raw']
    )


def discover_camera_names(timeout_sec=5.0):
    """Poll the ROS graph and return the names of cameras currently publishing a compressed feed."""
    rclpy.init(args=None)
    node = rclpy.create_node('jetson_decompressor_topic_discovery')
    try:
        camera_names = set()
        deadline = time.time() + timeout_sec
        while time.time() < deadline and not camera_names:
            for topic_name, _ in node.get_topic_names_and_types():
                if topic_name.endswith(FFMPEG_TOPIC_SUFFIX):
                    camera_names.add(topic_name[:-len(FFMPEG_TOPIC_SUFFIX)].lstrip('/'))
            if not camera_names:
                rclpy.spin_once(node, timeout_sec=0.5)
        return sorted(camera_names)
    finally:
        node.destroy_node()
        rclpy.shutdown()


def choose_cameras(context, *args, **kwargs):
    camera_args_list = LaunchConfiguration('cameras').perform(context).split()
    discovered_cameras = discover_camera_names()

    if camera_args_list:
        cameras = [name for name in discovered_cameras if name in camera_args_list]
    else:
        cameras = discovered_cameras

    return [create_broadcaster_node(camera_name) for camera_name in cameras]


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cameras', default_value=''),
        OpaqueFunction(function=choose_cameras),
    ])