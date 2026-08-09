import os
import json
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration


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


def choose_cameras(context, *args, **kwargs):
    camera_args = LaunchConfiguration('cameras').perform(context)
    camera_args_list = camera_args.split()

    cameras_config_path = os.path.join(
        get_package_share_directory('cameras_cpp'),
        'config', 'cameras.json'
    )
    with open(cameras_config_path, 'r') as f:
        cameras_json = json.load(f)
    all_cameras = cameras_json.get('cameras', [])

    if len(camera_args_list) == 0:
        cameras = all_cameras
    else:
        cameras = [c for c in all_cameras if c['name'] in camera_args_list]

    nodes = []
    for camera in cameras:
        camera_name = camera['name']
        nodes.append(create_broadcaster_node(camera_name))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cameras', default_value=''),
        OpaqueFunction(function=choose_cameras),
    ])