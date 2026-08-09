import json
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

DEFAULT_PARAMS = [
    {'image_raw.ffmpeg.encoder': 'hevc_nvmpi'},
    {'image_raw.ffmpeg.bit_rate': 300000},
    {'image_raw.ffmpeg.gop_size': 7},
    {'image_raw.ffmpeg.qmax': 30},
    {'image_raw.ffmpeg.encoder_av_options': 'num_capture_buffers:16,profile:main,preset:ultrafast'},
    {'qos_overrides./image_raw/ffmpeg.publisher.depth': 5},
    {'qos_overrides./image_raw/ffmpeg.publisher.reliability': 'reliable'},
]


def create_camera_node(camera_name, camera_params_path):
    params = DEFAULT_PARAMS.copy()
    params.append(camera_params_path)

    return Node(
        package='v4l2_camera',
        executable='v4l2_camera_node',
        name=camera_name,
        output='screen',
        parameters=params,
        remappings=[
            ('/image_raw', f'/{camera_name}/image_raw'),
            ('/image_raw/ffmpeg', f'/{camera_name}/image_raw/ffmpeg'),
        ],
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
        camera_params_file = camera['params_file']
        camera_params_path = os.path.join(
            get_package_share_directory('cameras_cpp'),
            'config', camera_params_file
        )
        nodes.append(create_camera_node(camera_name, camera_params_path))

    return nodes


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('cameras', default_value=''),
        OpaqueFunction(function=choose_cameras),
    ])
