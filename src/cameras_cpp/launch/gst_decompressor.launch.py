"""Base-station receiver for jetson_gst_compressor.launch.py.

For each camera in config/cameras.json, launches a gscam2 node that receives
the RTP/H.265 UDP stream (port = port_base + index, same ordering as the
compressor), decodes it, and publishes it into ROS on the usual topic:

    /{camera_name}/image_raw_decoded

so downstream consumers (HMI, detectors, ...) see the same topics as with the
ffmpeg/image_transport pipeline.

Requires gscam2 (and its GStreamer deps) on the base station:
    sudo apt install ros-$ROS_DISTRO-gscam2
    # or build from source: https://github.com/clydemcqueen/gscam2

Launch arguments:
    port_base   UDP port of the first camera (default: 5600,
                must match the compressor's port_base)

Example:
    ros2 launch cameras_cpp gst_decompressor.launch.py
"""

import json
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

# Must match jetson_gst_compressor.launch.py
RTP_PAYLOAD_TYPE = 96

RTP_CAPS = ('application/x-rtp,media=video,clock-rate=90000,'
            f'encoding-name=H265,payload={RTP_PAYLOAD_TYPE}')


def create_gscam_node(camera_name, port):
    """Build a gscam2 node decoding one camera's RTP/H.265 stream."""
    # gscam2 appends its own appsink; the pipeline must end in raw RGB video.
    gscam_config = (
        f'udpsrc port={port} caps={RTP_CAPS} '
        '! rtph265depay ! h265parse ! avdec_h265 '
        '! videoconvert ! video/x-raw,format=RGB'
    )
    return Node(
        package='gscam2',
        executable='gscam_main',
        name=f'{camera_name}_gst_decoder',
        output='screen',
        parameters=[
            {'gscam_config': gscam_config},
            {'camera_name': camera_name},
            {'frame_id': camera_name},
            {'use_gst_timestamps': False},
        ],
        remappings=[
            ('image_raw', f'/{camera_name}/image_raw_decoded'),
            ('camera_info', f'/{camera_name}/camera_info'),
        ],
        respawn=True,
        respawn_delay=2.0,
    )


def setup(context, *args, **kwargs):
    port_base = int(LaunchConfiguration('port_base').perform(context))

    share_dir = get_package_share_directory('cameras_cpp')
    cameras_config_path = os.path.join(share_dir, 'config', 'cameras.json')
    with open(cameras_config_path, 'r') as f:
        cameras_json = json.load(f)

    actions = []
    for index, camera in enumerate(cameras_json.get('cameras', [])):
        camera_name = camera['name']
        port = port_base + index
        actions.append(LogInfo(
            msg=f'{camera_name}: udp port {port} -> /{camera_name}/image_raw_decoded'))
        actions.append(create_gscam_node(camera_name, port))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port_base', default_value='5600',
                              description='UDP port of the first camera; camera i uses port_base + i'),
        OpaqueFunction(function=setup),
    ])
