"""GStreamer-based camera compressor for the Jetson.

Alternative to jetson_compressor.launch.py (v4l2_camera + ffmpeg/nvmpi
image_transport). Instead of publishing compressed frames as ROS messages
over DDS, each camera in config/cameras.json is captured with v4l2src,
hardware-encoded with the Jetson's nvv4l2h265enc, and streamed as RTP/H.265
over UDP directly to the base station - the video path bypasses ROS/DDS
entirely. No patched ffmpeg build is required; everything used here ships
with JetPack.

Each camera streams to its own UDP port: port = port_base + index in
cameras.json (the mapping is printed at launch).

Launch arguments (all optional):
    host        Base station IP to stream to       (default: 192.168.1.5)
    port_base   First UDP port                     (default: 5600)
    width       Capture width                      (default: 640)
    height      Capture height                     (default: 480)
    framerate   Capture framerate (fps)            (default: 30)
    bitrate     Encoder bitrate in bits/s          (default: 300000,
                matches the ffmpeg compressor settings)

Example:
    ros2 launch cameras_cpp jetson_gst_compressor.launch.py host:=192.168.1.5 bitrate:=2000000

View a stream on the base station (port per camera as printed at launch):
    gst-launch-1.0 udpsrc port=5600 \
        caps="application/x-rtp,media=video,clock-rate=90000,encoding-name=H265,payload=96" \
        ! rtph265depay ! h265parse ! avdec_h265 ! videoconvert ! autovideosink sync=false
"""

import json
import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, LogInfo, OpaqueFunction
from launch.substitutions import LaunchConfiguration

# Keyframe every 7 frames, matching the ffmpeg compressor's gop_size
IFRAME_INTERVAL = 7
RTP_PAYLOAD_TYPE = 96


def load_video_device(camera_params_path):
    """Pull video_device out of a camera's ros__parameters yaml file."""
    with open(camera_params_path, 'r') as f:
        params = yaml.safe_load(f)
    return params['/**']['ros__parameters']['video_device']


def create_gst_compressor(camera_name, video_device, host, port,
                          width, height, framerate, bitrate):
    """Build a capture -> HW encode -> RTP/UDP pipeline for one camera."""
    pipeline = [
        'v4l2src', f'device={video_device}',
        '!', f'video/x-raw,width={width},height={height},framerate={framerate}/1',
        '!', 'nvvidconv',
        '!', 'video/x-raw(memory:NVMM),format=I420',
        '!', 'nvv4l2h265enc',
        f'bitrate={bitrate}',
        f'iframeinterval={IFRAME_INTERVAL}',
        f'idrinterval={IFRAME_INTERVAL}',
        'insert-sps-pps=true',   # stream is joinable mid-flight
        'maxperf-enable=true',
        'preset-level=1',        # UltraFastPreset, lowest latency
        '!', 'h265parse',
        '!', 'rtph265pay', 'config-interval=1', f'pt={RTP_PAYLOAD_TYPE}',
        '!', 'udpsink', f'host={host}', f'port={port}', 'sync=false',
    ]
    return ExecuteProcess(
        cmd=['gst-launch-1.0'] + pipeline,
        name=f'{camera_name}_gst_compressor',
        output='screen',
        respawn=True,   # keep streaming through camera hiccups
        respawn_delay=2.0,
    )


def setup(context, *args, **kwargs):
    host = LaunchConfiguration('host').perform(context)
    port_base = int(LaunchConfiguration('port_base').perform(context))
    width = LaunchConfiguration('width').perform(context)
    height = LaunchConfiguration('height').perform(context)
    framerate = LaunchConfiguration('framerate').perform(context)
    bitrate = LaunchConfiguration('bitrate').perform(context)

    share_dir = get_package_share_directory('cameras_cpp')
    cameras_config_path = os.path.join(share_dir, 'config', 'cameras.json')
    with open(cameras_config_path, 'r') as f:
        cameras_json = json.load(f)

    actions = []
    for index, camera in enumerate(cameras_json.get('cameras', [])):
        camera_name = camera['name']
        camera_params_path = os.path.join(share_dir, 'config', camera['params_file'])
        video_device = load_video_device(camera_params_path)
        port = port_base + index

        actions.append(LogInfo(msg=f'{camera_name}: {video_device} -> udp://{host}:{port} (RTP/H.265)'))
        actions.append(create_gst_compressor(
            camera_name, video_device, host, port,
            width, height, framerate, bitrate))

    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('host', default_value='192.168.1.5',
                              description='Base station IP to stream to'),
        DeclareLaunchArgument('port_base', default_value='5600',
                              description='UDP port of the first camera; camera i uses port_base + i'),
        DeclareLaunchArgument('width', default_value='640',
                              description='Capture width'),
        DeclareLaunchArgument('height', default_value='480',
                              description='Capture height'),
        DeclareLaunchArgument('framerate', default_value='30',
                              description='Capture framerate (fps)'),
        DeclareLaunchArgument('bitrate', default_value='300000',
                              description='H.265 encoder bitrate in bits/s'),
        OpaqueFunction(function=setup),
    ])
