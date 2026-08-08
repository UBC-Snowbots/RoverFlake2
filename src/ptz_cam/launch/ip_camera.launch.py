"""Bring up the Hi3510 IP camera: gscam RTSP feed + zoom services.

Feed  : gscam decodes the camera's H.264 RTSP stream to /ip_camera/camera/image_raw (+ camera_info),
        for on-Jetson consumers (e.g. vision). See the network-transport note below.
Zoom  : ip_camera_zoom_node exposes /ip_camera/zoom_in_start, /ip_camera/zoom_out_start,
        /ip_camera/zoom_stop (std_srvs/Trigger).

All connection + zoom settings come from config/ip_camera_params.yaml (single source of
truth). Override at launch with:  params_file:=/path/to/other.yaml

--- Network transport (deliberately NOT done in this pass) ------------------------------
The camera already emits H.264. Sending the feed to the base station ~1km away should
forward those H.264 bits, NOT decode-to-raw-then-re-encode. The cameras_cpp
image_transport/republish (out.ffmpeg.*) pattern would do a full decode+re-encode on the
Jetson, which is both CPU-heavy and pointless when the source is already compressed.
Follow-up options to size the bitstream path (pick one later):
  * Passthrough the RTP H.264 (rtph264depay ! h264parse) into ffmpeg_image_transport
    FFMPEGPacket messages without re-encoding.
  * Or forward the encoded stream over the radio link outside ROS and only bring metadata
    onto the ROS graph.
For now we publish raw locally only; nothing here re-encodes.
"""

import os

import yaml
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def _launch_setup(context, *args, **kwargs):
    params_file = LaunchConfiguration('params_file').perform(context)
    with open(params_file, 'r') as f:
        cfg = yaml.safe_load(f)['ip_camera']

    # --- gscam pipeline ---------------------------------------------------
    # Explicit rtph264depay/h264parse/avdec_h264 over TCP. decodebin is deliberately
    # NOT used: it silently stalls with gscam for this camera. gscam appends the appsink.
    rtsp_url = (
        f"rtsp://{cfg['camera_user']}:{cfg['camera_pass']}"
        f"@{cfg['camera_ip']}:{cfg['rtsp_port']}/{cfg['stream_path']}"
    )
    gscam_config = (
        f"rtspsrc location={rtsp_url} latency={cfg['rtsp_latency_ms']} protocols=tcp "
        "! rtph264depay ! h264parse ! avdec_h264 ! videoconvert"
    )

    feed = Node(
        package='gscam',
        executable='gscam_node',
        name='gscam_node',
        namespace='ip_camera',
        output='screen',
        parameters=[{
            'gscam_config': gscam_config,
            'camera_name': cfg['camera_name'],
            'frame_id': cfg['frame_id'],
            'image_encoding': 'rgb8',
            'sync_sink': False,          # don't block the pipeline on the sink -> lower latency
            'use_gst_timestamps': False,
            'camera_info_url': '',       # no calibration yet
        }],
        # gscam publishes relative image_raw/camera_info; namespace resolves them to /ip_camera/*.
        remappings=[
            ('image_raw', '/ip_camera/camera/image_raw'),
            ('camera_info', '/ip_camera/camera_info'),
        ],
    )

    # --- zoom services ----------------------------------------------------
    zoom = Node(
        package='ptz_cam',
        executable='ip_camera_zoom',
        name='ip_camera_zoom_node',
        namespace='ip_camera',
        output='screen',
        parameters=[{
            'camera_ip': cfg['camera_ip'],
            'camera_user': cfg['camera_user'],
            'camera_pass': cfg['camera_pass'],
            'http_port': cfg['http_port'],
            'zoom_endpoint_path': cfg['zoom_endpoint_path'],
            'zoom_speed': cfg['zoom_speed'],
            'zoom_step': cfg['zoom_step'],
            'request_timeout': cfg['request_timeout'],
        }],
    )

    return [feed, zoom]


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('ptz_cam'), 'config', 'ip_camera_params.yaml'
    )
    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the IP camera params YAML (single source of truth).',
        ),
        OpaqueFunction(function=_launch_setup),
    ])