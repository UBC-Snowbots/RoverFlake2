"""Bring up the network telemetry pipeline: SSH reader + normalizer.

ssh_network_reader : polls the Ubiquiti radio over SSH and publishes the raw
                     JSON payload on network/raw (std_msgs/msg/String).
network_telemetry  : subscribes to network/raw and publishes normalized
                     per-link telemetry on network/telemetry.

All settings come from config/network_telemetry_params.yaml (single source of
truth). Override at launch with:  params_file:=/path/to/other.yaml
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_params = os.path.join(
        get_package_share_directory('network_telemetry'),
        'config',
        'network_telemetry_params.yaml',
    )
    params_file = LaunchConfiguration('params_file')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params_file',
            default_value=default_params,
            description='Path to the network telemetry params YAML.',
        ),
        Node(
            package='network_telemetry',
            executable='ssh_network_reader',
            name='ssh_network_reader',
            output='screen',
            parameters=[params_file],
        ),
        Node(
            package='network_telemetry',
            executable='network_reader',
            name='network_telemetry',
            output='screen',
            parameters=[params_file],
        ),
    ])
