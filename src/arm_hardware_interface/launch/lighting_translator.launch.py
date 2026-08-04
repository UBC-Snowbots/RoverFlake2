# Brings up the lighting translator on its own. The can node for the same loop
# has to be running too, this node only speaks topics.
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory('rover_lighting'),
        'config',
        'lighting_translator.yaml')

    config_arg = DeclareLaunchArgument(
        'config',
        default_value=default_config,
        description='Parameter file for the lighting translator')

    translator = Node(
        package='rover_lighting',
        executable='lighting_translator',
        name='lighting_translator',
        output='screen',
        parameters=[LaunchConfiguration('config')],
    )

    return LaunchDescription([config_arg, translator])