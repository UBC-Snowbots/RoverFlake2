import os
import yaml
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    heart_yaml_base = os.path.join(
        get_package_share_directory('rover_manager'),
        'config',
        'heart_base.yaml'
    )

    heart_node_base = Node(
        package='rover_manager',
        executable='heart_node',
        name='heart_control_base',
        parameters=[heart_yaml_base]
    )

    return LaunchDescription([
        heart_node_base
    ])
