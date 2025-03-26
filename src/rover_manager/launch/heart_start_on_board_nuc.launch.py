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
    heart_yaml_onboard = os.path.join(
        get_package_share_directory('rover_manager'),
        'config',
        'heart_onboard.yaml'
    )
    heart_node_onboard = Node(
        package='rover_manager',
        executable='heart_node',
        name='heart_onboard_nuc',
        parameters=[heart_yaml_onboard]
    )


    return LaunchDescription([
        heart_node_onboard
  
    ])
