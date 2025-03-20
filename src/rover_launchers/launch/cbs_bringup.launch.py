from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
import os

from ament_index_python.packages import get_package_share_directory


# Basically just a linker to the launch file in rover_manager


def generate_launch_description():
    cbs_launch_file_path = os.path.join(
        get_package_share_directory('rover_manager'),
        'launch/launch/cbs_system_bringup.launch.py'
    )

    return LaunchDescription([
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(cbs_launch_file_path)
        ),
    ])
