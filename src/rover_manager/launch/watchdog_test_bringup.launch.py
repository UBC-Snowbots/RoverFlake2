from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    param_file = os.path.join(
        get_package_share_directory('rover_manager'),
        'config',
        'system_config.yaml'
    )

    return LaunchDescription([
        Node(
            package='rover_manager',
            executable='watchdog_node',  # The actual C++ binary
            name='Watchdog',
            parameters=[param_file]
        )
    ])
