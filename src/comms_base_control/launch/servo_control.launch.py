from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('comms_base_control')
    param_file = os.path.join(pkg_share, 'config', 'servos.yaml')

    return LaunchDescription([
        Node(
            package='comms_base_control',
            executable='comms_relay',
            name='servo_control_node',
            output='screen',
            parameters=[param_file]
        )
    ])
