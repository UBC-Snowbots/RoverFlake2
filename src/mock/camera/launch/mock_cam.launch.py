from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():
    pkg_share = get_package_share_directory('mock')
    param_file = os.path.join(pkg_share, 'camera', 'config', 'mock_cam.yaml')

    return LaunchDescription([
        Node(
            package='mock',
            executable='mock_camera_node',
            name='mock_camera_node',
            output='screen',
            parameters=[param_file]
        )
    ])