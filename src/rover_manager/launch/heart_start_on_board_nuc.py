import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():


    heart_node_nuc = Node(
        package='rover_manager',
        executable='heart_node',
        name='heart_on_board_nuc'
    )


    return LaunchDescription([
        heart_node_nuc
  
    ])
