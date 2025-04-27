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
    # sensor_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('rover_sounds'),
    #         '/launch/sensor_launch.py'
    #     ])
    # )
   heart_yaml = os.path.join(
        get_package_share_directory('rover_manager'),
        'config',
        'heart.yaml'
    )

   dashboard_hmi = Node(
        package='rover_hmi',
        executable='dashboard_hmi_node',
        name='dashboard_hmi_node',
        parameters=[heart_yaml]

    )

    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('my_robot_package'),
    #         '/launch/navigation_launch.py'
    #     ])
    # )

   return LaunchDescription([
      dashboard_hmi
        # sensor_launch,
        # navigation_launch
    ])
