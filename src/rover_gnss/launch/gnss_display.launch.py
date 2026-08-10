"""Live GNSS display: nmea_reader off the serial receiver + marker node.

Points and labels come from a params yaml, so a new task means editing the
yaml (or pointing at a different one), not this launch file.

    ros2 launch rover_gnss gnss_display.launch.py
    ros2 launch rover_gnss gnss_display.launch.py rviz:=false
    ros2 launch rover_gnss gnss_display.launch.py \
        params:=/abs/path/to/my_task.yaml

nmea_reader publishes /gnss_fix and /base_fix, which latlon_markers turns
into the /gnss_points MarkerArray. For a hardware-free version of the same
thing see latlon_markers_demo.launch.py.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

PKG = 'rover_gnss'


def generate_launch_description():
    share = get_package_share_directory(PKG)
    params = LaunchConfiguration('params')
    use_rviz = LaunchConfiguration('rviz')

    return LaunchDescription([
        DeclareLaunchArgument(
            'params',
            default_value=os.path.join(share, 'config', 'gnss_display.yaml'),
            description='params file holding the waypoints and labels'),
        DeclareLaunchArgument(
            'rviz', default_value='true',
            description='also start RViz with the marker config'),

        # real receiver: /gnss_fix (GGA) and /base_fix (EBP)
        Node(package=PKG, executable='nmea_reader', name='nmea_reader',
             parameters=[params], output='screen'),

        Node(package=PKG, executable='latlon_markers', name='latlon_markers',
             parameters=[params], output='screen'),

        Node(package='rviz2', executable='rviz2', name='rviz2',
             condition=IfCondition(use_rviz),
             arguments=['-d', os.path.join(share, 'config',
                                           'gnss_markers.rviz')]),
    ])
