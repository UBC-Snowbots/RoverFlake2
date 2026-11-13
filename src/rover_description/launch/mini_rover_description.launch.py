"""Publish the mini rover URDF: robot_state_publisher only.

Publishes /robot_description, plus TF for every joint:
  - fixed joints (sensors, lidar_scan_link) -> /tf_static, always
  - moving joints (wheels, rockers, bogies, differential) -> /tf, only once
    something publishes /joint_states (joint_state_publisher_gui for display,
    the rover's driver / ros2_control on the real robot).

Include this from rover bringup so Nav2 etc. get base_link -> lidar_scan_link.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

URDF_FILE = os.path.join(
    get_package_share_directory("rover_description"), "urdf", "mini_rover.urdf"
)


def generate_launch_description():
    with open(URDF_FILE, "r") as f:
        robot_description = f.read()

    use_sim_time = LaunchConfiguration("use_sim_time")

    return LaunchDescription([
        DeclareLaunchArgument(
            "use_sim_time",
            default_value="false",
            description="Use /clock from a simulator",
        ),
        Node(
            package="robot_state_publisher",
            executable="robot_state_publisher",
            name="robot_state_publisher",
            output="screen",
            parameters=[{
                "robot_description": robot_description,
                "use_sim_time": use_sim_time,
            }],
        ),
    ])
