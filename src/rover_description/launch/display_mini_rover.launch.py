"""View the mini rover in RViz with joint sliders.

Runs mini_rover_description.launch.py + joint_state_publisher_gui + RViz.
The GUI publishes fake /joint_states. Don't use this on the real rover,
where the driver publishes joint states (use mini_rover_description.launch.py there).

Only RockerL_joint gets a slider for the rocker system; RockerR and the
differential follow it via <mimic>.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node

SHARE_DIR = get_package_share_directory("rover_description")


def generate_launch_description():
    rviz_config = os.path.join(SHARE_DIR, "config", "full_urdf_display.rviz")

    return LaunchDescription([
        DeclareLaunchArgument(
            "gui",
            default_value="true",
            description="Run joint_state_publisher_gui (sliders) to move the joints",
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(SHARE_DIR, "mini_rover_description.launch.py")
            ),
        ),
        Node(
            package="joint_state_publisher_gui",
            executable="joint_state_publisher_gui",
            name="joint_state_publisher_gui",
            condition=IfCondition(LaunchConfiguration("gui")),
        ),
        Node(
            package="rviz2",
            executable="rviz2",
            name="rviz2",
            output="log",
            arguments=["-d", rviz_config],
        ),
    ])
