# Brings up the mini rover drive: joystick, onboard CAN loop and the cmd_vel -> CAN translator.
#
#   ros2 launch drive_control bringup_mini_rover_drive.launch.py

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node


def generate_launch_description():
    can_loop_launch = os.path.join(
        get_package_share_directory("arm_hardware_interface"),
        "launch", "generic_can_interface_bringup_on_board.launch.py")

    return LaunchDescription([
        # For a controller plugged straight into this computer.
        Node(
            package="joy_linux",
            executable="joy_linux_node",
            name="joy_linux_node",
            output="screen",
        ),
		# /joy to /cmd_vel (same as main rover)
		Node(
		package='drive_control',
		executable='drive_control_node',
		name='drive_control_node',
		output='screen'
		),

        # Onboard CAN loop. Default loop_name is "main", so it listens on /can/main/outgoing.
        IncludeLaunchDescription(PythonLaunchDescriptionSource(can_loop_launch)),

        # Turns /cmd_vel into CAN messages on /can/main/outgoing.
        Node(
            package="arm_hardware_interface",
            executable="mini_rover_translator",
            name="mini_rover_translator",
            output="screen",
        ),
    ])
