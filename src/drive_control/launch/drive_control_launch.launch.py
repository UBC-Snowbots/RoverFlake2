# Importing necessary modules from the ROS2 package
from launch import LaunchDescription
from launch_ros.actions import Node

#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='joy',
            executable='joy_node',
            name='joy_node',
            output='screen'
        ),
        Node(
            package='drive_control',
            executable='drive_node',
            name='drive_node',
            output='screen'
        )
    ])
