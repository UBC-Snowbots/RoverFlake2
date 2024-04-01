"""
Created by: Cameron Basara
Date: March 31, 2024
Purpose: Launch file for a generic USB camera feed, reading from topic /usb_cam
"""
#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    ld = LaunchDescription()

    # # Declare the 'videonum' launch argument (if needed for identifying the camera)
    # videonum_arg = DeclareLaunchArgument(
    #     'videonum',
    #     default_value='0',
    #     description='Video device number.'
    # )

    # Define the 'image_view' node using 'showimage' from 'image_tools'
    image_view_node = Node(
        package='image_tools',
        executable='showimage',
        name='image_view',
        output='screen',
        remappings=[
            ('image', '/usb_cam')  # Adjust the topic if necessary
        ],
    )
    ld.add_action(image_view_node)

    # Return the LaunchDescription object
    return ld