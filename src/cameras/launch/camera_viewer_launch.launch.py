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

    # Declare launch arguments for camera topics
    camera_topics = [
        '/usb_cam',
        'red_mask_topic',
        'redmask_detection_topic',  # Add as many camera topics as needed
    ]

    # Create an image_view node for each camera topic
    for i, topic in enumerate(camera_topics):
        image_view_node = Node(
            package='image_tools',
            executable='showimage',
            name=f'image_view_{i}',
            output='screen',
            remappings=[ 
                ('image', topic)  # Adjust the topic for each camera
            ],
        )
        ld.add_action(image_view_node)

    # Return the LaunchDescription object
    return ld
