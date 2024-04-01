"""
Created by: Cameron Basara
Date: March 31, 2024
Purpose: Launch file for the Realsense D435i camera
"""
#!/usr/bin/env python3

import launch
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
import os

def generate_launch_description():
    # Declare launch arguments
    camera_name_arg = DeclareLaunchArgument('camera', default_value='cam_2')
    serial_no_arg = DeclareLaunchArgument('serial_no', default_value='116622072291')

    # Realsense camera node with configurations and remappings
    camera_node = Node(
        package='realsense2_camera',
        executable='realsense2_camera_node',
        namespace=LaunchConfiguration('camera'),
        parameters=[{
            'serial_no': LaunchConfiguration('serial_no'),
            'enable_gyro': True,
            'enable_accel': True,
            'enable_infra1': True,
            'enable_infra2': True,
            'unite_imu_method': 'linear_interpolation',
            'infra_width': 848,
            'infra_height': 480,
            'infra_fps': 30,
        }],
        remappings=[
            # Add additional remappings as needed
        ]
    )

    # Image viewer node using ExecuteProcess for rqt_image_view
    image_view_node = Node(
        package='rqt_image_view',
        executable='rqt_image_view',
        arguments=[os.path.join(LaunchConfiguration('camera'), 'color/image_raw')],
    )

    # Launch description with all actions
    ld = LaunchDescription()

    ld.add_action(camera_name_arg)
    ld.add_action(serial_no_arg)
    ld.add_action(camera_node)
    ld.add_action(image_view_node)

    return ld
