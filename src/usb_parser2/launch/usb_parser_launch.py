#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node 

# Defining python node
def generate_launch_description():
    ld = LaunchDescription()
    
    # Add more nodes as needed
    parserNode = Node(
            package='usb_parser.scripts',
            executable='data_parsing',
            name='data_parsing',
            output='both',
        )
        
    ld.add_action(parserNode)
    return ld
    
