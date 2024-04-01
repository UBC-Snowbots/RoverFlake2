#!/usr/bin/env python3

from launch import LaunchDescription
from launch_ros.actions import Node 

# Defining python node
def generate_launch_description():
    ld = LaunchDescription()
    
    # Data Parsing Node
    usbParserNode = Node(
            package='usb_parser2',
            executable='usb_parser_node',
            name='usbParserNode',
            output='both',
        )
        
    ld.add_action(usbParserNode)
    return ld
    
