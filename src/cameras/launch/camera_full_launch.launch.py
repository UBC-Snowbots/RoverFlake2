#!/usr/bin/env python3

"""
Created by: Cameron Basara
Date: , 2024
Purpose: Launch file for a generic USB camera feed, reading from topic /usb_cam
"""

import launch
from launch import LaunchDescription
from launch_ros.actions import Node as LaunchNode
from launch.actions import OpaqueFunction

import rclpy
from rclpy.node import Node

def list_topics():
    rclpy.init()
    node = Node('temp_node')

    # Retrieve image topics
    topics = node.get_topic_names_and_types()
    node.get_logger().info(f'Topics detected: {topics}')

    image_topic_types = [
        'sensor_msgs/msg/Image',
        'sensor_msgs/msg/CompressedImage',
        'sensor_msgs/msg/CameraInfo',
        'camera/infra1/image_rect_raw',
        'camera/infra2/image_rect_raw',
        'camera/color/image_raw',
        'camera/depth/image_rect_raw'
    ]

    # Retrieve any active image or camera feed
    topic_list = [topic for topic, types in topics if any(t in image_topic_types for t in types)]

    if len(topic_list) == 0:
        node.get_logger().info('No active camera topics found.')

    for topic in topic_list:
        node.get_logger().info(f'Active camera topics: {topic}')
    
    node.destroy_node()
    rclpy.shutdown()

    return topic_list

def launch_setup(context, *args, **kwargs):
    # Get camera topics
    camera_topics = list_topics()
    
    # Create an image_view node for each camera topic
    launch_entities = []
    for i, topic in enumerate(camera_topics):
        image_view_node = LaunchNode(
            package='image_tools',
            executable='showimage',
            name=f'image_view_{i}',
            output='screen',
            remappings=[
                ('image', topic)  # Adjust the topic for each camera
            ],
        )
        launch_entities.append(image_view_node)
    
    return launch_entities

def generate_launch_description():
    return LaunchDescription([
        OpaqueFunction(function=launch_setup)
    ])
