import launch
from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='realsense2_camera',
            executable='realsense2_camera_node',
            name='realsense2_camera',
            output='screen',
            parameters=[{
                'enable_color': True,
                'enable_infra1': True,
                'enable_infra2': True,
                'rgb_camera.profile': '640,480,30',
                'infra_width': 640,
                'infra_height': 480,
                'infra_fps': 30
            }]
        )
    ])