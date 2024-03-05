from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='rover_qt_gui',
            executable='arm_qt',
            name='arm_display_panel'
          #  parameters=[{'dev': '/dev/input/eventX'}]
        ),
    ])
