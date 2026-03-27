from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    arm_driver = Node(
        package='arm_hardware_interface',
        executable='moteus_driver',
        name='moteus_driver',
        output='screen'
    )
    arm_joy = Node(
        package='arm_control',
        executable='joy_arm_control',
        name='arm_joystick_controller',
        output='screen'
    )

    return LaunchDescription([arm_driver, arm_joy])
