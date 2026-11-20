from launch import LaunchDescription
from launch_ros.actions import Node


# Joystick arm control: joy_linux_node -> /joy -> joy_arm_control -> /arm/command.
# Started/stopped as one subsystem ("arm_joy") by the arm HMI's heart, see arm_hmi.launch.py.
def generate_launch_description():
    joy = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='arm_joy_linux_node',
        output='screen',
        parameters=[{
            # 'dev': '/dev/input/js0',
            'deadzone': 0.06,
            'autorepeat_rate': 100.0,
            'coalesce_interval': 0.01,
        }],
    )

    arm_joy = Node(
        package='arm_control',
        executable='joy_arm_control',
        name='arm_joystick_controller',
        output='screen',
    )

    return LaunchDescription([joy, arm_joy])
