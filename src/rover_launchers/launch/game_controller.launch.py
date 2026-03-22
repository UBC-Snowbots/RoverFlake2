from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_linux_params = {
    # 'dev': '/dev/input/js0',       # Joystick device file
    'deadzone': 0.06,              # Deadzone for joystick axes
    'autorepeat_rate': 100.0,       # Autorepeat rate in Hz
    'coalesce_interval': 0.01,    # Interval to coalesce events in s
    }

    joy_params = {
    # 'dev': '/dev/input/js0',       # Joystick device file
    'deadzone': 0.05,              # Deadzone for joystick axes
    'autorepeat_rate': 100.0,       # Autorepeat rate in Hz
    'coalesce_interval_ms': 10,    # Interval to coalesce events in ms
    }

    joy_linux_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',
        parameters=[joy_linux_params],
        remappings=[
            ('/joy', '/joy'),  # Remap topics if necessary
        ],
    )

    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        output='screen',
        parameters=[joy_params],
        remappings=[
            ('/joy', '/joy'),  # Remap topics if necessary
        ],
    )
    return LaunchDescription([
        # Ensure only one is active:
        joy_linux_node # Works with most joysticks, and accepts parameters
        # joy_node # theres a bug where this just doesnt accept parameters. However we need this one for switch controller
    ])
