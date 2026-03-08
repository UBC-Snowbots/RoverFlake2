from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    joy_params = {
    # 'dev': '/dev/input/js0',       # Joystick device file
    'deadzone': 0.05,              # Deadzone for joystick axes
    'autorepeat_rate': 100.0,       # Autorepeat rate in Hz
    'coalesce_interval': 0.01,    # Interval to coalesce events
    }

    joy_node = Node(
        package='joy_linux',
        executable='joy_linux_node',
        name='joy_node',
        output='screen',
        parameters=[joy_params],
        remappings=[
            ('/joy', '/joy'),  # Remap topics if necessary
        ],
    )
    return LaunchDescription([
        joy_node
    ])
