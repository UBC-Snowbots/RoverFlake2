from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    """Launch the SSH source and network telemetry processor."""
    ssh_reader = Node(
        package='network_telemetry',
        executable='ssh_network_reader',
        name='ssh_network_reader',
        output='screen',
        parameters=[{
            'host': LaunchConfiguration('host'),
            'port': LaunchConfiguration('port'),
            'username': LaunchConfiguration('username'),
            'password': LaunchConfiguration('password'),
            'ssh_key': LaunchConfiguration('ssh_key'),
            'command': LaunchConfiguration('command'),
            'timeout_seconds': LaunchConfiguration('timeout_seconds'),
            'poll_interval_seconds': LaunchConfiguration(
                'poll_interval_seconds'
            ),
            'output_topic': LaunchConfiguration('raw_topic'),
        }],
    )

    processor = Node(
        package='network_telemetry',
        executable='network_reader',
        name='network_telemetry',
        output='screen',
        parameters=[{
            'input_topic': LaunchConfiguration('raw_topic'),
            'output_topic': LaunchConfiguration('telemetry_topic'),
        }],
    )

    return LaunchDescription([
        DeclareLaunchArgument(
            'host',
            description='Ubiquiti device hostname or IP address.',
        ),
        DeclareLaunchArgument(
            'port',
            default_value='22',
            description='SSH port.',
        ),
        DeclareLaunchArgument(
            'username',
            default_value='ubnt',
            description='SSH username.',
        ),
        DeclareLaunchArgument(
            'password',
            default_value='',
            description='SSH password. Prefer ssh_key when possible.',
        ),
        DeclareLaunchArgument(
            'ssh_key',
            default_value='',
            description='Path to the SSH private key.',
        ),
        DeclareLaunchArgument(
            'command',
            default_value='cat /tmp/network.json',
            description='Remote command that prints Ubiquiti JSON.',
        ),
        DeclareLaunchArgument(
            'timeout_seconds',
            default_value='10.0',
            description='SSH command timeout.',
        ),
        DeclareLaunchArgument(
            'poll_interval_seconds',
            default_value='5.0',
            description='Seconds between SSH telemetry reads.',
        ),
        DeclareLaunchArgument(
            'raw_topic',
            default_value='network/raw',
            description='Topic carrying raw JSON from the SSH reader.',
        ),
        DeclareLaunchArgument(
            'telemetry_topic',
            default_value='network/telemetry',
            description='Topic carrying normalized telemetry.',
        ),
        ssh_reader,
        processor,
    ])
