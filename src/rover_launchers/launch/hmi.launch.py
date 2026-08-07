from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# Single parameterized HMI entry point: pick a saved layout by name, or an
# explicit comma-separated panel list. Both empty = default tiling.
def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('layout', default_value='',
                              description='Saved layout name to load at startup'),
        DeclareLaunchArgument('panels', default_value='',
                              description='Comma-separated panel titles to show'),
        Node(
            package='rover_hmi_core',
            executable='rover_hmi',
            name='rover_hmi',
            parameters=[{
                'layout': LaunchConfiguration('layout'),
                'panels': LaunchConfiguration('panels'),
            }],
        ),
    ])
