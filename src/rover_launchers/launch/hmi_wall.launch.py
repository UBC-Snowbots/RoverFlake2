from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


# HMI wall: one window per CBS monitor. Monitor binding is i3 config
# (setup_scripts/i3/rover_hmi.conf) keyed on the [instance] title prefix.
def generate_launch_description():
    instances = ['left', 'center', 'right']
    actions = []
    for inst in instances:
        actions.append(DeclareLaunchArgument(
            inst, default_value='',
            description=f'Startup layout for the {inst} window ("" = default tiling)'))
    actions.append(DeclareLaunchArgument(
        'wall', default_value='',
        description='Wall snapshot to restore on startup ("" = none)'))
    for inst in instances:
        actions.append(Node(
            package='rover_hmi_core',
            executable='rover_hmi',
            name=f'rover_hmi_{inst}',
            parameters=[{
                'instance': inst,
                'layout': LaunchConfiguration(inst),
                'wall': LaunchConfiguration('wall'),
            }],
        ))
    return LaunchDescription(actions)
