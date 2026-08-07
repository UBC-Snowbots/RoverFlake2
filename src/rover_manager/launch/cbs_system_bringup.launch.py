import os
import launch
import launch_ros
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import ThisLaunchFileDir
from ament_index_python.packages import get_package_share_directory

# Dont start the heart from here, heart is daemonized

def generate_launch_description():
    # sensor_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('rover_sounds'),
    #         '/launch/sensor_launch.py'
    #     ])
    # )

    speaker = Node(
        package='rover_sounds',
        executable='speaker_node',  # The actual C++ binary
        name='rover_speaker'
        # parameters=[param_file]
    )

    cbs_device_hardware_manager = Node(
        package='rover_manager',
        executable='cbs_hardware_manager_node',
        name='rover_cbs_hardware_manager'

    )
    cyborg_joystick_node = Node(
        package='joy',
        executable='joy_node',
        name='cyborg_joystick_interface'
    )

    hmi_router = Node(
        package='rover_manager',
        executable='router_a',
        name='hmi_router',
        parameters=[os.path.join(
            get_package_share_directory('rover_manager'),
            'config', 'cbs_button_map.yaml')]
    )

    # navigation_launch = IncludeLaunchDescription(
    #     PythonLaunchDescriptionSource([
    #         get_package_share_directory('my_robot_package'),
    #         '/launch/navigation_launch.py'
    #     ])
    # )

    return LaunchDescription([
        speaker,
        cbs_device_hardware_manager,
        cyborg_joystick_node,
        hmi_router
        # sensor_launch,
        # navigation_launch
    ])
