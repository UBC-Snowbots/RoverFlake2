# Brings up everything the arm needs on board: the CAN loop, the arm hardware
# nodes and the lighting translator.
#
#   ros2 launch arm_hardware_interface launch_arm_common.launch.py
#
# The CAN arguments are forwarded to generic_can_interface_bringup_on_board, ie
#
#   ros2 launch arm_hardware_interface launch_arm_common.launch.py \
#       can_interface:=can1 bring_up_interface:=true

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration


def generate_launch_description():
    launch_dir = os.path.join(
        get_package_share_directory('arm_hardware_interface'), 'launch')

    default_can_config = os.path.join(
        get_package_share_directory('arm_hardware_interface'),
        'config',
        'can_loop_on_board.yaml')

    default_lighting_config = os.path.join(
        get_package_share_directory('rover_lighting'),
        'config',
        'lighting_translator.yaml')

    args = [
        DeclareLaunchArgument(
            'loop_name', default_value='main',
            description="Topics are published under /can/<loop_name>/"),
        DeclareLaunchArgument(
            'can_interface', default_value='can0',
            description='SocketCAN interface name, ie can0'),
        DeclareLaunchArgument(
            'config_file', default_value=default_can_config,
            description="Path to the CAN loop's yaml config"),
        DeclareLaunchArgument(
            'bring_up_interface', default_value='false',
            description='Run can_bringup.sh before starting the node. Needs passwordless sudo.'),
        DeclareLaunchArgument(
            'lighting_config', default_value=default_lighting_config,
            description='Parameter file for the lighting translator'),
    ]

    can_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'generic_can_interface_bringup_on_board.launch.py')),
        launch_arguments={
            'loop_name': LaunchConfiguration('loop_name'),
            'can_interface': LaunchConfiguration('can_interface'),
            'config_file': LaunchConfiguration('config_file'),
            'bring_up_interface': LaunchConfiguration('bring_up_interface'),
        }.items(),
    )

    arm_hardware = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'arm_hardware_bringup.launch.py')))

    lighting_translator = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(launch_dir, 'lighting_translator.launch.py')),
        launch_arguments={
            'config': LaunchConfiguration('lighting_config'),
        }.items(),
    )

    return LaunchDescription(args + [can_bringup, arm_hardware, lighting_translator])
