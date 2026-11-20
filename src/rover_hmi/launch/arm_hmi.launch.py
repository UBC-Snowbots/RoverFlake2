import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import Shutdown
from launch_ros.actions import Node


# Arm HMI (main_hmi_node) + a dedicated heart that owns the arm processes.
# The HMI's "Arm Driver" / "Joy Control" buttons ask this heart to start/stop
# the subsystems listed in config/arm_hmi.yaml (stepper_arm_driver, arm_joy).
# Closing the HMI window shuts the whole launch down, and the heart kills its
# subsystems on exit.
def generate_launch_description():
    config = os.path.join(get_package_share_directory('rover_hmi'), 'config', 'arm_hmi.yaml')

    heart = Node(
        package='rover_manager',
        executable='heart_node',
        name='heart_arm_hmi',  # must match the key in arm_hmi.yaml
        output='screen',
        parameters=[config],
    )

    hmi = Node(
        package='rover_hmi',
        executable='main_hmi_node',
        name='main_hmi_node',
        output='screen',
        parameters=[config],
        on_exit=Shutdown(reason='arm HMI closed'),
    )

    return LaunchDescription([heart, hmi])
