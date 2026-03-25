import os
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

## PARAMS / CONSTANTS
SHARED_DIR = get_package_share_directory("rover_description")
URDF_PATH  = os.path.join(SHARED_DIR, "urdf")
CFG_PATH   = os.path.join(SHARED_DIR, "config")


def generate_launch_description():
    rviz_config_file = os.path.join(CFG_PATH, "full_urdf_display.rviz")
    urdf_file_path = os.path.join(URDF_PATH, "Chassis_jan25.SLDASM.urdf")

    with open(urdf_file_path, 'r') as f:
        robot_description_content = f.read()

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
    )
    joint_state_publisher_gui_node = Node(
        package="joint_state_publisher_gui",
        executable="joint_state_publisher_gui",
        name="joint_state_publisher_gui",
    )

    full_urdf_publisher = Node(
        package='robot_state_publisher',
        executable='robot_state_publisher',
        name='robot_state_publisher',
        output='screen',
        parameters=[{'robot_description': robot_description_content}],
    )

    return LaunchDescription([
        rviz_node,
        full_urdf_publisher,
        joint_state_publisher_gui_node,
    ])
