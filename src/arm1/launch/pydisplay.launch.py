import os
from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define the path to the URDF file
    pkg_share = FindPackageShare('arm1').find('arm1')
    urdf_file_path = os.path.join(pkg_share, 'urdf', 'arm1.urdf')

    # Read the URDF file content
    with open(urdf_file_path, 'r') as file:
        robot_description_content = file.read()

    return LaunchDescription([
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui'
        ),
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_description_content}]
        ),
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', os.path.join(pkg_share, 'urdf.rviz')]
        ),
    ])

