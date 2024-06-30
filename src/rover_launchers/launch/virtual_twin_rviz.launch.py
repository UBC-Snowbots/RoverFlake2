import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
# from launch import LaunchConfiguration
from ament_index_python.packages import get_package_share_directory
from launch_ros.descriptions import ComposableNode
from launch.actions import ExecuteProcess
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from xacro import process_file

def xacro_to_urdf(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    robot_description = process_file(absolute_file_path).toxml()
    return robot_description

def load_file(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return file.read()
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None

def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)

    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:  # parent of IOError, OSError *and* WindowsError where available
        return None



def generate_launch_description():
    rviz_config_file = (
        get_package_share_directory("rover_hmi") + "/config/virtual_twin.rviz"
    )

    urdf_file_path = (
        get_package_share_directory("rover_urdf") + "/urdf/chassis_urdf_24_rviz.urdf"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="screen",
        arguments=["-d", rviz_config_file],
    )

    rover_virtual_twin = Node(
        package='rover_hmi',
        executable='middle_rover_viz_node',
        name='rover_virtual_twin',
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
            parameters=[{'robot_description': xacro_to_urdf("rover_urdf", urdf_file_path) }]
        )

    return LaunchDescription([
        rviz_node,
        full_urdf_publisher,
        joint_state_broadcaster_spawner,
        joint_state_publisher_gui_node,
        # rover_virtual_twin,
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            arguments=["0", "0", "0", "0", "0", "0", "/world", "/base_link"],
            output="screen",
        ),
    ])
