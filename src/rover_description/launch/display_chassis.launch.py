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
    # moveit_config = (
    #     MoveItConfigsBuilder("arm2")
    #     .robot_description(file_path="config/arm2.urdf.xacro")
    #     #.robot_description_semantics(file_path"config/arm1.srdf")
    #     .robot_description_kinematics(file_path="config/kinematics.yaml")
    #     .to_moveit_configs()
    # )

    # Get parameters for the Servo node
   # servo_yaml = load_yaml("rover_urdfs", "config/rover_servo_params.yaml")
    #servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = (
        get_package_share_directory("rover_urdf") + "/config/full_urdf_display.rviz"
    )

    urdf_file_path = (
        get_package_share_directory("rover_urdf") + "/urdf/chassis_urdf_24_rviz.urdf"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        # arguments=["-d", rviz_config_file],
    )

       # ros2_control, if we need it
    # ros2_controllers_path = os.path.join(
    #     get_package_share_directory("arm2_moveit_config"),
    #     "config",
    #     "ros2_controllers.yaml",
    # )
    # ros2_control_node = Node(
    #     package="controller_manager",
    #     executable="ros2_control_node",
    #     parameters=[moveit_config.robot_description, ros2_controllers_path],
    #     output="screen",
    # )

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
        joint_state_publisher_gui_node

    ])