import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
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
    moveit_config = (
        MoveItConfigsBuilder("dev_arm")
        .robot_description(file_path="config/dev_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/dev_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .joint_limits(file_path="config/joint_limits.yaml")
        .to_moveit_configs()
    )

    # Get parameters for the Servo node
    servo_yaml = load_yaml("dev_arm_moveit_config", "config/rover_servo_params_dev_arm.yaml")
    servo_params = {"moveit_servo": servo_yaml}

    # RViz
    rviz_config_file = (
        get_package_share_directory("moveit_servo") + "/config/demo_rviz_config.rviz"
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        name="rviz2",
        output="log",
        arguments=["-d", rviz_config_file],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
        ],
    )

    # ros2_control using FakeSystem as hardware
    ros2_controllers_path = os.path.join(
        get_package_share_directory("dev_arm_moveit_config"),
        "config",
        "ros2_controllers.yaml",
    )
    ros2_control_node = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[moveit_config.robot_description, ros2_controllers_path],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        name="fake_joint_publisher",
        arguments=[
            "joint_state_broadcaster",
            "--controller-manager-timeout",
            "300",
            "--controller-manager",
            "/controller_manager",
        ],
        # remappings=[
        #     ('/joint_states', '/fake_joint_states'),
        # ]
    )

    dev_arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["dev_arm_controller", "-c", "/controller_manager"],
    )

    # Launch as much as possible in components
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            # Example of launching Servo as a node component
            # Assuming ROS2 intraprocess communications works well, this is a more efficient way.
            # ComposableNode(
            #     package="moveit_servo",
            #     plugin="moveit_servo::ServoServer",
            #     name="servo_server",
            #     parameters=[
            #         servo_params,
            #         moveit_config.robot_description,
            #         moveit_config.robot_description_semantic,
            #     ],
            # ),
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[moveit_config.robot_description],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[{"child_frame_id": "/link_0", "frame_id": "/world"}],
            ),
#            ComposableNode(
 #               package="moveit_servo",
  #              plugin="moveit_servo::JoyToServoPub",
   #             name="controller_to_servo_node",
    #            parameters=[moveit_config.robot_description],
     #       ),
            # ComposableNode(
            #     package="joy_linux",
            #     plugin="joy_linux::Joy",
            #     name="joy_node",
            #     parameters=[{"autorepeat_rate": 100.0, "coalesce_interval": 100}],
            # ),
        ],
        output="screen",
    )
    # Launch a standalone Servo node.
    # As opposed to a node component, this may be necessary (for example) if Servo is running on a different PC
    servo_node = Node(
        package="moveit_servo",
        executable="servo_node_main",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )
    custom_servo_node = Node(
        package="arm_control",
        executable="moveit_control",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ]
    )
    joy_params = {
        # 'dev': '/dev/input/js0',       # Joystick device file
        'deadzone': 0.05,              # Deadzone for joystick axes
        'autorepeat_rate': 100.0,       # Autorepeat rate in Hz
        'coalesce_interval': 0.01,    # Interval to coalesce events
    }
    # joy_node = Node(
    #     package='joy_linux',
    #     executable='joy_linux_node',  # Replace with the correct executable name if different
    #     name='joy_node',
    #     output='screen',
    #     parameters=[joy_params],
    #     remappings=[
    #         ('/joy', '/joy'),  # Remap topics if necessary
    #     ],
    # )

    return LaunchDescription(
        [
            # joy_node,
            rviz_node,
            ros2_control_node,
            # joint_state_broadcaster_spawner,
            dev_arm_controller_spawner,
      #      servo_node,
            custom_servo_node,
            container,
        ]
    )
