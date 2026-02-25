"""
Launch file for running IK (MoveIt Servo) with the REAL old arm hardware.

Based on dev_arm_moveit_bringup.launch.py but:
  - REMOVES ros2_control FakeSystem (ros2_control_node + joint_state_broadcaster)
    because the real ArmSerialInterface node publishes /joint_states directly.
  - REMOVES arm_controller spawner (no fake controller manager needed).
  - KEEPS robot_state_publisher, MoveIt Servo (custom moveit_control node), RViz,
    and the static TF broadcaster.
  - INCLUDES the arm_serial_driver node for hardware communication.
  - INCLUDES the HMI node for IK button control.

Usage:
  ros2 launch dev_arm_moveit_config_v2 real_arm_ik_bringup.launch.py
"""

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
from launch_ros.actions import ComposableNodeContainer
from launch_ros.descriptions import ComposableNode
import xacro
from moveit_configs_utils import MoveItConfigsBuilder
from xacro import process_file


def load_yaml(package_name, file_path):
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, "r") as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None


def generate_launch_description():
    # Use the v2 MoveIt config (old arm, joint_1..joint_6)
    moveit_config = (
        MoveItConfigsBuilder("dev_arm", package_name="dev_arm_moveit_config_v2")
        .robot_description(file_path="config/dev_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/dev_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    # Servo parameters
    servo_yaml = load_yaml(
        "dev_arm_moveit_config_v2", "config/rover_servo_params_dev_arm.yaml"
    )
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
            moveit_config.robot_description_kinematics,
        ],
    )

    # Robot state publisher + static TF (no FakeSystem / ros2_control)
    container = ComposableNodeContainer(
        name="moveit_servo_demo_container",
        namespace="/",
        package="rclcpp_components",
        executable="component_container_mt",
        composable_node_descriptions=[
            ComposableNode(
                package="robot_state_publisher",
                plugin="robot_state_publisher::RobotStatePublisher",
                name="robot_state_publisher",
                parameters=[
                    moveit_config.robot_description,
                    moveit_config.robot_description_semantic,
                    moveit_config.robot_description_kinematics,
                ],
            ),
            ComposableNode(
                package="tf2_ros",
                plugin="tf2_ros::StaticTransformBroadcasterNode",
                name="static_tf2_broadcaster",
                parameters=[
                    {"child_frame_id": "/link_0", "frame_id": "/world"}
                ],
            ),
        ],
        output="screen",
    )

    # Custom servo node (moveit_control) — runs MoveIt Servo internally,
    # subscribes to /arm_moveit_control/delta_twist_cmds (IK),
    # and bridges servo output to /arm/command for the real arm.
    custom_servo_node = Node(
        package="arm_control",
        executable="moveit_control",
        parameters=[
            servo_params,
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
        output="screen",
    )

    # Real arm hardware interface — publishes /joint_states, listens on /arm/command
    arm_hardware_interface_node = Node(
        package="arm_hardware_interface",
        executable="arm_serial_driver",
        name="arm_serial_driver",
        output="screen",
    )

    # Joystick node (for joystick IK control via moveit_control)
    joy_params = {
        "deadzone": 0.05,
        "autorepeat_rate": 100.0,
        "coalesce_interval": 0.01,
    }
    joy_node = Node(
        package="joy_linux",
        executable="joy_linux_node",
        name="joy_node",
        output="screen",
        parameters=[joy_params],
    )

    # HMI node (GUI with IK buttons)
    hmi_node = Node(
        package="rover_hmi",
        executable="main_hmi_node",
        name="hmi_node",
        output="screen",
    )

    return LaunchDescription(
        [
            # Core: robot model + TF
            container,
            # Visualization
            rviz_node,
            # MoveIt Servo (IK) + bridge to /arm/command
            custom_servo_node,
            # Real arm hardware
            arm_hardware_interface_node,
            # Input devices
            joy_node,
            hmi_node,
            # NOTE: NO ros2_control_node, NO joint_state_broadcaster, NO arm_controller spawner.
            # The real arm's ArmSerialInterface publishes /joint_states directly.
        ]
    )
