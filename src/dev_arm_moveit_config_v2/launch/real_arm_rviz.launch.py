"""
Plan in RViz, execute on the REAL old stepper arm.

  RViz -> move_group -> arm_controller -> topic_based_ros2_control
       -> arm_velocity_translator -> stepper_arm_driver -> USB -> Teensy

arm_controller starts INACTIVE so nothing moves until you home the arm and turn it on:
  ros2 control switch_controllers --activate arm_controller

Usage:
  ros2 launch dev_arm_moveit_config_v2 real_arm_rviz.launch.py
"""
from launch import LaunchDescription
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder
from moveit_configs_utils.launches import (
    generate_move_group_launch,
    generate_moveit_rviz_launch,
    generate_rsp_launch,
    generate_static_virtual_joint_tfs_launch,
)


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dev_arm", package_name="dev_arm_moveit_config_v2")
        .robot_description(file_path="config/dev_arm_real.urdf.xacro")
        .to_moveit_configs()
    )

    controller_manager = Node(
        package="controller_manager",
        executable="ros2_control_node",
        parameters=[str(moveit_config.package_path / "config/ros2_controllers_real.yaml")],
        remappings=[("/controller_manager/robot_description", "/robot_description")],
        output="screen",
    )

    joint_state_broadcaster_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["joint_state_broadcaster"],
    )

    arm_controller_spawner = Node(
        package="controller_manager",
        executable="spawner",
        arguments=["arm_controller", "--inactive"],
    )

    stepper_arm_driver = Node(
        package="arm_hardware_interface",
        executable="stepper_arm_driver",
        output="screen",
        # The broadcaster owns /joint_states, so the driver's raw readings go here instead
        remappings=[("/joint_states", "/arm/joint_states")],
    )

    translator = Node(
        package="arm_control",
        executable="arm_velocity_translator.py",
        output="screen",
    )

    return LaunchDescription(
        [
            generate_static_virtual_joint_tfs_launch(moveit_config),
            generate_rsp_launch(moveit_config),
            generate_move_group_launch(moveit_config),
            generate_moveit_rviz_launch(moveit_config),
            controller_manager,
            joint_state_broadcaster_spawner,
            arm_controller_spawner,
            stepper_arm_driver,
            translator,
        ]
    )
