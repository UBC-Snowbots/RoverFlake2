# Real-arm IK stack: servo_node + robot_state_publisher + RViz.
#
# Deliberately NOT launched here (vs dev_arm_moveit_bringup.launch.py):
#   - ros2_control / fake controllers / joint_state_broadcaster — the REAL
#     driver (moteus_driver, run separately) publishes /joint_states; a second
#     publisher would corrupt Servo's view of the arm.
#   - joy nodes — plug in whatever twist source you want (keyboard_servo_input,
#     joy_arm_control) in its own terminal.
#
# Run order (each its own terminal):
#   1. moteus_driver           (arm powered, homed via HMI HOME button)
#   2. this launch
#   3. servo_to_arm_bridge     (shadow by default; -p shadow:=false to go live)
#   4. keyboard_servo_input    (or a controller)

import os
import yaml
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, TimerAction
from ament_index_python.packages import get_package_share_directory
from moveit_configs_utils import MoveItConfigsBuilder


def load_yaml(package_name, file_path):
    absolute = os.path.join(get_package_share_directory(package_name), file_path)
    try:
        with open(absolute, "r") as f:
            return yaml.safe_load(f)
    except EnvironmentError:
        return None


def generate_launch_description():
    moveit_config = (
        MoveItConfigsBuilder("dev_arm", package_name="dev_arm_moveit_config_v3")
        .robot_description(file_path="config/dev_arm.urdf.xacro")
        .robot_description_semantic(file_path="config/dev_arm.srdf")
        .robot_description_kinematics(file_path="config/kinematics.yaml")
        .to_moveit_configs()
    )

    servo_yaml = load_yaml("dev_arm_moveit_config_v3", "config/rover_servo_params_dev_arm.yaml")
    servo_params = {"moveit_servo": servo_yaml}

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

    rsp_node = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[moveit_config.robot_description],
    )

    static_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=["0", "0", "0", "0", "0", "0", "world", "base_link"],
    )

    rviz_node = Node(
        package="rviz2",
        executable="rviz2",
        output="log",
        arguments=["-d", get_package_share_directory("moveit_servo") + "/config/demo_rviz_config.rviz"],
        parameters=[
            moveit_config.robot_description,
            moveit_config.robot_description_semantic,
            moveit_config.robot_description_kinematics,
        ],
    )

    start_servo = TimerAction(
        period=3.0,
        actions=[ExecuteProcess(
            cmd=["ros2", "service", "call", "/servo_node/start_servo", "std_srvs/srv/Trigger", "{}"],
            output="screen",
        )],
    )

    return LaunchDescription([servo_node, rsp_node, static_tf, rviz_node, start_servo])
