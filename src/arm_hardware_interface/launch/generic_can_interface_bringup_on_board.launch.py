# Launches one CAN loop: optionally brings the interface up, then starts the node.
#
#   ros2 launch rover_can can_loop.launch.py loop_name:=drive can_interface:=can0
#
# To launch more than one loop, include this file once per loop from your top
# level launch file, with a different loop_name / can_interface / config_file.

import os

import yaml

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, OpaqueFunction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def build_can_setup_command(script_path, interface, config):
    """Turns the 'link' section of the config file into can_bringup.sh arguments."""
    link = config.get("link", {})

    cmd = [script_path, "--interface", str(interface)]
    cmd += ["--bitrate", str(link.get("bitrate", 500000))]

    if "sample_point" in link:
        cmd += ["--sample-point", str(link["sample_point"])]

    if config.get("enable_can_fd", False):
        cmd += ["--fd"]
        if "data_bitrate" in link:
            cmd += ["--data-bitrate", str(link["data_bitrate"])]
        if "data_sample_point" in link:
            cmd += ["--data-sample-point", str(link["data_sample_point"])]

    if "restart_ms" in link:
        cmd += ["--restart-ms", str(link["restart_ms"])]
    if "txqueuelen" in link:
        cmd += ["--txqueuelen", str(link["txqueuelen"])]

    return cmd


def launch_setup(context, *args, **kwargs):
    loop_name = LaunchConfiguration("loop_name").perform(context)
    can_interface = LaunchConfiguration("can_interface").perform(context)
    config_file = LaunchConfiguration("config_file").perform(context)
    bring_up_interface = LaunchConfiguration("bring_up_interface").perform(context).lower() == "true"

    # The node reads this file itself, we only peek at it so the link settings
    # and the node settings stay in one place.
    with open(config_file, "r") as f:
        config = yaml.safe_load(f)["/**"]["ros__parameters"]

    actions = []

    if bring_up_interface:
        script_path = os.path.join(
            get_package_share_directory("rover_can"), "launch", "can_bringup.sh")
        actions.append(ExecuteProcess(
            cmd=build_can_setup_command(script_path, can_interface, config),
            output="screen"))

    actions.append(Node(
        package="rover_can",
        executable="can_node",
        # Node name is unique per loop so two loops can run side by side.
        name="can_node_" + loop_name,
        output="screen",
        parameters=[
            config_file,
            # Launch arguments win over the config file.
            {"loop_name": loop_name,
             "can_interface": can_interface},
        ],
        # If the USB adapter gets unplugged the node dies - bring it back.
        respawn=True,
        respawn_delay=2.0,
    ))

    return actions


def generate_launch_description():
    default_config = os.path.join(
        get_package_share_directory("rover_can"), "config", "can_loop.yaml")

    return LaunchDescription([
        DeclareLaunchArgument(
            "loop_name", default_value="main",
            description="Topics are published under /can/<loop_name>/"),
        DeclareLaunchArgument(
            "can_interface", default_value="can0",
            description="SocketCAN interface name, ie can0"),
        DeclareLaunchArgument(
            "config_file", default_value=default_config,
            description="Path to the loop's yaml config"),
        DeclareLaunchArgument(
            "bring_up_interface", default_value="false",
            description="Run can_setup.sh before starting the node. Needs passwordless sudo."),

        OpaqueFunction(function=launch_setup),
    ])