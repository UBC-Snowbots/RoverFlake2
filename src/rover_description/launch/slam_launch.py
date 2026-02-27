from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    # Fake odom -> base_link (identity transform so SLAM can run without EKF)
    static_odom_tf = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base",
        arguments=["0", "0", "0", "0", "0", "0", "odom", "base_link"],
    )

    slam_node = Node(
        package="slam_toolbox",
        executable="async_slam_toolbox_node",
        name="slam_toolbox",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "map_frame": "map",
            "odom_frame": "odom",
            "base_frame": "base_link",
            "scan_topic": "/scan",
            "mode": "mapping",
        }]
    )

    return LaunchDescription([
        static_odom_tf,
        slam_node,
    ])
