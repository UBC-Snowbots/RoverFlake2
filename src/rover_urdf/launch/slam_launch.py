from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="slam_toolbox",
            executable="async_slam_toolbox_node",
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "scan_topic": "/scan",
                "mode": "mapping",
                "transform_tolerance": 3,
                "transform_cache_size": 1000,
                "transform_publish_period": 0.0,
                "min_laser_range": 1.0,
                "max_laser_range": 10.0
            }]
        )
    ])
