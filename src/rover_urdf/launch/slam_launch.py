from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="slam_toolbox",
<<<<<<< HEAD
            executable="async_slam_toolbox_node",
=======
            executable="sync_slam_toolbox_node",
>>>>>>> e627b7e (Add odom and map transform lauch files, timestamp issue persists)
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": False,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "scan_topic": "/scan",
<<<<<<< HEAD
                "mode": "mapping",
                "transform_tolerance": 1,
                "transform_cache_size": 10000,
                "transform_publish_period": 0.05,
                "min_laser_range": 0.0,
                "max_laser_range": 25.0
=======
                "mode": "mapping"
>>>>>>> e627b7e (Add odom and map transform lauch files, timestamp issue persists)
            }]
        )
    ])
