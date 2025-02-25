from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package="slam_toolbox",
<<<<<<< HEAD
<<<<<<< HEAD
            executable="async_slam_toolbox_node",
=======
            executable="sync_slam_toolbox_node",
>>>>>>> e627b7e (Add odom and map transform lauch files, timestamp issue persists)
=======
            executable="async_slam_toolbox_node",
>>>>>>> aa6d646 (Required for nav2 atm)
            name="slam_toolbox",
            output="screen",
            parameters=[{
                "use_sim_time": True,
                "map_frame": "map",
                "odom_frame": "odom",
                "base_frame": "base_link",
                "scan_topic": "/scan",
<<<<<<< HEAD
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
=======
                "mode": "mapping",
                "transform_tolerance": 3,
                "transform_cache_size": 1000,
                "transform_publish_period": 0.0,
                "min_laser_range": 1.0,
                "max_laser_range": 10.0
>>>>>>> aa6d646 (Required for nav2 atm)
            }]
        )
    ])
