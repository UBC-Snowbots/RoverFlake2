from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():
    
    # Navsat transform node (GPS -> Odometry)
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        parameters=[{
            'frequency': 10.0,
            'delay': 3.0,
            'magnetic_declination_radians': 0.0,  # Set for your location
            'yaw_offset': 0.0,
            'broadcast_utm_transform': True,
            'publish_filtered_gps': True,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/gps/fix'),  # Your GNSS topic
            ('/odometry/filtered', '/odometry/local'),
        ]
    )
    
    # EKF node
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        parameters=[os.path.join(get_package_share_directory('localization_dev'), 
                                 'config', 'ekf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odometry/local')
        ]
    )
    
    # SLAM Toolbox
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(get_package_share_directory('localization_dev'),
                                'config', 'slam_toolbox_config.yaml')],
        output='screen'
    )

    madwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'publish_tf': False
        }]
    )
    
    return LaunchDescription([
        madwick_filter_node,
        navsat_transform_node,
        ekf_node,
        slam_toolbox_node,
    ])