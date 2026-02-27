from launch import LaunchDescription
from launch_ros.actions import Node
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    config_dir = get_package_share_directory('localization_dev')

    # Madgwick IMU filter
    madwick_filter_node = Node(
        package='imu_filter_madgwick',
        executable='imu_filter_madgwick_node',
        name='imu_filter',
        output='screen',
        parameters=[{
            'publish_tf': False
        }]
    )

    # LOCAL EKF: IMU + cmd_vel → odom -> base_link (starts immediately)
    ekf_local_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_local_node',
        parameters=[os.path.join(config_dir, 'config', 'ekf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odometry/local'),
        ]
    )

    # Navsat transform: GPS + IMU + local odom → /odometry/gps
    navsat_transform_node = Node(
        package='robot_localization',
        executable='navsat_transform_node',
        name='navsat_transform',
        output='screen',
        arguments=['--ros-args', '--log-level', 'debug'],
        parameters=[{
            'frequency': 10.0,
            'delay': 0.0,
            'magnetic_declination_radians': 0.0,
            'yaw_offset': 0.0,
            'broadcast_utm_transform': True,
            'publish_filtered_gps': True,
            'use_odometry_yaw': False,
            'wait_for_datum': False,
            'world_frame': 'odom',
            'base_link_frame': 'base_link',
        }],
        remappings=[
            ('/imu/data', '/imu/data'),
            ('/gps/fix', '/gnss_fix'),
            ('/odometry/filtered', '/odometry/local'),  # Feedback from local EKF
        ]
    )

    # GLOBAL EKF: IMU + GPS → map -> odom (absolute position)
    ekf_global_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_global_node',
        parameters=[os.path.join(config_dir, 'config', 'ekf.yaml')],
        remappings=[
            ('/odometry/filtered', '/odometry/global'),
        ]
    )

    # SLAM Toolbox: generates map only, no TF (global EKF owns map->odom)
    slam_toolbox_node = Node(
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        parameters=[os.path.join(config_dir, 'config', 'slam_toolbox_config.yaml')],
        output='screen'
    )

    return LaunchDescription([
        madwick_filter_node,
        ekf_local_node,
        navsat_transform_node,
        ekf_global_node,
        slam_toolbox_node,
    ])
