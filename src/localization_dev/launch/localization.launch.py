from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

import os
from ament_index_python.packages import get_package_share_directory
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource

def generate_launch_description():
    ukf_local_yaml = PathJoinSubstitution([
        FindPackageShare('localization_dev'),
        'config',
        'ukf_local.yaml'
    ])
    return LaunchDescription([

        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_local',
            output='screen',
            parameters=[ukf_local_yaml],
            remappings=[('odometry/filtered', 'odometry/filtered')]
        ),
        Node(
            package='imu_filter_madgwick',
            executable='imu_filter_madgwick_node',
            name='imu_filter',
            output='screen',
            parameters=[{
                'publish_tf': False
            }]
        ),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(
                    get_package_share_directory('phidgets_spatial'),
                    'launch',
                    'spatial-launch.py'
                )
            )
        )
    ])
