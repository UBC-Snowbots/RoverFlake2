from launch import LaunchDescription
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from launch.substitutions import PathJoinSubstitution

def generate_launch_description():
    ukf_local_yaml = PathJoinSubstitution([
        FindPackageShare('localization_dev'),
        'config',
        'ukf_local.yaml'
    ])
    ukf_global_yaml = PathJoinSubstitution([
        FindPackageShare('localization_dev'),
        'config',
        'ukf_global.yaml'
    ])
    navsat_yaml = PathJoinSubstitution([
        FindPackageShare('localization_dev'),
        'config',
        'navsat.yaml'
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
            package='robot_localization',
            executable='navsat_transform_node',
            name='navsat_transform',
            output='screen',
            parameters=[navsat_yaml]
        ),

        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_global',
            output='screen',
            parameters=[ukf_global_yaml]
        ),
    ])
