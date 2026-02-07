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
    return LaunchDescription([

        Node(
            package='robot_localization',
            executable='ukf_node',
            name='ukf_local',
            output='screen',
            parameters=[ukf_local_yaml],
            remappings=[('odometry/filtered', 'odometry/filtered')]
        ),
    ])
