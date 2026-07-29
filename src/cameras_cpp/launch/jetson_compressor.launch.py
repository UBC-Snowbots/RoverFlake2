import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, HistoryPolicy

def generate_launch_description():

    test_params = os.path.join(
    get_package_share_directory('cameras_cpp'),
    'config', 'jetson_test_params.yaml'
    )

    feed_test = Node(
        package='camera_ros',
        executable='camera_node',
        name='test_camera_node',
        output='screen',
        parameters=[test_params],
        remappings=[
            ('/test_camera_node/image_raw', '/vehicle_1/rear_feed/image_raw'),  
            ('/test_camera_node/image_raw/ffmpeg', '/vehicle_1/rear_feed/image_raw/ffmpeg'),
            ('/test_camera_node/camera_info', '/vehicle_1/rear_feed/camera_info'),
            # Add more remappings here if needed
        ],
    )

    return LaunchDescription([
        feed_test
    ])