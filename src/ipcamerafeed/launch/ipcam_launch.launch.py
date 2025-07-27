from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        Node(
            package='ipcamerafeed',
            executable='ipcamera',  
            name='ipcamera'
        ),
        Node(
            package='ipcamerafeed',
            executable='ipcamerazoom',  
            name='ipcamerazoom'
        ),
        Node(
            package='ipcamerafeed',
            executable='pitch_tilt_node',
            name='pitch_tilt'
        ),
        Node(
            package='image_publisher',
            executable='image_publisher_node',  
            name='image_publisher',
            parameters=[{'filename' : 'rtsp://192.168.0.95/stream0'}]
        ),
        Node(
            package='image_view',
            executable='image_view',
            name='ipcamera_view',
            remappings=[
                ('image', '/image_raw') # image raw ropic from the image publisher
            ]
        )
    ])
