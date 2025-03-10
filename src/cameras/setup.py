from setuptools import find_packages, setup
import os 
import glob

package_name = 'cameras'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch/*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='cameronbasara@gmail.com',
    description='Launch Files for opening cameras in ROS2, converting from SnowBots repo',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_publisher = cameras.camera_publisher:main',
            'camera_publisher_full = cameras.camera_publisher_full:main',
            'cameras_node = cameras.cameras_node:main',
            'full_cameras_node = cameras.full_cameras_node:main'
        ],
    },
)
