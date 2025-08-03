from setuptools import find_packages, setup
from glob import glob
import os

package_name = 'rover_vision'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'numpy',
        'opencv-python',
        'pyrealsense2',
    ],
    zip_safe=True,
    maintainer='kingcammy',
    maintainer_email='cameronbasara@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_pub_node = rover_vision.camera_pub_node:main',
            'colour_detection_node = rover_vision.colour_detection_node:main',
        ],
    },
)
