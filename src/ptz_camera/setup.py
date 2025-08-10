from setuptools import setup
import os
from glob import glob

package_name = 'ptz_camera'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    py_modules=[
        'ipcamera',
        'ipcamerazoom',
        'pitch_tilt_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='YOUR_NAME',
    maintainer_email='your@email.com',
    description='PTZ camera control node for ROS 2',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ipcamera = ipcamera:main',
            'ipcamerazoom = ipcamerazoom:main',
            'pitch_tilt_node = pitch_tilt_node:main',
        ],
    },
)
