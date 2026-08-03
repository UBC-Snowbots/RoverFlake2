from setuptools import setup
import os
from glob import glob

package_name = 'ptz_cam'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],  # assumes your .py files are in a ptz_cam/ directory
    data_files=[
        ('share/ament_index/resource_index/packages', [f'resource/{package_name}']),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'setuptools',
        'onvif-zeep',
        'pyserial',
        'opencv-python',
    ],
    zip_safe=True,
    maintainer='Cameron Basara',
    maintainer_email='cameronbasara@gmail.com',
    description='ROS 2 package for IP camera RTSP streaming and PTZ control via ONVIF and Serial.',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'ipcamera = ptz_cam.ipcamera:main',
            'ipcamerazoom = ptz_cam.ipcamerazoom:main',
            'pitch_tilt_node = ptz_cam.pitch_tilt_node:main',
        ],
    },
)
