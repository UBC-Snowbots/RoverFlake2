from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rover_navigate'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
         glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
         glob('config/*.yaml')),
    ],
    install_requires=['setuptools', 'numpy'],
    zip_safe=True,
    maintainer='rowan',
    maintainer_email='zawadzkirowan@gmail.com',
    description='Path planning and navigation for the rover',
    license='TODO: License declaration',
    extras_require={
        'test': ['pytest'],
    },
    entry_points={
        'console_scripts': [
            'synthetic_lidar = rover_navigate.synthetic_lidar:main',
            'simple_planner = rover_navigate.simple_planner:main',
        ],
    },
)
