from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'traffic_light'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    py_modules=[
        'traffic_light.traffic_light_node',
        'traffic_light.light_detection_node',
    ],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cameron',
    maintainer_email='cameronbasara@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'traffic_light_node = traffic_light.traffic_light_node:main',
            'light_dectection_node = traffic_light.light_detection_node:main', 
        ],
    },
)
