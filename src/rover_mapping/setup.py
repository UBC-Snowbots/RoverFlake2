from glob import glob

from setuptools import setup

package_name = 'rover_mapping'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*.mvc')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Basara',
    maintainer_email='cameronbasara@gmail.com',
    description='Path recording, waypoint tagging, replay and '
                'report-map export over mapviz',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_gps = rover_mapping.fake_gps:main',
            'mission_manager = rover_mapping.mission_manager:main',
            'tag = rover_mapping.tag_cli:main',
            'export_report_map = rover_mapping.export_report_map:main',
        ],
    },
)
