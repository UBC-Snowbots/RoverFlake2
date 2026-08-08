from setuptools import setup

package_name = 'rover_mapping'

setup(
    name=package_name,
    version='0.2.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Cameron Basara',
    maintainer_email='cameronbasara@gmail.com',
    description='GNSS mission recording, waypoint tagging, and '
                'report-map export',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_gps = rover_mapping.fake_gps:main',
            'mission_manager = rover_mapping.mission_manager:main',
            'tag = rover_mapping.tag_cli:main',
            'fetch_tiles = rover_mapping.fetch_tiles:main',
            'export_map = rover_mapping.exporter:main',
        ],
    },
)
