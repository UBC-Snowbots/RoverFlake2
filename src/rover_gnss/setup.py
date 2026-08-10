from glob import glob

from setuptools import find_packages, setup

package_name = 'rover_gnss'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob('config/*')),
    ],
    install_requires=['setuptools', 'pyserial', 'pynmea2'],
    zip_safe=True,
    maintainer='ubuntu',
    maintainer_email='deadfloppy@protonmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'nmea_reader = rover_gnss.nmea_reader:main',
            'latlon_markers = rover_gnss.latlon_markers:main',
            'fake_gnss = rover_gnss.fake_gnss:main'
        ],
    },
)
