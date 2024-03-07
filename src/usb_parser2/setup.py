from setuptools import find_packages, setup
import os 
import glob

package_name = 'usb_parser2'

REQUIRES = [i.strip() for i in open("requirements.txt").readlines()]

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
    install_requires=REQUIRES,
    zip_safe=True,
    maintainer='Camerom Basara',
    maintainer_email='cameronbasara@gmail.com',
    description='Package designed to complete the usb parsing portion of the search and rescue task. Launch file sets set pose movement in motion, which plugs usb into port, then parser script is run which finally will alert the driver of which reactor has malfunctioned.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usbparser = usb_parser2.user_parser_node:main'
        ],
    },
)
