from setuptools import find_packages, setup
import os 
import glob

package_name = 'usb_parser2'

# REQUIRES = [i.strip() for i in open("requirements.txt").readlines()]

setup(
    name=package_name,
    version='0.0.0',
    packages=(find_packages()),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch/*.launch.py'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Camerom Basara',
    maintainer_email='cameronbasara@gmail.com',
    description='Package designed to complete the usb parsing portion of the search and rescue task.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'usb_parser_node = usb_parser2.usb_parser_node:main'
        ],
    },
)
