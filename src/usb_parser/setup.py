from setuptools import setup
import os
from glob import glob

package_name = 'usb_parser'

setup(
    name=package_name,
    version='0.7.0',
    packages=[package_name],
    install_requires=['setuptools'],
    zip_safe=True,
    author='You',
    author_email='cameronbasara@gmail.com',
    maintainer='Cameron Basara',
    maintainer_email='cameronbasara@gmail.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS2 usb parser for comp, more description in data_parsing.py',
    license='Apache License, Version 2.0',
    tests_require=['pytest'],
    data_files=[
        # Add other directories you need to install, e.g., config files, etc.
        (os.path.join('share', package_name), ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        
    ],
    entry_points={
        'console_scripts': [
            'demo = usb_parser.scripts.data_parsing:main'
        ],
    },
)