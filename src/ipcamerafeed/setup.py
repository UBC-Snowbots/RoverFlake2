from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'ipcamerafeed'


setup(
   name=package_name,
   version='0.0.0',
   packages=find_packages(exclude=['test']),
   data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('srv/*.srv')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    include_package_data=True,
    package_data={
        '': ['srv/*.srv'],
    },
   install_requires=[
       'setuptools',
       'opencv-python',
       'onvif-zeep',
       'pyserial'
   ],
   zip_safe=True,
   maintainer='kingcammy',
   maintainer_email='cameronbasara@gmail.com',
   description='TODO: Package description',
   license='Apache-2.0',
   tests_require=['pytest'],
   entry_points={
       'console_scripts': [
           'ipcamera = ipcamerafeed.ipcamera:main',
           'ipcamerazoom = ipcamerafeed.ipcamerazoom:main',
           'pitch_tilt_node = ipcamerafeed.pitch_tilt_node:main',
       ],
   },
)
