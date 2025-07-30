from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'science_gsensor'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=[
        'setuptools',
        'pyserial'],
    zip_safe=True,
    maintainer='rv',
    maintainer_email='zawadzkirowan@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'send_data = science_gsensor.send_data_node:main',
            'plot = science_gsensor.plot_node:main',
        ],
    },
)
