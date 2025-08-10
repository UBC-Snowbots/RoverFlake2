from setuptools import find_packages, setup

package_name = 'pose'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_yindex/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'geographiclib'],
    zip_safe=True,
    maintainer='parallels',
    maintainer_email='bobmasibo@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'pose_publisher = pose.pose_publisher:main',
            'imu_gps_fuse = pose.imu_gps_fuse:main',
            'fusion = pose.fusion:main',
            'pubpt = pose.pubpt:main',
        ],
    },
)
