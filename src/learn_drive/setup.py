from setuptools import find_packages, setup

package_name = 'learn_drive'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='yeetbananis',
    maintainer_email='timofeygen@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'square_driver = learn_drive.square_driver:main',
            'square_closed_loop = learn_drive.square_closed_loop:main',
            'turtle_tf_broadcaster = learn_drive.turtle_tf_broadcaster:main',
            'go_to_goal = learn_drive.go_to_goal:main',
            'imu_to_rover_tf = learn_drive.imu_to_rover_tf:main',
            'imu_rover_motion = learn_drive.imu_rover_motion:main',
            'fake_rover = learn_drive.fake_rover:main',
            'fake_lidar = learn_drive.fake_lidar:main',
            'sweep_driver = learn_drive.sweep_driver:main',
        ],
    },
)
