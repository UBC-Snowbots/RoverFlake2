from setuptools import find_packages, setup

package_name = 'sweep'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
    'setuptools',
    'sweeppy',],
    zip_safe=True,
    maintainer='kingcammy',
    maintainer_email='cameronbasara@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sweep_slam_node = sweep.sweep_slam_node:main'
        ],
    },
)
