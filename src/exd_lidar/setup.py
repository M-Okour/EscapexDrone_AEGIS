import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'exd_lidar'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='M. Okour',
    maintainer_email='okour86@hotmail.com',
    description='LiDAR subsystem: LiDAR driver and obstacle extraction.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'lidar_node = exd_lidar.lidar_node:main',
            'obstacle_node = exd_lidar.obstacle_node:main',
        ],
    },
)
