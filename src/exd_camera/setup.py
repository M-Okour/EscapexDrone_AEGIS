import os
from glob import glob

from setuptools import find_packages, setup

package_name = 'exd_camera'

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
    description='Camera subsystem: camera driver and object detection.',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = exd_camera.camera_node:main',
            'detector_node = exd_camera.detector_node:main',
        ],
    },
)
