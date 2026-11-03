from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car1_slam'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
        ('share/' + package_name + '/rviz', glob(os.path.join('rviz', '*.rviz'))),
        ('share/' + package_name + '/maps', glob(os.path.join('maps', '*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='dev@example.com',
    description='SLAM mapping for car1 robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'map_save = car1_slam.map_save:main',
            'slam_teleop = car1_slam.slam_teleop:main',
        ],
    },
)
