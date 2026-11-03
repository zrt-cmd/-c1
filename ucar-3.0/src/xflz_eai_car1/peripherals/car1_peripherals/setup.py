from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car1_peripherals'

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
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='dev@example.com',
    description='Peripheral sensor integration for car1 robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'teleop_key_control = car1_peripherals.teleop_key_control:main',
        ],
    },
)
