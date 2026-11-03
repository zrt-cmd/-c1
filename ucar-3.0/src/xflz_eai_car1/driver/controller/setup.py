from setuptools import setup
import os
from glob import glob

package_name = 'controller'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@example.com',
    description='Chassis controller for car1 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odom_publisher = controller.odom_publisher_node:main',
            'init_pose = controller.init_pose_node:main',
        ],
    },
)
