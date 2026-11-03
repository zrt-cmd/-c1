from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'car1_app'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob(os.path.join('launch', '*launch.[pxy][yma]*'))),
        ('share/' + package_name + '/config', glob(os.path.join('config', '*.yaml'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='dev@example.com',
    description='Smart applications for car1 robot',
    license='Apache-2.0',
    entry_points={
        'console_scripts': [
            'line_following = car1_app.line_following:main',
            'object_tracking = car1_app.object_tracking:main',
            'hand_gesture = car1_app.hand_gesture:main',
            'lidar_controller = car1_app.lidar_controller:main',
            'track_and_grab = car1_app.track_and_grab:main',
            'visual_grasp = car1_app.visual_grasp:main',
            'navigation_grasp_transport = car1_app.navigation_grasp_transport:main',
            'test_hsv_detection = car1_app.test_hsv_detection:main',
            'test_3d_localization = car1_app.test_3d_localization:main',
        ],
    },
)
