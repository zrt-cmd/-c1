from setuptools import setup
import os
from glob import glob

package_name = 'car1_arm_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'actions'), glob('config/actions/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='todo@example.com',
    description='Arm controller for xflz_eai_car1',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'arm_controller = car1_arm_controller.arm_controller:main',
            'arm_demo = car1_arm_controller.arm_demo:main',
            'action_player = car1_arm_controller.action_player:main',
            'test_fk_ik_loop = car1_arm_controller.test_fk_ik_loop:main',
        ],
    },
)
