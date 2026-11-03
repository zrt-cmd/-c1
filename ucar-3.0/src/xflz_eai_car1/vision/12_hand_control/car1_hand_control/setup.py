from setuptools import find_packages, setup

package_name = 'car1_hand_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/hand_control.launch.py']),
        ('share/' + package_name + '/config', ['config/hand_control_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iflytek',
    maintainer_email='iflytek@example.com',
    description='Exp10 pose landmark detection (RKNN BlazePose 33 points, RKNNLite on RK3576)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_control = hand_control_core.hand_control_node:main',
        ],
    },
)
