import os
from glob import glob
from setuptools import setup

package_name = 'calibration'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*.*'))),
        (os.path.join('share', package_name, 'config'), glob(os.path.join('config', '*.*'))),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@example.com',
    description='Calibration tools for chassis odometry and arm servo offsets',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'calibrate_linear = calibration.calibrate_linear:main',
            'calibrate_angular = calibration.calibrate_angular:main',
            'calibrate_servo = calibration.calibrate_servo:main',
        ],
    },
)
