import os
from glob import glob
from setuptools import setup

package_name = 'car1_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=['bringup'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='TODO',
    maintainer_email='TODO@email.com',
    description='Bringup package for xflz_eai_car1',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'robot_description_pub = bringup.robot_description_pub:main',
        ],
    },
)
