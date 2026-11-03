from setuptools import setup
import os
from glob import glob

package_name = 'eye_hand_calibration'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name, 'scripts'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Launch files
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        # Config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Kiro AI Assistant',
    maintainer_email='robot@example.com',
    description='Eye-in-hand calibration for depth camera on robotic arm using AprilTag',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'apriltag_detector = eye_hand_calibration.apriltag_detector:main',
            'data_collector = eye_hand_calibration.data_collector:main',
            'depth_corrector = eye_hand_calibration.depth_corrector:main',
            'calibration_verifier = eye_hand_calibration.calibration_verifier:main',
            'collect_calibration_data = scripts.collect_calibration_data:main',
            'compute_hand_eye_transform = scripts.compute_hand_eye_transform:main',
            'generate_corrected_urdf = scripts.generate_corrected_urdf:main',
        ],
    },
)
