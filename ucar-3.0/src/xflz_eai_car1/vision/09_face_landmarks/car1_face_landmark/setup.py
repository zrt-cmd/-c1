from setuptools import find_packages, setup

package_name = 'car1_face_landmark'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/face_landmark.launch.py']),
        ('share/' + package_name + '/config', ['config/face_config.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='iflytek',
    maintainer_email='iflytek@example.com',
    description='Exp09 face landmark detection (RKNN Blaze 468 points, RKNNLite on RK3576)',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'face_landmark = face_landmark_core.face_landmark_node:main',
        ],
    },
)
