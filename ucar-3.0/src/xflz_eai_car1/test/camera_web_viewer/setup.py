from setuptools import setup
import os
from glob import glob

package_name = 'camera_web_viewer'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'static'), glob('static/*.html')),
        (os.path.join('share', package_name, 'static'), glob('static/*.css')),
        (os.path.join('share', package_name, 'static'), glob('static/*.js')),
        (os.path.join('share', package_name, 'static/libs'), glob('static/libs/*.js')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='AIoTMaker',
    maintainer_email='user@todo.todo',
    description='RGBD Camera Web Viewer for HP60C',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'server = camera_web_viewer.server:main'
        ],
    },
)
