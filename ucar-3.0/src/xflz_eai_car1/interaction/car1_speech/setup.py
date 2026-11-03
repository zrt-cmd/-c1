from setuptools import setup
import os
from glob import glob

package_name = 'car1_speech'

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
    maintainer='Your Name',
    maintainer_email='your_email@example.com',
    description='Speech interaction for xflz_eai_car1 robot',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'wake_word_node = car1_speech.wake_word_node:main',
            'asr_node = car1_speech.asr_node:main',
            'tts_node = car1_speech.tts_node:main',
        ],
    },
)
