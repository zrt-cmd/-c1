from setuptools import setup

package_name = 'car1_sdk'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='developer',
    maintainer_email='developer@example.com',
    description='Utility library for car1 robot (PID, common helpers)',
    license='Apache-2.0',
    tests_require=['pytest'],
)
