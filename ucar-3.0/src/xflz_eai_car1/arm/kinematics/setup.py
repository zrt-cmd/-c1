from setuptools import setup

package_name = 'car1_kinematics'

setup(
    name=package_name,
    version='0.0.1',
    packages=['kinematics'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='car1',
    maintainer_email='car1@todo.todo',
    description='Kinematics for car1 arm',
    license='TODO',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fk_demo = kinematics.fk_demo:main',
            'ik_demo = kinematics.ik_demo:test_ik',
            'pwm_to_fk_demo = kinematics.pwm_to_fk_demo:main',
            'ik_test_node = kinematics.ik_test_node:main',
            'interactive_ik_test = kinematics.interactive_ik_test:main',
        ],
    },
)
