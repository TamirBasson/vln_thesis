from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_control_hardware'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tamir',
    maintainer_email='tamirbasson99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_control_node = robot_control_hardware.joy_control_node:main',
            'cmd2pwm = robot_control_hardware.cmd2pwm:main',
            'pwm2GPIO = robot_control_hardware.pwm2GPIO:main',
            'cmd2gpio= robot_control_hardware.cmd2gpio:main',
        ],
    },
)
