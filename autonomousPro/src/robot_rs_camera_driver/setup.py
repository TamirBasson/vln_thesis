from setuptools import find_packages, setup
import os
from glob import glob
package_name = 'robot_rs_camera_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/scripts', glob('scripts/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='root',
    maintainer_email='tamirbasson99@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'realsense_rgbd_publisher = robot_rs_camera_driver.realsense_rgbd_publisher:main',
            'yolo_dectetor_node = robot_rs_camera_driver.yolo_detector_node:main'
        ],
    },
)
