from setuptools import setup
import os
from glob import glob

package_name = 'my_bringup'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tamir',
    maintainer_email='tamir@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'image_pose_logger = my_bringup.image_pose_logger:main',
            'live_pose_plotter = my_bringup.live_pose_plotter:main',
            'my_nav_goalpose_node = my_bringup.my_nav_goalpose:main',
            'my_nav_waypoint_node = my_bringup.my_nav_waypoint:main',
        ],
    },
)
