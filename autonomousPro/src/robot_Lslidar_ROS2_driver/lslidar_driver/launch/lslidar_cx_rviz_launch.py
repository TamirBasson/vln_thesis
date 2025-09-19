#!/usr/bin/python3
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import LifecycleNode
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch.actions import DeclareLaunchArgument

import lifecycle_msgs.msg
import os
import subprocess


def generate_launch_description():
    driver_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'params', 'lslidar_cx.yaml')
    rviz_dir = os.path.join(get_package_share_directory('lslidar_driver'), 'rviz_cfg', 'lslidar_cx.rviz')

    p = subprocess.Popen("echo $ROS_DISTRO", stdout=subprocess.PIPE, shell=True)
    driver_node = ""
    rviz_node = ""
    ros_version = p.communicate()[0]
    #print(ros_version)
    if ros_version == b'dashing\n' or ros_version == b'eloquent\n':
        print("ROS VERSION: dashing/eloquent")
        driver_node = LifecycleNode(package='lslidar_driver',
                                    node_namespace='cx',
                                    node_executable='lslidar_driver_node',
                                    node_name='lslidar_driver_node',
                                    output='screen',
                                    parameters=[driver_dir],
                                    )
        rviz_node = Node(
            package='rviz2',
            node_namespace='cx',
            node_executable='rviz2',
            node_name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen')
    elif ros_version == b'foxy\n' or ros_version == b'galactic\n' or ros_version == b'humble\n':
        print("ROS VERSION: foxy/galactic/humble")
        driver_node = LifecycleNode(package='lslidar_driver',
                                    namespace='cx',
                                    executable='lslidar_driver_node',
                                    name='lslidar_driver_node',
                                    output='screen',
                                    emulate_tty=True,
                                    parameters=[driver_dir],
                                    )
        rviz_node = Node(
            package='rviz2',
            namespace='cx',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_dir],
            output='screen')

    else:
        print("Please configure the ros environment")
        exit()

    # Include pointcloud_to_laserscan node
    pointcloud_to_laserscan_node = Node(
        package='pointcloud_to_laserscan',
        executable='pointcloud_to_laserscan_node',
        remappings=[
            ('cloud_in', [LaunchConfiguration('cx'), '/lslidar_point_cloud']),
            ('scan', '/scan') 
        ],
        parameters=[{
            'target_frame': 'map',
            'transform_tolerance': 0.01,
            'min_height': 0.0,
            'max_height': 1.0,
            'angle_min': -3.14159,  # -180 degrees
            'angle_max': 3.14159,  # +180 degrees
            'angle_increment': 0.0087,  # M_PI/360.0
            'scan_time': 0.3333,
            'range_min': 0.45,
            'range_max': 6.0,
            'use_inf': True,
            'inf_epsilon': 1.0
        }],
        name='pointcloud_to_laserscan'
    )


    return LaunchDescription([
        DeclareLaunchArgument(
            name='cx',
            default_value='cx',
            description='Namespace for sample topics'

      ),
        rviz_node,
        driver_node,
        pointcloud_to_laserscan_node
    ])
