#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import Command, LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    # Define paths for URDF and RViz config
    urdf_path = PathJoinSubstitution([
        FindPackageShare("robot_description"), "urdf", "autonomous_robot.urdf.xacro"
    ])

    rviz_config_path = PathJoinSubstitution([
        FindPackageShare("robot_description"), "rviz", "urdf.rviz"
    ])

    # Declare arguments
    declare_urdf_path = DeclareLaunchArgument(
        name="urdf_path",
        default_value=urdf_path,
        description="Path to the robot URDF file"
    )

    declare_rviz_config_path = DeclareLaunchArgument(
        name="rviz_config_path",
        default_value=rviz_config_path,
        description="Path to the RViz configuration file"
    )

    # Robot State Publisher
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        output="screen",
        parameters=[{
            "robot_description": Command(["xacro ", LaunchConfiguration("urdf_path")]),
            "use_sim_time": True
        }]
    )

    # Gazebo Launch with empty world
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), "launch", "gazebo.launch.py"
            ])
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([
                FindPackageShare("gazebo_ros"), "worlds", "empty.world"
            ])
        }.items()
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Spawn the robot entity in Gazebo
    spawn_robot_entity = Node(
        package="gazebo_ros",
        executable="spawn_entity.py",
        output="screen",
        arguments=["-topic", "robot_description", "-entity", "autonomous_robot", "-z", "0.5"]
    )

    # RViz Launch
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_path")],
        parameters=[{"use_sim_time": True}]
    )

    # Twist Mux for velocity control
    twist_mux_params = os.path.join(get_package_share_directory('robot_description'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/cmd_vel')]
        )

    return LaunchDescription([
        declare_urdf_path,
        declare_rviz_config_path,
        robot_state_publisher,
        gazebo_launch,
        joint_state_publisher,
        spawn_robot_entity,
        rviz_launch,
        twist_mux
    ]) 