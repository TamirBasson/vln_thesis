import os
import sys

import launch
from launch.conditions import IfCondition
from launch.substitutions import PythonExpression
from ament_index_python.packages import get_package_share_directory


def generate_launch_description():
    world_file_name = "hospital.world"
    world = os.path.join(get_package_share_directory('aws_robomaker_hospital_world'), 'worlds', world_file_name)

    gazebo_ros = get_package_share_directory('gazebo_ros')
    
    # Performance optimized Gazebo server launch
    gazebo_server = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzserver.launch.py')),
        launch_arguments={
            'world': world,
            'verbose': 'false',  # Reduce console output for faster loading
            'physics': 'ode',    # Use ODE physics engine (default, good performance)
            'server_required': 'true',
            'gui_required': 'false',
            'recording': 'false',
            'debug': 'false',
            'paused': 'false',
            'use_sim_time': 'true',
            'lockstep': 'false',  # Don't wait for real-time, load as fast as possible
            'extra_gazebo_args': '--verbose-plugins'  # Only show plugin loading info
        }.items()
    )
    
    # Optimized Gazebo client (GUI) - only start if needed
    gazebo_client = launch.actions.IncludeLaunchDescription(
        launch.launch_description_sources.PythonLaunchDescriptionSource(
            os.path.join(gazebo_ros, 'launch', 'gzclient.launch.py')),
        condition=launch.conditions.IfCondition(launch.substitutions.LaunchConfiguration('gui')),
        launch_arguments={
            'verbose': 'false'
        }.items()
    )

    ld = launch.LaunchDescription([
        launch.actions.DeclareLaunchArgument(
          'world',
          default_value=[world, ''],
          description='SDF world file'),
        launch.actions.DeclareLaunchArgument(
            name='gui',
            default_value='true',
            description='Start Gazebo GUI'),
        launch.actions.DeclareLaunchArgument(
            name='verbose',
            default_value='false',
            description='Enable verbose output'),
        launch.actions.DeclareLaunchArgument(
            name='paused',
            default_value='false',
            description='Start simulation paused'),
        gazebo_server,
        gazebo_client
    ])
    return ld


if __name__ == '__main__':
    generate_launch_description()

