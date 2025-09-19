#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import ExecuteProcess, DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Save current map for SIMULATION environment
    """
    
    # Save directly to source directory
    source_maps_dir = '/home/tamir/autonomousPro/src/maps/maps/simulation_env'
    map_file_path = os.path.join(source_maps_dir, 'combined_scans')
    pbstream_path = os.path.join(source_maps_dir, 'combined_scans.pbstream')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',  # Simulation
            description='Use simulation time'
        ),
        
        # Save the occupancy grid map
        Node(
            package='nav2_map_server',
            executable='map_saver_cli',
            name='map_saver',
            output='screen',
            arguments=['-f', map_file_path],
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'occupied_threshold': 0.65,
                'free_threshold': 0.25
            }]
        ),
        
        # Save Cartographer state (pbstream)
        ExecuteProcess(
            cmd=[
                'ros2', 'service', 'call', '/write_state',
                'cartographer_ros_msgs/srv/WriteState',
                f'{{filename: "{pbstream_path}"}}'
            ],
            name='save_cartographer_state',
            output='screen'
        )
    ])