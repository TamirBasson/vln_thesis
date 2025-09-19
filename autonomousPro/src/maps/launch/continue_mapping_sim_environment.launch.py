#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration

def generate_launch_description():
    """
    Continue mapping from existing warehouse map WITH Nav2 navigation
    """
    
    # Get package directories
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    robot_simulation_dir = get_package_share_directory('robot_simulation')
    nav2_bringup_dir = get_package_share_directory('nav2_bringup')
    
    # Path to saved map state and configurations (load from source directory)
    source_maps_dir = '/home/tamir/autonomousPro/src/maps/maps/simulation_env'
    pbstream_path = os.path.join(source_maps_dir, 'combined_scans.pbstream')
    config_dir = os.path.join(robot_navigation_dir, 'config')
    nav2_params_file = os.path.join(robot_simulation_dir, 'config', 'nav2_cart_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time'
        ),
        
        # Cartographer Mapping Node
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            arguments=[
                '-configuration_directory', config_dir,
                '-configuration_basename', 'my_robot.lua',
                '-load_state_filename', pbstream_path,
            ]
        ),
        
        # Cartographer Occupancy Grid Node
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }],
            arguments=['-resolution', '0.05']
        ),
        
        # Nav2 Navigation Stack
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(nav2_bringup_dir, 'launch', 'navigation_launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'true',
                'params_file': nav2_params_file,
                'use_lifecycle_mgr': 'false',
                'map_subscribe_transient_local': 'true'
            }.items()
        ),
        
        # Lifecycle manager for Nav2
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_navigation',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': [
                    'controller_server',
                    'smoother_server', 
                    'planner_server',
                    'behavior_server',
                    'bt_navigator',
                    'waypoint_follower',
                    'velocity_smoother'
                ]
            }]
        )
    ])
