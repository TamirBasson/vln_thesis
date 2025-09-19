#!/usr/bin/env python3

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from launch.substitutions import LaunchConfiguration
import launch.conditions

def generate_launch_description():
    """
    Real Robot: Continue mapping WITH proper localization
    Uses AMCL for localization in known areas + Cartographer for mapping new areas
    """
    
    # Get package directories
    robot_navigation_dir = get_package_share_directory('robot_navigation')
    robot_simulation_dir = get_package_share_directory('robot_simulation')
    
    # Path to saved map state and configurations (load from source directory)
    source_maps_dir = '/home/tamir/autonomousPro/src/maps/maps/real_env'
    pbstream_path = os.path.join(source_maps_dir, 'combined_scans.pbstream')
    map_yaml_path = os.path.join(source_maps_dir, 'combined_scans.yaml')
    config_dir = os.path.join(robot_navigation_dir, 'config')
    nav2_params_file = os.path.join(robot_navigation_dir, 'config', 'nav2_cart_params.yaml')
    
    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='false',  # REAL ROBOT - no simulation time
            description='Use simulation time'
        ),
        
        DeclareLaunchArgument(
            'initial_pose_x',
            default_value='0.0',
            description='Initial X position of robot on map (meters)'
        ),
        
        DeclareLaunchArgument(
            'initial_pose_y', 
            default_value='0.0',
            description='Initial Y position of robot on map (meters)'
        ),
        
        DeclareLaunchArgument(
            'initial_pose_yaw',
            default_value='0.0',
            description='Initial yaw orientation of robot on map (radians)'
        ),
        
        DeclareLaunchArgument(
            'enable_mapping',
            default_value='true',
            description='Enable Cartographer mapping (set false for pure navigation)'
        ),
        
        # Map Server - loads existing map for AMCL localization
        Node(
            package='nav2_map_server',
            executable='map_server',
            name='map_server',
            output='screen',
            parameters=[{
                'yaml_filename': map_yaml_path,
                'use_sim_time': LaunchConfiguration('use_sim_time')
            }]
        ),
        
        # AMCL - robust localization in known areas
        Node(
            package='nav2_amcl',
            executable='amcl',
            name='amcl',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'global_frame_id': 'map',
                'odom_frame_id': 'odom',
                'base_frame_id': 'base_footprint',
                'scan_topic': 'scan',
                'set_initial_pose': True,
                'initial_pose': {
                    'x': LaunchConfiguration('initial_pose_x'),
                    'y': LaunchConfiguration('initial_pose_y'),
                    'z': 0.0,
                    'yaw': LaunchConfiguration('initial_pose_yaw')
                },
                # Real robot optimized parameters
                'max_particles': 3000,
                'min_particles': 1000,
                'update_min_d': 0.1,      # More sensitive for real robot
                'update_min_a': 0.15,     # More sensitive for real robot
                'resample_interval': 1,
                'transform_tolerance': 0.3,  # Tighter tolerance for real robot
                'recovery_alpha_slow': 0.001,
                'recovery_alpha_fast': 0.1,
                'laser_likelihood_max_dist': 2.5,
                'laser_max_range': 8.0,
                'laser_min_range': 0.2,
                'laser_model_type': 'likelihood_field',
                'max_beams': 60,
                'z_hit': 0.6,            # Higher confidence in hits for real robot
                'z_short': 0.05,
                'z_max': 0.05,
                'z_rand': 0.3,           # Lower random noise for real robot
                'sigma_hit': 0.15        # Tighter hit tolerance
            }]
        ),
        
        # Cartographer Mapping Node - for new areas only
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
            ],
            condition=launch.conditions.IfCondition(LaunchConfiguration('enable_mapping'))
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
            arguments=['-resolution', '0.05'],
            condition=launch.conditions.IfCondition(LaunchConfiguration('enable_mapping'))
        ),
        
        # Nav2 Navigation Stack - using robot_navigation package
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(robot_navigation_dir, 'launch', 'navigation.launch.py')
            ),
            launch_arguments={
                'use_sim_time': 'false',  # REAL ROBOT
                'params_file': nav2_params_file,
                'autostart': 'true',
                'namespace': ''
            }.items()
        ),
        
        # Lifecycle manager for map server + AMCL only
        # (Navigation lifecycle manager is handled by robot_navigation launch file)
        Node(
            package='nav2_lifecycle_manager',
            executable='lifecycle_manager',
            name='lifecycle_manager_localization',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
                'autostart': True,
                'node_names': ['map_server', 'amcl']
            }]
        )
    ])