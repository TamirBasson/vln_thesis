#!/usr/bin/env python3
"""
Visual Language Navigation with GUI Launch File

This launch file starts the complete visual language navigation system with GUI:
1. Autonomous Service Node (VLM + LLM + Motion Planner)
2. Robot Chat GUI (User Interface)

Usage:
    ros2 launch detect_vl vl_navigation_gui.launch.py
    
Optional arguments:
    ros2 launch detect_vl vl_navigation_gui.launch.py 
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, LogInfo, TimerAction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import Node


def generate_launch_description():
    # Get package directory
    detect_vl_dir = get_package_share_directory('detect_vl')
    
    declare_memory_file = DeclareLaunchArgument(
        name='memory_file',
        default_value='memory.yaml',
        description='Memory file to use (memory.yaml or memory_house.yaml)'
    )
    
    declare_environment = DeclareLaunchArgument(
        name='environment',
        default_value='Home',
        description='Default environment context (Home, Warehouse, Office, etc.)'
    )
    

    memory_file = LaunchConfiguration('memory_file')
    environment = LaunchConfiguration('environment')
    
    # =========================================================================
    # 1. Autonomous Service Node - Visual Language Navigation
    # =========================================================================
    service_node = Node(
        package='detect_vl',
        executable='start_service.py',
        name='vl_navigation_service',
        output='screen',
        parameters=[{
            'use_sim_time': True,
            'memory_file': memory_file,
            'environment_context': environment
        }],
        remappings=[
            # Camera topics (simulation)
            ('/camera/camera/image_raw', '/camera/camera/image_raw'),
            ('/camera/camera/depth/image_raw', '/camera/camera/depth/image_raw'),
            # Camera to map transform
            ('/camera2map', '/camera2map'),
            # Navigation topics
            ('/robot_state', '/robot_state'),
            ('task/rect_depth', 'task/rect_depth'),
        ],
        emulate_tty=True,
    )
    
    # =========================================================================
    # 2. Robot Chat GUI - User Interface
    # =========================================================================
    # Delay GUI launch to ensure service is ready
    gui_node = TimerAction(
        period=2.0,  # Wait 2 seconds for service to initialize
        actions=[
            Node(
                package='detect_vl',
                executable='robot_chat_gui.py',
                name='robot_chat_gui',
                output='screen',
                parameters=[{
                    'default_environment': environment
                }],
                remappings=[
                    # GUI communication topics
                    ('/service_question', '/service_question'),
                    ('/environment_context', '/environment_context'),
                    ('/robot_status', '/robot_status'),
                ],
                emulate_tty=True,
            )
        ]
    )
    

    
    # =========================================================================
    # Info Messages
    # =========================================================================
    startup_info = LogInfo(
        msg=[
            '\n',
            '=' * 70, '\n',
            '🚀 Visual Language Navigation System with GUI\n',
            '=' * 70, '\n',
            '\n',
            '📡 Starting Components:\n',
            '  1. ✅ Autonomous Service Node (VLM + LLM + Motion Planner)\n',
            '  2. 🖥️  Robot Chat GUI (launching in 2 seconds...)\n',
            '\n',
            '📝 Configuration:\n',
            '  - Memory File: ', memory_file, '\n',
            '  - Environment: ', environment, '\n',
            '\n',
            '🎮 Usage:\n',
            '  1. Wait for GUI window to appear\n',
            '  2. Set environment context (default: ', environment, ')\n',
            '  3. Send navigation commands via chat interface\n',
            '  4. Watch real-time status updates\n',
            '\n',
            '💡 Example Commands:\n',
            '  - "Move to the table"\n',
            '  - "Go near the chair"\n',
            '  - "Navigate to the bed, turn right, and stop"\n',
            '\n',
            '⚠️  Press Ctrl+C to stop all nodes\n',
            '=' * 70, '\n'
        ]
    )
    
    # =========================================================================
    # Launch Description
    # =========================================================================
    return LaunchDescription([
        # Arguments
        declare_memory_file,
        declare_environment,
        
        # Info
        startup_info,
        
        # Nodes
        service_node,
        gui_node,

    ])

