from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

#This launch file is used to launch the Cartographer localization node with the saved map file.
#only after you create the map file using the Cartographer mapping node.

def generate_launch_description():
    # Use FindPackageShare to locate the package paths dynamically
    config_directory = PathJoinSubstitution([
        FindPackageShare("autonomous_robot_description"), "config"
    ])
    pbstream_path = "/home/tamir/autonomous_robot/autonomous_robot_description/warehouse_AWS.pbstream"


    return LaunchDescription([
        DeclareLaunchArgument(
            'use_sim_time',
            default_value='true',
            description='Use simulation time if true'
        ),
        DeclareLaunchArgument(
            'configuration_directory',
            default_value=config_directory,
            description='Directory with Cartographer configuration files'
        ),
        DeclareLaunchArgument(
            'configuration_basename',
            default_value='my_robot.lua',
            description='Cartographer configuration file'
        ),
        DeclareLaunchArgument(
            'warehouse_AWS',
            default_value=pbstream_path,
            description='Path to the .pbstream file'
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_node',
            name='cartographer_node',
            output='screen',
            parameters=[{
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
            arguments=[
                '-configuration_directory', LaunchConfiguration('configuration_directory'),
                '-configuration_basename', LaunchConfiguration('configuration_basename'),
                '-load_state_filename', LaunchConfiguration('warehouse_AWS'),
            ]
        ),
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom_broadcaster',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),
        Node(
            package='cartographer_ros',
            executable='cartographer_occupancy_grid_node',
            name='cartographer_occupancy_grid_node',
            output='screen',
            arguments=['-resolution', '0.05']
        ),
    ])
