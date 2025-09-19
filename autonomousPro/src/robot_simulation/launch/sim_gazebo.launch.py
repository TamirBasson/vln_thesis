import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.conditions import IfCondition, UnlessCondition
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

    declare_use_warehouse = DeclareLaunchArgument(
        name="use_warehouse",
        default_value="false",
        description="Use warehouse world (true) or small house world (false)"
    )

    declare_use_small_house = DeclareLaunchArgument(
        name="use_small_house",
        default_value="true",
        description="Use small house world (true) or other worlds (false)"
    )

    declare_use_hospital = DeclareLaunchArgument(
        name="use_hospital",
        default_value="false",
        description="Use hospital world (true) or other worlds (false)"
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

    # Gazebo Launch - Warehouse World
    gazebo_warehouse_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aws_robomaker_small_warehouse_world"), "launch", "no_roof_small_warehouse.launch.py"
            ])
        ]),
        launch_arguments={
            "world_name": PathJoinSubstitution([
                FindPackageShare("aws_robomaker_small_warehouse_world"), "worlds", "no_roof_small_warehouse.world"
            ])
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_warehouse"))
    )

    # Gazebo Launch - Small House World
    gazebo_small_house_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aws_robomaker_small_house_world"), "launch", "small_house.launch.py"
            ])
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([
                FindPackageShare("aws_robomaker_small_house_world"), "worlds", "small_house.world"
            ]),
            "gui": "true"
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_small_house"))
    )

    # Gazebo Launch - Hospital World (Optimized)
    gazebo_hospital_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource([
            PathJoinSubstitution([
                FindPackageShare("aws_robomaker_hospital_world"), "launch", "hospital.launch.py"
            ])
        ]),
        launch_arguments={
            "world": PathJoinSubstitution([
                FindPackageShare("aws_robomaker_hospital_world"), "worlds", "hospital_two_floors.world"
            ]),
            "gui": "true",
            "verbose": "true",
            "physics": "ode",
            "paused": "false",
            "use_sim_time": "true",
            "extra_gazebo_args": "--verbose"
        }.items(),
        condition=IfCondition(LaunchConfiguration("use_hospital"))
    )

    # Joint State Publisher
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # Spawn the robot entity in Gazebo with delay
    spawn_robot_entity = TimerAction(
        period=2.0,
        actions=[
            Node(
                package="gazebo_ros",
                executable="spawn_entity.py",
                output="screen",
                arguments=["-topic", "robot_description", "-entity", "autonomous_robot"]
            )
        ]
    )

    # # Spawning the differential controller
    spawn_diff_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["diff_cont"]
    )

    # Spawning the joint board controller
    spawn_joint_board_controller = Node(
        package="controller_manager",
        executable="spawner",
        output="screen",
        arguments=["joint_broad"]
    )

    # RViz Launch
    rviz_launch = Node(
        package="rviz2",
        executable="rviz2",
        output="screen",
        arguments=["-d", LaunchConfiguration("rviz_config_path")],
        parameters=[{"use_sim_time": True}]
    )

    # Twist Mux changing the topic name, instead of /cmd_vel it will be /diff_cont/cmd_vel_unstamped 
    # and giving priority.
    twist_mux_params = os.path.join(get_package_share_directory('robot_description'),'config','twist_mux.yaml')
    twist_mux = Node(
            package="twist_mux",
            executable="twist_mux",
            parameters=[twist_mux_params, {'use_sim_time': True}],
            remappings=[('/cmd_vel_out','/diff_cont/cmd_vel_unstamped')]
        )


    return LaunchDescription([
        declare_urdf_path,
        declare_rviz_config_path,
        declare_use_warehouse,
        declare_use_small_house,
        declare_use_hospital,
        robot_state_publisher,
        gazebo_warehouse_launch,
        gazebo_small_house_launch,
        gazebo_hospital_launch,
        joint_state_publisher,
        spawn_robot_entity,
        spawn_diff_controller,
        spawn_joint_board_controller,
        rviz_launch,
        twist_mux
    ])