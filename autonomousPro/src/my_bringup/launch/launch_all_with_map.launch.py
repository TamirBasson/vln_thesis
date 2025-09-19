from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
import os
from ament_index_python.packages import get_package_share_directory

def generate_launch_description():

    mpu9250_launch = os.path.join(
        get_package_share_directory('robot_mpu9250driver'), 'launch', 'mpu9250driver_launch.py')

    lslidar_launch = os.path.join(
        get_package_share_directory('lslidar_driver'), 'launch', 'lslidar_cx_launch.py')

    control_launch = os.path.join(
        get_package_share_directory('robot_control_hardware'), 'launch', 'control_system.launch.py')

    cartographer_launch = os.path.join(
        get_package_share_directory('maps'), 'launch', 'continue_mapping_real_env.launch.py')

    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        )
    )
    

    return LaunchDescription([
        # Step 1: Start MPU9250
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(mpu9250_launch)
        ),

        # Step 2: Wait 3s, then start LSLiDAR
        TimerAction(
            period=3.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(lslidar_launch)
                )
            ]
        ),

        # Step 3: Wait another 3s, then start control system
        TimerAction(
            period=6.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(control_launch)
                )
            ]
        ),

        # Step 4: Wait another 4s, then start RealSense
        TimerAction(
            period=10.0,
            actions=[
                realsense_launch
            ]
        ),

        # Step 5: Wait another 5s, then start Cartographer
        TimerAction(
            period=15.0,
            actions=[
                IncludeLaunchDescription(
                    PythonLaunchDescriptionSource(cartographer_launch)
                )
            ]
        ),

        # Step 6: Wait another 5s, then start Nav Goal Pose Node
        # (Navigation is now included in the Cartographer launch above)
        TimerAction(
            period=20.0,
            actions=[
                ExecuteProcess(
                    cmd=['ros2', 'run', 'my_bringup', 'my_nav_goalpose_node'],
                    output='screen'
                )
            ]
        ),
    ])
