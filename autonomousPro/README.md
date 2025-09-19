# Autonomous Robot Navigation System

This project implements an autonomous robot navigation system with real-time pose tracking and environment mapping capabilities. The system uses ROS2 for robot control and includes various components for sensor integration, navigation, and visualization.

## Project Structure

```
.
├── src/
│   ├── my_bringup/                 # Main launch and configuration files
│   │   ├── my_bringup/            # Python modules
│   │   └── launch/                # Launch files
│   ├── robot_description/         # Robot URDF and meshes
│   │   ├── urdf/                  # URDF files
│   │   ├── meshes/               # 3D models
│   │   └── rviz/                 # RViz configurations
│   ├── robot_control_hardware/    # Hardware control interfaces
│   │   ├── src/                  # Source code
│   │   └── include/              # Header files
│   ├── robot_navigation/         # Navigation algorithms
│   │   ├── config/              # Navigation parameters
│   │   └── launch/              # Navigation launch files
│   ├── robot_simulation/        # Simulation environment
│   │   ├── worlds/             # Gazebo world files
│   │   └── launch/             # Simulation launch files
│   ├── robot_rs_camera_driver/  # RealSense camera driver
│   ├── robot_mpu9250_driver/    # IMU driver
│   ├── robot_Lslidar_ROS2_driver/ # LiDAR driver
│   └── common_interface/        # Common interfaces and utilities
├── image_data/                  # Captured images from the robot
├── poses.csv                    # Logged pose data
└── log/                        # System logs
```

## Key Components

### 1. Robot Description
- URDF models and meshes
- RViz configurations
- Robot visualization tools

### 2. Hardware Integration
- RealSense Camera: For visual perception
- MPU9250 IMU: For orientation tracking
- LiDAR: For environment mapping
- Custom hardware control interfaces

### 3. Navigation System
- Path planning algorithms
- Localization components
- Environment mapping
- Collision avoidance

### 4. Simulation Environment
- Gazebo world files
- Robot simulation models
- Test scenarios

## Installation

1. Clone the repository:
```bash
git clone [repository-url]
```

2. Install dependencies:
```bash
cd autonomousPro
rosdep install --from-paths src --ignore-src -r -y
```

3. Build the workspace:
```bash
colcon build
```

## Usage

### Simulation Mode
1. Launch the simulation environment:
```bash
ros2 launch robot_simulation sim_gazebo.launch.py
```

2. Launch the mapping system:
```bash
ros2 launch robot_simulation cartographer_sim.launch.py
```

3. Launch the navigation system:
```bash
ros2 launch robot_simulation navigation.launch.py
```

### Real Robot Mode
1. Connect to the robot hardware:
```bash
ros2 launch robot_control_hardware control_system.launch.py
```

2. Launch the mapping system:
```bash
ros2 launch robot_navigation cartographer_real.launch.py
```

3. Launch the navigation system:
```bash
ros2 launch robot_navigation navigation.launch.py
```

### Common Commands
- Start mapping (simulation):
```bash
ros2 launch robot_simulation cartographer_sim.launch.py
```

- Start mapping (real robot):
```bash
ros2 launch robot_navigation cartographer_real.launch.py
```

- Start navigation:
```bash
ros2 launch robot_navigation navigation.launch.py
```

- View occupancy grid:
```bash
ros2 launch robot_navigation occupancy_grid.launch.py
```

## Configuration

### Navigation Parameters
- Navigation parameters can be found in `robot_navigation/config/`
- Adjust parameters based on your environment and robot capabilities

### Hardware Settings
- Hardware-specific settings are in respective driver packages
- Configure according to your hardware setup

### Simulation Settings
- World files and robot models in `robot_simulation/worlds/`
- Adjust simulation parameters as needed

## Dependencies

- ROS2 (Humble)
- Python 3.8+
- OpenCV
- NetworkX
- NumPy
- Matplotlib
- Pandas
- SciPy
- Gazebo (for simulation)

## Contributing

1. Fork the repository
2. Create a feature branch
3. Commit your changes
4. Push to the branch
5. Create a Pull Request

## License

[Specify your license here]

## Contact

[Your contact information] 