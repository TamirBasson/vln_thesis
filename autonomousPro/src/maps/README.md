# 🗺️ Maps Package

ROS2 package for warehouse mapping with Cartographer + Nav2 navigation.

## 📁 Structure
```
maps/
├── package.xml                           # ROS2 package definition
├── CMakeLists.txt                        # Build configuration
├── launch/
│   ├── continue_mapping_sim_environment.launch.py  # Simulation mapping + Nav2
│   ├── continue_mapping_real_env.launch.py         # Real robot mapping + Nav2
│   ├── save_map_simulation_env.launch.py           # Save simulation map
│   └── save_map_real_env.launch.py                 # Save real robot map
└── maps/
    ├── real_env/                # Real robot maps
    │   ├── combined_scans.pgm
    │   ├── combined_scans.yaml
    │   └── combined_scans.pbstream
    └── simulation_env/          # Simulation maps
        ├── combined_scans.pgm
        ├── combined_scans.yaml
        └── combined_scans.pbstream
```

## 🚀 Usage

### 1. Simulation Environment
```bash
# Start simulation
ros2 launch robot_simulation sim_gazebo.launch.py

# Start mapping with Nav2 (in another terminal)
ros2 launch maps continue_mapping_sim_environment.launch.py

# Save simulation map
ros2 launch maps save_map_simulation_env.launch.py
```

### 2. Real Robot Environment
```bash
# Start real robot hardware (your robot launch file)

# Start mapping with localization (set initial pose)
ros2 launch maps continue_mapping_real_env.launch.py \
  initial_pose_x:=2.0 initial_pose_y:=1.5 initial_pose_yaw:=0.0

# Save real robot map
ros2 launch maps save_map_real_env.launch.py
```

## 🔄 Workflow
1. **Start simulation**: `ros2 launch robot_simulation sim_gazebo.launch.py`
2. **Start mapping + navigation**: `ros2 launch maps continue_mapping_environment.launch.py`
3. **Use RViz**: Set navigation goals with "2D Goal Pose" - robot will navigate while mapping
4. **Autonomous exploration**: Robot maps new areas while navigating to goals
5. **Save map**: `./quick_save_map.sh`
6. **Repeat** - always uses the same map name, automatically overwrites

## ✨ Features
- 🗺️ **Simultaneous Mapping & Navigation**: Update maps while navigating
- 🎯 **Goal-based Exploration**: Set goals in RViz to explore specific areas
- 🔄 **Continuous Updates**: Map gets refined with each navigation session
- 📍 **Path Planning**: Nav2 plans optimal paths through known areas

## 📝 Notes
- The map is always saved with the same name (`combined_scans`)
- Previous map data is automatically overwritten
- Map includes both occupancy grid (.pgm/.yaml) and Cartographer state (.pbstream)
- Use Ctrl+C to stop mapping before saving