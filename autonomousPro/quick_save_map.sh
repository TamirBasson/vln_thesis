#!/bin/bash
# Quick map saving script

echo "🗺️  Saving current map..."

# Source ROS2 environment
source /opt/ros/humble/setup.bash
source /home/tamir/autonomousPro/install/setup.bash

# Launch the save map functionality (simulation environment)
ros2 launch maps save_map_simulation_env.launch.py

echo "✅ Map saved!"