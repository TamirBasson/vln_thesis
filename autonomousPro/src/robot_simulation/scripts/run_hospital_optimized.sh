#!/bin/bash

# Gazebo Hospital World Performance Optimization Script
# This script sets optimized environment variables for better performance

echo "Starting Gazebo Hospital World with Performance Optimizations..."

# Set Gazebo performance environment variables
export GAZEBO_MODEL_DATABASE_URI=""  # Disable online model database for faster loading
export GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH:/home/tamir/thesis/autonomousPro/install/aws_robomaker_hospital_world/share/aws_robomaker_hospital_world
export GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH:/home/tamir/thesis/autonomousPro/install/aws_robomaker_hospital_world/lib

# Physics optimization
export GAZEBO_PHYSICS_ENGINE="ode"
export ODE_MAX_CONTACTS="20"  # Reduce contact calculations

# Memory and CPU optimization
export GAZEBO_MASTER_URI=http://localhost:11345
export GAZEBO_MODEL_PATH=$GAZEBO_MODEL_PATH:/home/tamir/thesis/autonomousPro/install/aws_robomaker_hospital_world/share/aws_robomaker_hospital_world/models

# GPU optimization (if available)
export __GL_SYNC_TO_VBLANK=0  # Disable VSync for better performance
export __GL_THREADED_OPTIMIZATIONS=1  # Enable GPU threading

# Reduce console output for faster startup
export GAZEBO_VERBOSE=0

echo "Environment variables set. Starting launch..."
echo "Note: First load may take 30-60 seconds due to complex models"

# Source ROS2 environment
source /home/tamir/thesis/autonomousPro/install/setup.bash

# Launch with optimizations
ros2 launch robot_simulation sim_gazebo.launch.py use_hospital:=true

echo "Gazebo Hospital World launched."
