#!/bin/bash

# Launch Gazebo with Husky robot model
# This script sets up the Gazebo environment for AV simulation

echo "Launching Gazebo with Husky robot..."

# Source ROS environment
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
else
    echo "Error: ROS not found. Please install ROS first."
    exit 1
fi

# Check if Husky simulation packages are installed
if ! rospack find husky_gazebo &> /dev/null; then
    echo "Warning: husky_gazebo package not found."
    echo "To install: sudo apt-get install ros-noetic-husky-simulator"
fi

# Launch Gazebo with empty world
roslaunch gazebo_ros empty_world.launch &

# Wait for Gazebo to start
sleep 5

# Spawn Husky robot
if rospack find husky_gazebo &> /dev/null; then
    roslaunch husky_gazebo spawn_husky.launch &
else
    echo "Launching basic Gazebo environment without Husky model"
fi

echo "Gazebo launched successfully!"
echo "You can now play bag files to see the robot move."