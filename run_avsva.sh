#!/bin/bash
# AVSVA Launcher Script

# Source ROS
if [ -f "/opt/ros/noetic/setup.bash" ]; then
    source /opt/ros/noetic/setup.bash
elif [ -f "/opt/ros/melodic/setup.bash" ]; then
    source /opt/ros/melodic/setup.bash
fi

# Get script directory
SCRIPT_DIR="$( cd "$( dirname "${BASH_SOURCE[0]}" )" && pwd )"
cd "$SCRIPT_DIR"

# Run AVSVA
python3 avsva_app.py
