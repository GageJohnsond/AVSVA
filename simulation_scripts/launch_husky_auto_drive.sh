#!/bin/bash
# launch_husky_auto_drive.sh
# One-click Husky simulation + RViz + auto forward drive (NO package needed)

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== Starting Husky Live Simulation + Auto Drive ===${NC}"

# Full path to the Python script (same folder as this .sh file)
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/auto_drive_forward.py"

# Kill everything on Ctrl+C
trap 'echo -e "${RED}Shutting down all processes...${NC}"; jobs -p | xargs -r kill; exit' SIGINT SIGTERM EXIT

# 1. Start roscore
echo -e "${YELLOW}Starting roscore...${NC}"
roscore &
sleep 5

# 2. Launch Gazebo + Husky in playpen
echo -e "${YELLOW}Launching Gazebo with Husky robot...${NC}"
roslaunch husky_gazebo empty_world.launch &
sleep 12

# 3. Launch RViz with perfect Husky config
echo -e "${YELLOW}Launching RViz (pre-configured view)...${NC}"
roslaunch husky_viz view_robot.launch &
sleep 5

# 4. Start auto-driving forward (this line changed!)
echo -e "${YELLOW}Starting auto-drive: Husky going straight at 0.5 m/s...${NC}"
python3 "$PYTHON_SCRIPT" __log:=husky_auto_drive linear_speed:=0.5 &

echo -e "${GREEN}=== EVERYTHING IS RUNNING ===${NC}"
echo -e "${GREEN}Husky is now driving straight forward automatically!${NC}"
echo -e "${GREEN}Watch it in Gazebo + live sensor data in RViz${NC}"
echo -e "Press Ctrl+C in this terminal to stop everything."

# Keep script running
wait