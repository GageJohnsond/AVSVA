#!/bin/bash

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

echo -e "${GREEN}=== Starting Husky Live Simulation + Auto Drive ===${NC}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PYTHON_SCRIPT="$SCRIPT_DIR/auto_drive_forward.py"

trap 'echo -e "${RED}Shutting down all processes...${NC}"; jobs -p | xargs -r kill; exit' SIGINT SIGTERM EXIT

echo -e "${YELLOW}Starting roscore...${NC}"
roscore &
sleep 5

echo -e "${YELLOW}Launching Gazebo with Husky robot...${NC}"
roslaunch husky_gazebo empty_world.launch &
sleep 12

echo -e "${YELLOW}Launching RViz...${NC}"
roslaunch husky_viz view_robot.launch &
sleep 5

echo -e "${YELLOW}Starting auto-drive: Husky going straight at 0.5 m/s...${NC}"
python3 "$PYTHON_SCRIPT" __log:=husky_auto_drive linear_speed:=0.5 &

echo -e "${GREEN}=== EVERYTHING IS RUNNING ===${NC}"
echo -e "${GREEN}Husky is now driving straight forward automatically!${NC}"
echo -e "${GREEN}Watch it in Gazebo + live sensor data in RViz${NC}"
echo -e "Press Ctrl+C in this terminal to stop everything."

wait