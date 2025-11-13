#!/bin/bash

# AV Simulator - ROS Integration Quick Setup and Launch Script

echo "=========================================="
echo "AV Simulator with ROS Integration"
echo "Quick Setup and Launch"
echo "=========================================="
echo ""

# Check for ROS installation
if [ -z "$ROS_DISTRO" ]; then
    echo "⚠️  ROS environment not detected"
    echo ""
    echo "Attempting to source ROS..."
    
    if [ -f "/opt/ros/noetic/setup.bash" ]; then
        source /opt/ros/noetic/setup.bash
        echo "✓ ROS Noetic sourced"
    elif [ -f "/opt/ros/melodic/setup.bash" ]; then
        source /opt/ros/melodic/setup.bash
        echo "✓ ROS Melodic sourced"
    else
        echo "❌ ROS not found!"
        echo ""
        echo "Please install ROS first:"
        echo "  sudo apt install ros-noetic-desktop-full"
        echo ""
        echo "Or see ROS_SETUP_GUIDE.md for detailed instructions"
        echo ""
        read -p "Continue without ROS? (limited functionality) [y/N]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Yy]$ ]]; then
            exit 1
        fi
    fi
else
    echo "✓ ROS environment detected: $ROS_DISTRO"
fi

echo ""

# Check Python version
PYTHON_VERSION=$(python3 --version 2>&1 | awk '{print $2}')
echo "Python version: $PYTHON_VERSION"

if python3 -c 'import sys; exit(0 if sys.version_info >= (3, 8) else 1)' 2>/dev/null; then
    echo "✓ Python version OK"
else
    echo "❌ Python 3.8+ required"
    exit 1
fi

echo ""

# Check if dependencies are installed
echo "Checking dependencies..."

python3 -c "import PyQt5" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "📦 Installing Python dependencies..."
    pip install -r requirements_ros.txt --break-system-packages || pip install -r requirements_ros.txt
fi

echo "✓ Dependencies checked"
echo ""

# Create output directories
mkdir -p /home/claude/csv_output
mkdir -p /home/claude/reports
echo "✓ Output directories created"
echo ""

# Check for ROS tools
echo "Checking ROS tools..."

if command -v roscore &> /dev/null; then
    echo "  ✓ roscore found"
else
    echo "  ⚠️  roscore not found (ROS may not be installed)"
fi

if command -v rviz &> /dev/null; then
    echo "  ✓ rviz found"
else
    echo "  ⚠️  rviz not found"
fi

if command -v gazebo &> /dev/null; then
    echo "  ✓ gazebo found"
else
    echo "  ⚠️  gazebo not found"
fi

echo ""

# Offer to start roscore
if command -v roscore &> /dev/null; then
    # Check if roscore is already running
    if rostopic list &> /dev/null; then
        echo "✓ roscore is already running"
    else
        read -p "Start roscore in background? [Y/n]: " -n 1 -r
        echo
        if [[ ! $REPLY =~ ^[Nn]$ ]]; then
            roscore &
            ROSCORE_PID=$!
            echo "✓ roscore started (PID: $ROSCORE_PID)"
            sleep 2
        fi
    fi
fi

echo ""
echo "=========================================="
echo "Starting AV Simulator..."
echo "=========================================="
echo ""

# Launch the application
python3 av_simulator_ros.py

# Cleanup on exit
echo ""
echo "Application closed."

if [ ! -z "$ROSCORE_PID" ]; then
    echo "Stopping roscore..."
    kill $ROSCORE_PID 2>/dev/null
fi

echo "Done."
