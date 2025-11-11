#!/bin/bash

# Launcher script for AV Simulator

echo "=========================================="
echo "AV Simulator - Autonomous Vehicle"
echo "Simulation and Vulnerability Analyzer"
echo "=========================================="
echo ""

# Check if Python 3 is installed
if ! command -v python3 &> /dev/null; then
    echo "Error: Python 3 is not installed."
    echo "Please install Python 3.8 or later."
    exit 1
fi

# Check if required packages are installed
echo "Checking dependencies..."

python3 -c "import PyQt5" 2>/dev/null
if [ $? -ne 0 ]; then
    echo "PyQt5 not found. Installing dependencies..."
    pip install -r requirements.txt --break-system-packages
fi

# Create output directories if they don't exist
mkdir -p /home/claude/csv_output
mkdir -p /home/claude/reports

echo "Starting application..."
echo ""

# Run the application
python3 av_simulator.py

echo ""
echo "Application closed."
