# AVSVA - Autonomous Vehicle Simulation and Vulnerability Analyzer

**Texas Tech University - Raider Security - CS Senior Capstone Project**

PyQt5 application for demonstrating and analyzing ROS security vulnerabilities using the Clearpath Husky robot in simulation.

## Features

- Launch Husky robot simulations (Gazebo/RViz) with one click
- Execute 5 types of ROS vulnerability attacks
- Record and analyze attack scenarios in bag files
- Custom injection tool for security research
- Configurable detection thresholds

## Prerequisites

- Ubuntu 20.04+ with ROS Noetic
- Python 3.6+
- Clearpath Husky simulation packages

```bash
sudo apt-get install ros-noetic-husky-simulator ros-noetic-husky-viz
pip install PyQt5 rospkg
```

## Installation

```bash
cd ~/
git clone <repository-url> avsva_project
cd avsva_project
chmod +x install_avsva.sh
./install_avsva.sh
```

Or manually:
```bash
chmod +x avsva_app.py simulation_scripts/*.sh simulation_scripts/*.py attack_scripts/*.py
source /opt/ros/noetic/setup.bash
```

## Usage

```bash
python3 avsva_app.py
```

### Quick Start

1. **Robot Simulation Tab**: Click "Start Simulation" to launch Gazebo + RViz + auto-drive
2. **Vulnerability Injection Tab**:
   - **Preset Attacks**: Execute pre-configured attacks
   - **Custom Injection**: Create custom attacks on any topic
3. **Analysis Tab**: Load bag files, analyze security threats with configurable detection
4. **Generate Report Tab**: Create and export security analysis reports

### Vulnerabilities

1. **CMD_VEL Topic Injection** - Hijacks robot control
2. **Odometry Spoofing** - Publishes false position data
3. **Node Shutdown** - Terminates critical nodes via XMLRPC
4. **Parameter Manipulation** - Modifies runtime parameters
5. **IMU Spoofing** - Injects false sensor data

## Project Structure

```
avsva_project/
├── avsva_app.py                   # Main application
├── simulation_scripts/            # Simulation launcher and auto-drive
├── attack_scripts/                # Attack implementations
├── recorded_bags/                 # Auto-created for recordings
└── README.md
```

## Troubleshooting

**Simulation won't start:**
```bash
killall -9 roscore gzserver gzclient rviz
source /opt/ros/noetic/setup.bash
```

**Attacks don't work:**
- Ensure simulation is running first
- Check: `rostopic list`

## Safety

⚠️ Educational/research use only. Never deploy on production robots.

## Authors

Raider Security Team
Texas Tech University Computer Science
Senior Capstone Project 2024-2025
In partnership with Army Research Lab

---

*For demonstration and educational purposes only.*
