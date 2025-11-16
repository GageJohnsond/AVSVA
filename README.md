# AVSVA - Autonomous Vehicle Simulation and Vulnerability Analyzer

**Texas Tech University - Raider Security - CS Senior Capstone Project**

A comprehensive PyQt5-based application for demonstrating and analyzing ROS (Robot Operating System) security vulnerabilities using the Clearpath Husky robot in simulation.

## Overview

AVSVA provides a graphical interface for:
- Launching and controlling Husky robot simulations in Gazebo/RViz
- Injecting various security vulnerabilities into live simulations
- Recording ROS bag files of attack scenarios
- Analyzing recorded data for security research

## Features

### 🎮 Simulation Control
- One-click launch of Gazebo, RViz, and autonomous driving
- Real-time system log monitoring
- Clean shutdown of all simulation processes

### ⚠️ Vulnerability Injection
Five categories of ROS vulnerabilities with detailed explanations:

1. **CMD_VEL Topic Injection** (Critical)
   - Hijacks robot control by publishing competing velocity commands
   - Demonstrates lack of topic authentication

2. **Odometry Sensor Spoofing** (High)
   - Publishes false position and velocity data
   - Misleads navigation and localization systems

3. **Node Shutdown Attack** (Critical)
   - Uses XMLRPC API to remotely terminate critical nodes
   - Demonstrates denial of service vulnerability

4. **Parameter Manipulation** (Medium)
   - Modifies runtime parameters like speed
   - Shows lack of parameter server access control

5. **IMU Data Spoofing** (High)
   - Injects false inertial measurement data
   - Disrupts orientation and balance systems

### 📊 Bag File Analysis
- Record attack scenarios for later analysis
- Load and inspect bag files with rosbag info
- Attack indicator detection
- Export data for research papers

## Prerequisites

### System Requirements
- Ubuntu 20.04 or later (or compatible Linux distribution)
- ROS Noetic (or ROS Melodic)
- Python 3.6+
- Clearpath Husky simulation packages

### ROS Packages Required
```bash
sudo apt-get update
sudo apt-get install ros-noetic-husky-simulator
sudo apt-get install ros-noetic-husky-viz
sudo apt-get install ros-noetic-gazebo-ros
sudo apt-get install ros-noetic-rviz
```

### Python Dependencies
```bash
pip install PyQt5
pip install rospkg
# ROS Python packages are typically installed with ROS
```

## Installation

1. **Clone or download the AVSVA files:**
```bash
cd ~/
mkdir avsva_project
cd avsva_project
```

2. **Place all files in the project directory with the following structure:**
   - `avsva_app.py` (main application)
   - `simulation_scripts/` (simulation files)
     - `auto_drive_forward.py` (autonomous driving node)
     - `launch_husky_auto_drive.sh` (simulation launcher)
   - `attack_scripts/` (attack demonstration files)
     - `attack_cmd_vel.py`
     - `attack_odom.py`
     - `attack_shutdown.py`
     - `attack_param.py`
     - `attack_imu.py`

3. **Make scripts executable:**
```bash
chmod +x avsva_app.py
chmod +x simulation_scripts/launch_husky_auto_drive.sh
chmod +x simulation_scripts/auto_drive_forward.py
chmod +x attack_scripts/attack_*.py
```

4. **Source your ROS environment:**
```bash
source /opt/ros/noetic/setup.bash
# Add to ~/.bashrc for persistence
echo "source /opt/ros/noetic/setup.bash" >> ~/.bashrc
```

## Usage

### Starting the Application

1. **Open a terminal and navigate to the project directory:**
```bash
cd ~/avsva_project
```

2. **Run the AVSVA application:**
```bash
python3 avsva_app.py
```

### Using the Simulation Tab

1. Click **"▶ Start Simulation"** to launch:
   - ROS Core
   - Gazebo with Husky robot in empty world
   - RViz with robot visualization
   - Autonomous forward-driving node

2. Monitor the system log for status updates

3. (Optional) Click **"● Start Recording"** to record a bag file

4. Click **"■ Stop Simulation"** when finished

### Injecting Vulnerabilities

1. Navigate to the **"⚠️ Vulnerability Injection"** tab

2. **Read the vulnerability descriptions** to understand:
   - What the attack does
   - Why it's a vulnerability
   - Expected impact

3. **Ensure simulation is running** before executing attacks

4. Click **"Execute Attack"** on any vulnerability card to inject it

5. **Observe the effects** in Gazebo and RViz:
   - CMD_VEL: Robot stops and spins
   - Odometry: Navigation confusion
   - Shutdown: Robot stops completely
   - Parameter: Speed changes
   - IMU: Orientation issues

6. Click **"Stop Attack"** to end the injection

### Analyzing Bag Files

1. Navigate to the **"📊 Bag File Analysis"** tab

2. Click **"🔄 Refresh List"** to see recorded bags

3. Click on a bag file to analyze it

4. Review the analysis output showing:
   - Bag file metadata
   - Topics recorded
   - Message counts
   - Duration
   - Attack indicators

5. Use **"📂 Load External Bag File"** to analyze bags from other sources

## Project Structure

```
avsva_project/
├── avsva_app.py                   # Main PyQt5 application
├── simulation_scripts/            # Simulation files
│   ├── launch_husky_auto_drive.sh # Simulation launcher script
│   └── auto_drive_forward.py      # Autonomous driving node
├── attack_scripts/                # Attack demonstration files
│   ├── attack_cmd_vel.py          # CMD_VEL injection attack
│   ├── attack_odom.py             # Odometry spoofing attack
│   ├── attack_shutdown.py         # Node shutdown attack
│   ├── attack_param.py            # Parameter manipulation attack
│   └── attack_imu.py              # IMU spoofing attack
├── recorded_bags/                 # Auto-created for bag files
└── README.md                      # This file
```

## Technical Details

### ROS Topics Used
- `/husky_velocity_controller/cmd_vel` - Velocity commands
- `/husky_velocity_controller/odom` - Odometry data
- `/imu/data` - Inertial measurement data
- `/tf` - Transform data
- `/rosout` - System logs

### Attack Mechanisms

**CMD_VEL Injection:**
- Publishes at 30Hz (faster than legitimate 20Hz)
- Sends conflicting stop and spin commands
- Demonstrates topic hijacking

**Odometry Spoofing:**
- Publishes false position (-10, 5)
- Fake backward velocity (-0.5 m/s)
- Shows sensor data vulnerability

**Node Shutdown:**
- Uses XMLRPC API calls
- Targets /husky_auto_drive node
- Demonstrates DoS attack

**Parameter Manipulation:**
- Modifies /husky_auto_drive/linear_speed
- Tests with negative and excessive values
- Shows configuration vulnerability

**IMU Spoofing:**
- Publishes at 100Hz
- False angular velocity (10 rad/s)
- Impossible acceleration (50 m/s²)

## Troubleshooting

### Simulation Won't Start
- Ensure ROS is properly sourced: `source /opt/ros/noetic/setup.bash`
- Check if roscore is already running: `ps aux | grep ros`
- Kill existing ROS processes: `killall -9 roscore rosmaster gzserver gzclient rviz`

### Attacks Don't Work
- Verify simulation is running before executing attacks
- Check topic names with: `rostopic list`
- Ensure Python scripts are executable
- Check ROS_MASTER_URI: `echo $ROS_MASTER_URI`

### Bag Recording Fails
- Ensure simulation is running
- Check disk space: `df -h`
- Verify rosbag is installed: `which rosbag`
- Check write permissions in recorded_bags/

### GUI Issues
- Ensure PyQt5 is installed: `pip show PyQt5`
- Update PyQt5: `pip install --upgrade PyQt5`
- Check for display issues: `echo $DISPLAY`

## Research Applications

### Capstone Project Demonstration
1. Show baseline autonomous operation
2. Inject vulnerabilities one at a time
3. Document behavioral changes
4. Analyze recorded data
5. Propose mitigation strategies

### Academic Paper Support
- Generates reproducible attack scenarios
- Records data for statistical analysis
- Provides visual demonstrations
- Documents ROS security vulnerabilities

### Defense Mechanism Testing
- Test ROS-Defender or similar security tools
- Compare behavior with/without protections
- Measure detection rates
- Evaluate mitigation effectiveness

## Connection to ROS-Defender Paper

This application demonstrates vulnerabilities discussed in:
**"ROS-Defender: SDN-based Security Policy Enforcement for Robotic Applications"**
by Rivera et al.

The paper proposes using Software-Defined Networking (SDN) to:
- Monitor ROS communication
- Enforce access control policies
- Detect anomalous behavior
- React to compromised nodes

AVSVA can be used to:
1. Demonstrate the vulnerabilities ROS-Defender addresses
2. Generate attack data for testing defense mechanisms
3. Compare secured vs unsecured ROS systems
4. Validate detection algorithms

## Safety Considerations

⚠️ **Important:**
- This tool is for educational and research purposes only
- Only use in controlled simulation environments
- Never deploy attacks on production robots
- Understand ethical implications of security research
- Follow responsible disclosure practices

## Future Enhancements

Planned features:
- [ ] Real-time plotting of attack effects
- [ ] Automated vulnerability scanning
- [ ] ROS-Defender integration
- [ ] Multiple robot support
- [ ] Network traffic visualization
- [ ] Statistical analysis tools
- [ ] Report generation
- [ ] SROS (Secure ROS) compatibility testing

## Contributing

This is a capstone project for Texas Tech University. For questions or collaboration:
- Review code and suggest improvements
- Report bugs or issues
- Propose new vulnerability demonstrations
- Share research findings

## References

1. Rivera, S., et al. "ROS-Defender: SDN-based Security Policy Enforcement for Robotic Applications." SafeThings 2019.

2. Clearpath Robotics. "Husky UGV - Outdoor Field Research Robot." https://clearpathrobotics.com/husky/

3. Quigley, M., et al. "ROS: an open-source Robot Operating System." ICRA Workshop 2009.

4. White, R., et al. "SROS: Securing ROS over the wire, in the graph, and through the kernel." arXiv:1611.07060, 2016.

## License

This software is provided for educational purposes as part of an academic capstone project at Texas Tech University.

## Authors

**Raider Security Team**
Texas Tech University
Computer Science Department
Senior Capstone Project 2024-2025

In partnership with Army Research Lab

---

*For demonstration and educational purposes only. Not for use in production environments.*
