# AVSVA Quick Start Guide

## 🚀 Quick Installation (5 Minutes)

### Prerequisites Check
```bash
# Check if ROS is installed
rosversion -d

# Check Python version (need 3.6+)
python3 --version

# Check if you have PyQt5
python3 -c "import PyQt5; print('PyQt5 OK')"
```

### One-Command Install
```bash
cd ~/avsva_project
./install_avsva.sh
```

## 🎯 Quick Usage Guide

### Starting AVSVA
```bash
cd ~/avsva_project
./run_avsva.sh
```

### Your First Vulnerability Demo (2 Minutes)

1. **Start Simulation**
   - Click "▶ Start Simulation"
   - Wait ~30 seconds for Gazebo and RViz to load
   - You should see the Husky robot driving forward

2. **Inject Vulnerability**
   - Go to "Vulnerability Injection" tab
   - Click "Execute Attack" on "CMD_VEL Topic Injection"
   - Watch the robot stop and start spinning!

3. **Stop Attack**
   - Click "Stop Attack" to end the injection
   - Robot should return to normal forward motion

4. **Stop Simulation**
   - Go back to "Simulation Control" tab
   - Click "■ Stop Simulation"

## 📊 Recording a Demo

1. Start simulation
2. Click "● Start Recording"
3. Inject vulnerabilities
4. Click "■ Stop Recording"
5. Go to "Bag File Analysis" tab to view results

## 🎓 For Your Capstone Presentation

### Demo Flow (5 minutes)
1. **Introduction** (30 sec)
   - Show the AVSVA interface
   - Explain it's for ROS security research

2. **Baseline Operation** (1 min)
   - Start simulation
   - Show robot driving normally in Gazebo
   - Show sensor data in RViz

3. **Attack #1: CMD_VEL Injection** (1 min)
   - Go to vulnerability tab
   - Read the description out loud
   - Execute attack
   - Show robot losing control

4. **Attack #2: Node Shutdown** (1 min)
   - Stop first attack
   - Execute node shutdown attack
   - Show robot completely stopping
   - Explain denial of service

5. **Analysis** (1 min)
   - Show recorded bag file
   - Discuss attack indicators
   - Mention ROS-Defender paper

6. **Conclusion** (30 sec)
   - Emphasize lack of ROS security
   - Mention Army Research Lab partnership
   - Propose future work

## 🎬 Video Recording Tips

If recording your demo:
```bash
# Install SimpleScreenRecorder
sudo apt-get install simplescreenrecorder

# Or use OBS Studio
sudo apt-get install obs-studio
```

**Recording checklist:**
- [ ] Start screen recording
- [ ] Launch AVSVA
- [ ] Show clean interface
- [ ] Start simulation (use 2x speed in editing)
- [ ] Demonstrate 2-3 vulnerabilities
- [ ] Show bag analysis
- [ ] Stop simulation
- [ ] Stop recording

## 🐛 Common Issues & Quick Fixes

### "Simulation won't start"
```bash
# Kill any existing ROS processes
killall -9 roscore rosmaster gzserver gzclient rviz

# Try again
./run_avsva.sh
```

### "Can't find ROS"
```bash
# Source ROS manually
source /opt/ros/noetic/setup.bash

# Then run
python3 avsva_app.py
```

### "PyQt5 import error"
```bash
# Reinstall PyQt5
pip3 install --upgrade PyQt5 --user
```

### "Attack doesn't work"
- Make sure simulation is running first
- Check that Gazebo is fully loaded (robot visible)
- Try stopping and restarting the simulation

## 📝 Presentation Talking Points

### Why This Matters
- ROS used in military, industrial, and autonomous vehicles
- No built-in security features
- Real-world consequences of attacks

### What We Demonstrated
- 5 different attack vectors
- All require minimal code
- All exploit fundamental ROS design flaws

### ROS-Defender Connection
- Paper from SafeThings 2019
- Proposes SDN-based security
- Our tool validates their threat model
- Can be used to test their defenses

### Future Work
- Implement ROS-Defender protections
- Test against our attack suite
- Measure detection rates
- Propose additional mitigations

## 🔗 Important Commands

```bash
# List all ROS topics
rostopic list

# Monitor a specific topic
rostopic echo /husky_velocity_controller/cmd_vel

# Show ROS graph
rosrun rqt_graph rqt_graph

# Replay a bag file
rosbag play recorded_bags/husky_attack_*.bag

# Get bag info
rosbag info recorded_bags/husky_attack_*.bag
```

## 📧 Support

For issues or questions:
1. Check README.md for detailed docs
2. Review the ROS-Defender paper
3. Check ROS Wiki: http://wiki.ros.org
4. Consult with capstone advisor

## ✅ Pre-Presentation Checklist

- [ ] AVSVA installs and runs
- [ ] Can start simulation successfully
- [ ] All 5 attacks work
- [ ] Can record bag files
- [ ] Gazebo and RViz both display correctly
- [ ] Practiced demo flow (5 minutes)
- [ ] Have backup video recording
- [ ] PowerPoint/slides prepared
- [ ] Paper references ready

## 🎯 Key Metrics for Paper

If writing a paper, collect:
- Attack success rate (%)
- Time to compromise (seconds)
- Detection difficulty (manual observation)
- Impact severity (1-10 scale)
- Code complexity (lines of code)
- Required access level (network, node, API)

## 🏆 Demo Success Criteria

Your demo is successful if you can:
1. ✅ Launch simulation reliably
2. ✅ Execute at least 3 different attacks
3. ✅ Show visible impact on robot behavior
4. ✅ Explain why each vulnerability exists
5. ✅ Record and analyze bag files
6. ✅ Connect to ROS-Defender research

---

**Good luck with your capstone presentation! 🎓**

*Remember: This is for educational purposes. Never attack real robots!*
