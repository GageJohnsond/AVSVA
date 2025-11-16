# AVSVA System Architecture & Workflow

## System Architecture Diagram

```
┌─────────────────────────────────────────────────────────────────────┐
│                         AVSVA Application (PyQt5)                   │
│                                                                     │
│  ┌──────────────┐  ┌──────────────┐  ┌────────────────────────┐  │
│  │ Simulation   │  │ Vulnerability│  │  Bag File Analysis     │  │
│  │   Control    │  │  Injection   │  │                        │  │
│  │     Tab      │  │     Tab      │  │      Tab               │  │
│  └──────┬───────┘  └──────┬───────┘  └──────┬─────────────────┘  │
│         │                  │                  │                     │
└─────────┼──────────────────┼──────────────────┼─────────────────────┘
          │                  │                  │
          ▼                  ▼                  ▼
┌─────────────────┐ ┌──────────────────┐ ┌──────────────┐
│ Simulation      │ │ Vulnerability    │ │ BagRecorder  │
│ Controller      │ │ Executor (x5)    │ │   Thread     │
│   Thread        │ │   Threads        │ │              │
└────────┬────────┘ └────────┬─────────┘ └──────┬───────┘
         │                   │                   │
         │                   │                   │
         ▼                   ▼                   ▼
┌─────────────────────────────────────────────────────────┐
│                      ROS Environment                    │
│                                                         │
│  ┌──────────┐  ┌──────────┐  ┌──────────┐  ┌────────┐│
│  │ roscore  │  │  Gazebo  │  │   RViz   │  │ rosbag ││
│  └─────┬────┘  └─────┬────┘  └─────┬────┘  └────┬───┘│
│        │             │              │             │    │
│  ┌─────▼─────────────▼──────────────▼─────────────▼──┐│
│  │              ROS Communication Layer               ││
│  │      (Topics, Services, Parameters, XMLRPC)       ││
│  └───────────────────────────────────────────────────┘│
│                          │                             │
│  ┌───────────────────────▼──────────────────────────┐ │
│  │              Robot Control Stack                  │ │
│  │  ┌──────────────┐  ┌──────────────────────────┐ │ │
│  │  │auto_drive.py │  │  /cmd_vel  /odom  /imu   │ │ │
│  │  │    Node      │  │       Topics              │ │ │
│  │  └──────────────┘  └──────────────────────────┘ │ │
│  └───────────────────────────────────────────────────┘ │
└─────────────────────────────────────────────────────────┘
                          │
                          ▼
              ┌───────────────────────┐
              │    Husky Robot        │
              │   (in Gazebo)         │
              └───────────────────────┘
```

## Data Flow Diagrams

### 1. Normal Operation (No Attack)

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ auto_drive   │────▶│  /cmd_vel    │────▶│    Husky     │
│   Node       │     │    Topic     │     │   Robot      │
└──────────────┘     └──────────────┘     └──────┬───────┘
                                                  │
                                                  ▼
                                          ┌──────────────┐
                                          │   Sensors    │
                                          │ /odom /imu   │
                                          └──────────────┘
```

### 2. CMD_VEL Attack (Topic Hijacking)

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│ auto_drive   │────▶│              │     │    Husky     │
│   Node       │     │  /cmd_vel    │◀────│   Robot      │
│ (legitimate) │     │    Topic     │     │  (confused)  │
└──────────────┘     │ (contested!) │     └──────────────┘
                     │              │
┌──────────────┐     │              │
│ attack_cmd   │────▶│              │
│  _vel.py     │     └──────────────┘
│ (malicious)  │      ▲   ▲   ▲   ▲
└──────────────┘      │   │   │   │
                  30Hz │   │   │   │ Attacker publishes
                       │   │   │   │ faster, dominates
```

### 3. Odometry Spoofing Attack

```
┌──────────────┐                    ┌──────────────┐
│    Husky     │                    │ Navigation   │
│   Sensors    │                    │    Stack     │
│ (real data)  │                    │  (confused)  │
└──────┬───────┘                    └──────▲───────┘
       │                                    │
       │  Real Odom                 Fake Odom │
       ▼                                    │
┌──────────────┐                    ┌──────────────┐
│  /odom Topic │                    │attack_odom.py│
│ (contested!) │◀───────────────────│ (spoofing)   │
└──────────────┘                    └──────────────┘
       │
       │ Robot thinks it's at (-10, 5)
       │ Actually at (0, 0)
       ▼
   Wrong decisions!
```

### 4. Node Shutdown Attack

```
┌──────────────┐     ┌──────────────┐     ┌──────────────┐
│auto_drive.py │     │ ROS Master   │     │attack_shutdown│
│    Node      │     │  (XMLRPC)    │     │    .py       │
└──────┬───────┘     └──────┬───────┘     └──────┬───────┘
       │                    │                     │
       │ 1. Registered      │                     │
       │◀───────────────────┤                     │
       │                    │                     │
       │                    │ 2. Lookup node      │
       │                    │◀────────────────────┤
       │                    │                     │
       │                    │ 3. Return URI       │
       │                    ├────────────────────▶│
       │                    │                     │
       │ 4. SHUTDOWN!       │                     │
       │◀───────────────────┴─────────────────────┤
       │                                          │
       ▼                                          │
   [DEAD]                                         │
                                            [SUCCESS]
```

### 5. Bag Recording Flow

```
┌─────────────────────────────────────────────────┐
│            AVSVA BagRecorder Thread             │
└──────────────────────┬──────────────────────────┘
                       │
                       │ rosbag record -O file.bag
                       ▼
            ┌──────────────────────┐
            │  Topics to Record:   │
            │  • /cmd_vel          │
            │  • /odom             │
            │  • /imu/data         │
            │  • /tf               │
            │  • /rosout           │
            └──────────┬───────────┘
                       │
                       ▼
            ┌──────────────────────┐
            │  husky_attack_       │
            │  20241114_153045.bag │
            └──────────┬───────────┘
                       │
                       │ rosbag info
                       ▼
            ┌──────────────────────┐
            │  Bag File Analysis   │
            │  • Duration: 45.2s   │
            │  • Messages: 2,341   │
            │  • Size: 12.4 MB     │
            └──────────────────────┘
```

## Attack Execution Timeline

### CMD_VEL Injection Attack

```
Time: 0s ─────────────────────────────────────────────▶ 10s
      │                                                   │
      ├─ AVSVA: User clicks "Execute Attack"             │
      │                                                   │
    0.1s ─ Thread: VulnerabilityExecutor starts          │
      │                                                   │
    0.2s ─ ROS: rospy.init_node('attacker')             │
      │                                                   │
    0.3s ─ ROS: Create publisher on /cmd_vel            │
      │                                                   │
    0.5s ─ Attack: Start publishing at 30Hz             │
      │    │                                             │
      │    ├─ 0.5s: Pub msg #1 (stop + spin)            │
      │    ├─ 0.53s: Pub msg #2                         │
      │    ├─ 0.56s: Pub msg #3                         │
      │    └─ ... every 33ms ...                        │
      │                                                   │
    1.0s ─ Gazebo: Robot starts spinning                │
      │                                                   │
    5.0s ─ User: Observes spinning behavior             │
      │                                                   │
    8.0s ─ AVSVA: User clicks "Stop Attack"             │
      │                                                   │
    8.1s ─ Thread: VulnerabilityExecutor.stop()         │
      │                                                   │
    8.2s ─ ROS: Node shutdown                           │
      │                                                   │
    9.0s ─ Gazebo: Robot resumes forward motion         │
      │                                                   │
```

## Thread Communication Architecture

```
┌─────────────────────────────────────────────────────┐
│              Main UI Thread (PyQt5)                 │
│  ┌───────────┐  ┌───────────┐  ┌───────────┐      │
│  │  Start    │  │  Execute  │  │   Stop    │      │
│  │  Button   │  │  Button   │  │  Button   │      │
│  └─────┬─────┘  └─────┬─────┘  └─────┬─────┘      │
└────────┼──────────────┼──────────────┼─────────────┘
         │              │              │
         │ .start()     │ .start()     │ .stop()
         ▼              ▼              ▼
┌─────────────┐  ┌─────────────┐  ┌─────────────┐
│ Simulation  │  │Vulnerability│  │ BagRecorder │
│   Thread    │  │   Thread    │  │   Thread    │
└──────┬──────┘  └──────┬──────┘  └──────┬──────┘
       │                │                │
       │ log_signal     │ log_signal     │ log_signal
       ▼                ▼                ▼
┌─────────────────────────────────────────────────────┐
│           UI Log Display (QTextEdit)                │
│  [12:34:56] Starting simulation...                  │
│  [12:34:58] Gazebo launched                         │
│  [12:35:02] Attack started                          │
│  [12:35:10] Attack stopped                          │
└─────────────────────────────────────────────────────┘
```

## State Machine Diagram

### Simulation State Machine

```
                    ┌─────────┐
                    │ STOPPED │
                    └────┬────┘
                         │
              Start Sim  │
                         ▼
                    ┌─────────┐
              ┌────▶│STARTING │
              │     └────┬────┘
              │          │
   Fail       │          │ Success
   Retry      │          ▼
              │     ┌─────────┐      Stop Sim
              └─────│ RUNNING │─────────────┐
                    └────┬────┘             │
                         │                  │
                         │ Can Record       │
                         ▼                  │
                    ┌─────────┐             │
              ┌────▶│RECORDING│             │
              │     └────┬────┘             │
              │          │                  │
   Continue   │          │ Stop Rec         │
   Recording  │          ▼                  │
              │     ┌─────────┐             │
              └─────│ RUNNING │             │
                    └─────────┘             │
                         │                  │
                         ▼                  ▼
                    ┌──────────┐      ┌─────────┐
                    │ STOPPING │─────▶│ STOPPED │
                    └──────────┘      └─────────┘
```

### Attack State Machine

```
                    ┌─────────┐
                    │  IDLE   │
                    └────┬────┘
                         │
             Execute     │
                         ▼
                    ┌─────────┐
                    │STARTING │
                    └────┬────┘
                         │
                         │ ROS Node Init
                         ▼
                    ┌─────────┐
                    │ ACTIVE  │◀───┐
                    │Publishing│   │
                    └────┬────┘    │
                         │         │
                         │ Loop    │
                         └─────────┘
                         │
              Stop       │
                         ▼
                    ┌─────────┐
                    │STOPPING │
                    └────┬────┘
                         │
                         │ Cleanup
                         ▼
                    ┌─────────┐
                    │  IDLE   │
                    └─────────┘
```

## File System Layout

```
~/avsva_project/
│
├── avsva_app.py              # Main application (45 KB)
│
├── simulation_scripts/       # Simulation files
│   ├── launch_husky_auto_drive.sh  # Simulation launcher
│   └── auto_drive_forward.py       # Autonomous driving node
│
├── attack_scripts/           # Attack demonstration files
│   ├── attack_cmd_vel.py     # CMD_VEL injection
│   ├── attack_odom.py        # Odometry spoofing
│   ├── attack_shutdown.py    # Node shutdown
│   ├── attack_param.py       # Parameter manipulation
│   └── attack_imu.py         # IMU spoofing
│
├── install_avsva.sh          # Installation script
├── run_avsva.sh              # Launcher (auto-generated)
│
├── README.md                 # Full documentation
├── QUICKSTART.md             # Quick reference
├── PROJECT_SUMMARY.md        # This document
│
└── recorded_bags/            # Auto-created directory
    ├── husky_attack_20241114_153045.bag
    ├── husky_attack_20241114_154312.bag
    └── ...
```

## Network Topology

```
┌────────────────────────────────────────────────────┐
│            ROS Master (XMLRPC Server)              │
│              URI: http://localhost:11311/          │
└──────┬─────────────────────────────────────┬───────┘
       │                                     │
       │ Register/Lookup                    │ Register/Lookup
       ▼                                     ▼
┌─────────────────┐                  ┌─────────────────┐
│ auto_drive.py   │                  │ attack_*.py     │
│ Port: 43210     │◀────/cmd_vel────▶│ Port: random    │
└─────────────────┘     (TCP)        └─────────────────┘
       │                                     ▲
       │ Publish                            │
       │                                    │ Publish
       ▼                                    │ (competing)
┌─────────────────────────────────────┐    │
│      /cmd_vel Topic                 │◀───┘
│   (Multiple Publishers Allowed!)    │
└──────────────────┬──────────────────┘
                   │
                   │ Subscribe
                   ▼
            ┌─────────────────┐
            │ Gazebo Husky    │
            │ Controller      │
            └─────────────────┘
```

## Component Interaction Sequence

### Complete Attack Scenario

```
User  │  AVSVA  │  Thread  │   ROS   │  Gazebo  │  Bag
      │         │          │         │          │
  1   │  Start  │          │         │          │
  ────┼────────▶│          │         │          │
      │         │          │         │          │
  2   │         │  Launch  │         │          │
      │         ├─────────▶│         │          │
      │         │          │         │          │
  3   │         │          │ roscore │          │
      │         │          ├────────▶│          │
      │         │          │         │          │
  4   │         │          │ Gazebo  │          │
      │         │          ├─────────┼─────────▶│
      │         │          │         │          │
  5   │ Record  │          │         │          │
  ────┼────────▶│          │         │          │
      │         │          │         │          │
  6   │         │          │         │          │ Start
      │         ├──────────┼─────────┼──────────┼────▶
      │         │          │         │          │
  7   │ Attack! │          │         │          │
  ────┼────────▶│          │         │          │
      │         │          │         │          │
  8   │         │  Start   │         │          │
      │         ├─────────▶│         │          │
      │         │          │         │          │
  9   │         │          │ Init    │          │
      │         │          │ Node    │          │
      │         │          │         │          │
 10   │         │          │ Pub     │          │
      │         │          │ /cmd_vel│          │
      │         │          ├─────────┼─────────▶│
      │         │          │         │          │
 11   │         │          │         │ Spin!    │
      │         │          │         │◀─────────│
      │         │          │         │          │
 12   │         │          │         │          │ Record
      │         │          │         │          │◀─────
      │         │          │         │          │
 13   │  Stop   │          │         │          │
  ────┼────────▶│          │         │          │
      │         │          │         │          │
 14   │         │  Stop    │         │          │
      │         ├─────────▶│         │          │
      │         │          │         │          │
 15   │         │          │Shutdown │          │
      │         │          │         │          │
 16   │         │          │         │ Normal   │
      │         │          │         │◀─────────│
```

## Security Research Workflow

```
┌─────────────────────────────────────────────────────┐
│              Research Workflow                      │
└─────────────────────────────────────────────────────┘

1. Literature Review
   └─▶ Read ROS-Defender paper
       Identify vulnerabilities

2. Threat Modeling
   └─▶ Catalog attack vectors
       Assess severity
       Plan demonstrations

3. Implementation (AVSVA)
   └─▶ Build GUI
       Code attacks
       Test thoroughly

4. Experimentation
   └─▶ Run baseline (no attack)
       Record bag file
       Execute attack
       Record attack bag
       Observe behavior

5. Data Collection
   └─▶ Analyze bag files
       Measure impact
       Document findings
       Screenshot/video

6. Analysis
   └─▶ Compare baseline vs attack
       Quantify impact
       Identify indicators
       Propose mitigations

7. Reporting
   └─▶ Write paper
       Create presentation
       Demo at capstone
       Share with Army Research Lab
```

## Key Insights

### Why These Attacks Work

1. **No Authentication**: ROS has no built-in authentication
2. **No Encryption**: All communication is plaintext
3. **Open API**: XMLRPC endpoints are unprotected
4. **Multiple Publishers**: Topics allow competing publishers
5. **No Access Control**: Parameter server is world-writable

### How ROS-Defender Would Help

```
Normal ROS              ROS with ROS-Defender
──────────              ─────────────────────

┌───────────┐           ┌───────────┐
│  Attack   │           │  Attack   │
└─────┬─────┘           └─────┬─────┘
      │                       │
      │ Malicious             │ Malicious
      │ Messages              │ Messages
      ▼                       ▼
┌───────────┐           ┌───────────┐
│  /topic   │           │   ROSDN   │  ← SDN Firewall
└─────┬─────┘           │  Filter   │
      │                 └─────┬─────┘
      │ Allowed             │
      ▼                       │ BLOCKED! ✗
┌───────────┐                 │
│   Robot   │           ┌─────▼─────┐
│ (pwned)   │           │ ROSWatch  │  ← Anomaly Detection
└───────────┘           │  Alert!   │
                        └───────────┘
```

---

This architecture documentation provides a complete technical view of how AVSVA works from multiple perspectives: system design, data flow, state management, and research methodology.
