# Autonomous Vehicle Simulation and Vulnerability Analyzer

A desktop application for simulating autonomous vehicle behavior using ROS bag files, analyzing sensor data, and testing vulnerabilities through anomaly injection.

## Features

- **Simulation Playback**: Load and play back ROS .bag files to simulate vehicle behavior
- **Data Characterization**: Analyze CSV sensor data for correlations and anomalies
- **Vulnerability Testing**: Inject controlled anomalies to test system resilience
- **Visualization**: Interactive plots, heatmaps, and trajectory displays
- **Reporting**: Generate comprehensive analysis reports

## Installation

### Prerequisites

- Python 3.8 or later
- PyQt5
- ROS 1 (Noetic) - optional, for real bag file processing

### Setup

1. Install Python dependencies:

```bash
pip install -r requirements.txt
```

2. For full ROS integration (optional):

```bash
# On Ubuntu 20.04/22.04
sudo apt-get install ros-noetic-desktop-full
source /opt/ros/noetic/setup.bash
```

## Usage

### Running the Application

```bash
python av_simulator.py
```

### Basic Workflow

1. **Load Data**:
   - Click "Load .bag File" to select a ROS bag file
   - The system will automatically export topics to CSV

2. **Run Simulation**:
   - Click "Start Simulation" to begin playback
   - Use the Simulation Playback tab to control playback speed
   - Monitor vehicle position and sensor data in real-time

3. **Analyze Data**:
   - Click "Analyze Data" to compute correlations and detect anomalies
   - View correlation heatmaps and anomaly tables
   - Export results for further analysis

4. **Test Vulnerabilities**:
   - Click "Inject Anomaly" to access vulnerability testing
   - Select topic, modification type, and parameters
   - Execute injection and observe system behavior
   - Compare before/after states

5. **Generate Reports**:
   - Click "Generate Report" to compile findings
   - Export reports in text or PDF format

## Application Structure

```
av_simulator.py          - Main application and GUI
bag_player.py           - ROS bag file handling
data_analyzer.py        - Data processing and analysis
vulnerability_injector.py - Anomaly injection
simulator.py            - Trajectory simulation
visualizer.py           - Plotting and visualization
report_generator.py     - Report compilation
config.py              - Configuration management
```

## Modules

### BagPlayer
Handles ROS bag file operations including export to CSV and playback.

### DataAnalyzer
Processes CSV data to compute correlations and detect anomalies using statistical methods.

### VulnerabilityInjector
Injects controlled anomalies into data for vulnerability testing:
- Zero velocity commands
- Add Gaussian noise
- GPS spoofing
- Packet delays/drops
- Replay attacks

### Simulator
Performs offline trajectory simulation using Husky kinematic model and detects collisions.

### Visualizer
Generates plots including:
- Trajectory visualizations
- Correlation heatmaps
- Time series plots
- Anomaly highlights

### ReportGenerator
Compiles comprehensive reports with analysis results and recommendations.

## Configuration

Edit `config.py` or create a JSON config file to customize:
- Anomaly detection thresholds
- Supported anomaly types
- Export directories
- Playback parameters

Example config.json:
```json
{
    "anomaly_detection_threshold": 3.0,
    "collision_threshold": 0.5,
    "csv_export_dir": "/home/user/csv_output",
    "report_export_dir": "/home/user/reports"
}
```

## Use Cases

### UC#1: Simulate Vehicle via Bag Playback
Load a .bag file and play back recorded sensor data to observe vehicle behavior.

### UC#2: Characterize Data
Analyze CSV files to identify correlations between sensor readings and detect anomalies.

### UC#3: Inject Anomalies and Test Failures
Modify simulation data to inject faults and observe system response.

### UC#4: Generate Report and Visualizations
Compile analysis results into comprehensive reports with visualizations.

## Supported File Formats

- **Input**: 
  - ROS .bag files (ROS 1 format)
  - CSV files exported from bag files
  - .tar archives containing multiple bags

- **Output**:
  - CSV (analysis results)
  - Text reports
  - PDF reports (when available)

## Data Processing

The application processes several key ROS topics:
- `/cmd_vel` - Velocity commands
- `/imu/data` - Inertial measurement unit data
- `/navsat/fix` - GPS coordinates
- `/odometry` - Position and orientation

## Performance

- Handles bag files up to several GB
- CSV analysis for datasets with 10,000+ rows
- Real-time playback visualization
- Background processing to maintain GUI responsiveness

## Safety & Security

- Creates copies of original files before modification
- Sandboxed anomaly injection (offline only)
- No direct hardware integration
- Audit logging of all operations

## Troubleshooting

### Common Issues

1. **PyQt5 Import Error**:
   ```bash
   pip install PyQt5 --upgrade
   ```

2. **ROS Not Found**:
   - Application works without ROS for CSV analysis
   - Full bag playback requires ROS 1 (Noetic)

3. **Memory Issues with Large Files**:
   - Increase system swap space
   - Use data sampling for initial analysis

## Project Information

- **Course**: CS 3365 - Software Engineering
- **Team**: Raider Security (Project 14)
- **Members**: Reid Layne, Tyler Bowen, Gage Johnson, Nick Sanchez
- **Institution**: Texas Tech University

## License

This is an academic project developed for educational purposes.

## Acknowledgments

- Dr. Robert F. Erbacher (Army Research Lab) for data provision
- Dr. Tommy Dang for project facilitation
- Interview participants for domain expertise

## Contact

For questions or issues, contact the development team through the project website:
https://gingerbreadgames.github.io/CapstoneWebPage/
