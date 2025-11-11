# Autonomous Vehicle Simulation and Vulnerability Analyzer - User Guide

## Table of Contents
1. [Introduction](#introduction)
2. [Getting Started](#getting-started)
3. [Main Dashboard](#main-dashboard)
4. [Simulation Playback](#simulation-playback)
5. [Data Analysis](#data-analysis)
6. [Vulnerability Testing](#vulnerability-testing)
7. [Report Generation](#report-generation)
8. [Advanced Features](#advanced-features)
9. [Troubleshooting](#troubleshooting)

## Introduction

The Autonomous Vehicle Simulation and Vulnerability Analyzer (AV Simulator) is a comprehensive desktop application designed for security researchers, vehicle engineers, and educators to:

- Simulate autonomous vehicle behavior from recorded sensor data
- Analyze patterns and correlations in vehicle telemetry
- Test system vulnerabilities through controlled anomaly injection
- Generate detailed reports for documentation and compliance

This guide will walk you through all features of the application.

## Getting Started

### Installation

1. Ensure Python 3.8+ is installed on your system
2. Navigate to the application directory
3. Run the installation script:

```bash
./run_simulator.sh
```

Or manually:

```bash
pip install -r requirements.txt
python av_simulator.py
```

### First Launch

When you first launch the application, you'll see the main dashboard with:
- Quick action buttons on the left sidebar
- File browser in the center
- System log console at the bottom
- Status bar showing system metrics

## Main Dashboard

### Quick Actions Sidebar

The sidebar provides one-click access to core functions:

**📁 Load .bag File** (Blue)
- Click to open file browser
- Select a ROS bag file (.bag extension)
- System automatically exports topics to CSV
- Status updates appear in log console

**▶ Start Simulation** (Blue)
- Becomes active after loading a bag file
- Initiates playback of recorded data
- Switches to Simulation Playback tab

**📊 Analyze Data** (Green)
- Performs statistical analysis on CSV data
- Computes correlations between variables
- Detects anomalies using z-score method
- Displays results in Data Analysis tab

**⚠ Inject Anomaly** (Red)
- Opens Vulnerability Testing interface
- Allows controlled modification of data
- Used for security testing

**📄 Generate Report** (Yellow)
- Compiles all analysis results
- Creates comprehensive documentation
- Exports in multiple formats

### File Browser

The central file browser accepts:
- Individual .bag files
- .tar archives containing multiple bags
- Drag-and-drop functionality
- Click to browse traditional file selection

Once a file is loaded, metadata appears including:
- Filename
- File size
- Topics discovered
- Duration

### System Log Console

The console displays real-time messages:
- Green text on dark background
- Timestamped entries
- System initialization messages
- File operations
- Analysis progress
- Error notifications

### Status Bar

Bottom status bar shows:
- System status (Ready/Busy)
- ROS connection status
- Memory usage (current/total)
- CPU utilization percentage

## Simulation Playback

### Controls

**Playback Buttons:**
- ▶ Play: Start or resume playback
- ⏸ Pause: Temporarily halt playback
- ⏹ Stop: End playback and reset

**Timeline Scrubber:**
- Visual progress bar
- Click to jump to specific time
- Shows playback position

**Speed Control:**
- Dropdown menu
- Options: 0.5x, 1x, 2x, 5x
- Allows accelerated or slow-motion review

### Viewport

The main viewport displays either:

**3D Gazebo View** (if available):
- Real-time 3D visualization
- Vehicle model rendering
- Environment display

**Topic Logs** (fallback):
- Text-based data streams
- Real-time topic values
- Formatted for readability

### Overlay Information

Position overlay shows:
- Vehicle X, Y, Z coordinates
- Linear velocity (m/s)
- Heading angle (degrees)
- Status indicators

### Topic Logs Panel

Displays real-time data from key topics:
- `/cmd_vel` - Velocity commands (linear.x, angular.z)
- `/imu/data` - IMU readings (acceleration, orientation)
- `/navsat/fix` - GPS coordinates (lat, lon)
- `/odometry` - Position tracking

## Data Analysis

### Filter Controls

**Dataset Selection:**
- Dropdown menu of available CSV files
- Automatically populated from export
- Select active dataset for analysis

**Time Range:**
- Start and End fields
- Filter data by timestamp
- Useful for focusing on specific periods

**Column Selection:**
- Choose "All" or specific columns
- Limits analysis scope
- Improves performance for large files

**Apply Filters Button:**
- Executes filter configuration
- Updates all visualizations
- Refreshes analysis results

### Analysis Tabs

#### Correlations Tab

Displays correlation heatmap:
- Color-coded matrix
- Red: Positive correlation
- Blue: Negative correlation
- Green: Neutral/weak correlation
- Numerical values shown in cells

**Interpretation:**
- Values range from -1 to 1
- |r| > 0.7: Strong correlation
- |r| < 0.3: Weak correlation
- Helps identify sensor relationships

#### Anomalies Tab

Shows detected outliers in table format:

| Column | Description |
|--------|-------------|
| Timestamp | When anomaly occurred |
| Variable | Affected sensor/parameter |
| Expected | Normal value range |
| Actual | Observed value |
| Z-Score | Statistical deviation |

**Color Coding:**
- Red text: Significant anomalies (|z| > 3)
- Values sorted by severity

**Export Button:**
- Save results to CSV
- Choose export location
- Includes all anomaly data

#### Statistics Tab

Provides summary statistics:
- Mean, median, mode
- Standard deviation
- Min/max values
- Count of samples
- Distribution information

### Raw Data Tab

Browse unprocessed CSV data:
- Paginated view
- Search functionality
- Column sorting
- Export capability

## Vulnerability Testing

### Injection Parameters

**Select Topic:**
- `/cmd_vel`: Velocity commands
- `/imu/data`: Inertial measurements
- `/navsat/fix`: GPS data
- `/odometry`: Position data

**Modification Type:**
- Set to Zero: Nullify values
- Add Noise: Gaussian noise injection
- Spoof GPS: False location data
- Delay: Introduce latency
- Drop Packets: Simulate packet loss

**Value/Parameter Field:**
- Specify injection details
- Example: "Set linear.x to 0.0"
- Supports mathematical expressions
- Context-sensitive help

**Injection Types (Checkboxes):**
- ☑ Spoofing: False data injection
- ☐ Delays: Message latency
- ☐ Corruption: Data integrity attacks
- ☐ Replay: Repeat old messages

### Execute Injection

Click "⚠ Execute Injection" button to:
1. Create modified bag file
2. Re-run simulation
3. Monitor for failures
4. Compare results

### Before/After Comparison

Split-screen view shows:

**Before Injection:**
- Original data values
- Normal system state
- Baseline behavior

**After Injection:**
- Modified data values
- Anomalous values in red
- System response

### Failure State Alert

Red alert box appears when failures detected:
- Navigation deviations
- Control losses
- Sensor conflicts
- Collision risks

Includes:
- Failure description
- Severity metrics
- Attack vector identification
- Mitigation suggestions

## Report Generation

### Comprehensive Reports

Generated reports include:

1. **Executive Summary**
   - High-level overview
   - Key findings
   - Risk assessment

2. **Correlation Analysis**
   - Matrix visualization
   - Strong correlations highlighted
   - Interpretation notes

3. **Anomaly Detection**
   - List of detected anomalies
   - Statistical details
   - Temporal patterns

4. **Vulnerability Testing**
   - Injection test results
   - Failure states
   - Security implications

5. **Recommendations**
   - Mitigation strategies
   - Best practices
   - Future testing suggestions

### Export Options

**Format Selection:**
- Text (.txt): Plain text format
- PDF (.pdf): Professional documents
- CSV (.csv): Data tables

**Export Process:**
1. Click "Generate Report"
2. Choose format and location
3. System compiles results
4. File saved to selected path
5. Confirmation message displayed

## Advanced Features

### Custom Configuration

Edit `config.py` or create JSON config:

```json
{
    "anomaly_detection_threshold": 3.0,
    "max_playback_speed": 5.0,
    "collision_threshold": 0.5,
    "csv_export_dir": "/custom/path",
    "report_export_dir": "/reports/path"
}
```

### Batch Processing

Process multiple bag files:
1. Load files sequentially
2. Automated analysis pipeline
3. Batch report generation

### API Integration

The application modules can be imported:

```python
from data_analyzer import DataAnalyzer
from simulator import Simulator

analyzer = DataAnalyzer()
results = analyzer.compute_correlations(dfs)
```

### Keyboard Shortcuts

- `Ctrl+O`: Open file
- `Ctrl+S`: Save report
- `Space`: Play/Pause simulation
- `Ctrl+R`: Generate report
- `Ctrl+Q`: Quit application

## Troubleshooting

### Application Won't Start

**Problem**: Error on launch
**Solution**: 
```bash
pip install -r requirements.txt --upgrade
python3 --version  # Check Python version
```

### Bag File Won't Load

**Problem**: Invalid file format
**Solution**:
- Verify file is ROS 1 format
- Check file isn't corrupted
- Ensure sufficient disk space

### Analysis Produces No Results

**Problem**: Empty analysis output
**Solution**:
- Verify CSV files exported correctly
- Check data isn't all NaN values
- Ensure proper timestamp column

### Visualization Not Displaying

**Problem**: Blank plot area
**Solution**:
- Update matplotlib: `pip install matplotlib --upgrade`
- Check data loaded successfully
- Verify no filter excludes all data

### Memory Issues

**Problem**: Application crashes with large files
**Solution**:
- Close other applications
- Process files in chunks
- Use data sampling feature
- Increase system swap space

### ROS Not Detected

**Problem**: ROS features unavailable
**Solution**:
- Application works without ROS for CSV analysis
- Install ROS Noetic for full features
- Source ROS setup: `source /opt/ros/noetic/setup.bash`

## Tips and Best Practices

### Workflow Recommendations

1. **Start Small**: Test with smaller bag files first
2. **Incremental Analysis**: Run analyses step-by-step
3. **Save Often**: Export intermediate results
4. **Document Findings**: Use report generation frequently
5. **Version Control**: Keep track of injection parameters

### Performance Optimization

- Use filters to limit data scope
- Close unused tabs
- Export results before new analysis
- Restart application periodically for long sessions

### Security Considerations

- Work on copies of original data
- Document all injection tests
- Review failure states carefully
- Follow responsible disclosure for real vulnerabilities

### Academic Use

- Cite data sources properly
- Document methodology in reports
- Include parameter settings
- Maintain research ethics

## Support

For additional help:
- Check the README.md for technical details
- Review module documentation
- Contact project team via website
- Refer to interview summaries for domain knowledge

## Appendix

### Supported ROS Topics

| Topic | Message Type | Description |
|-------|-------------|-------------|
| /cmd_vel | geometry_msgs/Twist | Velocity commands |
| /imu/data | sensor_msgs/Imu | IMU measurements |
| /navsat/fix | sensor_msgs/NavSatFix | GPS data |
| /odometry | nav_msgs/Odometry | Position/orientation |

### Statistical Methods

**Correlation**: Pearson correlation coefficient
**Anomaly Detection**: Z-score method (threshold: 3.0)
**Simulation**: Unicycle kinematic model

### File Locations

Default directories:
- CSV Export: `/home/claude/csv_output`
- Reports: `/home/claude/reports`
- Logs: Application working directory

---

**Version**: 1.0
**Last Updated**: November 2025
**Project**: CS 3365 Capstone - Raider Security Team
