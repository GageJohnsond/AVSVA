# File Manifest
## Autonomous Vehicle Simulation and Vulnerability Analyzer

### Complete File List

#### Core Application Files

| File | Size | Description |
|------|------|-------------|
| `av_simulator.py` | 29 KB | Main application with PyQt5 GUI and orchestration logic |
| `bag_player.py` | 2.2 KB | ROS bag file handling and CSV export |
| `data_analyzer.py` | 3.9 KB | Statistical analysis, correlation, and anomaly detection |
| `vulnerability_injector.py` | 4.4 KB | Anomaly injection for security testing |
| `simulator.py` | 6.0 KB | Trajectory simulation and collision detection |
| `visualizer.py` | 5.8 KB | Matplotlib plotting and visualization |
| `report_generator.py` | 7.4 KB | Report compilation and export |
| `config.py` | 3.3 KB | Configuration management |

**Total Core Code**: ~62 KB

#### Documentation Files

| File | Size | Description |
|------|------|-------------|
| `README.md` | 5.7 KB | Quick start guide and overview |
| `USER_GUIDE.md` | 12 KB | Comprehensive user documentation |
| `PROJECT_SUMMARY.md` | 9.9 KB | Implementation summary and status |
| `QUICKSTART.md` | 2.6 KB | 5-minute getting started guide |

**Total Documentation**: ~30 KB

#### Configuration Files

| File | Size | Description |
|------|------|-------------|
| `requirements.txt` | 89 bytes | Python package dependencies |
| `run_simulator.sh` | 916 bytes | Bash launcher script |
| `verify_install.py` | ~2 KB | Installation verification script |

#### Project Structure

```
av-simulator/
│
├── av_simulator.py          # Main application entry point
│
├── Core Modules/
│   ├── bag_player.py        # ROS bag handling
│   ├── data_analyzer.py     # Data processing
│   ├── vulnerability_injector.py  # Security testing
│   ├── simulator.py         # Trajectory simulation
│   ├── visualizer.py        # Plotting
│   ├── report_generator.py  # Report creation
│   └── config.py            # Configuration
│
├── Documentation/
│   ├── README.md            # Quick start
│   ├── USER_GUIDE.md        # Detailed guide
│   ├── PROJECT_SUMMARY.md   # Implementation summary
│   ├── QUICKSTART.md        # Fast setup
│   └── FILE_MANIFEST.md     # This file
│
├── Configuration/
│   ├── requirements.txt     # Dependencies
│   ├── run_simulator.sh     # Launcher
│   └── verify_install.py    # Verification
│
└── Runtime Directories/
    ├── csv_output/          # Exported CSV files (created)
    └── reports/             # Generated reports (created)
```

### Module Dependencies

```
av_simulator.py
├── PyQt5 (GUI framework)
├── matplotlib (embedded plots)
├── bag_player
├── data_analyzer
├── vulnerability_injector
├── simulator
├── visualizer
├── report_generator
└── config

bag_player.py
└── os, subprocess

data_analyzer.py
├── pandas
├── numpy
└── scipy

vulnerability_injector.py
├── pandas
├── numpy
└── os

simulator.py
├── numpy
└── pandas

visualizer.py
├── matplotlib
└── seaborn

report_generator.py
├── pandas
└── datetime

config.py
├── json
└── os
```

### External Dependencies

From `requirements.txt`:

```
PyQt5>=5.15.0          # GUI framework
matplotlib>=3.5.0      # Plotting library
seaborn>=0.11.0        # Statistical visualizations
pandas>=1.3.0          # Data manipulation
numpy>=1.21.0          # Numerical computing
scipy>=1.7.0           # Scientific computing
```

### Class Structure

```
MainWindow (QMainWindow)
├── RosThread (QThread)
├── BagPlayer
├── DataAnalyzer
├── VulnerabilityInjector
├── Simulator
│   └── HuskyKinematicModel
├── Visualizer
├── ReportGenerator
└── Config
```

### Data Flow

```
User Input
    ↓
MainWindow (GUI)
    ↓
┌───────────────────────────────────────┐
│                                       │
↓                                       ↓
BagPlayer                    DataAnalyzer
    ↓                              ↓
CSV Files                   Correlations/Anomalies
    ↓                              ↓
VulnerabilityInjector        Visualizer
    ↓                              ↓
Modified Data                   Plots
    ↓                              ↓
Simulator              ReportGenerator
    ↓                              ↓
Trajectory Analysis            Reports
    ↓                              ↓
└──────────────→ User Output ←─────────┘
```

### File Locations (Runtime)

#### Input Files
- User-selected .bag files (any location)
- User-selected CSV files (any location)

#### Output Files
- Exported CSVs: `/home/claude/csv_output/`
- Generated reports: `/home/claude/reports/`
- Modified bags: Same directory as source

#### Configuration Files
- Default config: Built-in to `config.py`
- Custom config: User-specified JSON file

#### Log Files
- Console output only (no file logging by default)
- Can be redirected: `python av_simulator.py > app.log 2>&1`

### Installation Footprint

**Disk Space Required**:
- Application files: ~100 KB
- Python packages: ~500 MB
- Runtime data: Varies (depends on .bag files)

**Recommended**: 1 GB free space minimum

### Permissions Required

- Read: User's home directory
- Write: `/home/claude/csv_output/`, `/home/claude/reports/`
- Execute: Python interpreter

### Platform Compatibility

**Tested On**:
- Linux (Ubuntu 20.04+)
- Architecture: x86_64

**Should Work On**:
- macOS 10.14+
- Windows 10+ (with minor path adjustments)

### Version Information

- Application Version: 1.0
- Python: 3.8+
- PyQt5: 5.15+
- Date: November 2025

### Checksums (for verification)

To verify file integrity:

```bash
sha256sum *.py *.txt *.sh
```

### Backup Recommendations

**Essential Files** (version control):
- All `.py` files
- `requirements.txt`
- Documentation `.md` files

**Optional** (regeneratable):
- CSV exports
- Generated reports
- Configuration cache

### Update Procedure

1. Backup current files
2. Replace Python modules
3. Update dependencies: `pip install -r requirements.txt --upgrade`
4. Run verification: `python verify_install.py`
5. Test with sample data

### Uninstallation

To remove the application:

```bash
# Remove application files
rm -rf /path/to/av-simulator/

# Remove Python packages (optional)
pip uninstall PyQt5 matplotlib seaborn pandas numpy scipy

# Remove output directories (optional)
rm -rf /home/claude/csv_output/
rm -rf /home/claude/reports/
```

### Support Files

Additional files that can be created:

- `config.json` - Custom configuration
- `test_data/` - Sample .bag files
- `examples/` - Usage examples
- `tests/` - Unit tests

### Build Information

- Build Date: November 10, 2025
- Build Environment: Linux/Ubuntu
- Python Version: 3.12.7
- Build Type: Development

### Credits

**Development Team** (Raider Security):
- Reid Layne - Functional requirements
- Tyler Bowen - UI design
- Gage Johnson - Data workflows
- Nick Sanchez - Security testing

**Institution**: Texas Tech University  
**Course**: CS 3365 - Software Engineering  
**Project**: Capstone Project 14

---

**This manifest documents all deliverable files for the Autonomous Vehicle Simulation and Vulnerability Analyzer project.**
