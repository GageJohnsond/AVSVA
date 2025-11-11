# Autonomous Vehicle Simulation and Vulnerability Analyzer
## Master Index and Quick Reference

---

## 📁 Complete File Listing

### 🚀 Quick Start (Start Here!)

1. **[QUICKSTART.md](computer:///mnt/user-data/outputs/QUICKSTART.md)** - Get running in 5 minutes
2. **[README.md](computer:///mnt/user-data/outputs/README.md)** - Installation and overview
3. **[verify_install.py](computer:///mnt/user-data/outputs/verify_install.py)** - Test your installation

### 📖 Documentation

| Document | Purpose | When to Read |
|----------|---------|--------------|
| **[USER_GUIDE.md](computer:///mnt/user-data/outputs/USER_GUIDE.md)** | Complete usage instructions | When learning the app |
| **[PROJECT_SUMMARY.md](computer:///mnt/user-data/outputs/PROJECT_SUMMARY.md)** | Implementation details | For technical overview |
| **[FILE_MANIFEST.md](computer:///mnt/user-data/outputs/FILE_MANIFEST.md)** | All files explained | For file organization |
| **README.md** | Setup and features | Before installation |
| **QUICKSTART.md** | Fast 5-minute guide | For quick demo |

### 💻 Application Code

| File | Lines | Purpose |
|------|-------|---------|
| **[av_simulator.py](computer:///mnt/user-data/outputs/av_simulator.py)** | ~800 | Main application and GUI |
| **[bag_player.py](computer:///mnt/user-data/outputs/bag_player.py)** | ~80 | ROS bag file handling |
| **[data_analyzer.py](computer:///mnt/user-data/outputs/data_analyzer.py)** | ~130 | Data analysis engine |
| **[vulnerability_injector.py](computer:///mnt/user-data/outputs/vulnerability_injector.py)** | ~140 | Security testing |
| **[simulator.py](computer:///mnt/user-data/outputs/simulator.py)** | ~200 | Trajectory simulation |
| **[visualizer.py](computer:///mnt/user-data/outputs/visualizer.py)** | ~180 | Plotting and charts |
| **[report_generator.py](computer:///mnt/user-data/outputs/report_generator.py)** | ~200 | Report creation |
| **[config.py](computer:///mnt/user-data/outputs/config.py)** | ~100 | Configuration |

### ⚙️ Configuration

| File | Purpose |
|------|---------|
| **[requirements.txt](computer:///mnt/user-data/outputs/requirements.txt)** | Python dependencies |
| **[run_simulator.sh](computer:///mnt/user-data/outputs/run_simulator.sh)** | Launch script |
| **verify_install.py** | Installation checker |

---

## 🎯 Usage Pathways

### For First-Time Users
```
1. QUICKSTART.md → 2. Install dependencies → 3. Run app → 4. Load sample data
```

### For Developers
```
1. PROJECT_SUMMARY.md → 2. FILE_MANIFEST.md → 3. Read source code → 4. Customize
```

### For End Users
```
1. README.md → 2. USER_GUIDE.md → 3. Use application → 4. Generate reports
```

---

## 🔧 Common Tasks

### Installation
```bash
# Quick install
pip install -r requirements.txt
python verify_install.py

# Or use launcher
./run_simulator.sh
```

### Running the App
```bash
python av_simulator.py
```

### Basic Workflow
```
Load .bag File → Analyze Data → Generate Report
```

### Advanced Workflow  
```
Load .bag → Inject Anomaly → Compare Results → Generate Report
```

---

## 📊 Key Features

### ✅ Implemented
- [x] ROS bag file simulation
- [x] CSV data analysis
- [x] Correlation computation
- [x] Anomaly detection
- [x] Vulnerability injection
- [x] Trajectory simulation
- [x] Interactive visualizations
- [x] Report generation
- [x] Professional GUI

### 🔄 Future Enhancements
- [ ] Real ROS bag integration
- [ ] Live Gazebo visualization
- [ ] Machine learning anomalies
- [ ] Cloud processing
- [ ] Real-time monitoring

---

## 📚 Technical Specifications

### Architecture
- **Language**: Python 3.8+
- **GUI**: PyQt5
- **Analysis**: pandas, numpy, scipy
- **Visualization**: matplotlib, seaborn

### Design Pattern
- Modular object-oriented
- Model-View-Controller inspired
- Separation of concerns
- Extensible architecture

### Requirements Met
- ✅ All SRS functional requirements (F1-F5)
- ✅ All use cases (UC#1-4)
- ✅ All UI specifications
- ✅ All design components

---

## 🎓 Academic Context

**Project**: CS 3365 Software Engineering Capstone  
**Team**: Raider Security (Project 14)  
**Institution**: Texas Tech University  
**Date**: November 2025

**Team Members**:
- Reid Layne - reilayne@ttu.edu
- Tyler Bowen - tylbowen@ttu.edu
- Gage Johnson - gage.d.johnson@ttu.edu
- Nick Sanchez - nicolas232.sanchez@ttu.edu

**Website**: https://gingerbreadgames.github.io/CapstoneWebPage/

---

## 🆘 Getting Help

### Documentation Priority
1. **Quick answer**: QUICKSTART.md
2. **How-to**: USER_GUIDE.md
3. **Technical**: PROJECT_SUMMARY.md
4. **Setup issues**: README.md
5. **File questions**: FILE_MANIFEST.md

### Troubleshooting
```bash
# Installation issues
python verify_install.py

# Package issues
pip install -r requirements.txt --upgrade

# Import errors
python -c "import PyQt5; print('OK')"
```

### Common Issues

| Problem | Solution |
|---------|----------|
| App won't start | `pip install PyQt5 --upgrade` |
| No analysis output | Check CSV files exported |
| Blank visualizations | `pip install matplotlib --upgrade` |
| Memory issues | Process smaller files first |

---

## 📦 Deliverables Summary

### Code Files: 8
- Main application + 7 modules

### Documentation: 5  
- User guide, README, summaries, quickstart, manifest

### Configuration: 3
- Requirements, launcher, verification

### Total Files: 16
All files are in `/mnt/user-data/outputs/`

---

## 🚀 Quick Commands

```bash
# Install
pip install -r requirements.txt

# Verify
python verify_install.py

# Run
python av_simulator.py

# Or use launcher
./run_simulator.sh

# Check version
python av_simulator.py --version  # (if implemented)
```

---

## 📋 Checklist for Success

### Installation
- [ ] Python 3.8+ installed
- [ ] Dependencies installed
- [ ] Verification passed

### First Run
- [ ] Application launches
- [ ] GUI displays correctly
- [ ] Can navigate tabs

### Basic Usage
- [ ] Can load a file
- [ ] Analysis produces results
- [ ] Visualizations appear
- [ ] Report generates

### Advanced Usage
- [ ] Simulation playback works
- [ ] Anomaly injection succeeds
- [ ] Before/after comparison shows
- [ ] All features functional

---

## 🎯 Next Steps

1. **Read** QUICKSTART.md (5 min)
2. **Install** dependencies (2 min)
3. **Verify** with verify_install.py (1 min)
4. **Run** application (30 sec)
5. **Explore** with sample data (10 min)
6. **Reference** USER_GUIDE.md as needed

---

## 📞 Support

For questions, issues, or contributions:
- Check documentation first
- Contact team via project website
- Review source code comments

---

## ⚖️ License

Academic project - Texas Tech University  
Developed for educational purposes

---

**This index provides a comprehensive overview and quick navigation to all project resources.**

**Version**: 1.0  
**Last Updated**: November 10, 2025  
**Status**: Complete and Ready for Use ✅
