# Quick Start Guide
## Autonomous Vehicle Simulation and Vulnerability Analyzer

### 5-Minute Setup

#### Step 1: Install Dependencies (1 minute)

```bash
pip install -r requirements.txt
```

Or use the automatic installer:

```bash
./run_simulator.sh
```

#### Step 2: Launch Application (30 seconds)

```bash
python av_simulator.py
```

#### Step 3: Load Data (1 minute)

1. Click **"📁 Load .bag File"**
2. Select your ROS bag file
3. Wait for CSV export to complete
4. Status updates appear in console

#### Step 4: Run Analysis (2 minutes)

1. Click **"📊 Analyze Data"**
2. View correlation heatmap
3. Check anomaly table
4. Export results if needed

#### Step 5: Generate Report (30 seconds)

1. Click **"📄 Generate Report"**
2. Choose save location
3. Select format (TXT/PDF)
4. Review generated report

---

### Key Features at a Glance

| Feature | Button | Tab | Time |
|---------|--------|-----|------|
| Load Data | 📁 Load .bag File | Dashboard | 1 min |
| Simulate | ▶ Start Simulation | Simulation Playback | Varies |
| Analyze | 📊 Analyze Data | Data Analysis | 2 min |
| Test Vulnerabilities | ⚠ Inject Anomaly | Vulnerability Testing | 3 min |
| Create Report | 📄 Generate Report | Any | 30 sec |

---

### Common Workflows

#### Workflow 1: Basic Analysis

```
Load .bag File → Analyze Data → Generate Report
```

**Time**: ~5 minutes

#### Workflow 2: Simulation Review

```
Load .bag File → Start Simulation → Review Playback
```

**Time**: Depends on bag duration

#### Workflow 3: Vulnerability Testing

```
Load .bag File → Inject Anomaly → Compare Results → Generate Report
```

**Time**: ~10 minutes

---

### Keyboard Shortcuts

- `Ctrl+O` - Open file
- `Ctrl+S` - Save report  
- `Space` - Play/Pause
- `Ctrl+R` - Generate report
- `Ctrl+Q` - Quit

---

### Troubleshooting

**Problem**: Application won't start
```bash
pip install PyQt5 --upgrade
```

**Problem**: No data in analysis
- Check CSV files exported correctly
- Verify data directory exists

**Problem**: Visualization blank
```bash
pip install matplotlib --upgrade
```

---

### Need More Help?

- 📖 See `USER_GUIDE.md` for detailed instructions
- 📋 See `README.md` for technical details
- 📧 Contact team via project website

---

### First-Time User Checklist

- [ ] Python 3.8+ installed
- [ ] Dependencies installed
- [ ] Application launches successfully
- [ ] Can load a test file
- [ ] Analysis produces results
- [ ] Report generation works

---

**You're ready to go!** 🚀

Start by loading a .bag file and exploring the interface.
